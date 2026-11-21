// Compile the production GATT implementation unchanged against a timed IPC shim.
#include "ble_gatt.cpp"
#include <cassert>
#include <cstring>
#include <iostream>
#include <vector>
using namespace ams::ble;

enum class DiscoveryMode { AfterSettling, AfterMtu, AfterBond, Never };
static DiscoveryMode mode;
static Result register_result{0}, query_result{0}, mtu_result{0};
static bool bonded, mtu_reply;
static int unregister_count;
static u64 connected_at;
static std::vector<u64> query_times;
static std::vector<u16> read_uuids;
static std::vector<int> read_auths;
static std::vector<int> cccd_auths;
static std::vector<u32> mtu_connections;
static bool unrelated_completions;
static u64 completion_delay;
static int auth_refusals;
static unsigned notification_registrations;
static bool cccd_completed;
static bool cccd_use_desc_uuid;
static bool cccd_complete_via_notification;
static u32 cccd_failure;
static u32 cccd_plain_failure;
static bool fail_pnp_timeout = false;

static std::vector<u8> pnp_id_payload;
static std::vector<u8> device_name_payload;
static bool fail_pnp_read = false;

static void Complete(u32 conn_id, u8 operation, BtdrvGattAttributeUuid uuid, int auth = BtdrvGattAuthReqType_NoMitm) {
    if (events::disconnect_on_wait) events::Schedule(EventRecord{events::Disconnect, {}}, 1);
    if (events::sleep_on_wait) events::Schedule(EventRecord{events::RadioOff, {}}, 1);
    EventRecord event{};
    event.type = BtdrvBleEventType_ClientNotify;
    event.info.client_notify = {events::failure, conn_id, operation, uuid};
    if (operation == BtdrvGattOperationType_CharacteristicRead && uuid.value == GattCharPnpId) {
        if (fail_pnp_timeout) {
            return;
        }
        if (!pnp_id_payload.empty()) {
            if (fail_pnp_read) {
                event.info.client_notify.result = 9;
            } else {
                event.info.client_notify.size = static_cast<u16>(pnp_id_payload.size());
                std::memcpy(event.info.client_notify.data, pnp_id_payload.data(), pnp_id_payload.size());
            }
        }
    } else if (operation == BtdrvGattOperationType_CharacteristicRead && uuid.value == GattCharDeviceName && !device_name_payload.empty()) {
        event.info.client_notify.size = static_cast<u16>(device_name_payload.size());
        std::memcpy(event.info.client_notify.data, device_name_payload.data(), device_name_payload.size());
    }
    const bool cccd = operation == BtdrvGattOperationType_DescriptorWrite;
    if (cccd) {
        event.info.client_notify.result = (auth == BtdrvGattAuthReqType_None && cccd_plain_failure != 0)
            ? cccd_plain_failure : cccd_failure;
        if (cccd_use_desc_uuid) {
            event.info.client_notify.char_uuid = {};
            event.info.client_notify.desc_uuid = {GattDescClientCharConfig};
        } else if (cccd_complete_via_notification && event.info.client_notify.result == 0) {
            event.info.client_notify.op_type = BtdrvGattOperationType_Notification;
            event.info.client_notify.char_uuid = {GattCharReport};
        }
    }
    if (events::completed && (!cccd || cccd_completed)) events::Schedule(event, completion_delay);
    if (unrelated_completions) {
        auto noise = event;
        noise.info.client_notify.result = 0;
        noise.info.client_notify.desc_uuid = {};
        ++noise.info.client_notify.conn_id; events::Schedule(noise, 1);
        noise = event;
        noise.info.client_notify.desc_uuid = {};
        noise.info.client_notify.op_type = (operation == BtdrvGattOperationType_DescriptorWrite) ? 1 : static_cast<u8>(operation + 1);
        events::Schedule(noise, 2);
        noise = event;
        noise.info.client_notify.desc_uuid = {};
        ++noise.info.client_notify.char_uuid.value;
        events::Schedule(noise, 3);
    }
}

static EventRecord Cache(u32 conn_id) {
    EventRecord e{};
    e.type = BtdrvBleEventType_ClientCacheSave;
    auto &c = e.info.client_cache_save;
    c.conn_id = conn_id; c.count = 6;
    c.attr_list[0] = {BtdrvGattAttributeType_Service, 1, 20, 0, true, {{GattServiceHid}}};
    c.attr_list[1] = {BtdrvGattAttributeType_Characteristic, 2, 0, BtdrvGattCharacteristicProperty_Notify, true, {{GattCharReport}}};
    c.attr_list[2] = {BtdrvGattAttributeType_Descriptor, 3, 0, 0, true, {{GattDescClientCharConfig}}};
    c.attr_list[3] = {BtdrvGattAttributeType_Characteristic, 4, 0, BtdrvGattCharacteristicProperty_Write, true, {{GattCharReport}}};
    c.attr_list[4] = {BtdrvGattAttributeType_Characteristic, 5, 0, 0, true, {{GattCharHidInformation}}};
    c.attr_list[5] = {BtdrvGattAttributeType_Characteristic, 6, 0, 0, true, {{GattCharReportMap}}};
    return e;
}

static EventRecord Mtu(u32 conn_id) {
    EventRecord e{};
    e.type = BtdrvBleEventType_ClientConfigureMtu;
    e.info.client_configure_mtu = {0, conn_id, 256};
    return e;
}

static void PublishCache(u32 conn_id) {
    // The first chunk has usable input, but output and authentication metadata
    // arrive later. Readiness must not snapshot only the first chunk.
    auto first = Cache(conn_id), second = first;
    first.info.client_cache_save.count = 3;
    auto &c = second.info.client_cache_save;
    c.count = 3;
    for (int i = 0; i < 3; ++i) c.attr_list[i] = c.attr_list[i + 3];
    events::Schedule(first, 20);
    events::Schedule(second, 80);
}

Result btdrvGetGattAttribute(ams::bluetooth::Address, u32 conn_id) {
    query_times.push_back(events::now_ms);
    // Match the observed successful IPC with no callbacks when called too early.
    if (R_SUCCEEDED(query_result) && events::now_ms >= connected_at + 1500 &&
        (mode == DiscoveryMode::AfterSettling || (mode == DiscoveryMode::AfterBond && bonded))) {
        PublishCache(conn_id);
    }
    return query_result;
}
Result btdrvConfigureAttMtu(u32 conn_id, u16 mtu) {
    assert(mtu == 0x100);
    mtu_connections.push_back(conn_id);
    if (R_SUCCEEDED(mtu_result)) {
        if (mode == DiscoveryMode::AfterMtu) PublishCache(conn_id);
        if (mtu_reply) events::Schedule(Mtu(conn_id), 100);
    }
    return mtu_result;
}
Result btdrvReadGattCharacteristic(u32 conn_id, bool, const BtdrvGattId *, const BtdrvGattId *attr, int auth) {
    assert(auth == BtdrvGattAuthReqType_None || auth == BtdrvGattAuthReqType_NoMitm);
    read_uuids.push_back(attr->uuid.value);
    read_auths.push_back(auth);
    if (auth == BtdrvGattAuthReqType_NoMitm && auth_refusals > 0) { --auth_refusals; return ResultInsufficientAuthentication(); }
    Complete(conn_id, BtdrvGattOperationType_CharacteristicRead, attr->uuid, auth);
    return {0};
}
Result btdrvRegisterGattNotification(u32, bool, const BtdrvGattId *, const BtdrvGattId *) { ++notification_registrations; return register_result; }
Result btdrvUnregisterGattNotification(u32, bool, const BtdrvGattId *, const BtdrvGattId *) { ++unregister_count; return {0}; }
Result btdrvWriteGattDescriptor(u32 conn_id, bool, const BtdrvGattId *, const BtdrvGattId *attr, const BtdrvGattId *, const u8 *data, size_t size, int auth) {
    assert((auth == BtdrvGattAuthReqType_None || auth == BtdrvGattAuthReqType_NoMitm) && size == 2 && data[0] == 1 && data[1] == 0);
    cccd_auths.push_back(auth);
    if (auth == BtdrvGattAuthReqType_NoMitm && auth_refusals > 0) { --auth_refusals; return ResultInsufficientAuthentication(); }
    Complete(conn_id, BtdrvGattOperationType_DescriptorWrite, attr->uuid, auth);
    return {0};
}

static void Reset(DiscoveryMode next = DiscoveryMode::AfterSettling) {
    for (auto &slot : gatt::g_slots) slot = {};
    events::queue.clear();
    events::disconnected = false;
    events::disconnect_on_wait = events::sleep_on_wait = false;
    events::radio = events::completed = true;
    events::failure = 0;
    connected_at = events::now_ms = 0;
    mode = next;
    bonded = false;
    mtu_reply = true;
    register_result = query_result = mtu_result = {0};
    unregister_count = 0;
    notification_registrations = 0;
    unrelated_completions = false;
    completion_delay = 20;
    auth_refusals = 0;
    cccd_completed = true;
    cccd_use_desc_uuid = false;
    cccd_complete_via_notification = false;
    cccd_failure = 0;
    cccd_plain_failure = 0;
    query_times.clear();
    read_uuids.clear();
    read_auths.clear();
    cccd_auths.clear();
    mtu_connections.clear();
    pnp_id_payload.clear();
    device_name_payload.clear();
    fail_pnp_read = false;
    fail_pnp_timeout = false;
}

static void AssertSubscribed(u32 slot, u32 conn_id) {
    gatt::HidService hid{};
    assert(R_SUCCEEDED(gatt::Subscribe(slot, conn_id, &hid)));
    assert(hid.has_output_report && hid.output_report.handle == 4);
    assert(!read_uuids.empty());
    assert(std::find(read_uuids.begin(), read_uuids.end(), GattCharHidInformation) != read_uuids.end());
    assert(std::find(read_uuids.begin(), read_uuids.end(), GattCharReportMap) == read_uuids.end());
}

int main() {
    // Fresh discovery at 800ms allows an earlier query, while a reused cache
    // must retain the two-second fallback. Keep the MTU and final settling.
    Reset(DiscoveryMode::Never);
    auto fresh = Cache(260);
    events::Schedule(fresh, 800);
    const ams::bluetooth::Address fresh_address{{9,8,7,6,5,4}};
    assert(R_SUCCEEDED(gatt::Discover(0, 260, fresh_address)));
    assert(query_times.front() == 800);
    assert(events::now_ms == 1400);
    AssertSubscribed(0, 260);

    const ams::bluetooth::Address a{{1}}, b{{2}};
    // No preloaded attribute events: the fake driver only produces them in
    // response to a request issued after connection setup has progressed.
    Reset();
    assert(R_SUCCEEDED(gatt::Discover(0, 260, a)));
    assert(query_times.size() == 1 && query_times[0] >= 2000);
    assert(mtu_connections == std::vector<u32>{260});
    AssertSubscribed(0, 260);
    gatt::Unsubscribe(0, 260);
    assert(unregister_count == 1);

    Reset(DiscoveryMode::AfterMtu);
    assert(R_SUCCEEDED(gatt::Discover(0, 260, a)));
    AssertSubscribed(0, 260);

    // Critical regression: empty pre-bond table must not prevent CreateBond.
    // Emulate the coordinator completing pairing, then receiving attributes.
    Reset(DiscoveryMode::AfterBond);
    assert(R_SUCCEEDED(gatt::Discover(0, 260, a)));
    assert(gatt::g_slots[0].table.count == 0);
    bonded = true;
    AssertSubscribed(0, 260);
    assert(query_times.size() == 2);
    assert(gatt::g_slots[0].table.count == 6);

    // An MTU timeout/refusal is optional, not proof of an unusable HID link.
    for (bool timeout : {true, false}) {
        Reset();
        mtu_reply = false;
        if (!timeout) mtu_result = {7};
        assert(R_SUCCEEDED(gatt::Discover(0, 260, a)));
        AssertSubscribed(0, 260);
        assert(events::now_ms == (timeout ? 7540u : 2540u));
    }

    // A different controller's MTU response must not finish this setup wait.
    Reset(DiscoveryMode::Never);
    mtu_reply = false;
    events::Schedule(Mtu(3), 2100);
    assert(R_SUCCEEDED(gatt::Discover(0, 260, a)));
    assert(events::now_ms == 7500);
    gatt::HidService hid{};
    assert(gatt::Subscribe(0, 260, &hid).value == 2);
    assert(events::now_ms <= 13000); // post-bond failure is still bounded

    // Cancellation is honored during both settling and the MTU wait.
    for (int kind : {events::Disconnect, events::RadioOff}) {
        for (u64 when : {500u, 2050u, 2300u}) {
            Reset();
            events::Schedule(EventRecord{kind, {}}, when);
            assert(gatt::Discover(0, 260, a).value == 4);
            assert(events::now_ms == when);
            if (when < 2000) assert(query_times.empty() && mtu_connections.empty());
        }
    }

    Reset();
    assert(R_SUCCEEDED(gatt::Discover(0, 260, a)));
    AssertSubscribed(0, 260);
    gatt::ResetSlot(0);
    assert(gatt::HasCachedAddress(0, a));
    // No new discovery events on reconnect: reuse the address cache in another slot.
    mode = DiscoveryMode::Never;
    assert(R_SUCCEEDED(gatt::Discover(1, 774, a)));
    AssertSubscribed(1, 774);
    auto late = Cache(260); late.info.client_cache_save.attr_list[0].id.uuid.value = 0;
    gatt::OnEvent(late);
    AssertSubscribed(1, 774);
    for (int i = 0; i < 100; ++i) gatt::OnEvent(Cache(774));
    assert(gatt::TakeRediscovered(1));
    assert(gatt::g_slots[1].table.count == 6);
    // Polling rediscovery between chunks must not clear all previous handles.
    auto chunk = Cache(774);
    chunk.info.client_cache_save.count = 1;
    chunk.info.client_cache_save.attr_list[0] = chunk.info.client_cache_save.attr_list[3];
    gatt::OnEvent(chunk);
    assert(gatt::TakeRediscovered(1));
    assert(gatt::g_slots[1].table.count == 6);
    AssertSubscribed(1, 774);

    events::failure = 26; // raw callback failure must not be labelled successful authentication
    assert(gatt::Subscribe(1, 774, &hid).value == 8);
    events::failure = 0;
    events::disconnect_on_wait = true;
    assert(gatt::Subscribe(1, 774, &hid).value == 4);
    events::disconnect_on_wait = events::disconnected = false;
    events::sleep_on_wait = true;
    assert(gatt::Subscribe(1, 774, &hid).value == 4);
    events::sleep_on_wait = false;
    events::radio = true;

    register_result = {7};
    assert(gatt::Subscribe(1, 774, &hid).value == 7);
    register_result = {0};
    events::queue.clear();
    events::completed = false;
    cccd_auths.clear();
    assert(gatt::Subscribe(1, 774, &hid).value == 2); // zero-0x2a671 rule: HID Info timeout aborts before sending CCCD
    assert(cccd_auths.empty());
    events::completed = true;
    events::disconnected = true;
    assert(gatt::Subscribe(1, 774, &hid).value == 4);
    assert(gatt::Discover(1, 774, a).value == 4);
    events::disconnected = false;
    assert(R_SUCCEEDED(gatt::Discover(1, 1028, b)));
    assert(gatt::g_slots[1].table.count == 0); // new address cannot inherit attributes
    query_result = {7};
    assert(gatt::Subscribe(1, 1028, &hid).value == 7);
    assert(gatt::Discover(4, 1028, b).value == 1);

    // Reconnect security can take a full pairing exchange. Noise from the two
    // active pads, a different operation, or a different characteristic cannot
    // finish our authenticated read. The CCCD waits up to 2s on a live link.
    Reset(); assert(R_SUCCEEDED(gatt::Discover(0, 260, a)));
    unrelated_completions = true;
    completion_delay = 1500;
    const auto started = events::now_ms;
    AssertSubscribed(0, 260);
    assert(events::now_ms == started + 3000);
    events::completed = false;
    assert(gatt::Subscribe(0, 260, &hid).value == 2); // timeout must not proceed into BUSY
    events::completed = true;
    events::failure = 26;
    assert(gatt::Subscribe(0, 260, &hid).value == 8);

    Reset(); assert(R_SUCCEEDED(gatt::Discover(0, 260, a)));
    auth_refusals = 2; AssertSubscribed(0, 260); // locally rejected requests can retry
    auth_refusals = 5;
    assert(gatt::Subscribe(0, 260, &hid).value == 5); // bounded, never plaintext fallback

    Reset(); assert(R_SUCCEEDED(gatt::Discover(0, 260, a)));
    gatt::g_slots[0].table.count = 4; // HID input/CCCD exist, authentication metadata missing
    assert(gatt::Subscribe(0, 260, &hid).value == 3);
    assert(notification_registrations == 0);

    Reset(); assert(R_SUCCEEDED(gatt::Discover(0, 260, a)));
    unrelated_completions = true;
    cccd_completed = false;
    assert(gatt::Subscribe(0, 260, &hid).value == 2); // unrelated noise alone fails CCCD with timeout
    cccd_completed = true;
    cccd_failure = 26;
    assert(gatt::Subscribe(0, 260, &hid).value == 8);

    // CCCD wait also accepts desc_uuid == 0x2902 or incoming Report Notification
    Reset(); assert(R_SUCCEEDED(gatt::Discover(0, 260, a)));
    cccd_failure = 0;
    cccd_use_desc_uuid = true;
    AssertSubscribed(0, 260);
    cccd_use_desc_uuid = false;
    cccd_complete_via_notification = true;
    AssertSubscribed(0, 260);
    cccd_complete_via_notification = false;

    // Stadia DIS PnP ID + output WriteWithResponse selection + >64 attribute capacity
    Reset(DiscoveryMode::Never);
    pnp_id_payload = {0x02, 0xD1, 0x18, 0x00, 0x94, 0x00, 0x01};
    {
        auto &tbl = gatt::g_slots[0].table;
        tbl.address = a;
        u16 h = 1;
        // Pad GAP + DIS near handle 80 (>64 old limit) to verify MaxAttributes = 128
        tbl.attributes[tbl.count++] = {BtdrvGattAttributeType_Service, h++, 10, 0, true, {{GattServiceGap}}};
        tbl.attributes[tbl.count++] = {BtdrvGattAttributeType_Characteristic, h++, 0, 0, true, {{GattCharDeviceName}}};
        for (int i = 0; i < 65; ++i) {
            tbl.attributes[tbl.count++] = {BtdrvGattAttributeType_Characteristic, h++, 0, 0, true, {{0x2A99}}};
        }
        const u16 dis_start = h++;
        tbl.attributes[tbl.count++] = {BtdrvGattAttributeType_Service, dis_start, static_cast<u16>(dis_start + 1), 0, true, {{GattServiceDeviceInfo}}};
        tbl.attributes[tbl.count++] = {BtdrvGattAttributeType_Characteristic, static_cast<u16>(dis_start + 1), 0, 0, true, {{GattCharPnpId}}};
        const u16 hid_start = static_cast<u16>(dis_start + 2);
        tbl.attributes[tbl.count++] = {BtdrvGattAttributeType_Service, hid_start, static_cast<u16>(hid_start + 10), 0, true, {{GattServiceHid}}};
        tbl.attributes[tbl.count++] = {BtdrvGattAttributeType_Characteristic, static_cast<u16>(hid_start + 1), 0, 0, true, {{GattCharHidInformation}}};
        tbl.attributes[tbl.count++] = {BtdrvGattAttributeType_Characteristic, static_cast<u16>(hid_start + 2), 0, 0, true, {{GattCharReportMap}}};
        // Input report with Notify + Write (less preferred for output) followed by dedicated Write-only output report
        tbl.attributes[tbl.count++] = {BtdrvGattAttributeType_Characteristic, static_cast<u16>(hid_start + 3), 0, BtdrvGattCharacteristicProperty_Notify | BtdrvGattCharacteristicProperty_Write, true, {{GattCharReport}}};
        tbl.attributes[tbl.count++] = {BtdrvGattAttributeType_Descriptor, static_cast<u16>(hid_start + 4), 0, 0, true, {{GattDescClientCharConfig}}};
        const u16 out_handle = static_cast<u16>(hid_start + 5);
        tbl.attributes[tbl.count++] = {BtdrvGattAttributeType_Characteristic, out_handle, 0, BtdrvGattCharacteristicProperty_Write, true, {{GattCharReport}}};

        gatt::HidService stadia_hid{};
        assert(R_SUCCEEDED(gatt::Subscribe(0, 260, &stadia_hid)));
        assert(stadia_hid.has_pnp_id && stadia_hid.vendor_id == 0x18D1 && stadia_hid.product_id == 0x9400);
        assert(stadia_hid.has_output_report && stadia_hid.output_report.handle == out_handle);
        assert(stadia_hid.output_write_with_response == true);

        // Best-effort PnP ID failure falls back to GAP Device Name without failing Subscribe
        fail_pnp_read = true;
        const char name[] = "StadiaQPRV-4967";
        device_name_payload.assign(name, name + sizeof(name) - 1);
        gatt::HidService fallback_hid{};
        assert(R_SUCCEEDED(gatt::Subscribe(0, 260, &fallback_hid)));
        assert(!fallback_hid.has_pnp_id);
        assert(std::strcmp(fallback_hid.device_name, "StadiaQPRV-4967") == 0);

        // Best-effort PnP ID timeout must stop immediately without sending GAP Device Name (zero 0x2a671)
        read_uuids.clear();
        fail_pnp_read = false;
        fail_pnp_timeout = true;
        gatt::HidService pnp_timeout_hid{};
        assert(R_SUCCEEDED(gatt::Subscribe(0, 260, &pnp_timeout_hid)));
        assert(read_uuids == std::vector<u16>({GattCharHidInformation, GattCharPnpId}));
        assert(!pnp_timeout_hid.has_pnp_id && pnp_timeout_hid.device_name[0] == '\0');
        fail_pnp_timeout = false;

        // When HID Information read times out in StandardNoMitm, abort immediately without sending CCCD
        read_uuids.clear();
        cccd_auths.clear();
        events::completed = false;
        gatt::HidService timeout_hid{};
        assert(gatt::Subscribe(0, 260, &timeout_hid).value == 2);
        assert(read_uuids.size() == 1 && read_uuids[0] == GattCharHidInformation);
        assert(cccd_auths.empty());
        events::completed = true;

        // NoMitmCccdOnly (Stadia Stage 0): skips 0x2A4A AND skips 0x2A50/0x2A00 (which stall 28.36s on Stadia),
        // writes CCCD directly with NoMitm so bt_nso triggers real BTM_SetEncryption -> SMP_Pair!
        read_uuids.clear(); read_auths.clear(); cccd_auths.clear();
        gatt::HidService nomitm_direct{};
        assert(R_SUCCEEDED(gatt::Subscribe(0, 260, &nomitm_direct, gatt::SubscribeAuthMode::NoMitmCccdOnly)));
        assert(cccd_auths == std::vector<int>({BtdrvGattAuthReqType_NoMitm}));
        assert(nomitm_direct.write_auth_req == BtdrvGattAuthReqType_NoMitm);
        assert(nomitm_direct.has_output_report && nomitm_direct.output_report.handle == out_handle);
        assert(read_uuids.empty()); // Zero DIS/GAP/HID-Info reads!

        // PlainThenNoMitm: skips 0x2A4A & 0x2A50, tries plaintext CCCD first; succeeds with write_auth_req=None
        read_uuids.clear(); read_auths.clear(); cccd_auths.clear();
        gatt::HidService plain_ok{};
        assert(R_SUCCEEDED(gatt::Subscribe(0, 260, &plain_ok, gatt::SubscribeAuthMode::PlainThenNoMitm)));
        assert(cccd_auths == std::vector<int>({BtdrvGattAuthReqType_None}));
        assert(plain_ok.write_auth_req == BtdrvGattAuthReqType_None);
        assert(plain_ok.has_output_report && plain_ok.output_report.handle == out_handle);
        assert(read_uuids.empty());

        // PlainThenNoMitm: plaintext CCCD gets ATT 0x05 (Insufficient Auth -> GATT IDLE), escalates to NoMitm -> succeeds!
        read_uuids.clear(); read_auths.clear(); cccd_auths.clear();
        cccd_plain_failure = 5;
        gatt::HidService escalated{};
        assert(R_SUCCEEDED(gatt::Subscribe(0, 260, &escalated, gatt::SubscribeAuthMode::PlainThenNoMitm)));
        assert(cccd_auths == std::vector<int>({BtdrvGattAuthReqType_None, BtdrvGattAuthReqType_NoMitm}));
        assert(escalated.write_auth_req == BtdrvGattAuthReqType_NoMitm);
        assert(read_uuids.empty());
        cccd_plain_failure = 0;

        // PlainThenNoMitm: plaintext CCCD times out -> must NOT escalate into 0x2a671 BUSY
        cccd_auths.clear();
        cccd_completed = false;
        gatt::HidService plain_to{};
        assert(gatt::Subscribe(0, 260, &plain_to, gatt::SubscribeAuthMode::PlainThenNoMitm).value == 2);
        assert(cccd_auths == std::vector<int>({BtdrvGattAuthReqType_None}));
        cccd_completed = true;

        // PlainOnlyAfterBond (Stadia Stage 1 & 2): skips 0x2A4A & 0x2A50, writes CCCD with None only, never escalates to NoMitm
        read_uuids.clear(); cccd_auths.clear();
        gatt::HidService after_bond{};
        assert(R_SUCCEEDED(gatt::Subscribe(0, 260, &after_bond, gatt::SubscribeAuthMode::PlainOnlyAfterBond)));
        assert(cccd_auths == std::vector<int>({BtdrvGattAuthReqType_None}));
        assert(after_bond.write_auth_req == BtdrvGattAuthReqType_None);
        assert(read_uuids.empty());

        cccd_auths.clear();
        cccd_plain_failure = 5;
        assert(gatt::Subscribe(0, 260, &after_bond, gatt::SubscribeAuthMode::PlainOnlyAfterBond).value == 8);
        assert(cccd_auths == std::vector<int>({BtdrvGattAuthReqType_None}));
        cccd_plain_failure = 0;
    }

    std::cout << "GATT discovery/authentication/completion isolation/cache/cancellation regressions passed\n";
}
