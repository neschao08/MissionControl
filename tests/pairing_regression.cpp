#include "ble_pairing.cpp"
#include <cassert>
#include <iostream>
using namespace ams::ble;
static constexpr ams::bluetooth::Address Target = {{0x14,0xcb,0x65,0xe8,0x2b,0xe2}};
static unsigned removes, ext_removes, connects, cancels, bond_cancels, bonds, set_encryptions, ble_sec_grants;
static u64 connected_at;
static bool remove_fails, ext_remove_fails, cancel_bond_fails, connect_fails, respond, late_connection, new_bond, bond_fails;
static int fail_connect_event_once = 0;
static u32 set_encryption_raw_rc;
static bool set_encryption_emits_bond;
static bool callback_on_bond_cancel, radio_off_on_bond_cancel;
static Result Connect() {
    u32 conn_id = InvalidConnId;
    return pairing::Connect(0, 3, Target, &conn_id, TimeSpan::FromSeconds(15));
}
Result btdrvRemoveBond(ams::bluetooth::Address addr) {
    assert(AddressEqual(addr, Target)); ++removes;
    if (late_connection) events::conn_id = 259;
    return {remove_fails ? 5u : 0u};
}
Result btdrvCancelBond(ams::bluetooth::Address addr) {
    assert(AddressEqual(addr, Target)); ++bond_cancels;
    if (callback_on_bond_cancel) {
        events::bond = true;
        events::disconnected = true;
    }
    if (late_connection) {
        events::disconnected = false;
        events::conn_id = 259;
    }
    if (radio_off_on_bond_cancel) events::radio = false;
    return {cancel_bond_fails ? 8u : 0u};
}
Result btdrvConnectGattServer(u8 client, ams::bluetooth::Address addr, bool direct, u64 aruid) {
    assert(client == 3 && AddressEqual(addr, Target) && direct && aruid == 0);
    ++connects; connected_at = events::now_ms;
    if (fail_connect_event_once > 0) {
        --fail_connect_event_once;
        EventRecord rej{};
        rej.info.client_connection.result = 1;
        rej.info.client_connection.status = 1;
        rej.info.client_connection.client_if = 3;
        rej.info.client_connection.conn_id = 0;
        rej.info.client_connection.address = Target;
        rej.info.client_connection.reason = 0x26;
        events::Schedule(BtdrvBleEventType_ClientConnection, 50, rej);
        return {connect_fails ? 6u : 0u};
    }
    if (respond) {
        EventRecord record;
        record.info.client_connection.address = Target;
        events::Schedule(BtdrvBleEventType_ClientConnection, 20, record);
    }
    return {connect_fails ? 6u : 0u};
}
Result btdrvCancelConnectGattServer(u8 client, ams::bluetooth::Address addr, bool direct) {
    assert(client == 3 && AddressEqual(addr, Target) && direct && events::radio);
    ++cancels; return {0};
}
Result btdrvCreateBond(ams::bluetooth::Address addr, u32 transport) {
    assert(AddressEqual(addr, Target) && transport == 2 && events::radio && !events::disconnected);
    ++bonds;
    if (new_bond) events::Schedule(events::Bond, 100);
    return {bond_fails ? 7u : 0u};
}
Result btdrvextSecRemoveDevice(ams::bluetooth::Address addr) {
    assert(AddressEqual(addr, Target));
    ++ext_removes;
    return {ext_remove_fails ? 99u : 0u};
}
Result btdrvextSetEncryption(ams::bluetooth::Address addr, u8 transport, u8 sec_act) {
    assert(AddressEqual(addr, Target) && transport == 2 && sec_act == 2 && events::radio && !events::disconnected);
    ++set_encryptions;
    if (set_encryption_emits_bond) events::Schedule(events::Bond, 250);
    return {set_encryption_raw_rc};
}
Result btdrvextBleSecurityGrant(ams::bluetooth::Address addr, u8 res) {
    assert(AddressEqual(addr, Target) && res == 0 && events::radio && !events::disconnected);
    ++ble_sec_grants;
    return {0};
}
static void Reset() {
    events::queue.clear(); events::now_ms = 0;
    events::radio = true;
    events::SetSlotConnecting(0, Target);
    removes = ext_removes = connects = cancels = bond_cancels = bonds = set_encryptions = ble_sec_grants = 0; connected_at = 0;
    remove_fails = ext_remove_fails = cancel_bond_fails = connect_fails = late_connection = bond_fails = false;
    set_encryption_raw_rc = 0;
    set_encryption_emits_bond = false;
    callback_on_bond_cancel = radio_off_on_bond_cancel = false;
    fail_connect_event_once = 0;
    respond = new_bond = true;
}
int main() {
    Reset(); assert(R_SUCCEEDED(Connect()));
    assert(removes == 1 && connects == 1 && cancels == 0 && connected_at == 0);
    assert(pairing::PrepareSecurity(0, Target)); assert(bonds == 1 && events::now_ms == 620);
    Reset(); late_connection = true;
    u32 adopted = InvalidConnId;
    assert(R_SUCCEEDED(pairing::Connect(0, 3, Target, &adopted, TimeSpan::FromSeconds(15))));
    assert(adopted == 259 && removes == 1 && connects == 0); // preserve a late live transport
    Reset(); respond = false;
    EventRecord invalid;
    invalid.info.client_connection.address = Target;
    invalid.info.client_connection.conn_id = InvalidConnId;
    events::Schedule(BtdrvBleEventType_ClientConnection, 10, invalid);
    assert(R_FAILED(Connect()) && cancels == 1);
    Reset(); remove_fails = true; assert(R_FAILED(Connect())); assert(connects == 0);
    Reset(); connect_fails = true; assert(R_FAILED(Connect()));
    Reset(); respond = false; assert(R_FAILED(Connect())); assert(cancels == 1 && events::now_ms == 15500);
    Reset(); respond = false; events::Schedule(events::Sleep, 200); assert(R_FAILED(Connect())); assert(cancels == 0);
    Reset(); respond = false;
    EventRecord wrong;
    wrong.info.client_connection.address = Target;
    wrong.info.client_connection.client_if = 4;
    events::Schedule(BtdrvBleEventType_ClientConnection, 10, wrong);
    assert(R_FAILED(Connect())); assert(cancels == 1); // another client's completion isn't ours
    Reset(); events::radio = false; assert(R_FAILED(Connect())); assert(removes == 0);
    Reset(); assert(R_SUCCEEDED(pairing::ClearPeer(0, Target)));
    assert(bond_cancels == 1 && removes == 1 && connects == 0);
    Reset(); events::conn_id = 260;
    assert(R_FAILED(pairing::ClearPeer(0, Target)) && bond_cancels == 0 && removes == 0); // live slot
    Reset(); events::disconnected = true; events::conn_id = 260;
    assert(R_SUCCEEDED(pairing::ClearPeer(0, Target))); // disconnected event makes the stale id non-live
    assert(bond_cancels == 1 && removes == 1);
    Reset(); events::radio = false;
    assert(R_FAILED(pairing::ClearPeer(0, Target)) && bond_cancels == 0 && removes == 0); // asleep
    Reset(); cancel_bond_fails = true;
    const Result cancel_failure = pairing::ClearPeer(0, Target);
    assert(R_FAILED(cancel_failure) && cancel_failure.GetValue() == 8 && bond_cancels == 1 && removes == 1);
    Reset(); remove_fails = true;
    const Result remove_failure = pairing::ClearPeer(0, Target);
    assert(R_FAILED(remove_failure) && remove_failure.GetValue() == 5 && bond_cancels == 1 && removes == 1);
    Reset(); late_connection = true;
    assert(R_FAILED(pairing::ClearPeer(0, Target)) && bond_cancels == 1 && removes == 0); // late transport during cancel
    Reset(); radio_off_on_bond_cancel = true;
    assert(R_FAILED(pairing::ClearPeer(0, Target)) && bond_cancels == 1 && removes == 0); // radio sleeps between calls
    Reset(); callback_on_bond_cancel = true;
    assert(R_SUCCEEDED(pairing::ClearPeer(0, Target)) && bond_cancels == 1 && removes == 1);
    assert(events::bond && events::disconnected);
    // Connect re-arms after this synchronous cleanup callback; the next
    // pairing attempt still needs a fresh security indication.
    Reset(); callback_on_bond_cancel = true;
    assert(R_SUCCEEDED(pairing::ClearPeer(0, Target)));
    callback_on_bond_cancel = false;
    assert(R_SUCCEEDED(Connect()) && removes == 2 && connects == 1);
    assert(pairing::PrepareSecurity(0, Target) && bonds == 1);
    Reset(); events::bond = true; new_bond = false;
    assert(!pairing::PrepareSecurity(0, Target)); assert(bonds == 1 && events::now_ms == 10000); // stale indication
    Reset(); assert(pairing::PrepareSecurity(0, Target)); assert(events::now_ms == 600);
    Reset(); bond_fails = true; assert(!pairing::PrepareSecurity(0, Target)); assert(events::now_ms == 0);
    Reset();
    EventRecord refused;
    refused.info.client_connection.address = Target;
    refused.info.client_connection.status = 2;
    refused.info.client_connection.reason = 0x16;
    events::Schedule(BtdrvBleEventType_ClientConnection, 110, refused);
    assert(!pairing::PrepareSecurity(0, Target)); assert(events::now_ms == 110);
    Reset(); events::Schedule(events::Sleep, 200); assert(!pairing::PrepareSecurity(0, Target));
    Reset(); events::disconnected = true; assert(!pairing::PrepareSecurity(0, Target)); assert(bonds == 0);
    Reset(); events::radio = false; assert(!pairing::PrepareSecurity(0, Target)); assert(bonds == 0);

    // Stadia-style early core connection indication (< 50ms): wait for 1500ms SMP settle on live link
    Reset(); new_bond = false;
    events::Schedule(events::Bond, 3);
    assert(pairing::PrepareSecurity(0, Target));
    assert(bonds == 1 && events::now_ms == 1503);

    // Early indication followed by a second core connection indication: finish 500ms after the second
    Reset(); new_bond = false;
    events::Schedule(events::Bond, 3);
    events::Schedule(events::Bond, 300);
    assert(pairing::PrepareSecurity(0, Target));
    assert(bonds == 1 && events::now_ms == 800);

    // Disconnect during early bond settle fails promptly
    Reset(); new_bond = false;
    events::Schedule(events::Bond, 3);
    events::Schedule(BtdrvBleEventType_ClientConnection, 200, refused);
    assert(!pairing::PrepareSecurity(0, Target));
    assert(bonds == 1 && events::now_ms == 200);

    // Best-effort IPS BSA_SecRemoveDevice (cmd 4) runs in Connect & ClearPeer without aborting on error
    Reset(); ext_remove_fails = true;
    assert(R_SUCCEEDED(Connect()));
    assert(removes == 1 && ext_removes == 1 && connects == 1);
    Reset(); ext_remove_fails = true;
    assert(R_SUCCEEDED(pairing::ClearPeer(0, Target)));
    assert(bond_cancels == 1 && removes == 1 && ext_removes == 1);

    // Strategy 0: GattOnDemand succeeds immediately without calling CreateBond or SetEncryption
    Reset(); events::bond = true;
    assert(pairing::PrepareSecurityWithStrategy(0, Target, pairing::SecurityStrategy::GattOnDemand));
    assert(bonds == 0 && set_encryptions == 0 && !events::bond && events::now_ms == 0);
    Reset(); events::disconnected = true;
    assert(!pairing::PrepareSecurityWithStrategy(0, Target, pairing::SecurityStrategy::GattOnDemand));
    Reset(); events::radio = false;
    assert(!pairing::PrepareSecurityWithStrategy(0, Target, pairing::SecurityStrategy::GattOnDemand));

    // Strategy 1: DirectSetEncryption ignores non-zero residual w0 return value from void BTA_DmSetEncryption
    Reset(); set_encryption_raw_rc = 0xDEADBEEF;
    assert(pairing::PrepareSecurityWithStrategy(0, Target, pairing::SecurityStrategy::DirectSetEncryption));
    assert(bonds == 0 && set_encryptions == 1 && events::now_ms == 2000);

    // DirectSetEncryption with bond event settles 500ms after event
    Reset(); set_encryption_raw_rc = 0x123456; set_encryption_emits_bond = true;
    assert(pairing::PrepareSecurityWithStrategy(0, Target, pairing::SecurityStrategy::DirectSetEncryption));
    assert(bonds == 0 && set_encryptions == 1 && events::now_ms == 750);

    // DirectSetEncryption aborts immediately on disconnect or sleep
    Reset(); set_encryption_raw_rc = 0x42;
    events::Schedule(BtdrvBleEventType_ClientConnection, 180, refused);
    assert(!pairing::PrepareSecurityWithStrategy(0, Target, pairing::SecurityStrategy::DirectSetEncryption));
    assert(set_encryptions == 1 && events::now_ms == 180);
    Reset(); events::Schedule(events::Sleep, 120);
    assert(!pairing::PrepareSecurityWithStrategy(0, Target, pairing::SecurityStrategy::DirectSetEncryption));
    assert(set_encryptions == 1 && events::now_ms == 120);

    // Strategy 2: CreateBond delegates directly to PrepareSecurity
    Reset();
    assert(pairing::PrepareSecurityWithStrategy(0, Target, pairing::SecurityStrategy::CreateBond));
    assert(bonds == 1 && set_encryptions == 0 && ble_sec_grants == 0 && events::now_ms == 600);

    // Strategy 3: BleSecurityGrant invokes IPS cmd 6 twice (0ms + 150ms) and settles for 750ms
    Reset();
    assert(pairing::PrepareSecurityWithStrategy(0, Target, pairing::SecurityStrategy::BleSecurityGrant));
    assert(ble_sec_grants == 2 && bonds == 0 && set_encryptions == 0 && events::now_ms == 900);

    // BleSecurityGrant fast_reconnect keeps steady 150ms + 750ms = 900ms settle so CCCD clears Stadia ~2.0s SMP
    Reset();
    assert(pairing::PrepareSecurityWithStrategy(0, Target, pairing::SecurityStrategy::BleSecurityGrant, true));
    assert(ble_sec_grants == 2 && bonds == 0 && set_encryptions == 0 && events::now_ms == 900);

    // BleSecurityGrant aborts promptly if link drops during 150ms or 750ms window
    Reset();
    events::Schedule(BtdrvBleEventType_ClientConnection, 100, refused);
    assert(!pairing::PrepareSecurityWithStrategy(0, Target, pairing::SecurityStrategy::BleSecurityGrant));
    assert(ble_sec_grants == 1 && events::now_ms == 100);
    Reset();
    events::Schedule(BtdrvBleEventType_ClientConnection, 500, refused);
    assert(!pairing::PrepareSecurityWithStrategy(0, Target, pairing::SecurityStrategy::BleSecurityGrant));
    assert(ble_sec_grants == 2 && events::now_ms == 500);

    // Connect with clear_existing_bond=false preserves existing LTK record (no RemoveBond / SecRemoveDevice)
    Reset();
    {
        u32 cid = InvalidConnId;
        assert(R_SUCCEEDED(pairing::Connect(0, 3, Target, &cid, TimeSpan::FromSeconds(15), false)));
        assert(removes == 0 && ext_removes == 0 && connects == 1);
    }

    // Immediate (<800ms) stack refusal during Classic inquiry (result != 0, reason != 0x003e) retries cleanly
    Reset();
    fail_connect_event_once = 1;
    assert(R_SUCCEEDED(Connect()));
    assert(connects == 2 && removes == 1 && cancels == 1 && events::now_ms == 570);

    // Direct background probe with allow_refusal_retry=false fails immediately without retry storm
    Reset();
    fail_connect_event_once = 1;
    {
        u32 cid = InvalidConnId;
        assert(R_FAILED(pairing::Connect(0, 3, Target, &cid, TimeSpan::FromMilliSeconds(1800), false, false)));
        assert(connects == 1 && cancels == 1 && removes == 0 && events::now_ms == 50);
    }

    std::cout << "BLE production reconnect/pairing regressions passed\n";
}
