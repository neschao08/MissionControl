// Compile production ble_events.cpp unchanged to verify:
// 1. Descending client_if allocation (6,5,4,3) consistent between GetClientIfForSlot and GetSleepLink.
// 2. Dual-path managed BLE input report dispatch outside g_lock without overflowing the 64-entry ring.
// 3. Rumble CharacteristicWrite (0x2A4D) swallowing to prevent ring overflow & dropped disconnects.
// 4. Per-slot input report counting isolation and BluetoothCrash / core event handling.
#include "ble/ble_events.cpp"
#include <cassert>
#include <iostream>
#include <vector>

using namespace ams;
using namespace ams::ble;

static u32 g_ssp_replies = 0;
Result btdrvRespondToSspRequest(BtdrvAddress, u32, bool, s32) { ++g_ssp_replies; return {0}; }
Result btdrvSetBleConnectionParameter(BtdrvAddress, const BtdrvBleConnectionParameter *, bool) { return {0}; }

class MockController : public controller::EmulatedSwitchController {
    public:
        u32 received_count = 0;
        std::vector<u8> last_packet;
        Result HandleBleInputReport(const u8 *data, size_t size) override {
            ++received_count;
            last_packet.assign(data, data + size);
            return {0};
        }
};

static void EmitClientReg(u8 client_if, u8 status = 1) {
    BtdrvBleEventInfo info{};
    info.client_registration = {0, client_if, status, {0, 0}};
    events::OnBleEvent(BtdrvBleEventType_ClientRegistration, &info);
}

static void DrainAll() {
    EventRecord rec{};
    while (events::Pop(&rec)) {}
}

int main() {
    events::Activate();
    events::TakeOwnership();
    events::SetRadioEnabled(true);

    // 1. btm registers client_ifs 3, 4, 5, 6 -> verify descending allocation 6, 5, 4, 3
    for (u8 cif : {3u, 4u, 5u, 6u}) EmitClientReg(cif, 1);
    assert(events::GetAvailableClientIfCount() == 4);
    assert(events::GetClientIfForSlot(0) == 6);
    assert(events::GetClientIfForSlot(1) == 5);
    assert(events::GetClientIfForSlot(2) == 4);
    assert(events::GetClientIfForSlot(3) == 3);

    // Verify GetSleepLink uses the exact same client_if mapping as GetClientIfForSlot
    const bluetooth::Address stadia_addr = {{0xcb, 0x4e, 0xc0, 0x96, 0x49, 0x67}};
    const bluetooth::Address xbox_addr   = {{0x14, 0xcb, 0x65, 0xe8, 0x2b, 0xe2}};
    events::SetSlotConnection(0, stadia_addr, 259);
    events::SetSlotConnection(1, xbox_addr, 516);
    assert(events::GetSleepLink(0).client_if == 6 && events::GetSleepLink(0).conn_id == 259);
    assert(events::GetSleepLink(1).client_if == 5 && events::GetSleepLink(1).conn_id == 516);

    DrainAll();
    static_cast<void>(events::TakeRingDropped());

    // 2. Notification BEFORE controller is presented -> must go into ring for Subscribe CCCD confirmation
    BtdrvBleEventInfo notif{};
    notif.client_notify.result = 0;
    notif.client_notify.conn_id = 259;
    notif.client_notify.op_type = BtdrvGattOperationType_Notification;
    notif.client_notify.char_uuid = MakeUuid16(GattCharReport);
    notif.client_notify.size = 10;
    const u8 pkt[10] = {0x08, 0x00, 0x20, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00};
    std::memcpy(notif.client_notify.data, pkt, 10);

    assert(events::OnBleEvent(BtdrvBleEventType_ClientNotify, &notif) == true);
    EventRecord popped{};
    assert(events::Pop(&popped) && popped.type == BtdrvBleEventType_ClientNotify);
    assert(events::GetSlotInputReportCount(0) == 0);

    // 3. Register controller on slot 0 -> flood 500 notifications on managed BLE stream
    auto ctrl0 = std::make_shared<MockController>();
    events::RegisterSlotController(0, 259, ctrl0);
    for (int i = 0; i < 500; ++i) {
        assert(events::OnBleEvent(BtdrvBleEventType_ClientNotify, &notif) == true);
    }
    assert(ctrl0->received_count == 500);
    assert(ctrl0->last_packet.size() == 10 && ctrl0->last_packet[2] == 0x20);
    assert(events::GetSlotInputReportCount(0) == 500);
    assert(events::GetSlotInputReportCount(1) == 0); // Per-slot isolation!
    assert(events::TakeRingDropped() == 0);          // 0 drops despite 500 notifications >> RingSize(64)!
    assert(!events::Pop(&popped));                   // Ring stays completely empty for control events!

    // 4. Flood 500 rumble CharacteristicWrite (0x2A4D) completions -> swallowed cleanly, 0 ring drops!
    BtdrvBleEventInfo wr_ack{};
    wr_ack.client_notify.result = 0;
    wr_ack.client_notify.conn_id = 259;
    wr_ack.client_notify.op_type = BtdrvGattOperationType_CharacteristicWrite;
    wr_ack.client_notify.char_uuid = MakeUuid16(GattCharReport);
    for (int i = 0; i < 500; ++i) {
        assert(events::OnBleEvent(BtdrvBleEventType_ClientNotify, &wr_ack) == true);
    }
    assert(events::TakeRingDropped() == 0);
    assert(!events::Pop(&popped));

    // Non-zero rumble write ATT error is captured for Stadia auto-fallback without entering the control ring
    wr_ack.client_notify.result = 5;
    assert(events::OnBleEvent(BtdrvBleEventType_ClientNotify, &wr_ack) == true);
    assert(!events::Pop(&popped));
    u32 captured_rumble_err = 0;
    assert(events::TakeRumbleWriteError(259, &captured_rumble_err) && captured_rumble_err == 5);
    assert(!events::TakeRumbleWriteError(259, &captured_rumble_err));

    // Subsequent disconnect event MUST arrive intact without being dropped
    BtdrvBleEventInfo disc{};
    disc.client_connection.result = 0;
    disc.client_connection.status = BleConnectionStatus_Disconnected;
    disc.client_connection.conn_id = 259;
    disc.client_connection.address = stadia_addr;
    disc.client_connection.reason = 0x13;
    assert(events::OnBleEvent(BtdrvBleEventType_ClientConnection, &disc) == true);
    assert(events::IsSlotDisconnected(0) && events::GetSlotDisconnectReason(0) == 0x13);
    assert(events::GetActiveController(259) == nullptr);
    assert(events::Pop(&popped) && popped.type == BtdrvBleEventType_ClientConnection);

    // 5. Core events (SSP + Connection type=3 + BluetoothCrash reset)
    events::SetSlotConnection(0, stadia_addr, 771);
    BtdrvEventInfo core_conn{};
    core_conn.connection.v12.type = 3;
    core_conn.connection.v12.addr = stadia_addr;
    core_conn.connection.v12.reserved[0] = 0xAB;
    assert(events::OnCoreEvent(BtdrvEventType_Connection, &core_conn) == true);
    assert(events::TakeSlotBondComplete(0) == true);
    assert(events::TakeSlotBondComplete(0) == false);

    BtdrvEventInfo ssp{};
    ssp.ssp_request.v12.addr = stadia_addr;
    ssp.ssp_request.v12.passkey = 123456;
    assert(events::OnCoreEvent(BtdrvEventType_SspRequest, &ssp) == true);
    assert(g_ssp_replies == 1);

    // BluetoothCrash clears all client_ifs and marks active slots disconnected, forwarded to btm (returns false)
    BtdrvEventInfo crash{};
    assert(events::OnCoreEvent(BtdrvEventType_BluetoothCrash, &crash) == false);
    assert(events::GetAvailableClientIfCount() == 0);
    assert(events::IsSlotDisconnected(0) && events::IsSlotDisconnected(1));

    std::cout << "BLE production events (dual-path dispatch, rumble filter, descending client_if) regressions passed\n";
}
