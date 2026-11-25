// Compile production stadia_ble_controller.cpp unchanged to verify packet framing,
// disconnection locking, write_auth_req propagation, and Write-With-Response rumble throttling.
#include "stadia_ble_controller.cpp"
#include <cassert>
#include <iostream>
#include <vector>

using namespace ams;
using namespace ams::controller;

struct GattWriteCall {
    u32 conn_id;
    bool primary;
    u16 service_uuid;
    u16 char_uuid;
    u8 char_inst;
    std::vector<u8> payload;
    BtdrvGattAuthReqType auth_req;
    bool with_response;
};

static std::vector<GattWriteCall> g_writes;
static u32 g_write_rc = 0;

Result btdrvWriteGattCharacteristic(u32 conn_id, bool primary, const BtdrvGattId *service_id, const BtdrvGattId *char_id, const void *data, size_t size, BtdrvGattAuthReqType auth_req, bool with_response) {
    const u8 *bytes = static_cast<const u8 *>(data);
    g_writes.push_back({
        conn_id, primary, service_id->uuid.value, char_id->uuid.value, char_id->instance_id,
        std::vector<u8>(bytes, bytes + size), auth_req, with_response
    });
    return {g_write_rc};
}

int main() {
    const bluetooth::Address addr = {{0xcb, 0x4e, 0xc0, 0x96, 0x49, 0x67}};
    const BtdrvGattId svc_id = {{0x1812}, 0};
    const BtdrvGattId out_id = {{0x2A4D}, 1}; // Real Stadia handle 0x0044 inst=1

    // 1. Test AuthReq=None propagation + 10-byte stripped & 11-byte prefixed packets + invalid size rejection
    {
        g_writes.clear(); g_write_rc = 0;
        ble::events::g_radio_enabled = true;
        controller::g_connected_controllers = 1;
        os::g_now_ms = 1000;

        StadiaBleController ctrl(addr, 259, svc_id, true, out_id, true, true, BtdrvGattAuthReqType_None);
        assert(ctrl.TestDeviceType() == 0x03);

        // 10-byte stripped HOGP packet: East (printed B = 0x20) -> Switch A, L+R (0x06)
        const u8 pkt10[10] = {0x08, 0x00, 0x26, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00};
        assert(R_SUCCEEDED(ctrl.HandleBleInputReport(pkt10, sizeof(pkt10))));
        assert(ctrl.TestDataEvents() == 1);
        assert(ctrl.TestButtons() == (stadia::ButtonBit::A | stadia::ButtonBit::L | stadia::ButtonBit::R));

        // 11-byte prefixed packet (leading 0x03): South (printed A = 0x40) -> Switch B, Capture (0x01) -> Home, Stadia (0x10) ignored
        const u8 pkt11[11] = {0x03, 0x08, 0x11, 0x40, 0x01, 0xFF, 0xFF, 0x01, 0x80, 0x7F, 0x00};
        assert(R_SUCCEEDED(ctrl.HandleBleInputReport(pkt11, sizeof(pkt11))));
        assert(ctrl.TestDataEvents() == 2);
        assert(ctrl.TestButtons() == (stadia::ButtonBit::B | stadia::ButtonBit::Home | stadia::ButtonBit::ZL));
        assert(ctrl.TestLeftX() == 0 && ctrl.TestLeftY() == 0);
        assert(ctrl.TestRightX() == 4095 && ctrl.TestRightY() == 4095);

        // Invalid sizes & wrong report id must be rejected safely
        const u8 bad_id[11] = {0x01, 0x08, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00};
        const u8 short_pkt[9] = {};
        const u8 long_pkt[12] = {};
        assert(ctrl.HandleBleInputReport(bad_id, sizeof(bad_id)).value == 10);
        assert(ctrl.HandleBleInputReport(short_pkt, sizeof(short_pkt)).value == 10);
        assert(ctrl.HandleBleInputReport(long_pkt, sizeof(long_pkt)).value == 10);
        assert(ctrl.HandleBleInputReport(pkt10, 0).value == 10);
        assert(ctrl.TestDataEvents() == 2);

        // 2. Rumble: 4-byte LE strong/weak (with dual-motor startup curve), auth_req=None, with_response=true, inst=1
        SwitchMotorData motor{};
        motor.left_motor.low_band_amp = 1.0f;   // strong = 0xFFFF
        motor.right_motor.high_band_amp = 0.5f; // weak = MapRumbleMagnitude(0.5f, false)
        const u16 exp_weak = stadia::MapRumbleMagnitude(0.5f, false);
        assert(R_SUCCEEDED(ctrl.TestSetVibration(&motor)));
        assert(g_writes.size() == 1);
        assert(g_writes[0].conn_id == 259 && g_writes[0].char_inst == 1);
        assert(g_writes[0].auth_req == BtdrvGattAuthReqType_None && g_writes[0].with_response == true);
        assert(g_writes[0].payload == std::vector<u8>({0xFF, 0xFF, static_cast<u8>(exp_weak & 0xFF), static_cast<u8>(exp_weak >> 8)}));

        // ATT error on rumble write auto-escalates auth None -> NoMitm, then WithRsp -> NoRsp
        ctrl.OnRumbleWriteComplete(5);
        os::g_now_ms += 60;
        motor.left_motor.low_band_amp = 0.4f;
        assert(R_SUCCEEDED(ctrl.TestSetVibration(&motor)));
        assert(g_writes.size() == 2);
        assert(g_writes[1].auth_req == BtdrvGattAuthReqType_NoMitm && g_writes[1].with_response == true);
        ctrl.OnRumbleWriteComplete(0);

        // Reset back to baseline controller for remaining throttle tests
        g_writes.clear();
        ble::events::g_pending_rumble_error = 0;
        StadiaBleController ctrl_t(addr, 259, svc_id, true, out_id, true, true, BtdrvGattAuthReqType_None);
        motor.left_motor.low_band_amp = 1.0f;
        assert(R_SUCCEEDED(ctrl_t.TestSetVibration(&motor)));
        assert(g_writes.size() == 1);
        ctrl_t.OnRumbleWriteComplete(0);

        // Write-With-Response throttle: 24ms is throttled (< 36ms), 36ms passes!
        os::g_now_ms += 24;
        motor.left_motor.low_band_amp = 0.5f;
        assert(R_SUCCEEDED(ctrl_t.TestSetVibration(&motor)));
        assert(g_writes.size() == 1); // Throttled!

        os::g_now_ms += 12; // total 36ms
        assert(R_SUCCEEDED(ctrl_t.TestSetVibration(&motor)));
        assert(g_writes.size() == 2);
        ctrl_t.OnRumbleWriteComplete(0);

        // Transient GATT BUSY (0x2a671) on non-stop update backs off m_last_rumble_tick
        os::g_now_ms += 120;
        g_write_rc = 0x2a671;
        motor.left_motor.low_band_amp = 0.6f;
        assert(ctrl_t.TestSetVibration(&motor).value == 0x2a671);
        assert(g_writes.size() == 3);
        // 16ms later: backed off, does NOT hammer GATT channel immediately
        os::g_now_ms += 16;
        assert(R_SUCCEEDED(ctrl_t.TestSetVibration(&motor)));
        assert(g_writes.size() == 3);

        // Transient GATT BUSY on STOP packet must queue pending STOP so ServiceBleTasks() / next frame retries STOP!
        SwitchMotorData stop_motor{};
        os::g_now_ms += 16;
        assert(ctrl_t.TestSetVibration(&stop_motor).value == 0x2a671);
        assert(g_writes.size() == 4);
        g_write_rc = 0;
        os::g_now_ms += 16; // Even 16ms later, STOP is retried immediately via ServiceBleTasks()!
        ctrl_t.ServiceBleTasks();
        assert(g_writes.size() == 5);
        assert(g_writes[4].payload == std::vector<u8>({0x00, 0x00, 0x00, 0x00}));
        ctrl_t.OnRumbleWriteComplete(0);

        // Duplicate STOP is suppressed
        os::g_now_ms += 100;
        assert(R_SUCCEEDED(ctrl_t.TestSetVibration(&stop_motor)));
        assert(g_writes.size() == 5);

        // Radio off blocks GATT writes safely
        ble::events::g_radio_enabled = false;
        motor.left_motor.low_band_amp = 0.9f;
        assert(R_FAILED(ctrl_t.TestSetVibration(&motor)));
        ble::events::g_radio_enabled = true;

        // DisconnectBle blocks subsequent input & output reports safely
        const u8 basic_out[2] = {0x10, 0x00};
        assert(R_SUCCEEDED(ctrl.HandleOutputDataReport(basic_out, sizeof(basic_out))));
        assert(ctrl.TestOutputCalls() == 1);

        // CommandOutputReport subcmd 0x04 / 0x30 injects extra basic input report (pairing/grip confirmation)
        const u32 inject_before = ctrl.TestInjectCalls();
        u8 cmd04[12] = {}; cmd04[0] = 0x01; cmd04[10] = 0x04;
        assert(R_SUCCEEDED(ctrl.HandleOutputDataReport(cmd04, sizeof(cmd04))));
        u8 cmd30[12] = {}; cmd30[0] = 0x01; cmd30[10] = 0x30;
        assert(R_SUCCEEDED(ctrl.HandleOutputDataReport(cmd30, sizeof(cmd30))));
        assert(ctrl.TestInjectCalls() == inject_before + 2);

        ctrl.DisconnectBle();
        assert(ctrl.HandleBleInputReport(pkt10, sizeof(pkt10)).value == 11);
        assert(ctrl.HandleOutputDataReport(basic_out, sizeof(basic_out)).value == 11);
    }

    // 3. Verify NoMitm auth_req propagation and has_output_report=false safety
    {
        g_writes.clear(); g_write_rc = 0;
        os::g_now_ms = 5000;
        StadiaBleController ctrl_nomitm(addr, 515, svc_id, true, out_id, true, false, BtdrvGattAuthReqType_NoMitm);
        SwitchMotorData motor{};
        motor.left_motor.low_band_amp = 0.5f;
        assert(R_SUCCEEDED(ctrl_nomitm.TestSetVibration(&motor)));
        assert(g_writes.size() == 1 && g_writes[0].auth_req == BtdrvGattAuthReqType_NoMitm && g_writes[0].with_response == false);

        // WriteNoResponse uses 32ms interval
        os::g_now_ms += 32;
        motor.left_motor.low_band_amp = 0.7f;
        assert(R_SUCCEEDED(ctrl_nomitm.TestSetVibration(&motor)));
        assert(g_writes.size() == 2);

        // Multi-controller throttling (4 controllers -> 96ms)
        controller::g_connected_controllers = 4;
        os::g_now_ms += 64;
        motor.left_motor.low_band_amp = 0.9f;
        assert(R_SUCCEEDED(ctrl_nomitm.TestSetVibration(&motor)));
        assert(g_writes.size() == 2); // throttled under 96ms
        os::g_now_ms += 32; // total 96ms
        assert(R_SUCCEEDED(ctrl_nomitm.TestSetVibration(&motor)));
        assert(g_writes.size() == 3);

        // has_output_report=false is a clean no-op
        StadiaBleController ctrl_no_out(addr, 515, svc_id, true, out_id, false, true, BtdrvGattAuthReqType_None);
        assert(R_SUCCEEDED(ctrl_no_out.TestSetVibration(&motor)));
        assert(g_writes.size() == 3);
    }

    // 4. Initialize() writes 0x6012=0x03 & pure-white 0x6050 colours; SendConnectRumble() sends pulse & retries auth
    {
        g_writes.clear(); g_write_rc = 0;
        ble::events::g_radio_enabled = true;
        StadiaBleController ctrl_c(addr, 259, svc_id, true, out_id, true, true, BtdrvGattAuthReqType_None);
        assert(R_SUCCEEDED(ctrl_c.Initialize()));
        assert(ctrl_c.TestSpiDeviceTypeWrites() == 1 && ctrl_c.TestSpiDeviceType() == 0x03);
        assert(ctrl_c.TestColourWrites() == 1);
        const auto cols = ctrl_c.TestColours();
        assert(cols.body.r == 0xf5 && cols.body.g == 0xf5 && cols.body.b == 0xf5);
        assert(cols.buttons.r == 0xb4 && cols.buttons.g == 0xb4 && cols.buttons.b == 0xb4);
        assert(cols.left_grip.r == 0xf5 && cols.right_grip.r == 0xf5);

        // Connect pulse: strong=0x9000 weak=0x9000, non-busy error triggers one NoMitm retry
        g_write_rc = 0x1234;
        assert(R_FAILED(ctrl_c.SendConnectRumble(0x9000, 0x9000)));
        assert(g_writes.size() == 2);
        assert(g_writes[0].auth_req == BtdrvGattAuthReqType_None && g_writes[1].auth_req == BtdrvGattAuthReqType_NoMitm);
        assert(g_writes[0].payload == std::vector<u8>({0x00, 0x90, 0x00, 0x90}));

        // GATT BUSY (0x2a671) must NOT blindly escalate auth
        g_writes.clear();
        StadiaBleController ctrl_busy(addr, 259, svc_id, true, out_id, true, true, BtdrvGattAuthReqType_None);
        g_write_rc = 0x2a671;
        assert(ctrl_busy.SendConnectRumble(0x9000, 0x9000).value == 0x2a671);
        assert(g_writes.size() == 1);
        assert(g_writes[0].auth_req == BtdrvGattAuthReqType_None);

        // Stop pulse succeeds cleanly
        g_writes.clear(); g_write_rc = 0;
        assert(R_SUCCEEDED(ctrl_busy.SendConnectRumble(0, 0)));
        assert(g_writes.size() == 1 && g_writes[0].payload == std::vector<u8>({0, 0, 0, 0}));
    }

    // 5. Real-world v15/v19 HOS pairing rumble trace (6ms start->stop collision), dynamic MinPulse (122ms for 0.5572), 0x2a671 retry, 200ms watchdog, and 33Hz held-state heartbeat
    {
        g_writes.clear(); g_write_rc = 0;
        controller::g_connected_controllers = 1;
        ble::events::g_radio_enabled = true;
        ble::events::g_wake_calls = 0;
        os::g_now_ms = 10000;

        StadiaBleController ctrl_rt(addr, 6, svc_id, true, out_id, true, true, BtdrvGattAuthReqType_None);

        // 5a. Connect pulse: t=0 start (low_band_amp ~0.5572 -> ~122ms pulse) -> t=6ms STOP arrives while in-flight!
        SwitchMotorData motor_on{};
        motor_on.left_motor.low_band_amp = 0.5572f;
        SwitchMotorData motor_off{};
        assert(R_SUCCEEDED(ctrl_rt.TestSetVibration(&motor_on)));
        assert(g_writes.size() == 1);

        os::g_now_ms += 6; // t=10006: HOS cmd#15 stop arrives while ATT #1 is still in-flight!
        assert(R_SUCCEEDED(ctrl_rt.TestSetVibration(&motor_off)));
        assert(g_writes.size() == 1); // Did NOT collide with 0x2a671!

        os::g_now_ms += 9; // t=10015: ATT ack #1 arrives, but < 122ms pulse so motor stays running briefly
        ctrl_rt.OnRumbleWriteComplete(0);
        ctrl_rt.ServiceBleTasks();
        assert(g_writes.size() == 1);

        os::g_now_ms += 125; // t=10140 (< 162ms): still running
        ctrl_rt.ServiceBleTasks();
        assert(g_writes.size() == 1);

        os::g_now_ms += 22; // t=10162 (== 162ms): intensity-scaled pulse elapsed -> ServiceBleTasks() delivers STOP!
        ctrl_rt.ServiceBleTasks();
        assert(g_writes.size() == 2);
        assert(g_writes[1].payload == std::vector<u8>({0, 0, 0, 0}));
        ctrl_rt.OnRumbleWriteComplete(0);

        // 5b. Pending STOP retries automatically across transient 0x2a671 (GATT BUSY)
        g_writes.clear();
        os::g_now_ms = 12000;
        assert(R_SUCCEEDED(ctrl_rt.TestSetVibration(&motor_on)));
        assert(g_writes.size() == 1);
        ctrl_rt.OnRumbleWriteComplete(0);
        os::g_now_ms += 170; // past 162ms pulse
        g_write_rc = 0x2a671;
        assert(ctrl_rt.TestSetVibration(&motor_off).value == 0x2a671);
        assert(g_writes.size() == 2);
        // Still busy on first retry
        os::g_now_ms += 20;
        ctrl_rt.ServiceBleTasks();
        assert(g_writes.size() == 3);
        // Succeeds on next ServiceBleTasks() tick once link clears!
        g_write_rc = 0;
        os::g_now_ms += 20;
        ctrl_rt.ServiceBleTasks();
        assert(g_writes.size() == 4);
        assert(g_writes[3].payload == std::vector<u8>({0, 0, 0, 0}));
        ctrl_rt.OnRumbleWriteComplete(0);

        // 5c. Watchdog auto-stop at 200ms if HOS never sends a follow-up stop packet (e.g. Session 1 c2180372)
        g_writes.clear();
        os::g_now_ms = 14000;
        assert(R_SUCCEEDED(ctrl_rt.TestSetVibration(&motor_on)));
        assert(g_writes.size() == 1);
        ctrl_rt.OnRumbleWriteComplete(0);
        os::g_now_ms += 199;
        ctrl_rt.ServiceBleTasks();
        assert(g_writes.size() == 1); // Not expired yet
        os::g_now_ms += 1; // t=14200
        ctrl_rt.ServiceBleTasks();
        assert(g_writes.size() == 2 && g_writes[1].payload == std::vector<u8>({0, 0, 0, 0}));
        ctrl_rt.OnRumbleWriteComplete(0);

        // 5d. Continuous stream (every 60ms) extends watchdog without premature cutoff, refreshes at 1500ms, then stops cleanly 200ms after last frame
        g_writes.clear();
        os::g_now_ms = 16000;
        for (int step = 0; step < 28; ++step) {
            const size_t before = g_writes.size();
            assert(R_SUCCEEDED(ctrl_rt.TestSetVibration(&motor_on)));
            if (g_writes.size() > before) {
                // Only non-zero motor_on refresh writes (every >=1500ms) may occur, never a stop!
                assert(g_writes.back().payload != std::vector<u8>({0, 0, 0, 0}));
                ctrl_rt.OnRumbleWriteComplete(0);
            }
            os::g_now_ms += 60;
            ctrl_rt.ServiceBleTasks();
        }
        assert(g_writes.size() == 2); // Initial write at t=16000 + one 1500ms keepalive refresh at t=17500, zero premature stops!
        os::g_now_ms += 140; // 200ms since last motor_on (step=27 was at 17620 -> 17680+140=17820)
        ctrl_rt.ServiceBleTasks();
        assert(g_writes.size() == 3 && g_writes[2].payload == std::vector<u8>({0, 0, 0, 0}));
        ctrl_rt.OnRumbleWriteComplete(0);

        // 5e. Held-state 33Hz input keepalive preserves pressed button & stick when idle, and stays silent during 15ms live input
        g_writes.clear();
        os::g_now_ms = 20000;
        const u8 hold_a_pkt[10] = {0x08, 0x00, 0x20, 0x10, 0xF0, 0x80, 0x80, 0x00, 0x00, 0x00}; // A button + pushed left stick
        assert(R_SUCCEEDED(ctrl_rt.HandleBleInputReport(hold_a_pkt, sizeof(hold_a_pkt))));
        ctrl_rt.TestResetInjectCalls();

        // Live input every 15ms -> 0 extra keepalives
        os::g_now_ms += 15;
        assert(R_SUCCEEDED(ctrl_rt.HandleBleInputReport(hold_a_pkt, sizeof(hold_a_pkt))));
        ctrl_rt.ServiceBleTasks();
        assert(ctrl_rt.TestInjectCalls() == 0);

        os::g_now_ms += 15;
        assert(R_SUCCEEDED(ctrl_rt.HandleBleInputReport(hold_a_pkt, sizeof(hold_a_pkt))));
        ctrl_rt.ServiceBleTasks();
        assert(ctrl_rt.TestInjectCalls() == 0);

        // Idle for 30ms -> keepalive fires WITH held A button & non-centered stick preserved!
        os::g_now_ms += 30;
        ctrl_rt.ServiceBleTasks();
        assert(ctrl_rt.TestInjectCalls() == 1);
        assert(ctrl_rt.TestLastInjectedButtons() == stadia::ButtonBit::A);
        assert(ctrl_rt.TestLastInjectedLeftX() == ctrl_rt.TestLeftX());
        assert(ctrl_rt.TestLastInjectedLeftY() == ctrl_rt.TestLeftY());
    }

    // 6. v18 Real-world scenarios: Find-Controllers max continuous hold, Mario Wonder micro-buzz gating,
    //    45ms crisp transient kick, and Ace Attorney heavy boost + echo left-motor cutoff
    {
        g_writes.clear(); g_write_rc = 0;
        controller::g_connected_controllers = 1;
        ble::events::g_radio_enabled = true;
        os::g_now_ms = 30000;
        StadiaBleController ctrl_v18(addr, 7, svc_id, true, out_id, true, true, BtdrvGattAuthReqType_None);

        // 6a. Find-Controllers: 04bf4040_04bf4040 -> continuous 0xFFFF/0xFFFF across 3s of PacketType=0, instant CancelVibration()
        const SwitchEncodedMotorData find_enc = {{0x04, 0xbf, 0x40, 0x40, 0x04, 0xbf, 0x40, 0x40}};
        SwitchMotorData find_val{};
        find_val.left_motor.high_band_amp = 0.5f;
        find_val.right_motor.high_band_amp = 0.5f;
        assert(R_SUCCEEDED(ctrl_v18.TestOnVibrationPacket(&find_enc, &find_val, &find_val, true)));
        assert(g_writes.size() == 1);
        assert(g_writes[0].payload == std::vector<u8>({0xFF, 0xFF, 0xFF, 0xFF}));
        ctrl_v18.OnRumbleWriteComplete(0);

        const SwitchEncodedMotorData keepalive_enc = {{0, 0, 0, 0, 0, 0, 0, 0}};
        for (int step = 1; step <= 200; ++step) { // 200 * 15ms = 3000ms continuous
            os::g_now_ms += 15;
            const size_t before = g_writes.size();
            assert(R_SUCCEEDED(ctrl_v18.TestOnVibrationPacket(&keepalive_enc, &find_val, &find_val, false)));
            if (g_writes.size() > before) {
                assert(g_writes.back().payload == std::vector<u8>({0xFF, 0xFF, 0xFF, 0xFF}));
                ctrl_v18.OnRumbleWriteComplete(0);
            }
            ctrl_v18.ServiceBleTasks();
        }
        assert(g_writes.size() == 3); // t=0 + t=1500ms + t=3000ms watchdog refresh, zero drops!

        assert(R_SUCCEEDED(ctrl_v18.TestCancelVibration()));
        assert(g_writes.size() == 4);
        assert(g_writes[3].payload == std::vector<u8>({0x00, 0x00, 0x00, 0x00}));
        ctrl_v18.OnRumbleWriteComplete(0);

        // 6b. Mario Wonder micro-buzz (0.85% low, 2.63% high) -> completely gated (0 BLE writes)
        g_writes.clear();
        os::g_now_ms = 40000;
        const SwitchEncodedMotorData game_enc = {{0x01, 0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0x40}};
        SwitchMotorData mario_buzz{};
        mario_buzz.left_motor.low_band_amp = 0.0085f;
        mario_buzz.right_motor.high_band_amp = 0.0263f;
        assert(R_SUCCEEDED(ctrl_v18.TestOnVibrationPacket(&game_enc, &mario_buzz, &mario_buzz, true)));
        assert(g_writes.empty());

        // 6c. Intensity-scaled transient kicks: light (~88ms) vs strong connect (~122ms)
        g_writes.clear();
        os::g_now_ms = 42000;
        SwitchMotorData light_peak{};
        light_peak.right_motor.high_band_amp = 0.12f;
        SwitchMotorData silent{};
        assert(R_SUCCEEDED(ctrl_v18.TestOnVibrationPacket(&game_enc, &light_peak, &silent, true)));
        assert(g_writes.size() == 1);
        assert(g_writes[0].payload[0] == 0 && g_writes[0].payload[1] == 0);
        assert(g_writes[0].payload[2] != 0 || g_writes[0].payload[3] != 0);
        ctrl_v18.OnRumbleWriteComplete(0);

        os::g_now_ms += 115; // < ~134ms
        assert(R_SUCCEEDED(ctrl_v18.TestOnVibrationPacket(&keepalive_enc, &silent, &silent, false)));
        assert(g_writes.size() == 1);

        os::g_now_ms += 25; // 140ms >= 134ms
        assert(R_SUCCEEDED(ctrl_v18.TestOnVibrationPacket(&keepalive_enc, &silent, &silent, false)));
        assert(g_writes.size() == 2);
        assert(g_writes[1].payload == std::vector<u8>({0x00, 0x00, 0x00, 0x00}));
        ctrl_v18.OnRumbleWriteComplete(0);

        g_writes.clear();
        os::g_now_ms = 44000;
        SwitchMotorData kick_peak{};
        kick_peak.left_motor.low_band_amp = 0.5572f;
        assert(R_SUCCEEDED(ctrl_v18.TestOnVibrationPacket(&game_enc, &kick_peak, &silent, true)));
        assert(g_writes.size() == 1);
        assert(g_writes[0].payload[0] != 0 || g_writes[0].payload[1] != 0);
        ctrl_v18.OnRumbleWriteComplete(0);

        os::g_now_ms += 145; // < 162ms (stronger kick lasts longer than light tap!)
        assert(R_SUCCEEDED(ctrl_v18.TestOnVibrationPacket(&keepalive_enc, &silent, &silent, false)));
        assert(g_writes.size() == 1);

        os::g_now_ms += 17; // == 162ms
        assert(R_SUCCEEDED(ctrl_v18.TestOnVibrationPacket(&keepalive_enc, &silent, &silent, false)));
        assert(g_writes.size() == 2);
        assert(g_writes[1].payload == std::vector<u8>({0x00, 0x00, 0x00, 0x00}));
        ctrl_v18.OnRumbleWriteComplete(0);

        // 6d. Ace Attorney heavy burst (lo=0.354, hi=0.511) -> dual heavy drive, then echo (lo=0.018, hi=0.10) cuts left ERM immediately
        g_writes.clear();
        os::g_now_ms = 50000;
        SwitchMotorData objection_data{};
        objection_data.left_motor.low_band_amp = 0.354f;
        objection_data.left_motor.high_band_amp = 0.511f;
        objection_data.right_motor = objection_data.left_motor;
        assert(R_SUCCEEDED(ctrl_v18.TestOnVibrationPacket(&game_enc, &objection_data, &objection_data, true)));
        assert(g_writes.size() == 1);
        const u16 strong_val = static_cast<u16>(g_writes[0].payload[0] | (g_writes[0].payload[1] << 8));
        const u16 weak_val   = static_cast<u16>(g_writes[0].payload[2] | (g_writes[0].payload[3] << 8));
        assert(strong_val >= 0xA800 && weak_val >= 0x8800);
        ctrl_v18.OnRumbleWriteComplete(0);

        os::g_now_ms += 165;
        SwitchMotorData echo_data{};
        echo_data.left_motor.low_band_amp = 0.018f;
        echo_data.left_motor.high_band_amp = 0.10f;
        echo_data.right_motor = echo_data.left_motor;
        assert(R_SUCCEEDED(ctrl_v18.TestOnVibrationPacket(&game_enc, &echo_data, &echo_data, true)));
        assert(g_writes.size() == 2);
        assert(g_writes[1].payload[0] == 0x00 && g_writes[1].payload[1] == 0x00); // Heavy left ERM off immediately!
        assert(g_writes[1].payload[2] != 0x00 || g_writes[1].payload[3] != 0x00); // Light right ERM carries faint echo cleanly
        ctrl_v18.OnRumbleWriteComplete(0);
    }

    std::cout << "StadiaBleController production framing/rumble/watchdog/keepalive regressions passed\n";
}
