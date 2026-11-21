// Path 13: Exhaustive boundary + 100,000-packet fuzzing + multi-threaded concurrency stress test
// against production stadia_ble_controller.cpp & stadia_mapping.hpp.
#include "stadia_ble_controller.cpp"
#include <atomic>
#include <cassert>
#include <iostream>
#include <memory>
#include <random>
#include <thread>
#include <vector>

using namespace ams;
using namespace ams::controller;

static std::atomic<u64> g_gatt_write_calls{0};
Result btdrvWriteGattCharacteristic(u32, bool, const BtdrvGattId *, const BtdrvGattId *, const void *, size_t size, BtdrvGattAuthReqType, bool) {
    assert(size == 4);
    g_gatt_write_calls.fetch_add(1, std::memory_order_relaxed);
    return {0};
}

int main() {
    // 1. Exhaustive single-dimension checks:
    //    - All 256 possible hat byte values (0x00..0xFF): HID Hat Switch uses low 4 bits (dpad & 0x0F);
    //      0..7 map to valid 8-way D-pad directions and 8..15 are neutral regardless of upper 4 padding bits.
    for (int hat = 0; hat <= 255; ++hat) {
        stadia::StadiaInputReport0x03 r{static_cast<u8>(hat), 0, 0, 0x80, 0x80, 0x80, 0x80, 0, 0, 0};
        const auto m = stadia::MapInputReport0x03(r, 0.5f);
        if ((hat & 0x0F) > 7) {
            assert(m.buttons_mask == 0);
        } else {
            assert(m.buttons_mask != 0);
        }
    }

    //    - All 256 possible stick axis values (0x00..0xFF): strictly in [0, 4095], monotonic!
    u16 prev_x = 0, prev_y = 4095;
    for (int v = 0; v <= 255; ++v) {
        stadia::StadiaInputReport0x03 r{0x08, 0, 0, static_cast<u8>(v), static_cast<u8>(v), static_cast<u8>(v), static_cast<u8>(v), 0, 0, 0};
        const auto m = stadia::MapInputReport0x03(r, 0.5f);
        assert(m.left_x <= 4095 && m.left_y <= 4095 && m.right_x <= 4095 && m.right_y <= 4095);
        assert(m.left_x == m.right_x && m.left_y == m.right_y);
        assert(m.left_x >= prev_x);
        assert(m.left_y <= prev_y); // Y axis inverted for Switch
        prev_x = m.left_x;
        prev_y = m.left_y;
    }

    //    - All 256 possible trigger values at thresholds 0%, 25%, 50%, 75%, 100%
    for (float th : {0.0f, 0.25f, 0.50f, 0.75f, 1.0f}) {
        for (int t = 0; t <= 255; ++t) {
            // Set digital early-trigger bits 0x04/0x08 to verify they are ALWAYS ignored
            stadia::StadiaInputReport0x03 r{0x08, 0x0C, 0x00, 0x80, 0x80, 0x80, 0x80, static_cast<u8>(t), static_cast<u8>(t), 0};
            const auto m = stadia::MapInputReport0x03(r, th);
            const bool expect = (static_cast<float>(t) / 255.0f) > th;
            assert(((m.buttons_mask & stadia::ButtonBit::ZL) != 0) == expect);
            assert(((m.buttons_mask & stadia::ButtonBit::ZR) != 0) == expect);
        }
    }

    // 2. 100,000-packet pseudo-random fuzzing:
    //    - Verify Stadia center button bit (buttons0 & 0x10) NEVER changes buttons_mask under ANY combination!
    //    - Verify sticks always stay within [0, 4095] and only valid Switch buttons are ever set!
    constexpr u32 AllValidMask =
        stadia::ButtonBit::Y | stadia::ButtonBit::X | stadia::ButtonBit::B | stadia::ButtonBit::A |
        stadia::ButtonBit::R | stadia::ButtonBit::ZR | stadia::ButtonBit::Minus | stadia::ButtonBit::Plus |
        stadia::ButtonBit::StickR | stadia::ButtonBit::StickL | stadia::ButtonBit::Home | stadia::ButtonBit::Capture |
        stadia::ButtonBit::Down | stadia::ButtonBit::Up | stadia::ButtonBit::Right | stadia::ButtonBit::Left |
        stadia::ButtonBit::L | stadia::ButtonBit::ZL;

    std::mt19937 rng(0x57AD1A22u);
    for (int i = 0; i < 100000; ++i) {
        stadia::StadiaInputReport0x03 base{};
        base.dpad = static_cast<u8>(rng() & 0xFF);
        base.buttons0 = static_cast<u8>((rng() & 0xFF) & ~0x10); // Stadia bit cleared
        base.buttons1 = static_cast<u8>(rng() & 0xFF);
        base.left_stick_x = static_cast<u8>(rng() & 0xFF);
        base.left_stick_y = static_cast<u8>(rng() & 0xFF);
        base.right_stick_x = static_cast<u8>(rng() & 0xFF);
        base.right_stick_y = static_cast<u8>(rng() & 0xFF);
        base.left_trigger = static_cast<u8>(rng() & 0xFF);
        base.right_trigger = static_cast<u8>(rng() & 0xFF);
        base.consumer = static_cast<u8>(rng() & 0xFF);

        auto with_stadia = base;
        with_stadia.buttons0 |= 0x10; // Press center Stadia button

        const auto m0 = stadia::MapInputReport0x03(base, 0.5f);
        const auto m1 = stadia::MapInputReport0x03(with_stadia, 0.5f);
        assert(m0.buttons_mask == m1.buttons_mask); // Center Stadia key MUST remain 100% unmapped!
        assert((m0.buttons_mask & ~AllValidMask) == 0);
        assert(m0.left_x <= 4095 && m0.left_y <= 4095 && m0.right_x <= 4095 && m0.right_y <= 4095);
    }

    // 3. Multi-threaded concurrency stress: simultaneous input packets, rumble output, and DisconnectBle
    const bluetooth::Address addr = {{0xcb, 0x4e, 0xc0, 0x96, 0x49, 0x67}};
    const BtdrvGattId svc_id = {{0x1812}, 0};
    const BtdrvGattId out_id = {{0x2A4D}, 1};
    ble::events::g_radio_enabled = true;
    controller::g_connected_controllers = 1;

    for (int round = 0; round < 20; ++round) {
        auto ctrl = std::make_shared<StadiaBleController>(addr, 259 + round, svc_id, true, out_id, true, true, BtdrvGattAuthReqType_None);
        std::atomic<bool> stop{false};

        std::thread input_t([&] {
            u8 pkt[10] = {0x08, 0x00, 0x20, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00};
            for (int n = 0; n < 2000 && !stop.load(std::memory_order_relaxed); ++n) {
                pkt[2] = static_cast<u8>((n & 1) ? 0x20 : 0x40);
                static_cast<void>(ctrl->HandleBleInputReport(pkt, sizeof(pkt)));
            }
        });
        std::thread output_t([&] {
            u8 out_basic[2] = {0x10, 0x00};
            u8 out_cmd[2] = {0x01, 0x00};
            for (int n = 0; n < 2000 && !stop.load(std::memory_order_relaxed); ++n) {
                static_cast<void>(ctrl->HandleOutputDataReport((n & 1) ? out_basic : out_cmd, 2));
                SwitchMotorData m{};
                m.left_motor.low_band_amp = (n & 1) ? 0.5f : 0.0f;
                static_cast<void>(ctrl->TestSetVibration(&m));
            }
        });

        std::this_thread::yield();
        ctrl->DisconnectBle();
        stop.store(true, std::memory_order_relaxed);
        input_t.join();
        output_t.join();

        // After DisconnectBle, any further calls must fail with InvalidState
        u8 pkt[10] = {0x08, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00};
        assert(ctrl->HandleBleInputReport(pkt, sizeof(pkt)).value == 11);
    }

    std::cout << "Stadia exhaustive boundary + 100,000-packet fuzz + 20-round MT concurrency stress passed\n";
}
