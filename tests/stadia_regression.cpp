#include "../mc_mitm/source/controllers/stadia_mapping.hpp"
#include <cassert>
#include <iostream>

using namespace ams::controller::stadia;

static void AssertMapped(const StadiaInputReport0x03 &r, uint32_t expected_mask, float threshold = 0.5f) {
    const auto m = MapInputReport0x03(r, threshold);
    assert(m.buttons_mask == expected_mask);
}

int main() {
    // Neutral / idle packet captured from real StadiaQPRV-4967 over BLE HOGP (10 bytes after stripping report ID 0x03)
    const StadiaInputReport0x03 idle{0x08, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00};
    {
        const auto m = MapInputReport0x03(idle, 0.5f);
        assert(m.buttons_mask == 0);
        assert(m.left_x == 2048 && m.left_y == 2048);
        assert(m.right_x == 2048 && m.right_y == 2048);
    }

    // Positional face buttons from captured packets:
    // East (printed B) -> A
    AssertMapped({0x08, 0x00, 0x20, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::A);
    // South (printed A) -> B
    AssertMapped({0x08, 0x00, 0x40, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::B);
    // West (printed X) -> Y
    AssertMapped({0x08, 0x00, 0x10, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::Y);
    // North (printed Y) -> X
    AssertMapped({0x08, 0x00, 0x08, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::X);

    // First center row: right (Menu) -> +, left (Options) -> -
    AssertMapped({0x08, 0x20, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::Plus);
    AssertMapped({0x08, 0x40, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::Minus);

    // Second center row: right (Capture icon) -> Home, left (Assistant) -> Capture
    AssertMapped({0x08, 0x01, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::Home);
    AssertMapped({0x08, 0x02, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::Capture);

    // Center Stadia button must remain unmapped (empty)
    AssertMapped({0x08, 0x10, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, 0);

    // Shoulders and stick clicks from captured packets
    AssertMapped({0x08, 0x00, 0x04, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::L);
    AssertMapped({0x08, 0x00, 0x02, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::R);
    AssertMapped({0x08, 0x00, 0x01, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::StickL);
    AssertMapped({0x08, 0x80, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::StickR);

    // Analog triggers with 50% threshold (ignore digital trigger bits 0x04/0x08 even when set early)
    AssertMapped({0x08, 0x04, 0x00, 0x80, 0x80, 0x80, 0x80, 0x55, 0x00, 0x00}, 0);
    AssertMapped({0x08, 0x04, 0x00, 0x80, 0x80, 0x80, 0x80, 0x7F, 0x00, 0x00}, 0);
    AssertMapped({0x08, 0x04, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00}, ButtonBit::ZL);
    AssertMapped({0x08, 0x04, 0x00, 0x80, 0x80, 0x80, 0x80, 0xEB, 0x00, 0x00}, ButtonBit::ZL);
    AssertMapped({0x08, 0x08, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x4F, 0x00}, 0);
    AssertMapped({0x08, 0x08, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x7F, 0x00}, 0);
    AssertMapped({0x08, 0x08, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x80, 0x00}, ButtonBit::ZR);
    AssertMapped({0x08, 0x08, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0xFF, 0x00}, ButtonBit::ZR);

    // 8-way D-Pad + neutral
    AssertMapped({0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::Up);
    AssertMapped({0x01, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::Up | ButtonBit::Right);
    AssertMapped({0x02, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::Right);
    AssertMapped({0x03, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::Down | ButtonBit::Right);
    AssertMapped({0x04, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::Down);
    AssertMapped({0x05, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::Down | ButtonBit::Left);
    AssertMapped({0x06, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::Left);
    AssertMapped({0x07, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00}, ButtonBit::Up | ButtonBit::Left);

    // Sticks: captured up/right extremes and out-of-range 0x00 clamping
    {
        const auto m = MapInputReport0x03({0x08, 0x00, 0x00, 0x01, 0x01, 0xFF, 0xFF, 0x00, 0x00, 0x00}, 0.5f);
        assert(m.left_x == 0 && m.left_y == 4095);
        assert(m.right_x == 4095 && m.right_y == 0);

        const auto clamped = MapInputReport0x03({0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0.5f);
        assert(clamped.left_x == 0 && clamped.left_y == 4095);
        assert(clamped.right_x == 0 && clamped.right_y == 4095);
    }

    // v21 Dual-motor rumble mapping (1.8% / 12% noise gates, 2x/1.5x gain, sqrt light curve, and 110..160ms pulses)
    {
        assert(MapRumbleMagnitude(0.0f, true) == 0);
        assert(MapRumbleMagnitude(0.0f, false) == 0);
        assert(MapRumbleMagnitude(-0.5f, true) == 0);

        // Mario Wonder tiny footstep/ambient vibrations (0.85%, 1.56% < 1.8%) must stay completely silent,
        // while real light game taps (2.63%, 5%) drive the nimble right ERM clearly without waking heavy left ERM.
        assert(MapRumbleMagnitude(0.0085f, false) == 0);
        assert(MapRumbleMagnitude(0.0156f, false) == 0);
        assert(MapRumbleMagnitude(0.0263f, false) >= 0x5500);
        assert(MapRumbleMagnitude(0.0500f, false) >= 0x7000);
        assert(MapRumbleMagnitude(0.1000f, true) == 0);

        const uint16_t s_low  = MapRumbleMagnitude(0.15f, true);
        const uint16_t s_mid  = MapRumbleMagnitude(0.40f, true);
        const uint16_t s_full = MapRumbleMagnitude(0.65f, true);
        const uint16_t w_low  = MapRumbleMagnitude(0.05f, false);
        const uint16_t w_mid  = MapRumbleMagnitude(0.25f, false);
        const uint16_t w_full = MapRumbleMagnitude(0.50f, false);

        assert(s_low > 0 && s_low < s_mid && s_mid < s_full && s_full == 0xFFFF);
        assert(w_low > 0 && w_low < w_mid && w_mid < w_full && w_full == 0xFFFF);
        assert(MapRumbleMagnitude(1.5f, true) == 0xFFFF);
        assert(MapRumbleMagnitude(1.5f, false) == 0xFFFF);

        // Light low-band tap (4% @ 160Hz) and high-freq low-band rumble (25% @ 250Hz) route cleanly to right ERM at 1.0x
        const auto light_tap = MapSwitchRumbleToStadia(0.04f, 160.0f, 0.0f, 320.0f, 0.0f, 160.0f, 0.0f, 320.0f);
        assert(light_tap.strong == 0 && light_tap.weak >= 0x6800);
        const auto hi_freq_low = MapSwitchRumbleToStadia(0.25f, 250.0f, 0.0f, 320.0f, 0.0f, 160.0f, 0.0f, 320.0f);
        assert(hi_freq_low.strong == 0 && hi_freq_low.weak > 0);

        // Ace Attorney Objection heavy composite burst boosts strong motor, then echo tail (1.8% low, 8% high) cuts left ERM off immediately
        const auto objection = MapSwitchRumbleToStadia(0.354f, 160.0f, 0.511f, 320.0f, 0.354f, 160.0f, 0.511f, 320.0f);
        assert(objection.strong > MapRumbleMagnitude(0.354f, true));
        assert(objection.strong >= 0xE000 && objection.weak == 0xFFFF);
        const auto echo_tail = MapSwitchRumbleToStadia(0.018f, 160.0f, 0.080f, 320.0f, 0.018f, 160.0f, 0.080f, 320.0f);
        assert(echo_tail.strong == 0 && echo_tail.weak > 0);

        // Find-Controllers packet signature check
        const uint8_t find_pkt[8] = {0x04, 0xbf, 0x40, 0x40, 0x04, 0xbf, 0x40, 0x40};
        const uint8_t norm_pkt[8] = {0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40};
        assert(IsFindControllerMotorBytes(find_pkt));
        assert(!IsFindControllerMotorBytes(norm_pkt));

        // Intensity-scaled transient pulse duration (110..160ms including 25ms ERM spin-up overhead)
        assert(ComputeTransientPulseMs(0, 0) == 0);
        const int64_t p_light_w = ComputeTransientPulseMs(0, 0x0800);
        const int64_t p_light_s = ComputeTransientPulseMs(0x0800, 0);
        const int64_t p_conn    = ComputeTransientPulseMs(0xE5A7, 0);
        const int64_t p_max     = ComputeTransientPulseMs(0xFFFF, 0xFFFF);
        assert(p_light_w >= 110 && p_light_w <= 115);
        assert(p_light_s >= 120 && p_light_s <= 123);
        assert(p_conn >= 154 && p_conn <= 158);
        assert(p_max == 160);
        assert(p_light_w < p_light_s && p_light_s < p_conn && p_conn < p_max);
    }

    std::cout << "Stadia BLE input report mapping regressions passed\n";
}
