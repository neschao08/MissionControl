// Link production switch_hd_rumble_decoder.cpp & switch_vibration_processor.cpp
// unchanged to verify multi-sample peak decoding on captured HOS 22.5.0 rumble packets.
#include "switch_vibration_processor.hpp"
#include "emulated_switch_controller.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace ams::controller;

static void PackLe32(SwitchEncodedVibrationSamples *out, u32 v) {
    u8 *p = reinterpret_cast<u8 *>(out);
    p[0] = static_cast<u8>(v & 0xFFu);
    p[1] = static_cast<u8>((v >> 8) & 0xFFu);
    p[2] = static_cast<u8>((v >> 16) & 0xFFu);
    p[3] = static_cast<u8>((v >> 24) & 0xFFu);
}

int main() {
    static_assert(sizeof(SwitchEncodedMotorData) == 8);

    EmulatedSwitchController ctrl{};
    SwitchVibrationProcessor proc(&ctrl);

    // 1. Real HOS 22.5.0 connect rumble packet: m=c2180372_00000000 (sample[0] lo_amp ~ 0.5572, sample[1..2] = 0)
    {
        SwitchEncodedMotorData pkt{};
        PackLe32(&pkt.left_motor,  0x720318c2u); // bytes: c2 18 03 72
        PackLe32(&pkt.right_motor, 0x00000000u);
        proc.ProcessMotorData(&pkt);
        assert(ctrl.vibration_calls == 1);
        assert(ctrl.last_samples_available);
        assert(ctrl.last_motor.left_motor.low_band_amp > 0.55f);
        assert(ctrl.last_motor.left_motor.low_band_amp < 0.57f);
        assert(ctrl.last_motor.right_motor.low_band_amp == 0.0f);
        // Post-packet sustained tail dropped back to 0!
        assert(ctrl.last_sustained.left_motor.low_band_amp == 0.0f);
        assert(ctrl.last_sustained.left_motor.high_band_amp == 0.0f);
    }

    // 2. Real HOS 22.5.0 short pulse packet: m=00000000_02786040
    //    sample[0] = 0.0, sample[1] high_band_amp = 0.5000, sample[2] = 0.0
    //    Peak decoding MUST extract high_band_amp == 0.5f while sustained reflects silent sample[2]!
    {
        SwitchEncodedMotorData pkt{};
        PackLe32(&pkt.left_motor,  0x00000000u);
        PackLe32(&pkt.right_motor, 0x40607802u); // bytes: 02 78 60 40
        proc.ProcessMotorData(&pkt);
        assert(ctrl.vibration_calls == 2);
        assert(ctrl.last_samples_available);
        assert(std::fabs(ctrl.last_motor.right_motor.high_band_amp - 0.5f) < 0.001f);
        assert(ctrl.last_motor.right_motor.low_band_amp == 0.0f);
        assert(ctrl.last_sustained.right_motor.high_band_amp == 0.0f);
    }

    // 3. Neutral stop packet: m=00014040_00014040 -> all zero amplitudes
    {
        SwitchEncodedMotorData pkt{};
        PackLe32(&pkt.left_motor,  0x40400100u); // bytes: 00 01 40 40
        PackLe32(&pkt.right_motor, 0x40400100u);
        proc.ProcessMotorData(&pkt);
        assert(ctrl.vibration_calls == 3);
        assert(ctrl.last_motor.left_motor.low_band_amp == 0.0f);
        assert(ctrl.last_motor.left_motor.high_band_amp == 0.0f);
        assert(ctrl.last_motor.right_motor.low_band_amp == 0.0f);
        assert(ctrl.last_motor.right_motor.high_band_amp == 0.0f);
    }

    // 4. Find-Controllers packet m=04bf4040_04bf4040 followed by PacketType=0 keepalive frames (00000000_00000000)
    //    Sustained high_band_amp MUST remain 0.5f during PacketType=0 until neutral/Reset() clears it!
    {
        SwitchEncodedMotorData find_pkt{};
        PackLe32(&find_pkt.left_motor,  0x4040bf04u); // bytes: 04 bf 40 40
        PackLe32(&find_pkt.right_motor, 0x4040bf04u);
        proc.ProcessMotorData(&find_pkt);
        assert(ctrl.last_samples_available);
        assert(std::fabs(ctrl.last_sustained.left_motor.high_band_amp - 0.5f) < 0.001f);
        assert(std::fabs(ctrl.last_sustained.right_motor.high_band_amp - 0.5f) < 0.001f);

        SwitchEncodedMotorData keepalive_pkt{};
        proc.ProcessMotorData(&keepalive_pkt);
        assert(!ctrl.last_samples_available);
        assert(std::fabs(ctrl.last_sustained.left_motor.high_band_amp - 0.5f) < 0.001f);
        assert(std::fabs(ctrl.last_sustained.right_motor.high_band_amp - 0.5f) < 0.001f);

        proc.Reset();
        proc.ProcessMotorData(&keepalive_pkt);
        assert(!ctrl.last_samples_available);
        assert(ctrl.last_sustained.left_motor.high_band_amp == 0.0f);
        assert(ctrl.last_sustained.right_motor.high_band_amp == 0.0f);
    }

    std::cout << "SwitchVibrationProcessor peak + sustained decoding regressions passed\n";
}
