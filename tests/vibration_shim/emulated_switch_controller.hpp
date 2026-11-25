#pragma once
#include "switch_hd_rumble_types.hpp"

namespace ams::controller {

    struct EmulatedSwitchController {
        SwitchMotorData last_motor{};
        SwitchMotorData last_sustained{};
        u32 vibration_calls = 0;
        u32 packet_calls = 0;
        bool last_samples_available = false;

        Result SetVibration(const SwitchMotorData *motor_data) {
            last_motor = *motor_data;
            ++vibration_calls;
            return {0};
        }

        Result OnVibrationPacket(const SwitchEncodedMotorData *, const SwitchMotorData *peak_data, const SwitchMotorData *sustained_data, bool samples_available) {
            last_sustained = *sustained_data;
            last_samples_available = samples_available;
            ++packet_calls;
            if (samples_available) {
                return this->SetVibration(peak_data);
            }
            return {0};
        }
    };

}
