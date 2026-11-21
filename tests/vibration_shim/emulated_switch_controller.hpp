#pragma once
#include "switch_hd_rumble_types.hpp"

namespace ams::controller {

    struct EmulatedSwitchController {
        SwitchMotorData last_motor{};
        u32 vibration_calls = 0;

        Result SetVibration(const SwitchMotorData *motor_data) {
            last_motor = *motor_data;
            ++vibration_calls;
            return {0};
        }
    };

}
