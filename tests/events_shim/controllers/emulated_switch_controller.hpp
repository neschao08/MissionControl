#pragma once
#include "switch.h"
namespace ams::controller {
    class EmulatedSwitchController {
        public:
            virtual ~EmulatedSwitchController() = default;
            virtual Result HandleBleInputReport(const u8 *data, size_t size) = 0;
            virtual void OnRumbleWriteComplete(u32 result) { (void)result; }
    };
}
