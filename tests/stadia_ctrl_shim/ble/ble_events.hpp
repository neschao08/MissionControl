#pragma once
#include <cstdint>
namespace ams::ble::events {
    inline bool g_radio_enabled = true;
    inline uint32_t g_pending_rumble_error = 0;
    inline bool IsRadioEnabled() { return g_radio_enabled; }
    inline bool TakeRumbleWriteError(uint32_t, uint32_t *out_result) {
        if (g_pending_rumble_error == 0) return false;
        if (out_result) *out_result = g_pending_rumble_error;
        g_pending_rumble_error = 0;
        return true;
    }
}
