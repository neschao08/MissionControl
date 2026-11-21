#pragma once
namespace ams::ble::log {
    inline bool IsEnabled() { return false; }
    inline void Write(const char *, ...) {}
}
