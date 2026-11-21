#pragma once
namespace ams::ble::log {
    inline bool IsEnabled() { return false; }
    template<typename... Args> inline void Write(const char *, Args...) {}
}
