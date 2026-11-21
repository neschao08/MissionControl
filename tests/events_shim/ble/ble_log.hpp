#pragma once
namespace ams::ble::log {
    inline bool g_enabled = false;
    inline bool IsEnabled() { return g_enabled; }
    template<typename... Args> inline void Write(const char *, Args...) {}
}
