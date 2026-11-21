#pragma once
namespace ams::mitm {
    struct GlobalConfig {
        struct { bool low_latency = true; } ble;
    };
    inline GlobalConfig g_cfg;
    inline GlobalConfig *GetGlobalConfig() { return &g_cfg; }
}
