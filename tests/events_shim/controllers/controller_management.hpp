#pragma once
#include "switch.h"
namespace ams::controller {
    inline size_t g_connected_count = 0;
    inline size_t GetConnectedControllerCount() { return g_connected_count; }
}
