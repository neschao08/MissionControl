#pragma once
#include <cstddef>
namespace ams::controller {
    inline size_t g_connected_controllers = 1;
    inline size_t GetConnectedControllerCount() { return g_connected_controllers; }
}
