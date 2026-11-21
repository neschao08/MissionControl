#pragma once
#include "ble_types.hpp"
namespace ams::ble {
    struct ResultNoConnection : Result {
        ResultNoConnection() : Result{4} {}
        static bool Includes(Result r) { return r.value == 4; }
    };
    struct ResultInsufficientAuthentication : Result {
        ResultInsufficientAuthentication() : Result{5} {}
        static bool Includes(Result r) { return r.value == 5; }
    };
}
