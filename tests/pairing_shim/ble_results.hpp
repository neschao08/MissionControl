#pragma once
namespace ams::ble {
    struct ResultNoConnection : Result {
        ResultNoConnection() : Result{3} {}
        static bool Includes(Result r) { return r.value == 3; }
    };
}
