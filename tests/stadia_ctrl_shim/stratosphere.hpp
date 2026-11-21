#pragma once
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <mutex>

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using s64 = int64_t;

struct Result {
    u32 value;
    u32 GetValue() const { return value; }
};

#define R_SUCCEEDED(r) ((r).value == 0)
#define R_FAILED(r) (!R_SUCCEEDED(r))
#define R_THROW(r) return (r)
#define R_RETURN(r) return (r)
#define R_SUCCEED() return Result{0}
#define R_TRY(r) do { auto rc_ = (r); if (R_FAILED(rc_)) return rc_; } while (0)
#define R_UNLESS(p, r) do { if (!(p)) return (r); } while (0)

namespace ams::svc {
    inline Result ResultInvalidSize() { return {10}; }
    inline Result ResultInvalidState() { return {11}; }
}
namespace ams::sf {
    inline Result ResultNotSupported() { return {12}; }
}
namespace ams::hos {
    constexpr int Version_9_0_0 = 9;
    inline int g_version = 22;
    inline int GetVersion() { return g_version; }
}
namespace ams {
    struct TimeSpan {
        s64 ns;
        static constexpr TimeSpan FromSeconds(s64 s) { return {s * 1000000000LL}; }
        static constexpr TimeSpan FromMilliSeconds(s64 ms) { return {ms * 1000000LL}; }
        constexpr s64 GetNanoSeconds() const { return ns; }
        constexpr s64 GetMilliSeconds() const { return ns / 1000000LL; }
    };
}
namespace ams::os {
    using SdkMutex = std::mutex;
    inline s64 g_now_ms = 1000;
    struct Tick {
        s64 ms;
        s64 GetInt64Value() const { return ms; }
        TimeSpan ToTimeSpan() const { return TimeSpan::FromMilliSeconds(ms); }
    };
    inline Tick GetSystemTick() { return {g_now_ms}; }
    inline Tick ConvertToTick(TimeSpan ts) { return {ts.GetMilliSeconds()}; }
}

struct BtdrvGattAttributeUuid { u16 value; };
struct BtdrvGattId { BtdrvGattAttributeUuid uuid; u8 instance_id; };
enum BtdrvGattAuthReqType : u8 {
    BtdrvGattAuthReqType_None = 0,
    BtdrvGattAuthReqType_NoMitm = 1,
};

namespace ams::bluetooth {
    struct Address { u8 address[6]; };
    struct HidReport {
        u16 size;
        u8 data[0x40];
    };
    struct HidReportEventInfo {
        struct {
            struct {
                u32 res;
                u8 proto_mode;
                Address addr;
                u8 pad;
                HidReport report;
            } v9;
        } data_report;
    };
}

Result btdrvWriteGattCharacteristic(u32 conn_id, bool primary, const BtdrvGattId *service_id, const BtdrvGattId *char_id, const void *data, size_t size, BtdrvGattAuthReqType auth_req, bool with_response);
