#pragma once
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using s32 = int32_t;
using s64 = int64_t;

struct Result { u32 value; u32 GetValue() const { return value; } };
#define R_SUCCEEDED(r) ((r).value == 0)
#define R_FAILED(r) (!R_SUCCEEDED(r))

namespace ams {
    struct TimeSpan {
        s64 ms;
        static constexpr TimeSpan FromMilliSeconds(s64 v) { return {v}; }
        static constexpr TimeSpan FromSeconds(s64 v) { return {v * 1000}; }
        constexpr TimeSpan operator+(TimeSpan o) const { return {ms + o.ms}; }
        constexpr TimeSpan operator-(TimeSpan o) const { return {ms - o.ms}; }
        constexpr bool operator>=(TimeSpan o) const { return ms >= o.ms; }
    };
}
namespace ams::os {
    using SdkMutex = std::mutex;
    enum EventClearMode { EventClearMode_AutoClear, EventClearMode_ManualClear };
    struct Event {
        explicit Event(EventClearMode) {}
        void Signal() {}
        void Clear() {}
        void Wait() {}
        void TimedWait(TimeSpan) {}
    };
    inline s64 g_tick_ms = 1000;
    struct Tick {
        s64 v;
        s64 GetInt64Value() const { return v; }
        TimeSpan ToTimeSpan() const { return {v}; }
    };
    inline Tick GetSystemTick() { return {g_tick_ms}; }
}
namespace ams::hos {
    constexpr int Version_12_0_0 = 12;
    inline int GetVersion() { return 22; }
}

struct BtdrvAddress { u8 address[6]; };
struct BtdrvGattAttributeUuid { u32 size; u8 uuid[0x10]; };
struct BtdrvGattId { BtdrvGattAttributeUuid uuid; u8 instance_id; };
struct BtdrvGattAttribute {
    u8 type;
    u16 handle;
    u16 group_end_handle;
    u8 property;
    bool is_primary;
    BtdrvGattId id;
};
struct BtdrvBleConnectionParameter {
    u16 min_conn_interval;
    u16 max_conn_interval;
    u16 min_ce_length;
    u16 max_ce_length;
    u16 slave_latency;
    u16 supervision_tout;
};

enum BtdrvBleEventType : u32 {
    BtdrvBleEventType_ClientRegistration            = 0,
    BtdrvBleEventType_ServerRegistration            = 1,
    BtdrvBleEventType_ConnectionUpdate              = 2,
    BtdrvBleEventType_PreferredConnectionParameters = 3,
    BtdrvBleEventType_ClientConnection              = 4,
    BtdrvBleEventType_ServerConnection              = 5,
    BtdrvBleEventType_ScanResult                    = 6,
    BtdrvBleEventType_ScanFilter                    = 7,
    BtdrvBleEventType_ClientNotify                  = 8,
    BtdrvBleEventType_ClientCacheSave               = 9,
    BtdrvBleEventType_ClientCacheLoad               = 10,
    BtdrvBleEventType_ClientConfigureMtu            = 11,
};

enum BtdrvEventType : u32 {
    BtdrvEventType_SspRequest     = 3,
    BtdrvEventType_Connection     = 4,
    BtdrvEventType_BluetoothCrash = 10,
};

enum BtdrvGattOperationType : u8 {
    BtdrvGattOperationType_CharacteristicRead  = 0,
    BtdrvGattOperationType_CharacteristicWrite = 1,
    BtdrvGattOperationType_DescriptorWrite     = 3,
    BtdrvGattOperationType_Notification        = 4,
};

struct BtdrvBleEventInfo {
    union {
        u8 raw[0x400];
        struct { u32 result; u8 client_if; u8 status; u8 pad[2]; } client_registration;
        struct { u32 result; u32 conn_id; u16 conn_interval; u16 conn_latency; u16 supervision_tout; u8 pad[2]; } connection_update;
        struct { u32 conn_id; u16 min_conn_interval; u16 max_conn_interval; u16 slave_latency; u16 supervision_tout; } preferred_connection_parameters;
        struct { u32 result; u8 status; u8 client_if; u8 pad[2]; u32 conn_id; BtdrvAddress address; u16 reason; } client_connection;
        struct { u32 result; u8 status; } scan_result;
        struct { u32 result; u8 action; } scan_filter;
        struct {
            u32 result; u32 conn_id; u8 op_type; u8 pad[3];
            BtdrvGattAttributeUuid serv_uuid;
            BtdrvGattAttributeUuid char_uuid;
            BtdrvGattAttributeUuid desc_uuid;
            u16 size; u8 data[0x200];
        } client_notify;
        struct { u32 result; u32 conn_id; u32 count; BtdrvGattAttribute attr_list[10]; } client_cache_save;
        struct { u32 result; u32 conn_id; u16 mtu; } client_configure_mtu;
    };
};

struct BtdrvEventInfo {
    union {
        u8 raw[0x400];
        struct {
            struct { BtdrvAddress addr; char name[0xF9]; u8 cod[3]; u8 flag; u8 pad; s32 passkey; } v12;
        } ssp_request;
        struct {
            struct { u32 type; BtdrvAddress addr; u8 reserved[0xfe]; } v12;
        } connection;
    };
};

namespace ams::bluetooth {
    using Address = ::BtdrvAddress;
    using BleEventType = ::BtdrvBleEventType;
    using BleEventInfo = ::BtdrvBleEventInfo;
    using EventInfo = ::BtdrvEventInfo;
}

Result btdrvRespondToSspRequest(BtdrvAddress, u32, bool, s32);
Result btdrvSetBleConnectionParameter(BtdrvAddress, const BtdrvBleConnectionParameter *, bool);
