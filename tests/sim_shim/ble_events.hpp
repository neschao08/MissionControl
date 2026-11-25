#pragma once
#include "ble_types.hpp"
#include <functional>

namespace ams::ble::gatt { void OnEvent(const EventRecord &); }

namespace ams::ble::events {
    struct SimSlot {
        bool active = false;
        bluetooth::Address address = {};
        u32 conn_id = InvalidConnId;
        bool disconnected = false;
        u16 disconnect_reason = 0;
        bool bond_complete = false;
    };

    inline SimSlot slots[MaxBleControllers] = {};
    inline bool radio = true;
    inline u64 now_ms = 0;
    inline std::function<void(u64)> on_tick_advance;

    constexpr int InvalidType = -1;
    constexpr int InternalCoreBond = -10;
    constexpr int InternalSleep = -11;
    constexpr int InternalCallback = -12;

    struct Scheduled {
        u64 at_ms;
        EventRecord record;
        u32 target_slot;
        std::function<void()> callback;
    };
    inline std::deque<Scheduled> queue;

    inline void ScheduleEvent(const EventRecord &record, u64 delay_ms, u32 target_slot = InvalidSlot) {
        queue.push_back({now_ms + delay_ms, record, target_slot, nullptr});
        std::stable_sort(queue.begin(), queue.end(), [](const auto &a, const auto &b) { return a.at_ms < b.at_ms; });
    }

    inline void ScheduleCallback(u64 delay_ms, std::function<void()> cb) {
        EventRecord rec{};
        rec.type = InternalCallback;
        queue.push_back({now_ms + delay_ms, rec, InvalidSlot, std::move(cb)});
        std::stable_sort(queue.begin(), queue.end(), [](const auto &a, const auto &b) { return a.at_ms < b.at_ms; });
    }

    inline void ScheduleBond(u32 slot, u64 delay_ms) {
        EventRecord rec{};
        rec.type = InternalCoreBond;
        ScheduleEvent(rec, delay_ms, slot);
    }

    inline void ScheduleSleep(u64 delay_ms) {
        EventRecord rec{};
        rec.type = InternalSleep;
        ScheduleEvent(rec, delay_ms);
    }

    inline void SetSlotConnecting(u32 slot, const bluetooth::Address &addr) {
        slots[slot] = {true, addr, InvalidConnId, false, 0, false};
    }
    inline s64 NowMs() { return static_cast<s64>(now_ms); }
    inline bool IsRadioEnabled() { return radio; }
    inline bool notif_dirty = false;
    inline void NoteNotificationRegistered(u32) {}
    inline bool TakeNotifTableDirty() { const bool v = notif_dirty; notif_dirty = false; return v; }
    inline bool IsSlotDisconnected(u32 slot) { return slot >= MaxBleControllers || slots[slot].disconnected; }
    inline u32 GetSlotConnectionId(u32 slot) {
        if (slot >= MaxBleControllers || !slots[slot].active || slots[slot].disconnected) return InvalidConnId;
        return slots[slot].conn_id;
    }
    inline u16 GetSlotDisconnectReason(u32 slot) { return slot < MaxBleControllers ? slots[slot].disconnect_reason : 0; }
    inline bool TakeSlotBondComplete(u32 slot) {
        if (slot >= MaxBleControllers) return false;
        const bool v = slots[slot].bond_complete;
        slots[slot].bond_complete = false;
        return v;
    }

    inline void DispatchInternal(const Scheduled &item) {
        if (item.record.type == InternalCallback) {
            if (item.callback) item.callback();
            return;
        }
        if (item.record.type == InternalSleep) {
            radio = false;
            return;
        }
        if (item.record.type == InternalCoreBond) {
            if (item.target_slot < MaxBleControllers) slots[item.target_slot].bond_complete = true;
            return;
        }
        if (item.record.type == BtdrvBleEventType_ClientConnection) {
            const auto &c = item.record.info.client_connection;
            const bool connected = (c.result == 0) && (c.status == BleConnectionStatus_Connected);
            for (u32 s = 0; s < MaxBleControllers; ++s) {
                if (slots[s].active && (AddressEqual(slots[s].address, c.address) || (c.conn_id != InvalidConnId && slots[s].conn_id == c.conn_id))) {
                    if (connected) {
                        slots[s].conn_id = c.conn_id;
                        slots[s].disconnected = false;
                    } else if (slots[s].conn_id == InvalidConnId || slots[s].conn_id == c.conn_id) {
                        slots[s].disconnected = true;
                        slots[s].disconnect_reason = c.reason;
                        if (c.result != 0) notif_dirty = true;
                    }
                    break;
                }
            }
        }
        gatt::OnEvent(item.record);
    }

    template<typename F>
    bool WaitFor(int want, TimeSpan timeout, EventRecord *out, F pred) {
        const u64 deadline = now_ms + timeout.milliseconds;
        for (;;) {
            while (!queue.empty() && queue.front().at_ms <= now_ms) {
                auto item = std::move(queue.front());
                queue.pop_front();
                DispatchInternal(item);
                if (item.record.type == InternalCallback) {
                    continue;
                }
                if ((want == InvalidType || item.record.type == want) && pred(item.record)) {
                    if (out) *out = item.record;
                    return true;
                }
            }
            if (want == InvalidType) {
                EventRecord probe{};
                probe.type = InvalidType;
                if (pred(probe)) {
                    if (out) *out = probe;
                    return true;
                }
            }
            if (queue.empty() || queue.front().at_ms > deadline) {
                now_ms = deadline;
                if (on_tick_advance) on_tick_advance(now_ms);
                return false;
            }
            now_ms = std::max(now_ms, queue.front().at_ms);
            if (on_tick_advance) on_tick_advance(now_ms);
        }
    }

    inline void Pump(TimeSpan d) {
        WaitFor(InvalidType, d, nullptr, [](const EventRecord &) { return false; });
    }
}
