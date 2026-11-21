/*
 * Copyright (c) 2020-2026 ndeadly
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "ble_hid_host.hpp"
#include "ble_events.hpp"
#include "ble_gatt.hpp"
#include "ble_log.hpp"
#include "ble_profile.hpp"
#include "ble_results.hpp"
#include "ble_recovery.hpp"
#include "ble_pairing.hpp"
#include "ble_advertising.hpp"
#include "../mcmitm_config.hpp"
#include "../mcmitm_initialization.hpp"
#include "../bluetooth_mitm/bluetooth/bluetooth_ble.hpp"
#include "../controllers/controller_management.hpp"
#include "../controllers/stadia_ble_controller.hpp"
#include "../controllers/xbox_one_ble_controller.hpp"

namespace ams::ble {

    namespace {

        constexpr s32 ThreadPriority = 9;
        constexpr size_t ThreadStackSize = 0x4000;
        alignas(os::ThreadStackAlignment) constinit u8 g_thread_stack[ThreadStackSize];
        constinit os::ThreadType g_thread;
        constinit bool g_launched = false;

        constexpr size_t LeHidThreadStackSize = 0x2000;
        alignas(os::ThreadStackAlignment) constinit u8 g_le_hid_thread_stack[LeHidThreadStackSize];
        constinit os::ThreadType g_le_hid_thread;
        constinit EventRecord g_le_hid_record;

        // Application UUID for our own GATT client, should we need to register one
        constexpr BtdrvGattAttributeUuid GattClientAppUuid = { 0x10, { 0x4d, 0x43, 0x42, 0x4c, 0x45, 0x48, 0x4f, 0x53, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 } };

        constinit u8 g_client_if = InvalidClientIf;
        RadioRecovery g_radio_recovery;
        u32 g_hid_data_path_generation = UINT32_MAX;

        constexpr s64 PairingRetryBackoffMs        = 5000;
        constexpr s64 StadiaRetryBackoffMs         = 1500;
        constexpr s64 BackgroundReconnectBackoffMs = 6000;
        constexpr s64 BackgroundScanIdlePeriodMs   = 2500;
        constexpr s64 BackgroundScanBusyPeriodMs   = 5000;
        constexpr u8  StadiaStrategyCount          = 3;

        constexpr const char KnownStadiaFilePath[] = "sdmc:/config/MissionControl/ble_known_stadia.bin";
        constexpr u32 KnownStadiaMagic = 0x53544431; // "STD1"

        struct KnownStadiaStore {
            u32 magic;
            u32 count;
            bluetooth::Address addresses[MaxBleControllers];
        };

        constinit os::SdkMutex g_known_stadia_lock;
        bluetooth::Address g_known_stadia[MaxBleControllers] = {};
        size_t g_known_stadia_count = 0;

        struct PairingFailure {
            bluetooth::Address address = {};
            s64 tick = 0;
            s64 backoff_ms = PairingRetryBackoffMs;
        };
        PairingFailure g_pairing_failures[MaxBleControllers];

        struct StadiaStrategyState {
            bluetooth::Address address = {};
            u8 stage = 0;
            u8 failures_in_cycle = 0;
            bool valid = false;
        };
        StadiaStrategyState g_stadia_strategies[MaxBleControllers] = {};

        bool PairingBackoffActive(const bluetooth::Address &address) {
            for (const auto &failure : g_pairing_failures) {
                if (failure.tick && AddressEqual(failure.address, address) &&
                    ((os::GetSystemTick() - os::Tick(failure.tick)).ToTimeSpan().GetMilliSeconds() < failure.backoff_ms)) return true;
            }
            return false;
        }

        void NotePairingFailure(const bluetooth::Address &address, s64 backoff_ms = PairingRetryBackoffMs) {
            auto *oldest = &g_pairing_failures[0];
            for (auto &failure : g_pairing_failures) {
                if (AddressEqual(failure.address, address)) { oldest = &failure; break; }
                if (failure.tick < oldest->tick) oldest = &failure;
            }
            *oldest = {address, os::GetSystemTick().GetInt64Value(), backoff_ms};
        }

        StadiaStrategyState *FindOrAllocStadiaStrategy(const bluetooth::Address &address) {
            StadiaStrategyState *empty = nullptr;
            for (auto &entry : g_stadia_strategies) {
                if (entry.valid && AddressEqual(entry.address, address)) {
                    return &entry;
                }
                if (!entry.valid && !empty) {
                    empty = &entry;
                }
            }
            StadiaStrategyState *slot = empty ? empty : &g_stadia_strategies[0];
            *slot = { address, 0, 0, true };
            return slot;
        }

        u8 GetStadiaStrategyStage(const bluetooth::Address &address) {
            return FindOrAllocStadiaStrategy(address)->stage;
        }

        void NoteStadiaStrategySuccess(const bluetooth::Address &address, u8 stage) {
            auto *state = FindOrAllocStadiaStrategy(address);
            state->stage = stage % StadiaStrategyCount;
            state->failures_in_cycle = 0;
        }

        // Advances to the next Stadia security stage. Returns true only after a full 3-stage cycle
        // has failed so we don't trigger a 60s radio cycle between stages.
        bool AdvanceStadiaStrategy(const bluetooth::Address &address) {
            auto *state = FindOrAllocStadiaStrategy(address);
            state->stage = static_cast<u8>((state->stage + 1) % StadiaStrategyCount);
            ++state->failures_in_cycle;
            if (state->failures_in_cycle >= StadiaStrategyCount) {
                state->failures_in_cycle = 0;
                return true;
            }
            return false;
        }

        /* Multi-device slot management */

        struct BleDeviceSlot {
            enum class State : u8 {
                Vacant,
                Connecting,
                Discovering,
                Pairing,
                Connected,
                Disconnecting
            };

            State state = State::Vacant;
            BleControllerType controller_type = BleControllerType::Unknown;
            bool was_connected = false;
            u32 conn_id = InvalidConnId;
            u8 client_if = InvalidClientIf;
            u16 negotiated_max_interval = 0;
            bluetooth::Address address = {};
            gatt::HidService hid = {};
            std::shared_ptr<controller::EmulatedSwitchController> controller = nullptr;
        };

        constinit os::SdkMutex g_slots_lock;
        BleDeviceSlot g_host_slots[MaxBleControllers];

        struct RememberedControllerType {
            bluetooth::Address address = {};
            BleControllerType type = BleControllerType::Unknown;
            bool valid = false;
        };
        constexpr size_t MaxRememberedTypes = 16;
        constinit os::SdkMutex g_remembered_lock;
        RememberedControllerType g_remembered_types[MaxRememberedTypes] = {};
        size_t g_remembered_next = 0;

        void RememberControllerType(const bluetooth::Address &address, BleControllerType type) {
            if (type == BleControllerType::Unknown) {
                return;
            }
            std::scoped_lock lk(g_remembered_lock);
            for (auto &entry : g_remembered_types) {
                if (entry.valid && AddressEqual(entry.address, address)) {
                    entry.type = type;
                    return;
                }
            }
            g_remembered_types[g_remembered_next] = { address, type, true };
            g_remembered_next = (g_remembered_next + 1) % MaxRememberedTypes;
        }

        BleControllerType RecallControllerType(const bluetooth::Address &address) {
            std::scoped_lock lk(g_remembered_lock);
            for (const auto &entry : g_remembered_types) {
                if (entry.valid && AddressEqual(entry.address, address)) {
                    return entry.type;
                }
            }
            return BleControllerType::Unknown;
        }

        BleControllerType ResolveControllerType(const BleDeviceSlot &slot) {
            if (slot.hid.has_pnp_id) {
                if ((slot.hid.vendor_id == controller::stadia::VendorId) && (slot.hid.product_id == controller::stadia::ProductId)) {
                    return BleControllerType::Stadia;
                }
                if (slot.hid.vendor_id == 0x045e) {
                    return BleControllerType::XboxOne;
                }
            }
            if (slot.controller_type != BleControllerType::Unknown) {
                return slot.controller_type;
            }
            const auto recalled = RecallControllerType(slot.address);
            if (recalled != BleControllerType::Unknown) {
                return recalled;
            }
            if (slot.hid.device_name[0]) {
                if (StartsWithIgnoreCase(slot.hid.device_name, "Stadia")) return BleControllerType::Stadia;
                if (std::strncmp(slot.hid.device_name, "Xbox", 4) == 0) return BleControllerType::XboxOne;
            }
            return BleControllerType::XboxOne;
        }

        size_t GetConnectedCount() {
            std::scoped_lock lk(g_slots_lock);
            size_t count = 0;
            for (size_t i = 0; i < MaxBleControllers; ++i) {
                if (g_host_slots[i].state == BleDeviceSlot::State::Connected) {
                    ++count;
                }
            }
            return count;
        }

        int FindFreeSlot() {
            std::scoped_lock lk(g_slots_lock);
            for (size_t i = 0; i < MaxBleControllers; ++i) {
                if (g_host_slots[i].state == BleDeviceSlot::State::Vacant) {
                    return static_cast<int>(i);
                }
            }
            return -1;
        }

        u32 FindSlotForAddress(const bluetooth::Address &address, u32 fallback) {
            for (u32 i = 0; i < MaxBleControllers; ++i) {
                if ((g_host_slots[i].state == BleDeviceSlot::State::Vacant) && gatt::HasCachedAddress(i, address)) return i;
            }
            return fallback;
        }

        u32 FindConnectedSlotByAddress(const bluetooth::Address &address, u32 exclude_slot = InvalidSlot) {
            std::scoped_lock lk(g_slots_lock);
            for (size_t i = 0; i < MaxBleControllers; ++i) {
                if (i == exclude_slot) {
                    continue;
                }
                if ((g_host_slots[i].state != BleDeviceSlot::State::Vacant) && AddressEqual(g_host_slots[i].address, address)) {
                    return static_cast<u32>(i);
                }
            }
            return InvalidSlot;
        }

        bool IsAddressConnected(const bluetooth::Address &address, u32 exclude_slot = InvalidSlot) {
            return FindConnectedSlotByAddress(address, exclude_slot) != InvalidSlot;
        }

        bool IsKnownStadia(const bluetooth::Address &address) {
            std::scoped_lock lk(g_known_stadia_lock);
            for (size_t i = 0; i < g_known_stadia_count; ++i) {
                if (AddressEqual(g_known_stadia[i], address)) {
                    return true;
                }
            }
            return false;
        }

        bool HasDisconnectedKnownStadia() {
            bluetooth::Address snapshot[MaxBleControllers] = {};
            size_t count = 0;
            {
                std::scoped_lock lk(g_known_stadia_lock);
                count = g_known_stadia_count;
                for (size_t i = 0; i < count; ++i) {
                    snapshot[i] = g_known_stadia[i];
                }
            }
            for (size_t i = 0; i < count; ++i) {
                if (!IsAddressConnected(snapshot[i]) && !PairingBackoffActive(snapshot[i])) {
                    return true;
                }
            }
            return false;
        }

        void SaveKnownStadiaLocked() {
            KnownStadiaStore store = {};
            store.magic = KnownStadiaMagic;
            store.count = static_cast<u32>(g_known_stadia_count);
            for (size_t i = 0; i < g_known_stadia_count; ++i) {
                store.addresses[i] = g_known_stadia[i];
            }

            static_cast<void>(fs::CreateDirectory("sdmc:/config"));
            static_cast<void>(fs::CreateDirectory("sdmc:/config/MissionControl"));

            fs::FileHandle file;
            Result rc = fs::OpenFile(std::addressof(file), KnownStadiaFilePath, fs::OpenMode_Write);
            if (R_FAILED(rc)) {
                static_cast<void>(fs::CreateFile(KnownStadiaFilePath, sizeof(store)));
                rc = fs::OpenFile(std::addressof(file), KnownStadiaFilePath, fs::OpenMode_Write);
                if (R_FAILED(rc)) {
                    return;
                }
            }
            ON_SCOPE_EXIT { fs::CloseFile(file); };
            static_cast<void>(fs::SetFileSize(file, sizeof(store)));
            static_cast<void>(fs::WriteFile(file, 0, &store, sizeof(store), fs::WriteOption::Flush));
        }

        void RememberKnownStadia(const bluetooth::Address &address) {
            bool changed = false;
            {
                std::scoped_lock lk(g_known_stadia_lock);
                bool exists = false;
                for (size_t i = 0; i < g_known_stadia_count; ++i) {
                    if (AddressEqual(g_known_stadia[i], address)) {
                        exists = true;
                        break;
                    }
                }
                if (!exists) {
                    if (g_known_stadia_count < MaxBleControllers) {
                        g_known_stadia[g_known_stadia_count++] = address;
                    } else {
                        for (size_t i = 1; i < MaxBleControllers; ++i) {
                            g_known_stadia[i - 1] = g_known_stadia[i];
                        }
                        g_known_stadia[MaxBleControllers - 1] = address;
                    }
                    SaveKnownStadiaLocked();
                    changed = true;
                }
            }
            if (changed) {
                events::LogAddress("remembered Stadia controller for auto-reconnect", address);
            }
        }

        void LoadKnownStadia() {
            fs::FileHandle file;
            if (R_FAILED(fs::OpenFile(std::addressof(file), KnownStadiaFilePath, fs::OpenMode_Read))) {
                return;
            }
            ON_SCOPE_EXIT { fs::CloseFile(file); };

            KnownStadiaStore store = {};
            size_t read_size = 0;
            if (R_FAILED(fs::ReadFile(std::addressof(read_size), file, 0, &store, sizeof(store))) ||
                (read_size < sizeof(store)) || (store.magic != KnownStadiaMagic)) {
                return;
            }

            const size_t n = std::min<size_t>(store.count, MaxBleControllers);
            {
                std::scoped_lock lk(g_known_stadia_lock);
                g_known_stadia_count = n;
                for (size_t i = 0; i < n; ++i) {
                    g_known_stadia[i] = store.addresses[i];
                }
            }
            for (size_t i = 0; i < n; ++i) {
                RememberControllerType(store.addresses[i], BleControllerType::Stadia);
                events::LogAddress("loaded known Stadia controller", store.addresses[i]);
            }
        }

        /* LE HID event stream: bounded lookup with ownership across dispatch */

        void LeHidThreadFunc(void *) {
            Event event;
            Result rc = btdrvRegisterBleHidEvent(&event);
            log::Write("btdrvRegisterBleHidEvent rc=0x%x", rc.GetValue());
            if (R_FAILED(rc)) {
                return;
            }

            os::SystemEvent system_event;
            system_event.AttachReadableHandle(event.revent, false, os::EventClearMode_AutoClear);

            for (;;) {
                system_event.Wait();

                rc = btdrvGetLeHidEventInfo(&g_le_hid_record.info, sizeof(g_le_hid_record.info), &g_le_hid_record.type);
                if (R_FAILED(rc)) {
                    static u32 error_logs = 0;
                    if (error_logs < 5) {
                        ++error_logs;
                        log::Write("btdrvGetLeHidEventInfo rc=0x%x", rc.GetValue());
                    }
                    continue;
                }

                if (g_le_hid_record.type != BtdrvBleEventType_ClientNotify) {
                    // The HID data path can deliver connection lifecycle events too.
                    // Apply them to slot state, not just to the logging/GATT queue.
                    static_cast<void>(events::OnBleEvent(g_le_hid_record.type, &g_le_hid_record.info));
                    continue;
                }

                auto &notify = g_le_hid_record.info.client_notify;

                if ((notify.op_type == GattNotifyType_Notification) && (notify.result == 0) && IsUuid16(notify.char_uuid, GattCharReport)) {
                    // Keep the controller alive until dispatch finishes, including during disconnect.
                    if (auto controller = events::GetActiveController(notify.conn_id)) {
                        const size_t report_len = std::min<size_t>(notify.size, sizeof(notify.data));
                        events::NoteReceivedInputReport("le_hid", notify.conn_id, notify.data, report_len);
                        const u64 start_tick = profile::RecordReportStart();
                        static_cast<void>(controller->HandleBleInputReport(notify.data, report_len));
                        profile::RecordReportEnd(start_tick);
                    } else {
                        events::PushRecord(g_le_hid_record);
                    }
                } else if ((notify.op_type == GattNotifyType_WriteCharacteristic) && IsUuid16(notify.char_uuid, GattCharReport)) {
                    events::NoteRumbleWriteCompletion(notify.conn_id, notify.result);
                } else {
                    events::NoteGattResult(notify.conn_id, notify.op_type, notify.char_uuid, notify.result);
                    events::PushRecord(g_le_hid_record);
                }
            }
        }

        constexpr s32 LowLatencyThreadPriority = -10; // 1 level below mc::HidReportThread (-11) to avoid preempting Classic BT report draining
        constexpr s32 DefaultThreadPriority = 9;

        void StartLeHidListener() {
            const s32 priority = mitm::GetGlobalConfig()->ble.low_latency ? LowLatencyThreadPriority : DefaultThreadPriority;
            R_ABORT_UNLESS(os::CreateThread(&g_le_hid_thread,
                LeHidThreadFunc,
                nullptr,
                g_le_hid_thread_stack,
                LeHidThreadStackSize,
                priority
            ));

            os::SetThreadNamePointer(&g_le_hid_thread, "mc::BleHidEventThread");
            os::StartThread(&g_le_hid_thread);
        }

        /* Bring-up */

        Result RegisterGattClient() {
            events::SetRegistering(true);
            ON_SCOPE_EXIT { events::SetRegistering(false); };

            Result rc = btdrvRegisterGattClient(&GattClientAppUuid);
            log::Write("btdrvRegisterGattClient rc=0x%x", rc.GetValue());
            R_TRY(rc);

            EventRecord e;
            if (!events::WaitFor(BtdrvBleEventType_ClientRegistration, TimeSpan::FromSeconds(5), &e, [](const EventRecord &r) { return r.info.client_registration.status == 1; })) {
                R_THROW(svc::ResultTimedOut());
            }

            g_client_if = e.info.client_registration.client_if;
            R_SUCCEED();
        }

        Result PrepareClient() {
            events::TakeOwnership();
            events::WaitRadioEnabled();

            Result rc = btdrvEnableBle();
            log::Write("btdrvEnableBle rc=0x%x", rc.GetValue());
            events::Pump(TimeSpan::FromSeconds(6));

            if (events::GetAvailableClientIfCount() > 0) {
                log::Write("found %zu gatt client_ifs from btm (primary=%u)", events::GetAvailableClientIfCount(), events::GetClientIfForSlot(0));
                R_SUCCEED();
            }

            for (int attempt = 0; attempt < 3; ++attempt) {
                if (R_SUCCEEDED(RegisterGattClient())) {
                    log::Write("registered gatt client_if=%u", g_client_if);
                    R_SUCCEED();
                }
                os::SleepThread(TimeSpan::FromSeconds(2));
            }

            R_THROW(svc::ResultTimedOut());
        }

        void DismissSlot(u32 slot_index, bool close_transport = true);

        void DisconnectAll(bool close_transport = true) {
            for (size_t i = 0; i < MaxBleControllers; ++i) {
                if (g_host_slots[i].state != BleDeviceSlot::State::Vacant) {
                    DismissSlot(static_cast<u32>(i), close_transport);
                }
            }
            events::Pump(TimeSpan::FromSeconds(1));
        }

        void CleanupSleepIfNeeded() {
            events::WaitForSleepPreparation();
            if (events::TakeSleepCleanup()) {
                bool had_links = events::GetActiveConnectionCount() != 0;
                {
                    std::scoped_lock lk(g_slots_lock);
                    for (const auto &slot : g_host_slots) {
                        had_links = had_links || (slot.state != BleDeviceSlot::State::Vacant);
                    }
                }
                DisconnectAll(false); // the PSC thread already closed the transport
                g_radio_recovery.Sleep(had_links);
            }
        }

        void WaitForRadio() {
            CleanupSleepIfNeeded();
            if (!events::IsRadioEnabled()) {
                log::Write("BLE suspended, waiting for wake");
                events::WaitRadioEnabled();
                log::Write("BLE awake");
            }
            // Covers a complete sleep/wake transition while setup was unwinding.
            CleanupSleepIfNeeded();
            if (events::TakeRadioCycled()) {
                events::Pump(TimeSpan::FromSeconds(2));
                log::Write("radio awake: %zu GATT clients available", events::GetAvailableClientIfCount());
            }
        }

        /* Scanning and connection */

        Result ConfigureScanFilter(u32 action, bool quiet = false) {
            events::ExpectFilterEvent();
            Result rc = action == 2 ? btdrvClearBleScanFilters() : btdrvEnableBleScanFilter(action == 3);
            if (!quiet || R_FAILED(rc)) {
                log::Write("scan filter request action=%u rc=0x%x", action, rc.GetValue());
            }
            if (R_FAILED(rc)) {
                // A rejected IPC does not have an asynchronous completion.
                events::CancelExpectedFilterEvent();
                return rc;
            }
            EventRecord e = {};
            const bool completed = events::WaitFor(BtdrvBleEventType_ScanFilter, TimeSpan::FromSeconds(1), &e,
                [=](const EventRecord &r) { return r.info.scan_filter.action == action; });
            if (!completed) log::Write("scan filter completion timed out action=%u", action);
            R_UNLESS(completed, svc::ResultTimedOut());
            R_UNLESS(e.info.scan_filter.result == 0, svc::ResultInvalidState());
            R_SUCCEED();
        }

        Result Scan(bluetooth::Address *out_address, BleControllerType *out_type, u8 *out_flags, TimeSpan duration, bool background_reconnect = false, u32 *out_stale_slot = nullptr) {
            if (out_stale_slot) {
                *out_stale_slot = InvalidSlot;
            }
            const auto should_abort_scan = [&] {
                if (!events::IsRadioEnabled() || events::HasAdoptedConnection()) {
                    return true;
                }
                return background_reconnect ? events::IsPairingWindowOpen() : !events::IsPairingWindowOpen();
            };

            events::SetQuietScanLogs(background_reconnect);
            ON_SCOPE_EXIT { events::SetQuietScanLogs(false); };
            events::SetScanning(true);
            ON_SCOPE_EXIT { events::SetScanning(false); };

            // Filter and scan calls complete asynchronously. Keep ownership until
            // their completions have been drained, including when no pad is found.
            ON_SCOPE_EXIT {
                if (events::IsRadioEnabled()) {
                    Result restore_rc = ConfigureScanFilter(3, background_reconnect);
                    if (R_FAILED(restore_rc)) log::Write("scan filter restore failed rc=0x%x", restore_rc.GetValue());
                }
            };
            R_TRY(ConfigureScanFilter(2, background_reconnect));
            R_TRY(ConfigureScanFilter(4, background_reconnect));
            R_UNLESS(!should_abort_scan(), svc::ResultCancelled());

            Result rc = btdrvStartBleScan();
            if (!background_reconnect && ResultScanAlreadyActive::Includes(rc) && events::IsRadioEnabled()) {
                // btm starts its own 10-second BLE scan when "Change Grip/Order" opens, causing
                // btdrvStartBleScan to fail with 0x2a871 for 10 seconds. Stop that scan and claim it immediately.
                log::Write("btdrvStartBleScan hit 0x2a871 (already active); stopping existing scan and retrying");
                const Result preempt_rc = btdrvStopBleScan();
                if (R_SUCCEEDED(preempt_rc)) {
                    events::WaitFor(BtdrvBleEventType_ScanResult, TimeSpan::FromMilliSeconds(500), nullptr,
                        [](const EventRecord &r) { return r.info.scan_result.status == 1; });
                    if (!should_abort_scan()) {
                        rc = btdrvStartBleScan();
                    }
                }
            }
            if (!background_reconnect || (R_FAILED(rc) && !ResultScanAlreadyActive::Includes(rc))) {
                log::Write("btdrvStartBleScan rc=0x%x", rc.GetValue());
            }
            if (R_FAILED(rc)) {
                if (!background_reconnect) {
                    // Give the shared radio time before retrying a rejected scan.
                    events::Pump(TimeSpan::FromSeconds(2));
                }
                return rc;
            }

            u32 advertisements = 0, rejected_transport = 0, rejected_identity = 0, unavailable = 0, failed = 0;
            bluetooth::Address logged_addresses[8] = {};
            size_t logged_count = 0;
            bool scan_complete = false;
            u32 stale_slot = InvalidSlot;
            EventRecord e = {};
            const bool found = events::WaitFor(events::InvalidType, duration, &e, [&](const EventRecord &r) {
                if (should_abort_scan()) {
                    return true;
                }
                if (r.type != BtdrvBleEventType_ScanResult) {
                    return false;
                }
                const auto &scan = r.info.scan_result;
                if (scan.status == 1) {
                    scan_complete = true;
                    return true;
                }
                if (scan.status != BleScanStatus_NewDevice) return false;
                ++advertisements;
                const auto summary = ParseAdvertisements(scan.ad_list, scan.count);
                BleControllerType candidate_type = BleControllerType::Unknown;
                const char *decision = "candidate";
                if (background_reconnect) {
                    if (scan.result != 0) { ++failed; decision = "scan-error"; }
                    else if (!IsKnownStadia(scan.address)) { ++rejected_identity; decision = "not-known-stadia"; }
                    else if (IsAddressConnected(scan.address) || PairingBackoffActive(scan.address)) { ++unavailable; decision = "active-or-backoff"; }
                    else { candidate_type = BleControllerType::Stadia; }
                } else {
                    if (scan.result != 0) { ++failed; decision = "scan-error"; }
                    else if (!IsBleAdvertisement(summary, scan.device_type)) { ++rejected_transport; decision = "classic"; }
                    else if (!IsSupportedBleCandidate(summary, &candidate_type)) { ++rejected_identity; decision = "identity"; }
                    else if (IsAddressConnected(scan.address)) {
                        const u32 existing_slot = FindConnectedSlotByAddress(scan.address);
                        // If a connected Stadia controller is actively advertising Limited Discoverable
                        // (flags=0x05, orange pairing mode), the old slot is stale/dead; preempt it immediately.
                        if ((candidate_type == BleControllerType::Stadia) && ((summary.flags & 0x01) != 0) && (existing_slot != InvalidSlot)) {
                            stale_slot = existing_slot;
                            decision = "stale-reconnect";
                        } else {
                            ++unavailable;
                            decision = "active-or-backoff";
                        }
                    } else if (PairingBackoffActive(scan.address)) { ++unavailable; decision = "active-or-backoff"; }
                }
                const bool candidate = (std::strcmp(decision, "candidate") == 0) || (std::strcmp(decision, "stale-reconnect") == 0);

                // Bound diagnostic SD writes: one detail per address (up to eight
                // per scan), plus the selected candidate. No input-thread I/O.
                if (log::IsEnabled() && (candidate || (!background_reconnect && logged_count < std::size(logged_addresses) &&
                        std::none_of(logged_addresses, logged_addresses + logged_count,
                            [&](const auto &address) { return AddressEqual(address, scan.address); })))) {
                    if (logged_count < std::size(logged_addresses)) logged_addresses[logged_count++] = scan.address;
                    events::LogAddress(background_reconnect ? "bg reconnect scan device" : "scan device", scan.address);
                    log::Write("  result=%u type=%u addr_type=%u rssi=%ld name='%s' appearance=0x%04x flags=0x%02x hid=%u decision=%s",
                        scan.result, scan.device_type, scan.ble_addr_type, static_cast<long>(scan.rssi), summary.name,
                        summary.appearance, summary.flags, summary.has_hid_service, decision);
                }
                return candidate;
            });

            if (!scan_complete && events::IsRadioEnabled()) {
                Result stop_rc = btdrvStopBleScan();
                if (!background_reconnect || R_FAILED(stop_rc)) {
                    log::Write("btdrvStopBleScan rc=0x%x", stop_rc.GetValue());
                }
                if (R_SUCCEEDED(stop_rc)) {
                    scan_complete = events::WaitFor(BtdrvBleEventType_ScanResult, TimeSpan::FromSeconds(1), nullptr,
                        [](const EventRecord &r) { return r.info.scan_result.status == 1; });
                    if (!scan_complete) log::Write("scan stop completion timed out");
                }
                R_TRY(stop_rc);
                R_UNLESS(scan_complete, svc::ResultTimedOut());
            }

            if (!background_reconnect) {
                log::Write("scan summary: advertisements=%u classic=%u identity=%u unavailable=%u errors=%u dropped=%u",
                    advertisements, rejected_transport, rejected_identity, unavailable, failed, events::TakeRingDropped());
            }

            R_UNLESS(!should_abort_scan(), svc::ResultCancelled());
            if (stale_slot != InvalidSlot) {
                if (out_stale_slot) {
                    *out_stale_slot = stale_slot;
                }
                R_SUCCEED();
            }
            R_UNLESS(found && (e.type == BtdrvBleEventType_ScanResult) &&
                (e.info.scan_result.status == BleScanStatus_NewDevice), svc::ResultTimedOut());

            *out_address = e.info.scan_result.address;
            if (out_type || out_flags) {
                const auto summary = ParseAdvertisements(e.info.scan_result.ad_list, e.info.scan_result.count);
                if (out_flags) {
                    *out_flags = summary.flags;
                }
                if (out_type) {
                    *out_type = IdentifyCandidate(summary);
                    if (*out_type == BleControllerType::Unknown && background_reconnect && IsKnownStadia(*out_address)) {
                        *out_type = BleControllerType::Stadia;
                    }
                    if (*out_type != BleControllerType::Unknown) {
                        RememberControllerType(*out_address, *out_type);
                    } else {
                        *out_type = RecallControllerType(*out_address);
                    }
                }
            }
            events::LogAddress(background_reconnect ? "bg reconnect found Stadia" : "found controller", *out_address);

            if (log::IsEnabled()) {
                SetSysBluetoothDevicesSettings settings = {};
                Result info_rc = btdrvGetPairedDeviceInfo(*out_address, &settings);
                log::Write("btdrvGetPairedDeviceInfo rc=0x%x key_present=%u key_type=%u device_type=%u",
                    info_rc.GetValue(), settings.link_key_present, settings.key_type, settings.device_type);
            }

            R_SUCCEED();
        }

        bool Adopt(bluetooth::Address *out_address, u32 *out_conn_id) {
            if (!events::TakeAdoptedConnection(out_address, out_conn_id)) {
                return false;
            }
            log::Write("controller connected on its own, conn_id=%u", *out_conn_id);
            return true;
        }

        /* Presenting and dismissing controllers */

        void RebalanceConnectionParameters() {
            const size_t connected_count = GetConnectedCount();
            if (connected_count == 0) {
                return;
            }

            const u16 target_max = events::GetTargetMaxConnectionInterval(connected_count);
            if (target_max == 0) {
                return;
            }

            for (size_t i = 0; i < MaxBleControllers; ++i) {
                if (g_host_slots[i].state == BleDeviceSlot::State::Connected) {
                    if (g_host_slots[i].negotiated_max_interval != target_max) {
                        g_host_slots[i].negotiated_max_interval = events::RequestLowLatencyConnectionParameters(g_host_slots[i].address, connected_count);
                    }
                }
            }
        }

        void PresentSlot(u32 slot_index) {
            auto &slot = g_host_slots[slot_index];
            if (!slot.hid.has_output_report) {
                log::Write("slot %u: no output Report characteristic, rumble unavailable", slot_index);
            }

            const BleControllerType controller_type = ResolveControllerType(slot);
            slot.controller_type = controller_type;
            RememberControllerType(slot.address, controller_type);
            if (controller_type == BleControllerType::Stadia) {
                RememberKnownStadia(slot.address);
            }

            std::shared_ptr<controller::EmulatedSwitchController> controller;
            if (controller_type == BleControllerType::Stadia) {
                controller = std::make_shared<controller::StadiaBleController>(
                    slot.address, slot.conn_id, slot.hid.service.id, slot.hid.service.is_primary,
                    slot.hid.output_report.id, slot.hid.has_output_report, slot.hid.output_write_with_response,
                    slot.hid.write_auth_req);
            } else {
                controller = std::make_shared<controller::XboxOneBleController>(
                    slot.address, slot.conn_id, slot.hid.service.id, slot.hid.service.is_primary,
                    slot.hid.output_report.id, slot.hid.has_output_report);
            }

            {
                std::scoped_lock lk(g_slots_lock);
                slot.controller = controller;
                slot.state = BleDeviceSlot::State::Connected;
                slot.was_connected = true;
            }

            // Initialize settings and storage before the input thread can see this controller.
            controller::AttachVirtualController(controller);
            if (GetConnectedCount() == 1) profile::BeginSession();
            for (auto &failure : g_pairing_failures) {
                if (AddressEqual(failure.address, slot.address)) failure = {};
            }

            const u64 reports_before = events::GetSlotInputReportCount(slot_index);
            if (controller_type == BleControllerType::Stadia) {
                const u8 neutral[sizeof(controller::stadia::StadiaInputReport0x03)] = { 0x08, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00 };
                static_cast<void>(controller->HandleBleInputReport(neutral, sizeof(neutral)));
            } else {
                const u8 neutral[sizeof(controller::XboxOneInputReport0x01)] = { 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80 };
                static_cast<void>(controller->HandleBleInputReport(neutral, sizeof(neutral)));
            }
            events::RegisterSlotController(slot_index, slot.conn_id, controller);

            // Request low-latency parameters and rebalance active BLE slots
            slot.negotiated_max_interval = 0;
            RebalanceConnectionParameters();

            // If no real Stadia HID packet arrived on this slot within 150ms of attach, send a second
            // neutral frame so the Switch "Change Grip/Order" UI reliably sees an active controller.
            if (controller_type == BleControllerType::Stadia) {
                events::WaitFor(events::InvalidType, TimeSpan::FromMilliSeconds(150), nullptr, [&](const EventRecord &) {
                    return events::IsSlotDisconnected(slot_index) || !events::IsRadioEnabled() ||
                        (events::GetSlotInputReportCount(slot_index) != reports_before);
                });
                if (!events::IsSlotDisconnected(slot_index) && events::IsRadioEnabled() &&
                    (events::GetSlotInputReportCount(slot_index) == reports_before)) {
                    const u8 neutral[sizeof(controller::stadia::StadiaInputReport0x03)] = { 0x08, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00 };
                    static_cast<void>(controller->HandleBleInputReport(neutral, sizeof(neutral)));
                }
            }

            log::Write("slot %u: %s controller presented to system (conn_id=%u, write_auth=%u, total active=%zu)",
                slot_index, controller_type == BleControllerType::Stadia ? "Stadia" : "Xbox",
                slot.conn_id, slot.hid.write_auth_req, GetConnectedCount());
        }

        void DismissSlot(u32 slot_index, bool close_transport) {
            events::WaitForSleepPreparation();
            std::shared_ptr<controller::EmulatedSwitchController> controller;
            bluetooth::Address address = {};
            u32 conn_id = InvalidConnId;
            bool already_disconnect_pending = false;

            {
                std::scoped_lock lk(g_slots_lock);
                auto &slot = g_host_slots[slot_index];
                if (slot.state == BleDeviceSlot::State::Vacant) {
                    return;
                }

                already_disconnect_pending = slot.state == BleDeviceSlot::State::Disconnecting;
                if (!already_disconnect_pending) {
                    slot.state = BleDeviceSlot::State::Disconnecting;
                }
                conn_id = slot.conn_id;
                address = slot.address;

                if (!already_disconnect_pending) {
                    events::UnregisterSlotController(slot_index);
                    controller = std::move(slot.controller);
                    slot.controller = nullptr;
                    slot.negotiated_max_interval = 0;
                }
            }

            if (controller) {
                // Ensure any in-flight HandleBleInputReport / HandleOutputDataReport on another core has finished
                controller->DisconnectBle();
            }

            // Cancel the client request and remove notification registrations before
            // recycling the slot. Keep event ownership until the disconnect completes.
            if (close_transport && !already_disconnect_pending && events::IsRadioEnabled()) {
                if (conn_id != InvalidConnId) {
                    gatt::Unsubscribe(slot_index, conn_id);
                    if (!events::IsSlotDisconnected(slot_index)) {
                        Result rc = btdrvDisconnectGattServer(conn_id);
                        log::Write("btdrvDisconnectGattServer slot=%u conn_id=%u rc=0x%x", slot_index, conn_id, rc.GetValue());
                        events::WaitFor(events::InvalidType, TimeSpan::FromSeconds(2), nullptr, [&](const EventRecord &) {
                            return events::IsSlotDisconnected(slot_index) || !events::IsRadioEnabled();
                        });
                    }
                }
                const u8 client_if = g_host_slots[slot_index].client_if;
                if (client_if != InvalidClientIf) {
                    static_cast<void>(btdrvCancelConnectGattServer(client_if, address, true));
                }
            }

            if (close_transport && events::IsRadioEnabled() && events::IsSlotDisconnected(slot_index)) {
                const bool preserve_stadia_ltk =
                    (g_host_slots[slot_index].controller_type == BleControllerType::Stadia) &&
                    g_host_slots[slot_index].was_connected;
                if (preserve_stadia_ltk) {
                    log::Write("slot %u: preserving Stadia BLE bond record after connected session disconnect", slot_index);
                } else {
                    const Result rc = pairing::ClearPeer(slot_index, address);
                    if (R_FAILED(rc)) {
                        log::Write("slot %u: failed to clear disconnected peer security state rc=0x%x", slot_index, rc.GetValue());
                    }
                }
            }

            if (controller) {
                profile::FlushNow();
                controller::DetachVirtualController(address);
                log::Write("slot %u: controller removed from system", slot_index);
            }

            if (!events::ClearSlot(slot_index, close_transport)) {
                const u32 current_conn_id = events::GetSlotConnectionId(slot_index);
                const u32 retained_conn_id = current_conn_id != InvalidConnId ? current_conn_id : conn_id;
                {
                    std::scoped_lock lk(g_slots_lock);
                    auto &slot = g_host_slots[slot_index];
                    if (slot.state == BleDeviceSlot::State::Disconnecting) {
                        slot.conn_id = retained_conn_id;
                    }
                }
                if (!already_disconnect_pending) {
                    log::Write("slot %u: retaining disconnecting BLE link conn_id=%u until teardown is confirmed", slot_index, retained_conn_id);
                }
                return;
            }

            gatt::ResetSlot(slot_index);
            {
                std::scoped_lock lk(g_slots_lock);
                g_host_slots[slot_index] = {};
            }

        }

        void ProcessPendingDisconnections() {
            bool any_disconnected = false;
            for (size_t i = 0; i < MaxBleControllers; ++i) {
                const u32 slot_idx = static_cast<u32>(i);
                if (g_host_slots[i].state == BleDeviceSlot::State::Disconnecting) {
                    if (events::IsSlotDisconnected(slot_idx)) {
                        log::Write("slot %u: disconnecting BLE link ended (reason=0x%04x)", slot_idx, events::GetSlotDisconnectReason(slot_idx));
                        DismissSlot(slot_idx);
                        any_disconnected = true;
                    }
                } else if (g_host_slots[i].state == BleDeviceSlot::State::Connected) {
                    if (events::IsSlotDisconnected(slot_idx)) {
                        log::Write("slot %u: controller disconnected (reason=0x%04x)", slot_idx, events::GetSlotDisconnectReason(slot_idx));
                        DismissSlot(slot_idx);
                        any_disconnected = true;
                    } else if (gatt::TakeRediscovered(slot_idx)) {
                        log::Write("slot %u: services re-discovered, subscribing again", slot_idx);
                        gatt::HidService hid;
                        const auto auth_mode = (g_host_slots[i].controller_type == BleControllerType::Stadia)
                            ? ((g_host_slots[i].hid.write_auth_req == BtdrvGattAuthReqType_NoMitm)
                                ? gatt::SubscribeAuthMode::NoMitmCccdOnly
                                : gatt::SubscribeAuthMode::PlainOnlyAfterBond)
                            : gatt::SubscribeAuthMode::StandardNoMitm;
                        static_cast<void>(gatt::Subscribe(slot_idx, g_host_slots[i].conn_id, &hid, auth_mode));
                    }
                }
            }

            const bool count_changed = events::ConsumeControllerCountChanged();
            if (any_disconnected || count_changed) {
                RebalanceConnectionParameters();
            }
        }

        void ScanAndPairNextController(bool background_reconnect = false) {
            if (events::ConsumeControllerCountChanged()) {
                RebalanceConnectionParameters();
            }

            int slot_idx = FindFreeSlot();
            if (slot_idx < 0) {
                return;
            }
            u32 slot = static_cast<u32>(slot_idx);

            bluetooth::Address address;
            u32 conn_id = InvalidConnId;
            u32 session_generation = events::GetClientRegistrationGeneration();
            BleControllerType scanned_type = BleControllerType::Unknown;
            u8 scanned_flags = 0;
            bool connected = Adopt(&address, &conn_id);

            if (connected) {
                scanned_type = RecallControllerType(address);
            } else {
                const bool another_controller_connected = (events::GetActiveConnectionCount() > 0) || (controller::GetConnectedControllerCount() > 0);
                const TimeSpan scan_duration = background_reconnect
                    ? TimeSpan::FromMilliSeconds(another_controller_connected ? 800 : 1000)
                    : TimeSpan::FromSeconds(another_controller_connected ? 2 : 5);
                u32 stale_slot = InvalidSlot;
                Result rc = Scan(&address, &scanned_type, &scanned_flags, scan_duration, background_reconnect, &stale_slot);
                if (events::ConsumeControllerCountChanged()) {
                    RebalanceConnectionParameters();
                }
                if (stale_slot != InvalidSlot) {
                    log::Write("slot %u: Stadia re-advertised in pairing mode (flags=0x05); dismissing stale slot immediately", stale_slot);
                    DismissSlot(stale_slot);
                    return;
                }
                if (Adopt(&address, &conn_id)) {
                    connected = true;
                    scanned_type = RecallControllerType(address);
                } else if (R_SUCCEEDED(rc)) {
                    if (IsAddressConnected(address, slot)) {
                        return;
                    }
                    slot = FindSlotForAddress(address, slot);
                    g_host_slots[slot].state = BleDeviceSlot::State::Connecting;
                    g_host_slots[slot].controller_type = scanned_type;
                    g_host_slots[slot].was_connected = false;
                    g_host_slots[slot].address = address;
                    const u8 client_if = events::GetClientIfForSlot(slot);
                    g_host_slots[slot].client_if = client_if;
                    session_generation = events::GetClientRegistrationGeneration();
                    // Preserve bt_nso's in-RAM LTK for Stadia general-discoverable reconnect (flags=0x06)
                    // when not retrying a failure; clear old bond records for pairing mode (flags=0x05) or retries.
                    const bool preserve_bond =
                        (scanned_type == BleControllerType::Stadia) &&
                        ((scanned_flags & 0x01) == 0) &&
                        (FindOrAllocStadiaStrategy(address)->failures_in_cycle == 0);
                    connected = R_SUCCEEDED(pairing::Connect(slot, client_if, address, &conn_id, TimeSpan::FromSeconds(15), !preserve_bond));
                    if (!connected) {
                        NotePairingFailure(address, background_reconnect ? BackgroundReconnectBackoffMs : PairingRetryBackoffMs);
                        DismissSlot(slot);
                        return;
                    }
                } else {
                    // With another controller connected, pair a two-second scan with this two-second pause.
                    if (!background_reconnect && svc::ResultTimedOut::Includes(rc) && events::IsPairingWindowOpen() && another_controller_connected) {
                        events::Pump(TimeSpan::FromSeconds(2));
                    }
                    return;
                }
            }

            if (IsAddressConnected(address, slot)) {
                DismissSlot(slot);
                return;
            }

            if (g_host_slots[slot].state == BleDeviceSlot::State::Vacant) slot = FindSlotForAddress(address, slot);
            g_host_slots[slot].conn_id = conn_id;
            g_host_slots[slot].controller_type = scanned_type;
            g_host_slots[slot].address = address;
            g_host_slots[slot].state = BleDeviceSlot::State::Discovering;
            if (!events::IsAddressActive(address)) {
                events::SetSlotConnecting(slot, address);
                events::SetSlotConnection(slot, address, conn_id);
            }
            if (events::IsSlotDisconnected(slot)) {
                DismissSlot(slot);
                return;
            }
            events::SetSlotClaimed(slot, true);

            Result rc = gatt::Discover(slot, conn_id, address);
            if (R_FAILED(rc)) {
                log::Write("slot %u: GATT setup failed before pairing rc=0x%x", slot, rc.GetValue());
                NotePairingFailure(address);
                DismissSlot(slot);
                return;
            }

            g_host_slots[slot].state = BleDeviceSlot::State::Pairing;
            if (session_generation != events::GetClientRegistrationGeneration()) {
                log::Write("slot %u: native clients changed during setup; discarding old session", slot);
                DismissSlot(slot, false);
                return;
            }

            const bool is_stadia = (scanned_type == BleControllerType::Stadia) || (RecallControllerType(address) == BleControllerType::Stadia);
            u8 stadia_stage = 0;
            pairing::SecurityStrategy sec_strategy = pairing::SecurityStrategy::CreateBond;
            gatt::SubscribeAuthMode sub_mode = gatt::SubscribeAuthMode::StandardNoMitm;
            if (is_stadia) {
                stadia_stage = GetStadiaStrategyStage(address);
                switch (stadia_stage) {
                    case 0:
                        // Primary: grant Stadia's proactive SMP Slave Security Request via IPS cmd 6
                        // (BTA_DmBleSecurityGrant(addr, 0)) so SMP executes Legacy Just Works pairing +
                        // encryption instead of stalling in SMP_ST_WAIT_APP_DECISION until 30.15s timeout,
                        // then write CCCD with AuthReq=None on the encrypted link.
                        sec_strategy = pairing::SecurityStrategy::BleSecurityGrant;
                        sub_mode = gatt::SubscribeAuthMode::PlainOnlyAfterBond;
                        break;
                    case 1:
                        // Fallback 1: grant any pending Slave Security Request and trigger GATT NoMitm encryption.
                        sec_strategy = pairing::SecurityStrategy::BleSecurityGrant;
                        sub_mode = gatt::SubscribeAuthMode::NoMitmCccdOnly;
                        break;
                    default:
                        // Fallback 2: grant pending request + IPS cmd 5 BTA_DmSetEncryption + AuthReq=None CCCD.
                        sec_strategy = pairing::SecurityStrategy::DirectSetEncryption;
                        sub_mode = gatt::SubscribeAuthMode::PlainOnlyAfterBond;
                        break;
                }
                log::Write("slot %u: Stadia security plan stage=%u (sec=%u sub=%u flags=0x%02x)",
                    slot, stadia_stage, static_cast<u32>(sec_strategy), static_cast<u32>(sub_mode), scanned_flags);
            }

            const auto handle_attempt_failure = [&](const char *phase) {
                if (is_stadia) {
                    if (background_reconnect) {
                        NotePairingFailure(address, BackgroundReconnectBackoffMs);
                        log::Write("slot %u: Stadia bg reconnect %s failed (flags=0x%02x backoff=%ldms)",
                            slot, phase, scanned_flags, static_cast<long>(BackgroundReconnectBackoffMs));
                    } else {
                        const bool cycle_exhausted = AdvanceStadiaStrategy(address);
                        const s64 backoff = cycle_exhausted ? PairingRetryBackoffMs : StadiaRetryBackoffMs;
                        if (cycle_exhausted) {
                            g_radio_recovery.Request();
                        }
                        NotePairingFailure(address, backoff);
                        log::Write("slot %u: Stadia %s failed (stage=%u next=%u cycle_exhausted=%u backoff=%ldms)",
                            slot, phase, stadia_stage, GetStadiaStrategyStage(address), cycle_exhausted, static_cast<long>(backoff));
                    }
                } else {
                    g_radio_recovery.Request();
                    NotePairingFailure(address, PairingRetryBackoffMs);
                    log::Write("slot %u: backing off for %ldms before trying to pair again", slot, static_cast<long>(PairingRetryBackoffMs));
                }
                if (events::IsRadioEnabled() && (sec_strategy == pairing::SecurityStrategy::CreateBond)) {
                    const Result cancel_rc = btdrvCancelBond(address);
                    log::Write("btdrvCancelBond after %s failure slot=%u rc=0x%x", phase, slot, cancel_rc.GetValue());
                }
                DismissSlot(slot);
                os::SleepThread(TimeSpan::FromMilliSeconds(is_stadia ? 500 : 1000));
            };

            if (!pairing::PrepareSecurityWithStrategy(slot, address, sec_strategy)) {
                handle_attempt_failure("security");
                return;
            }

            rc = ResultSuccess();
            const u32 generation = events::GetClientRegistrationGeneration();
            if (g_hid_data_path_generation != generation) {
                auto hid_uuid = MakeUuid16(GattServiceHid);
                rc = btdrvRegisterGattHidDataPath(&hid_uuid);
                log::Write("btdrvRegisterGattHidDataPath slot=%u rc=0x%x", slot, rc.GetValue());
                if (R_FAILED(rc)) {
                    DismissSlot(slot);
                    return;
                }
                g_hid_data_path_generation = generation;
            }
            events::ClearGattFailure(slot);
            gatt::HidService hid = {};
            rc = gatt::Subscribe(slot, conn_id, &hid, sub_mode);
            if (R_FAILED(rc)) {
                log::Write("slot %u: subscription failed rc=0x%x gatt_result=%u", slot, rc.GetValue(), events::GetGattFailure(slot));
                handle_attempt_failure("subscription");
                return;
            }

            if (session_generation != events::GetClientRegistrationGeneration()) {
                DismissSlot(slot, false);
                return;
            }
            if (is_stadia) {
                NoteStadiaStrategySuccess(address, stadia_stage);
            }
            g_host_slots[slot].hid = hid;
            log::Write("slot %u: paired and subscribed (write_auth=%u)", slot, hid.write_auth_req);
            PresentSlot(slot);
        }

        bool RecoverRadioIfNeeded() {
            static size_t reported_active_links = 0;
            const size_t active_links = events::GetActiveConnectionCount();
            const bool deferred = g_radio_recovery.Pending() && mitm::GetGlobalConfig()->ble.radio_cycle &&
                events::IsPairingWindowOpen() && events::IsRadioEnabled() && active_links != 0;
            if (deferred && active_links != reported_active_links) {
                log::Write("BLE pairing recovery deferred while %zu BLE links remain active", active_links);
            }
            reported_active_links = deferred ? active_links : 0;
            if (!g_radio_recovery.WaitingForClients()) {
                if (!g_radio_recovery.Begin(mitm::GetGlobalConfig()->ble.radio_cycle,
                        events::IsPairingWindowOpen(), events::IsRadioEnabled(), events::GetActiveConnectionCount(),
                        os::GetSystemTick().ToTimeSpan().GetMilliSeconds())) {
                    // A wake inside the reset cooldown must not spend the only
                    // zero-link recovery window connecting a pad first.
                    if (mitm::GetGlobalConfig()->ble.radio_cycle && g_radio_recovery.Pending() &&
                            events::IsPairingWindowOpen() && events::IsRadioEnabled() && events::GetActiveConnectionCount() == 0) {
                        events::Pump(TimeSpan::FromMilliSeconds(100));
                        return false;
                    }
                    return true;
                }

                // No active BLE link is allowed through the recovery gate.
                log::Write("BLE pairing recovery: no active BLE links, resetting stack");

                const u32 generation = events::GetClientRegistrationGeneration();
                Result rc = btdrvEmulateBluetoothCrash(BtdrvFatalReason_Unknown1);
                log::Write("BLE pairing recovery: radio cycle rc=0x%x", rc.GetValue());
                if (R_FAILED(rc)) return true;
                g_radio_recovery.AwaitClients(generation);
                g_hid_data_path_generation = UINT32_MAX;
            }

            // A timeout must not resume scanning with stale client handles. Keep
            // waiting on later coordinator passes without issuing another reset.
            events::WaitFor(events::InvalidType, TimeSpan::FromSeconds(8), nullptr, [&](const EventRecord &) {
                return !events::IsRadioEnabled() || g_radio_recovery.AcceptClients(
                    events::GetClientRegistrationGeneration(), events::GetAvailableClientIfCount());
            });
            if (g_radio_recovery.WaitingForClients() || !events::IsRadioEnabled()) {
                log::Write("BLE pairing recovery: still waiting for fresh GATT clients");
                return false;
            }

            for (auto &failure : g_pairing_failures) failure = {};
            log::Write("BLE pairing recovery: new GATT clients available; hold Xbox sync buttons to pair again");
            // Give Classic controllers time to reconnect before BLE scanning
            // competes with their connection setup on the shared stack.
            events::Pump(TimeSpan::FromSeconds(12));
            return true;
        }

        /* Coordinator loop */

        void HostThreadFunc(void *) {
            mitm::WaitInitialized();
            bluetooth::ble::WaitInitialized();

            os::SleepThread(TimeSpan::FromSeconds(20));
            log::Write("starting multi-device BLE host (stadia-ble-v7-hos2250, HOS=0x%x)", static_cast<u32>(hos::GetVersion()));
            LoadKnownStadia();

            if (R_FAILED(PrepareClient())) {
                log::Write("giving up: no gatt client available");
                return;
            }

            StartLeHidListener();
            events::SetSink(gatt::OnEvent);
            u32 pairing_window_generation = events::PairingWindowGeneration();
            s64 last_bg_scan_ms = 0;

            for (;;) {
                WaitForRadio();
                ProcessPendingDisconnections();
                const u32 current_pairing_window_generation = events::PairingWindowGeneration();
                if (current_pairing_window_generation != pairing_window_generation) {
                    pairing_window_generation = current_pairing_window_generation;
                    std::fill(std::begin(g_pairing_failures), std::end(g_pairing_failures), PairingFailure{});
                    for (auto &strategy : g_stadia_strategies) {
                        strategy.failures_in_cycle = 0;
                    }
                    log::Write("BLE pairing window opened; ready to search (free slot=%s)", FindFreeSlot() >= 0 ? "yes" : "no");
                }
                profile::FlushIfDue();
                if (!RecoverRadioIfNeeded()) continue;

                if ((events::IsPairingWindowOpen() || events::HasAdoptedConnection()) && events::IsRadioEnabled() && (FindFreeSlot() >= 0)) {
                    ScanAndPairNextController(false);
                } else if (events::IsRadioEnabled() && (FindFreeSlot() >= 0) && HasDisconnectedKnownStadia()) {
                    const s64 now_ms = os::GetSystemTick().ToTimeSpan().GetMilliSeconds();
                    const bool other_pads = (events::GetActiveConnectionCount() > 0) || (controller::GetConnectedControllerCount() > 0);
                    const s64 period_ms = other_pads ? BackgroundScanBusyPeriodMs : BackgroundScanIdlePeriodMs;
                    if ((last_bg_scan_ms == 0) || ((now_ms - last_bg_scan_ms) >= period_ms)) {
                        last_bg_scan_ms = now_ms;
                        ScanAndPairNextController(true);
                    } else {
                        events::Pump(TimeSpan::FromMilliSeconds(100));
                    }
                } else {
                    events::Pump(TimeSpan::FromMilliSeconds(100));
                }
            }
        }

    }

    bool IsEnabled() {
        return mitm::GetGlobalConfig()->ble.enable && (hos::GetVersion() >= hos::Version_13_0_0);
    }

    void Launch() {
        if (!IsEnabled()) {
            return;
        }

        auto config = mitm::GetGlobalConfig();
        if (config->ble.log) {
            log::Initialize();
        }
        if (config->ble.profile) {
            profile::Initialize();
        }
        events::Activate();

        R_ABORT_UNLESS(os::CreateThread(&g_thread,
            HostThreadFunc,
            nullptr,
            g_thread_stack,
            ThreadStackSize,
            ThreadPriority
        ));

        os::SetThreadNamePointer(&g_thread, "mc::BleHidHostThread");
        os::StartThread(&g_thread);

        g_launched = true;
    }

    void WaitFinished() {
        if (g_launched) {
            os::WaitThread(&g_thread);
        }
    }

}
