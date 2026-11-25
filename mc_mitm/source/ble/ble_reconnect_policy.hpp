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
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace ams::ble::reconnect {

    inline constexpr int64_t DirectProbeIdlePeriodMs          = 2000;
    inline constexpr int64_t DirectProbeBusyPeriodMs          = 4500;
    inline constexpr int64_t OnDemandScanIdlePeriodMs         = 8000;
    inline constexpr int64_t OnDemandScanBusyPeriodMs         = 25000;
    inline constexpr int64_t OnDemandScanRetryPeriodMs        = 2000;
    inline constexpr int64_t StadiaScanDurationMs             = 4500;
    inline constexpr int64_t ForegroundIdleScanDurationMs     = 5000;
    inline constexpr int64_t TransientDisconnectBackoffMs     = 1000;
    inline constexpr uint8_t ClearBondAuthFailureThreshold    = 2;

    enum class BgAction : uint8_t {
        None,
        DirectProbe,
        BackgroundScan,
    };

    constexpr int64_t ForegroundScanDurationMs(bool other_pads) {
        return other_pads ? StadiaScanDurationMs : ForegroundIdleScanDurationMs;
    }

    constexpr bool ShouldArmSleepRecovery(bool has_non_stadia_links) {
        return has_non_stadia_links;
    }

    constexpr bool ShouldPreserveStadiaBondOnDismiss() {
        return true;
    }

    /**
     * Tracks per-device BLE reconnect state for remembered Stadia controllers,
     * coordinating direct probes, on-demand background scans after cache resets,
     * and conservative bond preservation across transient failures.
     */
    template <size_t MaxDevices = 4>
    class StadiaReconnectTracker {
        static_assert(MaxDevices > 0, "MaxDevices must be positive");

        struct Entry {
            uint8_t addr[6] = {};
            bool valid = false;
            bool needs_scan_resolution = false;
            bool first_scan_pending = false;
            uint8_t auth_failures = 0;
            uint32_t quiet_probe_attempts = 0;
        };

        Entry m_entries[MaxDevices] = {};

        static bool AddrEqual(const uint8_t a[6], const uint8_t b[6]) {
            return std::memcmp(a, b, 6) == 0;
        }

        const Entry *FindEntry(const uint8_t addr[6]) const {
            for (const auto &entry : m_entries) {
                if (entry.valid && AddrEqual(entry.addr, addr)) {
                    return &entry;
                }
            }
            return nullptr;
        }

        Entry *FindEntry(const uint8_t addr[6]) {
            for (auto &entry : m_entries) {
                if (entry.valid && AddrEqual(entry.addr, addr)) {
                    return &entry;
                }
            }
            return nullptr;
        }

        Entry *FindOrAllocEntry(const uint8_t addr[6]) {
            if (auto *existing = FindEntry(addr)) {
                return existing;
            }
            for (auto &entry : m_entries) {
                if (!entry.valid) {
                    entry = {};
                    std::memcpy(entry.addr, addr, 6);
                    entry.valid = true;
                    return &entry;
                }
            }
            for (size_t i = 1; i < MaxDevices; ++i) {
                m_entries[i - 1] = m_entries[i];
            }
            auto &slot = m_entries[MaxDevices - 1];
            slot = {};
            std::memcpy(slot.addr, addr, 6);
            slot.valid = true;
            return &slot;
        }

        public:
            void RegisterKnown(const uint8_t addr[6]) {
                static_cast<void>(FindOrAllocEntry(addr));
            }

            void OnAddressCacheReset() {
                for (auto &entry : m_entries) {
                    if (!entry.valid) {
                        continue;
                    }
                    entry.quiet_probe_attempts = 0;
                    if (entry.needs_scan_resolution) {
                        entry.first_scan_pending = true;
                    }
                }
            }

            bool NeedsScanResolution(const uint8_t addr[6]) const {
                const auto *entry = FindEntry(addr);
                return entry ? entry->needs_scan_resolution : false;
            }

            bool ShouldSilenceProbeLog(const uint8_t addr[6]) const {
                const auto *entry = FindEntry(addr);
                if (!entry) {
                    return false;
                }
                return (entry->quiet_probe_attempts > 0) && !entry->needs_scan_resolution;
            }

            /**
             * Updates probe state after a direct reconnect attempt.
             * Returns true if this peer just transitioned into on-demand background scan mode.
             */
            bool NoteProbeAttemptAndResult(const uint8_t addr[6], bool connected, bool immediate_stack_refusal, bool pairing_window_open) {
                auto *entry = FindOrAllocEntry(addr);
                if (connected) {
                    entry->needs_scan_resolution = false;
                    entry->first_scan_pending = false;
                    ++entry->quiet_probe_attempts;
                    return false;
                }
                if (immediate_stack_refusal && !pairing_window_open) {
                    const bool transitioned = !entry->needs_scan_resolution;
                    entry->needs_scan_resolution = true;
                    entry->first_scan_pending = true;
                    entry->quiet_probe_attempts = 0;
                    return transitioned;
                }
                if (!immediate_stack_refusal) {
                    entry->needs_scan_resolution = false;
                    entry->first_scan_pending = false;
                }
                ++entry->quiet_probe_attempts;
                return false;
            }

            uint32_t TakeQuietProbeCount(const uint8_t addr[6]) {
                auto *entry = FindEntry(addr);
                if (!entry) {
                    return 0;
                }
                const uint32_t count = entry->quiet_probe_attempts;
                entry->quiet_probe_attempts = 0;
                return count;
            }

            void NoteAdvertisementSeen(const uint8_t addr[6]) {
                auto *entry = FindEntry(addr);
                if (!entry) {
                    return;
                }
                entry->needs_scan_resolution = false;
                entry->first_scan_pending = false;
                entry->quiet_probe_attempts = 0;
            }

            bool HasFirstScanPending() const {
                for (const auto &entry : m_entries) {
                    if (entry.valid && entry.first_scan_pending) {
                        return true;
                    }
                }
                return false;
            }

            bool ConsumeFirstScanPending() {
                bool any_pending = false;
                for (auto &entry : m_entries) {
                    if (entry.valid && entry.first_scan_pending) {
                        any_pending = true;
                        entry.first_scan_pending = false;
                    }
                }
                return any_pending;
            }

            static constexpr BgAction ChooseAction(bool has_direct_ready, bool has_scan_needed, bool any_first_scan_pending,
                                                 bool other_pads, int64_t now_ms, int64_t last_probe_ms, int64_t last_scan_ms) {
                const int64_t scan_period_ms = other_pads ? OnDemandScanBusyPeriodMs : OnDemandScanIdlePeriodMs;
                if (has_scan_needed && (any_first_scan_pending || (last_scan_ms == 0) || ((now_ms - last_scan_ms) >= scan_period_ms))) {
                    return BgAction::BackgroundScan;
                }
                const int64_t probe_period_ms = other_pads ? DirectProbeBusyPeriodMs : DirectProbeIdlePeriodMs;
                if (has_direct_ready && ((last_probe_ms == 0) || ((now_ms - last_probe_ms) >= probe_period_ms))) {
                    return BgAction::DirectProbe;
                }
                return BgAction::None;
            }

            uint8_t GetAuthFailures(const uint8_t addr[6]) const {
                const auto *entry = FindEntry(addr);
                return entry ? entry->auth_failures : 0;
            }

            bool ShouldClearBondBeforeConnect(const uint8_t addr[6], uint8_t adv_flags) const {
                if ((adv_flags & 0x01) != 0) {
                    return true;
                }
                return GetAuthFailures(addr) >= ClearBondAuthFailureThreshold;
            }

            void NoteAuthFailure(const uint8_t addr[6]) {
                auto *entry = FindOrAllocEntry(addr);
                if (entry->auth_failures < std::numeric_limits<uint8_t>::max()) {
                    ++entry->auth_failures;
                }
            }

            void NoteConnectSuccess(const uint8_t addr[6]) {
                auto *entry = FindOrAllocEntry(addr);
                entry->auth_failures = 0;
                entry->needs_scan_resolution = false;
                entry->first_scan_pending = false;
                entry->quiet_probe_attempts = 0;
            }
    };

}
