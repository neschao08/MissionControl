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
#include "btm_mitm_service.hpp"
#include "btm_shim.h"
#include "btm_device_condition.hpp"
#include "btm_device_info.hpp"
#include "../ble/ble_hid_host.hpp"
#include "../ble/ble_log.hpp"
#include "../controllers/controller_management.hpp"
#include <atomic>

namespace ams::mitm::btm {

    namespace {

        std::atomic<u32> g_btm_diag_log_count{0};

        bool TryTakeBtmLogSlot(u32 *out_idx) {
            if (!ble::log::IsEnabled()) {
                return false;
            }
            u32 cur = g_btm_diag_log_count.load(std::memory_order_relaxed);
            while (cur < 16) {
                if (g_btm_diag_log_count.compare_exchange_weak(cur, cur + 1, std::memory_order_relaxed)) {
                    *out_idx = cur + 1;
                    return true;
                }
            }
            return false;
        }

        void RenameConnectedDevices(BtmDeviceConditionLegacy *devices, size_t count) {
            for (unsigned int i = 0; i < count; ++i) {
                auto device = &devices[i];
                if ((device->profile == BtmProfile_Hid) && !controller::IsOfficialSwitchControllerName(device->name)) {
                    std::strncpy(device->name, controller::LicensedProControllerName, sizeof(device->name) - 1);
                }
            }
        }

        template <typename T>
        void AppendVirtualDevices(T *device_condition, size_t max_size, sf::Out<s32> &total_out) {
            bluetooth::Address virtual_devices[4];
            const size_t virtual_count = controller::GetVirtualDevices(virtual_devices, util::size(virtual_devices));

            s32 total = total_out.GetValue();
            const s32 existing_count = total;
            for (size_t i = 0; (i < virtual_count) && (static_cast<size_t>(total) < max_size); ++i) {
                bool already_present = false;
                for (s32 j = 0; j < existing_count; ++j) {
                    if (std::memcmp(&device_condition[j].address, &virtual_devices[i], sizeof(bluetooth::Address)) == 0) {
                        already_present = true;
                        break;
                    }
                }
                if (already_present) {
                    continue;
                }

                auto device = &device_condition[total++];
                std::memset(device, 0, sizeof(*device));
                device->address = virtual_devices[i];
                device->profile = BtmProfile_Hid;
                device->profile_condition.hid.sniff_mode = BtmSniffMode_15ms;
                device->profile_condition.hid.slot_mode  = BtmSlotMode_2;
                device->profile_condition.hid.vid        = 0x057e;
                device->profile_condition.hid.pid        = 0x2009;
                std::strncpy(device->name, controller::ProControllerName, sizeof(device->name) - 1);
            }
            total_out.SetValue(total);
        }

        template <typename T>
        void AppendVirtualDeviceInfos(T *device_info, size_t max_size, sf::Out<s32> &total_out) {
            bluetooth::Address known_devices[4];
            const size_t known_count = ble::GetKnownStadiaAddresses(known_devices, util::size(known_devices));

            bluetooth::Address active_devices[4];
            const size_t active_count = controller::GetVirtualDevices(active_devices, util::size(active_devices));

            bluetooth::Address combined_devices[8];
            size_t combined_count = 0;
            const auto add_unique = [&](const bluetooth::Address &addr) {
                for (size_t k = 0; k < combined_count; ++k) {
                    if (std::memcmp(&combined_devices[k], &addr, sizeof(bluetooth::Address)) == 0) {
                        return;
                    }
                }
                if (combined_count < util::size(combined_devices)) {
                    combined_devices[combined_count++] = addr;
                }
            };
            for (size_t i = 0; i < known_count; ++i) {
                add_unique(known_devices[i]);
            }
            for (size_t i = 0; i < active_count; ++i) {
                add_unique(active_devices[i]);
            }

            s32 total = total_out.GetValue();
            const s32 existing_count = total;
            for (size_t i = 0; (i < combined_count) && (static_cast<size_t>(total) < max_size); ++i) {
                bool already_present = false;
                for (s32 j = 0; j < existing_count; ++j) {
                    if (std::memcmp(&device_info[j].addr, &combined_devices[i], sizeof(bluetooth::Address)) == 0) {
                        already_present = true;
                        break;
                    }
                }
                if (already_present) {
                    continue;
                }

                auto device = &device_info[total++];
                std::memset(device, 0, sizeof(*device));
                device->addr = combined_devices[i];
                device->class_of_device = { 0x00, 0x25, 0x08 };
                std::memset(device->link_key.link_key, 0xAA, sizeof(device->link_key.link_key));
                device->key_type = 4;
                device->profile = BtmProfile_Hid;
                device->profile_info.hid.vid = 0x057e;
                device->profile_info.hid.pid = 0x2009;
                std::strncpy(device->name, controller::ProControllerName, sizeof(device->name) - 1);
            }
            total_out.SetValue(total);
        }

    }

    Result BtmMitmService::AcquireDeviceConditionEvent(sf::OutCopyHandle out_handle) {
        if (m_client_info.program_id != ncm::SystemProgramId::Hid) {
            os::NativeHandle handle = os::InvalidNativeHandle;
            R_TRY(btmAcquireDeviceConditionEventFwd(m_forward_service.get(), &handle));
            out_handle.SetValue(handle, true);
            R_SUCCEED();
        }

        if (!condition::IsInitialized()) {
            // Forward to the real function to obtain the system event handle
            os::NativeHandle handle = os::InvalidNativeHandle;
            R_TRY(btmAcquireDeviceConditionEventFwd(m_forward_service.get(), &handle));

            // Attach the real system event handle to our own event
            condition::GetSystemEvent()->AttachReadableHandle(handle, false, os::EventClearMode_AutoClear);

            // Return our forwarder event handle to the caller instead
            out_handle.SetValue(condition::GetForwardEvent()->GetReadableHandle(), false);

            condition::SignalInitialized();
        } else {
            // hid gets the event it already has rather than a second one nothing would signal
            out_handle.SetValue(condition::GetForwardEvent()->GetReadableHandle(), false);
        }

        R_SUCCEED();
    }

    Result BtmMitmService::AcquireDeviceInfoEvent(sf::OutCopyHandle out_handle) {
        if (u32 log_idx = 0; TryTakeBtmLogSlot(&log_idx)) {
            ble::log::Write("btm diag #%u AcquireDeviceInfoEvent pid=0x%016llx",
                log_idx, static_cast<unsigned long long>(m_client_info.program_id.value));
        }

        if (m_client_info.program_id != ncm::SystemProgramId::Hid) {
            os::NativeHandle handle = os::InvalidNativeHandle;
            R_TRY(btmAcquireDeviceInfoEventFwd(m_forward_service.get(), &handle));
            out_handle.SetValue(handle, true);
            R_SUCCEED();
        }

        if (!info::IsInitialized()) {
            os::NativeHandle handle = os::InvalidNativeHandle;
            R_TRY(btmAcquireDeviceInfoEventFwd(m_forward_service.get(), &handle));

            info::GetSystemEvent()->AttachReadableHandle(handle, false, os::EventClearMode_AutoClear);
            out_handle.SetValue(info::GetForwardEvent()->GetReadableHandle(), false);

            info::SignalInitialized();
        } else {
            out_handle.SetValue(info::GetForwardEvent()->GetReadableHandle(), false);
        }

        R_SUCCEED();
    }

    Result BtmMitmService::GetDeviceCondition(ams::btm::Profile profile, const sf::OutArray<ams::btm::DeviceCondition> &out, sf::Out<s32> total_out) {
        auto device_condition = reinterpret_cast<BtmDeviceCondition *>(out.GetPointer());
        R_TRY(btmGetDeviceConditionFwd(m_forward_service.get(), profile, device_condition, out.GetSize(), total_out.GetPointer()));

        for (int i = 0; i < total_out.GetValue(); ++i) {
            auto device = &device_condition[i];
            if ((device->profile == BtmProfile_Hid) && !controller::IsOfficialSwitchControllerName(device->name)) {
                std::strncpy(device->name, controller::LicensedProControllerName, sizeof(device->name) - 1);
            }
        }

        // Append controllers btm doesn't know about (Bluetooth LE).
        if ((profile == BtmProfile_Hid) || (profile == BtmProfile_None)) {
            AppendVirtualDevices(device_condition, out.GetSize(), total_out);
        }

        R_SUCCEED();
    }

    Result BtmMitmService::GetDeviceInfo(ams::btm::Profile profile, const sf::OutArray<ams::btm::DeviceInfo> &out, sf::Out<s32> total_out) {
        auto device_info = reinterpret_cast<BtmDeviceInfo *>(out.GetPointer());
        R_TRY(btmGetDeviceInfoFwd(m_forward_service.get(), profile, device_info, out.GetSize(), total_out.GetPointer()));
        const s32 raw_total = total_out.GetValue();

        for (int i = 0; i < total_out.GetValue(); ++i) {
            auto device = &device_info[i];
            if ((device->profile == BtmProfile_Hid) && !controller::IsOfficialSwitchControllerName(device->name)) {
                std::strncpy(device->name, controller::LicensedProControllerName, sizeof(device->name) - 1);
            }
        }

        if ((profile == BtmProfile_Hid) || (profile == BtmProfile_None)) {
            AppendVirtualDeviceInfos(device_info, out.GetSize(), total_out);
        }

        if (u32 log_idx = 0; TryTakeBtmLogSlot(&log_idx)) {
            ble::log::Write("btm diag #%u GetDeviceInfo pid=0x%016llx profile=%d raw=%d final=%d",
                log_idx, static_cast<unsigned long long>(m_client_info.program_id.value),
                static_cast<int>(profile), raw_total, total_out.GetValue());
        }

        R_SUCCEED();
    }

    Result BtmMitmService::GetShortenedDeviceInfo(ams::btm::Profile profile, const sf::OutArray<ams::btm::ShortenedDeviceInfo> &out, sf::Out<s32> total_out) {
        auto device_info = reinterpret_cast<BtmShortenedDeviceInfo *>(out.GetPointer());
        R_TRY(btmGetShortenedDeviceInfoFwd(m_forward_service.get(), profile, device_info, out.GetSize(), total_out.GetPointer()));
        const s32 raw_total = total_out.GetValue();

        for (int i = 0; i < total_out.GetValue(); ++i) {
            auto device = &device_info[i];
            if ((device->profile == BtmProfile_Hid) && !controller::IsOfficialSwitchControllerName(device->name)) {
                std::strncpy(device->name, controller::LicensedProControllerName, sizeof(device->name) - 1);
            }
        }

        if ((profile == BtmProfile_Hid) || (profile == BtmProfile_None)) {
            AppendVirtualDeviceInfos(device_info, out.GetSize(), total_out);
        }

        if (u32 log_idx = 0; TryTakeBtmLogSlot(&log_idx)) {
            ble::log::Write("btm diag #%u GetShortenedDeviceInfo pid=0x%016llx profile=%d raw=%d final=%d",
                log_idx, static_cast<unsigned long long>(m_client_info.program_id.value),
                static_cast<int>(profile), raw_total, total_out.GetValue());
        }

        R_SUCCEED();
    }

    Result BtmMitmService::GetShortenedDeviceCondition(ams::btm::Profile profile, const sf::OutArray<ams::btm::ShortenedDeviceCondition> &out, sf::Out<s32> total_out) {
        auto device_condition = reinterpret_cast<BtmShortenedDeviceCondition *>(out.GetPointer());
        R_TRY(btmGetShortenedDeviceConditionFwd(m_forward_service.get(), profile, device_condition, out.GetSize(), total_out.GetPointer()));

        for (int i = 0; i < total_out.GetValue(); ++i) {
            auto device = &device_condition[i];
            if ((device->profile == BtmProfile_Hid) && !controller::IsOfficialSwitchControllerName(device->name)) {
                std::strncpy(device->name, controller::LicensedProControllerName, sizeof(device->name) - 1);
            }
        }

        if ((profile == BtmProfile_Hid) || (profile == BtmProfile_None)) {
            AppendVirtualDevices(device_condition, out.GetSize(), total_out);
        }

        R_SUCCEED();
    }

    /* Deprecated */

    /* 1.0.0 - 5.0.2 */
    Result BtmMitmService::GetDeviceConditionDeprecated1(sf::Out<ams::btm::DeviceConditionListV100> out) {
        auto device_condition_list = reinterpret_cast<BtmDeviceConditionListV100 *>(out.GetPointer());
        R_TRY(btmGetDeviceConditionDeprecated1Fwd(m_forward_service.get(), device_condition_list));
        RenameConnectedDevices(device_condition_list->devices, device_condition_list->device_count);

        R_SUCCEED();
    }

    /* 5.1.0 - 7.0.1 */
    Result BtmMitmService::GetDeviceConditionDeprecated2(sf::Out<ams::btm::DeviceConditionListV510> out) {
        auto device_condition_list = reinterpret_cast<BtmDeviceConditionListV510 *>(out.GetPointer());
        R_TRY(btmGetDeviceConditionDeprecated2Fwd(m_forward_service.get(), device_condition_list));
        RenameConnectedDevices(device_condition_list->devices, device_condition_list->device_count);

        R_SUCCEED();
    }

    /* 8.0.0 - 8.1.1 */
    Result BtmMitmService::GetDeviceConditionDeprecated3(sf::Out<ams::btm::DeviceConditionListV800> out) {
        auto device_condition_list = reinterpret_cast<BtmDeviceConditionListV800 *>(out.GetPointer());
        R_TRY(btmGetDeviceConditionDeprecated3Fwd(m_forward_service.get(), device_condition_list));
        RenameConnectedDevices(device_condition_list->devices, device_condition_list->device_count);

        R_SUCCEED();
    }

    /* 9.0.0 - 12.1.0 */
    Result BtmMitmService::GetDeviceConditionDeprecated4(sf::Out<ams::btm::DeviceConditionListV900> out) {
        auto device_condition_list = reinterpret_cast<BtmDeviceConditionListV900 *>(out.GetPointer());
        R_TRY(btmGetDeviceConditionDeprecated4Fwd(m_forward_service.get(), device_condition_list));
        RenameConnectedDevices(device_condition_list->devices, device_condition_list->device_count);

        R_SUCCEED();
    }

    /* 1.0.0 - 12.1.0 */
    Result BtmMitmService::GetDeviceInfoDeprecated(sf::Out<ams::btm::DeviceInfoList> out) {
        auto device_info_list = reinterpret_cast<BtmDeviceInfoList *>(out.GetPointer());
        R_TRY(btmGetDeviceInfoDeprecatedFwd(m_forward_service.get(), device_info_list));

        for (unsigned int i = 0; i < device_info_list->device_count; ++i) {
            auto device = &device_info_list->devices[i];
            if ((device->profile == BtmProfile_Hid) && !controller::IsOfficialSwitchControllerName(device->name.name)) {
                std::strncpy(device->name.name, controller::LicensedProControllerName, sizeof(device->name) - 1);
            }
        }

        R_SUCCEED();
    }

}
