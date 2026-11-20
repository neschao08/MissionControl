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
#include "ble_gatt.hpp"
#include "ble_results.hpp"
#include "ble_events.hpp"
#include "ble_log.hpp"
#include <algorithm>
#include <cstring>

namespace ams::ble::gatt {

    namespace {

        struct SlotGattState {
            AttributeTable table = {};
            u32 conn_id = InvalidConnId;
            bool replace_pending = false;
            bool discovery_complete = false;
            bool rediscovered = false;
        };

        constinit SlotGattState g_slots[MaxBleControllers] = {};

        const BtdrvGattAttribute *FindService(u32 slot_index, u16 uuid16) {
            if (slot_index >= MaxBleControllers) {
                return nullptr;
            }
            auto &tbl = g_slots[slot_index].table;
            for (size_t i = 0; i < tbl.count; ++i) {
                auto &attr = tbl.attributes[i];
                if ((attr.type == BtdrvGattAttributeType_Service) && IsUuid16(attr.id.uuid, uuid16)) {
                    return &attr;
                }
            }
            return nullptr;
        }

        bool IsWithinService(const BtdrvGattAttribute &attr, const BtdrvGattAttribute &service) {
            return (attr.handle > service.handle) && (attr.handle <= service.group_end_handle);
        }

        // Descriptors belong to the characteristic that precedes them by handle
        const BtdrvGattAttribute *OwningCharacteristic(u32 slot_index, const BtdrvGattAttribute &desc) {
            if (slot_index >= MaxBleControllers) {
                return nullptr;
            }
            auto &tbl = g_slots[slot_index].table;
            const BtdrvGattAttribute *best = nullptr;
            for (size_t i = 0; i < tbl.count; ++i) {
                auto &attr = tbl.attributes[i];
                if ((attr.type == BtdrvGattAttributeType_Characteristic) && (attr.handle < desc.handle)) {
                    if (!best || (attr.handle > best->handle)) {
                        best = &attr;
                    }
                }
            }
            return best;
        }

        const BtdrvGattAttribute *FindDescriptor(u32 slot_index, const BtdrvGattAttribute &characteristic, u16 uuid16) {
            if (slot_index >= MaxBleControllers) {
                return nullptr;
            }
            auto &tbl = g_slots[slot_index].table;
            for (size_t i = 0; i < tbl.count; ++i) {
                auto &desc = tbl.attributes[i];
                if ((desc.type == BtdrvGattAttributeType_Descriptor) && IsUuid16(desc.id.uuid, uuid16)) {
                    const auto *owner = OwningCharacteristic(slot_index, desc);
                    if (owner && owner->handle == characteristic.handle) return &desc;
                }
            }
            return nullptr;
        }

        bool HasHidInputReport(u32 slot_index) {
            const auto *service = FindService(slot_index, GattServiceHid);
            if (!service) return false;
            const auto &table = g_slots[slot_index].table;
            for (size_t i = 0; i < table.count; ++i) {
                const auto &attr = table.attributes[i];
                if ((attr.type == BtdrvGattAttributeType_Characteristic) && IsWithinService(attr, *service) &&
                    IsUuid16(attr.id.uuid, GattCharReport) && (attr.property & BtdrvGattCharacteristicProperty_Notify) &&
                    FindDescriptor(slot_index, attr, GattDescClientCharConfig)) return true;
            }
            return false;
        }

        Result WaitForOperation(u32 slot, u32 conn_id, u8 operation, u16 characteristic, TimeSpan timeout, EventRecord *out_event = nullptr, bool *out_timed_out = nullptr) {
            if (out_timed_out) {
                *out_timed_out = false;
            }
            EventRecord event = {};
            const bool completed = events::WaitFor(events::InvalidType, timeout, &event, [&](const EventRecord &record) {
                if (events::IsSlotDisconnected(slot) || !events::IsRadioEnabled()) return true;
                if (record.type != BtdrvBleEventType_ClientNotify) return false;
                const auto &notify = record.info.client_notify;
                if (notify.conn_id != conn_id) return false;
                if ((notify.op_type == operation) && IsUuid16(notify.char_uuid, characteristic)) return true;
                if (operation == BtdrvGattOperationType_DescriptorWrite) {
                    if ((notify.op_type == BtdrvGattOperationType_DescriptorWrite) &&
                        (IsUuid16(notify.char_uuid, GattDescClientCharConfig) || IsUuid16(notify.desc_uuid, GattDescClientCharConfig))) {
                        return true;
                    }
                    if ((notify.op_type == BtdrvGattOperationType_Notification) && IsUuid16(notify.char_uuid, GattCharReport)) {
                        return true;
                    }
                }
                return false;
            });
            R_UNLESS(!events::IsSlotDisconnected(slot) && events::IsRadioEnabled(), ResultNoConnection());
            if (!completed) {
                log::Write("slot %u: GATT completion timeout conn_id=%u op=%u char=0x%04x", slot, conn_id, operation, characteristic);
                if (out_timed_out) {
                    *out_timed_out = true;
                    R_SUCCEED();
                }
                R_THROW(svc::ResultTimedOut());
            }
            // An unrelated notification or a different pad's completion is not
            // evidence that this request (and its encryption) succeeded.
            if (event.info.client_notify.result != 0) {
                log::Write("slot %u: GATT completion failed conn_id=%u op=%u char=0x%04x result=%u", slot, conn_id, operation, characteristic, event.info.client_notify.result);
                R_THROW(svc::ResultInvalidState());
            }
            if (out_event) {
                *out_event = event;
            }
            R_SUCCEED();
        }

        const BtdrvGattAttribute *FindCharacteristic(u32 slot_index, const BtdrvGattAttribute &service, u16 uuid16) {
            if (slot_index >= MaxBleControllers) {
                return nullptr;
            }
            const auto &tbl = g_slots[slot_index].table;
            for (size_t i = 0; i < tbl.count; ++i) {
                const auto &attr = tbl.attributes[i];
                if ((attr.type == BtdrvGattAttributeType_Characteristic) && IsWithinService(attr, service) && IsUuid16(attr.id.uuid, uuid16)) {
                    return &attr;
                }
            }
            return nullptr;
        }

        bool ReadCharacteristicBestEffort(u32 slot_index, u32 conn_id, u16 service_uuid, u16 char_uuid, u8 *out_data, size_t max_size, size_t *out_size) {
            const auto *service = FindService(slot_index, service_uuid);
            if (!service) return false;
            const auto *chr = FindCharacteristic(slot_index, *service, char_uuid);
            if (!chr) return false;

            const Result rc = btdrvReadGattCharacteristic(conn_id, service->is_primary, &service->id, &chr->id, BtdrvGattAuthReqType_NoMitm);
            log::Write("btdrvReadGattCharacteristic(0x%04x) slot=%u handle=0x%04x rc=0x%x", char_uuid, slot_index, chr->handle, rc.GetValue());
            if (R_FAILED(rc)) return false;

            EventRecord event = {};
            if (R_FAILED(WaitForOperation(slot_index, conn_id, BtdrvGattOperationType_CharacteristicRead, char_uuid, TimeSpan::FromSeconds(2), &event))) {
                return false;
            }

            const size_t n = std::min(static_cast<size_t>(event.info.client_notify.size), std::min(sizeof(event.info.client_notify.data), max_size));
            if (n > 0) {
                std::memcpy(out_data, event.info.client_notify.data, n);
            }
            *out_size = n;
            return true;
        }

    }

    void OnEvent(const EventRecord &e) {
        if (e.type != BtdrvBleEventType_ClientCacheSave) {
            return;
        }

        if (e.info.client_cache_save.result != 0) return;
        const u32 conn_id = e.info.client_cache_save.conn_id;
        u32 slot = InvalidSlot;
        for (size_t s = 0; s < MaxBleControllers; ++s) {
            if (g_slots[s].conn_id == conn_id) {
                slot = static_cast<u32>(s);
                break;
            }
        }
        if (slot == InvalidSlot) {
            return;
        }

        auto &s = g_slots[slot];
        if (s.discovery_complete) {
            s.rediscovered = true;
        }

        if (s.replace_pending) {
            s.replace_pending = false;
            s.table.count = 0;
        }

        for (u32 i = 0; i < std::min<u32>(e.info.client_cache_save.count, 10); ++i) {
            const auto &attr = e.info.client_cache_save.attr_list[i];
            size_t index = 0;
            while ((index < s.table.count) && (s.table.attributes[index].handle != attr.handle)) ++index;
            if (index < MaxAttributes) {
                s.table.attributes[index] = attr;
                if (index == s.table.count) ++s.table.count;
            }
        }
    }

    Result Discover(u32 slot_index, u32 conn_id, const bluetooth::Address &address) {
        R_UNLESS(slot_index < MaxBleControllers, svc::ResultInvalidArgument());
        auto &s = g_slots[slot_index];
        // Reconnect order need not match the previous slot order.
        if (!AddressEqual(s.table.address, address)) {
            s.table = {};
            for (const auto &cached : g_slots) {
                if (AddressEqual(cached.table.address, address)) {
                    s.table = cached.table;
                    break;
                }
            }
        }
        s.table.address = address;
        s.conn_id = conn_id;
        s.discovery_complete = false;
        s.rediscovered = false;
        s.replace_pending = true;

        const auto link_lost = [&] {
            return events::IsSlotDisconnected(slot_index) || !events::IsRadioEnabled();
        };
        const auto disconnected = [&](const EventRecord &) { return link_lost(); };

        // Wait for fresh discovery before querying. Fall back to the original
        // two-second settle if the stack is using its cache or emits no event.
        // Do not gate CreateBond on an attribute table that may arrive later.
        events::WaitFor(events::InvalidType, TimeSpan::FromSeconds(2), nullptr, [&](const EventRecord &) {
            return link_lost() || !s.replace_pending; // first fresh cache chunk, not a reused address cache
        });
        R_UNLESS(!link_lost(), ResultNoConnection());

        Result rc = btdrvGetGattAttribute(address, conn_id);
        log::Write("btdrvGetGattAttribute slot=%u rc=0x%x", slot_index, rc.GetValue());

        rc = btdrvConfigureAttMtu(conn_id, 0x100);
        log::Write("btdrvConfigureAttMtu slot=%u rc=0x%x", slot_index, rc.GetValue());
        if (R_SUCCEEDED(rc)) {
            const bool completed = events::WaitFor(events::InvalidType, TimeSpan::FromSeconds(5), nullptr, [&](const EventRecord &r) {
                return disconnected(r) || ((r.type == BtdrvBleEventType_ClientConfigureMtu) &&
                    (r.info.client_configure_mtu.conn_id == conn_id));
            });
            R_UNLESS(!link_lost(), ResultNoConnection());
            // A larger MTU is optional; a missing/refused response does not
            // prove that pairing failed. Input still fits the default MTU.
            if (!completed) log::Write("slot %u: MTU completion timed out, continuing to pairing", slot_index);
        }
        events::WaitFor(events::InvalidType, TimeSpan::FromMilliSeconds(500), nullptr, disconnected);
        R_UNLESS(!link_lost(), ResultNoConnection());
        log::Write("slot %u: GATT setup finished, attributes=%u hid_input=%u; pairing next",
            slot_index, s.table.count, HasHidInputReport(slot_index));
        R_SUCCEED();
    }

    bool TakeRediscovered(u32 slot_index) {
        if (slot_index >= MaxBleControllers) {
            return false;
        }
        const bool rediscovered = g_slots[slot_index].rediscovered;
        g_slots[slot_index].rediscovered = false;
        return rediscovered;
    }

    Result Subscribe(u32 slot_index, u32 conn_id, HidService *out) {
        if (slot_index >= MaxBleControllers) {
            R_THROW(svc::ResultInvalidArgument());
        }

        R_UNLESS(!events::IsSlotDisconnected(slot_index) && events::IsRadioEnabled(), ResultNoConnection());
        if (!HasHidInputReport(slot_index)) {
            // Called after CreateBond. Retry the cache query now that link
            // setup/security has progressed, instead of disconnecting beforehand.
            Result rc = btdrvGetGattAttribute(g_slots[slot_index].table.address, conn_id);
            log::Write("btdrvGetGattAttribute after pairing slot=%u rc=0x%x", slot_index, rc.GetValue());
            R_TRY(rc);
            // Cache tables arrive in chunks. Give the query's remaining chunks
            // the same settling window as initial discovery before snapshotting.
            events::WaitFor(events::InvalidType, TimeSpan::FromMilliSeconds(500), nullptr, [&](const EventRecord &) {
                return events::IsSlotDisconnected(slot_index) || !events::IsRadioEnabled();
            });
            const bool ready = events::WaitFor(events::InvalidType, TimeSpan::FromSeconds(5), nullptr, [&](const EventRecord &) {
                return events::IsSlotDisconnected(slot_index) || !events::IsRadioEnabled() || HasHidInputReport(slot_index);
            });
            R_UNLESS(!events::IsSlotDisconnected(slot_index) && events::IsRadioEnabled(), ResultNoConnection());
            if (!ready) log::Write("slot %u: HID attributes unavailable after pairing (attributes=%u)", slot_index, g_slots[slot_index].table.count);
            R_UNLESS(ready, svc::ResultTimedOut());
        }

        auto found = FindService(slot_index, GattServiceHid);
        if (!found) {
            log::Write("slot %u: no HID service in attribute table", slot_index);
            R_THROW(svc::ResultNotFound());
        }

        auto &s = g_slots[slot_index];
        const BtdrvGattAttribute service = *found;
        const size_t count = s.table.count;

        out->service = service;
        out->output_report = {};
        out->has_output_report = false;
        out->output_write_with_response = false;
        out->vendor_id = 0;
        out->product_id = 0;
        out->has_pnp_id = false;
        std::memset(out->device_name, 0, sizeof(out->device_name));

        // Authenticated reads request encryption (pairing first if needed).
        // Skip long GattCharReportMap (0x2A4B) which precedes HID Information on Stadia
        // and stalls BSA without completing; use short GattCharHidInformation (0x2A4A) only.
        bool found_hid_info = false;
        bool hid_info_timed_out = false;
        for (size_t i = 0; i < count; ++i) {
            const auto attr = s.table.attributes[i];
            if ((attr.type != BtdrvGattAttributeType_Characteristic) || !IsWithinService(attr, service)) {
                continue;
            }

            // The stack drops the link itself when encryption fails
            R_UNLESS(!events::IsSlotDisconnected(slot_index) && events::IsRadioEnabled(), ResultNoConnection());

            if (IsUuid16(attr.id.uuid, GattCharHidInformation)) {
                found_hid_info = true;
                Result rc;
                for (int attempt = 0; ; ++attempt) {
                    R_UNLESS(!events::IsSlotDisconnected(slot_index) && events::IsRadioEnabled(), ResultNoConnection());
                    rc = btdrvReadGattCharacteristic(conn_id, service.is_primary, &service.id, &attr.id, BtdrvGattAuthReqType_NoMitm);
                    log::Write("btdrvReadGattCharacteristic slot=%u handle=0x%04x rc=0x%x", slot_index, attr.handle, rc.GetValue());
                    // Refused locally while the pad's connection parameter update is in flight; it clears within a second or two
                    if (!ResultInsufficientAuthentication::Includes(rc) || (attempt >= 4) || events::IsSlotDisconnected(slot_index)) {
                        break;
                    }
                    events::Pump(TimeSpan::FromSeconds(1));
                }
                R_TRY(rc);
                bool timed_out = false;
                R_TRY(WaitForOperation(slot_index, conn_id, BtdrvGattOperationType_CharacteristicRead,
                    GattCharHidInformation, TimeSpan::FromSeconds(2), nullptr, &timed_out));
                if (timed_out) {
                    hid_info_timed_out = true;
                    log::Write("slot %u: HID Information read timed out on live link; continuing to CCCD", slot_index);
                }
            }
        }
        R_UNLESS(found_hid_info, svc::ResultNotFound());

        Result result = svc::ResultNotFound();
        for (size_t i = 0; i < count; ++i) {
            const auto attr = s.table.attributes[i];
            if ((attr.type != BtdrvGattAttributeType_Characteristic) || !IsWithinService(attr, service) || !IsUuid16(attr.id.uuid, GattCharReport)) {
                continue;
            }

            R_UNLESS(!events::IsSlotDisconnected(slot_index), ResultNoConnection());

            // Rumble goes to the writable Report; prefer a non-Notify output Report characteristic when multiple exist.
            if (attr.property & (BtdrvGattCharacteristicProperty_Write | BtdrvGattCharacteristicProperty_WriteNoResponse)) {
                const bool is_non_notify = (attr.property & BtdrvGattCharacteristicProperty_Notify) == 0;
                const bool current_is_notify = out->has_output_report && ((out->output_report.property & BtdrvGattCharacteristicProperty_Notify) != 0);
                if (!out->has_output_report || (is_non_notify && current_is_notify)) {
                    out->output_report = attr;
                    out->has_output_report = true;
                    out->output_write_with_response = (attr.property & BtdrvGattCharacteristicProperty_WriteNoResponse) == 0;
                }
            }

            if (!(attr.property & BtdrvGattCharacteristicProperty_Notify)) {
                continue;
            }

            auto cccd = FindDescriptor(slot_index, attr, GattDescClientCharConfig);
            if (!cccd) {
                continue;
            }

            Result rc = btdrvRegisterGattNotification(conn_id, service.is_primary, &service.id, &attr.id);
            log::Write("btdrvRegisterGattNotification slot=%u handle=0x%04x rc=0x%x", slot_index, attr.handle, rc.GetValue());

            R_TRY(rc);

            const u8 enable_notifications[2] = { 0x01, 0x00 };
            rc = btdrvWriteGattDescriptor(conn_id, service.is_primary, &service.id, &attr.id, &cccd->id, enable_notifications, sizeof(enable_notifications), BtdrvGattAuthReqType_NoMitm);
            log::Write("btdrvWriteGattDescriptor(CCCD) slot=%u handle=0x%04x rc=0x%x", slot_index, cccd->handle, rc.GetValue());
            R_TRY(rc);
            bool cccd_timed_out = false;
            R_TRY(WaitForOperation(slot_index, conn_id, BtdrvGattOperationType_DescriptorWrite, GattCharReport, TimeSpan::FromSeconds(2), nullptr, &cccd_timed_out));
            if (cccd_timed_out) {
                log::Write("slot %u: CCCD write timed out on live link; continuing", slot_index);
            }
            result = ResultSuccess();
        }

        if (R_SUCCEEDED(result)) {
            if (!hid_info_timed_out) {
                // Best-effort device identification (PnP ID / GAP Device Name); failure does not abort setup.
                u8 pnp_buf[16] = {};
                size_t pnp_len = 0;
                if (ReadCharacteristicBestEffort(slot_index, conn_id, GattServiceDeviceInfo, GattCharPnpId, pnp_buf, sizeof(pnp_buf), &pnp_len) && pnp_len >= 7) {
                    out->vendor_id = static_cast<u16>(pnp_buf[1] | (pnp_buf[2] << 8));
                    out->product_id = static_cast<u16>(pnp_buf[3] | (pnp_buf[4] << 8));
                    out->has_pnp_id = true;
                    log::Write("slot %u: PnP ID vid=0x%04x pid=0x%04x", slot_index, out->vendor_id, out->product_id);
                } else if (ReadCharacteristicBestEffort(slot_index, conn_id, GattServiceGap, GattCharDeviceName, reinterpret_cast<u8 *>(out->device_name), sizeof(out->device_name) - 1, &pnp_len)) {
                    out->device_name[std::min(pnp_len, sizeof(out->device_name) - 1)] = '\0';
                    log::Write("slot %u: GAP device name='%s'", slot_index, out->device_name);
                }
            } else {
                log::Write("slot %u: skipping PnP/name reads after HID Info timeout", slot_index);
            }
            R_UNLESS(!events::IsSlotDisconnected(slot_index) && events::IsRadioEnabled(), ResultNoConnection());
        }

        s.discovery_complete = R_SUCCEEDED(result);
        R_RETURN(result);
    }

    void Unsubscribe(u32 slot_index, u32 conn_id) {
        const auto *service = FindService(slot_index, GattServiceHid);
        if (!service) return;
        const auto &table = g_slots[slot_index].table;
        for (size_t i = 0; i < table.count; ++i) {
            const auto &attr = table.attributes[i];
            if ((attr.type == BtdrvGattAttributeType_Characteristic) && IsWithinService(attr, *service) &&
                IsUuid16(attr.id.uuid, GattCharReport) && (attr.property & BtdrvGattCharacteristicProperty_Notify)) {
                static_cast<void>(btdrvUnregisterGattNotification(conn_id, service->is_primary, &service->id, &attr.id));
            }
        }
    }

    bool HasCachedAddress(u32 slot_index, const bluetooth::Address &address) {
        return slot_index < MaxBleControllers && g_slots[slot_index].table.count > 0 &&
            AddressEqual(g_slots[slot_index].table.address, address);
    }

    void ResetSlot(u32 slot_index) {
        if (slot_index < MaxBleControllers) {
            g_slots[slot_index].conn_id = InvalidConnId;
            g_slots[slot_index].replace_pending = false;
            g_slots[slot_index].discovery_complete = false;
            g_slots[slot_index].rediscovered = false;
        }
    }

}
