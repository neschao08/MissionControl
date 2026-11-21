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
#include "ble_pairing.hpp"
#include "ble_events.hpp"
#include "ble_results.hpp"
#include "ble_log.hpp"

namespace ams::ble::pairing {
    // BTA convention: 1 = BR/EDR, 2 = LE. The native driver may ignore it.
    constexpr u32 BondTransport_Le = 2;
    // BTM_BLE_SEC_ENCRYPT_NO_MITM (2): triggers SMP_Pair via btm_ble_set_encryption without CreateBond's 35-tick state leak.
    constexpr u8 BtmBleSecAct_EncryptNoMitm = 2;

    static Result WaitForConnection(u32 slot, u8 client_if, const bluetooth::Address &address, u32 *out_conn_id, TimeSpan timeout) {
        EventRecord e;
        if (!events::WaitFor(events::InvalidType, timeout, &e, [&](const EventRecord &r) {
            return !events::IsRadioEnabled() || events::IsSlotDisconnected(slot) ||
                (r.type == BtdrvBleEventType_ClientConnection && AddressEqual(r.info.client_connection.address, address) && r.info.client_connection.client_if == client_if);
        })) {
            R_THROW(svc::ResultTimedOut());
        }

        R_UNLESS(events::IsRadioEnabled() && e.type == BtdrvBleEventType_ClientConnection, ResultNoConnection());
        if ((e.info.client_connection.result != 0) || (e.info.client_connection.status != BleConnectionStatus_Connected)) {
            log::Write("slot %u: connection failed result=%u status=%u", slot, e.info.client_connection.result, e.info.client_connection.status);
            R_THROW(ResultNoConnection());
        }

        R_UNLESS(e.info.client_connection.conn_id != InvalidConnId, ResultNoConnection());
        *out_conn_id = e.info.client_connection.conn_id;
        R_UNLESS(!events::IsSlotDisconnected(slot), ResultNoConnection());
        log::Write("slot %u: connected conn_id=%u", slot, *out_conn_id);
        R_SUCCEED();
    }

    Result Connect(u32 slot, u8 client_if, const bluetooth::Address &address, u32 *out_conn_id, TimeSpan timeout) {
        events::SetSlotConnecting(slot, address);
        R_UNLESS(events::IsRadioEnabled(), ResultNoConnection());

        if (client_if == InvalidClientIf) {
            log::Write("slot %u: no client_if available", slot);
            return ResultNoConnection();
        }

        const Result remove_rc = btdrvRemoveBond(address);
        log::Write("btdrvRemoveBond slot=%u rc=0x%x", slot, remove_rc.GetValue());
        R_TRY(remove_rc);
        R_UNLESS(events::IsRadioEnabled(), ResultNoConnection());
        // A previously cancelled direct request can complete during the IPC.
        // Retain that live handle rather than opening twice.
        if (const u32 late_conn_id = events::GetSlotConnectionId(slot); late_conn_id != InvalidConnId) {
            *out_conn_id = late_conn_id;
            log::Write("slot %u: adopting late connection conn_id=%u", slot, late_conn_id);
            R_SUCCEED();
        }
        // On HOS 22.5.0+, standard btdrvRemoveBond returns early when a BLE-only peer is absent from the
        // Classic peer table. Best-effort invoke the IPS BSA_SecRemoveDevice hook (cmd 4) to clear BLE records.
        const Result ext_remove_rc = btdrvextSecRemoveDevice(address);
        log::Write("btdrvextSecRemoveDevice slot=%u rc=0x%x", slot, ext_remove_rc.GetValue());
        R_UNLESS(events::IsRadioEnabled(), ResultNoConnection());
        if (const u32 late_conn_id = events::GetSlotConnectionId(slot); late_conn_id != InvalidConnId) {
            *out_conn_id = late_conn_id;
            log::Write("slot %u: adopting late connection conn_id=%u", slot, late_conn_id);
            R_SUCCEED();
        }
        // Arm the slot after any synchronous cleanup events.
        events::SetSlotConnecting(slot, address);

        Result rc = btdrvConnectGattServer(client_if, address, true, 0);
        log::Write("btdrvConnectGattServer slot=%u client_if=%u rc=0x%x", slot, client_if, rc.GetValue());
        R_TRY(rc);

        rc = WaitForConnection(slot, client_if, address, out_conn_id, timeout);
        if (R_FAILED(rc) && events::IsRadioEnabled()) {
            static_cast<void>(btdrvCancelConnectGattServer(client_if, address, true));
        }
        R_RETURN(rc);
    }

    Result ClearPeer(u32 slot, const bluetooth::Address &address) {
        R_UNLESS(slot < MaxBleControllers, ResultNoConnection());
        R_UNLESS(events::IsRadioEnabled(), ResultNoConnection());
        R_UNLESS(events::GetSlotConnectionId(slot) == InvalidConnId, ResultNoConnection());

        Result rc = btdrvCancelBond(address);
        log::Write("btdrvCancelBond slot=%u rc=0x%x", slot, rc.GetValue());
        const Result cancel_rc = rc;

        R_UNLESS(events::IsRadioEnabled(), ResultNoConnection());
        R_UNLESS(events::GetSlotConnectionId(slot) == InvalidConnId, ResultNoConnection());
        rc = btdrvRemoveBond(address);
        log::Write("btdrvRemoveBond slot=%u rc=0x%x", slot, rc.GetValue());
        if (events::IsRadioEnabled() && events::GetSlotConnectionId(slot) == InvalidConnId) {
            const Result ext_rc = btdrvextSecRemoveDevice(address);
            log::Write("btdrvextSecRemoveDevice(clear) slot=%u rc=0x%x", slot, ext_rc.GetValue());
        }
        if (R_FAILED(rc)) return rc;
        if (R_FAILED(cancel_rc)) return cancel_rc;
        R_RETURN(rc);
    }

    bool PrepareSecurity(u32 slot, const bluetooth::Address &address) {
        if (!events::IsRadioEnabled() || events::IsSlotDisconnected(slot)) return false;
        log::Write("slot %u: requesting LE security with CreateBond", slot);
        // Discovery or old bond removal can produce the same core indication
        // as pairing. Only a new indication may satisfy this attempt.
        static_cast<void>(events::TakeSlotBondComplete(slot));
        Result rc = btdrvCreateBond(address, BondTransport_Le);
        log::Write("btdrvCreateBond slot=%u rc=0x%x", slot, rc.GetValue());
        if (R_FAILED(rc)) {
            return false;
        }

        // Some stacks/controllers (e.g. Stadia BLE) emit an initial core connection
        // indication (type=3) synchronously during or within 50ms of CreateBond while
        // SMP / ConnectionUpdate is still in flight. Wait for a second indication or a
        // 1.5s settling window before touching authenticated GATT characteristics.
        bool early_bond = events::TakeSlotBondComplete(slot);
        if (!early_bond) {
            events::WaitFor(events::InvalidType, TimeSpan::FromMilliSeconds(50), nullptr, [&](const EventRecord &) {
                early_bond = events::TakeSlotBondComplete(slot);
                return early_bond || events::IsSlotDisconnected(slot) || !events::IsRadioEnabled();
            });
        }
        if (events::IsSlotDisconnected(slot) || !events::IsRadioEnabled()) {
            log::Write("slot %u: pairing aborted immediately after CreateBond (disconnected=%u reason=0x%04x radio=%u)",
                slot, events::IsSlotDisconnected(slot), events::GetSlotDisconnectReason(slot), events::IsRadioEnabled());
            return false;
        }

        if (early_bond) {
            log::Write("slot %u: early core connection indication during CreateBond; waiting 1500ms for SMP settle", slot);
            bool second_bond = false;
            events::WaitFor(events::InvalidType, TimeSpan::FromMilliSeconds(1500), nullptr, [&](const EventRecord &) {
                second_bond = events::TakeSlotBondComplete(slot);
                return second_bond || events::IsSlotDisconnected(slot) || !events::IsRadioEnabled();
            });
            if (second_bond && !events::IsSlotDisconnected(slot) && events::IsRadioEnabled()) {
                events::WaitFor(events::InvalidType, TimeSpan::FromMilliSeconds(500), nullptr, [&](const EventRecord &) {
                    return events::IsSlotDisconnected(slot) || !events::IsRadioEnabled();
                });
            }
            if (!events::IsSlotDisconnected(slot) && events::IsRadioEnabled()) {
                log::Write("slot %u: link alive after early bond settle; verifying encrypted GATT access", slot);
                return true;
            }
            log::Write("slot %u: pairing failed during early bond settle (second=%u disconnected=%u reason=0x%04x radio=%u)",
                slot, second_bond, events::IsSlotDisconnected(slot), events::GetSlotDisconnectReason(slot), events::IsRadioEnabled());
            return false;
        }

        bool bonded = false;
        events::WaitFor(events::InvalidType, TimeSpan::FromMilliSeconds(9950), nullptr, [&](const EventRecord &) {
            bonded = bonded || events::TakeSlotBondComplete(slot);
            return bonded || events::IsSlotDisconnected(slot) || !events::IsRadioEnabled();
        });
        if (bonded) {
            // The probe's bond indication can precede a refused pairing's
            // disconnect and late discovery events. Drain them before use.
            events::WaitFor(events::InvalidType, TimeSpan::FromMilliSeconds(500), nullptr, [&](const EventRecord &) {
                return events::IsSlotDisconnected(slot) || !events::IsRadioEnabled();
            });
        }
        if (bonded && !events::IsSlotDisconnected(slot) && events::IsRadioEnabled()) {
            log::Write("slot %u: bond indication received; verifying encrypted GATT access", slot);
            return true;
        }
        log::Write("slot %u: pairing failed (bond_indication=%u disconnected=%u reason=0x%04x radio=%u)",
            slot, bonded, events::IsSlotDisconnected(slot), events::GetSlotDisconnectReason(slot), events::IsRadioEnabled());
        return false;
    }

    bool PrepareSecurityWithStrategy(u32 slot, const bluetooth::Address &address, SecurityStrategy strategy) {
        if (strategy == SecurityStrategy::CreateBond) {
            return PrepareSecurity(slot, address);
        }

        if (!events::IsRadioEnabled() || events::IsSlotDisconnected(slot)) {
            return false;
        }

        static_cast<void>(events::TakeSlotBondComplete(slot));

        if (strategy == SecurityStrategy::GattOnDemand) {
            log::Write("slot %u: deferring LE security to GATT on-demand encryption (no CreateBond)", slot);
            return true;
        }

        // SecurityStrategy::DirectSetEncryption:
        // Invoke BTA_DmSetEncryption(addr, LE=2, cb, BTM_BLE_SEC_ENCRYPT_NO_MITM=2) via IPS cmd 5.
        // Note: BTA_DmSetEncryption is void and the IPS cave epilogue preserves residual w0, so a
        // non-zero raw_rc does NOT indicate IPC failure; rely on live link / radio state instead.
        log::Write("slot %u: requesting LE security with BTA_DmSetEncryption(transport=%u, sec_act=%u)",
            slot, BondTransport_Le, BtmBleSecAct_EncryptNoMitm);
        const Result raw_rc = btdrvextSetEncryption(address, static_cast<u8>(BondTransport_Le), BtmBleSecAct_EncryptNoMitm);
        log::Write("btdrvextSetEncryption slot=%u raw_rc=0x%x (void return ignored)", slot, raw_rc.GetValue());

        if (events::IsSlotDisconnected(slot) || !events::IsRadioEnabled()) {
            log::Write("slot %u: direct encryption aborted immediately (disconnected=%u reason=0x%04x radio=%u)",
                slot, events::IsSlotDisconnected(slot), events::GetSlotDisconnectReason(slot), events::IsRadioEnabled());
            return false;
        }

        bool bond_evt = false;
        events::WaitFor(events::InvalidType, TimeSpan::FromMilliSeconds(1500), nullptr, [&](const EventRecord &) {
            bond_evt = bond_evt || events::TakeSlotBondComplete(slot);
            return bond_evt || events::IsSlotDisconnected(slot) || !events::IsRadioEnabled();
        });
        if (bond_evt && !events::IsSlotDisconnected(slot) && events::IsRadioEnabled()) {
            events::WaitFor(events::InvalidType, TimeSpan::FromMilliSeconds(500), nullptr, [&](const EventRecord &) {
                return events::IsSlotDisconnected(slot) || !events::IsRadioEnabled();
            });
        }

        if (!events::IsSlotDisconnected(slot) && events::IsRadioEnabled()) {
            log::Write("slot %u: link alive after BTA_DmSetEncryption settle (bond_evt=%u); proceeding to GATT", slot, bond_evt);
            return true;
        }

        log::Write("slot %u: direct encryption failed (bond_evt=%u disconnected=%u reason=0x%04x radio=%u)",
            slot, bond_evt, events::IsSlotDisconnected(slot), events::GetSlotDisconnectReason(slot), events::IsRadioEnabled());
        return false;
    }

}
