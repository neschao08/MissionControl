// End-to-end HOS 22.5.0 bt_nso state-machine simulator compiling production ble_pairing.cpp + ble_gatt.cpp unchanged.
#include "ble_pairing.cpp"
#include "ble_gatt.cpp"
#include "../mc_mitm/source/controllers/stadia_mapping.hpp"
#include <cassert>
#include <iostream>
#include <map>
#include <vector>

using namespace ams::ble;

namespace {

    constexpr ams::bluetooth::Address StadiaAddr = {{0xcb, 0x4e, 0xc0, 0x96, 0x49, 0x67}};
    constexpr ams::bluetooth::Address XboxAddr   = {{0x14, 0xcb, 0x65, 0xe8, 0x2b, 0xe2}};

    enum class PeripheralBehavior {
        StadiaAcceptPlaintext,            // Accepts CCCD write with AuthReq=None immediately
        StadiaRequireEncViaAtt05,         // Rejects plaintext CCCD with ATT 0x05; accepts once SMP_Pair encrypts
        StadiaRequireEncViaAtt0F,         // Rejects plaintext CCCD with ATT 0x0F; Stage 0 SMP times out -> Stage 1 DirectSetEncryption succeeds
        StadiaOnlyStage2CreateBondPlain,  // Stage 0 & Stage 1 fail -> Stage 2 CreateBond (sec_state=1) + AuthReq=None bypasses 0xc56fc stall
        StadiaStaleSecRecord,             // Fails SMP with 0x89 unless IPS cmd4 cleared stale BLE security record
        StadiaProactiveSlaveSecReq,       // Exact real StadiaQPRV-4967 trace: sends SMP Slave Security Request at +80ms -> WAIT_APP_DECISION (sec_state=1, 30.15s disconnect unless cmd 6 grants!)
        XboxClassicBondThenNoMitm,        // Xbox One BLE: CreateBond completes cleanly at 120ms, sec_state=0, encrypted=true
    };

    // Faithful state machine extracted from HOS 22.5.0 bt_nso disassembly:
    // - gatt_determine_sec_act (0xc56c0): auth==0 bypasses sec_state at 0xc56e0; auth==1 stalls if sec_state==1 at 0xc56fc;
    //   auth==1 with sec_state==0 invokes BTM_SetEncryption(sec_act=2) -> SMP_Pair (0xc5848 -> 0xa446c -> 0xf5ad0).
    // - gatt_process_error_rsp (0xc60a8): ATT error resets channel state back to IDLE (6).
    // - Sending a second GATT op while channel state != 6 returns 0x2a671 (GATT_OPERATION_BUSY).
    // - btdrvRemoveBond (0x524c): no-op for LE-only peers not in Classic peer table.
    // - IPS cmd 4 (0x48570 BSA_SecRemoveDevice): actually clears BLE security record.
    // - IPS cmd 5 (0x6dfe0 BTA_DmSetEncryption): void return leaves garbage w0, starts LE encryption.
    // - btdrvCreateBond: sets sec_state=1 on LE-only Stadia, fires type=3 at 3ms without disconnecting,
    //   and arms a 28.40s (35-tick) zombie timer that fires a second type=3.
    struct SimConn {
        u32 conn_id = InvalidConnId;
        u32 slot = InvalidSlot;
        u8 client_if = 6;
        ams::bluetooth::Address addr = {};
        PeripheralBehavior behavior = PeripheralBehavior::StadiaAcceptPlaintext;
        bool connected = false;
        bool encrypted = false;
        u8 sec_state = 0;           // 0 = IDLE, 1 = BUSY/BONDING (causes NoMitm to stall at 0xc56fc)
        u8 smp_state = 0;           // 0 = IDLE, 1 = SMP_ST_WAIT_APP_DECISION
        u8 smp_cb_evt = 0;          // 2 = SMP_SEC_REQUEST_EVT (cleared by SMP_SecurityGrant 0xf5bf0)
        bool smp_timeout_armed = false;
        u8 gatt_state = 6;          // 6 = IDLE, !=6 = BUSY -> 0x2a671
        bool cccd_disabled_first = false;
        bool cccd_enabled = false;
        bool notif_registered = false;     // bt_nso BSA_GattcRegisterNotification actually sent!
    };

    struct BtNsoSim {
        u32 next_conn_id = 3;
        std::map<u32, SimConn> conns;
        std::map<u8, bool> gatt_manager_notif_table; // Models bt_nso 0xa5e0/0xa540/0x2c26c table!
        std::map<u64, bool> stale_ble_sec_records;
        std::map<u64, bool> stored_ble_ltk;
        std::map<u64, PeripheralBehavior> peer_behaviors;
        bool ips_cmd4_enabled = true;
        bool ips_cmd5_enabled = true;
        bool ips_cmd6_enabled = true;
        u32 busy_0x2a671_count = 0;
        u32 create_bond_count = 0;
        u32 ext_remove_count = 0;
        u32 ext_set_enc_count = 0;
        u32 ext_sec_grant_count = 0;
        u32 smp_pair_count = 0;
        u32 ltk_fast_encrypt_count = 0;
        u32 smp_30s_timeout_disconnects = 0;
        u32 zombie_timer_fires = 0;
        u32 cccd_write_count = 0;
        bool force_disconnect_after_ms_enabled = false;
        u64 force_disconnect_delay_ms = 0;

        static u64 AddrKey(const ams::bluetooth::Address &a) {
            u64 k = 0;
            for (int i = 0; i < 6; ++i) k = (k << 8) | a.address[i];
            return k;
        }

        void Reset() {
            for (auto &s : gatt::g_slots) s = {};
            for (auto &s : events::slots) s = {};
            events::queue.clear();
            events::radio = true;
            events::now_ms = 0;
            next_conn_id = 3;
            conns.clear();
            gatt_manager_notif_table.clear();
            stale_ble_sec_records.clear();
            stored_ble_ltk.clear();
            peer_behaviors.clear();
            ips_cmd4_enabled = true;
            ips_cmd5_enabled = true;
            ips_cmd6_enabled = true;
            busy_0x2a671_count = 0;
            create_bond_count = 0;
            ext_remove_count = 0;
            ext_set_enc_count = 0;
            ext_sec_grant_count = 0;
            smp_pair_count = 0;
            ltk_fast_encrypt_count = 0;
            smp_30s_timeout_disconnects = 0;
            zombie_timer_fires = 0;
            cccd_write_count = 0;
            force_disconnect_after_ms_enabled = false;
        }

        SimConn *FindConnByAddr(const ams::bluetooth::Address &addr) {
            for (auto &[id, c] : conns) {
                if (c.connected && AddressEqual(c.addr, addr)) return &c;
            }
            return nullptr;
        }

        SimConn *FindConnById(u32 conn_id) {
            auto it = conns.find(conn_id);
            if (it == conns.end() || !it->second.connected) return nullptr;
            return &it->second;
        }

        void PublishRealStadiaAttributes(u32 conn_id) {
            // Exact attribute structure captured in real 16:14 ble.log from StadiaQPRV-4967 (27 attrs across 3 chunks)
            EventRecord c1{}, c2{}, c3{};
            c1.type = c2.type = c3.type = BtdrvBleEventType_ClientCacheSave;
            c1.info.client_cache_save.conn_id = c2.info.client_cache_save.conn_id = c3.info.client_cache_save.conn_id = conn_id;

            // Chunk 1: GAP + DIS
            auto &a1 = c1.info.client_cache_save;
            a1.count = 4;
            a1.attr_list[0] = {BtdrvGattAttributeType_Service, 0x0001, 0x000b, 0, true, {{GattServiceGap}, 0}};
            a1.attr_list[1] = {BtdrvGattAttributeType_Characteristic, 0x0003, 0, BtdrvGattCharacteristicProperty_Read, true, {{GattCharDeviceName}, 0}};
            a1.attr_list[2] = {BtdrvGattAttributeType_Service, 0x0010, 0x0025, 0, true, {{GattServiceDeviceInfo}, 0}};
            a1.attr_list[3] = {BtdrvGattAttributeType_Characteristic, 0x0024, 0, BtdrvGattCharacteristicProperty_Read, true, {{GattCharPnpId}, 0}};

            // Chunk 2: HID Service + Input Report (0x0040, inst=0, prop=0x12) + CCCD (0x0041) + Output Report (0x0044, inst=1, prop=0x0a)
            auto &a2 = c2.info.client_cache_save;
            a2.count = 6;
            a2.attr_list[0] = {BtdrvGattAttributeType_Service, 0x003c, 0xffff, 0, true, {{GattServiceHid}, 0}};
            a2.attr_list[1] = {BtdrvGattAttributeType_Characteristic, 0x0040, 0, BtdrvGattCharacteristicProperty_Read | BtdrvGattCharacteristicProperty_Notify, true, {{GattCharReport}, 0}};
            a2.attr_list[2] = {BtdrvGattAttributeType_Descriptor, 0x0041, 0, 0, true, {{GattDescClientCharConfig}, 0}};
            a2.attr_list[3] = {BtdrvGattAttributeType_Descriptor, 0x0042, 0, 0, true, {{0x2908}, 0}};
            a2.attr_list[4] = {BtdrvGattAttributeType_Characteristic, 0x0044, 0, BtdrvGattCharacteristicProperty_Read | BtdrvGattCharacteristicProperty_Write, true, {{GattCharReport}, 1}};
            a2.attr_list[5] = {BtdrvGattAttributeType_Descriptor, 0x0046, 0, 0, true, {{0x2908}, 0}};

            // Chunk 3: Report Map (0x0048) + HID Information (0x004a)
            auto &a3 = c3.info.client_cache_save;
            a3.count = 2;
            a3.attr_list[0] = {BtdrvGattAttributeType_Characteristic, 0x0048, 0, BtdrvGattCharacteristicProperty_Read, true, {{GattCharReportMap}, 0}};
            a3.attr_list[1] = {BtdrvGattAttributeType_Characteristic, 0x004a, 0, BtdrvGattCharacteristicProperty_Read, true, {{GattCharHidInformation}, 0}};

            events::ScheduleEvent(c1, 20);
            events::ScheduleEvent(c2, 35);
            events::ScheduleEvent(c3, 50);
        }

        void DisconnectConn(u32 conn_id, u16 reason = 0x16, u32 disc_result = 0) {
            auto *c = FindConnById(conn_id);
            if (!c) return;
            c->connected = false;
            c->gatt_state = 6;
            // Faithful bt_nso 0x2c26c bug model: only clears g_gatt_manager notification table when result == 0!
            // When result == 26 (supervision timeout), bt_nso skips BSA_GattcDeregisterNotification & 0xa540!
            if (disc_result == 0) {
                gatt_manager_notif_table[c->client_if] = false;
            }
            EventRecord disc{};
            disc.type = BtdrvBleEventType_ClientConnection;
            disc.info.client_connection.result = disc_result;
            disc.info.client_connection.status = BleConnectionStatus_Disconnected;
            disc.info.client_connection.conn_id = conn_id;
            disc.info.client_connection.address = c->addr;
            disc.info.client_connection.reason = reason;
            events::ScheduleEvent(disc, 10);
        }
    };

    BtNsoSim g_sim;

} // namespace

// --- Mocked btdrv / btdrvext IPCs backed by BtNsoSim ---

Result btdrvRemoveBond(ams::bluetooth::Address addr) {
    // HOS 22.5.0 without 0x524c NOP: returns 0 (no-op) for LE-only Stadia, only clears Xbox Classic peer entry
    if (AddressEqual(addr, XboxAddr)) {
        g_sim.stale_ble_sec_records[BtNsoSim::AddrKey(addr)] = false;
    }
    return {0};
}

Result btdrvextSecRemoveDevice(ams::bluetooth::Address addr) {
    ++g_sim.ext_remove_count;
    if (!g_sim.ips_cmd4_enabled) {
        return {0x2a605};
    }
    g_sim.stale_ble_sec_records[BtNsoSim::AddrKey(addr)] = false;
    g_sim.stored_ble_ltk[BtNsoSim::AddrKey(addr)] = false;
    return {0};
}

Result btdrvCancelBond(ams::bluetooth::Address) {
    return {0};
}

Result btdrvConnectGattServer(u8 client_if, ams::bluetooth::Address addr, bool, u64) {
    const u32 cid = g_sim.next_conn_id;
    g_sim.next_conn_id += 256;

    u32 slot = InvalidSlot;
    for (u32 s = 0; s < MaxBleControllers; ++s) {
        if (events::slots[s].active && AddressEqual(events::slots[s].address, addr)) {
            slot = s;
            break;
        }
    }

    SimConn conn{};
    conn.conn_id = cid;
    conn.slot = slot;
    conn.client_if = client_if;
    conn.addr = addr;
    conn.connected = true;
    conn.gatt_state = 6; // IDLE
    conn.sec_state = 0;
    auto it = g_sim.peer_behaviors.find(BtNsoSim::AddrKey(addr));
    conn.behavior = (it != g_sim.peer_behaviors.end()) ? it->second : PeripheralBehavior::StadiaAcceptPlaintext;
    g_sim.conns[cid] = conn;

    EventRecord ev{};
    ev.type = BtdrvBleEventType_ClientConnection;
    ev.info.client_connection.result = 0;
    ev.info.client_connection.status = BleConnectionStatus_Connected;
    ev.info.client_connection.client_if = client_if;
    ev.info.client_connection.conn_id = cid;
    ev.info.client_connection.address = addr;
    events::ScheduleEvent(ev, 25);

    if (conn.behavior == PeripheralBehavior::StadiaProactiveSlaveSecReq) {
        // Real StadiaQPRV-4967 trace: sends SMP Slave Security Request ~80ms after ACL connect.
        // Faithful to btm_ble_link_sec_check (0xa43b0) + smp_proc_sec_req (0xf4b40) + btm_proc_smp_cback (0xa49b0):
        // - If peer LTK is already stored in bt_nso RAM (key_type & 1), immediately encrypts without WAIT_APP_DECISION.
        // - Otherwise sets sec_state=1, smp_state=1 (SMP_ST_WAIT_APP_DECISION), cb_evt=2 (SMP_SEC_REQUEST_EVT),
        //   and arms 30.15s SMP_WAIT_FOR_RSP_TOUT that disconnects with reason=0x0016 unless BTA_DmBleSecurityGrant is called!
        events::ScheduleCallback(80, [cid]() {
            auto *live = g_sim.FindConnById(cid);
            if (!live) return;
            if (g_sim.stored_ble_ltk[BtNsoSim::AddrKey(live->addr)]) {
                ++g_sim.ltk_fast_encrypt_count;
                live->encrypted = true;
                live->sec_state = 0;
                live->smp_state = 0;
                live->smp_cb_evt = 0;
                return;
            }
            live->sec_state = 1;
            live->smp_state = 1;
            live->smp_cb_evt = 2;
            live->smp_timeout_armed = true;
            events::ScheduleCallback(30150, [cid]() {
                auto *c2 = g_sim.FindConnById(cid);
                if (!c2) return;
                if (c2->smp_timeout_armed && c2->smp_state == 1 && c2->smp_cb_evt == 2) {
                    ++g_sim.smp_30s_timeout_disconnects;
                    c2->smp_timeout_armed = false;
                    c2->sec_state = 0;
                    g_sim.DisconnectConn(cid, 0x0016);
                }
            });
        });
    }

    if (g_sim.force_disconnect_after_ms_enabled) {
        events::ScheduleCallback(g_sim.force_disconnect_delay_ms, [cid]() {
            g_sim.DisconnectConn(cid, 0x08);
        });
    }
    return {0};
}

Result btdrvCancelConnectGattServer(u8, ams::bluetooth::Address, bool) {
    return {0};
}

Result btdrvGetGattAttribute(ams::bluetooth::Address, u32 conn_id) {
    if (!g_sim.FindConnById(conn_id)) return {4};
    g_sim.PublishRealStadiaAttributes(conn_id);
    return {0};
}

Result btdrvConfigureAttMtu(u32 conn_id, u16 mtu) {
    if (!g_sim.FindConnById(conn_id)) return {4};
    EventRecord ev{};
    ev.type = BtdrvBleEventType_ClientConfigureMtu;
    ev.info.client_configure_mtu = {0, conn_id, mtu};
    events::ScheduleEvent(ev, 40);
    return {0};
}

Result btdrvCreateBond(ams::bluetooth::Address addr, u32) {
    ++g_sim.create_bond_count;
    auto *c = g_sim.FindConnByAddr(addr);
    if (!c) return {4};

    if (c->behavior == PeripheralBehavior::XboxClassicBondThenNoMitm) {
        // Xbox: real Classic/LE bond completes cleanly at 120ms, sec_state returns to 0
        c->encrypted = true;
        c->sec_state = 0;
        events::ScheduleBond(c->slot, 120);
        return {0};
    }

    // Stadia on HOS 22.5.0: reproduces exact 16:14 ble.log trace!
    // sec_state gets stuck at 1, type=3 fires at 3ms (no disconnect), and 28.40s zombie timer is armed.
    c->sec_state = 1;
    if (c->behavior == PeripheralBehavior::StadiaOnlyStage2CreateBondPlain) {
        // In Stage 2 scenario, ACL link actually got encrypted even though bt_nso left sec_state==1
        c->encrypted = true;
    }
    events::ScheduleBond(c->slot, 3);
    const u32 slot = c->slot;
    events::ScheduleCallback(28400, [slot]() {
        ++g_sim.zombie_timer_fires;
        events::ScheduleBond(slot, 0);
    });
    return {0};
}

Result btdrvextSetEncryption(ams::bluetooth::Address addr, u8 transport, u8 sec_act) {
    assert(transport == 2 && sec_act == 2);
    ++g_sim.ext_set_enc_count;
    auto *c = g_sim.FindConnByAddr(addr);
    if (!c) return {0xDEADBEEF};

    // Faithful to bt_nso: if SMP is stalled in WAIT_APP_DECISION (sec_state==1), SetEncryption is queued/blocked!
    if (g_sim.ips_cmd5_enabled &&
        c->sec_state == 0 &&
        c->behavior != PeripheralBehavior::StadiaOnlyStage2CreateBondPlain &&
        !g_sim.stale_ble_sec_records[BtNsoSim::AddrKey(addr)]) {
        ++g_sim.smp_pair_count;
        const u32 cid = c->conn_id;
        events::ScheduleCallback(180, [cid]() {
            if (auto *live = g_sim.FindConnById(cid)) {
                live->encrypted = true;
                live->sec_state = 0;
            }
        });
    }
    // Faithful to HOS 22.5.0 IPS cave: BTA_DmSetEncryption is void, w0 retains non-zero garbage!
    return {0x00117924u};
}

Result btdrvextBleSecurityGrant(ams::bluetooth::Address addr, u8 res) {
    ++g_sim.ext_sec_grant_count;
    if (!g_sim.ips_cmd6_enabled) {
        return {0x0011792cu};
    }
    auto *c = g_sim.FindConnByAddr(addr);
    if (!c) return {0};
    // Faithful to SMP_SecurityGrant (0xf5bf0): only acts if state==1 && cb_evt==2; otherwise safe no-op.
    if (c->smp_state == 1 && c->smp_cb_evt == 2) {
        c->smp_cb_evt = 0;
        c->smp_timeout_armed = false;
        if (res == 0) {
            // smp_proc_sec_grant -> SMP_IO_CAP_REQ_EVT -> SMP_API_IO_RSP_EVT -> Legacy Just Works Pairing + Encryption + LTK!
            ++g_sim.smp_pair_count;
            const u32 cid = c->conn_id;
            events::ScheduleCallback(220, [cid]() {
                if (auto *live = g_sim.FindConnById(cid)) {
                    live->smp_state = 0;
                    live->sec_state = 0;
                    live->encrypted = true;
                    g_sim.stored_ble_ltk[BtNsoSim::AddrKey(live->addr)] = true;
                }
            });
        } else {
            g_sim.DisconnectConn(c->conn_id, 0x0016);
        }
    }
    return {0};
}

static Result SimStartGattOp(SimConn *c, u8 op_type, u16 uuid, int auth, bool is_cccd, u8 cccd_val = 1) {
    if (!c || !c->connected) return {4};
    // Exact bt_nso check: if channel state != 6 (IDLE), return 0x2a671 GATT_OPERATION_BUSY!
    if (c->gatt_state != 6) {
        ++g_sim.busy_0x2a671_count;
        return {ResultGattOperationBusyValue};
    }

    c->gatt_state = 1; // BUSY until response or disconnect

    // gatt_determine_sec_act (0xc56c0):
    if (auth == BtdrvGattAuthReqType_NoMitm) {
        if (c->sec_state == 1) {
            // 0xc56fc: GATT_SEC_PENDING -> hangs forever! Never sends ATT, never clears gatt_state!
            return {0};
        }
        if (!c->encrypted) {
            // 0xc5848 -> BTM_SetEncryption(sec_act=2) -> SMP_Pair!
            ++g_sim.smp_pair_count;
            if (g_sim.stale_ble_sec_records[BtNsoSim::AddrKey(c->addr)]) {
                // Stale BLE security record causes SMP failure -> GATT_AUTH_FAIL (0x89 = 137) and restores IDLE
                const u32 cid = c->conn_id;
                events::ScheduleCallback(80, [cid, op_type, uuid]() {
                    if (auto *live = g_sim.FindConnById(cid)) live->gatt_state = 6;
                    EventRecord ev{};
                    ev.type = BtdrvBleEventType_ClientNotify;
                    ev.info.client_notify = {137, cid, op_type, {uuid}};
                    events::ScheduleEvent(ev, 0);
                });
                return {0};
            }
            if (c->behavior == PeripheralBehavior::StadiaRequireEncViaAtt0F ||
                c->behavior == PeripheralBehavior::StadiaOnlyStage2CreateBondPlain) {
                // Simulate Stage 0 on-demand SMP timing out to verify fallback to Stage 1 / Stage 2
                return {0};
            }
            // Successful on-demand SMP encryption -> gatt_notify_enc_cmpl (0xc5640) resends queued ATT op!
            const u32 cid = c->conn_id;
            events::ScheduleCallback(220, [cid, op_type, uuid, is_cccd, cccd_val]() {
                auto *live = g_sim.FindConnById(cid);
                if (!live) return;
                live->encrypted = true;
                live->gatt_state = 6; // IDLE after ATT response
                if (is_cccd) {
                    if (cccd_val == 0) live->cccd_disabled_first = true;
                    else live->cccd_enabled = true;
                }
                EventRecord ev{};
                ev.type = BtdrvBleEventType_ClientNotify;
                ev.info.client_notify = {0, cid, op_type, {uuid}};
                if (uuid == GattCharPnpId) {
                    const u8 pnp[7] = {0x02, 0xD1, 0x18, 0x00, 0x94, 0x00, 0x01};
                    ev.info.client_notify.size = 7;
                    std::memcpy(ev.info.client_notify.data, pnp, 7);
                }
                events::ScheduleEvent(ev, 0);
            });
            return {0};
        }
    }

    // auth == None (0xc56e0: bypasses sec_state!) OR already encrypted: send ATT PDU directly!
    const u32 cid = c->conn_id;
    events::ScheduleCallback(30, [cid, op_type, uuid, is_cccd, cccd_val]() {
        auto *live = g_sim.FindConnById(cid);
        if (!live) return;
        // gatt_process_error_rsp / response always returns channel to IDLE (6)!
        live->gatt_state = 6;

        u32 att_result = 0;
        if (is_cccd && !live->encrypted) {
            if (live->behavior == PeripheralBehavior::StadiaRequireEncViaAtt05 ||
                live->behavior == PeripheralBehavior::StadiaStaleSecRecord) {
                att_result = 5;  // ATT Insufficient Authentication
            } else if (live->behavior == PeripheralBehavior::StadiaRequireEncViaAtt0F ||
                       live->behavior == PeripheralBehavior::StadiaOnlyStage2CreateBondPlain) {
                att_result = 15; // ATT Insufficient Encryption
            }
        }
        if (is_cccd && att_result == 0) {
            if (cccd_val == 0) live->cccd_disabled_first = true;
            else live->cccd_enabled = true;
        }

        EventRecord ev{};
        ev.type = BtdrvBleEventType_ClientNotify;
        ev.info.client_notify = {att_result, cid, op_type, {uuid}};
        if (att_result == 0 && uuid == GattCharPnpId) {
            const u8 pnp[7] = {0x02, 0xD1, 0x18, 0x00, 0x94, 0x00, 0x01};
            ev.info.client_notify.size = 7;
            std::memcpy(ev.info.client_notify.data, pnp, 7);
        } else if (att_result == 0 && uuid == GattCharDeviceName) {
            const char name[] = "StadiaQPRV-4967";
            ev.info.client_notify.size = sizeof(name) - 1;
            std::memcpy(ev.info.client_notify.data, name, sizeof(name) - 1);
        }
        events::ScheduleEvent(ev, 0);
    });
    return {0};
}

Result btdrvReadGattCharacteristic(u32 conn_id, bool, const BtdrvGattId *, const BtdrvGattId *attr, int auth) {
    return SimStartGattOp(g_sim.FindConnById(conn_id), BtdrvGattOperationType_CharacteristicRead, attr->uuid.value, auth, false);
}

Result btdrvRegisterGattNotification(u32 conn_id, bool, const BtdrvGattId *, const BtdrvGattId *) {
    auto *c = g_sim.FindConnById(conn_id);
    if (!c) return {4};
    // Faithful bt_nso 0x6408 -> 0x640c model: if g_gatt_manager already has a stale entry for
    // client_if (e.g. left over from a previous result=26 disconnect), bt_nso returns 0 WITHOUT
    // calling BSA_GattcRegisterNotification (0x19200)!
    if (g_sim.gatt_manager_notif_table[c->client_if]) {
        return {0};
    }
    g_sim.gatt_manager_notif_table[c->client_if] = true;
    c->notif_registered = true;
    return {0};
}

Result btdrvUnregisterGattNotification(u32 conn_id, bool, const BtdrvGattId *, const BtdrvGattId *) {
    auto *c = g_sim.FindConnById(conn_id);
    if (!c) return {4};
    g_sim.gatt_manager_notif_table[c->client_if] = false;
    c->notif_registered = false;
    return {0};
}

Result btdrvWriteGattDescriptor(u32 conn_id, bool, const BtdrvGattId *, const BtdrvGattId *attr, const BtdrvGattId *, const u8 *data, size_t size, int auth) {
    assert(size == 2 && (data[0] == 0 || data[0] == 1) && data[1] == 0);
    ++g_sim.cccd_write_count;
    return SimStartGattOp(g_sim.FindConnById(conn_id), BtdrvGattOperationType_DescriptorWrite, attr->uuid.value, auth, true, data[0]);
}

// Helper mirroring ble_hid_host.cpp's 3-stage Stadia strategy rotation
struct StadiaHostSim {
    u8 stage = 0;
    u8 failures_in_cycle = 0;
    u32 radio_cycles = 0;

    bool AttemptConnect(u32 slot, const ams::bluetooth::Address &addr, gatt::HidService *out_hid, u32 *out_conn_id, bool is_stadia) {
        u32 conn_id = InvalidConnId;
        if (R_FAILED(pairing::Connect(slot, static_cast<u8>(6 - slot), addr, &conn_id, TimeSpan::FromSeconds(15)))) {
            return false;
        }
        if (R_FAILED(gatt::Discover(slot, conn_id, addr))) {
            g_sim.DisconnectConn(conn_id);
            events::Pump(TimeSpan::FromMilliSeconds(50));
            return false;
        }

        pairing::SecurityStrategy sec = pairing::SecurityStrategy::CreateBond;
        gatt::SubscribeAuthMode sub = gatt::SubscribeAuthMode::StandardNoMitm;
        if (is_stadia) {
            if (stage == 0) {
                sec = pairing::SecurityStrategy::GattOnDemand;
                sub = gatt::SubscribeAuthMode::NoMitmCccdOnly;
            } else if (stage == 1) {
                sec = pairing::SecurityStrategy::DirectSetEncryption;
                sub = gatt::SubscribeAuthMode::PlainOnlyAfterBond;
            } else {
                sec = pairing::SecurityStrategy::CreateBond;
                sub = gatt::SubscribeAuthMode::PlainOnlyAfterBond;
            }
        }

        const bool sec_ok = pairing::PrepareSecurityWithStrategy(slot, addr, sec);
        const bool sub_ok = sec_ok && R_SUCCEEDED(gatt::Subscribe(slot, conn_id, out_hid, sub));
        if (!sub_ok) {
            if (is_stadia) {
                stage = static_cast<u8>((stage + 1) % 3);
                if (++failures_in_cycle >= 3) {
                    failures_in_cycle = 0;
                    ++radio_cycles;
                }
            }
            g_sim.DisconnectConn(conn_id);
            events::Pump(TimeSpan::FromMilliSeconds(50));
            static_cast<void>(pairing::ClearPeer(slot, addr));
            gatt::ResetSlot(slot);
            return false;
        }

        if (is_stadia) failures_in_cycle = 0;
        *out_conn_id = conn_id;
        return true;
    }
};

static void Test1_ReproduceOldV3BugAndProveFix() {
    // Prove the simulator reproduces the exact 16:14 ble.log failure (sec_state=1 stall -> 0x2a671 BUSY -> 28.40s zombie)
    g_sim.Reset();
    g_sim.peer_behaviors[BtNsoSim::AddrKey(StadiaAddr)] = PeripheralBehavior::StadiaRequireEncViaAtt05;
    u32 conn_id = InvalidConnId;
    assert(R_SUCCEEDED(pairing::Connect(0, 6, StadiaAddr, &conn_id, TimeSpan::FromSeconds(15))));
    assert(R_SUCCEEDED(gatt::Discover(0, conn_id, StadiaAddr)));
    assert(pairing::PrepareSecurity(0, StadiaAddr)); // CreateBond sets sec_state=1, early 3ms bond + 1500ms settle
    assert(g_sim.conns[conn_id].sec_state == 1);

    // Calling StandardNoMitm (old behavior) times out at 0x2A4A; our NEW zero-0x2a671 guard stops BEFORE CCCD!
    gatt::HidService hid{};
    const Result rc = gatt::Subscribe(0, conn_id, &hid, gatt::SubscribeAuthMode::StandardNoMitm);
    assert(svc::ResultTimedOut::Includes(rc));
    assert(g_sim.busy_0x2a671_count == 0); // NEW guard prevented sending CCCD into 0x2a671!
    assert(g_sim.cccd_write_count == 0);

    // Prove that if someone DID ignore the timeout like v3 did, bt_nso returns exact 0x2a671 BUSY
    const BtdrvGattId sid{{GattServiceHid}, 0}, cid_attr{{GattCharReport}, 0}, desc{{GattDescClientCharConfig}, 0};
    const u8 en[2] = {1, 0};
    const Result busy_rc = btdrvWriteGattDescriptor(conn_id, true, &sid, &cid_attr, &desc, en, 2, BtdrvGattAuthReqType_NoMitm);
    assert(busy_rc.value == ResultGattOperationBusyValue);
    assert(g_sim.busy_0x2a671_count == 1);

    // Fast-forward past 28.40s to verify the bt_nso zombie timer reproduces exactly
    events::Pump(TimeSpan::FromSeconds(30));
    assert(g_sim.zombie_timer_fires == 1);
}

static void Test2_Stage0_DirectNoMitmEncryptsEvenWhenPlainCccdWouldHaveReturnedOk() {
    // Reproduces real StadiaQPRV-4967 hardware finding from 22:40 ble.log:
    // Stadia returns result=0 to plaintext CCCD without encrypting, so Stage 0 MUST use NoMitmCccdOnly
    // to force bt_nso 0xc5848 -> BTM_SetEncryption -> SMP_Pair AND skip 0x2A50 (which stalled 28.36s).
    g_sim.Reset();
    g_sim.peer_behaviors[BtNsoSim::AddrKey(StadiaAddr)] = PeripheralBehavior::StadiaAcceptPlaintext;
    StadiaHostSim host;
    gatt::HidService hid{};
    u32 conn_id = InvalidConnId;
    assert(host.AttemptConnect(0, StadiaAddr, &hid, &conn_id, true));
    assert(host.stage == 0 && host.radio_cycles == 0);
    assert(g_sim.create_bond_count == 0);
    assert(g_sim.smp_pair_count == 1);      // Actually encrypted the link!
    assert(g_sim.cccd_write_count == 2);    // CCCD 0x0000 -> 0x0001 toggle with NoMitm!
    assert(g_sim.busy_0x2a671_count == 0);
    assert(g_sim.conns[conn_id].encrypted && g_sim.conns[conn_id].cccd_disabled_first && g_sim.conns[conn_id].cccd_enabled);
    assert(hid.write_auth_req == BtdrvGattAuthReqType_NoMitm);
    assert(!hid.has_pnp_id);                // Must NOT read 0x2A50 on Stadia!
    assert(hid.has_output_report && hid.output_report.handle == 0x0044 && hid.output_report.id.instance_id == 1);
    assert(hid.output_write_with_response == true);
}

static void Test3_Stage0_Att05DeviceDirectNoMitmSmp() {
    g_sim.Reset();
    g_sim.peer_behaviors[BtNsoSim::AddrKey(StadiaAddr)] = PeripheralBehavior::StadiaRequireEncViaAtt05;
    StadiaHostSim host;
    gatt::HidService hid{};
    u32 conn_id = InvalidConnId;
    assert(host.AttemptConnect(0, StadiaAddr, &hid, &conn_id, true));
    assert(host.stage == 0 && host.radio_cycles == 0);
    assert(g_sim.create_bond_count == 0);        // Never touched broken CreateBond!
    assert(g_sim.smp_pair_count == 1);           // Triggered real on-demand SMP_Pair via 0xc5848!
    assert(g_sim.cccd_write_count == 2);         // CCCD 0x0000 -> 0x0001 toggle -> SMP -> OK!
    assert(g_sim.busy_0x2a671_count == 0);       // Zero 0x2a671 BUSY!
    assert(g_sim.zombie_timer_fires == 0);
    assert(g_sim.conns[conn_id].encrypted && g_sim.conns[conn_id].cccd_disabled_first && g_sim.conns[conn_id].cccd_enabled);
    assert(hid.write_auth_req == BtdrvGattAuthReqType_NoMitm);
    assert(!hid.has_pnp_id);
}

static void Test4_Stage0TimeoutRotatesToStage1DirectSetEncryptionAndRemembers() {
    g_sim.Reset();
    g_sim.peer_behaviors[BtNsoSim::AddrKey(StadiaAddr)] = PeripheralBehavior::StadiaRequireEncViaAtt0F;
    StadiaHostSim host;
    gatt::HidService hid{};
    u32 conn_id = InvalidConnId;

    // Attempt 1 (Stage 0): plain -> ATT 0x0F -> NoMitm SMP times out -> clean abort, 0 BUSY, no radio cycle!
    assert(!host.AttemptConnect(0, StadiaAddr, &hid, &conn_id, true));
    assert(host.stage == 1 && host.radio_cycles == 0);
    assert(g_sim.busy_0x2a671_count == 0);

    // Attempt 2 (Stage 1): DirectSetEncryption ignores non-zero garbage w0, encrypts, PlainOnlyAfterBond writes CCCD!
    assert(host.AttemptConnect(0, StadiaAddr, &hid, &conn_id, true));
    assert(host.stage == 1 && host.radio_cycles == 0);
    assert(g_sim.ext_set_enc_count == 1);
    assert(g_sim.create_bond_count == 0);
    assert(g_sim.busy_0x2a671_count == 0);
    assert(g_sim.conns[conn_id].encrypted && g_sim.conns[conn_id].cccd_enabled);
    assert(hid.write_auth_req == BtdrvGattAuthReqType_None);

    // Disconnect and reconnect: remembers preferred stage 1 and succeeds on the FIRST attempt!
    g_sim.DisconnectConn(conn_id);
    events::Pump(TimeSpan::FromMilliSeconds(50));
    gatt::ResetSlot(0);
    u32 conn_id2 = InvalidConnId;
    assert(host.AttemptConnect(0, StadiaAddr, &hid, &conn_id2, true));
    assert(host.stage == 1 && g_sim.ext_set_enc_count == 2 && g_sim.busy_0x2a671_count == 0);
}

static void Test5_FallbackToStage2CreateBondWithPlainOnlyBypass() {
    g_sim.Reset();
    g_sim.peer_behaviors[BtNsoSim::AddrKey(StadiaAddr)] = PeripheralBehavior::StadiaOnlyStage2CreateBondPlain;
    StadiaHostSim host;
    gatt::HidService hid{};
    u32 conn_id = InvalidConnId;

    // Stage 0 fails -> Stage 1 fails -> Stage 2 (CreateBond + PlainOnlyAfterBond) bypasses sec_state==1 via 0xc56e0!
    assert(!host.AttemptConnect(0, StadiaAddr, &hid, &conn_id, true));
    assert(host.stage == 1 && host.radio_cycles == 0);
    assert(!host.AttemptConnect(0, StadiaAddr, &hid, &conn_id, true));
    assert(host.stage == 2 && host.radio_cycles == 0);
    assert(host.AttemptConnect(0, StadiaAddr, &hid, &conn_id, true));
    assert(host.stage == 2 && host.radio_cycles == 0);
    assert(g_sim.create_bond_count == 1);
    assert(g_sim.conns[conn_id].sec_state == 1); // Still 1 in bt_nso, yet CCCD succeeded via AuthReq=None!
    assert(g_sim.conns[conn_id].cccd_enabled);
    assert(g_sim.busy_0x2a671_count == 0);
    assert(hid.write_auth_req == BtdrvGattAuthReqType_None);

    // Even when the 28.40s zombie timer later fires, connection remains healthy!
    events::Pump(TimeSpan::FromSeconds(30));
    assert(g_sim.zombie_timer_fires == 1);
    assert(g_sim.conns[conn_id].connected && !events::IsSlotDisconnected(0));
}

static void Test6_StaleBleSecRecordRequiresIpsCmd4Cleanup() {
    // First prove that without IPS cmd 4, standard btdrvRemoveBond leaves stale BLE record and SMP fails (0x89)
    g_sim.Reset();
    g_sim.ips_cmd4_enabled = false;
    g_sim.peer_behaviors[BtNsoSim::AddrKey(StadiaAddr)] = PeripheralBehavior::StadiaStaleSecRecord;
    g_sim.stale_ble_sec_records[BtNsoSim::AddrKey(StadiaAddr)] = true;
    StadiaHostSim host_no_ips;
    gatt::HidService hid{};
    u32 cid = InvalidConnId;
    assert(!host_no_ips.AttemptConnect(0, StadiaAddr, &hid, &cid, true));
    assert(g_sim.stale_ble_sec_records[BtNsoSim::AddrKey(StadiaAddr)] == true);

    // Now enable IPS cmd 4: btdrvextSecRemoveDevice clears stale BLE security record -> Stage 0 succeeds immediately!
    g_sim.Reset();
    g_sim.ips_cmd4_enabled = true;
    g_sim.peer_behaviors[BtNsoSim::AddrKey(StadiaAddr)] = PeripheralBehavior::StadiaStaleSecRecord;
    g_sim.stale_ble_sec_records[BtNsoSim::AddrKey(StadiaAddr)] = true;
    StadiaHostSim host_with_ips;
    assert(host_with_ips.AttemptConnect(0, StadiaAddr, &hid, &cid, true));
    assert(g_sim.ext_remove_count >= 1);
    assert(g_sim.stale_ble_sec_records[BtNsoSim::AddrKey(StadiaAddr)] == false);
    assert(g_sim.conns[cid].cccd_enabled && g_sim.busy_0x2a671_count == 0);
}

static void Test7_CoexistenceXboxAndStadiaSimultaneous() {
    g_sim.Reset();
    g_sim.peer_behaviors[BtNsoSim::AddrKey(XboxAddr)] = PeripheralBehavior::XboxClassicBondThenNoMitm;
    g_sim.peer_behaviors[BtNsoSim::AddrKey(StadiaAddr)] = PeripheralBehavior::StadiaRequireEncViaAtt05;
    StadiaHostSim host;
    gatt::HidService xbox_hid{}, stadia_hid{};
    u32 xbox_cid = InvalidConnId, stadia_cid = InvalidConnId;

    // Slot 0: Xbox One BLE (CreateBond + StandardNoMitm)
    assert(host.AttemptConnect(0, XboxAddr, &xbox_hid, &xbox_cid, false));
    assert(xbox_hid.write_auth_req == BtdrvGattAuthReqType_NoMitm);
    assert(g_sim.conns[xbox_cid].cccd_enabled);

    // Slot 1: Stadia BLE (GattOnDemand + PlainThenNoMitm) while Xbox remains live
    assert(host.AttemptConnect(1, StadiaAddr, &stadia_hid, &stadia_cid, true));
    assert(stadia_hid.write_auth_req == BtdrvGattAuthReqType_NoMitm);
    assert(g_sim.conns[stadia_cid].cccd_enabled);
    assert(g_sim.conns[xbox_cid].connected && g_sim.conns[stadia_cid].connected);
    assert(g_sim.busy_0x2a671_count == 0);
}

static void Test8_StressReconnectDisconnectSleep50Cycles() {
    g_sim.Reset();
    const PeripheralBehavior behaviors[] = {
        PeripheralBehavior::StadiaAcceptPlaintext,
        PeripheralBehavior::StadiaRequireEncViaAtt05,
        PeripheralBehavior::StadiaRequireEncViaAtt0F,
        PeripheralBehavior::StadiaOnlyStage2CreateBondPlain,
        PeripheralBehavior::StadiaStaleSecRecord,
    };

    for (int iter = 0; iter < 50; ++iter) {
        const auto b = behaviors[iter % 5];
        g_sim.peer_behaviors[BtNsoSim::AddrKey(StadiaAddr)] = b;
        if (b == PeripheralBehavior::StadiaStaleSecRecord) {
            g_sim.stale_ble_sec_records[BtNsoSim::AddrKey(StadiaAddr)] = true;
        }

        // Inject mid-handshake drop every 7th iteration to test clean unwind + retry
        if (iter % 7 == 3) {
            g_sim.force_disconnect_after_ms_enabled = true;
            g_sim.force_disconnect_delay_ms = 60;
            StadiaHostSim drop_host;
            gatt::HidService dropped_hid{};
            u32 drop_cid = InvalidConnId;
            assert(!drop_host.AttemptConnect(0, StadiaAddr, &dropped_hid, &drop_cid, true));
            g_sim.force_disconnect_after_ms_enabled = false;
            assert(g_sim.busy_0x2a671_count == 0);
            continue;
        }

        StadiaHostSim host;
        gatt::HidService hid{};
        u32 cid = InvalidConnId;
        bool connected = false;
        for (int tries = 0; tries < 3 && !connected; ++tries) {
            connected = host.AttemptConnect(0, StadiaAddr, &hid, &cid, true);
        }
        assert(connected);
        assert(g_sim.conns[cid].cccd_enabled);
        assert(g_sim.busy_0x2a671_count == 0);

        // Verify a real Stadia button packet maps cleanly after connection
        const ams::controller::stadia::StadiaInputReport0x03 press_lr{0x08, 0x00, 0x06, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00};
        const auto mapped = ams::controller::stadia::MapInputReport0x03(press_lr, 0.5f);
        assert(mapped.buttons_mask == (ams::controller::stadia::ButtonBit::L | ams::controller::stadia::ButtonBit::R));

        // Clean disconnect before next cycle
        g_sim.DisconnectConn(cid);
        events::Pump(TimeSpan::FromMilliSeconds(50));
        static_cast<void>(pairing::ClearPeer(0, StadiaAddr));
        gatt::ResetSlot(0);
    }
    assert(g_sim.busy_0x2a671_count == 0);
}

static void Test9_StadiaProactiveSlaveSecReq30sTimeoutAndCmd6Fix() {
    // Part A: Reproduce exact real ble_v5_test1.log failure (without IPS cmd 6):
    // Stadia connects, sends Slave Security Request at +80ms -> SMP_ST_WAIT_APP_DECISION (sec_state=1).
    // Even if plaintext CCCD succeeds, SetEncryption stalls on sec_state==1, and at +30.15s bt_nso drops link (0x0016)!
    g_sim.Reset();
    g_sim.ips_cmd6_enabled = false;
    g_sim.peer_behaviors[BtNsoSim::AddrKey(StadiaAddr)] = PeripheralBehavior::StadiaProactiveSlaveSecReq;
    u32 cid = InvalidConnId;
    assert(R_SUCCEEDED(pairing::Connect(0, 6, StadiaAddr, &cid, TimeSpan::FromSeconds(15), true)));
    assert(R_SUCCEEDED(gatt::Discover(0, cid, StadiaAddr)));
    assert(g_sim.conns[cid].sec_state == 1 && g_sim.conns[cid].smp_state == 1);
    assert(pairing::PrepareSecurityWithStrategy(0, StadiaAddr, pairing::SecurityStrategy::DirectSetEncryption));
    gatt::HidService hid{};
    assert(R_SUCCEEDED(gatt::Subscribe(0, cid, &hid, gatt::SubscribeAuthMode::PlainOnlyAfterBond)));
    assert(!g_sim.conns[cid].encrypted);
    // Advance 32 seconds -> reproduces the exact 30.15s reason=0x0016 local host disconnect!
    events::Pump(TimeSpan::FromSeconds(32));
    assert(g_sim.smp_30s_timeout_disconnects == 1);
    assert(!g_sim.conns[cid].connected && events::IsSlotDisconnected(0));
    assert(events::GetSlotDisconnectReason(0) == 0x0016);

    // Part B: With IPS cmd 6 (BTA_DmBleSecurityGrant(addr, 0)) enabled:
    // Pairing mode (flags=0x05, clear_existing_bond=true): cmd 6 grants SMP request -> Legacy Just Works encrypts,
    // stores LTK, clears sec_state=0, subscribes CCCD, and stays connected >120 seconds with ZERO disconnects!
    g_sim.Reset();
    g_sim.ips_cmd6_enabled = true;
    g_sim.peer_behaviors[BtNsoSim::AddrKey(StadiaAddr)] = PeripheralBehavior::StadiaProactiveSlaveSecReq;
    u32 cid2 = InvalidConnId;
    assert(R_SUCCEEDED(pairing::Connect(0, 6, StadiaAddr, &cid2, TimeSpan::FromSeconds(15), true)));
    assert(R_SUCCEEDED(gatt::Discover(0, cid2, StadiaAddr)));
    assert(g_sim.conns[cid2].sec_state == 1 && g_sim.conns[cid2].smp_state == 1);
    assert(pairing::PrepareSecurityWithStrategy(0, StadiaAddr, pairing::SecurityStrategy::BleSecurityGrant));
    assert(g_sim.ext_sec_grant_count == 2);
    assert(g_sim.smp_pair_count == 1);
    assert(g_sim.conns[cid2].encrypted && g_sim.conns[cid2].sec_state == 0 && g_sim.conns[cid2].smp_state == 0);
    assert(g_sim.stored_ble_ltk[BtNsoSim::AddrKey(StadiaAddr)] == true);
    gatt::HidService hid2{};
    assert(R_SUCCEEDED(gatt::Subscribe(0, cid2, &hid2, gatt::SubscribeAuthMode::PlainOnlyAfterBond)));
    assert(g_sim.conns[cid2].cccd_enabled && g_sim.busy_0x2a671_count == 0);
    // Run 120 full seconds (4x past the 30.15s SMP timer) -> ZERO disconnects!
    events::Pump(TimeSpan::FromSeconds(120));
    assert(g_sim.smp_30s_timeout_disconnects == 0);
    assert(g_sim.conns[cid2].connected && !events::IsSlotDisconnected(0));

    // Part C: User turns Stadia off & turns it back on (flags=0x06 reconnect, LTK preserved in bt_nso):
    // DismissSlot preserves LTK (no ClearPeer), Connect uses clear_existing_bond=false ->
    // Slave Security Request hits stored LTK -> instant HCI LE Start Encryption, >120s ZERO disconnects!
    g_sim.DisconnectConn(cid2, 0x0013);
    events::Pump(TimeSpan::FromMilliSeconds(50));
    gatt::ResetSlot(0);
    assert(g_sim.stored_ble_ltk[BtNsoSim::AddrKey(StadiaAddr)] == true);

    u32 cid3 = InvalidConnId;
    assert(R_SUCCEEDED(pairing::Connect(0, 6, StadiaAddr, &cid3, TimeSpan::FromSeconds(15), false)));
    assert(R_SUCCEEDED(gatt::Discover(0, cid3, StadiaAddr)));
    assert(g_sim.ltk_fast_encrypt_count == 1);
    assert(g_sim.conns[cid3].encrypted && g_sim.conns[cid3].sec_state == 0 && g_sim.conns[cid3].smp_state == 0);
    assert(pairing::PrepareSecurityWithStrategy(0, StadiaAddr, pairing::SecurityStrategy::BleSecurityGrant));
    gatt::HidService hid3{};
    assert(R_SUCCEEDED(gatt::Subscribe(0, cid3, &hid3, gatt::SubscribeAuthMode::PlainOnlyAfterBond)));
    assert(g_sim.conns[cid3].cccd_enabled);
    events::Pump(TimeSpan::FromSeconds(120));
    assert(g_sim.smp_30s_timeout_disconnects == 0);
    assert(g_sim.conns[cid3].connected && !events::IsSlotDisconnected(0));
}

static void Test10_Result26DirtyNotifTableAndCccdToggleReconnect() {
    // Exact reproduction of real ble_v6_test1.log reconnect #1 -> result=26 disconnect -> reconnect #2 & #3!
    // Conn #1 ends with ClientConnection result=26 status=2 reason=0x0008 (supervision timeout), so bt_nso 0x2c26c
    // skips clearing g_gatt_manager's notification table. On Conn #2 (flags=0x06 LTK reconnect) and Conn #3
    // (flags=0x05 fresh pair), production gatt::Subscribe MUST:
    //   1. Call btdrvUnregisterGattNotification on the live conn_id first to purge the stale entry,
    //   2. Call btdrvRegisterGattNotification so BSA_GattcRegisterNotification (0x19200) actually runs,
    //   3. Toggle CCCD 0x0000 -> 0x0001 cleanly with zero 0x2a671 GATT_OPERATION_BUSY!
    g_sim.Reset();
    g_sim.ips_cmd6_enabled = true;
    g_sim.peer_behaviors[BtNsoSim::AddrKey(StadiaAddr)] = PeripheralBehavior::StadiaProactiveSlaveSecReq;

    // Conn #1 (flags=0x05 pairing mode)
    u32 cid1 = InvalidConnId;
    assert(R_SUCCEEDED(pairing::Connect(0, 6, StadiaAddr, &cid1, TimeSpan::FromSeconds(15), true)));
    assert(R_SUCCEEDED(gatt::Discover(0, cid1, StadiaAddr)));
    assert(pairing::PrepareSecurityWithStrategy(0, StadiaAddr, pairing::SecurityStrategy::BleSecurityGrant));
    gatt::HidService hid1{};
    assert(R_SUCCEEDED(gatt::Subscribe(0, cid1, &hid1, gatt::SubscribeAuthMode::PlainOnlyAfterBond)));
    assert(g_sim.conns[cid1].notif_registered);
    assert(g_sim.conns[cid1].cccd_disabled_first && g_sim.conns[cid1].cccd_enabled);
    assert(g_sim.busy_0x2a671_count == 0);

    // Conn #1 ends with result=26 (exact ble_v6_test1.log line 156!) -> leaves dirty entry in g_gatt_manager!
    g_sim.DisconnectConn(cid1, 0x0008, /*disc_result=*/26);
    events::Pump(TimeSpan::FromMilliSeconds(50));
    gatt::Unsubscribe(0, cid1); // fails on closed conn_id just like real DismissSlot!
    gatt::ResetSlot(0);
    assert(g_sim.gatt_manager_notif_table[6] == true); // Dirty entry STILL present!

    // Conn #2 (flags=0x06 auto-reconnect with preserved LTK)
    u32 cid2 = InvalidConnId;
    assert(R_SUCCEEDED(pairing::Connect(0, 6, StadiaAddr, &cid2, TimeSpan::FromSeconds(15), false)));
    assert(R_SUCCEEDED(gatt::Discover(0, cid2, StadiaAddr)));
    assert(pairing::PrepareSecurityWithStrategy(0, StadiaAddr, pairing::SecurityStrategy::BleSecurityGrant));
    gatt::HidService hid2{};
    assert(R_SUCCEEDED(gatt::Subscribe(0, cid2, &hid2, gatt::SubscribeAuthMode::PlainOnlyAfterBond)));
    // Must have purged dirty entry and truly called BSA_GattcRegisterNotification + toggled CCCD!
    assert(g_sim.conns[cid2].notif_registered);
    assert(g_sim.conns[cid2].cccd_disabled_first && g_sim.conns[cid2].cccd_enabled);
    assert(g_sim.busy_0x2a671_count == 0);

    // Conn #2 ends with result=0 -> Conn #3 (flags=0x05) also works cleanly
    g_sim.DisconnectConn(cid2, 0x0008, /*disc_result=*/0);
    events::Pump(TimeSpan::FromMilliSeconds(50));
    gatt::ResetSlot(0);

    u32 cid3 = InvalidConnId;
    assert(R_SUCCEEDED(pairing::Connect(0, 6, StadiaAddr, &cid3, TimeSpan::FromSeconds(15), true)));
    assert(R_SUCCEEDED(gatt::Discover(0, cid3, StadiaAddr)));
    assert(pairing::PrepareSecurityWithStrategy(0, StadiaAddr, pairing::SecurityStrategy::BleSecurityGrant));
    gatt::HidService hid3{};
    assert(R_SUCCEEDED(gatt::Subscribe(0, cid3, &hid3, gatt::SubscribeAuthMode::PlainOnlyAfterBond)));
    assert(g_sim.conns[cid3].notif_registered);
    assert(g_sim.conns[cid3].cccd_disabled_first && g_sim.conns[cid3].cccd_enabled);
    assert(g_sim.busy_0x2a671_count == 0);
}

int main() {
    Test1_ReproduceOldV3BugAndProveFix();
    Test2_Stage0_DirectNoMitmEncryptsEvenWhenPlainCccdWouldHaveReturnedOk();
    Test3_Stage0_Att05DeviceDirectNoMitmSmp();
    Test4_Stage0TimeoutRotatesToStage1DirectSetEncryptionAndRemembers();
    Test5_FallbackToStage2CreateBondWithPlainOnlyBypass();
    Test6_StaleBleSecRecordRequiresIpsCmd4Cleanup();
    Test7_CoexistenceXboxAndStadiaSimultaneous();
    Test8_StressReconnectDisconnectSleep50Cycles();
    Test9_StadiaProactiveSlaveSecReq30sTimeoutAndCmd6Fix();
    Test10_Result26DirtyNotifTableAndCccdToggleReconnect();
    std::cout << "HOS 22.5.0 bt_nso simulator end-to-end regressions (10 scenarios, 50 stress cycles + 120s soak) passed\n";
}
