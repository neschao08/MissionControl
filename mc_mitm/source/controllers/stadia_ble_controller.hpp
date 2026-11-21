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
#include "emulated_switch_controller.hpp"
#include "stadia_mapping.hpp"
#include <atomic>

namespace ams::controller {

    /*
     * Google Stadia Controller connected over Bluetooth LE (HID over GATT).
     *
     * Input notifications arrive on Report ID 0x03 (10-byte payload after HOGP strips the report id).
     * Output rumble writes go to the writable HID Report characteristic (Report ID 0x05, 4-byte LE motors).
     */
    class StadiaBleController : public EmulatedSwitchController {

        public:
            static constexpr const HardwareID hardware_ids[] = {
                {stadia::VendorId, stadia::ProductId},
            };

            StadiaBleController(bluetooth::Address address, u32 conn_id, const BtdrvGattId &service_id, bool service_primary, const BtdrvGattId &output_report_id, bool has_output_report = true, bool output_write_with_response = true, u8 write_auth_req = BtdrvGattAuthReqType_NoMitm)
            : EmulatedSwitchController(address, {stadia::VendorId, stadia::ProductId})
            , m_conn_id(conn_id)
            , m_service_id(service_id)
            , m_service_primary(service_primary)
            , m_output_report_id(output_report_id)
            , m_has_output_report(has_output_report)
            , m_output_write_with_response(output_write_with_response)
            , m_write_auth_req(write_auth_req) {
                // Report as Official Pro Controller (0x03) so HOS enables 0x6050 custom colors and full HD rumble.
                m_device_info.device_type = 0x03;
                m_device_info.device_name = "Pro Controller";
            }

            Result Initialize() override;
            Result HandleBleInputReport(const u8 *data, size_t size) override;
            Result HandleOutputDataReport(const u8 *report_buffer, size_t size) override;
            Result SendConnectRumble(u16 strong, u16 weak) override;
            void ServiceBleTasks() override;
            void OnRumbleWriteComplete(u32 result) override;
            void DisconnectBle() override;

        protected:
            void ParseInputReport(const u8 *report_buffer, size_t size) override;
            Result SetVibration(const SwitchMotorData *motor_data) override;
            Result WriteDataReport(const void *report_buffer, size_t size) override;

        private:
            Result TryWriteRumbleLocked(u16 strong, u16 weak, s64 now_tick);
            s64 GetMinRumbleIntervalNs() const;

            os::SdkMutex m_report_mutex;
            os::SdkMutex m_rumble_mutex;
            u32 m_conn_id;
            BtdrvGattId m_service_id;
            bool m_service_primary;
            BtdrvGattId m_output_report_id;
            bool m_has_output_report;
            bool m_output_write_with_response;
            u8 m_write_auth_req;
            bool m_connected = true;

            s64 m_last_input_report_tick = 0;

            u16 m_desired_rumble_strong = 0;
            u16 m_desired_rumble_weak = 0;
            u16 m_sent_rumble_strong = 0;
            u16 m_sent_rumble_weak = 0;
            s64 m_last_rumble_tick = 0;
            s64 m_rumble_min_stop_tick = 0;
            s64 m_rumble_expire_tick = 0;
            bool m_rumble_pending = false;
            std::atomic<bool> m_rumble_in_flight{false};
            std::atomic<s64> m_rumble_in_flight_tick{0};
            std::atomic<u32> m_pending_att_error{0};

            u32 m_cmd_log_count = 0;
            u32 m_basic_out_log_count = 0;
            u32 m_other_out_log_count = 0;
            u32 m_rumble_log_count = 0;
            u8  m_rumble_fallback_stage = 0;

            bluetooth::HidReportEventInfo m_event_info = {};
    };

}
