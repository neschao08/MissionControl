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
#include "stadia_ble_controller.hpp"
#include "controller_management.hpp"
#include "../ble/ble_events.hpp"
#include "../ble/ble_log.hpp"
#include <algorithm>
#include <cstring>
#include <stratosphere.hpp>

namespace ams::controller {

    static_assert(stadia::ButtonBit::Y       == static_cast<u32>(SwitchButton::Y));
    static_assert(stadia::ButtonBit::X       == static_cast<u32>(SwitchButton::X));
    static_assert(stadia::ButtonBit::B       == static_cast<u32>(SwitchButton::B));
    static_assert(stadia::ButtonBit::A       == static_cast<u32>(SwitchButton::A));
    static_assert(stadia::ButtonBit::R       == static_cast<u32>(SwitchButton::R));
    static_assert(stadia::ButtonBit::ZR      == static_cast<u32>(SwitchButton::ZR));
    static_assert(stadia::ButtonBit::Minus   == static_cast<u32>(SwitchButton::Minus));
    static_assert(stadia::ButtonBit::Plus    == static_cast<u32>(SwitchButton::Plus));
    static_assert(stadia::ButtonBit::StickR  == static_cast<u32>(SwitchButton::StickR));
    static_assert(stadia::ButtonBit::StickL  == static_cast<u32>(SwitchButton::StickL));
    static_assert(stadia::ButtonBit::Home    == static_cast<u32>(SwitchButton::Home));
    static_assert(stadia::ButtonBit::Capture == static_cast<u32>(SwitchButton::Capture));
    static_assert(stadia::ButtonBit::Down    == static_cast<u32>(SwitchButton::Down));
    static_assert(stadia::ButtonBit::Up      == static_cast<u32>(SwitchButton::Up));
    static_assert(stadia::ButtonBit::Right   == static_cast<u32>(SwitchButton::Right));
    static_assert(stadia::ButtonBit::Left    == static_cast<u32>(SwitchButton::Left));
    static_assert(stadia::ButtonBit::L       == static_cast<u32>(SwitchButton::L));
    static_assert(stadia::ButtonBit::ZL      == static_cast<u32>(SwitchButton::ZL));

    Result StadiaBleController::HandleBleInputReport(const u8 *data, size_t size) {
        R_UNLESS(hos::GetVersion() >= hos::Version_9_0_0, sf::ResultNotSupported());

        constexpr size_t PayloadSize = sizeof(stadia::StadiaInputReport0x03);
        const bool is_stripped = (size == PayloadSize);
        const bool is_prefixed = (size == PayloadSize + 1) && (data[0] == stadia::InputReportId);
        R_UNLESS(is_stripped || is_prefixed, svc::ResultInvalidSize());

        std::scoped_lock lk(m_report_mutex);
        R_UNLESS(m_connected, svc::ResultInvalidState());

        auto &event = m_event_info.data_report.v9;
        event.res = 0;
        event.proto_mode = 0;
        event.addr = m_address;
        event.pad = 0;
        event.report.size = sizeof(stadia::StadiaInputReport0x03) + 1;

        if (is_stripped) {
            event.report.data[0] = stadia::InputReportId;
            std::memcpy(&event.report.data[1], data, PayloadSize);
        } else {
            std::memcpy(event.report.data, data, PayloadSize + 1);
        }

        EmulatedSwitchController::HandleDataReportEvent(&m_event_info);
        R_SUCCEED();
    }

    Result StadiaBleController::HandleOutputDataReport(const u8 *report_buffer, size_t size) {
        R_UNLESS(size >= 1, svc::ResultInvalidSize());

        const auto report_id = static_cast<SwitchHidReportId>(report_buffer[0]);
        if (report_id == SwitchHidReportId::BasicOutputReport) {
            std::scoped_lock lk(m_rumble_mutex);
            R_UNLESS(m_connected, svc::ResultInvalidState());
            R_RETURN(EmulatedSwitchController::HandleOutputDataReport(report_buffer, size));
        }

        std::scoped_lock lk(m_report_mutex, m_rumble_mutex);
        R_UNLESS(m_connected, svc::ResultInvalidState());
        const u8 subcmd = ((report_id == SwitchHidReportId::CommandOutputReport) && (size > 10)) ? report_buffer[10] : 0;
        if ((report_id == SwitchHidReportId::CommandOutputReport) && (m_cmd_log_count < 20) && ble::log::IsEnabled()) {
            ++m_cmd_log_count;
            const u8 arg0 = (size > 11) ? report_buffer[11] : 0;
            ble::log::Write("stadia cmd #%u conn_id=%u subcmd=0x%02x arg0=0x%02x size=%zu",
                m_cmd_log_count, m_conn_id, subcmd, arg0, size);
        }
        R_TRY(EmulatedSwitchController::HandleOutputDataReport(report_buffer, size));
        if ((report_id == SwitchHidReportId::CommandOutputReport) && ((subcmd == 0x04) || (subcmd == 0x30))) {
            u8 basic_report[sizeof(SwitchHidBasicInputReport)] = {};
            const size_t basic_size = this->FillBasicInputReport(basic_report, sizeof(basic_report));
            static_cast<void>(this->InjectInputReport(basic_report, basic_size));
        }
        R_SUCCEED();
    }

    void StadiaBleController::DisconnectBle() {
        std::scoped_lock lk(m_report_mutex, m_rumble_mutex);
        m_connected = false;
    }

    void StadiaBleController::ParseInputReport(const u8 *report_buffer, size_t size) {
        if (size < sizeof(stadia::StadiaReportData)) {
            return;
        }

        const auto *report = reinterpret_cast<const stadia::StadiaReportData *>(report_buffer);
        if (report->id != stadia::InputReportId) {
            return;
        }

        const auto mapped = stadia::MapInputReport0x03(report->input0x03, m_trigger_threshold);
        m_left_stick.SetValues(mapped.left_x, mapped.left_y);
        m_right_stick.SetValues(mapped.right_x, mapped.right_y);
        m_buttons = SwitchButtons(mapped.buttons_mask);
    }

    Result StadiaBleController::SetVibration(const SwitchMotorData *motor_data) {
        if (!m_has_output_report) {
            R_SUCCEED();
        }

        const s64 min_single_interval_ns = m_output_write_with_response
            ? TimeSpan::FromMilliSeconds(48).GetNanoSeconds()
            : TimeSpan::FromMilliSeconds(32).GetNanoSeconds();
        constexpr s64 MultiControllerMinRumbleIntervalNs   = TimeSpan::FromMilliSeconds(64).GetNanoSeconds();
        constexpr s64 HeavyMultiControllerRumbleIntervalNs = TimeSpan::FromMilliSeconds(96).GetNanoSeconds();

        const float strong_amp = std::clamp(std::max(motor_data->left_motor.low_band_amp, motor_data->right_motor.low_band_amp), 0.0f, 1.0f);
        const float weak_amp   = std::clamp(std::max(motor_data->left_motor.high_band_amp, motor_data->right_motor.high_band_amp), 0.0f, 1.0f);

        const u16 strong = stadia::MapRumbleMagnitude(strong_amp, true);
        const u16 weak   = stadia::MapRumbleMagnitude(weak_amp, false);

        u32 att_err = 0;
        if (ble::events::TakeRumbleWriteError(m_conn_id, &att_err)) {
            if ((m_rumble_fallback_stage == 0) && (m_write_auth_req == BtdrvGattAuthReqType_None)) {
                m_write_auth_req = BtdrvGattAuthReqType_NoMitm;
                m_rumble_fallback_stage = 1;
                ble::log::Write("stadia rumble conn_id=%u att_err=%u; switching write auth None -> NoMitm", m_conn_id, att_err);
            } else if ((m_rumble_fallback_stage < 2) && m_output_write_with_response) {
                m_output_write_with_response = false;
                m_rumble_fallback_stage = 2;
                ble::log::Write("stadia rumble conn_id=%u att_err=%u; switching write mode WithRsp -> NoRsp", m_conn_id, att_err);
            }
        }

        const s64 now_tick = os::GetSystemTick().GetInt64Value();
        const TimeSpan elapsed_time = (m_last_rumble_tick > 0)
            ? os::Tick(now_tick - m_last_rumble_tick).ToTimeSpan()
            : TimeSpan::FromSeconds(1);

        const s64 elapsed_ms = elapsed_time.GetMilliSeconds();
        const s64 elapsed_ns = elapsed_time.GetNanoSeconds();

        const bool is_stop     = (strong == 0) && (weak == 0);
        const bool was_stopped = (m_last_rumble_strong == 0) && (m_last_rumble_weak == 0);

        if (is_stop) {
            if (was_stopped) {
                R_SUCCEED();
            }
        } else {
            if ((strong == m_last_rumble_strong) && (weak == m_last_rumble_weak) && (elapsed_ms < 180)) {
                R_SUCCEED();
            }
            const size_t total_controllers = controller::GetConnectedControllerCount();
            const s64 min_rumble_interval_ns = (total_controllers >= 4) ? HeavyMultiControllerRumbleIntervalNs
                                             : (total_controllers >= 2) ? MultiControllerMinRumbleIntervalNs
                                             : min_single_interval_ns;
            if (elapsed_ns < min_rumble_interval_ns) {
                R_SUCCEED();
            }
        }

        stadia::StadiaReportData report = {};
        report.id = stadia::OutputReportId;
        report.output0x05.strong_magnitude = strong;
        report.output0x05.weak_magnitude   = weak;

        const Result rc = this->WriteDataReport(&report, sizeof(stadia::StadiaOutputReport0x05) + 1);
        if ((m_rumble_log_count < 8) && ble::log::IsEnabled()) {
            ++m_rumble_log_count;
            ble::log::Write("stadia rumble #%u conn_id=%u strong=0x%04x weak=0x%04x auth=%u with_rsp=%u rc=0x%x",
                m_rumble_log_count, m_conn_id, strong, weak, m_write_auth_req, m_output_write_with_response ? 1u : 0u, rc.GetValue());
        }
        if (R_FAILED(rc)) {
            // On transient GATT BUSY (0x2a671) during Write-With-Response rumble:
            // back off non-stop updates to avoid hammering the GATT channel, while leaving
            // m_last_rumble_* non-zero on stop failures so the next frame retries stop.
            if (!is_stop) {
                m_last_rumble_tick = now_tick;
            }
            R_RETURN(rc);
        }
        m_last_rumble_strong = strong;
        m_last_rumble_weak   = weak;
        m_last_rumble_tick   = now_tick;
        R_SUCCEED();
    }

    Result StadiaBleController::WriteDataReport(const void *report_buffer, size_t size) {
        R_UNLESS(ble::events::IsRadioEnabled(), svc::ResultInvalidState());
        if (!m_has_output_report) {
            R_SUCCEED();
        }
        R_UNLESS(size >= 1, svc::ResultInvalidSize());
        const u8 *data = static_cast<const u8 *>(report_buffer);
        R_RETURN(btdrvWriteGattCharacteristic(m_conn_id, m_service_primary, &m_service_id, &m_output_report_id, &data[1], size - 1, static_cast<BtdrvGattAuthReqType>(m_write_auth_req), m_output_write_with_response));
    }

}
