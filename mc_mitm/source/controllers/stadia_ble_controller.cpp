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

    Result StadiaBleController::Initialize() {
        R_TRY(EmulatedSwitchController::Initialize());

        // Align SPI device type byte (0x6012) with official Pro Controller alongside 0x6050 colours.
        const Result type_rc = this->WriteSpiDeviceType(0x03);
        if (R_FAILED(type_rc) && ble::log::IsEnabled()) {
            ble::log::Write("stadia device_type write failed conn_id=%u rc=0x%x", m_conn_id, type_rc.GetValue());
        }

        // Apply pure-white minimalist Pro icon colours and overwrite any cached spi_flash.bin 0x6050 entry.
        const Result colour_rc = this->WriteControllerColours(
            {0xf5, 0xf5, 0xf5}, // body
            {0xb4, 0xb4, 0xb4}, // buttons
            {0xf5, 0xf5, 0xf5}, // left_grip
            {0xf5, 0xf5, 0xf5}  // right_grip
        );
        if (R_FAILED(colour_rc) && ble::log::IsEnabled()) {
            ble::log::Write("stadia colour write failed conn_id=%u rc=0x%x", m_conn_id, colour_rc.GetValue());
        }

        R_SUCCEED();
    }

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
        m_last_input_report_tick = os::GetSystemTick().GetInt64Value();
        R_SUCCEED();
    }

    Result StadiaBleController::HandleOutputDataReport(const u8 *report_buffer, size_t size) {
        R_UNLESS(size >= 1, svc::ResultInvalidSize());

        const auto report_id = static_cast<SwitchHidReportId>(report_buffer[0]);
        if (report_id == SwitchHidReportId::BasicOutputReport) {
            std::scoped_lock lk(m_rumble_mutex);
            R_UNLESS(m_connected, svc::ResultInvalidState());
            if ((size >= 10) && (m_basic_out_log_count < 16) && ble::log::IsEnabled()) {
                // Ignore neutral rumble [00 01 40 40 00 01 40 40] so only real vibration packets take log slots.
                const bool is_neutral = (report_buffer[2] == 0x00) && (report_buffer[3] == 0x01) &&
                                        (report_buffer[4] == 0x40) && (report_buffer[5] == 0x40) &&
                                        (report_buffer[6] == 0x00) && (report_buffer[7] == 0x01) &&
                                        (report_buffer[8] == 0x40) && (report_buffer[9] == 0x40);
                if (!is_neutral) {
                    ++m_basic_out_log_count;
                    ble::log::Write("stadia basic-out #%u conn_id=%u en=%u m=%02x%02x%02x%02x_%02x%02x%02x%02x",
                        m_basic_out_log_count, m_conn_id, m_vibration_processor.GetEnabled() ? 1u : 0u,
                        report_buffer[2], report_buffer[3], report_buffer[4], report_buffer[5],
                        report_buffer[6], report_buffer[7], report_buffer[8], report_buffer[9]);
                }
            }
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
        } else if ((report_id != SwitchHidReportId::CommandOutputReport) && (m_other_out_log_count < 8) && ble::log::IsEnabled()) {
            ++m_other_out_log_count;
            ble::log::Write("stadia other-out #%u conn_id=%u id=0x%02x size=%zu",
                m_other_out_log_count, m_conn_id, static_cast<u8>(report_id), size);
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
        m_rumble_pending = false;
        m_find_controller_active = false;
        m_desired_rumble_strong = 0;
        m_desired_rumble_weak = 0;
        m_rumble_expire_tick = 0;
        m_rumble_min_stop_tick = 0;
        m_rumble_in_flight.store(false, std::memory_order_release);
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

    void StadiaBleController::OnRumbleWriteComplete(u32 result) {
        m_rumble_in_flight.store(false, std::memory_order_release);
        if (result != 0) {
            m_pending_att_error.store(result, std::memory_order_release);
        }
    }

    s64 StadiaBleController::GetMinRumbleIntervalNs() const {
        const s64 min_single_interval_ns = m_output_write_with_response
            ? TimeSpan::FromMilliSeconds(36).GetNanoSeconds()
            : TimeSpan::FromMilliSeconds(32).GetNanoSeconds();
        constexpr s64 MultiControllerMinRumbleIntervalNs   = TimeSpan::FromMilliSeconds(64).GetNanoSeconds();
        constexpr s64 HeavyMultiControllerRumbleIntervalNs = TimeSpan::FromMilliSeconds(96).GetNanoSeconds();

        const size_t total_controllers = controller::GetConnectedControllerCount();
        if (total_controllers >= 4) return HeavyMultiControllerRumbleIntervalNs;
        if (total_controllers >= 2) return MultiControllerMinRumbleIntervalNs;
        return min_single_interval_ns;
    }

    Result StadiaBleController::TryWriteRumbleLocked(u16 strong, u16 weak, s64 now_tick) {
        u32 att_err = m_pending_att_error.exchange(0, std::memory_order_acq_rel);
        if (att_err == 0) {
            static_cast<void>(ble::events::TakeRumbleWriteError(m_conn_id, &att_err));
        } else {
            // Drain any duplicate error recorded for the same ATT completion.
            u32 unused_err = 0;
            static_cast<void>(ble::events::TakeRumbleWriteError(m_conn_id, &unused_err));
        }
        if (att_err != 0) {
            if ((m_rumble_fallback_stage == 0) && (m_write_auth_req == BtdrvGattAuthReqType_None)) {
                m_write_auth_req = BtdrvGattAuthReqType_NoMitm;
                m_rumble_fallback_stage = 1;
                ble::log::Write("stadia rumble conn_id=%u att_err=%u; switching write auth None -> NoMitm", m_conn_id, att_err);
            } else if ((m_rumble_fallback_stage < 2) && m_output_write_with_response) {
                m_output_write_with_response = false;
                m_rumble_fallback_stage = 2;
                m_rumble_in_flight.store(false, std::memory_order_release);
                ble::log::Write("stadia rumble conn_id=%u att_err=%u; switching write mode WithRsp -> NoRsp", m_conn_id, att_err);
            }
        }

        stadia::StadiaReportData report = {};
        report.id = stadia::OutputReportId;
        report.output0x05.strong_magnitude = strong;
        report.output0x05.weak_magnitude   = weak;

        if (m_output_write_with_response) {
            m_rumble_in_flight_tick.store(now_tick, std::memory_order_release);
            m_rumble_in_flight.store(true, std::memory_order_release);
        }

        const Result rc = this->WriteDataReport(&report, sizeof(stadia::StadiaOutputReport0x05) + 1);
        if ((m_rumble_log_count < 32) && ble::log::IsEnabled()) {
            ++m_rumble_log_count;
            ble::log::Write("stadia rumble #%u conn_id=%u strong=0x%04x weak=0x%04x auth=%u with_rsp=%u rc=0x%x",
                m_rumble_log_count, m_conn_id, strong, weak, m_write_auth_req, m_output_write_with_response ? 1u : 0u, rc.GetValue());
        }

        const bool is_stop = (strong == 0) && (weak == 0);
        if (R_FAILED(rc)) {
            m_rumble_in_flight.store(false, std::memory_order_release);
            m_rumble_pending = true;
            if (!is_stop) {
                m_last_rumble_tick = now_tick;
            }
            ble::events::WakeHostLoop();
            R_RETURN(rc);
        }

        const bool was_stopped = (m_sent_rumble_strong == 0) && (m_sent_rumble_weak == 0);
        if (is_stop) {
            m_rumble_min_stop_tick = 0;
            m_rumble_expire_tick = 0;
        } else if (was_stopped) {
            m_rumble_min_stop_tick = now_tick + os::ConvertToTick(TimeSpan::FromMilliSeconds(45)).GetInt64Value();
        }

        m_sent_rumble_strong = strong;
        m_sent_rumble_weak   = weak;
        m_last_rumble_tick   = now_tick;
        if ((m_desired_rumble_strong == strong) && (m_desired_rumble_weak == weak)) {
            m_rumble_pending = false;
        }
        R_SUCCEED();
    }

    stadia::StadiaRumbleMagnitudes StadiaBleController::MapMotorData(const SwitchMotorData *motor_data) {
        return stadia::MapSwitchRumbleToStadia(
            motor_data->left_motor.low_band_amp,  motor_data->left_motor.low_band_freq,
            motor_data->left_motor.high_band_amp, motor_data->left_motor.high_band_freq,
            motor_data->right_motor.low_band_amp,  motor_data->right_motor.low_band_freq,
            motor_data->right_motor.high_band_amp, motor_data->right_motor.high_band_freq
        );
    }

    Result StadiaBleController::ApplyRumbleTargetLocked(u16 peak_strong, u16 peak_weak, u16 sustained_strong, u16 sustained_weak, s64 expire_ms) {
        const s64 now_tick = os::GetSystemTick().GetInt64Value();
        if (m_rumble_in_flight.load(std::memory_order_acquire)) {
            const s64 in_flight_tick = m_rumble_in_flight_tick.load(std::memory_order_acquire);
            if ((in_flight_tick > 0) && (os::Tick(now_tick - in_flight_tick).ToTimeSpan().GetMilliSeconds() >= 150)) {
                m_rumble_in_flight.store(false, std::memory_order_release);
            }
        }
        const bool in_flight   = m_rumble_in_flight.load(std::memory_order_acquire);
        const bool is_stop     = (peak_strong == 0) && (peak_weak == 0);
        const bool was_stopped = (m_sent_rumble_strong == 0) && (m_sent_rumble_weak == 0);

        if (is_stop) {
            m_desired_rumble_strong = 0;
            m_desired_rumble_weak   = 0;
            m_rumble_expire_tick    = 0;
            if (was_stopped && !in_flight) {
                m_rumble_pending = false;
                R_SUCCEED();
            }
            if (in_flight || ((m_rumble_min_stop_tick > 0) && (now_tick < m_rumble_min_stop_tick))) {
                m_rumble_pending = true;
                ble::events::WakeHostLoop();
                R_SUCCEED();
            }
            R_RETURN(this->TryWriteRumbleLocked(0, 0, now_tick));
        }

        const TimeSpan elapsed_time = (m_last_rumble_tick > 0)
            ? os::Tick(now_tick - m_last_rumble_tick).ToTimeSpan()
            : TimeSpan::FromSeconds(1);

        // Single-packet transient kick (non-zero peak followed immediately by silent tail in the same packet).
        if ((sustained_strong == 0) && (sustained_weak == 0)) {
            if (in_flight || (elapsed_time.GetNanoSeconds() < this->GetMinRumbleIntervalNs())) {
                m_desired_rumble_strong = peak_strong;
                m_desired_rumble_weak   = peak_weak;
                m_rumble_expire_tick    = now_tick + os::ConvertToTick(TimeSpan::FromMilliSeconds(60)).GetInt64Value();
                m_rumble_pending        = true;
                ble::events::WakeHostLoop();
                R_SUCCEED();
            }

            m_desired_rumble_strong = peak_strong;
            m_desired_rumble_weak   = peak_weak;
            const Result rc = this->TryWriteRumbleLocked(peak_strong, peak_weak, now_tick);
            if (R_FAILED(rc)) {
                m_rumble_expire_tick = now_tick + os::ConvertToTick(TimeSpan::FromMilliSeconds(90)).GetInt64Value();
                R_RETURN(rc);
            }

            // Crisp cutoff: schedule zero stop right after the 45ms minimum kick window.
            m_desired_rumble_strong = 0;
            m_desired_rumble_weak   = 0;
            m_rumble_min_stop_tick  = now_tick + os::ConvertToTick(TimeSpan::FromMilliSeconds(45)).GetInt64Value();
            m_rumble_expire_tick    = m_rumble_min_stop_tick;
            m_rumble_pending        = true;
            ble::events::WakeHostLoop();
            R_SUCCEED();
        }

        // Sustained rumble or active multi-packet vibration.
        m_desired_rumble_strong = peak_strong;
        m_desired_rumble_weak   = peak_weak;
        m_rumble_expire_tick    = now_tick + os::ConvertToTick(TimeSpan::FromMilliSeconds(expire_ms)).GetInt64Value();

        if (in_flight) {
            if ((peak_strong != m_sent_rumble_strong) || (peak_weak != m_sent_rumble_weak)) {
                m_rumble_pending = true;
                ble::events::WakeHostLoop();
            }
            R_SUCCEED();
        }

        if ((peak_strong == m_sent_rumble_strong) && (peak_weak == m_sent_rumble_weak) && (elapsed_time.GetMilliSeconds() < 1500)) {
            m_rumble_pending = false;
            R_SUCCEED();
        }

        if (elapsed_time.GetNanoSeconds() < this->GetMinRumbleIntervalNs()) {
            if ((peak_strong != m_sent_rumble_strong) || (peak_weak != m_sent_rumble_weak)) {
                m_rumble_pending = true;
                ble::events::WakeHostLoop();
            }
            R_SUCCEED();
        }

        R_RETURN(this->TryWriteRumbleLocked(peak_strong, peak_weak, now_tick));
    }

    Result StadiaBleController::CancelVibration() {
        if (!m_has_output_report) {
            R_SUCCEED();
        }

        m_find_controller_active = false;
        m_desired_rumble_strong  = 0;
        m_desired_rumble_weak    = 0;
        m_rumble_expire_tick     = 0;
        m_rumble_min_stop_tick   = 0;

        const s64 now_tick = os::GetSystemTick().GetInt64Value();
        if (m_rumble_in_flight.load(std::memory_order_acquire)) {
            const s64 in_flight_tick = m_rumble_in_flight_tick.load(std::memory_order_acquire);
            if ((in_flight_tick > 0) && (os::Tick(now_tick - in_flight_tick).ToTimeSpan().GetMilliSeconds() >= 150)) {
                m_rumble_in_flight.store(false, std::memory_order_release);
            }
        }
        const bool in_flight   = m_rumble_in_flight.load(std::memory_order_acquire);
        const bool was_stopped = (m_sent_rumble_strong == 0) && (m_sent_rumble_weak == 0);

        if (was_stopped && !in_flight) {
            m_rumble_pending = false;
            R_SUCCEED();
        }
        if (in_flight) {
            m_rumble_pending = true;
            ble::events::WakeHostLoop();
            R_SUCCEED();
        }
        R_RETURN(this->TryWriteRumbleLocked(0, 0, now_tick));
    }

    Result StadiaBleController::OnVibrationPacket(const SwitchEncodedMotorData *encoded_data, const SwitchMotorData *peak_data, const SwitchMotorData *sustained_data, bool samples_available) {
        if (!m_has_output_report) {
            R_SUCCEED();
        }

        if (samples_available) {
            if ((encoded_data != nullptr) && stadia::IsFindControllerMotorBytes(reinterpret_cast<const uint8_t *>(encoded_data))) {
                if (!m_find_controller_active && (m_find_log_count < 16) && ble::log::IsEnabled()) {
                    ++m_find_log_count;
                    ble::log::Write("stadia find-controller start conn_id=%u", m_conn_id);
                }
                m_find_controller_active = true;
                R_RETURN(this->ApplyRumbleTargetLocked(0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 800));
            }

            if (m_find_controller_active) {
                if ((m_find_log_count < 16) && ble::log::IsEnabled()) {
                    ++m_find_log_count;
                    ble::log::Write("stadia find-controller stop conn_id=%u", m_conn_id);
                }
                m_find_controller_active = false;
            }

            const auto peak      = MapMotorData(peak_data);
            const auto sustained = MapMotorData(sustained_data);
            R_RETURN(this->ApplyRumbleTargetLocked(peak.strong, peak.weak, sustained.strong, sustained.weak, 600));
        }

        // PacketType == 0: keep alive held rumble or flush scheduled 45ms transient cutoff.
        if (m_find_controller_active) {
            R_RETURN(this->ApplyRumbleTargetLocked(0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 800));
        }

        const auto sustained = MapMotorData(sustained_data);
        if ((sustained.strong != 0) || (sustained.weak != 0)) {
            R_RETURN(this->ApplyRumbleTargetLocked(sustained.strong, sustained.weak, sustained.strong, sustained.weak, 600));
        }

        if (m_rumble_pending && (m_desired_rumble_strong == 0) && (m_desired_rumble_weak == 0)) {
            const s64 now_tick = os::GetSystemTick().GetInt64Value();
            if (m_rumble_in_flight.load(std::memory_order_acquire)) {
                const s64 in_flight_tick = m_rumble_in_flight_tick.load(std::memory_order_acquire);
                if ((in_flight_tick > 0) && (os::Tick(now_tick - in_flight_tick).ToTimeSpan().GetMilliSeconds() >= 150)) {
                    m_rumble_in_flight.store(false, std::memory_order_release);
                }
            }
            if (!m_rumble_in_flight.load(std::memory_order_acquire) && ((m_rumble_min_stop_tick == 0) || (now_tick >= m_rumble_min_stop_tick))) {
                if ((m_sent_rumble_strong == 0) && (m_sent_rumble_weak == 0)) {
                    m_rumble_pending = false;
                    R_SUCCEED();
                }
                R_RETURN(this->TryWriteRumbleLocked(0, 0, now_tick));
            }
        }

        R_SUCCEED();
    }

    Result StadiaBleController::SetVibration(const SwitchMotorData *motor_data) {
        if (!m_has_output_report) {
            R_SUCCEED();
        }
        const auto mapped = MapMotorData(motor_data);
        R_RETURN(this->ApplyRumbleTargetLocked(mapped.strong, mapped.weak, mapped.strong, mapped.weak, 200));
    }

    void StadiaBleController::ServiceBleTasks() {
        // 1. Preserve held-state 33Hz input keepalive while idle so HOS hid continuously pumps output reports.
        {
            std::scoped_lock lk(m_report_mutex);
            if (!m_connected) {
                return;
            }
            const s64 now_tick = os::GetSystemTick().GetInt64Value();
            const s64 elapsed_ms = (m_last_input_report_tick > 0)
                ? os::Tick(now_tick - m_last_input_report_tick).ToTimeSpan().GetMilliSeconds()
                : 1000;
            if ((m_last_input_report_tick == 0) || (elapsed_ms >= 30)) {
                u8 report_buffer[0x200] = {};
                const size_t report_size = this->FillInputReport(report_buffer, sizeof(report_buffer));
                if (report_size > 0) {
                    this->ModifyInputReport(report_buffer, report_size);
                    static_cast<void>(this->InjectInputReport(report_buffer, report_size));
                    m_last_input_report_tick = now_tick;
                }
            }
        }

        // 2. Service pending rumble retries/stops and the 200ms auto-stop safety watchdog.
        {
            std::scoped_lock lk(m_rumble_mutex);
            if (!m_connected || !m_has_output_report) {
                return;
            }

            const s64 now_tick = os::GetSystemTick().GetInt64Value();
            if (m_rumble_in_flight.load(std::memory_order_acquire)) {
                const s64 in_flight_tick = m_rumble_in_flight_tick.load(std::memory_order_acquire);
                if ((in_flight_tick > 0) && (os::Tick(now_tick - in_flight_tick).ToTimeSpan().GetMilliSeconds() >= 150)) {
                    m_rumble_in_flight.store(false, std::memory_order_release);
                }
            }

            if ((m_rumble_expire_tick > 0) && (now_tick >= m_rumble_expire_tick)) {
                m_desired_rumble_strong = 0;
                m_desired_rumble_weak   = 0;
                m_rumble_expire_tick    = 0;
                m_find_controller_active = false;
                if ((m_sent_rumble_strong != 0) || (m_sent_rumble_weak != 0)) {
                    m_rumble_pending = true;
                }
            }

            if (!m_rumble_pending || m_rumble_in_flight.load(std::memory_order_acquire)) {
                return;
            }

            if ((m_desired_rumble_strong == 0) && (m_desired_rumble_weak == 0)) {
                if ((m_sent_rumble_strong == 0) && (m_sent_rumble_weak == 0)) {
                    m_rumble_pending = false;
                    return;
                }
                if ((m_rumble_min_stop_tick > 0) && (now_tick < m_rumble_min_stop_tick)) {
                    return;
                }
                static_cast<void>(this->TryWriteRumbleLocked(0, 0, now_tick));
                return;
            }

            const s64 elapsed_ns = (m_last_rumble_tick > 0)
                ? os::Tick(now_tick - m_last_rumble_tick).ToTimeSpan().GetNanoSeconds()
                : TimeSpan::FromSeconds(1).GetNanoSeconds();
            if (elapsed_ns >= this->GetMinRumbleIntervalNs()) {
                static_cast<void>(this->TryWriteRumbleLocked(m_desired_rumble_strong, m_desired_rumble_weak, now_tick));
            }
        }
    }

    Result StadiaBleController::SendConnectRumble(u16 strong, u16 weak) {
        std::scoped_lock lk(m_rumble_mutex);
        R_UNLESS(m_connected, svc::ResultInvalidState());
        if (!m_has_output_report) {
            R_SUCCEED();
        }

        stadia::StadiaReportData report = {};
        report.id = stadia::OutputReportId;
        report.output0x05.strong_magnitude = strong;
        report.output0x05.weak_magnitude   = weak;

        Result rc = this->WriteDataReport(&report, sizeof(stadia::StadiaOutputReport0x05) + 1);
        if (ble::log::IsEnabled()) {
            ble::log::Write("stadia connect-rumble conn_id=%u strong=0x%04x weak=0x%04x auth=%u with_rsp=%u rc=0x%x",
                m_conn_id, strong, weak, m_write_auth_req, m_output_write_with_response ? 1u : 0u, rc.GetValue());
        }
        // Retry once with NoMitm auth if an encrypted peer requires authenticated writes (ignoring GATT BUSY 0x2a671).
        if (R_FAILED(rc) && (m_write_auth_req == BtdrvGattAuthReqType_None) && (rc.GetValue() != 0x2a671)) {
            m_write_auth_req = BtdrvGattAuthReqType_NoMitm;
            rc = this->WriteDataReport(&report, sizeof(stadia::StadiaOutputReport0x05) + 1);
            if (ble::log::IsEnabled()) {
                ble::log::Write("stadia connect-rumble retry conn_id=%u auth=%u with_rsp=%u rc=0x%x",
                    m_conn_id, m_write_auth_req, m_output_write_with_response ? 1u : 0u, rc.GetValue());
            }
        }
        R_RETURN(rc);
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
