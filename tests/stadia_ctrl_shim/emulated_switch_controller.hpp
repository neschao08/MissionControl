#pragma once
#include "stratosphere.hpp"

namespace ams::controller {
    struct HardwareID { u16 vid; u16 pid; };

    enum class SwitchHidReportId : u8 {
        CommandOutputReport = 0x01,
        BasicOutputReport   = 0x10,
    };

    struct SwitchHidBasicInputReport {
        u8 data[0x31];
    };

    enum class SwitchButton : u32 {
        Y       = 1u << 0,
        X       = 1u << 1,
        B       = 1u << 2,
        A       = 1u << 3,
        R       = 1u << 6,
        ZR      = 1u << 7,
        Minus   = 1u << 8,
        Plus    = 1u << 9,
        StickR  = 1u << 10,
        StickL  = 1u << 11,
        Home    = 1u << 12,
        Capture = 1u << 13,
        Down    = 1u << 16,
        Up      = 1u << 17,
        Right   = 1u << 18,
        Left    = 1u << 19,
        L       = 1u << 22,
        ZL      = 1u << 23,
    };

    struct SwitchButtons {
        u32 mask = 0;
        SwitchButtons() = default;
        explicit SwitchButtons(u32 m) : mask(m) {}
    };

    struct SwitchAnalogStick {
        u16 x = 2048, y = 2048;
        void SetValues(u16 nx, u16 ny) { x = nx; y = ny; }
    };

    struct SwitchMotorChannel {
        float low_band_amp = 0.0f;
        float high_band_amp = 0.0f;
    };
    struct SwitchMotorData {
        SwitchMotorChannel left_motor;
        SwitchMotorChannel right_motor;
    };

    class EmulatedSwitchController {
        public:
            EmulatedSwitchController(bluetooth::Address addr, HardwareID id) : m_address(addr), m_id(id) {}
            virtual ~EmulatedSwitchController() = default;

            virtual Result HandleBleInputReport(const u8 *data, size_t size) = 0;
            virtual Result HandleOutputDataReport(const u8 *report_buffer, size_t size) {
                ++m_output_calls;
                m_last_output_id = size > 0 ? report_buffer[0] : 0;
                return {0};
            }
            virtual Result InjectInputReport(const u8 *report_buffer, size_t size) {
                (void)report_buffer;
                (void)size;
                ++m_inject_calls;
                return {0};
            }
            virtual void DisconnectBle() = 0;

            void HandleDataReportEvent(const bluetooth::HidReportEventInfo *info) {
                ++m_data_report_events;
                this->ParseInputReport(info->data_report.v9.report.data, info->data_report.v9.report.size);
            }

            // Public test hooks to drive protected methods directly
            Result TestSetVibration(const SwitchMotorData *m) { return this->SetVibration(m); }
            u32 TestButtons() const { return m_buttons.mask; }
            u16 TestLeftX() const { return m_left_stick.x; }
            u16 TestLeftY() const { return m_left_stick.y; }
            u16 TestRightX() const { return m_right_stick.x; }
            u16 TestRightY() const { return m_right_stick.y; }
            u32 TestDataEvents() const { return m_data_report_events; }
            u32 TestOutputCalls() const { return m_output_calls; }
            u32 TestInjectCalls() const { return m_inject_calls; }
            void TestSetTriggerThreshold(float t) { m_trigger_threshold = t; }

        protected:
            size_t FillBasicInputReport(u8 *report_buffer, size_t size) {
                const size_t n = std::min<size_t>(size, sizeof(SwitchHidBasicInputReport));
                std::memset(report_buffer, 0, n);
                if (n > 0) report_buffer[0] = 0x30;
                return n;
            }
            virtual void ParseInputReport(const u8 *report_buffer, size_t size) = 0;
            virtual Result SetVibration(const SwitchMotorData *motor_data) = 0;
            virtual Result WriteDataReport(const void *report_buffer, size_t size) = 0;

            bluetooth::Address m_address;
            HardwareID m_id;
            SwitchAnalogStick m_left_stick;
            SwitchAnalogStick m_right_stick;
            SwitchButtons m_buttons;
            float m_trigger_threshold = 0.5f;
            u32 m_data_report_events = 0;
            u32 m_output_calls = 0;
            u32 m_inject_calls = 0;
            u8 m_last_output_id = 0;
    };
}
