#pragma once
#include "stratosphere.hpp"

namespace ams::controller {
    struct HardwareID { u16 vid; u16 pid; };

    struct RGBColour {
        u8 r;
        u8 g;
        u8 b;
    };

    struct SwitchControllerColours {
        RGBColour body;
        RGBColour buttons;
        RGBColour left_grip;
        RGBColour right_grip;
    };

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

            virtual Result Initialize() { return {0}; }
            virtual Result HandleBleInputReport(const u8 *data, size_t size) = 0;
            virtual Result SendConnectRumble(u16 strong, u16 weak) { (void)strong; (void)weak; return {0}; }
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
            SwitchControllerColours TestColours() const { return m_test_colours; }
            u32 TestColourWrites() const { return m_colour_writes; }
            u32 TestButtons() const { return m_buttons.mask; }
            u16 TestLeftX() const { return m_left_stick.x; }
            u16 TestLeftY() const { return m_left_stick.y; }
            u16 TestRightX() const { return m_right_stick.x; }
            u16 TestRightY() const { return m_right_stick.y; }
            u32 TestDataEvents() const { return m_data_report_events; }
            u32 TestOutputCalls() const { return m_output_calls; }
            u32 TestInjectCalls() const { return m_inject_calls; }
            u8 TestDeviceType() const { return m_device_info.device_type; }
            void TestSetTriggerThreshold(float t) { m_trigger_threshold = t; }

        protected:
            Result WriteControllerColours(RGBColour body, RGBColour buttons, RGBColour left_grip, RGBColour right_grip) {
                m_test_colours = {body, buttons, left_grip, right_grip};
                ++m_colour_writes;
                return {0};
            }
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
            struct {
                u8 device_type = 0x06;
                const char *device_name = "Lic Pro Controller";
            } m_device_info;
            struct {
                bool enabled = true;
                bool GetEnabled() const { return enabled; }
            } m_vibration_processor;
            SwitchAnalogStick m_left_stick;
            SwitchAnalogStick m_right_stick;
            SwitchButtons m_buttons;
            float m_trigger_threshold = 0.5f;
            u32 m_data_report_events = 0;
            u32 m_output_calls = 0;
            u32 m_inject_calls = 0;
            u32 m_colour_writes = 0;
            SwitchControllerColours m_test_colours = {};
            u8 m_last_output_id = 0;
    };
}
