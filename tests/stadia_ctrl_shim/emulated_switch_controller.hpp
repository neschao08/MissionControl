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
        float low_band_freq = 160.0f;
        float high_band_amp = 0.0f;
        float high_band_freq = 320.0f;
    };
    struct SwitchMotorData {
        SwitchMotorChannel left_motor;
        SwitchMotorChannel right_motor;
    };
    struct SwitchEncodedMotorData {
        u8 bytes[8];
    };

    class EmulatedSwitchController {
        public:
            EmulatedSwitchController(bluetooth::Address addr, HardwareID id) : m_address(addr), m_id(id) {}
            virtual ~EmulatedSwitchController() = default;

            virtual Result Initialize() { return {0}; }
            virtual Result HandleBleInputReport(const u8 *data, size_t size) = 0;
            virtual Result SendConnectRumble(u16 strong, u16 weak) { (void)strong; (void)weak; return {0}; }
            virtual void ServiceBleTasks() {}
            virtual void OnRumbleWriteComplete(u32 result) { (void)result; }
            virtual Result HandleOutputDataReport(const u8 *report_buffer, size_t size) {
                ++m_output_calls;
                m_last_output_id = size > 0 ? report_buffer[0] : 0;
                return {0};
            }
            virtual Result InjectInputReport(const u8 *report_buffer, size_t size) {
                ++m_inject_calls;
                if (report_buffer && size >= sizeof(u32) + 9) {
                    std::memcpy(&m_last_injected_buttons, &report_buffer[1], sizeof(u32));
                    std::memcpy(&m_last_injected_lx, &report_buffer[5], sizeof(u16));
                    std::memcpy(&m_last_injected_ly, &report_buffer[7], sizeof(u16));
                }
                return {0};
            }
            virtual void DisconnectBle() = 0;

            void HandleDataReportEvent(const bluetooth::HidReportEventInfo *info) {
                ++m_data_report_events;
                this->ParseInputReport(info->data_report.v9.report.data, info->data_report.v9.report.size);
            }

            // Public test hooks to drive protected methods directly
            Result TestSetVibration(const SwitchMotorData *m) { return this->SetVibration(m); }
            Result TestOnVibrationPacket(const SwitchEncodedMotorData *enc, const SwitchMotorData *peak, const SwitchMotorData *sustained, bool samples_available) {
                return this->OnVibrationPacket(enc, peak, sustained, samples_available);
            }
            Result TestCancelVibration() { return this->CancelVibration(); }
            SwitchControllerColours TestColours() const { return m_test_colours; }
            u32 TestColourWrites() const { return m_colour_writes; }
            u8 TestSpiDeviceType() const { return m_test_spi_device_type; }
            u32 TestSpiDeviceTypeWrites() const { return m_spi_device_type_writes; }
            u32 TestButtons() const { return m_buttons.mask; }
            u16 TestLeftX() const { return m_left_stick.x; }
            u16 TestLeftY() const { return m_left_stick.y; }
            u16 TestRightX() const { return m_right_stick.x; }
            u16 TestRightY() const { return m_right_stick.y; }
            u32 TestDataEvents() const { return m_data_report_events; }
            u32 TestOutputCalls() const { return m_output_calls; }
            u32 TestInjectCalls() const { return m_inject_calls; }
            void TestResetInjectCalls() { m_inject_calls = 0; }
            u32 TestLastInjectedButtons() const { return m_last_injected_buttons; }
            u16 TestLastInjectedLeftX() const { return m_last_injected_lx; }
            u16 TestLastInjectedLeftY() const { return m_last_injected_ly; }
            u8 TestDeviceType() const { return m_device_info.device_type; }
            void TestSetTriggerThreshold(float t) { m_trigger_threshold = t; }

        protected:
            Result WriteControllerColours(RGBColour body, RGBColour buttons, RGBColour left_grip, RGBColour right_grip) {
                m_test_colours = {body, buttons, left_grip, right_grip};
                ++m_colour_writes;
                return {0};
            }
            Result WriteSpiDeviceType(u8 device_type) {
                m_test_spi_device_type = device_type;
                ++m_spi_device_type_writes;
                return {0};
            }
            size_t FillBasicInputReport(u8 *report_buffer, size_t size) {
                const size_t n = std::min<size_t>(size, sizeof(SwitchHidBasicInputReport));
                std::memset(report_buffer, 0, n);
                if (n >= sizeof(u32) + 9) {
                    report_buffer[0] = 0x30;
                    std::memcpy(&report_buffer[1], &m_buttons.mask, sizeof(u32));
                    std::memcpy(&report_buffer[5], &m_left_stick.x, sizeof(u16));
                    std::memcpy(&report_buffer[7], &m_left_stick.y, sizeof(u16));
                }
                return n;
            }
            size_t FillInputReport(u8 *report_buffer, size_t size) {
                return this->FillBasicInputReport(report_buffer, size);
            }
            void ModifyInputReport(u8 *report_buffer, size_t size) {
                (void)report_buffer;
                (void)size;
            }
            virtual void ParseInputReport(const u8 *report_buffer, size_t size) = 0;
            virtual Result SetVibration(const SwitchMotorData *motor_data) = 0;
            virtual Result OnVibrationPacket(const SwitchEncodedMotorData *encoded_data, const SwitchMotorData *peak_data, const SwitchMotorData *sustained_data, bool samples_available) {
                (void)encoded_data;
                (void)sustained_data;
                if (samples_available) {
                    return this->SetVibration(peak_data);
                }
                return {0};
            }
            virtual Result CancelVibration() { return {0}; }
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
            u32 m_last_injected_buttons = 0;
            u16 m_last_injected_lx = 2048;
            u16 m_last_injected_ly = 2048;
            u32 m_colour_writes = 0;
            SwitchControllerColours m_test_colours = {};
            u8 m_test_spi_device_type = 0;
            u32 m_spi_device_type_writes = 0;
            u8 m_last_output_id = 0;
    };
}
