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
#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace ams::controller::stadia {

    constexpr uint8_t InputReportId  = 0x03;
    constexpr uint8_t OutputReportId = 0x05;

    constexpr uint16_t VendorId  = 0x18D1;
    constexpr uint16_t ProductId = 0x9400;

    // Bit positions matching SwitchButton in switch_buttons.hpp
    namespace ButtonBit {
        constexpr uint32_t Y       = 1u << 0;
        constexpr uint32_t X       = 1u << 1;
        constexpr uint32_t B       = 1u << 2;
        constexpr uint32_t A       = 1u << 3;
        constexpr uint32_t R       = 1u << 6;
        constexpr uint32_t ZR      = 1u << 7;
        constexpr uint32_t Minus   = 1u << 8;
        constexpr uint32_t Plus    = 1u << 9;
        constexpr uint32_t StickR  = 1u << 10;
        constexpr uint32_t StickL  = 1u << 11;
        constexpr uint32_t Home    = 1u << 12;
        constexpr uint32_t Capture = 1u << 13;
        constexpr uint32_t Down    = 1u << 16;
        constexpr uint32_t Up      = 1u << 17;
        constexpr uint32_t Right   = 1u << 18;
        constexpr uint32_t Left    = 1u << 19;
        constexpr uint32_t L       = 1u << 22;
        constexpr uint32_t ZL      = 1u << 23;
    }

    struct __attribute__((packed)) StadiaInputReport0x03 {
        uint8_t dpad;
        uint8_t buttons0;
        uint8_t buttons1;
        uint8_t left_stick_x;
        uint8_t left_stick_y;
        uint8_t right_stick_x;
        uint8_t right_stick_y;
        uint8_t left_trigger;
        uint8_t right_trigger;
        uint8_t consumer;
    };
    static_assert(sizeof(StadiaInputReport0x03) == 10);

    struct __attribute__((packed)) StadiaOutputReport0x05 {
        uint16_t strong_magnitude;
        uint16_t weak_magnitude;
    };
    static_assert(sizeof(StadiaOutputReport0x05) == 4);

    struct __attribute__((packed)) StadiaReportData {
        uint8_t id;
        union {
            StadiaInputReport0x03 input0x03;
            StadiaOutputReport0x05 output0x05;
        };
    };
    static_assert(sizeof(StadiaReportData) == 1 + sizeof(StadiaInputReport0x03));

    struct MappedStadiaState {
        uint32_t buttons_mask = 0;
        uint16_t left_x       = 2048;
        uint16_t left_y       = 2048;
        uint16_t right_x      = 2048;
        uint16_t right_y      = 2048;
    };

    inline constexpr uint16_t ConvertAxis(uint8_t raw, bool invert) {
        uint8_t v = std::clamp<uint8_t>(raw, 1, 255);
        if (invert) {
            v = static_cast<uint8_t>(256 - v);
        }
        return static_cast<uint16_t>((static_cast<uint32_t>(v - 1) * 4095 + 127) / 254);
    }

    inline constexpr MappedStadiaState MapInputReport0x03(const StadiaInputReport0x03 &src, float trigger_threshold) {
        MappedStadiaState out;

        out.left_x  = ConvertAxis(src.left_stick_x, false);
        out.left_y  = ConvertAxis(src.left_stick_y, true);
        out.right_x = ConvertAxis(src.right_stick_x, false);
        out.right_y = ConvertAxis(src.right_stick_y, true);

        const uint8_t hat = src.dpad & 0x0F;
        if (hat == 0 || hat == 1 || hat == 7) out.buttons_mask |= ButtonBit::Up;
        if (hat == 1 || hat == 2 || hat == 3) out.buttons_mask |= ButtonBit::Right;
        if (hat == 3 || hat == 4 || hat == 5) out.buttons_mask |= ButtonBit::Down;
        if (hat == 5 || hat == 6 || hat == 7) out.buttons_mask |= ButtonBit::Left;

        // Positional face buttons (East=A, South=B, West=Y, North=X)
        if (src.buttons1 & 0x20) out.buttons_mask |= ButtonBit::A;
        if (src.buttons1 & 0x40) out.buttons_mask |= ButtonBit::B;
        if (src.buttons1 & 0x10) out.buttons_mask |= ButtonBit::Y;
        if (src.buttons1 & 0x08) out.buttons_mask |= ButtonBit::X;

        // Top row center buttons: right(menu 0x20)=+, left(options 0x40)=-
        if (src.buttons0 & 0x20) out.buttons_mask |= ButtonBit::Plus;
        if (src.buttons0 & 0x40) out.buttons_mask |= ButtonBit::Minus;

        // Second row center buttons: right(capture 0x01)=Home, left(assistant 0x02)=Capture
        // Center Stadia button (buttons0 & 0x10) is intentionally left unmapped.
        if (src.buttons0 & 0x01) out.buttons_mask |= ButtonBit::Home;
        if (src.buttons0 & 0x02) out.buttons_mask |= ButtonBit::Capture;

        // Shoulders and stick clicks
        if (src.buttons1 & 0x04) out.buttons_mask |= ButtonBit::L;
        if (src.buttons1 & 0x02) out.buttons_mask |= ButtonBit::R;
        if (src.buttons1 & 0x01) out.buttons_mask |= ButtonBit::StickL;
        if (src.buttons0 & 0x80) out.buttons_mask |= ButtonBit::StickR;

        // Analog triggers (digital trigger bits 0x04/0x08 in buttons0 are ignored)
        const float left_norm  = static_cast<float>(src.left_trigger) / 255.0f;
        const float right_norm = static_cast<float>(src.right_trigger) / 255.0f;
        if (left_norm > trigger_threshold)  out.buttons_mask |= ButtonBit::ZL;
        if (right_norm > trigger_threshold) out.buttons_mask |= ButtonBit::ZR;

        return out;
    }

}
