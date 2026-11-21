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
#include "switch_vibration_processor.hpp"
#include "emulated_switch_controller.hpp"

namespace ams::controller {

    namespace {

        bool GetNextDecodedValue(SwitchHdRumbleDecoder *decoder, const SwitchEncodedVibrationSamples *encoded_samples, SwitchVibrationValues *out_sample) {
            SwitchVibrationSamples decoded_samples;
            decoder->DecodeSamples(encoded_samples, &decoded_samples);
            if (decoded_samples.count > 0) {
                // Use peak amplitude per band across multi-sample packets so short pulses in later sub-samples are not dropped.
                *out_sample = decoded_samples.samples[0];
                for (u8 i = 1; i < decoded_samples.count; ++i) {
                    const auto &s = decoded_samples.samples[i];
                    if (s.low_band_amp > out_sample->low_band_amp) {
                        out_sample->low_band_amp  = s.low_band_amp;
                        out_sample->low_band_freq = s.low_band_freq;
                    }
                    if (s.high_band_amp > out_sample->high_band_amp) {
                        out_sample->high_band_amp  = s.high_band_amp;
                        out_sample->high_band_freq = s.high_band_freq;
                    }
                }
            } else {
                // Repeat the current sample if no new samples were decoded
                decoder->GetCurrentOutputValue(out_sample);
            }

            return decoded_samples.count > 0;
        }

    }

    void SwitchVibrationProcessor::ProcessMotorData(const SwitchEncodedMotorData *encoded_motor_data) {
        if (!m_enabled) return;
        SwitchMotorData motor_data;

        bool samples_available = GetNextDecodedValue(&m_decoder_left,  &encoded_motor_data->left_motor,  &motor_data.left_motor)
                               | GetNextDecodedValue(&m_decoder_right, &encoded_motor_data->right_motor, &motor_data.right_motor);

        if (samples_available) {
            R_DISCARD(m_controller->SetVibration(&motor_data));
        }
    }

}
