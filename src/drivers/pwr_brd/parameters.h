/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

/**
 * @file parameters.h
 *
 * defines the list of parameters that are used within the power board module
 *
 * @author Amir Melzer <amir.melzer@mavt.ethz.ch>
 */

#include <px4_config.h>

#include <systemlib/param/param.h>
#include <mathlib/mathlib.h>

namespace pwr_brd
{

struct Parameters {
	float bias_cal_term_system_volt;
	float SF_cal_term_system_volt;
	float bias_cal_term_servo_volt;
	float SF_cal_term_servo_volt;
	float bias_cal_term_mot_l_amp;
	float SF_cal_term_mot_l_amp;
	float bias_cal_term_mot_r_amp;
	float SF_cal_term_mot_r_amp;
	float bias_cal_term_digital_amp;
	float SF_cal_term_digital_amp;
	float bias_cal_term_analog_amp;
	float SF_cal_term_analog_amp;
	float bias_cal_term_ext_amp;
	float SF_cal_term_ext_amp;
	float bias_cal_term_digital_volt;
	float SF_cal_term_digital_volt;
	float bias_cal_term_aux_amp;
	float SF_cal_term_aux_amp;
	int32_t pwr_brd_led_blink_int;
	int32_t pwr_brd_led_blink_number;
	int32_t pwr_brd_led_power_1;
	int32_t pwr_brd_led_power_2;
	int32_t pwr_brd_led_power_3;
	int32_t pwr_brd_led_power_4;
};

struct ParameterHandles {
	param_t bias_cal_term_system_volt;
	param_t SF_cal_term_system_volt;
	param_t bias_cal_term_servo_volt;
	param_t SF_cal_term_servo_volt;
	param_t bias_cal_term_mot_l_amp;
	param_t SF_cal_term_mot_l_amp;
	param_t bias_cal_term_mot_r_amp;
	param_t SF_cal_term_mot_r_amp;
	param_t bias_cal_term_digital_amp;
	param_t SF_cal_term_digital_amp;
	param_t bias_cal_term_analog_amp;
	param_t SF_cal_term_analog_amp;
	param_t bias_cal_term_ext_amp;
	param_t SF_cal_term_ext_amp;
	param_t bias_cal_term_digital_volt;
	param_t SF_cal_term_digital_volt;
	param_t bias_cal_term_aux_amp;
	param_t SF_cal_term_aux_amp;
	param_t pwr_brd_led_blink_int;
	param_t pwr_brd_led_blink_number;
	param_t pwr_brd_led_power_1;
	param_t pwr_brd_led_power_2;
	param_t pwr_brd_led_power_3;
	param_t pwr_brd_led_power_4;
};

/**
 * initialize ParameterHandles struct
 */
void initialize_parameter_handles(ParameterHandles &parameter_handles);


/**
 * Read out the parameters using the handles into the parameters struct.
 * @return 0 on success, <0 on error
 */
int update_parameters(const ParameterHandles &parameter_handles, Parameters &parameters);

} /* namespace pwr_brd */
