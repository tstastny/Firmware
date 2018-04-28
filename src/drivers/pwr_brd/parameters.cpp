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

/**
 * @file parameters.cpp
 *
 * @author Amir Melzer amir.melzer@mavt.ethz.ch>
 */

#include "parameters.h"

namespace pwr_brd
{

void initialize_parameter_handles(ParameterHandles &parameter_handles)
{
	parameter_handles.bias_cal_term_system_volt = param_find("PWRBRD_SYSV_B");
	parameter_handles.SF_cal_term_system_volt = param_find("PWRBRD_SYSV_SF");
	parameter_handles.bias_cal_term_servo_volt = param_find("PWRBRD_SERVOV_B");
	parameter_handles.SF_cal_term_servo_volt = param_find("PWRBRD_SERVO_SF");
	parameter_handles.bias_cal_term_mot_l_amp = param_find("PWRBRD_MOTL_B");
	parameter_handles.SF_cal_term_mot_l_amp = param_find("PWRBRD_MOTL_SF");
	parameter_handles.bias_cal_term_mot_r_amp = param_find("PWRBRD_MOTR_B");
	parameter_handles.SF_cal_term_mot_r_amp = param_find("PWRBRD_MOTR_SF");
	parameter_handles.bias_cal_term_digital_amp = param_find("PWRBRD_DIGI_B");
	parameter_handles.SF_cal_term_digital_amp = param_find("PWRBRD_DIGI_SF");
	parameter_handles.bias_cal_term_analog_amp = param_find("PWRBRD_ANALOG_B");
	parameter_handles.SF_cal_term_analog_amp = param_find("PWRBRD_ANALOG_SF");
	parameter_handles.bias_cal_term_ext_amp = param_find("PWRBRD_EXT_B");
	parameter_handles.SF_cal_term_ext_amp = param_find("PWRBRD_EXT_SF");
	parameter_handles.bias_cal_term_digital_volt = param_find("PWRBRD_DIGIV_B");
	parameter_handles.SF_cal_term_digital_volt = param_find("PWRBRD_DIGIV_SF");
	parameter_handles.bias_cal_term_aux_amp = param_find("PWRBRD_AUX_B");
	parameter_handles.SF_cal_term_aux_amp = param_find("PWRBRD_AUX_SF");
	parameter_handles.pwr_brd_led_blink_int = param_find("PWRBRD_LED_B_INT");
	parameter_handles.pwr_brd_led_blink_number = param_find("PWRBRD_LED_B_NUM");
	parameter_handles.pwr_brd_led_power_1 = param_find("PWRBRD_LED_POW_1");
	parameter_handles.pwr_brd_led_power_2 = param_find("PWRBRD_LED_POW_2");
	parameter_handles.pwr_brd_led_power_3 = param_find("PWRBRD_LED_POW_3");
	parameter_handles.pwr_brd_led_power_4 = param_find("PWRBRD_LED_POW_4");
}

int update_parameters(const ParameterHandles &parameter_handles, Parameters &parameters)
{
	int ret = PX4_OK;
	const char *paramerr = "FAIL PARM LOAD";

	if (param_get(parameter_handles.bias_cal_term_system_volt, &(parameters.bias_cal_term_system_volt)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.SF_cal_term_system_volt, &(parameters.SF_cal_term_system_volt)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.bias_cal_term_servo_volt, &(parameters.bias_cal_term_servo_volt)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.SF_cal_term_servo_volt, &(parameters.SF_cal_term_servo_volt)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.bias_cal_term_mot_l_amp, &(parameters.bias_cal_term_mot_l_amp)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.SF_cal_term_mot_l_amp, &(parameters.SF_cal_term_mot_l_amp)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.bias_cal_term_mot_r_amp, &(parameters.bias_cal_term_mot_r_amp)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.SF_cal_term_mot_r_amp, &(parameters.SF_cal_term_mot_r_amp)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.bias_cal_term_digital_amp, &(parameters.bias_cal_term_digital_amp)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.SF_cal_term_digital_amp, &(parameters.SF_cal_term_digital_amp)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.bias_cal_term_analog_amp, &(parameters.bias_cal_term_analog_amp)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.SF_cal_term_analog_amp, &(parameters.SF_cal_term_analog_amp)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.bias_cal_term_ext_amp, &(parameters.bias_cal_term_ext_amp)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.SF_cal_term_ext_amp, &(parameters.SF_cal_term_ext_amp)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.bias_cal_term_digital_volt, &(parameters.bias_cal_term_digital_volt)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.SF_cal_term_digital_volt, &(parameters.SF_cal_term_digital_volt)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.bias_cal_term_aux_amp, &(parameters.bias_cal_term_aux_amp)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.SF_cal_term_aux_amp, &(parameters.SF_cal_term_aux_amp)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.pwr_brd_led_blink_int, &(parameters.pwr_brd_led_blink_int)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.pwr_brd_led_blink_number, &(parameters.pwr_brd_led_blink_number)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.pwr_brd_led_power_1, &(parameters.pwr_brd_led_power_1)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.pwr_brd_led_power_2, &(parameters.pwr_brd_led_power_2)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.pwr_brd_led_power_3, &(parameters.pwr_brd_led_power_3)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.pwr_brd_led_power_4, &(parameters.pwr_brd_led_power_4)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	return ret;
}

} /* namespace pwr_brd */
