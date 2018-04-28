/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file Power board driver interface.
 */

#ifndef _DRV_PWR_BRD_H
#define _DRV_PWR_BRD_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define PWR_BRD_DEVICE_PATH	"/dev/pwr_brd"

#include <uORB/uORB.h>
#include <uORB/topics/sensor_pwr_brd.h>
#include <uORB/topics/parameter_update.h>

#define pwr_brd_report sensor_pwr_brd_s


struct pwr_brd_calibration_s {
	float _bias_cal_term_system_volt;
	float _SF_cal_term_system_volt;

	float _bias_cal_term_servo_volt;
	float _SF_cal_term_servo_volt;

	float _bias_cal_term_mot_l_amp;
	float _SF_cal_term_mot_l_amp;

	float _bias_cal_term_mot_r_amp;
	float _SF_cal_term_mot_r_amp;

	float _bias_cal_term_digital_amp;
	float _SF_cal_term_digital_amp;
	float _bias_cal_term_analog_amp;
	float _SF_cal_term_analog_amp;
	float _bias_cal_term_ext_amp;
	float _SF_cal_term_ext_amp;
	float _bias_cal_term_digital_volt;
	float _SF_cal_term_digital_volt;

	float _bias_cal_term_aux_amp;
	float _SF_cal_term_aux_amp;
};

struct pwr_brd_led_s {
	uint8_t _pwr_brd_led_blink_int;
	uint8_t _pwr_brd_led_blink_number;
	uint8_t _pwr_brd_led_power_1;
	uint8_t _pwr_brd_led_power_2;
	uint8_t _pwr_brd_led_power_3;
	uint8_t _pwr_brd_led_power_4;
};

/*
 * ioctl() definitions
 */

#define _PWR_BRDIOCBASE		(0x3200)
#define _PWR_BRDIOC(_n)		(_PX4_IOC(_PWR_BRDIOCBASE, _n))

/** set the pwr brd internal sample rate to at least (arg) Hz */
#define PWRBRDIOCSSAMPLERATE	_PWR_BRDIOC(0)

/** return the mag internal sample rate in Hz */
#define PWRBRDIOCGSAMPLERATE	_PWR_BRDIOC(1)

/** set the power board bias and SF terms */
#define PWRBRDIOCSSCALE			_PWR_BRDIOC(2)

/** set the power board LED terms */
#define PWRBRDIOLED				_PWR_BRDIOC(3)

#endif /* _DRV_PWR_BRD_H */
