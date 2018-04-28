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
 * Power board parameters
 */

/**
 * system voltage sensor bias
 *
 * @unit V
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_SYSV_B, 0.00f);

/**
 * system voltage sensor scalefactor
 *
 * @unit
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_SYSV_SF, 1.00f);
/**
 * servo voltage sensor bias
 *
 * @unit V
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_SERVOV_B, 0.00f);

/**
 * servo voltage sensor scalefactor
 *
 * @unit
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_SERVO_SF, 1.00f);

/**
 * motor left current sensor bias
 *
 * @unit A
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_MOTL_B, 0.00f);
/**
 * Motor left current sensor scalefactor
 *
 * @unit
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_MOTL_SF, 1.00f);

/**
 * motor right current sensor bias
 *
 * @unit A
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_MOTR_B, 0.00f);

/**
 * motor right current sensor scalefactor
 *
 * @unit
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_MOTR_SF, 1.00f);

/**
 * digital channel current sensor bias
 *
 * @unit A
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_DIGI_B, 0.00f);

/**
 * digital channel current sensor scalefactor
 *
 * @unit
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_DIGI_SF, 1.00f);

/**
 * analog channel current sensor bias
 *
 * @unit A
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_ANALOG_B, 0.00f);

/**
 * analog channel current sensor scalefactor
 *
 * @unit
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_ANALOG_SF, 1.00f);

/**
 * extension channel current sensor bias
 *
 * @unit A
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_EXT_B, 0.00f);

/**
 * extension channel current sensor scalefactor
 *
 * @unit
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_EXT_SF, 1.00f);

/**
 * digital voltage sensor bias
 *
 * @unit V
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_DIGIV_B, 0.00f);

/**
 * digital voltage sensor scalefactor
 *
 * @unit
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_DIGIV_SF, 1.00f);

/**
 * Aux current sensor bias
 *
 * @unit A
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_AUX_B, 0.00f);

/**
 * Aux current sensor scalefactor
 *
 * @unit
 * @group POW Calibration
 */
PARAM_DEFINE_FLOAT(PWRBRD_AUX_SF, 1.00f);

/**
 * Power board LEDs blink interval in sec
 *
 * @unit Sec
 * @group POW LEDs
 */
PARAM_DEFINE_INT32(PWRBRD_LED_B_INT, 2);

/**
 * Power board LEDs number of blinks
 *
 * @unit
 * @group POW LEDs
 */
PARAM_DEFINE_INT32(PWRBRD_LED_B_NUM, 2);

/**
 * Power board LED 1 power in percentage
 *
 * @unit %
 * @group POW LEDs
 */
PARAM_DEFINE_INT32(PWRBRD_LED_POW_1, 20);

/**
 * Power board LED 2 power in percentage
 *
 * @unit %
 * @group POW LEDs
 */
PARAM_DEFINE_INT32(PWRBRD_LED_POW_2, 20);

/**
 * Power board LED 3 power in percentage
 *
 * @unit %
 * @group POW LEDs
 */
PARAM_DEFINE_INT32(PWRBRD_LED_POW_3, 20);

/**
 * Power board LED 4 power in percentage
 *
 * @unit %
 * @group POW LEDs
 */
PARAM_DEFINE_INT32(PWRBRD_LED_POW_4, 20);
