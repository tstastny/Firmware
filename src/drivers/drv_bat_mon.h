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
 * @file current sensor driver interface.
 */

#ifndef _DRV_BAT_MON_H
#define _DRV_BAT_MON_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define BAT_MON_DEVICE_PATH	"/dev/bat_mon"
#define BAT_MON_1_DEVICE_PATH "/dev/bat_mon_1"
#define BAT_MON_2_DEVICE_PATH "/dev/bat_mon_2"
#define BAT_MON_3_DEVICE_PATH "/dev/bat_mon_3"

/**
 * Battery monitor sensors report structure.
 */
struct bat_mon_report {
	uint64_t  timestamp;
	uint16_t  temperature;
	uint16_t  voltage;
	int16_t   current;
	uint8_t   stateofcharge;
	uint16_t  batterystatus;
	uint16_t  serialnumber;
	uint32_t  safetystatus;
	uint32_t  operationstatus;
	uint16_t  cellvoltage[15];
};


/*
 * ObjDev tag for raw voltage and current data.
 */
//ORB_DECLARE(sensor_bat_mon_0);
//ORB_DECLARE(sensor_bat_mon_1);
//ORB_DECLARE(sensor_bat_mon_2);


/*
 * ioctl() definitions
*/

#define _BATMONIOCBASE		(0x2e00)
#define _BATMONIOC(_n)		(_IOC(_BATMONIOCBASE, _n))


#endif /* _DRV_BAT_MON_H */
