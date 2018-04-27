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
 * @file bq78350.h
 *
 * Author: Amir Melzer  <amir.melzer@mavt.ethz.ch>
 *
 */

#include <drivers/bat_mon/bat_mon.h>

#ifndef BQ78350_H_
#define BQ78350_H_

/* SBS Address:					*/
#define	SMBTAR_ADDCONF	0x16

#define	SMBTAR_ADDR0	0x20
#define	SMBTAR_ADDR1	0x22
#define	SMBTAR_ADDR2	0x24
#define	SMBTAR_ADDR3	0x26
#define	SMBTAR_ADDR4	0x28
#define	SMBTAR_ADDR5	0x2A
#define	SMBTAR_ADDR6	0x2C
#define	SMBTAR_ADDR7	0x2E

/* SBS commands:				*/
#define	MANUFACTURERACCESS		0x00
#define	REMAININGCAPACITYALARM	0x01
#define	REMAININGTIMEALARM		0x02
#define	BATTERYMODE				0x03
#define	ATRATE					0x04
#define	ATRATETIMETOFULL		0x05
#define	ATRATETIMETOEMPTY		0x06
#define	ATRATEOK				0x07
#define	TEMPERATURE				0x08
#define	VOLTAGE					0x09
#define	CURRENT					0x0A
#define	AVERAGECURRENT			0x0B
#define	MAXERROR				0x0C
#define	RELATIVESTATEOFCHARGE	0x0D
#define	ABSOLUTESTATEOFCHARGE	0x0E
#define	REMAININGCAPACITY		0x0F
#define	FULLCHARGECAPACITY		0x10
#define	RUNTIMETOEMPTY			0x11
#define	AVERAGETIMETOEMPTY		0x12
#define	AVERAGETIMETOFULL		0x13
#define	CHARGINGCURRENT			0x14
#define	CHARGINGVOLTAGE			0x15
#define	BATTERYSTATUS			0x16
#define	CYCLECOUNT				0x17
#define	DESIGNCAPACITY			0x18
#define	DESIGNVOLTAGE			0x19
#define	SPECIFICATIONINFO		0x1A
#define	MANUFACTURERDATE		0x1B
#define	SERIALNUMBER			0x1C
#define	MANUFACTURERNAME		0x20
#define	DEVICENAME				0x21
#define	DEVICECHEMISTRY			0x22
#define	MANUFACTURERDATA		0x23
#define	HOSTFETCONTROL			0x2B
#define	GPIOSTATUS				0x2C
#define	GPIOCONTROL				0x2D
#define	VAUXVOLTAGE				0x2E
#define	AUTHENTICATE			0x2F
#define	RESERVED				0x30
#define	CELLVOLTAGE15			0x31
#define	CELLVOLTAGE14			0x32
#define	CELLVOLTAGE13			0x33
#define	CELLVOLTAGE12			0x34
#define	CELLVOLTAGE11			0x35
#define	CELLVOLTAGE10			0x36
#define	CELLVOLTAGE9			0x37
#define	CELLVOLTAGE8			0x38
#define	CELLVOLTAGE7			0x39
#define	CELLVOLTAGE6			0x3A
#define	CELLVOLTAGE5			0x3B
#define	CELLVOLTAGE4			0x3C
#define	CELLVOLTAGE3			0x3D
#define	CELLVOLTAGE2			0x3E
#define	CELLVOLTAGE1			0x3F
#define	EXTAVECELLVOLTAGE		0x4D
#define	PENDINGEDV				0x4E
#define	STATEOFHEALTH			0x4F
#define	SAFETYALERT				0x50
#define	SAFETYSTATUS			0x51
#define	PFALERT					0x52
#define	PFSTATUS				0x53
#define	OPERATIONSTATUS			0x54
#define	CHARGINGSTATUS			0x55
#define	GAUGINGSTATUS			0x56
#define	MANUFACTURINGSTATUS		0x57
#define	AFESTATUS				0x58
#define	AFECONFIG				0x59
#define	AFEVCX					0x5A
#define	AFEDATA					0x5B
#define	LIFETIMEDATABLOCK1		0x60
#define	LIFETIMEDATABLOCK2		0x61
#define	LIFETIMEDATABLOCK3		0x62
#define	LIFETIMEDATABLOCK4		0x63
#define	LIFETIMEDATABLOCK5		0x64
#define	LIFETIMEDATABLOCK6		0x65
#define	MANUFACTURERERINFO		0x70
#define	DASTATUS1				0x71
#define	DASTATUS2				0x72
#define	CUV						0x80
#define	COV						0x81


#endif /* BQ78350_H_ */
