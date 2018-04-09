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
 * @file bat_mon.cpp
 * Driver for the BAT_MON sensor connected via I2C.
 *
 * Author: Amir Melzer  <amir.melzer@mavt.ethz.ch>
 * 		   Lorenz Meier <lm@inf.ethz.ch>
 *
 */

#include <px4_config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>

#include <drivers/drv_bat_mon.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/ringbuffer.h>
#include <drivers/device/device.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/sensor_bat_mon.h>

/* Default I2C bus */
#define PX4_I2C_BUS_DEFAULT		PX4_I2C_BUS_EXPANSION

/* Oddly, ERROR is not defined for C++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

//class __EXPORT Bat_mon : public device::I2C
class Bat_mon : public device::I2C
{
public:
	enum class Instance : uint8_t {
		Primary = 0,
		Secondary,
		Tertiary
	};

	Bat_mon(int bus, int address, unsigned conversion_interval, const char *path, Instance instance);
	virtual ~Bat_mon();

	virtual int	init();
	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	virtual void	print_info();

private:
	ringbuffer::RingBuffer		*_reports;
	perf_counter_t		_buffer_overflows;

	/* this class has pointer data members and should not be copied */
	Bat_mon(const Bat_mon &);
	Bat_mon &operator=(const Bat_mon &);

protected:
	virtual int	probe();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	virtual void	cycle() = 0;
	virtual int	measure() = 0;
	virtual int	collect() = 0;
	virtual int	deviceserialnumber() = 0;

	/**
	 * Update the subsystem status
	 */
	//void update_status();

	work_s			_work;
	int				_measure_ticks;
	bool			_collect_phase;
	bool			_measurement_phase;

	uint16_t	  	_temperature;
	uint16_t	  	_voltage;
	uint16_t	  	_current;
	uint8_t		  	_stateofcharge;
	uint16_t	  	_batterystatus;
	uint16_t	  	_serialnumber;
	uint32_t	  	_safetystatus;
	uint32_t	  	_operationstatus;
	uint16_t	  	_cellvoltage[15];

	orb_advert_t	_bat_mon_pub_0;
	orb_advert_t	_bat_mon_pub_1;
	orb_advert_t	_bat_mon_pub_2;

	int				_class_instance;
	int				_bat_mon_orb_instance;
	orb_advert_t	_report_bat_mon_pub;

	unsigned		_conversion_interval;

	perf_counter_t	_sample_perf;
	perf_counter_t	_comms_errors;


	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return		True if the device is present.
	*/
	int	probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void	start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void	stop();

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void	cycle_trampoline(void *arg);

	/**
	* add a new report to the reports queue
	*
	* @param report		bat_mon_s report
	*/
	void	new_report(const sensor_bat_mon_s &report);
};
