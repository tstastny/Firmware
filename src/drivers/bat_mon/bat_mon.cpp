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

#include <drivers/bat_mon/bat_mon.h>

#define BATMON_MEAS_RATE 100
#define CONVERSION_INTERVAL	(1000000 / BATMON_MEAS_RATE)	/* microseconds */

Bat_mon::Bat_mon(int bus, int address, unsigned conversion_interval, const char *path, Instance instance) :
	I2C("Bat_mon", path, bus, address, 100000),
	_reports(nullptr),
	_buffer_overflows(perf_alloc(PC_COUNT, "bat_mon_buffer_overflows")),
	_measure_ticks(0),
	_collect_phase(false),
	_measurement_phase(false),
	_temperature(0),
	_voltage(0),
	_current(0),
	_stateofcharge(0),
	_batterystatus(0),
	_serialnumber(0),
	_safetystatus(0),
	_operationstatus(0),
	_cellvoltage{0},
	_bat_mon_pub_0(nullptr),
	_bat_mon_pub_1(nullptr),
	_bat_mon_pub_2(nullptr),
	_class_instance(-1),
	_bat_mon_orb_instance(-1),
	_report_bat_mon_pub(nullptr),
	_conversion_interval(conversion_interval),
	_sample_perf(perf_alloc(PC_ELAPSED, "bat_mon_read")),
	_comms_errors(perf_alloc(PC_COUNT, "bat_mon_comms_errors"))
{
	// enable debug() calls
	_debug_enabled = false;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

Bat_mon::~Bat_mon()
{
	/* make sure we are truly inactive */
	stop();

	if (_class_instance != -1) {
		unregister_class_devname(BAT_MON_DEVICE_PATH, _class_instance);
	}

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
}

int
Bat_mon::init()
{
	int ret = ERROR;
	struct sensor_bat_mon_s arp;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_bat_mon_s));

	if (_reports == nullptr) {
		goto out;
	}

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(BAT_MON_DEVICE_PATH);

	/* publication init */
	measure();
	_reports->get(&arp);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* measurement will have generated a report, publish */
		_bat_mon_pub_0 = orb_advertise(ORB_ID(sensor_bat_mon), &arp);

		if (_bat_mon_pub_0 == nullptr) {
			warnx("uORB bat_mon_pub_0 started?");
		}
	}

	if (_class_instance == CLASS_DEVICE_SECONDARY) {
		/* measurement will have generated a report, publish */
		_bat_mon_pub_1 = orb_advertise(ORB_ID(sensor_bat_mon), &arp);

		if (_bat_mon_pub_1 == nullptr) {
			warnx("uORB bat_mon_pub_1 started?");
		}
	}

	if (_class_instance == CLASS_DEVICE_TERTIARY) {
		/* measurement will have generated a report, publish */
		_bat_mon_pub_2 = orb_advertise(ORB_ID(sensor_bat_mon), &arp);

		if (_bat_mon_pub_2 == nullptr) {
			warnx("uORB bat_mon_pub_2 started?");
		}
	}

	ret = OK;

out:
	return ret;
}

int
Bat_mon::probe()
{
	/* on initial power up the device needs more than one retry
	   for detection. Once it is running then retries aren't
	   needed
	*/
	_retries = 4;
	int ret = deviceserialnumber();
	_retries = 0;
	return ret;
}

int
Bat_mon::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(_conversion_interval);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(_conversion_interval)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case SENSORIOCRESET:
		/* XXX implement this */
		return OK;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
Bat_mon::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_bat_mon_s);
	sensor_bat_mon_s *abuf = reinterpret_cast<sensor_bat_mon_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(abuf)) {
				ret += sizeof(*abuf);
				abuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(_conversion_interval);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(abuf)) {
			ret = sizeof(*abuf);
		}

	} while (0);

	return ret;
}

void
Bat_mon::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_measurement_phase = true;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&Bat_mon::cycle_trampoline, this, 1);
}

void
Bat_mon::stop()
{
	work_cancel(HPWORK, &_work);
}

void
Bat_mon::cycle_trampoline(void *arg)
{
	Bat_mon *dev = (Bat_mon *)arg;

	dev->cycle();
}

void
Bat_mon::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	warnx("poll interval:  %u ticks", _measure_ticks);
	_reports->print_info("report queue");
}

void
Bat_mon::new_report(const sensor_bat_mon_s &report)
{
	if (!_reports->force(&report)) {
		perf_count(_buffer_overflows);
	}
}
