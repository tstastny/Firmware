/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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
 * @file bq78350.cpp
 * Driver for the BQ78350 sensor connected via I2C.
 *
 * Author: Amir Melzer  <amir.melzer@mavt.ethz.ch>
 * 		   Lorenz Meier <lm@inf.ethz.ch>
 *
 */

#include "bq78350.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif

/* Measurement rate is 2Hz */
#define MEAS_RATE 2
#define CONVERSION_INTERVAL	(1000000 / MEAS_RATE)	/* microseconds */

#define SERIAL_NUMBER_BQ78350 0

#define MAXIMALNUMBEROFCELLS 15
#define DEFAULTNUMBEROFCELLS 6

uint8_t number_of_cells = DEFAULTNUMBEROFCELLS;

class Bq78350 : public Bat_mon
{
public:
	Bq78350(int bus, int address = (SMBTAR_ADDR0 >> 1), const char *path = BAT_MON_1_DEVICE_PATH,
		Instance instance = Instance::Primary) :
		Bat_mon(bus, address, CONVERSION_INTERVAL, path, instance)
	{
	}

private:

	void cycle();
	int	measure();
	int	collect();
	int	deviceserialnumber();

	/**
	 * Send a SBS command for read back a single byte
	 */
	int	getOneBytesSBSReading(uint8_t sbscmd, uint8_t *sbsreading);

	/**
	 * Send a SBS command for read back a two bytes
	 */
	int getTwoBytesSBSReading(uint8_t sbscmd, uint16_t *sbsreading);

	/**
	 * Send a SBS command for reading back four bytes
	 */
	int getBlockSBSReading(uint8_t sbscmd, uint32_t *sbsreading);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int bq78350_main(int argc, char *argv[]);

/* collect Battery monitor measurements: */
int
Bq78350::measure()
{
	/* read the most recent measurement */
	perf_begin(_sample_perf);

	struct sensor_bat_mon_s report;


	if (OK != getTwoBytesSBSReading(TEMPERATURE, &_temperature)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getTwoBytesSBSReading(VOLTAGE, &_voltage)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getTwoBytesSBSReading(CURRENT, &_current)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getOneBytesSBSReading(RELATIVESTATEOFCHARGE, &_stateofcharge)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getTwoBytesSBSReading(BATTERYSTATUS, &_batterystatus)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getTwoBytesSBSReading(SERIALNUMBER, &_serialnumber)) {
		perf_count(_comms_errors);
		return -EIO;
	}


	if (OK != getBlockSBSReading(SAFETYSTATUS, &_safetystatus)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != getBlockSBSReading(OPERATIONSTATUS, &_operationstatus)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	for (unsigned i = 0; i < MAXIMALNUMBEROFCELLS; i++) {
		if (i < number_of_cells) {
			if (OK != getTwoBytesSBSReading((CELLVOLTAGE1 - i), &_cellvoltage[i])) {
				perf_count(_comms_errors);
				return -EIO;
			}

		} else {
			_cellvoltage[i] = 0;
		}
	}

	/* generate a new report */
	report.timestamp 		= hrt_absolute_time();		/* report timestamp		*/
	report.temperature	  	= _temperature;				/* report in [0.1 K]  	*/
	report.voltage 		  	= _voltage;					/* report in [mV] 	   	*/
	report.current 		  	= (int16_t)_current;		/* report in [mA]  		*/
	report.stateofcharge  	= _stateofcharge;			/* report in uint [%]  	*/
	report.batterystatus  	= _batterystatus;			/* report in Hex word  	*/
	report.serialnumber   	= _serialnumber;			/* report in uint word 	*/
	report.safetystatus 	= _safetystatus;			/* report in Hex word  	*/
	report.operationstatus  = _operationstatus;			/* report in Hex word  	*/

	for (unsigned i = 0; i < MAXIMALNUMBEROFCELLS; i++) {
		report.cellvoltage[i] = _cellvoltage[i];		/* report in [mv]		*/
	}

	//warnx("measurements Bq78350 board sensor: %6d %6d %6d %6d %6d %6d %6d %6d %6d %6d %6d %6d %6d %6d",_temperature, _voltage, _current, _batterystatus, _serialnumber, _cellvoltage1, _cellvoltage2, _cellvoltage3, _cellvoltage4, _cellvoltage5, _cellvoltage6, _cellvoltage7, _cellvoltage8, _cellvoltage9);

	orb_publish_auto(ORB_ID(sensor_bat_mon), &_report_bat_mon_pub, &report, &_bat_mon_orb_instance, ORB_PRIO_DEFAULT);

	new_report(report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	perf_end(_sample_perf);

	return OK;
}

int
Bq78350::collect()
{
	return OK;
}

void
Bq78350::cycle()
{
	/* collection phase? */
	if (_measurement_phase) {
		/* perform voltage measurement */
		if (OK != measure()) {
			start();
			return;
		}

		/* next phase is measurement */
		_measurement_phase = true;

		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&Bq78350::cycle_trampoline,
			   this,
			   _measure_ticks);
	}
}

/* Single Bytes of SBS command Reading */
int
Bq78350::getOneBytesSBSReading(uint8_t sbscmd, uint8_t *sbsreading)
{
	uint8_t data;

	/* fetch the raw value */
	if (OK != transfer(&sbscmd, 1, &data, 1)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	*sbsreading = data;

	return OK;
}

/* Two Bytes of SBS command reading */
int
Bq78350::getTwoBytesSBSReading(uint8_t sbscmd, uint16_t *sbsreading)
{
	uint8_t data[2];
	union {
		uint8_t	b[2];
		uint16_t w;
	} cvt;

	/* fetch the raw value */
	if (OK != transfer(&sbscmd, 1, &data[0], 2)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* assemble 16 bit value */
	cvt.b[0] = data[0];
	cvt.b[1] = data[1];

	*sbsreading = cvt.w;

	return OK;
}

/* Four Bytes of SBS command reading */
int
Bq78350::getBlockSBSReading(uint8_t sbscmd, uint32_t *sbsreading)
{
	uint8_t size[1];


	/* fetch the block size */
	if (OK != transfer(&sbscmd, 1, &size[0], 1)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	uint8_t data[size[0] + 1];
	union {
		uint8_t	b[4];
		uint32_t w;
	} cvt;

	/* fetch the raw value */
	if (OK != transfer(&sbscmd, 1, &data[0], size[0] + 1)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	memcpy(&(cvt.b[0]), &data[1], size[0]);

	*sbsreading = cvt.w;

	return OK;
}

int
Bq78350::deviceserialnumber()
{
	if (OK != getTwoBytesSBSReading(SERIALNUMBER, &_serialnumber)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (_serialnumber == SERIAL_NUMBER_BQ78350) {
		warnx("Bq78350 board sensor serial number: %d", _serialnumber);

	} else {
		return OK;        ///return -EIO;   /// temp marked for debug (modify the device number on the bat_mon)!!!!!!!!!!!!!!!!!!!!!!
	}

	return OK;
}

/**
 * Local functions in support of the shell command.
 */
namespace bq78350
{

Bq78350	*g_dev;

void	start(int i2c_bus, Bq78350::Instance instance);
void	test(Bq78350::Instance instance);
void	reset(Bq78350::Instance instance);
void	info();
void	usage();
/**
 * Start the driver.
 */
void
start(int i2c_bus, Bq78350::Instance instance)
{
	int fd;

	if (instance == Bq78350::Instance::Primary) {

		if (g_dev != nullptr) {
			PX4_INFO("starting a new battery monitoring instance:");
		}

		/* create the driver */
		g_dev = new Bq78350(i2c_bus, (SMBTAR_ADDR0 >> 1), BAT_MON_1_DEVICE_PATH, instance);

		if (g_dev == nullptr) {
			goto fail;
		}

		if (OK != g_dev->Bat_mon::init()) {
			goto fail;
		}

		/* set the poll rate to default, starts automatic data collection */
		fd = open(BAT_MON_1_DEVICE_PATH, O_RDONLY);

		if (fd < 0) {
			goto fail;
		}

		if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			goto fail;
		}

		exit(0);
	}

	if (instance == Bq78350::Instance::Secondary) {

		if (g_dev != nullptr) {
			PX4_INFO("starting a new battery monitoring instance:");
		}

		/* create the driver */
		g_dev = new Bq78350(i2c_bus, (SMBTAR_ADDR1 >> 1), BAT_MON_2_DEVICE_PATH, instance);

		if (g_dev == nullptr) {
			goto fail;
		}

		if (OK != g_dev->Bat_mon::init()) {
			goto fail;
		}

		/* set the poll rate to default, starts automatic data collection */
		fd = open(BAT_MON_2_DEVICE_PATH, O_RDONLY);

		if (fd < 0) {
			goto fail;
		}

		if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			goto fail;
		}

		exit(0);
	}

	if (instance == Bq78350::Instance::Tertiary) {

		if (g_dev != nullptr) {
			PX4_INFO("starting a new battery monitoring instance:");
		}

		/* create the driver */
		g_dev = new Bq78350(i2c_bus, (SMBTAR_ADDR2 >> 1), BAT_MON_3_DEVICE_PATH, instance);

		if (g_dev == nullptr) {
			goto fail;
		}

		if (OK != g_dev->Bat_mon::init()) {
			goto fail;
		}

		/* set the poll rate to default, starts automatic data collection */
		fd = open(BAT_MON_3_DEVICE_PATH, O_RDONLY);

		if (fd < 0) {
			goto fail;
		}

		if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			goto fail;
		}

		exit(0);
	}

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver bq78350 start failed");
}

void
test(Bq78350::Instance instance)
{
	struct sensor_bat_mon_s report;

	ssize_t sz;
	int ret;
	int fd;

	if (instance == Bq78350::Instance::Primary) {
		fd = open(BAT_MON_1_DEVICE_PATH, O_RDONLY);

	} else if (instance == Bq78350::Instance::Secondary) {
		fd = open(BAT_MON_2_DEVICE_PATH, O_RDONLY);

	} else if (instance == Bq78350::Instance::Tertiary) {
		fd = open(BAT_MON_3_DEVICE_PATH, O_RDONLY);

	} else {
		fd = 0;
	}

	if (fd < 0) {
		if (instance == Bq78350::Instance::Primary) {
			err(1, "%s open failed (try 'BQ78350 start' if the driver is not running)", BAT_MON_1_DEVICE_PATH);
		}

		if (instance == Bq78350::Instance::Secondary) {
			err(1, "%s open failed (try 'BQ78350 start' if the driver is not running)", BAT_MON_2_DEVICE_PATH);
		}

		if (instance == Bq78350::Instance::Tertiary) {
			err(1, "%s open failed (try 'BQ78350 start' if the driver is not running)", BAT_MON_3_DEVICE_PATH);
		}
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("time:            %lld", report.timestamp);
	warnx("serialnumber:     %6d", report.serialnumber);
	warnx("temperature:      %6d", report.temperature - 2731);
	warnx("voltage:          %6d", report.voltage);
	warnx("current:   	     %6d", report.current);
	warnx("stateofcharge:    %6d", report.stateofcharge);
	warnx("batterystatus:    %4x", report.batterystatus);
	warnx("safetystatus: 	 %8x", report.safetystatus);
	warnx("operationstatus:  %4x", report.operationstatus);

	for (unsigned i = 0; i < MAXIMALNUMBEROFCELLS; i++) {
		warnx("cellvoltage%d:     %6d", (i + 1), report.cellvoltage[i]);
	}

	/* set the queue depth to 10 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10)) {
		errx(1, "failed to set queue depth");
	}

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("time:            %lld", report.timestamp);
		warnx("serialnumber:     %6d", report.serialnumber);
		warnx("temperature:      %6d", report.temperature - 2731);
		warnx("voltage:          %6d", report.voltage);
		warnx("current:   	     %6d", report.current);
		warnx("stateofcharge:    %6d", report.stateofcharge);
		warnx("batterystatus:    %6x", report.batterystatus);
		warnx("safetystatus:     %6x", report.safetystatus);
		warnx("operationstatus:  %6x", report.operationstatus);

		for (unsigned j = 0; j < MAXIMALNUMBEROFCELLS; j++) {
			warnx("cellvoltage%d:     %6d", (j + 1), report.cellvoltage[j]);
		}
	}

	bq78350::reset(instance);
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset(Bq78350::Instance instance)
{
	//int fd = open(BAT_MON_1_DEVICE_PATH, O_RDONLY);
	int fd;

	if (instance == Bq78350::Instance::Primary) {
		fd = open(BAT_MON_1_DEVICE_PATH, O_RDONLY);

	} else if (instance == Bq78350::Instance::Secondary) {
		fd = open(BAT_MON_2_DEVICE_PATH, O_RDONLY);

	} else if (instance == Bq78350::Instance::Tertiary) {
		fd = open(BAT_MON_3_DEVICE_PATH, O_RDONLY);

	} else {
		fd = 0;
	}

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset',\n");
	warnx("options:");
	warnx("    -C    (number of cells)");
	warnx("    -U    (unit number)");
}

} // namespace

int
bq78350_main(int argc, char *argv[])
{
	int i2c_bus = PX4_I2C_BUS_DEFAULT;
	int unit_number = 0;
	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "CU:")) != EOF) {
		switch (ch) {
		case 'C':
			number_of_cells = atoi(optarg);
			break;

		case 'U':
			unit_number = atoi(optarg);
			break;

		default:
			bq78350::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		bq78350::start(i2c_bus, Bq78350::Instance(unit_number));
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		bq78350::test(Bq78350::Instance(unit_number));
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		bq78350::reset(Bq78350::Instance(unit_number));
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(argv[1], "status")) {
		bq78350::info();
	}

	bq78350::usage();
	exit(0);
}
