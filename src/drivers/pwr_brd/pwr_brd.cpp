/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file pwr_brd.cpp
 * Driver for the PWRBRD connected via I2C.
 *
 * Author: Amir Melzer  <amir.melzer@mavt.ethz.ch>
 */

#include "pwr_brd.h"

/** driver 'main' command */
extern "C" { __EXPORT int pwr_brd_main(int argc, char *argv[]); }


namespace pwr_brd
{

PWR_BRD *g_dev_ext;
PWR_BRD *g_dev_int;

void start(bool);
void test(bool);
void reset(bool);
void info(bool);
void usage();


/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void start(bool external_bus)
{
	int fd;
	PWR_BRD **g_dev_ptr = (external_bus ? &g_dev_ext : &g_dev_int);
	const char *path = (external_bus ? PWR_BRD_DEVICE_PATH : PWR_BRD_DEVICE_PATH);   // TODO:remove external bus

	if (*g_dev_ptr != nullptr)
		/* if already started, the still command succeeded */
	{
		PX4_ERR("already started");
		exit(0);
	}

	/* create the driver */
	if (external_bus) {
#if defined(PX4_I2C_BUS_EXPANSION)
		*g_dev_ptr = new PWR_BRD(PX4_I2C_BUS_EXPANSION, path);
#else
		PX4_ERR("External I2C not available");
		exit(0);
#endif

	} else {
		PX4_ERR("Internal I2C not available");
		exit(0);
	}

	if (*g_dev_ptr == nullptr) {
		goto fail;
	}


	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(path, O_RDONLY);


	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	close(fd);

	exit(0);
fail:

	if (*g_dev_ptr != nullptr) {
		delete (*g_dev_ptr);
		*g_dev_ptr = nullptr;
	}

	PX4_ERR("driver start failed");
	exit(1);

}


void test(bool external_bus)
{
	int fd = -1;
	const char *path = (external_bus ? PWR_BRD_DEVICE_PATH : PWR_BRD_DEVICE_PATH);
	struct pwr_brd_report p_report;
	ssize_t sz;


	/* get the driver */
	fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'pwr_brd start' if the driver is not running)",
			path);
		exit(1);
	}

	/* reset to Max polling rate*/
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX) < 0) {
		PX4_ERR("reset to Max polling rate");
		exit(1);
	}

	/* do a simple demand read */
	sz = read(fd, &p_report, sizeof(p_report));

	if (sz != sizeof(p_report)) {
		PX4_ERR("immediate pwr_brd read failed");
		exit(1);
	}

	PX4_INFO("single read");
	PX4_INFO("time:               %lld", p_report.timestamp);

	PX4_INFO("Power Board status: 0x%x", p_report.pwr_brd_status);
	PX4_INFO("LED status:         0x%x", p_report.pwr_brd_led_status);
	PX4_INFO("LED blink pattern:  0x%x", p_report.pwr_brd_blink_reg);

	PX4_INFO("LED1 power:   	   0x%x", (short)p_report.pwr_brd_led_1_pwr);
	PX4_INFO("LED2 power:   	   0x%x", (short)p_report.pwr_brd_led_2_pwr);
	PX4_INFO("LED3 power:   	   0x%x", (short)p_report.pwr_brd_led_3_pwr);
	PX4_INFO("LED4 power:   	   0x%x", (short)p_report.pwr_brd_led_4_pwr);

	PX4_INFO("System voltage:    %10.4f", (double)p_report.pwr_brd_system_volt);
	PX4_INFO("Motor L current:   %10.4f", (double)p_report.pwr_brd_mot_l_amp);
	PX4_INFO("Motor R current:   %10.4f", (double)p_report.pwr_brd_mot_r_amp);

	PX4_INFO("Digital voltage:   %10.4f", (double)p_report.pwr_brd_digital_volt);
	PX4_INFO("Digital current:   %10.4f", (double)p_report.pwr_brd_digital_amp);
	PX4_INFO("Servo voltage:     %10.4f", (double)p_report.pwr_brd_servo_volt);
	PX4_INFO("Servo current:     %10.4f", (double)p_report.pwr_brd_analog_amp);
	PX4_INFO("AUX current:       %10.4f", (double)p_report.pwr_brd_aux_amp);
	PX4_INFO("Extension current: %10.4f", (double)p_report.pwr_brd_ext_amp);

	PX4_INFO("PASS");
	exit(0);

}


void
reset(bool external_bus)
{
	const char *path = external_bus ? PWR_BRD_DEVICE_PATH : PWR_BRD_DEVICE_PATH;
	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		exit(1);
	}

	exit(0);

}

void
info(bool external_bus)
{
	PWR_BRD **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		PX4_ERR("driver not running");
		exit(1);
	}

	printf("state @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_info();

	exit(0);

}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'test', 'stop', 'reset'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (external bus)");

}

} // namespace pwr_brd


PWR_BRD :: PWR_BRD(int bus, const char *path) :
	I2C("PWR_BRD", path, bus, PWR_BRD_SLAVE_ADDRESS, PWR_BRD_BUS_SPEED),
	_running(false),
	_call_interval(0),
	_reports(nullptr),
	_collect_phase(false),
	_scale{},
	_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "pwr_brd_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "pwr_brd_bad_transfers")),
	_good_transfers(perf_alloc(PC_COUNT, "pwr_brd_good_transfers")),
	_measure_perf(perf_alloc(PC_ELAPSED, "pwr_brd_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "pwr_brd_comms_errors")),
	_duplicates(perf_alloc(PC_COUNT, "pwr_brd_duplicates")),
	_got_duplicate(false),
	_pwr_brd_led{},
	_bias_cal_term_system_volt(0.0f),
	_SF_cal_term_system_volt(1.0f),
	_bias_cal_term_servo_volt(0.0f),
	_SF_cal_term_servo_volt(1.0f),
	_bias_cal_term_mot_l_amp(0.0f),
	_SF_cal_term_mot_l_amp(1.0f),
	_bias_cal_term_mot_r_amp(0.0f),
	_SF_cal_term_mot_r_amp(1.0f),
	_bias_cal_term_digital_amp(0.0f),
	_SF_cal_term_digital_amp(1.0f),
	_bias_cal_term_analog_amp(0.0f),
	_SF_cal_term_analog_amp(1.0f),
	_bias_cal_term_ext_amp(0.0f),
	_SF_cal_term_ext_amp(1.0f),
	_bias_cal_term_digital_volt(0.0f),
	_SF_cal_term_digital_volt(1.0f),
	_bias_cal_term_aux_amp(0.0f),
	_SF_cal_term_aux_amp(1.0f),
	_pwr_brd_led_blink_int(2),
	_pwr_brd_led_blink_number(2),
	_pwr_brd_led_power_1(0),
	_pwr_brd_led_power_2(0),
	_pwr_brd_led_power_3(0),
	_pwr_brd_led_power_4(0),
	_params_sub(-1)
{
	_device_id.devid_s.devtype = DRV_PWR_DEVTYPE_PWRBRD;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));

	// default scaling
	_scale._bias_cal_term_system_volt = 0;
	_scale._SF_cal_term_system_volt = 1.0f;

	_scale._bias_cal_term_servo_volt = 0;
	_scale._SF_cal_term_servo_volt = 1.0f;

	_scale._bias_cal_term_mot_l_amp = 0;
	_scale._SF_cal_term_mot_l_amp = 1.0f;

	_scale._bias_cal_term_mot_r_amp = 0;
	_scale._SF_cal_term_mot_r_amp = 1.0f;

	_scale._bias_cal_term_digital_amp = 0;
	_scale._SF_cal_term_digital_amp = 1.0f;
	_scale._bias_cal_term_analog_amp = 0;
	_scale._SF_cal_term_analog_amp = 1.0f;
	_scale._bias_cal_term_ext_amp = 0;
	_scale._SF_cal_term_ext_amp = 1.0f;
	_scale._bias_cal_term_digital_volt = 0;
	_scale._SF_cal_term_digital_volt = 1.0f;

	_scale._bias_cal_term_aux_amp = 0;
	_scale._SF_cal_term_aux_amp = 1.0f;

	_pwr_brd_led._pwr_brd_led_blink_int = 2;

	_pwr_brd_led._pwr_brd_led_blink_number = 2;
	_pwr_brd_led._pwr_brd_led_power_1 = 0;
	_pwr_brd_led._pwr_brd_led_power_2 = 0;
	_pwr_brd_led._pwr_brd_led_power_3 = 0;
	_pwr_brd_led._pwr_brd_led_power_4 = 0;

	initialize_parameter_handles(_parameter_handles);
}

PWR_BRD :: ~PWR_BRD()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_topic != nullptr) {
		orb_unadvertise(_topic);
	}

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
	perf_free(_good_transfers);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
	perf_free(_duplicates);

}

int PWR_BRD::init()
{
	int ret = OK;

	/* do I2C init (and probe) first */
	ret = I2C::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("I2C setup failed");
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(pwr_brd_report));

	if (_reports == nullptr) {
		goto out;
	}

	//ToDo: add proper ID reg
	/* check  id*/
	//if (read_reg(PWR_BRD_CHIP_ID_REG) != PWR_BRD_CHIP_ID) {
	//	PX4_WARN("id of pwr_brd is not: 0x%02x", PWR_BRD_CHIP_ID);
	//	return -EIO;
	//}

	if (reset() != OK) {
		goto out;
	}

	if (measure()) {
		return -EIO;
	}

	up_udelay(10000);

	if (collect()) {
		return -EIO;
	}

	/* advertise sensor topic, measure manually to initialize valid report */
	struct pwr_brd_report prb;
	_reports->get(&prb);

	/* measurement will have generated a report, publish */
	_topic = orb_advertise(ORB_ID(sensor_pwr_brd), &prb);

	if (_topic == nullptr) {
		PX4_WARN("ADVERT FAIL");
	}

out:
	return ret;
}

int
PWR_BRD::probe()
{
	return OK;
	//ToDo: write a test function here!!!!!!!
	////if (OK == get_pwr_brd_data()) {
	////	return OK;
	////}

	return -EIO;
}

void
PWR_BRD::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_running = true;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&PWR_BRD::cycle_trampoline, this, 1);
}

void
PWR_BRD::stop()
{
	_running = false;
	work_cancel(HPWORK, &_work);
}

ssize_t
PWR_BRD::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(pwr_brd_report);
	struct pwr_brd_report *pwr_brd_buf = reinterpret_cast<struct pwr_brd_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_call_interval > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(pwr_brd_buf)) {
				ret += sizeof(struct pwr_brd_report);
				pwr_brd_buf++;
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
		usleep(PWR_BRD_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}


		if (_reports->get(pwr_brd_buf)) {
			ret = sizeof(struct pwr_brd_report);
		}
	} while (0);

	/* return the number of bytes transferred */
	return ret;

}


void
PWR_BRD::cycle_trampoline(void *arg)
{
	PWR_BRD *dev = reinterpret_cast<PWR_BRD *>(arg);

	/* make measurement */
	dev->cycle();
}

void
PWR_BRD::cycle()
{
	bool force_update = false;
	bool param_updated = false;

	if (!_running) {
		if (_params_sub >= 0) {
			orb_unsubscribe(_params_sub);
		}

	} else {
		if (_params_sub < 0) {
			_params_sub = orb_subscribe(ORB_ID(parameter_update));
			force_update = true;
		}
	}

	/* Check if any parameter changed */
	orb_check(_params_sub, &param_updated);

	if (param_updated || force_update) {

		struct parameter_update_s update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &update);
		parameters_update();
	}

	if (_collect_phase) {
		collect();
		unsigned wait_gap = _call_interval - USEC2TICK(PWR_BRD_CONVERSION_INTERVAL);

		if ((wait_gap != 0) && (_running)) {
			work_queue(HPWORK, &_work, (worker_t)&PWR_BRD::cycle_trampoline, this,
				   wait_gap); //need to wait some time before new measurement
			return;
		}

	}

	measure();

	if ((_running)) {
		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&PWR_BRD::cycle_trampoline,
			   this,
			   USEC2TICK(PWR_BRD_CONVERSION_INTERVAL));
	}

}

int
PWR_BRD::parameters_update()
{
	int ret = update_parameters(_parameter_handles, _parameters);

	_scale._bias_cal_term_system_volt = _parameters.bias_cal_term_system_volt;
	_scale._SF_cal_term_system_volt = _parameters.SF_cal_term_system_volt;
	_scale._bias_cal_term_servo_volt = _parameters.bias_cal_term_servo_volt;
	_scale._SF_cal_term_servo_volt = _parameters.SF_cal_term_servo_volt;
	_scale._bias_cal_term_mot_l_amp = _parameters.bias_cal_term_mot_l_amp;
	_scale._SF_cal_term_mot_l_amp = _parameters.SF_cal_term_mot_l_amp;
	_scale._bias_cal_term_mot_r_amp = _parameters.bias_cal_term_mot_r_amp;
	_scale._SF_cal_term_mot_r_amp = _parameters.SF_cal_term_mot_r_amp;
	_scale._bias_cal_term_digital_amp = _parameters.bias_cal_term_digital_amp;
	_scale._SF_cal_term_digital_amp = _parameters.SF_cal_term_digital_amp;
	_scale._bias_cal_term_analog_amp = _parameters.bias_cal_term_analog_amp;
	_scale._SF_cal_term_analog_amp = _parameters.SF_cal_term_analog_amp;
	_scale._bias_cal_term_ext_amp = _parameters.bias_cal_term_ext_amp;
	_scale._SF_cal_term_ext_amp = _parameters.SF_cal_term_ext_amp;
	_scale._bias_cal_term_digital_volt = _parameters.bias_cal_term_digital_volt;
	_scale._SF_cal_term_digital_volt = _parameters.SF_cal_term_digital_volt;
	_scale._bias_cal_term_aux_amp = _parameters.bias_cal_term_aux_amp;
	_scale._SF_cal_term_aux_amp = _parameters.SF_cal_term_aux_amp;
	_pwr_brd_led._pwr_brd_led_blink_int = _parameters.pwr_brd_led_blink_int;
	_pwr_brd_led._pwr_brd_led_blink_number = _parameters.pwr_brd_led_blink_number;
	_pwr_brd_led._pwr_brd_led_power_1 = _parameters.pwr_brd_led_power_1;
	_pwr_brd_led._pwr_brd_led_power_2 = _parameters.pwr_brd_led_power_2;
	_pwr_brd_led._pwr_brd_led_power_3 = _parameters.pwr_brd_led_power_3;
	_pwr_brd_led._pwr_brd_led_power_4 = _parameters.pwr_brd_led_power_4;

	return ret;
}

int
PWR_BRD::measure()
{
	_collect_phase = true;

	perf_begin(_measure_perf);

	/* start measure */
	int ret = OK;								////ToDo: write a test function here!

	if (ret != OK) {
		perf_count(_comms_errors);
		perf_cancel(_measure_perf);
		return -EIO;
	}

	perf_end(_measure_perf);

	return OK;
}

int
PWR_BRD::collect()
{
	_collect_phase = false;
	bool pwr_brd_notify = true;
	pwr_brd_report  prb;

	uint8_t pwr_brd_status = 0;
	uint8_t pwr_brd_led_status = 0;
	uint8_t	pwr_brd_blink_reg = 0;
	uint8_t	pwr_brd_led_1_pwr = 0;
	uint8_t	pwr_brd_led_2_pwr = 0;
	uint8_t	pwr_brd_led_3_pwr = 0;
	uint8_t	pwr_brd_led_4_pwr = 0;

	float pwr_brd_system_v = 0.0f;
	float pwr_brd_servo_v  = 0.0f;
	float pwr_brd_digital_v  = 0.0f;

	float pwr_brd_mot_l = 0.0f;
	float pwr_brd_mot_r = 0.0f;

	float pwr_brd_analog   = 0.0f;
	float pwr_brd_ext      = 0.0f;
	float pwr_brd_digital = 0.0f;
	float pwr_brd_aux      = 0.0f;

	/* read the most recent measurement */
	perf_begin(_sample_perf);

	if (OK != get_regs(LED_STATS_REG, &pwr_brd_led_status)) {
		return -EIO;
	}

	if (OK != get_regs(LED_BLINK_REG, &pwr_brd_blink_reg)) {
		return -EIO;
	}

	if (OK != get_regs(LED_POW_1_REG, &pwr_brd_led_1_pwr)) {
		return -EIO;
	}

	if (OK != get_regs(LED_POW_2_REG, &pwr_brd_led_2_pwr)) {
		return -EIO;
	}

	if (OK != get_regs(LED_POW_3_REG, &pwr_brd_led_3_pwr)) {
		return -EIO;
	}

	if (OK != get_regs(LED_POW_4_REG, &pwr_brd_led_4_pwr)) {
		return -EIO;
	}

	if (OK != get_regs(BRD_STATS_REG, &pwr_brd_status)) {
		return -EIO;
	}

	if (OK != get_voltage(SYSTEM_VOLT_REG, &pwr_brd_system_v)) {
		return -EIO;
	}

	if (OK != get_voltage(SERVO_VOLT_REG, &pwr_brd_servo_v)) {
		return -EIO;
	}

	if (OK != get_voltage(DIGITAL_VOLTAGE_REG, &pwr_brd_digital_v)) {
		return -EIO;
	}

	if (OK != get_channel_current(DIGITAL_CURRENT_REG, &pwr_brd_digital)) {
		return -EIO;
	}

	if (OK != get_channel_current(ANALOG_CURRENT_REG, &pwr_brd_analog)) {
		return -EIO;
	}

	if (OK != get_channel_current(AUX_CURRENT_REG, &pwr_brd_aux)) {
		return -EIO;
	}

	if (OK != get_motor_current(MOT_L_CURRENT_REG, &pwr_brd_mot_l)) {
		return -EIO;
	}

	if (ENA_MOT_R) {
		if (OK != get_motor_current(MOT_R_CURRENT_REG, &pwr_brd_mot_r)) {
			return -EIO;
		}

	} else {
		if (OK != get_motor_current(EXT_CURRENT_REG, &pwr_brd_ext)) {
			return -EIO;
		}
	}

	/* apply calibration values */
	pwr_brd_system_v = (MV_TO_V(pwr_brd_system_v) * SYSTEM_VOLTAGE_SCALE - _scale._bias_cal_term_system_volt) *
			   _scale._SF_cal_term_system_volt;
	pwr_brd_servo_v  = (MV_TO_V(pwr_brd_servo_v) * SERVO_VOLTAGE_SCALE - _scale._bias_cal_term_servo_volt) *
			   _scale._SF_cal_term_servo_volt;
	pwr_brd_digital_v = (MV_TO_V(pwr_brd_digital_v) * DIGITAL_VOLTAGE_SCALE - _scale._bias_cal_term_digital_volt) *
			    _scale._SF_cal_term_digital_volt;

	pwr_brd_mot_l = (pwr_brd_mot_l - _scale._bias_cal_term_mot_l_amp) * _scale._SF_cal_term_mot_l_amp;
	pwr_brd_mot_r = (pwr_brd_mot_r - _scale._bias_cal_term_mot_r_amp) * _scale._SF_cal_term_mot_r_amp;

	pwr_brd_digital = (pwr_brd_digital - _scale._bias_cal_term_digital_amp) * _scale._SF_cal_term_digital_amp;
	pwr_brd_analog = (pwr_brd_analog - _scale._bias_cal_term_analog_amp) * _scale._SF_cal_term_analog_amp;
	pwr_brd_ext = (pwr_brd_ext - _scale._bias_cal_term_ext_amp) * _scale._SF_cal_term_ext_amp;
	pwr_brd_aux	= (pwr_brd_aux - _scale._bias_cal_term_aux_amp) * _scale._SF_cal_term_aux_amp;

	/* generate a new report */
	prb.timestamp = hrt_absolute_time();

	prb.pwr_brd_status  	= pwr_brd_status;
	prb.pwr_brd_led_status  = pwr_brd_led_status;

	prb.pwr_brd_blink_reg  = pwr_brd_blink_reg;
	prb.pwr_brd_led_1_pwr  = pwr_brd_led_1_pwr;
	prb.pwr_brd_led_2_pwr  = pwr_brd_led_2_pwr;
	prb.pwr_brd_led_3_pwr  = pwr_brd_led_3_pwr;
	prb.pwr_brd_led_4_pwr  = pwr_brd_led_4_pwr;

	prb.pwr_brd_system_volt = pwr_brd_system_v;
	prb.pwr_brd_servo_volt  = pwr_brd_servo_v;
	prb.pwr_brd_digital_volt = pwr_brd_digital_v;

	prb.pwr_brd_mot_l_amp 	= pwr_brd_mot_l;
	prb.pwr_brd_mot_r_amp 	= pwr_brd_mot_r;

	prb.pwr_brd_digital_amp  = pwr_brd_digital;
	prb.pwr_brd_analog_amp   = pwr_brd_analog;
	prb.pwr_brd_ext_amp      = pwr_brd_ext;
	prb.pwr_brd_aux_amp  	 = pwr_brd_aux;

	_reports->force(&prb);

	/* notify anyone waiting for data */
	if (pwr_brd_notify) {
		poll_notify(POLLIN);
	}

	if (pwr_brd_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_pwr_brd), _topic, &prb);
	}

	perf_end(_sample_perf);
	return OK;

}

int
PWR_BRD::ioctl(struct file *filp, int cmd, unsigned long arg)
{

	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, PWR_BRD_MAX_DATA_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(PWR_BRD_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_call_interval = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_interval == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return 1000000 / _call_interval;

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
		return reset();

	case PWRBRDIOCSSAMPLERATE:
		return ioctl(filp, SENSORIOCSPOLLRATE, arg);

	case PWRBRDIOCGSAMPLERATE:
		return 1000000 / _call_interval;

	case PWRBRDIOCSSCALE: {
			/* copy power board bias and SF cal values*/
			struct pwr_brd_calibration_s *s = (struct pwr_brd_calibration_s *) arg;
			memcpy(&_scale, s, sizeof(_scale));

			_bias_cal_term_system_volt = _scale._bias_cal_term_system_volt;
			_SF_cal_term_system_volt = _scale._SF_cal_term_system_volt;
			_bias_cal_term_servo_volt = _scale._bias_cal_term_servo_volt;
			_SF_cal_term_servo_volt = _scale._SF_cal_term_servo_volt;
			_bias_cal_term_mot_l_amp = _scale._bias_cal_term_mot_l_amp;
			_SF_cal_term_mot_l_amp = _scale._SF_cal_term_mot_l_amp;
			_bias_cal_term_mot_r_amp = _scale._bias_cal_term_mot_r_amp;
			_SF_cal_term_mot_r_amp = _scale._SF_cal_term_mot_r_amp;
			_bias_cal_term_digital_amp = _scale._bias_cal_term_digital_amp;
			_SF_cal_term_digital_amp = _scale._SF_cal_term_digital_amp;
			_bias_cal_term_analog_amp = _scale._bias_cal_term_analog_amp;
			_SF_cal_term_analog_amp = _scale._SF_cal_term_analog_amp;
			_bias_cal_term_ext_amp = _scale._bias_cal_term_ext_amp;
			_SF_cal_term_ext_amp = _scale._SF_cal_term_ext_amp;
			_bias_cal_term_digital_volt = _scale._bias_cal_term_digital_volt;
			_SF_cal_term_digital_volt = _scale._SF_cal_term_digital_volt;
			_bias_cal_term_aux_amp = _scale._bias_cal_term_aux_amp;
			_SF_cal_term_aux_amp = _scale._SF_cal_term_aux_amp;
			return OK;
		}

	case PWRBRDIOLED: {
			/* copy power board LED term values*/
			struct pwr_brd_led_s *s = (struct pwr_brd_led_s *) arg;
			memcpy(&_pwr_brd_led, s, sizeof(_pwr_brd_led));

			_pwr_brd_led_blink_int  = _pwr_brd_led._pwr_brd_led_blink_int;
			_pwr_brd_led_blink_number = _pwr_brd_led._pwr_brd_led_blink_number;
			_pwr_brd_led_power_1 = _pwr_brd_led._pwr_brd_led_power_1;
			_pwr_brd_led_power_2 = _pwr_brd_led._pwr_brd_led_power_2;
			_pwr_brd_led_power_3 = _pwr_brd_led._pwr_brd_led_power_3;
			_pwr_brd_led_power_4 = _pwr_brd_led._pwr_brd_led_power_4;

			if ((_pwr_brd_led_power_1 > 100) || (_pwr_brd_led_power_2 > 100) || (_pwr_brd_led_power_3 > 100)
			    || (_pwr_brd_led_power_4 > 100)) {
				return -EIO;
			}

			if (OK != set_LED_blink_interval((uint8_t) _pwr_brd_led_blink_int)) {
				return -EIO;
			}

			if (OK != set_LED_number_of_blinks((uint8_t) _pwr_brd_led_blink_number)) {
				return -EIO;
			}

			if (OK != set_LED_power(LED_POW_1_REG, _pwr_brd_led_power_1)) {
				return -EIO;
			}

			if (OK != set_LED_power(LED_POW_2_REG, _pwr_brd_led_power_2)) {
				return -EIO;
			}

			if (OK != set_LED_power(LED_POW_3_REG, _pwr_brd_led_power_3)) {
				return -EIO;
			}

			if (OK != set_LED_power(LED_POW_4_REG, _pwr_brd_led_power_4)) {
				return -EIO;
			}

			return OK;
		}

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

int PWR_BRD::reset()
{
	//ToDo: write a reset function
	stop();
	return OK;
}


int
PWR_BRD::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		collect();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

/* Get registers value */
int
PWR_BRD::get_regs(uint8_t ptr, uint8_t *regs)
{
	uint8_t data[2];

	if (OK != transfer(&ptr, 1, &data[0], 2)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	*regs  = data[1];

	return OK;
}

/* Set registers value */
int
PWR_BRD::set_regs(uint8_t ptr, uint8_t value)
{
	uint8_t data[2];

	data[0] = ptr;
	data[1] = value;

	if (OK != transfer(&data[0], 2, nullptr, 0)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* read back the reg and verify */

	if (OK != transfer(&ptr, 1, &data[0], 2)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (data[1] != value) {
		return -EIO;
	}

	return OK;
}

/* Set LED power */
int
PWR_BRD::set_LED_power(uint8_t ptr, uint8_t pwr_brd_led_pwr)
{
	/* input power -> scale to percentage 0 -100% */

	pwr_brd_led_pwr = (uint8_t)((float)pwr_brd_led_pwr * 2.55f);

	if (OK != set_regs(ptr, pwr_brd_led_pwr)) {
		return -EIO;
	}

	return OK;
}

/* Set LED blink interval in sec*/
int
PWR_BRD::set_LED_blink_interval(uint8_t blink_interval_sec)
{
	uint8_t ptr = LED_BLINK_REG;
	uint8_t led_blink_Reg;

	if (OK != get_regs(ptr, &led_blink_Reg)) {
		return -EIO;
	}

	led_blink_Reg = ((led_blink_Reg << 4) & 0xf0) | (blink_interval_sec & 0x0f);

	if (OK != set_regs(LED_BLINK_REG, led_blink_Reg)) {
		return -EIO;
	}

	return OK;
}

/* Set LED number of blinks */
int
PWR_BRD::set_LED_number_of_blinks(uint8_t blink_number)
{
	uint8_t ptr = LED_BLINK_REG;
	uint8_t led_blink_Reg;

	if (OK != get_regs(ptr, &led_blink_Reg)) {
		return -EIO;
	}

	led_blink_Reg = ((blink_number << 4) & 0xf0) | (led_blink_Reg & 0x0f);

	if (OK != set_regs(LED_BLINK_REG, led_blink_Reg)) {
		return -EIO;
	}

	return OK;
}

/* Get system/servo voltage */
int
PWR_BRD::get_voltage(uint8_t ptr, float *voltage)
{
	uint8_t data[3];

	if (OK != transfer(&ptr, 1, &data[0], 3)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	*voltage = ((float)(((uint16_t)(data[2] & 0x3) << 8) | data[1])) * VOLTAGE_FULLSCALE / ADC_FULLSCALE;

	return OK;
}

/* Get channel current */
int
PWR_BRD::get_channel_current(uint8_t ptr, float *channel_current)
{
	uint8_t data[3];

	if (OK != transfer(&ptr, 1, &data[0], 3)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	*channel_current = ((float)(((uint16_t)(data[2] & 0x3) << 8) | data[1]) - ADC_FULLSCALE / 10.0f) * VOLTAGE_FULLSCALE /
			   ADC_FULLSCALE / CHANNEL_CURRENT_CONV_FARTOR;

	return OK;
}

/* Get motor current */
int
PWR_BRD::get_motor_current(uint8_t ptr, float *motor_current)
{
	uint8_t data[3];

	if (OK != transfer(&ptr, 1, &data[0], 3)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (ptr == MOT_L_CURRENT_REG) {
		*motor_current = ((float)(((uint16_t)(data[2] & 0x3) << 8) | data[1]) - ADC_FULLSCALE / 10.0f) * VOLTAGE_FULLSCALE /
				 ADC_FULLSCALE / MOTOR_L_CURRENT_CONV_FARTOR;

	} else if (ptr == MOT_R_CURRENT_REG) {
		*motor_current = ((float)(((uint16_t)(data[2] & 0x3) << 8) | data[1]) - ADC_FULLSCALE / 2.0f) * VOLTAGE_FULLSCALE /
				 ADC_FULLSCALE / MOTOR_R_CURRENT_CONV_FARTOR;

	} else if (ptr == EXT_CURRENT_REG) {
		*motor_current = ((float)(((uint16_t)(data[2] & 0x3) << 8) | data[1]) - ADC_FULLSCALE / 2.0f) * VOLTAGE_FULLSCALE /
				 ADC_FULLSCALE / EXT_CURRENT_CONV_FARTOR;

	} else {
		return -EIO;
	}

	return OK;
}

void
PWR_BRD::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_good_transfers);
	_reports->print_info("pwr_brd queue");

	printf("offsets system voltage (%.2f)\n", (double)_scale._bias_cal_term_system_volt);
	printf("scaling system voltage (%.2f)\n", (double)_scale._SF_cal_term_system_volt);

	printf("offsets mot L current (%.2f)\n", (double)_scale._bias_cal_term_mot_l_amp);
	printf("scaling mot L current(%.2f)\n", (double)_scale._SF_cal_term_mot_l_amp);

	printf("offsets mot R current(%.2f)\n", (double)_scale._bias_cal_term_mot_r_amp);
	printf("scaling mot R current(%.2f)\n", (double)_scale._SF_cal_term_mot_r_amp);

	printf("offsets servo voltage (%.2f)\n", (double)_scale._bias_cal_term_servo_volt);
	printf("scaling servo voltage (%.2f)\n", (double)_scale._SF_cal_term_servo_volt);

	printf("offsets analog current (%.2f)\n", (double)_scale._bias_cal_term_analog_amp);
	printf("scaling analog current (%.2f)\n", (double)_scale._SF_cal_term_analog_amp);

	printf("offsets digital voltage (%.2f)\n", (double)_scale._bias_cal_term_digital_volt);
	printf("scaling digital voltage (%.2f)\n", (double)_scale._SF_cal_term_digital_volt);

	printf("offsets digital current (%.2f)\n", (double)_scale._bias_cal_term_digital_amp);
	printf("scaling system current (%.2f)\n", (double)_scale._SF_cal_term_digital_amp);

	printf("offsets extension current (%.2f)\n", (double)_scale._bias_cal_term_ext_amp);
	printf("scaling extension current (%.2f)\n", (double)_scale._SF_cal_term_ext_amp);

	printf("offsets aux current (%.2f)\n", (double)_scale._bias_cal_term_aux_amp);
	printf("scaling aux current (%.2f)\n", (double)_scale._SF_cal_term_aux_amp);

	printf("\n");
}

int
pwr_brd_main(int argc, char *argv[])
{
	bool external_bus = true;
	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "X:")) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		default:
			pwr_brd::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		pwr_brd::start(external_bus);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		pwr_brd::test(external_bus);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		pwr_brd::reset(external_bus);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		pwr_brd::info(external_bus);
	}

	pwr_brd::usage();
	exit(1);
}
