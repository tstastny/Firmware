#ifndef PWR_BRD_HPP_
#define PWR_BRD_HPP_

#include <px4_config.h>
#include <systemlib/param/param.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>
#include <px4_log.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <nuttx/wqueue.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_pwr_brd.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include "parameters.h"

using namespace pwr_brd;

#define PWR_BRD_BUS				PX4_I2C_BUS_EXPANSION

#define PWR_BRD_SLAVE_ADDRESS   0x49    /* PWR_BRD I2C address									*/

#define LED_STATS_REG			0x00	/* Pointer to the address of the led board status register 				*/
#define LED_BLINK_REG			0x01	/* Pointer to the address of the led blink register  					*/
#define LED_POW_1_REG			0x02	/* Pointer to the address of the led power register of channel 1 		*/
#define LED_POW_2_REG			0x03	/* Pointer to the address of the led power register of channel 2  		*/
#define LED_POW_3_REG			0x04	/* Pointer to the address of the led power register of channel 3  		*/
#define LED_POW_4_REG			0x05	/* Pointer to the address of the led power register of channel 4  		*/
#define BRD_STATS_REG			0x06	/* Pointer to the address of the board status register 					*/
#define DIGITAL_CURRENT_REG		0x07	/* Pointer to the address of board digital current register				*/
#define ANALOG_CURRENT_REG		0x09	/* Pointer to the address of board analog current register				*/
#define EXT_CURRENT_REG			0x0b	/* Pointer to the address of board EXT current register					*/
#define DIGITAL_VOLTAGE_REG		0x0d	/* Pointer to the address of board digital voltage register				*/
#define MOT_L_CURRENT_REG		0x0f	/* Pointer to the address of board current register of motor channel    */
#define AUX_CURRENT_REG			0x11	/* Pointer to the address of board AUX current register					*/
#define SYSTEM_VOLT_REG			0x13	/* Pointer to the address of board system voltage register			  	*/
#define SERVO_VOLT_REG			0x15	/* Pointer to the address of board servo voltage register			  	*/

#define MOT_R_CURRENT_REG		0x0b	/* Pointer to the address of board current register of motor channel R  */

#define ADC_FULLSCALE				1024.0f
#define VOLTAGE_FULLSCALE			3300.0f
#define CHANNEL_CURRENT_CONV_FARTOR	132.0f
#define MOTOR_L_CURRENT_CONV_FARTOR	66.0f
#define MOTOR_R_CURRENT_CONV_FARTOR	33.0f
#define EXT_CURRENT_CONV_FARTOR	    33.0f
#define SYSTEM_VOLTAGE_SCALE		9.333f
#define SERVO_VOLTAGE_SCALE			2.0f
#define DIGITAL_VOLTAGE_SCALE		2.0f

#define ENA_MOT_R false

#define PWR_BRD_MAX_DATA_RATE     10

#define PWR_BRD_BUS_SPEED                     1000*100

/* This value is set based on Max output data rate value */
#define PWR_BRD_CONVERSION_INTERVAL          (1000000 / 100) /* microseconds */

/* arithmetic helper macro: convert millivolt to volt */
#define MV_TO_V(_x)		( (_x) * 0.001f )

class PWR_BRD : public device::I2C
{
public:
	PWR_BRD(int bus, const char *path);
	virtual ~PWR_BRD();

	virtual int             init();
	virtual ssize_t       read(struct file *filp, char *buffer, size_t buflen);
	virtual int       ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Stop automatic measurement.
	 */
	void            stop();

	/**
	  * Diagnostics - print some basic information about the driver.
	  */
	void            print_info();

protected:
	virtual int       probe();

private:
	work_s            _work{};

	bool _running;

	/* altitude conversion calibration */
	unsigned        _call_interval;

	pwr_brd_report _report {};
	ringbuffer::RingBuffer  *_reports;

	bool            _collect_phase;

	struct pwr_brd_calibration_s    _scale;

	orb_advert_t        _topic;

	perf_counter_t      _sample_perf;
	perf_counter_t      _bad_transfers;
	perf_counter_t      _good_transfers;
	perf_counter_t      _measure_perf;
	perf_counter_t      _comms_errors;
	perf_counter_t      _duplicates;

	bool            _got_duplicate;

	//pwr_brd_cal_term	_pwr_brd_cal_term;
	struct pwr_brd_led_s	_pwr_brd_led;

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

	unsigned _pwr_brd_led_blink_int;
	unsigned _pwr_brd_led_blink_number;
	unsigned _pwr_brd_led_power_1;
	unsigned _pwr_brd_led_power_2;
	unsigned _pwr_brd_led_power_3;
	unsigned _pwr_brd_led_power_4;

	int   		    	_params_sub{-1};			/**< notification of parameter updates */

	pwr_brd_report   _last_report {};           /**< used for info() */

	Parameters			_parameters{};			/**< local copies of interesting parameters */
	ParameterHandles	_parameter_handles{};	/**< handles for interesting parameters */

	/**
	 * Start automatic measurement.
	 */
	void            start();

	int     measure(); //start measure
	int     collect(); //get results and publish
	int		parameters_update(); //

	static void     cycle_trampoline(void *arg);
	void            cycle(); //main execution

	/**
	 * Read the specified number of bytes from PWR_BRD.
	 *
	 * @param reg       The register to read.
	 * @param data      Pointer to buffer for bytes read.
	 * @param len       Number of bytes to read
	 * @return          OK if the transfer was successful, -errno otherwise.
	 */
	int             get_data(uint8_t reg, uint8_t *data, unsigned len);

	/**
	 * Resets the chip.
	 */
	int             reset();

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int             self_test();

	/**
	 * Get power board registers values
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			get_regs(uint8_t ptr, uint8_t *regs);

	/**
	 * Set power board registers values
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			set_regs(uint8_t ptr, uint8_t value);

	/**
	 * Set LEDs power values
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			set_LED_power(uint8_t ptr, uint8_t pwr_brd_led_pwr);

	/**
	 * Set LEDs blink interval in sec
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			set_LED_blink_interval(uint8_t blink_interval_sec);

	/**
	 * Set LEDs number of blinks
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			set_LED_number_of_blinks(uint8_t blink_number);

	/**
	 * Get system/servo voltage values
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			get_voltage(uint8_t ptr, float *voltage);

	/**
	 * Get power board channel current values
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			get_channel_current(uint8_t ptr, float *channel_current);

	/**
	 * Get power board motor current values
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			get_motor_current(uint8_t ptr, float *motor_current);

	/**
	 * Get power board data
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			get_pwr_brd_data();

	/* do not allow to copy this class due to pointer data members */
	PWR_BRD(const PWR_BRD &);
	PWR_BRD operator=(const PWR_BRD &);

};



#endif /* PWR_BRD_HPP_ */
