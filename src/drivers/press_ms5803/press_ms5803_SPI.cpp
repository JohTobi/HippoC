/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file press_ms5803.cpp
 *
 * Driver for ms5803 pressure sensor connected via SPI.
 *
 * @author Eugen Solowjow <eugen.solowjow@gmail.com>
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sched.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <ctype.h>


#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <uORB/uORB.h>
#include <uORB/topics/pressure.h>

#include <float.h>

#include <drivers/device/spi.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>



/**
 * Pressure sensor driver start / stop handling function
 *
 * @ingroup apps
 */

static const char commandline_usage[] = "usage: press_ms5803 start|status|stop";

extern "C" __EXPORT int press_ms5803_main(int argc, char *argv[]);


#define CMD_RESET 0x1E //ADC reset command (ADC = Analog-Digital-Converter)
#define CMD_ADC_READ 0x00 //ADC read command
#define CMD_ADC_CONV 0x40 //ADC conversion command
#define CMD_ADC_D1 0x00 //ADC D1 conversion
#define CMD_ADC_D2 0x10 //ADC D2 conversion
#define CMD_ADC_256 0x00 //ADC OSR=256 (OSR = Oversamplingrate)
#define CMD_ADC_512 0x02 //ADC OSR=512
#define CMD_ADC_1024 0x04 //ADC OSR=1024
#define CMD_ADC_2048 0x06 //ADC OSR=2056
#define CMD_ADC_4096 0x08 //ADC OSR=4096
#define CMD_PROM_RD 0xA0 //Prom read command

/* Configuration Constants */
#define PRESS_MS5803_ADDR 	0x77 /* 7-bit address of sensor. For 0x77 set CSB pin to low, cf data sheet */
#define PRESS_MS5803_DEVICE_PATH	"/dev/press_ms5803"
#define PRESS_MS5803_MEASUREMENT_INTERVAL_US	(1000000 / 20)	///< time in microseconds, measure at 20Hz


class PRESS_MS5803 : public device::SPI
{
public:
	/**
	 * Constructor
	 */
	PRESS_MS5803(int bus, const char *path, spi_dev_e device);

	/**
	 * Destructor
	 */
	virtual ~PRESS_MS5803();

	/**
	 * Initialize sensor.
	 *
	 * @return		OK on success.
	 */
	virtual  int		init();


private:

	/**
	 * Start periodic reads from the sensor
	 */
	void			start();

	/**
	 * Stop periodic reads from the sensor
	 */
	void			stop();

	/**
 	* static function that is called by worker queue
 	*/
	static void		cycle_trampoline(void *arg);

		/**
	 	* load all calibration coefficients
	 	*/
	void 			loadCoefs();

	/**
	 * perform a read from the pressure sensor and publish measurements
	 */
	void			cycle();

	/**
	 * compute pressure and temperature values
	 */
	void			calcPT();

	/**
	 * Perform ADC conversion
	 */
	 uint32_t		cmd_adc(uint8_t cmd);

	 /**
 	 * Read pressure computation coeffecients from PROM
 	 */
	 uint16_t	 	read_prom(int i);

	// internal variables
	work_s						_work;		///< work queue for scheduling reads
	orb_advert_t			_press_topic;	///< uORB pressure topic
    // orb_id_t					_press_orb_id;	///< uORB pressure topic ID
	double						_pressure_value;	///< pressure in bar (-1 means unknown)
	double						_temperature_value;	///< temperature in C
    uint32_t 					C[8];                  //coefficient storage

    struct pressure_s pressure;
    bool _collect_phase;


};

namespace
{
	PRESS_MS5803 *g_press_ms5803;	/// device handle
}

PRESS_MS5803::PRESS_MS5803(int bus, const char *path, spi_dev_e device) :
	SPI("press_ms5803", path, bus, device, SPIDEV_MODE3, 100000),
	_work{},
	_press_topic(nullptr),
    // _press_orb_id(nullptr),
	_pressure_value(0.0f),
    _temperature_value(0.0f),
    _collect_phase(false)



{
	memset(&_work, 0, sizeof(_work));
}


PRESS_MS5803::~PRESS_MS5803()
{
	g_press_ms5803 = nullptr;
}

int
PRESS_MS5803::init()
{
	// init orb id
    // _press_orb_id = ORB_ID(pressure);
	orb_subscribe(ORB_ID(pressure));

	//initialise SPI bus
	int ret = ENOTTY;
	ret = SPI::init();

	if (ret != OK) {
			errx(1, "failed to init SPI");
			return ret;
	} else {
			start();
	}

    return ret;
}

void
PRESS_MS5803::start()
{
    _collect_phase = false;

loadCoefs();
// schedule a cycle to start measurements
work_queue(HPWORK, &_work, (worker_t)&PRESS_MS5803::cycle_trampoline, this, 1);
}

void
PRESS_MS5803::stop()
{
	work_cancel(HPWORK, &_work);
}

void
PRESS_MS5803::cycle_trampoline(void *arg)
{
	PRESS_MS5803 *dev = (PRESS_MS5803 *)arg;

	dev->cycle();
}

void
PRESS_MS5803::loadCoefs() //Read from sensor on start
{
	for (int i = 0; i < 8; i++){

            C[i] = 0;
            usleep(50000);  //Wait 50ms

			C[i] = read_prom(i);
	}
   /* warnx("C3 %f", C[3]); */
}

void
PRESS_MS5803::cycle()
{
	// calculate pressure and temperature from spi sensor
	calcPT();

	// publish to orb
    pressure.pressure_mbar = (float32)_pressure_value;
    pressure.temperature_degC = (float32)_temperature_value;

    if (_press_topic != nullptr) {
        orb_publish(ORB_ID(pressure), _press_topic, &pressure);
    } else {
        _press_topic = orb_advertise(ORB_ID(pressure),&pressure);
    }

	// notify anyone waiting for data
	poll_notify(POLLIN);

	// schedule a fresh cycle call when the measurement is done
	work_queue(HPWORK, &_work, (worker_t)&PRESS_MS5803::cycle_trampoline, this,
		 USEC2TICK(PRESS_MS5803_MEASUREMENT_INTERVAL_US));

}

void
PRESS_MS5803::calcPT()
{
	// read data from sensor
    uint32_t D1 = cmd_adc(CMD_ADC_D1 + CMD_ADC_256);
    uint32_t D2 = cmd_adc(CMD_ADC_D2 + CMD_ADC_256);
   /*  warnx("D1 %f", D1);
      warnx("D2 %f", D2); */


  // Computation according to manufacturer
	int64_t dT = D2 - ((uint64_t)C[5] << 8);
	int64_t OFF  = ((uint32_t)C[2] << 16) + ((dT * (C[4]) >> 7));
	int64_t SENS = ((uint32_t)C[1] << 15) + ((dT * (C[3]) >> 8));

	_temperature_value = (2000 + (((uint64_t)dT * C[6]) / (float)(1 << 23))) / 100;
	int32_t TEMP = 2000 + (int64_t)dT * (int64_t)C[6] / (int64_t)(1 << 23);

	if(TEMP < 2000) { // if temperature lower than 20 Celsius
        float T1 = (TEMP - 2000) * (TEMP - 2000);
        int64_t OFF1  = (5 * T1) / 2;
        int64_t SENS1 = (5 * T1) / 4;

        if(TEMP < -1500) { // if temperature lower than -15 Celsius
            T1 = (TEMP + 1500) * (TEMP + 1500);
            OFF1  += 7 * T1;
            SENS1 += 11 * T1 / 2;
        }
        OFF -= OFF1;
        SENS -= SENS1;
        _temperature_value = (float)TEMP / 100;
    }
		_pressure_value = ((((int64_t)D1 * SENS ) >> 21) - OFF) / (double) (1 << 15) / 100.0;
}

uint32_t
PRESS_MS5803::cmd_adc(uint8_t cmd)
{


	// initiate pressure conversion
	{
	uint8_t cmd_temp = CMD_ADC_CONV + cmd;
	transfer(&cmd_temp, nullptr, 1);
	}

  usleep(900);

	uint8_t cmd_read[4] = {0, 0, 0, 0};
	// read sequence
	cmd_read[0] = CMD_ADC_READ;
	transfer(cmd_read, cmd_read, sizeof(cmd_read));

	uint32_t val = (cmd_read[1] << 16) + (cmd_read[2] << 8) + cmd_read[3];

	return val;
}

uint16_t
PRESS_MS5803::read_prom(int coef_num)
{
	uint8_t cmd[3] = {0, 0, 0};

	// send PROM READ command
	cmd[0] = CMD_PROM_RD + coef_num * 2;
	transfer(cmd, cmd, sizeof(cmd));

	uint16_t val = (cmd[1] << 8) + cmd[2];

	return val;
}


int
press_ms5803_main(int argc, char *argv[])
{

    if (argc < 2) {
            errx(1, "missing command\n%s", commandline_usage);
        }

        if (!strcmp(argv[1], "start")) {

            if (g_press_ms5803) {
                warnx("already running");
                exit(0);
            }

            g_press_ms5803 = new PRESS_MS5803(PX4_SPI_BUS_EXT, PRESS_MS5803_DEVICE_PATH, (spi_dev_e)PX4_SPIDEV_EXT3);

            if (g_press_ms5803 != nullptr && OK != g_press_ms5803->init()) {
                delete g_press_ms5803;
                g_press_ms5803 = nullptr;
            }

            exit(0);
        }

        if (!strcmp(argv[1], "stop")) {
            if (!g_press_ms5803) {
                warnx("not running");
                exit(0);
            }

            delete g_press_ms5803;
            g_press_ms5803 = nullptr;
            warnx("stopped");

            exit(0);
        }

        if (!strcmp(argv[1], "status")) {
            if (g_press_ms5803) {
                warnx("is running");

            } else {
                warnx("is not running");
            }

            exit(0);
        }

        errx(1, "unrecognized command\n%s", commandline_usage);

/*
    const char *verb = argv[optind];

    if (!strcmp(verb, "start")) {
        if (g_press_ms5803 != nullptr) {
            errx(1, "already started");

        } else {
			// create new global object
            g_press_ms5803 = new PRESS_MS5803();

            if (g_press_ms5803 == nullptr) {
                errx(1, "new failed");
            }

            if (OK != g_press_ms5803->init()) {
                delete g_press_ms5803;
                g_press_ms5803 = nullptr;
                errx(1, "init failed");
            }
        }

		exit(0);
	}

	if (g_press_ms5803 == nullptr) {
		warnx("not started");
		exit(1);
	}

	if (!strcmp(verb, "stop")) {
		delete g_press_ms5803;
		g_press_ms5803 = nullptr;
		exit(0);
	}

	exit(0);
*/
}
