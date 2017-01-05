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
 * Driver for ms5803 pressure sensor connected via I2C.
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
#include <uORB/topics/subsystem_info.h>

#include <float.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

/**
 * Pressure sensor driver start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int press_ms5803_main(int argc, char *argv[]);


#define TRUE 1
#define FALSE 0
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
#define PCA9685_BUS PX4_I2C_BUS_EXPANSION
#define PRESS_MS5803_ADDR 	0x76 /* 7-bit address of sensor, cf data sheet */
#define PRESS_MS5803_DEVICE_PATH	"/dev/press_ms5803"


class PRESS_MS5803 : public device::I2C
{
public:
	/**
	 * Constructor
	 */
	PRESS_MS5803(int bus = PX4_I2C_BUS_EXPANSION, uint16_t press_ms5803_addr = PRESS_MS5803_ADDR);

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
};

namespace
{
	PRESS_MS5803 *g_press_ms5803;	/// device handle
}

PRESS_MS5803::PRESS_MS5803(int bus, uint16_t press_ms5803_addr) :
	I2C("press_ms5803", PRESS_MS5803_DEVICE_PATH, bus, press_ms5803_addr, 100000)


PRESS_MS5803::~PRESS_MS5803()
{
	stop();
	g_press_ms5803 = nullptr;
}

int
PRESS_MS5803::init()
{
}

void
PRESS_MS5803::start()
{
}

void
PRESS_MS5803::stop()
{
}


int
press_ms5803_main(int argc, char *argv[])
{

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
		press_ms5803_usage();
		exit(1);
	}

	if (!strcmp(verb, "stop")) {
		delete g_press_ms5803;
		g_press_ms5803 = nullptr;
		exit(0);
	}

	press_ms5803_usage();
	exit(0);
}
