/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file raw_pressure.c
 * HippoCampus Raw Pressure.
 *
 * @author Tobias Johannink 
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/pressure.h>

int _pressure_sub;

__EXPORT int raw_pressure_main(int argc, char *argv[]);

int raw_pressure_main(int argc, char *argv[])
{
    PX4_INFO("Output raw pressure!"); 

    _pressure_sub = orb_subscribe(ORB_ID(pressure));

    px4_pollfd_struct_t fds[] = {
        { .fd = _pressure_sub,    .events = POLLIN},
    };

    int error_counter = 0;

    for (int i = 0; i < 50; i++) {
        int poll_ret = px4_poll(fds, 1, 1000);

        if (poll_ret == 0) {
                   /* this means none of our providers is giving us data */
                   PX4_ERR("Got no data within a second");

        } else if (poll_ret < 0) {
                   /* this is seriously bad - should be an emergency */
               if (error_counter < 10 || error_counter % 50 == 0) {
                       /* use a counter to prevent flooding (and slowing us down) */
                       PX4_ERR("ERROR return value from poll(): %d", poll_ret);
               }

               error_counter++;

        } else {
        if (fds[0].revents & POLLIN) {
            struct pressure_s press;

            orb_copy(ORB_ID(pressure), _pressure_sub, &press);
            PX4_INFO("Pressure: %8.4f\t Temperature: %8.4f",
                     (double)press.pressure_mbar,
                     (double)press.temperature_degC);

        }
        }
    }

    PX4_INFO("exiting");

    return 0;
}
