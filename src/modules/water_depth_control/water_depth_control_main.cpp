
/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file water_depth_control_main.cpp
 *
 * HippoCampus Keep Water Depth Controller.
 *
 * Based on rover steering control example by Lorenz Meier <lorenz@px4.io>
 *
 * @author Tobias Johannink
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <geo/geo.h>

/**
 * Water depth control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int water_depth_control_main(int argc, char *argv[]);

class WaterDepthControl {
public:
    /**
     * Constructor
     */
    WaterDepthControl();

    /**
     * Destructor, also kills the main task
     */
    ~WaterDepthControl();

    /**
     * Start the keep water depth task.
     *
     * @return  OK on success.
     */
    int start();

private:

    bool    _task_should_exit;  /**< if true, task_main() should exit */
    int     _control_task;      /**< task handle */

    void task_main();

    static void task_main_trampoline(int argc, char *argv[]);


};

namespace water_depth_control
{

WaterDepthControl	*g_control;
}


//define Constructor
WaterDepthControl::WaterDepthControl() :

    _task_should_exit(false),
    _control_task(-1)

{

}

//define Destructor
WaterDepthControl::~WaterDepthControl()
{
    if (_control_task != -1) {
        /* task wakes up every 100ms or so at the longest */
        _task_should_exit = true;

        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do {
            /* wait 20ms */
            usleep(20000);

            /* if we have given up, kill it */
            if (++i > 50) {
                px4_task_delete(_control_task);
                break;
            }
        } while (_control_task != -1);
    }


    water_depth_control::g_control = nullptr;
}


//define start function
int WaterDepthControl::start()
{
    ASSERT(_control_task == -1);

    /* start the task */
    _control_task = px4_task_spawn_cmd("water_depth_control",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_MAX - 5,
                       1500,
                       (px4_main_t)&WaterDepthControl::task_main_trampoline,
                       nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

void WaterDepthControl::task_main_trampoline(int argc, char *argv[])
{
    water_depth_control::g_control->task_main();
}

//main task
void WaterDepthControl::task_main()
{
    PX4_INFO("Output water_depth_control!");
}


//main function
int water_depth_control_main(int argc, char *argv[])
{
     // PX4_INFO("Output water_depth_control!");

     if (argc < 2) {
         warnx("usage: water_depth_control {start|stop|status}");
         return 1;
     }

     if (!strcmp(argv[1], "start")) {

         if (water_depth_control::g_control != nullptr) {
             warnx("already running");
             return 1;
         }

         water_depth_control::g_control = new WaterDepthControl;

         if (water_depth_control::g_control == nullptr) {
             warnx("alloc failed");
             return 1;
         }

         if (OK != water_depth_control::g_control->start()) {
             delete water_depth_control::g_control;
             water_depth_control::g_control = nullptr;
             warnx("start failed");
             return 1;
         }

         return 0;
     }

     if (!strcmp(argv[1], "stop")) {
         if (water_depth_control::g_control == nullptr) {
             warnx("not running");
             return 1;
         }

         delete water_depth_control::g_control;
         water_depth_control::g_control = nullptr;
         return 0;
     }

     if (!strcmp(argv[1], "status")) {
         if (water_depth_control::g_control) {
             warnx("running");
             return 0;

         } else {
             warnx("not running");
             return 1;
         }
     }

     warnx("unrecognized command");
     return 1;

}
