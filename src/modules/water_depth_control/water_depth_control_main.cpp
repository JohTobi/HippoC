
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
 * @author Eugen Solowjow
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
#include <uORB/topics/pressure.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <geo/geo.h>
#include <uORB/topics/adc_report.h> // includes ADC readings

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

     int start();


private:

     bool       _task_should_exit;      /**< if true, task_main() should exit */
     int        _control_task;          /**< task handle */
     float      _thrust_sp;             /**< thrust setpoint */
     int        _v_att_sp_sub;          /**< vehicle attitude setpoint subscription */
     int        _pressure_raw;
     float      _pressure_set;
     int        _v_att_sub;             /**< vehicle attitude subscription */
     int        _params_sub;            /**< parameter updates subscription */
     int        _adc_sub_fd;         /**< raw sensor data subscription */


     orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */

     struct vehicle_attitude_setpoint_s _v_att_sp;      /**< vehicle attitude setpoint */
     struct actuator_controls_s			_actuators;			/**< actuator controls */
     struct vehicle_attitude_s           _v_att;             /**< vehicle attitude */
     struct adc_report_s 			_raw_adc;				/**< raw sensor values incl ADC */

     perf_counter_t     _loop_perf;     /**< loop performance counter */
     perf_counter_t     _controller_latency_perf;

     math::Vector<3>    _att_control;   /**< attitude control vector */



     struct {
         param_t roll_p;
         param_t pitch_p;
         param_t yaw_p;

         param_t roll_rate_p;
         param_t pitch_rate_p;
         param_t yaw_rate_p;

         param_t control_mode;

         param_t water_depth_p;
     }		_params_handles;		/**< handles for interesting parameters */

     struct {
         float roll_p;
         float pitch_p;
         float yaw_p;

         float roll_rate_p;
         float pitch_rate_p;
         float yaw_rate_p;

         int control_mode;

         float water_depth_p;
     }		_params;


     int   parameters_update();
     void   parameter_update_poll();


     void task_main();

     static void task_main_trampoline(int argc, char *argv[]);

     void control_attitude();

     void vehicle_attitude_setpoint_poll();

     void raw_adc_data_poll();
};

namespace water_depth_control
{
    WaterDepthControl	*g_control;
}

//define Constructor
WaterDepthControl::WaterDepthControl() :

    _task_should_exit(false),
    _control_task(-1),

    //subscriptions
    _v_att_sub(-1),
    _params_sub(-1),
    _adc_sub_fd(-1),

    // publications

    _actuators_0_pub(nullptr),

    /* performance counters */
    _loop_perf(perf_alloc(PC_ELAPSED, "water_depth_control")),
    _controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))

{
    memset(&_v_att_sp, 0, sizeof(_v_att_sp));
    memset(&_actuators, 0, sizeof(_actuators));
    memset(&_v_att, 0, sizeof(_v_att));

    _thrust_sp = 0.0f;
    _att_control.zero();

    _params_handles.roll_p			= 	param_find("UW_ROLL_P");
    _params_handles.roll_rate_p		= 	param_find("UW_ROLL_RATE_P");

    _params_handles.pitch_p			= 	param_find("UW_PITCH_P");
    _params_handles.pitch_rate_p	= 	param_find("UW_PITCH_RATE_P");

    _params_handles.yaw_p			= 	param_find("UW_YAW_P");
    _params_handles.yaw_rate_p		= 	param_find("UW_YAW_RATE_P");

    _params_handles.control_mode    =   param_find("UW_CONTROL_MODE");

    _pressure_set = param_find("WATER_DEPTH");
    _params_handles.water_depth_p = param_find("WATER_DEPTH_P");

    /* fetch initial parameter values */
    parameters_update();
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

int WaterDepthControl::parameters_update()
{
    param_get(_params_handles.roll_p, &(_params.roll_p));
    param_get(_params_handles.roll_rate_p, &(_params.roll_rate_p));

    param_get(_params_handles.pitch_p, &(_params.pitch_p));
    param_get(_params_handles.pitch_rate_p, &(_params.pitch_rate_p));

    param_get(_params_handles.yaw_p, &(_params.yaw_p));
    param_get(_params_handles.yaw_rate_p, &(_params.yaw_rate_p));

    param_get(_params_handles.control_mode, &(_params.control_mode));

    param_get(_pressure_set, &(_pressure_set));
    param_get(_params_handles.water_depth_p, &(_params.water_depth_p));

    return OK;
}

void WaterDepthControl::parameter_update_poll()
{
    bool updated;

    /* Check if parameters have changed */
    orb_check(_params_sub, &updated);

    if (updated) {
        /* read from param to clear updated flag (uORB API requirement) */
        struct parameter_update_s param_update;
        orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);

        parameters_update();
    }
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


void WaterDepthControl::vehicle_attitude_setpoint_poll()
{
    /* check if there is a new rates setpoint */
    bool updated;
    orb_check(_v_att_sp_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
    }
}

void WaterDepthControl::raw_adc_data_poll()
{
    /* Always update */
    bool updated = 1;

    /* copy adc raw data into local buffer */
    if (updated) {
        orb_copy(ORB_ID(adc_report), _adc_sub_fd, &_raw_adc);
    }
}

//define Pressure Depth Control
void WaterDepthControl::control_attitude()
{

            struct pressure_s press;

            orb_copy(ORB_ID(pressure), _pressure_raw, &press);

            //p-control
            float pressure_err =  press.pressure_mbar - _pressure_set;

            float control_depth = _params.water_depth_p * pressure_err;

            _thrust_sp = control_depth;

            usleep(100000);
}


//main task
void WaterDepthControl::task_main()
{
    //do subscriptions
    _v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _pressure_raw = orb_subscribe(ORB_ID(pressure));
    _params_sub = orb_subscribe(ORB_ID(parameter_update));
    _adc_sub_fd = orb_subscribe(ORB_ID(adc_report));

    /* initialize parameters cache */
    parameters_update();

    /* advertise actuator controls */
    _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);

    px4_pollfd_struct_t fds[1];

    fds[0].fd = _v_att_sub;
    fds[0].events = POLLIN;

    while (!_task_should_exit) {

        /* wait for up to 100ms for data */
        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

        /* timed out - periodic check for _task_should_exit */
        if (pret == 0) {
            continue;
        }

        /* this is undesirable but not much we can do - might want to flag unhappy status */
        if (pret < 0) {
            warn("mc att ctrl: poll error %d, %d", pret, errno);
            /* sleep a bit before next try */
            usleep(100000);
            continue;
        }

        perf_begin(_loop_perf);

        /* run controller on attitude changes */
        if (fds[0].revents & POLLIN) {

            /* copy attitude and control state topics */
            orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);

            /* check for updates in other topics */
            parameter_update_poll();

            //start controller
            control_attitude();

            //get ADC value and print it for debugging
            raw_adc_data_poll();
            printf("ADC 10:\t%8.4f\n", (double)_raw_adc.channel_value[6]);

            /* publish actuator controls */
            _actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
            _actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
            _actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
            _actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
            _actuators.timestamp = hrt_absolute_time();
            _actuators.timestamp_sample = _v_att.timestamp;

            orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);

            perf_end(_controller_latency_perf);
       }

        perf_end(_loop_perf);
    }

    _control_task = -1;
    return;

}



int water_depth_control_main(int argc, char *argv[])
{


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
