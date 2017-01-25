
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
#include <uORB/topics/pressure.h>
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

    int     _v_att_sub;             /**< vehicle attitude subscription */
    int     _v_att_sp_sub;           /**< vehicle attitude setpoint subscription */
    int     _v_rates_sp_sub;        /**< vehicle rates setpoint subscription */
    int     _params_sub;            /**< vehicle attitude subscription */
    int     _pressure_sub;          /**< pressure subscription */

    orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */

    struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
    struct vehicle_attitude_setpoint_s  _v_att_sp;          /**< vehicle attitude setpoint */
    struct actuator_controls_s			_actuators;			/**< actuator controls */
    struct vehicle_attitude_s           _v_att;             /**< vehicle attitude */

    perf_counter_t	_loop_perf;			/**< loop performance counter */
    perf_counter_t	_controller_latency_perf;

     math::Vector<3>		_angles_sp;	/**< angular rates on previous step */
   // math::Vector<3>		_rates_sp_prev; /**< previous rates setpoint */
   // math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
   // math::Vector<3>		_rates_int;		/**< angular rates integral error */
   // float				_thrust_sp;		/**< thrust setpoint */
    math::Vector<3>		_att_control;	/**< attitude control vector */
   // math::Matrix<3, 3>  _I;				/**< identity matrix */

    struct {
        param_t roll_p;
        param_t pitch_p;
        param_t yaw_p;

        param_t roll_rate_p;
        param_t pitch_rate_p;
        param_t yaw_rate_p;

        param_t control_mode;
    }		_params_handles;		/**< handles for interesting parameters */

    struct {
        float roll_p;
        float pitch_p;
        float yaw_p;

        float roll_rate_p;
        float pitch_rate_p;
        float yaw_rate_p;

        int control_mode;
    }		_params;




    /**
     * Update our local parameter cache.
     */
    int parameters_update();

    /**
     * Check for parameter update and handle it.
     */
    void		parameter_update_poll();

    /**
     * Check for rates setpoint updates.
     */
    void		vehicle_rates_setpoint_poll();

    /**
     * Check for attitude setpoint updates.
     */
    void        vehicle_attitude_setpoint_poll();


    void task_main();

    static void task_main_trampoline(int argc, char *argv[]);

    void        control_attitude();

};

namespace water_depth_control
{

WaterDepthControl	*g_control;
}


//define Constructor
WaterDepthControl::WaterDepthControl() :

    _task_should_exit(false),
    _control_task(-1),

        /* subscriptions */
  //  _v_rates_sp_sub(-1),
  //  _params_sub(-1),
  //  _v_att_sub(-1),

    /* publications */

    _actuators_0_pub(nullptr),

    /* performance counters */
    _loop_perf(perf_alloc(PC_ELAPSED, "uw_att_control")),
    _controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))

{

    memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
   // memset(&_m_ctrl_sp, 0, sizeof(_m_ctrl_sp));
    memset(&_v_att_sp, 0, sizeof(_v_att_sp));
    memset(&_actuators, 0, sizeof(_actuators));
    memset(&_v_att, 0, sizeof(_v_att));




   _angles_sp.zero();
  //  _rates_sp.zero();
  //  _rates_sp_prev.zero();
  //  _rates_int.zero();
  //  _thrust_sp = 0.0f;
  //  _att_control.zero();

  //  _I.identity();

    _params_handles.roll_p			= 	param_find("UW_ROLL_P");
    _params_handles.roll_rate_p		= 	param_find("UW_ROLL_RATE_P");

    _params_handles.pitch_p			= 	param_find("UW_PITCH_P");
    _params_handles.pitch_rate_p	= 	param_find("UW_PITCH_RATE_P");

    _params_handles.yaw_p			= 	param_find("UW_YAW_P");
    _params_handles.yaw_rate_p		= 	param_find("UW_YAW_RATE_P");

    _params_handles.control_mode    =   param_find("UW_DEPTH_CONTROL_MODE");


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

    return OK;
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

void WaterDepthControl::vehicle_rates_setpoint_poll()
{
    /* check if there is a new rates setpoint */
    bool updated;
    orb_check(_v_rates_sp_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
    }
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


void WaterDepthControl::control_attitude()
{

    _pressure_sub = orb_subscribe(ORB_ID(pressure));

    px4_pollfd_struct_t fds[1];

    fds[0].fd = _pressure_sub;
    fds[0].events = POLLIN;

       int error_counter = 0;


       for (int i = 0; i < 2; i++) {
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

    //PX4_INFO("Output water_depth_control!");
/*
    vehicle_rates_setpoint_poll();
    vehicle_attitude_setpoint_poll();

    // desired
    _angles_sp(0) = _v_att_sp.roll_body;
    _angles_sp(1)= _v_att_sp.pitch_body;
    _angles_sp(2)= _v_att_sp.yaw_body;
*/
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
   // PX4_INFO("Output water_depth_control!");

    _v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    _v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));

    /* initialize parameters cache */
    parameters_update();

    /* advertise actuator controls */
    _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);







    /* wakeup source: vehicle attitude */
    px4_pollfd_struct_t fds[1];

    fds[0].fd = _v_att_sub;
    fds[0].events = POLLIN;

    int counter =0;
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

            counter++;
            control_attitude();
            PX4_INFO("Counter: %8.4f\t",
                     (double)counter);

            /*
            // start controler
            switch (_params.control_mode){
            case 0: control_attitude();
                break;
            }
            */

         }
        if (counter>50){
            break;
        }
    }

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
