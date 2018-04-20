
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
 * @file remote_control_main.cpp
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
#include <uORB/topics/control_state.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/pressure.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <geo/geo.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/att_pos_mocap.h>

/**
 * Remote control app start / stop handling function
 *
 * @ingroup apps
 */

extern "C" __EXPORT int remote_control_main(int argc, char *argv[]);

class RemoteControl {
public:
    /**
      * Constructor
      */
     RemoteControl();
//test
     /**
      * Destructor, also kills the main task
      */
     ~RemoteControl();

     int start();
 
private:

    bool       _task_should_exit;      /**< if true, task_main() should exit */
    int        _control_task;          /**< task handle */
    float      _thrust_sp;             /**< thrust setpoint */
    int        _ctrl_state_sub;        /**< control state subscription */
    int        _v_att_sp_sub;          /**< vehicle attitude setpoint subscription */
    int        _pressure_raw;
    int        _v_att_sub;             /**< vehicle attitude subscription */
    int        _params_sub;            /**< parameter updates subscription */
    float      _roh_g;
    float      _p_zero;
    int        counter;
    int         _radio_output;


    

    float water_depth;                  /**< actual water depth in m */
    float water_depth_err;              /**< Error actual water depth and desired water depth */
    float water_depth_pd_err;           /**< PD-Controller: Water depth and water depth SMO */
    float control_depth;                /**< Value for engine */


    float _pressure_new;
    float _pressure_time_old;
    float _pressure_time_new;

    
    /**< SLIDING-MODE-OBSERVER (SMO) */
    float xhat1;            /**< Estimated depth in m */
    float xhat2;            /**< Estimated velocity in m/s */
    float xhat1_prev;       /**< Estimated depth at previous time step in m */
    float xhat2_prev;       /**< Estimated velocity at previous time step in m/s */
    float iterationtime;    /**< Time pro Iteration */
    float water_depth_smo;  /**< Outcome water depth SMO in m */





    orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
    orb_advert_t   _position_pub;

     struct control_state_s             _ctrl_state;    /**< control state */
     struct vehicle_rates_setpoint_s    _v_rates_sp;    /**< vehicle rates setpoint */
     struct vehicle_attitude_setpoint_s _v_att_sp;      /**< vehicle attitude setpoint */
     struct actuator_controls_s			_actuators;			/**< actuator controls */
     struct vehicle_attitude_s           _v_att;             /**< vehicle attitude */				/**< raw sensor values incl ADC */
     struct vehicle_local_position_s    _pos;           /**< Mavlink Topic to visualize the values*/
     struct manual_control_setpoint_s    _r_att_sp;

     perf_counter_t     _loop_perf;     /**< loop performance counter */
     perf_counter_t     _controller_latency_perf;


     math::Vector<3>    _att_control;   /**< attitude control vector */
     math::Vector<3>      torques;
       math::Vector<3> e_R_vec;
         math::Vector <3> omega;
 
     math::Matrix<3, 3> _R_sp;              /**< rotation matrix setpoint */
    math::Matrix<3, 3> e_R;
    

     struct {
         param_t depth;
         param_t roll_setpoint;
         param_t roll_p_set;
         param_t roll_d_set;

         // SMO: Model parameters
         param_t rho; // Observer parameter
         param_t tau; // Observer parameter
         param_t phi; // Observer parameter

         param_t depth_d_gain;
         param_t depth_p_gain;
     }		_params_handles;		/**< handles for interesting parameters */

     struct {
         float depth;
         float roll_setpoint;
         float roll_p_set;
         float roll_d_set;

         //SLIDING-MODE-OBSERVER (SMO)
         float rho;
         float tau;
         float phi;

         float depth_d_gain;
         float depth_p_gain;
     }		_params;


     int   parameters_update();
     void   parameter_update_poll();




     static void task_main_trampoline(int argc, char *argv[]);

     void control_attitude();

     float get_xhat2(float x1, float iterationtime);

     float get_xhat1(float x1, float iterationtime);

     // sat functio
     float sat(float x, float gamma);

     void vehicle_attitude_setpoint_poll();




     /**
      * Check for control state updates.
      */
     void control_state_poll();

     void task_main();




};

namespace remote_control
{
    RemoteControl	*g_control;
}

//define Constructor
RemoteControl::RemoteControl() :

    _task_should_exit(false),/**
         * Check for control state updates.
         */
    _control_task(-1),

    //subscriptions
    _ctrl_state_sub(-1),
    _v_att_sub(-1),
    _params_sub(-1),

    // publications

    _actuators_0_pub(nullptr),

    _ctrl_state{},

    /* performance counters */
    _loop_perf(perf_alloc(PC_ELAPSED, "remote_control")),
    _controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))

{
    memset(&_v_att_sp, 0, sizeof(_v_att_sp));
    memset(&_actuators, 0, sizeof(_actuators));
    memset(&_v_att, 0, sizeof(_v_att));

    /* Counter for setting surface air pressure */
    counter = 1;
    _roh_g = 98.1;

    _pressure_new = 0;
    _pressure_time_old = 0;
    _pressure_time_new = 0;

    // SMO
    xhat1 = 0.0; //Estimated depth in m
    xhat2 = 0.0; //Estimated velocity in m/s


    _params_handles.depth = param_find("DEPTH");
    _params_handles.roll_setpoint = param_find("ROLL_SP");
    _params_handles.roll_p_set = param_find("ROLL_P_SP");
    _params_handles.roll_d_set = param_find("ROLL_D_SP");

    _params_handles.rho             =   param_find("SMO_RHO");
    _params_handles.tau             =   param_find("SMO_TAU");
    _params_handles.phi             =   param_find("SMO_PHI");

    _params_handles.depth_p_gain    =   param_find("DEPTH_P_SP");
    _params_handles.depth_d_gain    =   param_find("DEPTH_D_SP");

    /* fetch initial parameter values */
    parameters_update();

}

//define Destructor
RemoteControl::~RemoteControl()
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

    remote_control::g_control = nullptr;
}

int RemoteControl::parameters_update()
{
    param_get(_params_handles.depth, &(_params.depth));
    param_get(_params_handles.roll_setpoint, &(_params.roll_setpoint));
    param_get(_params_handles.roll_p_set, &(_params.roll_p_set));
    param_get(_params_handles.roll_d_set, &(_params.roll_d_set));

    param_get(_params_handles.rho, &(_params.rho));
    param_get(_params_handles.tau, &(_params.tau));
    param_get(_params_handles.phi, &(_params.phi));

    param_get(_params_handles.depth_p_gain, &(_params.depth_p_gain));
    param_get(_params_handles.depth_d_gain, &(_params.depth_d_gain));

    return OK;
}

void RemoteControl::parameter_update_poll()
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
int RemoteControl::start()
{
    ASSERT(_control_task == -1);

    /* start the task */
    _control_task = px4_task_spawn_cmd("remote_control",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_MAX - 5,
                       1500,
                       (px4_main_t)&RemoteControl::task_main_trampoline,
                       nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

void RemoteControl::task_main_trampoline(int argc, char *argv[])
{
    remote_control::g_control->task_main();
}


void RemoteControl::vehicle_attitude_setpoint_poll()
{
    /* check if there is a new rates setpoint */
    bool updated;
    orb_check(_v_att_sp_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
    }
}



void RemoteControl::control_state_poll()
{
    /* check if there is a new message */
    bool updated;
    orb_check(_ctrl_state_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
    }
}

float RemoteControl::get_xhat2(float x1, float time) {

    xhat1 = get_xhat1(x1, time);


   // xhat2 = xhat2_prev + (iterationtime / (0.1 * tau))* (-xhat2_prev - rho * sat(xhat1 - x1,1));
    xhat2 = xhat2_prev + (time / _params.tau) * (-xhat2_prev - _params.rho * sat(xhat1 - x1,_params.phi));

    xhat2_prev = xhat2;
        return xhat2; //xhat2 = geschätzte Geschwindigkeit
}


float RemoteControl::get_xhat1(float x1, float time) {
    //xhat1 = xhat1_prev - (iterationtime / 0.1) * rho * sat(xhat1_prev - x1, 1);
        xhat1 = xhat1_prev - (time / 1) * _params.rho * sat(xhat1_prev - x1, _params.phi);
    xhat1_prev = xhat1; //xhat1 = geschätzte Tiefe
    return xhat1;
}

// sat functio
float RemoteControl::sat(float x, float gamma) {
    float y = math::max(math::min(1.0f, x / gamma), -1.0f);
    return y;
}

//define Pressure Depth Control
void RemoteControl::control_attitude()
{

            control_state_poll();

            /* pressure sensor control start */
            struct pressure_s press;

            /* get pressure value from sensor*/
            orb_copy(ORB_ID(pressure), _pressure_raw, &press);
            orb_copy(ORB_ID(manual_control_setpoint), _radio_output, &_r_att_sp);

     // Depth
     // Note: Gain/Params in file "remote_control_params.c" depend on used hardware (AUV length, weight, ...).
     //       Maybe, you must tune the gains!

            /* set surface air pressure  */
            if (counter == 1){
                _p_zero = press.pressure_mbar;
                counter = 0;
            }

            /* set actual pressure from the sensor and absolute time to a new controler value */
            _pressure_new = press.pressure_mbar;
            _pressure_time_new = hrt_absolute_time();

            /* calculate actual water depth */
            water_depth = ( _pressure_new - _p_zero ) / ( _roh_g ); //unit meter

            /* calculate iterationtime for Sliding Mode Observer */
            iterationtime = _pressure_time_new - _pressure_time_old;
            /* scale interationtime*/
            iterationtime = iterationtime * 0.0000015f;

            /* calculate d-component of the controler by using a Sliding Mode Observer for accounting possible future trends of the error */
            water_depth_smo = get_xhat1(water_depth,iterationtime);

            /* set actual pressure from the sensor and absolute time to a "old" controler value */
            _pressure_time_old = _pressure_time_new;

            /* calculate the water depth error */
            water_depth_err = water_depth - _params.depth;

            /* use a pd-controller for water depth error*/
            water_depth_pd_err = water_depth_err - water_depth_smo * _params.depth_d_gain;
                 /* multiply with a p-gain */
                 control_depth = _params.depth_p_gain * water_depth_pd_err;


     // Attitude
     // Note: Gain/Params in file "remote_control_params.c" depend on used hardware (AUV length, weight, ...).
     //       Maybe, you must tune the gains!

            /* get current rates from sensors */
            omega(0) = _v_att.rollspeed;
            // omega(1) for Pitchspeed
            // omega(2) for Yawspeed

            /* PD-Controller: Roll */
            float roll_err = _params.roll_setpoint - _v_att.roll;

            float roll_p = roll_err * _params.roll_p_set;

            float roll_d = omega(0) * _params.roll_d_set;

            torques(0) = roll_p + roll_d;


            /* Values for engine */
            _att_control(0) = torques(0);       // Roll     (Controller)
            _att_control(1) = -(_r_att_sp.x);   // Pitch    (Remote)
            //_att_control(1) = control_depth   // Pitch    (Controller - Use this instead of Pitch (Remote) for pitch control via depth sensor)
            _att_control(2) = _r_att_sp.y;      // Yaw      (Remote)
            _thrust_sp = _r_att_sp.z - 0.5f;    // Thrust   (Remote)

}




//main task
void RemoteControl::task_main()
{
    //do subscriptions
    _v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _pressure_raw = orb_subscribe(ORB_ID(pressure));
    _params_sub = orb_subscribe(ORB_ID(parameter_update));

    _ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
    _radio_output = orb_subscribe(ORB_ID(manual_control_setpoint));


    /* initialize parameters cache */
    parameters_update();

    /* advertise actuator controls */
    _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
    _position_pub = orb_advertise(ORB_ID(vehicle_local_position), &_pos);
    
    

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


            /* start controller */

                /**< Remote Controller */
                control_attitude();
            
            /* end controller */

            /* publish actuator controls */
            _actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
            _actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
            _actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
            _actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
            _actuators.timestamp = hrt_absolute_time();
            _actuators.timestamp_sample = _v_att.timestamp;
            
            orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);
            orb_publish(ORB_ID(vehicle_local_position), _position_pub, &_pos);
           

            perf_end(_controller_latency_perf);
       }

        perf_end(_loop_perf);
    }

    _control_task = -1;
    return;

}



int remote_control_main(int argc, char *argv[])
{
    if (argc < 2) {
            warnx("usage: remote_control {start|stop|status}");
            return 1;
        }

        if (!strcmp(argv[1], "start")) {

            if (remote_control::g_control != nullptr) {
                warnx("already running");
                return 1;
            }

            remote_control::g_control = new RemoteControl;

            if (remote_control::g_control == nullptr) {
                warnx("alloc failed");
                return 1;
            }

            if (OK != remote_control::g_control->start()) {
                delete remote_control::g_control;
                remote_control::g_control = nullptr;
                warnx("start failed");
                return 1;
            }

            return 0;
        }

        if (!strcmp(argv[1], "stop")) {
            if (remote_control::g_control == nullptr) {
                warnx("not running");
                return 1;
            }

            delete remote_control::g_control;
            remote_control::g_control = nullptr;
            return 0;
        }

  //      if (!strcmp(argv[1], "status")) {
 //           if (remote_control::g_control) {
 //               warnx("running");
//                return 0;

  //          } else {
   //             warnx("not running");
   //             return 1;
    //        }
   //     }
    
        warnx("unrecognized command");
        return 1;
}
