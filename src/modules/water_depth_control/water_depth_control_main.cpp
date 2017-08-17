
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
#include <uORB/topics/adc_report.h> // includes ADC readings
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/att_pos_mocap.h>

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
    int        _ctrl_state_sub;        /**< control state subscription */
    int        _v_att_sp_sub;          /**< vehicle attitude setpoint subscription */
    int        _pressure_raw;
    int        _v_att_sub;             /**< vehicle attitude subscription */
    int        _params_sub;            /**< parameter updates subscription */
    float      _roh_g;
    float      _p_zero;
    int        counter;

    float pitch_fac;
    
    float water_depth;                  /**< actual water depth in m */
    float water_depth_err;              /**< Error actual water depth and desired water depth */
    float water_depth_pd_err;           /**< PD-Controller: Water depth and water depth SMO */
    float control_depth;                /**< Value for engine */

    float _pressure_old;
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

    /**< ADC */
    float adc_input;
    float angle_input;
    float alpha_zero;
    float angle_error;
    float angle_p_control;

    int _adc_sub_fd;         /**< raw sensor data subscription */


    orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
    orb_advert_t   _position_pub;

     struct control_state_s             _ctrl_state;    /**< control state */
     struct vehicle_rates_setpoint_s    _v_rates_sp;    /**< vehicle rates setpoint */
     struct vehicle_attitude_setpoint_s _v_att_sp;      /**< vehicle attitude setpoint */
     struct actuator_controls_s			_actuators;			/**< actuator controls */
     struct vehicle_attitude_s           _v_att;             /**< vehicle attitude */
     struct adc_report_s 			_raw_adc;				/**< raw sensor values incl ADC */
     struct vehicle_local_position_s    _pos;           /**< Mavlink Topic to visualize the values*/

     perf_counter_t     _loop_perf;     /**< loop performance counter */
     perf_counter_t     _controller_latency_perf;


     math::Vector<3>    _att_control;   /**< attitude control vector */
     math::Vector<3>      torques;
    math::Vector<3> e_R_vec;
          math::Vector <3> omega;
 
     math::Matrix<3, 3> _R_sp;              /**< rotation matrix setpoint */
    math::Matrix<3, 3> e_R;
    

     struct {


         param_t roll_rate_p;
         param_t pitch_rate_p;
         param_t yaw_rate_p;


         param_t water_depth_sp;
         
         // SMO: Model parameters
         param_t rho; // Observer parameter
         param_t tau; // Observer parameter
         param_t phi; // Observer parameter
         
    
         param_t water_depth_pgain;
         param_t water_depth_dgain;

         param_t r_sp_xx;
         param_t r_sp_yx;
         param_t r_sp_zx;
         param_t r_sp_xy;
         param_t r_sp_yy;
         param_t r_sp_zy;
         param_t r_sp_xz;
         param_t r_sp_yz;
         param_t r_sp_zz;

         param_t pitch_gain;
         param_t yaw_gain;
         param_t roll_gain;

     }		_params_handles;		/**< handles for interesting parameters */

     struct {

         float roll_rate_p;
         float pitch_rate_p;
         float yaw_rate_p;

         float water_depth_sp;      /**> Desired water depth */
         
         //SLIDING-MODE-OBSERVER (SMO)
         ///Model parameters
         float rho;
         float tau;
         float phi;
         

         float water_depth_pgain;
         float water_depth_dgain;

         float r_sp_xx;
         float r_sp_yx;
         float r_sp_zx;
         float r_sp_xy;
         float r_sp_yy;
         float r_sp_zy;
         float r_sp_xz;
         float r_sp_yz;
         float r_sp_zz;
         
         float pitch_gain;
         float yaw_gain;
         float roll_gain;

     }		_params;


     int   parameters_update();
     void   parameter_update_poll();




     static void task_main_trampoline(int argc, char *argv[]);

     void control_attitude();
    
    void control_helix();

    void furuta_pendulum();
    
    float get_xhat2(float x1, float iterationtime);
    
    float get_xhat1(float x1, float iterationtime);
    
    // sat functio
    float sat(float x, float gamma);

     void vehicle_attitude_setpoint_poll();


     void raw_adc_data_poll();


     /**
      * Check for control state updates.
      */
     void control_state_poll();

     void task_main();




};

namespace water_depth_control
{
    WaterDepthControl	*g_control;
}

//define Constructor
WaterDepthControl::WaterDepthControl() :

    _task_should_exit(false),/**
         * Check for control state updates.
         */
    _control_task(-1),

    //subscriptions
    _ctrl_state_sub(-1),
    _v_att_sub(-1),
    _params_sub(-1),
    _adc_sub_fd(-1),

    // publications

    _actuators_0_pub(nullptr),

    _ctrl_state{},

    /* performance counters */
    _loop_perf(perf_alloc(PC_ELAPSED, "water_depth_control")),
    _controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))

{
    memset(&_v_att_sp, 0, sizeof(_v_att_sp));
    memset(&_actuators, 0, sizeof(_actuators));
    memset(&_v_att, 0, sizeof(_v_att));


    
 
    water_depth = 0;
    

    counter = 1;
    pitch_fac = 1.0;

    _roh_g = 98.1;
    

    _pressure_old = 0;
    _pressure_new = 0;
    
    _pressure_time_old = 0;
    _pressure_time_new = 0;
    
    
    iterationtime = 0;
   // rho = 10; //Observer parameter 1.5
   // tau = 1.5; //Observe rparameter 1.5
   // phi = 0.4; // 0.4
    ///Koordinaten
    xhat1 = 0.0; //Estimated depth in m
    xhat2 = 0.0; //Estimated velocity in m/s
    xhat1_prev = 0;
    //xhat1_prev[1] = 0; //Estimated depth at previous time step in m
    xhat2_prev = 0; //Estimated velocity at previous time step in m/s

    alpha_zero = 0;
   

    _params_handles.roll_rate_p		= 	param_find("UW_ROLL_RATE_P");

    _params_handles.pitch_rate_p	= 	param_find("UW_PITCH_RATE_P");

    _params_handles.yaw_rate_p		= 	param_find("UW_YAW_RATE_P");

    
    _params_handles.rho             =   param_find("RHO");
    _params_handles.tau             =   param_find("TAU");
    _params_handles.phi             =   param_find("PHI");
    

    _params_handles.water_depth_sp = param_find("WATER_DEPTH");
    _params_handles.water_depth_pgain = param_find("W_D_PGAIN");
    _params_handles.water_depth_dgain = param_find("W_D_DGAIN");
    

    _params_handles.r_sp_xx = param_find("R_SP_XX");
    _params_handles.r_sp_yx = param_find("R_SP_YX");
    _params_handles.r_sp_zx = param_find("R_SP_ZX");
    _params_handles.r_sp_xy = param_find("R_SP_XY");
    _params_handles.r_sp_yy = param_find("R_SP_YY");
    _params_handles.r_sp_zy = param_find("R_SP_ZY");
    _params_handles.r_sp_xz = param_find("R_SP_XZ");
    _params_handles.r_sp_yz = param_find("R_SP_YZ");
    _params_handles.r_sp_zz = param_find("R_SP_ZZ");

    
    _params_handles.pitch_gain = param_find("GC_GAIN_PITCH");
    _params_handles.yaw_gain = param_find("GC_GAIN_YAW");
    _params_handles.roll_gain = param_find("GC_GAIN_ROLL");


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
    param_get(_params_handles.roll_rate_p, &(_params.roll_rate_p));
    param_get(_params_handles.pitch_rate_p, &(_params.pitch_rate_p));
    param_get(_params_handles.yaw_rate_p, &(_params.yaw_rate_p));

    
    param_get(_params_handles.rho, &(_params.rho));
    param_get(_params_handles.tau, &(_params.tau));
    param_get(_params_handles.phi, &(_params.phi));
    

    param_get(_params_handles.water_depth_sp, &(_params.water_depth_sp));
    param_get(_params_handles.water_depth_pgain, &(_params.water_depth_pgain));
    param_get(_params_handles.water_depth_dgain, &(_params.water_depth_dgain));

    param_get(_params_handles.r_sp_xx, &(_params.r_sp_xx));
    param_get(_params_handles.r_sp_yx, &(_params.r_sp_yx));
    param_get(_params_handles.r_sp_zx, &(_params.r_sp_zx));
    param_get(_params_handles.r_sp_xy, &(_params.r_sp_xy));
    param_get(_params_handles.r_sp_yy, &(_params.r_sp_yy));
    param_get(_params_handles.r_sp_yz, &(_params.r_sp_yz));
    param_get(_params_handles.r_sp_xz, &(_params.r_sp_xz));
    param_get(_params_handles.r_sp_yz, &(_params.r_sp_yz));
    param_get(_params_handles.r_sp_zz, &(_params.r_sp_zz));


    
    param_get(_params_handles.pitch_gain, &(_params.pitch_gain));
    param_get(_params_handles.yaw_gain, &(_params.yaw_gain));
    param_get(_params_handles.roll_gain, &(_params.roll_gain));

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


void WaterDepthControl::control_state_poll()
{
    /* check if there is a new message */
    bool updated;
    orb_check(_ctrl_state_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
    }
}

float WaterDepthControl::get_xhat2(float x1, float time) {
    
    xhat1 = get_xhat1(x1, time);
    
    
   // xhat2 = xhat2_prev + (iterationtime / (0.1 * tau))* (-xhat2_prev - rho * sat(xhat1 - x1,1));
    xhat2 = xhat2_prev + (time / _params.tau) * (-xhat2_prev - _params.rho * sat(xhat1 - x1,_params.phi));
    
    xhat2_prev = xhat2;
        return xhat2; //xhat2 = geschätzte Geschwindigkeit
}
                                                            

float WaterDepthControl::get_xhat1(float x1, float time) {
    //xhat1 = xhat1_prev - (iterationtime / 0.1) * rho * sat(xhat1_prev - x1, 1);
        xhat1 = xhat1_prev - (time / 1) * _params.rho * sat(xhat1_prev - x1, _params.phi);
    xhat1_prev = xhat1; //xhat1 = geschätzte Tiefe
    return xhat1;
}

// sat functio
float WaterDepthControl::sat(float x, float gamma) {
    float y = math::max(math::min(1.0f, x / gamma), -1.0f);
    return y;
}




//define Pressure Depth Control
void WaterDepthControl::control_attitude()
{

            control_state_poll();

    /* pressure sensor control start */
            struct pressure_s press;
    
            /* get pressure value from sensor*/
            orb_copy(ORB_ID(pressure), _pressure_raw, &press);

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
            //_pressure_old = _pressure_new;
            _pressure_time_old = _pressure_time_new;

            /* calculate the water depth error */
            water_depth_err = water_depth - _params.water_depth_sp;
    
            /* use a pd-controller for water depth error*/
            water_depth_pd_err = water_depth_err - water_depth_smo * _params.water_depth_dgain;
                 /* multiply with a p-gain */
                 control_depth = _params.water_depth_pgain * water_depth_pd_err;
    
    if (control_depth > 1.0f){
        control_depth = 0.9;
    }
    
    if (control_depth < -1.0f){
        control_depth = -0.9;
    }
    
    /* pressure sensor control end */
    

    /* geometric control start */
    

            /* get parameters for geometric control */

                /* Orientation Matrix Setpoint */
                _R_sp(0, 0) = _params.r_sp_xx;       /**< _att_p_gain_xx */
                _R_sp(1, 0) = _params.r_sp_yx;       /**< _att_p_gain_yx */
                _R_sp(2, 0) = _params.r_sp_zx;       /**< _att_p_gain_zx */
                _R_sp(0, 1) = _params.r_sp_xy;       /**< _att_p_gain_xy */
                _R_sp(1, 1) = _params.r_sp_yy;       /**< _att_p_gain_yy */
                _R_sp(2, 1) = _params.r_sp_zy;       /**< _att_p_gain_zy */
                _R_sp(0, 2) = _params.r_sp_xz;       /**< _att_p_gain_xz */
                _R_sp(1, 2) = _params.r_sp_yz;       /**< _att_p_gain_yz */
                _R_sp(2, 2) = _params.r_sp_zz;       /**< _att_p_gain_zz */
    

            /* get current rotation matrix from control state quaternions */
            math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
            math::Matrix<3, 3> R = q_att.to_dcm();
    
            /* get current rates from sensors */
            //omega(0) = _v_att.rollspeed;
            omega(1) = _v_att.pitchspeed;
            omega(2) = _v_att.yawspeed;
    
            /* Compute matrix: attitude error */
            e_R =  (_R_sp.transposed() * R - R.transposed() * _R_sp) * 0.5;

            /* vee-map the error to get a vector instead of matrix e_R */
            //e_R_vec(0) = e_R(2,1);  // Roll
            e_R_vec(1) = e_R(0,2);  // Pitch
            e_R_vec(2) = e_R(1,0);  // Yaw

            /**< P-Control */
            //torques(0) = e_R_vec(0) * _params.roll_gain;     /**< Roll    */
            torques(1) = e_R_vec(1) * _params.pitch_gain;   /**< Pitch  */
            torques(2) = e_R_vec(2) * _params.yaw_gain;    /**< Yaw   */
    
            /**< PD-Control */
            //torques(0) = torques(0) - omega(0) * _params.roll_rate_p;    /**< Roll    */
            torques(1) = torques(1) - omega(1) * _params.pitch_rate_p;  /**< Pitch  */
            torques(2) = torques(2) - omega(2) * _params.yaw_rate_p;   /**< Yaw   */
    
    /* geometric control end */
    
    /* Values for engine */
    
            //_att_control(0) = torques(0);     /**< Roll   */
            _att_control(1) = torques(1);       /**< Pitch  */
            _att_control(2) = torques(2);       /**< Yaw    */
            _thrust_sp = control_depth;         /**< Thrust */
}

//define Furuta Pendulum
void WaterDepthControl::furuta_pendulum()
{
    
    //get ADC value and print it for debugging
    raw_adc_data_poll();

    adc_input = _raw_adc.channel_value[7];

    if (adc_input > 3.2f){
        adc_input = 3.2f;
    }

    angle_input = (0.625f * adc_input) - 1.0f;

    angle_error = alpha_zero - angle_input;

    if ((angle_error > 0.5f) || (angle_error < -0.5f)){
        angle_error = 0.0f;
    }

    angle_p_control = angle_error * _params.roll_gain;
    
    if (angle_p_control > 1.0f){
        angle_p_control = 1.0f;
    }
    
    if (angle_p_control < -1.0f){
        angle_p_control = -1.0f;
    }

    _att_control(0) = angle_p_control;     /**< Roll   */

}



//main task
void WaterDepthControl::task_main()
{
    //do subscriptions
    _v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _pressure_raw = orb_subscribe(ORB_ID(pressure));
    _params_sub = orb_subscribe(ORB_ID(parameter_update));

    _adc_sub_fd = orb_subscribe(ORB_ID(adc_report));

    _ctrl_state_sub = orb_subscribe(ORB_ID(control_state));


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

                /**< Water depth and geometric control for furuta pendulum */
                control_attitude();

                /**< Controller for furuta pendulum */
                furuta_pendulum();
            
            /* end controller */
            
 
            /* Show Parameters of SMO by using a Mavlink Topic and QGC */
            //_pos.x = water_depth;             // Water Depth
            //_pos.y = _params.water_depth_sp   // Water Depth Setpoint
            //_pos.z = water_depth_smo;         // Water Depth Sliding Mode Observer
            //_pos.vx = water_depth_err;        // Water Depth Error (P-Controller)
            //_pos.vy = water_depth_pd_err;     // Water Depth Error (PD-Controller)
            //_pos.vz = control_depth;          // Signal for Engine
            
            /* Show Parameters of Geometric Control by using a Mavlink Topic and QGC */
            //_pos.x = water_depth;
            //_pos.y = _params.water_depth_sp;
            //_pos.z = e_R_vec(1);    // Pitch
            //_pos.vx = e_R_vec(2);   // Yaw
            //_pos.vy = torques(1);   // Pitch
            //_pos.vz = torques(2);   // Yaw

            /* Show Parameters of Furuta Pendulum by using a Mavlink Topic and QGC */
            _pos.x = adc_input;
            _pos.y = angle_input;
            _pos.z = angle_error;
            _pos.vx = angle_p_control;
            //_pos.vy =
            //_pos.vz =

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



int water_depth_control_main(int argc, char *argv[])
{
    if (argc < 2) {
            warnx("usage: water_depth_control {start|stop|status|mystatus}");
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
    
        if (!strcmp(argv[1], "mystatus")) {
            struct pressure_s press_status;
            int _pressure_raw_status;
            
            _pressure_raw_status = orb_subscribe(ORB_ID(pressure));
            
            if (water_depth_control::g_control) {
                int counter_status = 0;
                
                while(counter_status < 12){
                    orb_copy(ORB_ID(pressure), _pressure_raw_status, &press_status);                    
                    PX4_INFO("Pressure:\t%8.4f",
                                        (double)press_status.pressure_mbar);
                    counter_status++;
                    usleep(500000);
                }
                return 0;
            } else {
                warnx("not running");
                return 1;
            }
        }
    
        warnx("unrecognized command");
        return 1;
}
