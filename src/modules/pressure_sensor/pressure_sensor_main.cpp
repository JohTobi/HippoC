
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
 * @file uw_att_control_main.cpp
 *
 * HippoCampus Underwater Attitude Controller. Feed Forward RC Input except roll. Balance roll.
 *
 * Based on rover steering control example by Lorenz Meier <lorenz@px4.io>
 *
 * @author Max Kirchhoff
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


/* Prototypes */

/**
 * Underwater attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int uw_att_control_main(int argc, char *argv[]);

class UnderwaterAttitudeControl {
public:
    /**
     * Constructor
     */
    UnderwaterAttitudeControl();

    /**
     * Destructor, also kills the main task
     */
    ~UnderwaterAttitudeControl();

    /**
     * Start the underwater attitude control task.
     *
     * @return		OK on success.
     */
    int		start();

private:

    bool	_task_should_exit;		/**< if true, task_main() should exit */
    int		_control_task;			/**< task handle */

    int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
    int		_m_ctrl_sp_sub;         /**< manual control setpoint subscription */
    int     _v_att_sp_sub;          /**< vehicle attitude setpoint subscription */
    int		_params_sub;			/**< parameter updates subscription */
    int     _v_att_sub;             /**< vehicle attitude subscription */

    orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */


    struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
    struct manual_control_setpoint_s	_m_ctrl_sp;         /**< manual control setpoint */
    struct vehicle_attitude_setpoint_s  _v_att_sp;          /**< vehicle attitude setpoint */
    struct actuator_controls_s			_actuators;			/**< actuator controls */
    struct vehicle_attitude_s           _v_att;             /**< vehicle attitude */


    perf_counter_t	_loop_perf;			/**< loop performance counter */
    perf_counter_t	_controller_latency_perf;

    math::Vector<3>		_angles_sp;	/**< angular rates on previous step */
    math::Vector<3>		_rates_sp_prev; /**< previous rates setpoint */
    math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
    math::Vector<3>		_rates_int;		/**< angular rates integral error */
    float				_thrust_sp;		/**< thrust setpoint */
    math::Vector<3>		_att_control;	/**< attitude control vector */

    math::Matrix<3, 3>  _I;				/**< identity matrix */

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
    int			parameters_update();

    /**
     * Check for parameter update and handle it.
     */
    void		parameter_update_poll();

    /**
     * Check for rates setpoint updates.
     */
    void		vehicle_rates_setpoint_poll();

    /**
     * Check for manual setpoint updates.
     */
    void		manual_control_setpoint_poll();

    /**
     * Check for attitude setpoint updates.
     */
    void        vehicle_attitude_setpoint_poll();


    /**
     * Attitude controller.
     */
    void        manual_control();

    void        assisted_control();

    void		control_attitude();
    void		control_roll_ignored(float pitch_err, float yaw_err);
    void        spin();

    void        r_body_control();       //use only if necessary

    void        quaternion_control();   //use only if necessary

    void        rates_control();        //for use with joystick or direct mapping

    /**
     * Check for vehicle motor limits status.
     */
    void		vehicle_motor_limits_poll();

    /**
     * Shim for calling task_main from task_create.
     */
    static void	task_main_trampoline(int argc, char *argv[]);

    /**
     * Main attitude control task.
     */
    void		task_main();
};

namespace uw_att_control
{

UnderwaterAttitudeControl	*g_control;
}



UnderwaterAttitudeControl::UnderwaterAttitudeControl() :

    _task_should_exit(false),
    _control_task(-1),

    /* subscriptions */
    _v_rates_sp_sub(-1),
    _m_ctrl_sp_sub(-1),
    _params_sub(-1),
    _v_att_sub(-1),

    /* publications */

    _actuators_0_pub(nullptr),



    /* performance counters */
    _loop_perf(perf_alloc(PC_ELAPSED, "uw_att_control")),
    _controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))

{

    memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
    memset(&_m_ctrl_sp, 0, sizeof(_m_ctrl_sp));
    memset(&_v_att_sp, 0, sizeof(_v_att_sp));
    memset(&_actuators, 0, sizeof(_actuators));
    memset(&_v_att, 0, sizeof(_v_att));




    _angles_sp.zero();
    _rates_sp.zero();
    _rates_sp_prev.zero();
    _rates_int.zero();
    _thrust_sp = 0.0f;
    _att_control.zero();

    _I.identity();

    _params_handles.roll_p			= 	param_find("UW_ROLL_P");
    _params_handles.roll_rate_p		= 	param_find("UW_ROLL_RATE_P");
    
    _params_handles.pitch_p			= 	param_find("UW_PITCH_P");
    _params_handles.pitch_rate_p		= 	param_find("UW_PITCH_RATE_P");
    
    _params_handles.yaw_p			= 	param_find("UW_YAW_P");
    _params_handles.yaw_rate_p		= 	param_find("UW_YAW_RATE_P");

    _params_handles.control_mode    =   param_find("UW_CONTROL_MODE");



    /* fetch initial parameter values */
    parameters_update();

}

UnderwaterAttitudeControl::~UnderwaterAttitudeControl()
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


    uw_att_control::g_control = nullptr;
}



int UnderwaterAttitudeControl::parameters_update()
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

void UnderwaterAttitudeControl::parameter_update_poll()
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

void UnderwaterAttitudeControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new rates setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void UnderwaterAttitudeControl::manual_control_setpoint_poll()
{
    /* check if there is a new rates setpoint */
    bool updated;
    orb_check(_m_ctrl_sp_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(manual_control_setpoint), _m_ctrl_sp_sub, &_m_ctrl_sp);
    }
}

void UnderwaterAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new rates setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void UnderwaterAttitudeControl::manual_control(){
    manual_control_setpoint_poll();

    _att_control(0) = _m_ctrl_sp.y; // roll
    _att_control(1) = _m_ctrl_sp.x; // pitch
    _att_control(2) = _m_ctrl_sp.r; // yaw
    _thrust_sp      = _m_ctrl_sp.z; // thrust


}

void UnderwaterAttitudeControl::assisted_control(){
    manual_control_setpoint_poll();

    _att_control(0) = -_v_att.roll*_params.roll_p -_v_att.rollspeed*_params.roll_rate_p;
    _att_control(1) = _m_ctrl_sp.x; // pitch
    _att_control(2) = _m_ctrl_sp.r; // yaw
    _thrust_sp      = _m_ctrl_sp.z; // thrust

}


 void UnderwaterAttitudeControl::control_attitude()
{
    vehicle_rates_setpoint_poll();
    vehicle_attitude_setpoint_poll();
    
    // desired
    _angles_sp(0) = _v_att_sp.roll_body;
    _angles_sp(1)= _v_att_sp.pitch_body;
    _angles_sp(2)= _v_att_sp.yaw_body;

    //security check
    if (fabs((double)_angles_sp(0))>3.1416 || fabs((double)_angles_sp(1))>3.1416/2 || fabs((double)_angles_sp(2))>3.1416){
        _att_control(0) = 0.0f;
        _att_control(1) = 0.0f;
        _att_control(2) = 0.0f;
        _thrust_sp=0.0f;
        PX4_WARN("wrong input! resetting and switching to manual control");
        int control_mode = 0;
        param_set(param_find("UW_CONTROL_MODE"), &control_mode);
        return;
    }

    // rates setpoint
    math::Vector<3> rates_sp;
    rates_sp(0) = _v_rates_sp.roll;
    rates_sp(1) = _v_rates_sp.pitch;
    rates_sp(2) = _v_rates_sp.yaw;

    bool rates_sp_given=false;
            for (int i=0; i<3; i++){
        if (fabs(double(rates_sp(i))>0.0)) rates_sp_given = true;
    }


/*-----------------------------------------------control attitude-----------------------------------------------*/


    // calculate errors, go shortest way for roll and yaw
    float roll_err = (_angles_sp(0) - _v_att.roll);
    if (roll_err < -M_PI_F){
        roll_err = roll_err + 2.0f*M_PI_F;
    } else if (roll_err > M_PI_F){roll_err = roll_err - 2.0f*M_PI_F; }

    float pitch_err = _angles_sp(1) - _v_att.pitch;

    float yaw_err = _angles_sp(2) - _v_att.yaw ;
    if (yaw_err < -M_PI_F){
        yaw_err = yaw_err + 2.0f*M_PI_F;
    } else if (yaw_err > M_PI_F){yaw_err = yaw_err - 2.0f*M_PI_F; }



    //sinus and cosinus for roll compensation
    float sr=sinf(_v_att.roll);
    float cr=cosf(_v_att.roll);

    //p-control
    float control_pitch = (pitch_err * _params.pitch_p)*cr+((yaw_err) * _params.yaw_p)*sr;
    float control_yaw = (yaw_err * _params.yaw_p)*cr - ((pitch_err) * _params.yaw_p)*sr;

    //d-control
    control_pitch=control_pitch - _v_att.pitchspeed * _params.pitch_rate_p;
    control_yaw=control_yaw - _v_att.yawspeed * _params.yaw_rate_p;

    // when offset is big and no rate setpoints are given, control yaw and pitch before roll
    if(((fabs((double)yaw_err) > 0.2|| fabs((double)pitch_err) > 0.2) && !rates_sp_given))
            _att_control(0) = 0.0f;
    else    _att_control(0) = roll_err * _params.roll_p - _v_att.rollspeed * _params.roll_rate_p;

    _att_control(1) = control_pitch;
    _att_control(2) = control_yaw;
    _thrust_sp=_v_att_sp.thrust;


    // ignore roll when vertical (singularity)
    if (_angles_sp(1) > 1.5f || _angles_sp(1) < -1.5f ){
        _att_control(0)=0.0f;
    }

    //major roll dominance beacuse yaw and pitch are nearly correct
    _att_control(1)=double(_att_control(1))/ (1.0+fabs(double(_att_control(0)*300.0f)));
    _att_control(2)=double(_att_control(2))/ (1.0+fabs(double(_att_control(0)*300.0f)));

    // use rate setpoints instead whenever not zero
    _att_control(0) = (int(_v_rates_sp.roll*100) == 0) ? _att_control(0) : _v_rates_sp.roll;
    _att_control(1) = (int(_v_rates_sp.pitch*100) == 0) ? _att_control(1) : _v_rates_sp.pitch;
    _att_control(2) = (int(_v_rates_sp.yaw*100) == 0) ? _att_control(2) : _v_rates_sp.yaw;
    _thrust_sp=_v_att_sp.thrust;

}
 void UnderwaterAttitudeControl::control_roll_ignored(float pitch_err, float yaw_err)
{



    //sinus and cosinus for roll compensation
    float sr=sinf(_v_att.roll);
    float cr=cosf(_v_att.roll);

    //p-control
    float control_pitch = (pitch_err * _params.pitch_p)*cr+((yaw_err) * _params.yaw_p)*sr;
    float control_yaw = (yaw_err * _params.yaw_p)*cr - ((pitch_err) * _params.yaw_p)*sr;

    //d-control
    control_pitch=control_pitch - _v_att.pitchspeed * _params.pitch_rate_p;
    control_yaw=control_yaw - _v_att.yawspeed * _params.yaw_rate_p;

    _att_control(0) = 0.0f;
    _att_control(1) = control_pitch;
    _att_control(2) = control_yaw;
    _thrust_sp=_v_att_sp.thrust;

}


 // usage: set yaw-,rollrate and thrust in vehicle_rates_setpoint, set pitch angle in vehicle_attitude_setpoint
 void UnderwaterAttitudeControl::spin()
{
     vehicle_rates_setpoint_poll();
     vehicle_attitude_setpoint_poll();


    //sinus and cosinus for roll compensation
    float sr=sinf(_v_att.roll);
    float cr=cosf(_v_att.roll);

    //p-control
    float control_pitch = ((_v_att_sp.pitch_body-_v_att.pitch) * _params.pitch_p)*cr+(_v_rates_sp.yaw * _params.yaw_p)*sr;
    float control_yaw = (_v_rates_sp.yaw * _params.yaw_p)*cr - (((_v_att_sp.pitch_body-_v_att.pitch) ) * _params.yaw_p)*sr;

    //d-control
    //control_pitch=control_pitch - _v_att.pitchspeed * _params.pitch_rate_p*cr;
    //control_yaw=control_yaw - _v_att.yawspeed * _params.yaw_rate_p*sr;

    _att_control(0) = _v_rates_sp.roll;
    _att_control(1) = control_pitch;
    _att_control(2) = control_yaw;
    _thrust_sp=_v_rates_sp.thrust;


}



 void UnderwaterAttitudeControl::r_body_control(){
    /** RotationMatrix/Quaternion based controler **/
    //poll data
    vehicle_rates_setpoint_poll();
    vehicle_attitude_setpoint_poll();

    // get current rotation matrix from control state quaternions
    math::Quaternion q(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
    math::Matrix<3,3> R = q.to_dcm();

    // construct attitude setpoint rotation matrix
    math::Matrix<3, 3> R_sp;
    R_sp.set(_v_att_sp.R_body);

    // get current rates
    math::Vector<3> rates;
    rates(0) = _v_att.rollspeed;
    rates(1) = _v_att.pitchspeed;
    rates(2) = _v_att.yawspeed;

    // get control gains
    math::Vector<3> p_gain(_params.roll_p, _params.pitch_p, _params.yaw_p);
    math::Vector<3> rates_p_gain(_params.roll_rate_p, _params.pitch_rate_p, _params.yaw_rate_p);


    /*-------------control attitude-------------*/

    math::Matrix<3,3> R_error = R.transposed()*R_sp;

    _rates_sp = R_error.to_euler();

    _att_control=_rates_sp.emult(p_gain)-rates.emult(rates_p_gain);



    // roll dominance. Choose between yaw/pitch dominance depending on offset
    if (fabs(_att_control(2)) < fabs(_att_control(1)))
        _att_control(2)=_att_control(2)/ ((1+abs((int)(_att_control(0)*10.0f)))*(1+abs((int)(_att_control(1)*300.0f))));
    else
        _att_control(1)=_att_control(1)/ ((1+abs((int)(_att_control(0)*10.0f)))*(1+abs((int)(_att_control(2)*300.0f))));

    // use rate setpoints instead whenever not zero

    _att_control(0) = (int(_v_rates_sp.roll*100) == 0) ? _att_control(0) : _v_rates_sp.roll;
    _att_control(1) = (int(_v_rates_sp.pitch*100) == 0) ? _att_control(1) : _v_rates_sp.pitch;
    _att_control(2) = (int(_v_rates_sp.yaw*100) == 0) ? _att_control(2) : _v_rates_sp.yaw;

    _thrust_sp=_v_att_sp.thrust;

    //PX4_WARN("Ist:\t%8.4f\t%8.4f\t%8.4f", (double) R.to_euler()(0), (double) R.to_euler()(1), (double) R.to_euler()(2));
    //PX4_WARN("Soll:\t%8.4f\t%8.4f\t%8.4f", (double) R_sp.to_euler()(0), (double) R_sp.to_euler()(1), (double) R_sp.to_euler()(2));
    //PX4_WARN("Error:\t%8.4f\t%8.4f\t%8.4f", (double) R_error.to_euler()(0), (double) R_error.to_euler()(1), (double) R_error.to_euler()(2));

}

  void UnderwaterAttitudeControl::quaternion_control(){

    /** quaternion based controler **/

    vehicle_rates_setpoint_poll();
    vehicle_attitude_setpoint_poll();

    // desired
    math::Quaternion q_ref(_v_att_sp.q_d[0], _v_att_sp.q_d[1], _v_att_sp.q_d[2], _v_att_sp.q_d[3]);
    // measured
    math::Quaternion q_m(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);



    // rates
    math::Vector<3> rates;
    rates(0) = _v_att.rollspeed;
    rates(1) = _v_att.pitchspeed;
    rates(2) = _v_att.yawspeed;

    // control gains
    math::Vector<3> p_gain(_params.roll_p, _params.pitch_p, _params.yaw_p);
    math::Vector<3> rates_p_gain(_params.roll_rate_p, _params.pitch_rate_p, _params.yaw_rate_p);
    /*-------------control attitude-------------*/
    // error
    math::Matrix<3,3> R_err(q_m.to_dcm().transposed()*q_ref.to_dcm());
    // axis error
    _rates_sp = R_err.to_euler();

    // aplly pd-gain
    _att_control = p_gain.emult(_rates_sp) - rates_p_gain.emult(rates);//}




    // roll dominance. Choose between yaw/pitch dominance depending on offset
    if (fabs(_att_control(2)) < fabs(_att_control(1)))
        _att_control(2)=_att_control(2)/ ((1+abs((int)(_att_control(0)*10.0f)))*(1+abs((int)(_att_control(1)*300.0f))));
    else
        _att_control(1)=_att_control(1)/ ((1+abs((int)(_att_control(0)*10.0f)))*(1+abs((int)(_att_control(2)*300.0f))));



   // use rate setpoints instead whenever not zero

  _att_control(0) = (int(_v_rates_sp.roll*100) == 0) ? _att_control(0) : _v_rates_sp.roll;
  _att_control(1) = (int(_v_rates_sp.pitch*100) == 0) ? _att_control(1) : _v_rates_sp.pitch;
  _att_control(2) = (int(_v_rates_sp.yaw*100) == 0) ? _att_control(2) : _v_rates_sp.yaw;
  _thrust_sp=_v_att_sp.thrust;

}

void UnderwaterAttitudeControl::rates_control(){

    vehicle_rates_setpoint_poll();
    vehicle_attitude_setpoint_poll();

    _att_control(0) =  _v_rates_sp.roll;
    _att_control(1) =  _v_rates_sp.pitch;
    _att_control(2) =  _v_rates_sp.yaw;
    _thrust_sp      =  _v_rates_sp.thrust;


}

void UnderwaterAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
    uw_att_control::g_control->task_main();
}


void UnderwaterAttitudeControl::task_main()
{

    /*
     * do subscriptions
     */
    _v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
    _m_ctrl_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    _v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    _params_sub = orb_subscribe(ORB_ID(parameter_update));


    /* initialize parameters cache */
    parameters_update();

    /* advertise actuator controls */
    _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);

    /* wakeup source: vehicle attitude */
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

            /* start controler */
            switch (_params.control_mode){
            case 0: manual_control();
                break;
            case 1: assisted_control();
                break;
            case 2: control_attitude();
                break;
            case 3: rates_control();
                break;
            case 4: spin();
                break;
            }






//roll, pitch dominance over thrust
//_thrust_sp=_v_att_sp.thrust/ ((1+abs((int)(_att_control(0)*10.0f)))*(1+abs((int)(_att_control(1)*10.0f))));

//PX4_WARN("roll%8.4f\tpitch%8.4f\tyaw%8.4f\tthrust%8.4f",(double)_att_control(0),(double)_att_control(1),(double)_att_control(2),(double)_thrust_sp);

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



int UnderwaterAttitudeControl::start()
{
    ASSERT(_control_task == -1);

    /* start the task */
    _control_task = px4_task_spawn_cmd("uw_att_control",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_MAX - 5,
                       1500,
                       (px4_main_t)&UnderwaterAttitudeControl::task_main_trampoline,
                       nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}


int uw_att_control_main(int argc, char *argv[])
{
    if (argc < 2) {
        warnx("usage: uw_att_control {start|stop|status}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (uw_att_control::g_control != nullptr) {
            warnx("already running");
            return 1;
        }

        uw_att_control::g_control = new UnderwaterAttitudeControl;

        if (uw_att_control::g_control == nullptr) {
            warnx("alloc failed");
            return 1;
        }

        if (OK != uw_att_control::g_control->start()) {
            delete uw_att_control::g_control;
            uw_att_control::g_control = nullptr;
            warnx("start failed");
            return 1;
        }

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (uw_att_control::g_control == nullptr) {
            warnx("not running");
            return 1;
        }

        delete uw_att_control::g_control;
        uw_att_control::g_control = nullptr;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (uw_att_control::g_control) {
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
