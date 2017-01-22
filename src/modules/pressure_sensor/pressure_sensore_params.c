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
#include <systemlib/param/param.h>

/**
 * @file uw_att_control_params.c
 *
 * Parameters for Underwater Attitude Control
 *
 */



/**
 * Roll P Gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 * increase to make controler more aggressive
 *
 */

PARAM_DEFINE_FLOAT(UW_ROLL_P, 1.0f);

/**
 * Roll D Gain
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 * increase to dampen controler
 *
 */

PARAM_DEFINE_FLOAT(UW_ROLL_RATE_P, 5.0f);

/**
 * Pitch P Gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 * increase to make controler more aggressive
 *
 */

PARAM_DEFINE_FLOAT(UW_PITCH_P, 1.0f);

/**
 * Pitch D Gain
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 * increase to dampen controler
 *
 */

PARAM_DEFINE_FLOAT(UW_PITCH_RATE_P, 5.0f);

/**
 * Yaw P Gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 * increase to make controler more aggressive
 *
 */

PARAM_DEFINE_FLOAT(UW_YAW_P, 1.0f);

/**
 * Yaw D Gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 * increase to dampen controler
 *
 */

PARAM_DEFINE_FLOAT(UW_YAW_RATE_P, 5.0f);

/**
 * Control Mode
 *
 * 0 - use angle setpoints
 * 1 - use Rotation Matrix setpoint
 * 2 - use Quaternion
 *
 */

PARAM_DEFINE_INT32(UW_CONTROL_MODE, 0);

