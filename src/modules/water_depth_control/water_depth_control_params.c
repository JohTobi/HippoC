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
 * @file water_depth_control_params.c
 *
 * Parameters for Water Depth Control
 *
 */

/**
 * <Water Depth>
 *
 * @unit meter
 */
PARAM_DEFINE_FLOAT(WATER_DEPTH, 1.0f);

/**
 * Roll P Gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 * increase to make controler more aggressive
 *
 */

PARAM_DEFINE_FLOAT(W_D_PGAIN, -1.0f);      //0.0190
PARAM_DEFINE_FLOAT(W_D_DGAIN, 0.1f);

PARAM_DEFINE_FLOAT(RHO, 1.0f); //10.0       helix: 1.0
PARAM_DEFINE_FLOAT(TAU, 1.0f);  //1.5       helix: -0.9
PARAM_DEFINE_FLOAT(PHI, 1.0f);  // 0.4      helix: 1.0

PARAM_DEFINE_FLOAT(UWC_YAW_RATE_SP, 0.2f);  //helix: 0.55
PARAM_DEFINE_FLOAT(UWC_PITCH_SP, 0.1f);     //helix: -0.9
PARAM_DEFINE_FLOAT(UWC_SPEED_SP, 0.2f);     //helix: 0.5

PARAM_DEFINE_FLOAT(R_SP_XX, 0.0f); //helix: 5.0
PARAM_DEFINE_FLOAT(R_SP_YX, 0.0f);
PARAM_DEFINE_FLOAT(R_SP_ZX, 1.0f);
PARAM_DEFINE_FLOAT(R_SP_XY, 0.0f);
PARAM_DEFINE_FLOAT(R_SP_YY, 1.0f);
PARAM_DEFINE_FLOAT(R_SP_ZY, 0.0f);
PARAM_DEFINE_FLOAT(R_SP_XZ, -1.0f);
PARAM_DEFINE_FLOAT(R_SP_YZ, 0.0f);
PARAM_DEFINE_FLOAT(R_SP_ZZ, 0.0f);

PARAM_DEFINE_FLOAT(GC_GAIN_PITCH, 2.0f);
PARAM_DEFINE_FLOAT(GC_GAIN_YAW, 2.0f);
PARAM_DEFINE_FLOAT(GC_GAIN_ROLL, 25.0f);

PARAM_DEFINE_FLOAT(UW_ROLL_RATE_P, 3.0f);
PARAM_DEFINE_FLOAT(UW_PITCH_RATE_P, 1.5f);
PARAM_DEFINE_FLOAT(UW_YAW_RATE_P, 1.5f);

