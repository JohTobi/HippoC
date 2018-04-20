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
PARAM_DEFINE_FLOAT(DEPTH, 0.0f);

/**
 * Roll P Gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 * increase to make controler more aggressive
 *
 */

PARAM_DEFINE_FLOAT(ROLL_SP, 0.0f);
PARAM_DEFINE_FLOAT(ROLL_P_SP, 1.5f);
PARAM_DEFINE_FLOAT(ROLL_D_SP, 0.1f);

PARAM_DEFINE_FLOAT(SMO_RHO, 10.0f);
PARAM_DEFINE_FLOAT(SMO_TAU, 1.5f);
PARAM_DEFINE_FLOAT(SMO_PHI, 0.4f);

PARAM_DEFINE_FLOAT(DEPTH_P_SP, 1.9f);
PARAM_DEFINE_FLOAT(DEPTH_D_SP, 0.1f);


