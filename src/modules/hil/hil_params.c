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
 * @file hil_params.c
 * Parameters for hil.
 *
 */

/**
 * Ref latitude * 1E7 for HIL 
 *
 * Ref latitude.
 *
 * @min -900000000
 * @max 900000000
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(HIL_REF_LAT, 418782246);

/**
 * Ref longitude * 1E7 for HIL 
 *
 * Ref longitude.
 *
 * @min -1800000000
 * @max 1800000000
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(HIL_REF_LON, 1234143249);

/**
 * Ref altitude * 1E3 for HIL 
 *
 * Ref altitude.
 *
 * @min -400000
 * @max 10000000
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(HIL_REF_ALT, 50000);

/**
 * HIL enabled
 *
 * 0: Disabled   1: Enabled
 *
 * @min 0
 * @max 10
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(HIL_ENABLED, 0);

/**
 * HIL enabled flag
 *
 * 0: Disabled   1: Enabled
 *
 * @min 0
 * @max 10
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(HIL_EN_FLAG, 0);
