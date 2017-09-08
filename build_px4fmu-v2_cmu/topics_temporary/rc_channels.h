/****************************************************************************
 *
 *   Copyright (C) 2013-2015 PX4 Development Team. All rights reserved.
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

/* Auto-generated by genmsg_cpp from file /home/mosam/Desktop/dummy_repos/renewed_attempt/Firmware/msg/rc_channels.msg */


#pragma once

#include <stdint.h>
#include <uORB/uORB.h>


#ifndef __cplusplus
#define RC_CHANNELS_FUNCTION_MAX 21
#define RC_CHANNELS_FUNCTION_THROTTLE 0
#define RC_CHANNELS_FUNCTION_ROLL 1
#define RC_CHANNELS_FUNCTION_PITCH 2
#define RC_CHANNELS_FUNCTION_YAW 3
#define RC_CHANNELS_FUNCTION_MODE 4
#define RC_CHANNELS_FUNCTION_RETURN 5
#define RC_CHANNELS_FUNCTION_POSCTL 6
#define RC_CHANNELS_FUNCTION_LOITER 7
#define RC_CHANNELS_FUNCTION_OFFBOARD 8
#define RC_CHANNELS_FUNCTION_ACRO 9
#define RC_CHANNELS_FUNCTION_FLAPS 10
#define RC_CHANNELS_FUNCTION_AUX_1 11
#define RC_CHANNELS_FUNCTION_AUX_2 12
#define RC_CHANNELS_FUNCTION_AUX_3 13
#define RC_CHANNELS_FUNCTION_AUX_4 14
#define RC_CHANNELS_FUNCTION_AUX_5 15
#define RC_CHANNELS_FUNCTION_PARAM_1 16
#define RC_CHANNELS_FUNCTION_PARAM_2 17
#define RC_CHANNELS_FUNCTION_PARAM_3_5 18
#define RC_CHANNELS_FUNCTION_RATTITUDE 19
#define RC_CHANNELS_FUNCTION_KILLSWITCH 20

#endif

/**
 * @addtogroup topics
 * @{
 */


#ifdef __cplusplus
struct __EXPORT rc_channels_s {
#else
struct rc_channels_s {
#endif
	uint64_t timestamp;
	uint64_t timestamp_last_valid;
	float channels[18];
	uint8_t channel_count;
	int8_t function[21];
	uint8_t rssi;
	bool signal_lost;
	uint32_t frame_drop_count;
#ifdef __cplusplus
	static const int32_t RC_CHANNELS_FUNCTION_MAX = 21;
	static const uint8_t RC_CHANNELS_FUNCTION_THROTTLE = 0;
	static const uint8_t RC_CHANNELS_FUNCTION_ROLL = 1;
	static const uint8_t RC_CHANNELS_FUNCTION_PITCH = 2;
	static const uint8_t RC_CHANNELS_FUNCTION_YAW = 3;
	static const uint8_t RC_CHANNELS_FUNCTION_MODE = 4;
	static const uint8_t RC_CHANNELS_FUNCTION_RETURN = 5;
	static const uint8_t RC_CHANNELS_FUNCTION_POSCTL = 6;
	static const uint8_t RC_CHANNELS_FUNCTION_LOITER = 7;
	static const uint8_t RC_CHANNELS_FUNCTION_OFFBOARD = 8;
	static const uint8_t RC_CHANNELS_FUNCTION_ACRO = 9;
	static const uint8_t RC_CHANNELS_FUNCTION_FLAPS = 10;
	static const uint8_t RC_CHANNELS_FUNCTION_AUX_1 = 11;
	static const uint8_t RC_CHANNELS_FUNCTION_AUX_2 = 12;
	static const uint8_t RC_CHANNELS_FUNCTION_AUX_3 = 13;
	static const uint8_t RC_CHANNELS_FUNCTION_AUX_4 = 14;
	static const uint8_t RC_CHANNELS_FUNCTION_AUX_5 = 15;
	static const uint8_t RC_CHANNELS_FUNCTION_PARAM_1 = 16;
	static const uint8_t RC_CHANNELS_FUNCTION_PARAM_2 = 17;
	static const uint8_t RC_CHANNELS_FUNCTION_PARAM_3_5 = 18;
	static const uint8_t RC_CHANNELS_FUNCTION_RATTITUDE = 19;
	static const uint8_t RC_CHANNELS_FUNCTION_KILLSWITCH = 20;

#endif
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(rc_channels);
