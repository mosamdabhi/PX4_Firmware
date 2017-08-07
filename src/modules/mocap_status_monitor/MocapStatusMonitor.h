#ifndef MOCAP_STATUS_MONITOR_H
#define MOCAP_STATUS_MONITOR_H
/****************************************************************************
 *
 * Copyright (C) 2016 Nathan Michael
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above
 * copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <cmath>
#include <cstdio>
#include <px4_posix.h>
#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>

#include "ParameterUtils.h"

class MocapStatusMonitor
{
public:
  MocapStatusMonitor();
  ~MocapStatusMonitor();

  bool initialize();
  void finalize();
  void update();

private:
  // Start-up/cleanup calls
  bool loadParameters();
  bool registerCallbacks();
  void closeSubscriptions();

  bool openDevices();
  void closeDevices();

  void batteryStatusMessageCallback(const battery_status_s& bs);
  void printBatteryStatus(const battery_status_s& bs);

  int playLowBatteryTune();
  int playCriticalBatteryTune();
  void stopBatteryTune();

  void enableWarnLEDS();
  void enableCriticalLEDS();
  void disableLEDS();

  px4_pollfd_struct_t fds;

  int battery_status_sub;

  int buzzer;
  int leds;

  float battery_ignore;
  float battery_low;
  float battery_critical;
  int board_type;
};
#endif
