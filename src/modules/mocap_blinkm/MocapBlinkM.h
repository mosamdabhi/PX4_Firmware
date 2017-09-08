#ifndef MOCAP_BLINKM_H
#define MOCAP_BLINKM_H
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
#include <uORB/topics/blinkm_control.h>

#include "ParameterUtils.h"

class MocapBlinkM
{
public:
  MocapBlinkM();
  ~MocapBlinkM();

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

  void blinkMControlMessageCallback(const blinkm_control_s& bc);
  void updateStatusLED(unsigned int control);

  px4_pollfd_struct_t fds;

  int blinkm_control_sub;
  int bl1, bl2;
  unsigned int default_intensity;
};
#endif
