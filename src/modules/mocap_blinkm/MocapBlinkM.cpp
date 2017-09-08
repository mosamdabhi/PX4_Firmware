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
#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_blinkm.h>

#include "MocapBlinkM.h"

namespace pu = parameter_utils;

MocapBlinkM::MocapBlinkM() { }
MocapBlinkM::~MocapBlinkM() { }

bool MocapBlinkM::initialize()
{
  if (!loadParameters())
  {
    puts("[MBM] failed to load parameters");
    return false;
  }

  if (!registerCallbacks())
  {
    puts("[MBM] failed to register callbacks");
    return false;
  }

  if (!openDevices())
  {
    puts("[MBM] failed to open devices");
    return false;
  }

  return true;
}

void MocapBlinkM::finalize()
{
  closeSubscriptions();
  closeDevices();
}

void MocapBlinkM::update()
{
  static unsigned int error_counter = 0;
  int pret = px4_poll(&fds, 1, 1000);
  if (pret < 0)
  {
    if (error_counter++ > 50)
    {
      printf("[MBM] ERROR return value from poll(): %d", pret);
      error_counter = 0;
    }
  }
  else if (pret > 0)
  {
    if (fds.revents & POLLIN)
    {
      struct blinkm_control_s bc;
      orb_copy(ORB_ID(blinkm_control), blinkm_control_sub, &bc);
      blinkMControlMessageCallback(bc);
    }
  }
}

bool MocapBlinkM::openDevices()
{
  bl1 = px4_open(BLINKM0_DEVICE_PATH, 0);
  bl2 = px4_open(BLINKM1_DEVICE_PATH, 0);

  if ((bl1 < 0) || (bl2 < 0))
  {
    puts("[MBM] BlinkM: open fail");
    return false;
  }

  if (ioctl(bl1, BLINKM_SET_MODE, 1))
  {
    puts("[MBM] BlinkM: ioctl fail");
    return false;
  }
  ioctl(bl1, BLINKM_SET_MODE, 0);

  if (ioctl(bl2, BLINKM_SET_MODE, 1))
  {
    puts("[MBM] BlinkM: ioctl fail");
    return false;
  }
  ioctl(bl2, BLINKM_SET_MODE, 0);

  return true;
}

void MocapBlinkM::blinkMControlMessageCallback(const blinkm_control_s& bc)
{
  updateStatusLED(bc.control);
}

void MocapBlinkM::closeSubscriptions()
{
  close(blinkm_control_sub);
}

void MocapBlinkM::closeDevices()
{
  close(bl1);
  close(bl2);
}

bool MocapBlinkM::loadParameters()
{
  default_intensity = pu::getUIntParam("MBM_INTENSITY");

  return true;
}

bool MocapBlinkM::registerCallbacks()
{
  blinkm_control_sub = orb_subscribe(ORB_ID(blinkm_control));
  if (blinkm_control_sub < 0)
  {
    puts("[MBM] blinkm_control_sub failed");
    return false;
  }

  if (orb_set_interval(blinkm_control_sub, 1000) < 0)
  {
    puts("[MBM] battery_status set interval failed");
    return false;
  }

  fds.fd = blinkm_control_sub;
  fds.events = POLLIN;

  return true;
}

void MocapBlinkM::updateStatusLED(unsigned int control)
{
  ioctl(bl2, BLINKM_SET_INTENSITY, control);

  return;
}
