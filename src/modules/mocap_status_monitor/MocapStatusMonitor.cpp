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
#include <drivers/drv_tone_alarm.h>
#include <drivers/drv_led.h>
#include <drivers/drv_rgbled.h>

#include "MocapStatusMonitor.h"

namespace pu = parameter_utils;

MocapStatusMonitor::MocapStatusMonitor() { }
MocapStatusMonitor::~MocapStatusMonitor() { }

bool MocapStatusMonitor::initialize()
{
  if (!loadParameters())
  {
    puts("[MSM] failed to load parameters");
    return false;
  }

  if (!registerCallbacks())
  {
    puts("[MSM] failed to register callbacks");
    return false;
  }

  if (!openDevices())
  {
    puts("[MSM] failed to open devices");
    return false;
  }

  return true;
}

void MocapStatusMonitor::finalize()
{
  closeSubscriptions();
  closeDevices();
}

void MocapStatusMonitor::update()
{
  static unsigned int error_counter = 0;
  int pret = px4_poll(&fds, 1, 1000);
  if (pret < 0)
  {
    if (error_counter++ > 50)
    {
      printf("[MSM] ERROR return value from poll(): %d", pret);
      error_counter = 0;
    }
  }
  else if (pret > 0)
  {
    if (fds.revents & POLLIN)
    {
      struct battery_status_s bs;
      orb_copy(ORB_ID(battery_status), battery_status_sub, &bs);
      batteryStatusMessageCallback(bs);
    }
  }
}

bool MocapStatusMonitor::openDevices()
{
  buzzer = px4_open(TONEALARM0_DEVICE_PATH, O_WRONLY);

  if (buzzer < 0)
  {
    puts("[MSM] Buzzer: open fail");
    return false;
  }

  leds = px4_open(RGBLED0_DEVICE_PATH, 0);

  if (leds < 0)
  {
    puts("[MSM] RGBLED: open fail");
    return false;
  }

  if (ioctl(leds, RGBLED_SET_MODE, RGBLED_MODE_ON))
  {
    puts("[MSM] RGBLED: ioctl fail");
    return false;
  }
  ioctl(leds, RGBLED_SET_COLOR, RGBLED_COLOR_OFF);

  return true;
}

void MocapStatusMonitor::printBatteryStatus(const battery_status_s& in)
{
  // HACK: NuttX printf double bug
  char buf1[32], buf2[32];
  sprintf(buf1, "%0.4f", (double)in.voltage_v);
  sprintf(buf2, "%0.4f", (double)in.voltage_filtered_v);
  printf("[MSM] status (raw, filtered) = %s, %s\n", buf1, buf2);
}

void MocapStatusMonitor::batteryStatusMessageCallback(const battery_status_s& bs)
{
#if 0
  printBatteryStatus(bs);
#endif

  if (bs.voltage_filtered_v > battery_ignore)
  {
    if (bs.voltage_filtered_v < battery_low)
    {
      if (bs.voltage_filtered_v > battery_critical)
      {
        playLowBatteryTune();
        enableWarnLEDS();
      }
      else
      {
        playCriticalBatteryTune();
        enableCriticalLEDS();
      }
    }
    else
    {
      stopBatteryTune();
      disableLEDS();
    }
  }

  return;
}

void MocapStatusMonitor::closeSubscriptions()
{
  close(battery_status_sub);
}

void MocapStatusMonitor::closeDevices()
{
  close(buzzer);
  close(leds);
}

bool MocapStatusMonitor::loadParameters()
{
  battery_ignore = pu::getFloatParam("MSM_IGN_THRESH");
  battery_low = pu::getFloatParam("MSM_LOW_THRESH");
  battery_critical = pu::getFloatParam("MSM_CRIT_THRESH");

  return true;
}

bool MocapStatusMonitor::registerCallbacks()
{
  battery_status_sub = orb_subscribe(ORB_ID(battery_status));
  if (battery_status_sub < 0)
  {
    puts("[MSM] battery_status_sub failed");
    return false;
  }

  if (orb_set_interval(battery_status_sub, 1000) < 0)
  {
    puts("[MSM] battery_status set interval failed");
    return false;
  }

  fds.fd = battery_status_sub;
  fds.events = POLLIN;

  return true;
}

int MocapStatusMonitor::playLowBatteryTune()
{
  return ioctl(buzzer, TONE_SET_ALARM, TONE_BATTERY_WARNING_SLOW_TUNE);
}

int MocapStatusMonitor::playCriticalBatteryTune()
{
  return ioctl(buzzer, TONE_SET_ALARM, TONE_BATTERY_WARNING_FAST_TUNE);
}

void MocapStatusMonitor::stopBatteryTune()
{
  ioctl(buzzer, TONE_SET_ALARM, TONE_STOP_TUNE);
}

void MocapStatusMonitor::enableWarnLEDS()
{
  px4_ioctl(leds, RGBLED_SET_COLOR, RGBLED_COLOR_RED);
  px4_ioctl(leds, RGBLED_SET_MODE, RGBLED_MODE_BREATHE);
}

void MocapStatusMonitor::enableCriticalLEDS()
{
  px4_ioctl(leds, RGBLED_SET_COLOR, RGBLED_COLOR_RED);
  px4_ioctl(leds, RGBLED_SET_MODE, RGBLED_MODE_ON);
}

void MocapStatusMonitor::disableLEDS()
{
  px4_ioctl(leds, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_OFF);
}
