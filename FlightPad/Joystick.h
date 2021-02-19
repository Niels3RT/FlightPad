/*  Joystick.h
 *   
 *  FlightPad by Niels Lueddecke
 *  based on:
 *   
 *  Based on the advanced HID library for Arduino: 
 *  https://github.com/NicoHood/HID
 *  Copyright (c) 2014-2015 NicoHood
 * 
 *  Copyright (c) 2020 Mikael Norrgård <http://daemonbite.com>
 *
 *  GNU GENERAL PUBLIC LICENSE
 *  Version 3, 29 June 2007
 *  
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *  
 */

#pragma once

#include <Arduino.h>
#include "HID.h"

extern const char* gp_serial;

// calibration data
typedef struct {
  uint16_t base;
  uint16_t mid;
  uint16_t top;
  float quot_up;
  float quot_down;
} CalData;

// The numbers after colon are bit fields, meaning how many bits the field uses.
// Remove those if there are problems
typedef struct {
  uint32_t buttons;

  int8_t x;
  int8_t y;
  int8_t jaux_x;
  int8_t jaux_y;
  int8_t z;
  int8_t throttle;
  int8_t slider0;
  int8_t dial;
} JoystickReport;


class Joystick_ : public PluggableUSBModule
{  
  private:
    uint8_t reportId;

  protected:
    int getInterface(uint8_t* interfaceCount);
    int getDescriptor(USBSetup& setup);
    bool setup(USBSetup& setup);
    uint8_t getShortName(char *name);

    uint8_t epType[1];
    uint8_t protocol;
    uint8_t idle;
    
  public:
    JoystickReport _JoystickReport;
    Joystick_(void);
    void reset(void);
    void send();
};
