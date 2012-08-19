/*
 * js_input.cpp
 *
 *  Created on: Aug 13, 2012
 *      Author: rasmus
 */

#include "js_input.h"
#include <stdio.h>
#include <iostream>
#include <sys/ioctl.h>
#include <errno.h>
#include <math.h>

JSInput::JSInput(std::string js_dev_name) :
    js_dev_(js_dev_name),
    js_FD_(-1)
{
  OpenDevice(js_dev_);
}

int JSInput::OpenDevice(std::string js_dev)
{
  js_dev_ = js_dev;
  js_FD_ = open(js_dev.c_str(), O_RDONLY | O_NONBLOCK);
  if(js_FD_ == -1)
  {
    std::cout << "Failed to open joystick device <" << js_dev << ">" << std::endl;
    return -1;
  }

  ioctl(js_FD_, JSIOCGAXES, &num_of_axis_);
  ioctl(js_FD_, JSIOCGBUTTONS, &num_of_buttons_);
  ioctl(js_FD_, JSIOCGNAME(1024), &name_of_joystick_);

  std::cout << "Joystick detected: " << name_of_joystick_ << "\nAxis: " << num_of_axis_ << "\nButtons: " << num_of_buttons_ << std::endl;

  return js_FD_;
}

bool JSInput::GetJSInput(JSData_t& js_data)
{
  if(!ReadEvents())
  {
    //std::cout << "Failed to read events from joystick device <" << js_dev_ << ">" << std::endl;
    return false;
  }
  js_data = js_data_;
  return true;
}

bool JSInput::ReadEvents()
{
  if(js_FD_ == -1)
  {
    return false;
  }

  bool ret_val = true;
  int read_return = 0;

  do
  {
    read_return = read(js_FD_, &js_event_, sizeof(struct js_event));
    if(read_return>0)
    {
      switch (js_event_.number) {
        case AXIS_R_STICK_X:
          if(js_data_.body_relative_rotation_enable)
          {
            js_data_.body_relative_y = - js_event_.value * (30.0)/32768.0;
          }
          break;
        case AXIS_R_STICK_Y:
          if(js_data_.body_relative_rotation_enable)
          {
            js_data_.body_relative_x = - js_event_.value * (30.0)/32768.0;
          }
          break;
        case BUTTON_R1:
          js_data_.body_relative_rotation_enable = js_event_.value;
          break;
        case AXIS_ROLL:
          if(js_data_.body_relative_rotation_enable)
          {
            js_data_.body_relative_roll = js_event_.value * (0.12*M_PI)/32768.0;
          }
          break;
        case AXIS_PITCH:
          if(js_data_.body_relative_rotation_enable)
          {
            js_data_.body_relative_pitch = - js_event_.value * (0.12*M_PI)/32768.0;
          }
          break;
        case AXIS_R2:
          js_data_.body_relative_z = js_event_.value  * 50.0/32768.0;
          break;
        case BUTTON_PS:
          if(js_event_.value)
          {
            js_data_.standby = !js_data_.standby;
          }
          break;
        default:
          break;
      }
      ret_val = true;
    }
    //TODO: Do something to events
  }
  while (read_return > 0);

  return ret_val;
}

