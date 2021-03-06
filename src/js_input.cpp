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
#include <unistd.h>
#include <string.h>
#include <stdexcept>

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
    throw std::runtime_error("Failed to open Joystick <" + js_dev + ">");
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

  int read_return = 0;

  //js event struct to read joystick input into
  struct js_event event;
  memset(&event,0, sizeof(js_event));

  do
  {
    read_return = read(js_FD_, &event, sizeof(struct js_event));
    if(read_return>0)
    {
      switch (event.type) {
        case JS_EVENT_AXIS:
          HandleAxisEvent(event);
          break;
        case JS_EVENT_BUTTON:
          HandleButtonEvent(event);
          break;
        case JS_EVENT_INIT:
          HandleInitEvent(event);
          break;
        default:
          //std::cout << "Unable to handle JS Event type: " << int(event.type) << std::endl;
          break;
      }

    }
  }
  while (read_return > 0);

  return true;
}

void JSInput::HandleAxisEvent(const js_event& event)
{
  //std::cout << "Handling axis event type number: " << int(event.number) << std::endl;
  switch (event.number) {
    case AXIS_R_STICK_X:
      if(js_data_.body_relative_translation_enable)
      {
        js_data_.body_relative_y = - event.value * (30.0)/32768.0;
      }
      else
      {
        js_data_.gait_y = - event.value * (30.0)/32768.0;
      }
      break;
    case AXIS_R_STICK_Y:
      if(js_data_.body_relative_translation_enable)
      {
        js_data_.body_relative_x = - event.value * (30.0)/32768.0;
      }
      else
      {
        js_data_.gait_x = - event.value * (30.0)/32768.0;
      }
      break;
    case AXIS_L_STICK_X:
      if(js_data_.body_relative_translation_enable)
      {
        js_data_.body_relative_roll = - event.value * (10.0)/32768.0;
      }
      else
      {
        js_data_.gait_yaw = - event.value * (0.12*M_PI)/32768.0;
      }
      break;
    case AXIS_L_STICK_Y:
      if(js_data_.body_relative_translation_enable)
      {
        js_data_.body_relative_pitch= - event.value * (10.0)/32768.0;
      }
      break;
    case AXIS_ROLL:
      if(js_data_.body_relative_rotation_enable)
      {
        js_data_.body_relative_roll = event.value * (0.12*M_PI)/32768.0;
      }
      break;
    case AXIS_PITCH:
      if(js_data_.body_relative_rotation_enable)
      {
        js_data_.body_relative_pitch = - event.value * (0.12*M_PI)/32768.0;
      }
      break;
    case AXIS_R2:
      js_data_.body_relative_z = event.value  * 30.0/32768.0 + 30.0;
      break;
    case AXIS_L2:
      js_data_.grip = - event.value  * 45.0/32768.0 + 45.0;
      break;
    default:
      break;
  }
}

void JSInput::HandleButtonEvent(const js_event& event)
{
  //std::cout << "Handling bntton event type number: " << int(event.number) << std::endl;
  switch (event.number) {
    case BUTTON_L1:
          js_data_.body_relative_rotation_enable = event.value;
          break;
    case BUTTON_R1:
          js_data_.body_relative_translation_enable = event.value;
          if(js_data_.body_relative_translation_enable)
          {
            js_data_.body_relative_x = 0;
            js_data_.body_relative_y = 0;
          }
          break;
    case BUTTON_PS:
      if(event.value)
      {
        js_data_.standby = !js_data_.standby;
      }
      break;
    case BUTTON_CROSS:
      if(event.value)
      {
        js_data_.standby = !js_data_.standby;
      }
      break;
    default:
      break;
  }
}

void JSInput::HandleInitEvent(const js_event& event)
{
}
