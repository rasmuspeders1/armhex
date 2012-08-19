/*
 * js_input.h
 *
 *  Created on: Aug 13, 2012
 *      Author: rasmus
 */

#ifndef JS_INPUT_H_
#define JS_INPUT_H_

#define MAX_JS_COUNT 10

#include <linux/joystick.h>
#include <string>
#include <fcntl.h>

enum
{
  AXIS_L_STICK_X = 0,
  AXIS_L_STICK_Y = 1,
  AXIS_R_STICK_X = 2,
  AXIS_R_STICK_Y = 3,
  AXIS_ROLL = 4,
  AXIS_PITCH = 5,
  BUTTON_L2 = 8,
  BUTTON_R2 = 9,
  BUTTON_L1 = 10,
  BUTTON_R1 = 11,
  AXIS_L2 = 12,
  AXIS_R2 = 13,
  AXIS_L1 = 14,
  AXIS_R1 = 15,
  BUTTON_PS = 16
};

struct JSData_t
{
    float gait_x;
    float gait_y;
    float gait_z;

    float body_relative_x;
    float body_relative_y;
    float body_relative_z;

    bool body_relative_rotation_enable;
    float body_relative_roll;
    float body_relative_pitch;
    float body_relative_yaw;

    bool standby;

    JSData_t():
      gait_x(0),
      gait_y(0),
      gait_z(0),
      body_relative_x(0),
      body_relative_y(0),
      body_relative_z(0),
      body_relative_rotation_enable(false),
      body_relative_roll(0),
      body_relative_pitch(0),
      body_relative_yaw(0),
      standby(false)
    {    }

};

class JSInput
{
  public:
    JSInput(std::string js_dev_name);
    bool GetJSInput(JSData_t&);
  private:
    /**
     * Opens the joystick device
     */
    int OpenDevice(std::string js_dev);

    /**
     * Reads events from the joystick input.
     */
    bool ReadEvents();

    //joystick dev path
    std::string js_dev_;

    //File descriptor for joystick device
    int js_FD_;

    //Data struct to hold interpreted data from joystick input
    JSData_t js_data_;

    //js event struct to read joystick input into
    struct js_event js_event_;

    char name_of_joystick_[1024];
    int num_of_axis_;
    int num_of_buttons_;


};

#endif /* JS_INPUT_H_ */
