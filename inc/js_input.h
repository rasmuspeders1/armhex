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
  AXIS_BUTTON_DPAD_UP = 8,
  AXIS_BUTTON_DPAD_RIGHT = 9,
  AXIS_BUTTON_DPAD_DOWN = 10,
  AXIS_BUTTON_DPAD_KEFT = 11,
  AXIS_L2 = 12,
  AXIS_R2 = 13,
  AXIS_L1 = 14,
  AXIS_R1 = 15,
  AXIS_BUTTON_TRIANGLE = 16,
  AXIS_BUTTON_CIRCLE = 17,
  AXIS_BUTTON_CROSS = 18,
  AXIS_BUTTON_SQUARE = 19,

};

enum
{
  BUTTON_SELECT = 0,
  BUTTON_L3 = 1,
  BUTTON_R3 = 2,
  BUTTON_START = 3,
  BUTTON_DPAD_UP = 4,
  BUTTON_DPAD_RIGHT = 5,
  BUTTON_DPAD_DOWN = 6,
  BUTTON_DPAD_LEFT = 7,
  BUTTON_L2 = 8,
  BUTTON_R2 = 9,
  BUTTON_L1 = 10,
  BUTTON_R1 = 11,
  BUTTON_TRIANGLE = 12,
  BUTTON_CIRCLE = 13,
  BUTTON_CROSS = 14,
  BUTTON_SQUARE = 15,
  BUTTON_PS = 16
};

struct JSData_t
{
    float gait_x;
    float gait_y;
    float gait_z;

    bool body_relative_translation_enable;
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
      body_relative_translation_enable(false),
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

    /**
     * Handles axis events
     */
    void HandleAxisEvent(const js_event& event);

    /**
     * Handles button events
     */
    void HandleButtonEvent(const js_event& event);

    /**
     * Handles init events
     */
    void HandleInitEvent(const js_event& event);

    //joystick dev path
    std::string js_dev_;

    //File descriptor for joystick device
    int js_FD_;

    //Data struct to hold interpreted data from joystick input
    JSData_t js_data_;

    char name_of_joystick_[1024];
    int num_of_axis_;
    int num_of_buttons_;


};

#endif /* JS_INPUT_H_ */
