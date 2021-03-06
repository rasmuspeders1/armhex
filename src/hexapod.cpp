/*
 * Hexapod.cpp
 *
 *  Created on: Jul 10, 2012
 *      Author: rasmus
 */

#include <iostream>
#include <time.h>
#include <string>
#include <iomanip>
#include <math.h>

#include <termios.h>
#include <fcntl.h>

#include <unistd.h>

#include "hexapod.h"

#include "gait.h"

//Define all servo addresses to human readable positions
#define LR_TIBIA 0
#define LR_FEMUR 1
#define LR_COXA 2

#define RR_TIBIA 3
#define RR_FEMUR 4
#define RR_COXA 5

#define LM_TIBIA 6
#define LM_FEMUR 7
#define LM_COXA 8

#define RM_TIBIA 9
#define RM_FEMUR 10
#define RM_COXA 11

#define LF_TIBIA 12
#define LF_FEMUR 13
#define LF_COXA 14

#define RF_TIBIA 15
#define RF_FEMUR 16
#define RF_COXA 17

#define GRIPPER_PAN 18
#define GRIPPER 19

Hexapod::Hexapod() :
    js_input_("/dev/input/js0"),
	  gait_(),
    body_(kinematics::Body()),
    lr_body_link_(kinematics::Link((5.0 / 6.0) * M_PI, 0.0, 55.75, 19.0, "Left rear body link", -1)),
    lr_coxa_link_(kinematics::Link(0.0, -90.0 / (180.0 / M_PI), 11.5, 0.0, "Left rear coxa link", 2)),
    lr_femur_link_(kinematics::Link(0.0, 180.0 / (180.0 / M_PI), 45.0, 0.0, "Left rear femur link", 1)),
    lr_tibia_link_(kinematics::Link(0.0, 0.0, 70.0, 0.0, "Left rear tibia link", 0)),

    lm_body_link_(kinematics::Link(M_PI / 2.0, 0.0, 35.75, 19.0, "Left middle body link", -1)),
    lm_coxa_link_(kinematics::Link(0.0, -90.0 / (180.0 / M_PI), 11.5, 0.0, "Left middle coxa link", 8)),
    lm_femur_link_(kinematics::Link(0.0, 180.0 / (180.0 / M_PI), 45.0, 0.0, "Left middle femur link", 7)),
    lm_tibia_link_(kinematics::Link(0.0, 0.0, 70.0, 0.0, "Left middle tibia link", 6)),

    lf_body_link_(kinematics::Link(M_PI / 6.0, 0.0, 55.75, 19.0, "Left front body link", -1)),
    lf_coxa_link_(kinematics::Link(0.0 / (180.0 / M_PI), -90.0 / (180.0 / M_PI), 11.5, 0.0, "Left front coxa link", 14)),
    lf_femur_link_(kinematics::Link(0.0 / (180.0 / M_PI), 180.0 / (180.0 / M_PI), 45.0, 0.0, "Left front femur link", 13)),
    lf_tibia_link_(kinematics::Link(0.0 / (180.0 / M_PI), 0.0, 70.0, 0.0, "Left front tibia link", 12)),

    rr_body_link_(kinematics::Link(-(5.0 / 6.0) * M_PI, 0.0, 55.75, 19.0, "Right rear body link", -1)),
    rr_coxa_link_(kinematics::Link(0.0, 90.0 / (180.0 / M_PI), 11.5, 0.0, "Right rear coxa link", 5)),
    rr_femur_link_(kinematics::Link(0.0, 180.0 / (180.0 / M_PI), 45.0, 0.0, "Right rear femur link", 4)),
    rr_tibia_link_(kinematics::Link(0.0, 0.0, 70.0, 0.0, "Right rear tibia link", 3)),

    rm_body_link_(kinematics::Link(-90.0 / (180.0 / M_PI), 0.0, 35.75, 19.0, "Right middle body link", -1)),
    rm_coxa_link_(kinematics::Link(0.0, 90.0 / (180.0 / M_PI), 11.5, 0.0, "Right middle coxa link", 11)),
    rm_femur_link_(kinematics::Link(0.0, 180.0 / (180.0 / M_PI), 45.0, 0.0, "Right middle femur link", 10)),
    rm_tibia_link_(kinematics::Link(0.0, 0.0, 70.0, 0.0, "Right middle tibia link", 9)),

    rf_body_link_(kinematics::Link(-30.0 / (180.0 / M_PI), 0.0, 55.75, 19.0, "Right front body link", -1)),
    rf_coxa_link_(kinematics::Link(0.0 / (180.0 / M_PI), 90.0 / (180.0 / M_PI), 11.5, 0.0, "Right front coxa link", 17)),
    rf_femur_link_(kinematics::Link(0.0 / (180.0 / M_PI), 180.0 / (180.0 / M_PI), 45.0, 0.0, "Right front femur link", 16)),
    rf_tibia_link_(kinematics::Link(0.0 / (180.0 / M_PI), 0.0, 70.0, 0.0, "Right front tibia link", 15)),

    lr_leg_(kinematics::Limb(body_, lr_body_link_, lr_coxa_link_, lr_femur_link_, lr_tibia_link_, "Left Rear Leg")),
    lm_leg_(kinematics::Limb(body_, lm_body_link_, lm_coxa_link_, lm_femur_link_, lm_tibia_link_, "Left Middle Leg")),
    lf_leg_(kinematics::Limb(body_, lf_body_link_, lf_coxa_link_, lf_femur_link_, lf_tibia_link_, "Left Front Leg")),

    rr_leg_(kinematics::Limb(body_, rr_body_link_, rr_coxa_link_, rr_femur_link_, rr_tibia_link_, "Right Rear Leg")),
    rm_leg_(kinematics::Limb(body_, rm_body_link_, rm_coxa_link_, rm_femur_link_, rm_tibia_link_, "Right Middle Leg")),
    rf_leg_(kinematics::Limb(body_, rf_body_link_, rf_coxa_link_, rf_femur_link_, rf_tibia_link_, "Right Front Leg"))

{
  std::cout << "Initializing Hexapod\n";

  //Configure all servo center offsets
  set_center_offset(LR_TIBIA, -100 + 900 + 900);
  set_center_offset(LR_FEMUR, -10 + 900);
  set_center_offset(LR_COXA, -10 + 900 + 180);

  set_center_offset(RR_TIBIA, 90);
  set_center_offset(RR_FEMUR, -20 + 900);
  set_center_offset(RR_COXA, -80 + 900 - 180);

  set_center_offset(LM_TIBIA, 880 + 900);
  set_center_offset(LM_FEMUR, 40 + 900);
  set_center_offset(LM_COXA, -20 + 900);

  set_center_offset(RM_TIBIA, 40);
  set_center_offset(RM_FEMUR, -20 + 900);
  set_center_offset(RM_COXA, 30 + 900);

  set_center_offset(LF_TIBIA, -150 + 900 + 900);
  set_center_offset(LF_FEMUR, -20 + 900);
  set_center_offset(LF_COXA, 60 + 900 - 180);

  set_center_offset(RF_TIBIA, 120);
  set_center_offset(RF_FEMUR, 0 + 900);
  set_center_offset(RF_COXA, 80 + 900 + 180);

  set_center_offset(GRIPPER, 0 + 900);
  set_center_offset(GRIPPER_PAN, -50 + 900);

  //Open and configure serial port
  std::string serial_dev = "/dev/rfcomm0";
  if(openSerialPort(serial_dev) != -1)
  {
    configureSerialPort();
  }
  else
  {
    throw std::runtime_error("Failed to open serial port to Maestro on <" + serial_dev + ">");
  }

  setGroupPositions(0, get_all_leg_servo_angles());

}

Hexapod::~Hexapod()
{

}

bool Hexapod::Update()
{
  // Check for errors from maestro servo controller.
  // abort if bad error.

//  if(!check_maestro_errors())
//  {
//    return false;
//  }

  UpdateInput();


  return true;
}

bool Hexapod::check_maestro_errors()
{
  uint16_t error_code = GetErrors();
  switch(error_code)
  {
    case 0:
      //No Errors
      //std::cout << "Servo controller returned error indication <" << std::hex << std::setw(4) << std::setfill('0') << error_code << ">" << " No errors."<< std::endl;
      break;
    case 0x020:
      std::cout << "Servo controller returned error indication <" << std::hex << std::setw(4) << std::setfill('0') << error_code << ">" << " Serial connection time out error. Ignored." << std::endl;
      break;
    default:
      std::cout << "Servo controller returned error indication <" << std::hex << std::setw(4) << std::setfill('0') << error_code << ">" << std::endl;
      return false;
  }

  return true;
}

void Hexapod::UpdateInput()
{
  const static kinematics::MatrixValue_t BODY_RELATIVE_ROTATION_STEP = 1.0 * M_PI / 180.0;      // 1.0 degrees per step
  const static kinematics::MatrixValue_t BODY_RELATIVE_TRANSLATION_STEP = 2.0;                  // 2.0 mm per step

  static JSData_t js_data;
  static bool standing_by = false;

  if(js_input_.GetJSInput(js_data))
  {
//    std::cout <<  "Got joystick input:"
//        << "\nGait X:  " << js_data.gait_x
//        << "\nGait Y:  " << js_data.gait_y
//        << "\nGait Yaw:  " << js_data.gait_yaw
//        << "\nBody relative roll:  " << js_data.body_relative_roll
//        << "\nBody relative pitch: " << js_data.body_relative_pitch
//        << "\nBody relative yaw:   " << js_data.body_relative_yaw
//        << "\nBody relative x:   " << js_data.body_relative_x
//        << "\nBody relative y:   " << js_data.body_relative_y
//        << "\nBody relative z:   " << js_data.body_relative_z
//        << "\nGripper Angle:   " << js_data.gripper_pan
//        << "\nGrip:   " << js_data.grip
//        << std::endl;
  }

  if(js_data.standby)
  {
    if(!standing_by)
    {
      standing_by = true;
      goHomeAllServos();
    }
    usleep(15000);
    return;
  }
  else
  {
    standing_by = false;
  }

  gait_.direction(js_data.gait_x, js_data.gait_y, js_data.gait_yaw);
  gait_.body_relative_position(js_data.body_relative_x, js_data.body_relative_y, js_data.body_relative_z+20);
  gait_.body_relative_rotation(js_data.body_relative_roll / (180.0 / M_PI), js_data.body_relative_pitch / (180.0 / M_PI), js_data.body_relative_yaw / (180.0 / M_PI));
  Pose target;
  try
  {
    target = gait_.target();
  }
  catch (const kinematics::matrix_operation_error &e)
  {
    std::cout << "gait_.target() failed with: " << e.what();
    abort();
  }
  body_.update_transformation_matrix(target.body_roll_, target.body_pitch_, target.body_yaw_, target.body_x_, target.body_y_, target.body_z_);


  lf_leg_.InverseKinematic(target.lf_limb_pos_);
  lm_leg_.InverseKinematic(target.lm_limb_pos_);
  lr_leg_.InverseKinematic(target.lr_limb_pos_);

  rf_leg_.InverseKinematic(target.rf_limb_pos_);
  rm_leg_.InverseKinematic(target.rm_limb_pos_);
  rr_leg_.InverseKinematic(target.rr_limb_pos_);

  setGroupPositions(0, get_all_leg_servo_angles());
  set_grip(0, js_data.grip/ (180.0 / M_PI));

}

std::vector<double> Hexapod::get_all_leg_servo_angles()
{
  std::vector<double> servo_positions;
  servo_positions.resize(18, 0);

  servo_positions[lr_coxa_link_.get_servo_addr()] = lr_coxa_link_.theta();
  servo_positions[lr_femur_link_.get_servo_addr()] = lr_femur_link_.theta();
  servo_positions[lr_tibia_link_.get_servo_addr()] = lr_tibia_link_.theta();

  servo_positions[lm_coxa_link_.get_servo_addr()] = lm_coxa_link_.theta();
  servo_positions[lm_femur_link_.get_servo_addr()] = lm_femur_link_.theta();
  servo_positions[lm_tibia_link_.get_servo_addr()] = lm_tibia_link_.theta();

  servo_positions[lf_coxa_link_.get_servo_addr()] = lf_coxa_link_.theta();
  servo_positions[lf_femur_link_.get_servo_addr()] = lf_femur_link_.theta();
  servo_positions[lf_tibia_link_.get_servo_addr()] = lf_tibia_link_.theta();

  servo_positions[rr_coxa_link_.get_servo_addr()] = rr_coxa_link_.theta();
  servo_positions[rr_femur_link_.get_servo_addr()] = rr_femur_link_.theta();
  servo_positions[rr_tibia_link_.get_servo_addr()] = rr_tibia_link_.theta();

  servo_positions[rm_coxa_link_.get_servo_addr()] = rm_coxa_link_.theta();
  servo_positions[rm_femur_link_.get_servo_addr()] = rm_femur_link_.theta();
  servo_positions[rm_tibia_link_.get_servo_addr()] = rm_tibia_link_.theta();

  servo_positions[rf_coxa_link_.get_servo_addr()] = rf_coxa_link_.theta();
  servo_positions[rf_femur_link_.get_servo_addr()] = rf_femur_link_.theta();
  servo_positions[rf_tibia_link_.get_servo_addr()] = rf_tibia_link_.theta();

  return servo_positions;
}

void Hexapod::set_grip(double angle, double grip)
{
  std::vector<double> grip_positions;
  grip_positions.push_back(angle);
  grip_positions.push_back(grip);
  setGroupPositions(GRIPPER_PAN, grip_positions);
}
