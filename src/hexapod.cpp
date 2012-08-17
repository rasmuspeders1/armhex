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

#include <termios.h>
#include <fcntl.h>

#include "hexapod.h"

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

Hexapod::Hexapod():
    js_input_("/dev/input/js0"),
    body_(kinematics::Body()),
    lr_body_link_(kinematics::Link((5.0/6.0) * M_PI, 0.0, 55.75, 19.0, "Left rear body link", -1)),
    lr_coxa_link_(kinematics::Link(0.0, -90.0 / (180.0 / M_PI), 11.5, 0.0, "Left rear coxa link", 2)),
    lr_femur_link_(kinematics::Link(0.0, 180.0 / (180.0 / M_PI), 45.0, 0.0, "Left rear femur link", 1)),
    lr_tibia_link_(kinematics::Link(0.0, 0.0, 70.0, 0.0, "Left rear tibia link", 0)),

    lm_body_link_(kinematics::Link(M_PI/2.0, 0.0, 35.75, 19.0, "Left middle body link", -1)),
    lm_coxa_link_(kinematics::Link(0.0, -90.0 / (180.0 / M_PI), 11.5, 0.0, "Left middle coxa link", 8)),
    lm_femur_link_(kinematics::Link(0.0, 180.0 / (180.0 / M_PI), 45.0, 0.0, "Left middle femur link", 7)),
    lm_tibia_link_(kinematics::Link(0.0, 0.0, 70.0, 0.0, "Left middle tibia link", 6)),

    lf_body_link_(kinematics::Link(M_PI/6.0,    0.0, 55.75, 19.0, "Left front body link", -1)),
    lf_coxa_link_(kinematics::Link( 0.0 / (180.0 / M_PI),  -90.0 / (180.0 / M_PI), 11.5, 0.0, "Left front coxa link", 14)),
    lf_femur_link_(kinematics::Link(0.0 / (180.0 / M_PI),  180.0 / (180.0 / M_PI), 45.0, 0.0, "Left front femur link", 13)),
    lf_tibia_link_(kinematics::Link(0.0 / (180.0 / M_PI),    0.0, 70.0, 0.0, "Left front tibia link", 12)),

    rr_body_link_(kinematics::Link(-150.0 / (180.0 / M_PI), 0.0, 55.75, 19.0, "Right rear body link", -1)),
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
  set_center_offset(LR_COXA,  -10 + 900 + 180);

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
  if(openSerialPort("/dev/ttyAMA0") != -1)
  {
    configureSerialPort();
  }

//  kinematics::Matrix A(3,3);
//  kinematics::Matrix B(3,3);
//  kinematics::Matrix C(3,1);
//
//  A[0][0] = 1;
//  A[0][1] = 2;
//  A[0][2] = 3;
//
//  A[1][0] = 4;
//  A[1][1] = 5;
//  A[1][2] = 6;
//
//  A[2][0] = 7;
//  A[2][1] = 8;
//  A[2][2] = 9;
//
//  B[0][0] = 1;
//  B[0][1] = 2;
//  B[0][2] = 3;
//
//  B[1][0] = 4;
//  B[1][1] = 5;
//  B[1][2] = 6;
//
//  B[2][0] = 7;
//  B[2][1] = 8;
//  B[2][2] = 9;
//
//  C[0][0] = 1;
//  C[1][0] = 2;
//  C[2][0] = 3;
//
//  (A*B).print();
//  (A*C).print();

  //body_.update_transformation_matrix(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);

   //lm_leg_.InverseKinematic(50, 90, 0);

  //body_.update_transformation_matrix(0.0, 0.0, 0.0, -10.0, 0.0, 0.0);

  //lf_leg_.InverseKinematic(85.0, 75.0, 0.0);
//  lm_leg_.InverseKinematic(0.0, 90.0, 0.0);
//  lr_leg_.InverseKinematic(-85.0, 75.0, 0.0);
//
//  rf_leg_.InverseKinematic(85.0, -75.0, 0.0);
//  rm_leg_.InverseKinematic(0.0, -90.0, 0.0);
//  rr_leg_.InverseKinematic(-85.0, -75.0, 0.0);

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

  static kinematics::MatrixValue_t body_x = 0;
  static kinematics::MatrixValue_t body_y = 0;
  static kinematics::MatrixValue_t body_z = 0;

  static kinematics::MatrixValue_t body_roll = 0;
  static kinematics::MatrixValue_t body_pitch = 0;

  static JSData_t js_data;

  if(js_input_.GetJSInput(js_data))
  {
//    std::cout <<  "Got joystick input:"
//        << "\nBody relative roll:  " << js_data.body_relative_roll
//        << "\nBody relative pitch: " << js_data.body_relative_pitch
//        << "\nBody relative yaw:   " << js_data.body_relative_yaw
//        << "\nBody relative x:   " << js_data.body_relative_x
//        << "\nBody relative y:   " << js_data.body_relative_y
//        << "\nBody relative z:   " << js_data.body_relative_z
//        << std::endl;
    body_roll = js_data.body_relative_roll;
    body_pitch = js_data.body_relative_pitch;
    body_z = 10.0 + js_data.body_relative_z;
  }



//  static bool dir = true;
//
//  if(dir)
//  {
//    body_x+=0.5;
//  }
//  else
//  {
//    body_x-=0.5;
//  }
//
//  if(body_x > 40)
//  {
//    dir = false;
//  }
//  if(body_x < -40)
//  {
//    dir = true;
//  }

  body_.update_transformation_matrix(body_roll, body_pitch, 0.0, body_x, body_y, body_z);

  lf_leg_.InverseKinematic(85.0, 75.0, 0.0);
  lm_leg_.InverseKinematic(0.0, 90.0, 0.0);
  lr_leg_.InverseKinematic(-85.0, 75.0, 0.0);

  rf_leg_.InverseKinematic(85.0, -75.0, 0.0);
  rm_leg_.InverseKinematic(0.0, -80.0, 0.0);
  rr_leg_.InverseKinematic(-85.0, -75.0, 0.0);

  setGroupPositions(0, get_all_leg_servo_angles());

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
