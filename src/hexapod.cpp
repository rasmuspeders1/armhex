/*
 * Hexapod.cpp
 *
 *  Created on: Jul 10, 2012
 *      Author: rasmus
 */

#include <iostream>
#include <time.h>
#include <string>

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
body(kinematics::Body()),
lr_body_link_(kinematics::Link(150.0 / (180.0 / M_1_PI), 0.0, 55.75, 19.0, "Left rear body link", -1)),
lr_coxa_link_(kinematics::Link(M_1_PI/4, -90.0 / (180.0 / M_1_PI), 11.5, 0.0, "Left rear coxa link", 2)),
lr_femur_link_(kinematics::Link(0.0, 180.0 / (180.0 / M_1_PI), 45.0, 0.0, "Left rear femur link", 1)),
lr_tibia_link_(kinematics::Link(0.0, 0.0, 70.0, 0.0, "Left rear tibia link", 0)),

lm_body_link_(kinematics::Link(90.0 / (180.0 / M_1_PI), 0.0, 35.75, 19.0, "Left middle body link", -1)),
lm_coxa_link_(kinematics::Link(0.0, -90.0 / (180.0 / M_1_PI), 11.5, 0.0, "Left middle coxa link", 8)),
lm_femur_link_(kinematics::Link(0.0, 180.0 / (180.0 / M_1_PI), 45.0, 0.0, "Left middle femur link", 7)),
lm_tibia_link_(kinematics::Link(0.0, 0.0, 70.0, 0.0, "Left middle tibia link", 6)),

lf_body_link_(kinematics::Link(30.0 / (180.0 / M_1_PI), 0.0, 35.75, 19.0, "Left front body link", -1)),
lf_coxa_link_(kinematics::Link(0.0, -90.0 / (180.0 / M_1_PI), 11.5, 0.0, "Left front coxa link", 14)),
lf_femur_link_(kinematics::Link(0.0, 180.0 / (180.0 / M_1_PI), 45.0, 0.0, "Left front femur link", 13)),
lf_tibia_link_(kinematics::Link(0.0, 0.0, 70.0, 0.0, "Left front tibia link", 12)),

rr_body_link_(kinematics::Link(-150.0 / (180.0 / M_1_PI), 0.0, 55.75, 19.0, "Right rear body link", -1)),
rr_coxa_link_(kinematics::Link(0.0, -90.0 / (180.0 / M_1_PI), 11.5, 0.0, "Right rear coxa link", 5)),
rr_femur_link_(kinematics::Link(0.0, 180.0 / (180.0 / M_1_PI), 45.0, 0.0, "Right rear femur link", 4)),
rr_tibia_link_(kinematics::Link(0.0, 0.0, 70.0, 0.0, "Right rear tibia link", 3)),

rm_body_link_(kinematics::Link(-90.0 / (180.0 / M_1_PI), 0.0, 35.75, 19.0, "Right middle body link", -1)),
rm_coxa_link_(kinematics::Link(0.0, -90.0 / (180.0 / M_1_PI), 11.5, 0.0, "Right middle coxa link", 11)),
rm_femur_link_(kinematics::Link(0.0, 180.0 / (180.0 / M_1_PI), 45.0, 0.0, "Right middle femur link", 10)),
rm_tibia_link_(kinematics::Link(0.0, 0.0, 70.0, 0.0, "Right middle tibia link", 9)),

rf_body_link_(kinematics::Link(-30.0 / (180.0 / M_1_PI), 0.0, 55.75, 19.0, "Right front body link", -1)),
rf_coxa_link_(kinematics::Link(0.0, -90.0 / (180.0 / M_1_PI), 11.5, 0.0, "Right front coxa link", 17)),
rf_femur_link_(kinematics::Link(0.0, 180.0 / (180.0 / M_1_PI), 45.0, 0.0, "Right front femur link", 16)),
rf_tibia_link_(kinematics::Link(0.0, 0.0, 70.0, 0.0, "Right front tibia link", 15)),

lr_leg_(kinematics::Limb(body, lr_body_link_, lr_coxa_link_, lr_femur_link_, lr_tibia_link_)),
lm_leg_(kinematics::Limb(body, lm_body_link_, lm_coxa_link_, lm_femur_link_, lm_tibia_link_)),
lf_leg_(kinematics::Limb(body, lf_body_link_, lf_coxa_link_, lf_femur_link_, lf_tibia_link_)),

rr_leg_(kinematics::Limb(body, rr_body_link_, rr_coxa_link_, rr_femur_link_, rr_tibia_link_)),
rm_leg_(kinematics::Limb(body, rm_body_link_, rm_coxa_link_, rm_femur_link_, rm_tibia_link_)),
rf_leg_(kinematics::Limb(body, rf_body_link_, rf_coxa_link_, rf_femur_link_, rf_tibia_link_))

{
  std::cout << "Initializing Hexapod\n";

  //Configure all servo center offsets
  setCenterOffset(LR_TIBIA, 800+900);
  setCenterOffset(LR_FEMUR, -20+900);
  setCenterOffset(LR_COXA, -20+900);

  setCenterOffset(RR_TIBIA, 80);
  setCenterOffset(RR_FEMUR, -20+900);
  setCenterOffset(RR_COXA, -80+900);

  setCenterOffset(LM_TIBIA, 880+900);
  setCenterOffset(LM_FEMUR, 40+900);
  setCenterOffset(LM_COXA, -20+900);

  setCenterOffset(RM_TIBIA, 80);
  setCenterOffset(RM_FEMUR, -20+900);
  setCenterOffset(RM_COXA, 30+900);

  setCenterOffset(LF_TIBIA, -40+900+900);
  setCenterOffset(LF_FEMUR, -20+900);
  setCenterOffset(LF_COXA, 80+900);

  setCenterOffset(RF_TIBIA, 100);
  setCenterOffset(RF_FEMUR, 0+900);
  setCenterOffset(RF_COXA, 60+900);

  setCenterOffset(GRIPPER, 0+900);
  setCenterOffset(GRIPPER_PAN, -50+900);


}

Hexapod::~Hexapod()
{

}

bool Hexapod::Update()
{
  //std::cout << "Controller Update" << std::endl;
  //get errors from maestro. If error code is larger than 0, an error occured on the meastro and we should exit gracefully.
//  if(get_errors())
//  {
//    return false;
//  }

  setGroupPositions(0, get_all_servo_angles());

  usleep(300000);
  goHomeAllServos();

  return false;
}

std::vector<float> Hexapod::get_all_servo_angles()
{
  std::vector<float> servo_positions;
  servo_positions.resize(20,0);

  servo_positions[lr_coxa_link_.get_servo_addr()] = lr_coxa_link_.get_theta();
  servo_positions[lr_femur_link_.get_servo_addr()] = lr_femur_link_.get_theta();
  servo_positions[lr_tibia_link_.get_servo_addr()] = lr_tibia_link_.get_theta();

  servo_positions[lm_coxa_link_.get_servo_addr()] = lm_coxa_link_.get_theta();
  servo_positions[lm_femur_link_.get_servo_addr()] = lm_femur_link_.get_theta();
  servo_positions[lm_tibia_link_.get_servo_addr()] = lm_tibia_link_.get_theta();

  servo_positions[lf_coxa_link_.get_servo_addr()] = lf_coxa_link_.get_theta();
  servo_positions[lf_femur_link_.get_servo_addr()] = lf_femur_link_.get_theta();
  servo_positions[lf_tibia_link_.get_servo_addr()] = lf_tibia_link_.get_theta();

  servo_positions[rr_coxa_link_.get_servo_addr()] = rr_coxa_link_.get_theta();
  servo_positions[rr_femur_link_.get_servo_addr()] = rr_femur_link_.get_theta();
  servo_positions[rr_tibia_link_.get_servo_addr()] = rr_tibia_link_.get_theta();

  servo_positions[rm_coxa_link_.get_servo_addr()] = rm_coxa_link_.get_theta();
  servo_positions[rm_femur_link_.get_servo_addr()] = rm_femur_link_.get_theta();
  servo_positions[rm_tibia_link_.get_servo_addr()] = rm_tibia_link_.get_theta();

  servo_positions[rf_coxa_link_.get_servo_addr()] = rf_coxa_link_.get_theta();
  servo_positions[rf_femur_link_.get_servo_addr()] = rf_femur_link_.get_theta();
  servo_positions[rf_tibia_link_.get_servo_addr()] = rf_tibia_link_.get_theta();

  return servo_positions;
}
