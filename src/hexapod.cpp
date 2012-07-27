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

Hexapod::Hexapod()
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

  body = kinematics::Body();

  lrBodyLink = kinematics::Link(150.0, 0.0, 55.75, 19.0, "Left rear body link", -1);
  lrCoxaLink = kinematics::Link(0.0, -90.0, 11.5, 0.0, "Left rear coxa link", 2);
  lrFemurLink = kinematics::Link(0.0, 180, 45.0, 0.0, "Left rear femur link", 1);
  lrTibiaLink = kinematics::Link(0.0, 0.0, 70.0, 0.0, "Left rear tibia link", 0);

  lmBodyLink = kinematics::Link(90.0, 0.0, 35.75, 19.0, "Left middle body link", -1);
  lmCoxaLink = kinematics::Link(0.0, -90.0, 11.5, 0.0, "Left middle coxa link", 8);
  lmFemurLink = kinematics::Link(0.0, 180.0, 45.0, 0.0, "Left middle femur link", 7);
  lmTibiaLink = kinematics::Link(0.0, 0.0, 70.0, 0.0, "Left middle tibia link", 6);

  lfBodyLink = kinematics::Link(30, 0.0, 35.75, 19.0, "Left front body link", -1);
  lfCoxaLink = kinematics::Link(0.0, -90.0, 11.5, 0.0, "Left front coxa link", 14);
  lfFemurLink = kinematics::Link(0.0, 180.0, 45.0, 0.0, "Left front femur link", 13);
  lfTibiaLink = kinematics::Link(0.0, 0.0, 70.0, 0.0, "Left front tibia link", 12);

  rrBodyLink = kinematics::Link(-150.0, 0.0, 55.75, 19.0, "Right rear body link", -1);
  rrCoxaLink = kinematics::Link(0.0, -90.0, 11.5, 0.0, "Right rear coxa link", 5);
  rrFemurLink = kinematics::Link(0.0, 180.0, 45.0, 0.0, "Right rear femur link", 4);
  rrTibiaLink = kinematics::Link(0.0, 0.0, 70.0, 0.0, "Right rear tibia link", 3);

  rmBodyLink = kinematics::Link(-90.0, 0.0, 35.75, 19.0, "Right middle body link", -1);
  rmCoxaLink = kinematics::Link(0.0, -90.0, 11.5, 0.0, "Right middle coxa link", 11);
  rmFemurLink = kinematics::Link(0.0, 180.0, 45.0, 0.0, "Right middle femur link", 10);
  rmTibiaLink = kinematics::Link(0.0, 0.0, 70.0, 0.0, "Right middle tibia link", 9);

  rfBodyLink = kinematics::Link(-30.0, 0.0, 55.75, 19.0, "Right front body link", -1);
  rfCoxaLink = kinematics::Link(0.0, -90.0, 11.5, 0.0, "Right front coxa link", 17);
  rfFemurLink = kinematics::Link(0.0, 180.0, 45.0, 0.0, "Right front femur link", 16);
  rfTibiaLink = kinematics::Link(0.0, 0.0, 70.0, 0.0, "Right front tibia link", 15);

  lrLeg = kinematics::Limb(body, lrBodyLink, lrCoxaLink, lrFemurLink, lrTibiaLink);
  lmLeg = kinematics::Limb(body, lmBodyLink, lmCoxaLink, lmFemurLink, lmTibiaLink);
  lfLeg = kinematics::Limb(body, lfBodyLink, lfCoxaLink, lfFemurLink, lfTibiaLink);

  rrLeg = kinematics::Limb(body, rrBodyLink, rrCoxaLink, rrFemurLink, rrTibiaLink);
  rmLeg = kinematics::Limb(body, rmBodyLink, rmCoxaLink, rmFemurLink, rmTibiaLink);
  rfLeg = kinematics::Limb(body, rfBodyLink, rfCoxaLink, rfFemurLink, rfTibiaLink);

}

Hexapod::~Hexapod()
{

}

bool Hexapod::Update()
{
  //std::cout << "Controller Update" << std::endl;

  positions.assign(20, 0);
  setGroupPositions(0, positions);

  return true;
}
