/*
 * Hexapod.cpp
 *
 *  Created on: Jul 10, 2012
 *      Author: rasmus
 */

#include <iostream>
#include "Hexapod.h"
#include <time.h>

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

#define GRIPPER 18
#define GRIPPER_PAN 19

Hexapod::Hexapod()
{
  std::cout << "Initializing Hexapod\n";
  setCenterOffset(LR_TIBIA, 20);
  setCenterOffset(LR_FEMUR, -20);
  setCenterOffset(LR_COXA, -20);

  setCenterOffset(RR_TIBIA, 80);
  setCenterOffset(RR_FEMUR, -20);
  setCenterOffset(RR_COXA, -80);

  setCenterOffset(LM_TIBIA, -60);
  setCenterOffset(LM_FEMUR, 40);
  setCenterOffset(LM_COXA, -20);

  setCenterOffset(RM_TIBIA, 80);
  setCenterOffset(RM_FEMUR, -20);
  setCenterOffset(RM_COXA, 30);

  setCenterOffset(LF_TIBIA, -40);
  setCenterOffset(LF_FEMUR, -20);
  setCenterOffset(LF_COXA, 80);

  setCenterOffset(RF_TIBIA, -40);
  setCenterOffset(RF_FEMUR, 0);
  setCenterOffset(RF_COXA, 60);
}

Hexapod::~Hexapod()
{

}

bool Hexapod::Update()
{
  //std::cout << "Controller Update" << std::endl;

  positions.assign(20, 90);

  setGroupPositions(0, positions);

  return true;
}
