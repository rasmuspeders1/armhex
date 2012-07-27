/*
 * Hexapod.h
 *
 *  Created on: Jul 10, 2012
 *      Author: rasmus
 */

#ifndef HEXAPOD_H_
#define HEXAPOD_H_

#include "maestro_controller.h"
#include "kinematics.h"

class Hexapod: public MaestroController
{
public:
  Hexapod();
  virtual ~Hexapod();

private:
  virtual bool Update();

  //Member variable holding the positions of all servos in degrees (floating point value)
  //The indices are the servo addresses on the MAestro controller.
  std::vector<float> positions;

  kinematics::Body body;

  kinematics::Link lrBodyLink;
  kinematics::Link lrCoxaLink;
  kinematics::Link lrFemurLink;
  kinematics::Link lrTibiaLink;

  kinematics::Link lmBodyLink;
  kinematics::Link lmCoxaLink;
  kinematics::Link lmFemurLink;
  kinematics::Link lmTibiaLink;

  kinematics::Link lfBodyLink;
  kinematics::Link lfCoxaLink;
  kinematics::Link lfFemurLink;
  kinematics::Link lfTibiaLink;

  kinematics::Link rrBodyLink;
  kinematics::Link rrCoxaLink;
  kinematics::Link rrFemurLink;
  kinematics::Link rrTibiaLink;

  kinematics::Link rmBodyLink;
  kinematics::Link rmCoxaLink;
  kinematics::Link rmFemurLink;
  kinematics::Link rmTibiaLink;

  kinematics::Link rfBodyLink;
  kinematics::Link rfCoxaLink;
  kinematics::Link rfFemurLink;
  kinematics::Link rfTibiaLink;

  kinematics::Limb lrLeg;
  kinematics::Limb lmLeg;
  kinematics::Limb lfLeg;

  kinematics::Limb rrLeg;
  kinematics::Limb rmLeg;
  kinematics::Limb rfLeg;

};

#endif /* HEXAPOD_H_ */
