/*
 * Hexapod.h
 *
 *  Created on: Jul 10, 2012
 *      Author: rasmus
 */

#ifndef HEXAPOD_H_
#define HEXAPOD_H_

#include "MaestroController.h"
#include "Kinematics.h"

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

};

#endif /* HEXAPOD_H_ */
