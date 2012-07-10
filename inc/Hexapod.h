/*
 * Hexapod.h
 *
 *  Created on: Jul 10, 2012
 *      Author: rasmus
 */

#ifndef HEXAPOD_H_
#define HEXAPOD_H_

#include "MaestroController.h"

class Hexapod: public MaestroController
{
public:
  Hexapod();
  ~Hexapod(){};
private:
  virtual bool UpdatePositions();

};

#endif /* HEXAPOD_H_ */
