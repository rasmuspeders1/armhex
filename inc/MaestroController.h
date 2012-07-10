/*
 * maestro.h
 *
 *  Created on: Jul 10, 2012
 *      Author: rasmus
 */

#ifndef MAESTRO_H_
#define MAESTRO_H_

class MaestroController
{
public:
  MaestroController();
  virtual ~MaestroController(){};
private:
  virtual bool UpdatePositions() = 0;

};


#endif /* MAESTRO_H_ */
