/*
 * Hexapod.cpp
 *
 *  Created on: Jul 10, 2012
 *      Author: rasmus
 */

#include <iostream>
#include "Hexapod.h"
#include <time.h>

Hexapod::Hexapod()
{
  std::cout << "Initializing Hexapod\n";
}

bool Hexapod::UpdatePositions()
{
  return true;
}

void Hexapod::Run()
{
  std::cout << "Hexapod running\n";
  std::vector<unsigned int> positions;

  for(unsigned int i = 0; i<17; ++i)
  {
    positions.push_back(1500);
  }

  if( !setGroupPositions(0, positions))
  {
    std::cout<< "Failed to set positions!\n";
  }
  usleep(3000000);
  goHomeAllServos();
}
