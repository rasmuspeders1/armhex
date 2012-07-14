/*
 * main.cpp
 *
 *  Created on: Jul 9, 2012
 *      Author: rasmus
 */

/*
 * Includes
 */
#include <stdio.h>
#include <cstdlib>
#include <math.h>
#include <time.h>
#include <string.h>
#include <sys/time.h>

#include "main.h"
#include "Hexapod.h"

/*
 * Defines
 */
#define UPDATE_USECS 20000

int main(void)
{

  Hexapod hexapod = Hexapod();
  hexapod.Run();


  return 0;
}


