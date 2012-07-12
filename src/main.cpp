/*
 * main.cpp
 *
 *  Created on: Jul 9, 2012
 *      Author: rasmus
 */

#include <signal.h>
#include <stdio.h>
#include <cstdlib>
#include <math.h>
#include <time.h>
#include <exception>

#include "main.h"
#include "Hexapod.h"


void SigIntHandler(int s){
           printf("Caught interrupt signal %d\n",s);
           exit(0);
}


int main(void)
{
//  /*
//   * Create signal handler to intercept Ctrl-C event and close nicely
//   */
//  struct sigaction sigIntHandler;
//
//  sigIntHandler.sa_handler = SigIntHandler;
//  sigemptyset(&sigIntHandler.sa_mask);
//  sigIntHandler.sa_flags = 0;
//
//  sigaction(SIGINT, &sigIntHandler, NULL);

  /*
   * Run Hexapod
   */
  Hexapod hexapod = Hexapod();

  hexapod.Run();

  return 0;
}
