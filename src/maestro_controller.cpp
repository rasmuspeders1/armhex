/*
 * maestro.cpp
 *
 *  Created on: Jul 10, 2012
 *      Author: rasmus
 */

#include <stdio.h>
#include <iostream>
#include <ios>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <time.h>
#include <algorithm>

#include "MaestroController.h"

MaestroController::MaestroController()
{
  std::cout << "Initializing MaestroController\n";

  //Initialize members
  doRun = false;

  //Initialize serialPort name to expected value
  serialPort = "/dev/ttyACM0";

  //Initialize centerOffset Vector to zeros
  centerOffsets.assign(24, 0);

  //Open and configure serial port
  openSerialPort();
  configureSerialPort();

  //First send "Go Home" command to maestro.
  goHomeAllServos();

}

MaestroController::~MaestroController()
{

}

void MaestroController::Run()
{
  //POSIX.1b structure for a time value. Like struct timeval.
  timespec beginTs;
  timespec endTs;
  timespec deltaTs;

  timespec sleepTs;
  sleepTs.tv_sec = 0;
  doRun = true;
  while (doRun)
  {
    clock_gettime(CLOCK_REALTIME, &beginTs);
    //Call Update method in child class
    //Child class should do anything it wants to do with the controller in this callback and return true.
    //Child class can return false to indicate and error condition and the loop will end and all servos will be "sent home".
    if(!Update())
    {
      std::cout << "Update call from Maestro controller returned false!"
          << std::endl;

      goHomeAllServos();
      break;
    }

    clock_gettime(CLOCK_REALTIME, &endTs);
    //Calculate sleep time
    deltaTs.tv_nsec = endTs.tv_nsec - beginTs.tv_nsec;
    if(deltaTs.tv_nsec < 0)
    {
      deltaTs.tv_nsec += 1000000000;
      deltaTs.tv_sec = endTs.tv_sec - beginTs.tv_sec - 1;
    }
    else
    {
      deltaTs.tv_sec = endTs.tv_sec - beginTs.tv_sec;
    }

    if(deltaTs.tv_sec == 0 && UPDATE_INTERVAL_NSECS > deltaTs.tv_nsec)
    {
      sleepTs.tv_nsec = UPDATE_INTERVAL_NSECS - deltaTs.tv_nsec;
      while (nanosleep(&sleepTs, &sleepTs) == -1)
        ;
    }
  }
}

int MaestroController::openSerialPort()
{
  //open serial port
  serialPortFD = open(serialPort.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  if(serialPortFD == -1)
  {
    std::cout << "Failed to open serial port <" << serialPort << ">\n";
  }
  else
  {
    std::cout << "Serial port opened\n";
    if(!isatty(serialPortFD))
    {
      std::cerr << "Serial port <" << serialPort << "> is not a TTY!\n";
      serialPortFD = -1;
    }
  }

  return serialPortFD;
}

int MaestroController::configureSerialPort()
{
  //Set Baud rates
  cfsetispeed(&serialPortConfig, B115200);
  cfsetospeed(&serialPortConfig, B115200);

  //Set no parity,
  serialPortConfig.c_cflag &= ~PARENB;

  //Set one stop bit
  serialPortConfig.c_cflag &= ~CSTOPB;

  //set number of data buts
  serialPortConfig.c_cflag &= ~CSIZE;
  serialPortConfig.c_cflag |= CS8;

  //apply the settings to the port
  tcsetattr(serialPortFD, TCSANOW, &serialPortConfig);

  std::cout << "Serial port configured\n";

  return serialPortFD;
}

void MaestroController::goHomeAllServos()
{
  //Write 0xA2 to maestro to turn off all servos
  cmdBuf[0] = 0xA2;
  write(serialPortFD, cmdBuf, 1);
}

bool MaestroController::setGroupPositions(uint8_t startAddr,
    std::vector<float> &positions)
{
  //Start address cannot be larger than 23 on maestro24
  if(startAddr > 23)
    return false;

  //Number of positions cannot be larger than 24 - start address on maestro24
  if(positions.size() > (24 - startAddr))
    return false;

  //positions vector cannot be empty
  if(positions.empty())
    return false;

  //calculate size of command array
  unsigned int commandArraySize = 3 + positions.size() * 2;

  //First byte is command type
  cmdBuf[0] = 0x9F;

  //second byte is number of targets
  cmdBuf[1] = positions.size();

  cmdBuf[2] = startAddr;

  for (unsigned int i = 0; i < positions.size(); ++i)
  {
    //the meastro operates in quarters of milliseconds.
    unsigned int targetVal = (centerOffsets[startAddr + i]
        + positions[i] * DP_RATIO + DP_OFFSET);

    //Check pulse width range
    if(targetVal > MAX_PULSE_WIDTH)
    {
      targetVal = MAX_PULSE_WIDTH * 4;
    }
    else
    {
      targetVal = targetVal * 4;
    }

    //create two byte value for command
    cmdBuf[3 + i * 2] = targetVal & 0x7F;
    cmdBuf[4 + i * 2] = (targetVal >> 7) & 0x7F;
  }

  unsigned int writtenBytes = 0;
  ssize_t writeResult = 0;
  while(writeResult >= 0 && writtenBytes < commandArraySize)
  {
    writeResult = write(serialPortFD, cmdBuf, commandArraySize);

    writtenBytes += writeResult;

  }
  if(writeResult < 0)
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool MaestroController::setCenterOffset(unsigned int address, int value)
{
  if(address > 23)
    return false;
  if(value > MAX_PULSE_WIDTH / 2 || value < -MAX_PULSE_WIDTH / 2)
    return false;

  centerOffsets[address] = value;

  return true;
}
