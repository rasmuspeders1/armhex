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

#include "MaestroController.h"

MaestroController::MaestroController()
{
  std::cout << "Initializing MaestroController\n";

  //Initialize serialPort name to expected value
  serialPort = "/dev/ttyACM0";
  openSerialPort();
  configureSerialPort();

  //First send "Go Home" command to maestro.
  goHomeAllServos();

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

  //Set stop bit
  serialPortConfig.c_cflag &= ~CSTOPB;

  //set number of data buts
  serialPortConfig.c_cflag &= ~CSIZE;
  serialPortConfig.c_cflag |= CS8;

  //apply the settings to the port
  tcsetattr(serialPortFD, TCSAFLUSH, &serialPortConfig);

  std::cout << "Serial port configured\n";

  return serialPortFD;
}

void MaestroController::goHomeAllServos()
{
  //Write 0xA2 to maestro to turn off all servos
  char cmd = 0xA2;
  write(serialPortFD, &cmd, 1);
}

bool MaestroController::setGroupPositions(uint8_t startAddr,
    std::vector<unsigned int> positions)
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

  bool returnVal = true;
  //calculate size of command array
  unsigned int commandArraySize = 3 + positions.size() * 2;
  std::cout << commandArraySize << std::endl;
  //Create array of correct length.
  //first 3 bytes are command type, number of values and start address. The rest are values each of two bytes size.
  uint8_t *cmd = new uint8_t[commandArraySize];

  //First byte is command type
  cmd[0] = 0x9F;

  //second byte is number of targets
  cmd[1] = positions.size();

  for (unsigned int i = 0; i < positions.size(); ++i)
  {
    //the meastro operates in quarters of milliseconds.
    unsigned int targetVal;

    //Check pulse width range
    if(positions[i] > MAX_PULSE_WIDTH)
    {
      targetVal = MAX_PULSE_WIDTH * 4;
    }
    else
    {
      targetVal = positions[i] * 4;
    }

    //create two byte value for command
    cmd[i * 2 + 3] = targetVal & 0x7F;
    cmd[i * 2 + 4] = (targetVal >> 7) & 0x7F;
  }

  unsigned int writtenBytes = 0;
  ssize_t writeResult = 0;
  for (;;)
  {
    writeResult = write(serialPortFD, cmd, commandArraySize);
    std::cout << "Write result: " << writeResult << std::endl;
    if(writeResult < 0)
    {
      returnVal = false;
      std::cout << "write result is negative!" << std::endl;
      break;
    }

    writtenBytes += writeResult;
    std::cout << writtenBytes << std::endl;
    if(writtenBytes == commandArraySize)
      break;

  }

  delete[] cmd;
  return returnVal;
}

