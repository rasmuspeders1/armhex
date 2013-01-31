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
#include <sys/ioctl.h>
#include <unistd.h>

#include "maestro_controller.h"

bool MaestroController::done_ = true;

MaestroController::MaestroController() :
    DP_RATIO_(1800.0 / M_PI),
    DP_OFFSET_(600),
    MAX_PULSE_WIDTH_(3000),
    UPDATE_INTERVAL_NSECS_(20000000),
    serial_port_(""),
    serial_port_FD_(-1)
{
  std::cout << "Initializing MaestroController\n";

  //Initialize centerOffset Vector to zeros
  center_offsets_.assign(24, 0);

  struct sigaction sigAct;
  sigemptyset(&sigAct.sa_mask);
  sigAct.sa_flags = 0;
  sigAct.sa_handler = MaestroController::sigHandler;
  sigaction(SIGQUIT, &sigAct, 0);
  sigaction(SIGINT, &sigAct, 0);
  sigaction(SIGTERM, &sigAct, 0);

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
  done_ = false;

  std::cout << "\nMaestro Controller starting" << std::endl;
  while (!done_)
  {
    clock_gettime(CLOCK_REALTIME, &beginTs);
    //Call Update method in child class
    //Child class should do anything it wants to do with the controller in this callback and return true.
    //Child class can return false to indicate and error condition and the loop will end and all servos will be "sent home".
    if(!Update())
    {
      std::cout << "Update call from Maestro controller returned false!" << std::endl;
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

    if(deltaTs.tv_sec == 0 && UPDATE_INTERVAL_NSECS_ > deltaTs.tv_nsec)
    {

      sleepTs.tv_nsec = UPDATE_INTERVAL_NSECS_ - deltaTs.tv_nsec;
      while (nanosleep(&sleepTs, &sleepTs) == -1)
        ;
    }
    else
    {
      printf("Real-time constraint violation: %f ms\n", deltaTs.tv_sec * (10^3) + (deltaTs.tv_nsec - UPDATE_INTERVAL_NSECS_)/1000000.0);
    }
  }
  std::cout << "Setting all servos off" << std::endl;
  goHomeAllServos();
  std::cout << "Closing Serial Port" << std::endl;
  close(serial_port_FD_);

  std::cout << "Maestro Controller stopping" << std::endl;
}

int MaestroController::openSerialPort(std::string serial_port_dev)
{
  //open serial port
  serial_port_ = serial_port_dev;
  serial_port_FD_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  if(serial_port_FD_ == -1)
  {
    std::cout << "Failed to open serial port <" << serial_port_ << ">\n";
  }
  else
  {
    std::cout << "Serial port opened\n";

    fcntl(serial_port_FD_, F_SETFL, FNDELAY); // Non-blocking read

    if(!isatty(serial_port_FD_))
    {
      std::cerr << "Serial port <" << serial_port_ << "> is not a TTY!\n";
      serial_port_FD_ = -1;
    }
  }

  return serial_port_FD_;
}

int MaestroController::configureSerialPort()
{

  // Get the current options for the port

  tcgetattr(serial_port_FD_, &serial_port_config_);
  struct termios original = serial_port_config_;

  //Set Baud rates
  cfsetispeed(&serial_port_config_, B57600);
  cfsetospeed(&serial_port_config_, B57600);

  //No parity bit
  serial_port_config_.c_cflag &= ~PARENB;
  serial_port_config_.c_cflag |= PARODD;

  // Set character length
  serial_port_config_.c_cflag &= ~CSIZE;       // Mask the character size bits
  serial_port_config_.c_cflag |= CS8;

  // One stop bit
  serial_port_config_.c_cflag &= ~CSTOPB;

  // No hardware flow control

  serial_port_config_.c_cflag &= ~CRTSCTS;

  // Raw input, no echo

  serial_port_config_.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | IEXTEN | ISIG);
  serial_port_config_.c_iflag &=  ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                        | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY | INPCK );

  // Raw output
  serial_port_config_.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                      ONOCR | OFILL | OLCUC | OPOST);

  // Wait timeout for each character
  serial_port_config_.c_cc[VMIN] = 1;
  serial_port_config_.c_cc[VTIME] = 0;


  //Enable receiver and ser local mode
  serial_port_config_.c_cflag |= (CLOCAL | CREAD);

  //Flush buffered chars
  tcflush(serial_port_FD_, TCIOFLUSH);

  //apply the settings to the port
  tcsetattr(serial_port_FD_, TCSANOW, &serial_port_config_);

  std::cout << "Serial port configured\n";

  return serial_port_FD_;
}

bool MaestroController::SendCommandInBuffer(uint16_t len)
{
  unsigned int writtenBytes = 0;
  ssize_t writeResult = 0;
  while (writeResult >= 0 && writtenBytes < len)
  {
    writeResult = write(serial_port_FD_, serial_buffer_, len);
    writtenBytes += writeResult;
  }
  if(writeResult < 0)
  {
    std::cerr << "Serial port write error. write returned: " << writeResult << std::endl;
    return false;
  }
  else
  {
    return true;
  }
}

int MaestroController::ReadToBuffer(uint16_t len)
{
  int bytes_read = 0;
  int last_bytes_read = 0;
  int count = 0;
  while(bytes_read < len)
  {
    bytes_read = read(serial_port_FD_, serial_buffer_, len-bytes_read);
    //std::cout << "bytes read: " << bytes_read << std::endl;
    if(bytes_read == last_bytes_read || bytes_read == -1)
    {
      if(count++ > 100)
      {
        std::cout << "Never got the expected " << len <<  " bytes!" << std::endl;
        break;
      }
    }
    else
    {
      last_bytes_read = bytes_read;
    }
    usleep(100);
  }
  //std::cout << "Loops in receiver " << count << std::endl;
  return bytes_read;
}

uint16_t MaestroController::GetErrors()
{
  //Write 0xA2 to maestro to turn off all servos
  serial_buffer_[0] = 0xAA;
  serial_buffer_[1] = 12;
  serial_buffer_[2] = 0x21;
  SendCommandInBuffer(3);

  int bytes_read = ReadToBuffer(2);

  if(bytes_read != 2)
  {
    std::cout << "Expected 2 but got " << bytes_read << " bytes when reading error code from serial port!" << std::endl;
    return 0xffff;
  }
  else
  {
    /*
     if(error_code != 0)
     {
     std::cout << "Got error <" << std::hex << std::setw(4) << std::setfill('0') << error_code << ">" << std::endl;
     }
     */
    return static_cast<uint16_t>(*serial_buffer_);
  }
}

void MaestroController::goHomeAllServos()
{
  //Write 0xA2 to maestro to turn off all servos
  serial_buffer_[0] = 0xA2;
  SendCommandInBuffer(1);
}

bool MaestroController::setGroupPositions(uint8_t startAddr,
    std::vector<double> positions)
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
  //unsigned int commandArraySize = 3 + positions.size() * 2;
  unsigned int commandArraySize = 5 + positions.size() * 2;

  //First byte is command type
  //cmdBuf[0] = 0x9F;

  //second byte is number of targets
  //cmdBuf[1] = positions.size();

  //cmdBuf[2] = startAddr;

  //First byte is command type
  serial_buffer_[0] = 0xAA;

  //second byte is pololu device number
  serial_buffer_[1] = 12;

  //third is 0x1F for set multiple targets
  serial_buffer_[2] = 0x1F;

  serial_buffer_[3] = positions.size();

  serial_buffer_[4] = startAddr;

  for (unsigned int i = 0; i < positions.size(); ++i)
  {
    //the meastro operates in quarters of milliseconds.
    unsigned int target_val = (center_offsets_[startAddr + i] + positions[i] * DP_RATIO_ + DP_OFFSET_) * 4.0;

    //Check pulse width range
    if(target_val > MAX_PULSE_WIDTH_ * 4)
    {
      target_val = MAX_PULSE_WIDTH_ * 4;
    }

    //create two byte value for command
//    cmdBuf[3 + i * 2] = targetVal & 0x7F;
//    cmdBuf[4 + i * 2] = (targetVal >> 7) & 0x7F;
    serial_buffer_[5 + i * 2] = target_val & 0x7F;
    serial_buffer_[6 + i * 2] = (target_val >> 7) & 0x7F;
  }

  return SendCommandInBuffer(commandArraySize);
}

bool MaestroController::set_center_offset(unsigned int address, int value)
{
  if(address > 23)
    return false;
  if(value > MAX_PULSE_WIDTH_ || value < -MAX_PULSE_WIDTH_)
    return false;

  center_offsets_[address] = value;

  return true;
}
