/*
 * maestro.h
 *
 *  Created on: Jul 10, 2012
 *      Author: rasmus
 */

#ifndef MAESTRO_H_
#define MAESTRO_H_

#define MAX_PULSE_WIDTH 3000

#include <termios.h>
#include <vector>
#include <string>
#include <stdint.h>




class MaestroController
{
  public:
    MaestroController();
    virtual ~MaestroController(){};

    /**
     * sets positions for a group of servos
     */
    bool setGroupPositions(uint8_t startAddr,
        std::vector<unsigned int> positions);

    /**
     * sends a command to turn of all servos on the controller.
     */
    void goHomeAllServos();

    virtual bool UpdatePositions() = 0;

  private:

    std::string serialPort;
    int serialPortFD;
    struct termios serialPortConfig;

    int openSerialPort();
    int configureSerialPort();





};

#endif /* MAESTRO_H_ */
