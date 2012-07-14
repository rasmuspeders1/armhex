/*
 * maestro.h
 *
 *  Created on: Jul 10, 2012
 *      Author: rasmus
 */

#ifndef MAESTRO_H_
#define MAESTRO_H_

#define MAX_PULSE_WIDTH 3000
#define CMD_BUF_LEN 256
//Update interval. MUST BE BELOW ONE SECOND.
#define UPDATE_INTERVAL_NSECS 20000000

//ms per degree
#define DP_RATIO 10
//angle to pulse width offset. 0 degrees is 600 hundred ms
#define DP_OFFSET 600

#include <termios.h>
#include <vector>
#include <string>
#include <stdint.h>

class MaestroController
{
  public:
    MaestroController();
    virtual ~MaestroController();

    /**
     * sets positions for a group of servos.
     * Takes a start address and a vector of degress as floating point values.
     * returns true if the command was successfully sent, false otherwise.
     * @param startAddr
     * @param positions
     * @return result
     */
    bool setGroupPositions(uint8_t startAddr,
        std::vector<float> &positions);

    /**
     * sends a command to turn of all servos on the controller.
     */
    void goHomeAllServos();

    /**
     * Starts the Update loop. The Update Callbeck is called every UPDATE_INTERVAL_NSECS nanoseconds.
     */
    void Run();

    /**
     * Sets the center offset for a single servo in ms
     * @param address
     * @param value
     * @return result
     */
    bool setCenterOffset(unsigned int address, int value);

  private:

    std::string serialPort;
    int serialPortFD;
    struct termios serialPortConfig;
    uint8_t cmdBuf[CMD_BUF_LEN];
    bool doRun;
    std::vector<int> centerOffsets;

    int openSerialPort();
    int configureSerialPort();

    virtual bool Update() = 0;

};

#endif /* MAESTRO_H_ */
