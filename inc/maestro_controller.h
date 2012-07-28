/*
 * maestro.h
 *
 *  Created on: Jul 10, 2012
 *      Author: rasmus
 */

#ifndef MAESTRO_H_
#define MAESTRO_H_

#include <termios.h>
#include <vector>
#include <string>
#include <stdint.h>
#include <signal.h>
#include <math.h>

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
    bool setGroupPositions(uint8_t startAddr, std::vector<float> positions);

    /**
     * sends a command to turn of all servos on the controller.
     */
    void goHomeAllServos();

    /**
     * sends the get errors command to the Maestro and returns the 16 bit error code.
     */
    uint16_t get_errors();

    bool send_command_in_buffer(uint16_t len);

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
    //ms to radians ratio
    const float DP_RATIO_;
    //angle to pulse width offset. 0 degrees is 600 hundred ms
    const float DP_OFFSET_;
    const float MAX_PULSE_WIDTH_;

    //Update interval. MUST BE BELOW ONE SECOND.
    const float UPDATE_INTERVAL_NSECS_;

    std::string serialPort;
    int serialPortFD;
    struct termios serialPortConfig;
    uint8_t cmdBuf[256];
    static bool done;
    std::vector<int> centerOffsets;

    int openSerialPort();
    int configureSerialPort();

    virtual bool Update() = 0;

    static void sigHandler(int signo)
    {
      done = true;
    }

};

#endif /* MAESTRO_H_ */
