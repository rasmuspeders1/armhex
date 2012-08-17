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
    bool setGroupPositions(uint8_t startAddr, std::vector<double> positions);

    /**
     * sends a command to turn of all servos on the controller.
     */
    void goHomeAllServos();

    /**
     * sends the get errors command to the Maestro and returns the 16 bit error code.
     */
    uint16_t GetErrors();



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
    bool set_center_offset(unsigned int address, int value);

    int openSerialPort(std::string serial_port_dev);
    int configureSerialPort();

  private:
    //ms to radians ratio
    const double DP_RATIO_;
    //angle to pulse width offset. 0 degrees is 600 hundred ms
    const double DP_OFFSET_;
    const double MAX_PULSE_WIDTH_;

    //Update interval. MUST BE BELOW ONE SECOND.
    const long UPDATE_INTERVAL_NSECS_;

    std::string serial_port_;
    int serial_port_FD_;
    struct termios serial_port_config_;
    uint8_t serial_buffer_[256];
    static bool done_;
    std::vector<int> center_offsets_;


    virtual bool Update() = 0;

    bool SendCommandInBuffer(uint16_t len);

    int ReadToBuffer(uint16_t len);

    static void sigHandler(int signo)
    {
      done_ = true;
    }

};

#endif /* MAESTRO_H_ */
