/**
 * <rc_pilot_serial.h>
 * 
 * @date 29-11-2023
 * @author Andrew McGrellis (andrewmcg@vt.edu)
 * 
 * @brief Serial communication module for the delta arm
 * 
 * The delta arm functions on a seperate beaglebone than the quadcopter's flight controller
 * so in order to use one xbee modem, the delta arm and quadcopter must communicate over serial.
 * This module is responsible for reading packets from the serial port and parsing them into
 * current and desired locations for the delta arm.
 * 
 * @addtogroup rcPilotSerial
 * @{
 */

#ifndef RC_PILOT_SERIAL_H_
#define RC_PILOT_SERIAL_H_

/**
 * @brief List of possible states for the state machine. States can be added as needed for new
 * functionality.
 */
typedef enum sm_states
{
    STANDBY = 0,
    TAKEOFF = 1,
    GUIDED = 2,
    LANDING = 3,
    SM_LOITER = 4,
    NAILING = 5,
    RETURN = 6,
} sm_states;

/**
 * @brief List of all data that is sent to over serial from the flight controller.
 * 
 */
typedef struct __attribute__((__packed__))
{
    int8_t state;          ///< state
    int8_t claw;           ///< claw open or close (1 close, 0 open)
    float x_d;             ///< Desired X Position
    float y_d;             ///< Desired Y Position
    float z_d;             ///< Desired Z Position
} data_packet_t;           ///< Data packet sent over serial

/**
 * @brief Initializes the serial port
 * 
 * Usage: ----, line ----
 * 
 * @return 0 on success, -1 on failure
 */
int serial_init(int bus, int baudrate, float timeout);

/**
 * @brief Checks if serial port has data available, and if so, reads the data packet in
 * 
 * @return 0 on read, -1 on failure
 
 */
int serial_receive(int bus);


#endif