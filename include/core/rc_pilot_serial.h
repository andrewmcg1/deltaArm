#ifndef RC_PILOT_SERIAL_H_
#define RC_PILOT_SERIAL_H_

#include <delta_defs.h>

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
} data_packet_t;

data_packet_t data_packet;


int serial_init(int bus, int baudrate, float timeout);
int serial_receive(int bus);


#endif