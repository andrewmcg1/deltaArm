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
} data_packet_t;

data_packet_t data_packet;


int serial_init(int bus, int baudrate, float timeout);
int serial_receive(int bus);


#endif