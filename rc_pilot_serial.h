#ifndef RC_PILOT_SERIAL_H_
#define RC_PILOT_SERIAL_H_


#define DRONE_ID 1
#define DELTA_ID 2

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
    uint32_t id;         ///< Unique id for the rigid body being described
    float x;               ///< x-position in the Optitrack frame
    float y;               ///< y-position in the Optitrack frame
    float z;               ///< z-position in the Optitrack frame
    float qx;              ///< qx of quaternion
    float qy;              ///< qy of quaternion
    float qz;              ///< qz of quaternion
    float qw;              ///< qw of quaternion
    int8_t trackingValid;  // (bool) of whether or not tracking was valid (0 or 1)
    int16_t state;          ///< state
    float x_d;             ///< Desired X Position
    float y_d;             ///< Desired Y Position
    float z_d;             ///< Desired Z Position
} data_packet_t;

data_packet_t delta_data_packet;
data_packet_t drone_data_packet;
data_packet_t data_packet;


int serial_init(int bus, int baudrate, float timeout);
int serial_receive(int bus);


#endif