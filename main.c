#include <stdio.h>
#include "deltaarm.h"
#include "rc_pilot_serial.h"

//#include "xbee_receive.h"

#include <rc/uart.h>

#define LOW_VOLTAGE 14.8

#define SERIAL_PORT 1
#define SERIAL_BAUD 57600
#define SERIAL_TIMEOUT_S 0.5

#define GOAL_TOLERANCE 10


int picked_up = 0;

int at_goal(point_t* location, point_t* goal)
{
    if(fabs(location->x - goal->x) < GOAL_TOLERANCE && fabs(location->y - goal->y) < GOAL_TOLERANCE && fabs(location->z - goal->z) < GOAL_TOLERANCE)
        return 0;
    else
        return -1;
}

int main()
{
    point_t delta_location, drone_location, goal_location;
    unsigned int enter_standy = 0;
    unsigned int leave_standby = 0;
    static int keep_running = 1;

    if(delta_init(&delta_location))
    {
        printf("ERROR: Delta arm failed to initialize\n");  ///< Initialize delta arm
        exit(1);
    }
    printf("Delta arm initialized\n");

    if(serial_init(SERIAL_PORT, SERIAL_BAUD, SERIAL_TIMEOUT_S))      ///< Initialize serial port
    {
        printf("ERROR: Serial failed to initialize\n");
        exit(1);
    }
    printf("Serial initialized\n");

    if(!delta_check_battery(LOW_VOLTAGE))
    {
        delta_standby();
    }
    

    while(keep_running)
    {
        if(delta_check_battery(LOW_VOLTAGE))
        {
            printf("ERROR: Delta arm battery too low\n");
            break;
        }

        if(picked_up)
        {
            delta_standby();
            break;
        }
        if(enter_standy)
        {
            delta_close_claw();
            enter_standy = 0;
        }

        if(serial_receive(SERIAL_PORT))
        {
            drone_location.x = drone_data_packet.x;
            drone_location.y = drone_data_packet.y;
            drone_location.z = drone_data_packet.z;
            switch(drone_data_packet.state)
            {
                case SM_LOITER:
                    if(!enter_standy)
                        delta_open_claw();

                    delta_location.x = delta_data_packet.x * 1000;
                    delta_location.y = delta_data_packet.y * 1000;   ///< Convert from meters to millimeters
                    delta_location.z = delta_data_packet.z * 1000;
                    delta_move(&delta_location);
                    break;
                default:
                    enter_standy = 1;
                    delta_standby();
                    break;
            }
            /*
            if(!at_goal(&delta_location, &goal_location))
            {
                delta_grab(&delta_location);
                picked_up = 1;
            }
            */
        }

        printf("\rID: %1d | State: %d | X: %.4f | Y: %.4f | Z: %.4f | ", drone_data_packet.id, drone_data_packet.state, drone_location.x, drone_location.y, drone_location.z);
        printf("ID: %1d | X: %.4f | Y: %.4f | Z: %.4f         ", delta_data_packet.id, delta_location.x, delta_location.y, delta_location.z);
        //printf(" | Bytes: %4d           ", rc_uart_bytes_available(SERIAL_PORT));

        fflush(stdout);


    }

    //rc_uart_close(SERIAL_BUS);

    return 0;
}
/*
int test_main()
{
    if(serial_init(SERIAL_BUS, SERIAL_BAUD, SERIAL_TIMEOUT_S))      ///< Initialize serial port
    {
        printf("ERROR: Serial failed to initialize\n");
        exit(1);
    }


    printf("\n");

    while(1)
    {
        serial_receive(SERIAL_BUS);

        printf("\r");

        printf("drone x: %f    ", drone_data_packet.x);
        printf("drone y: %f    ", drone_data_packet.y);
        printf("drone z: %f    ", drone_data_packet.z);

        printf("delta x: %f    ", delta_data_packet.x);
        printf("delta y: %f    ", delta_data_packet.y);
        printf("delta z: %f    ", delta_data_packet.z);

        fflush(stdout);  
    }

    return 0;
}
*/