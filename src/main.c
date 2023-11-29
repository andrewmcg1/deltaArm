#include <stdio.h>
#include <deltaarm.h>
#include <rc_pilot_serial.h>
#include <PID.h>

//#include "xbee_receive.h"

#include <rc/uart.h>

#define LOW_VOLTAGE 14.8

#define SERIAL_PORT 1
#define SERIAL_BAUD 57600
#define SERIAL_TIMEOUT_S 0.5

#define GOAL_TOLERANCE 10


int picked_up = 0;
/*
P_gain = 0.5;
I_gain = 0.5;
D_gain = 0.5;
*/

int at_goal(point_t* location, point_t* goal)
{
    if(fabs(location->x - goal->x) < GOAL_TOLERANCE && fabs(location->y - goal->y) < GOAL_TOLERANCE && fabs(location->z - goal->z) < GOAL_TOLERANCE)
        return 0;
    else
        return -1;
}

int main()
{
    point_t delta_location, delta_location_prev, drone_location, setpoint, offset;
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
        if(!delta_check_battery(LOW_VOLTAGE))
        {
            printf("ERROR: Delta arm battery too low\n");
            break;
        }

        if(enter_standy)
        {
            delta_close_claw();
            enter_standy = 0;
        }

        if(!serial_receive(SERIAL_PORT))
        {
            switch(drone_data_packet.state)
            {
                case SM_LOITER:
                    if(!enter_standy)
                        delta_open_claw();

                    delta_location.x = data_packet.x_d;
                    delta_location.y = data_packet.y_d;
                    delta_location.z = data_packet.z_d;
                    delta_location.time = utime_now();

/**
                    ///* TODO: goal location needs to be set to where it should go not where it is
                    setpoint.x = data_packet.x_d;
                    setpoint.y = data_packet.y_d;
                    setpoint.z = data_packet.z_d;

                    P_term_x = (setpoint.x - delta_location.x) * P_gain_x;
                    P_term_y = (setpoint.y - delta_location.y) * P_gain_y;
                    P_term_z = (setpoint.z - delta_location.z) * P_gain_z;

                    I_term_x += (setpoint.x - delta_location.x) * I_gain_x;
                    I_term_y += (setpoint.y - delta_location.y) * I_gain_y;
                    I_term_z += (setpoint.z - delta_location.z) * I_gain_z;

                    D_term_x = (delta_location.x - delta_location_prev.x) / (delta_location.time - delta_location_prev.time) * D_gain_x;
                    D_term_y = (delta_location.y - delta_location_prev.y) / (delta_location.time - delta_location_prev.time) * D_gain_y;
                    D_term_z = (delta_location.z - delta_location_prev.z) / (delta_location.time - delta_location_prev.time) * D_gain_z;

                    PID_sum_x = P_term_x + I_term_x + D_term_x;
                    PID_sum_y = P_term_y + I_term_y + D_term_y;
                    PID_sum_z = P_term_z + I_term_z + D_term_z;

                    
                    delta_location_prev.x = delta_location.x;
                    delta_location_prev.y = delta_location.y;
                    delta_location_prev.z = delta_location.z;
                    delta_location_prev.time = delta_location.time;
*/
                    
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

        printf("\rDelta Location: %3.3f | %3.3f | %3.3f | %3.3d", delta_location.x, delta_location.y, delta_location.z, delta_location.claw);
        //printf(" | Bytes: %4d           ", rc_uart_bytes_available(SERIAL_PORT));

        fflush(stdout);


    }

    rc_uart_close(SERIAL_PORT);

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