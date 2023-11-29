/**
 * @file setpoint_manager.c
 *
 *
 **/
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>  // for memset
#include <inttypes.h>

#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/math.h>

#include <feedback.h>
#include <flight_mode.h>
#include <input_manager.h>
#include <path.h>
#include <rc_pilot_defs.h>
#include <setpoint_manager.h>
#include <settings.h>
#include <state_estimator.h>


#include <controller.h>
#include <rc/math.h>
#include <settings.h>
#include <state_machine.h>



#define XYZ_MAX_ERROR 0.5   ///< meters.
#define MAX_X_SETPOINT  2.0 ///< meters.
#define MIN_X_SETPOINT -2.0 ///< meters.
#define MAX_Y_SETPOINT  1 ///< meters.
#define MIN_Y_SETPOINT -1 ///< meters.
#define MAX_Z_SETPOINT  0   ///< meters.
#define MIN_Z_SETPOINT -2.0 ///< meters.

setpoint_t setpoint;  // extern variable in setpoint_manager.h

void setpoint_update_XYZ_setpoint(void)
{

    return;
}

int setpoint_manager_init(void)
{
    if (setpoint.initialized)
    {
        fprintf(stderr, "ERROR in setpoint_manager_init, already initialized\n");
        return -1;
    }
    memset(&setpoint, 0, sizeof(setpoint_t));
    setpoint.initialized = 1;
    setpoint.following_rsp_cmd = false;
    return 0;
}

int setpoint_manager_update(void)
{
    // Filter variables to track previous roll and pitch setpoints
    // static double roll_prev = 0;
    // static double pitch_prev = 0;

    if (setpoint.initialized == 0)
    {
        fprintf(stderr, "ERROR in setpoint_manager_update, not initialized yet\n");
        return -1;
    }

// Beaglebone dependent code, not runnable while offboard testing
#ifndef OFFBOARD_TEST
    if (user_input.initialized == 0)
    {
        fprintf(stderr, "ERROR in setpoint_manager_update, input_manager not initialized yet\n");
        return -1;
    }

    // If PAUSED or UNINITIALIZED, do nothing
    if (rc_get_state() != RUNNING) return 0;
#endif
/*
    // Shutdown feedback on kill switch
    if (user_input.requested_arm_mode == DISARMED)
    {
        if (fstate.arm_state == ARMED) feedback_disarm();
        return 0;
    }
    // Arm when requested
    else if (user_input.requested_arm_mode == ARMED)
    {
        if (fstate.arm_state == DISARMED) feedback_arm();
    }

    // Logic for managing time and resetting the current waypoint for autonomous mode
    // TODO: migrate to state matchine?
    if ( ((user_input.flight_mode == AUTONOMOUS) || (user_input.flight_mode == LOITER_RSP)) && fstate.arm_state == ARMED)
    {
        __start_waypoint_counter();
    }
    else
    {
        __stop_waypoint_counter();
    }
*/
    return 0;
}

int setpoint_manager_cleanup(void)
{
    setpoint.initialized = 0;
    return 0;
}