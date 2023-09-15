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

#include <delta_defs.h>
#include <setpoint_manager.h>
#include <settings.h>
#include <controller.h>
#include <state_estimator.h>



#define XYZ_MAX_ERROR 0.05   ///< meters.
#define MAX_X_SETPOINT  0.5 ///< meters.
#define MIN_X_SETPOINT -0.5 ///< meters.
#define MAX_Y_SETPOINT  0.5 ///< meters.
#define MIN_Y_SETPOINT -0.5 ///< meters.
#define MAX_Z_SETPOINT  0   ///< meters.
#define MIN_Z_SETPOINT -2.0 ///< meters.

setpoint_t setpoint;  // extern variable in setpoint_manager.h

/**
 * Function only used locally
 */
/***********************************/

/**
 * @brief   Logic for starting to follow path, reset time and waypoint counter
 */
static void __start_waypoint_counter();
static void __stop_waypoint_counter();
static void __reset_waypoint_counter();

/***********************************/

void setpoint_update_Z(void)
{
    // Set Z-dot based on user input
    // setpoint.Z_dot = -user_input.thr_stick * settings.max_Z_velocity;
    double thr_stick_full_range = 2*user_input.thr_stick - 1;
    setpoint.Z_dot_ff = -1 * deadzone(thr_stick_full_range,2*LOITER_DEADZONE) * MAX_Z_VELOCITY;
    // setpoint.Z_dot = -1 * (2*user_input.thr_stick - 1) * settings.max_Z_velocity;

    // Constrain user input on setpoint changes for Z
    if (setpoint.Z > (state_estimate.Z + XYZ_MAX_ERROR))
    {
        setpoint.Z_dot_ff = fmin(0.0, setpoint.Z_dot_ff);
    }
    else if (setpoint.Z < (state_estimate.Z - XYZ_MAX_ERROR))
    {
        setpoint.Z_dot_ff = fmax(0.0, setpoint.Z_dot_ff);
    }
    setpoint.Z += setpoint.Z_dot_ff * DT;

    // Constrain Z setpoint
    rc_saturate_double(&setpoint.Z, MIN_Z_SETPOINT, MAX_Z_SETPOINT);

    return;
}

void setpoint_update_XY_pos(void)
{
    // Constrain user input on setpoint changes for X
    if (setpoint.X > (state_estimate.X + XYZ_MAX_ERROR))
    {
        setpoint.X_dot_ff = fmin(0.0, setpoint.X_dot_ff);
    }
    else if (setpoint.X < (state_estimate.X - XYZ_MAX_ERROR))
    {
        setpoint.X_dot_ff = fmax(0.0, setpoint.X_dot_ff);
    }
    setpoint.X += setpoint.X_dot_ff * DT;

    // Constrain user input on setpoint changes for Y
    if (setpoint.Y > (state_estimate.Y + XYZ_MAX_ERROR))
    {
        setpoint.Y_dot_ff = fmin(0.0, setpoint.Y_dot_ff);
    }
    else if (setpoint.Y < (state_estimate.Y - XYZ_MAX_ERROR))
    {
        setpoint.Y_dot_ff = fmax(0.0, setpoint.Y_dot_ff);
    }
    setpoint.Y += setpoint.Y_dot_ff * DT;


    // Lastly, Constrain X & Y setpoints
    rc_saturate_double(&setpoint.X, MIN_X_SETPOINT, MAX_X_SETPOINT);
    rc_saturate_double(&setpoint.Y, MIN_Y_SETPOINT, MAX_Y_SETPOINT);

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

    return 0;
}

int setpoint_manager_cleanup(void)
{
    setpoint.initialized = 0;
    return 0;
}

int set_new_path(const char* file_name)
{
    if (path_load_from_file(file_name) == -1)
    {
        fprintf(stderr, "ERROR: could not load new path file named %s\n", file_name);
        return -1;
    }

    __reset_waypoint_counter();
    return -1;
}

static void __start_waypoint_counter()
{
    // If this is the first time in autonomous mode and armed, save the current time
    if (setpoint.auto_armed_set == 0)
    {
        // If the system is armed and autonomous mode is set, record time in
        // time_auto_set
        setpoint.auto_armed_set = 1;
        setpoint.time_auto_set = rc_nanos_since_epoch();
    }

    setpoint.waypoint_time = rc_nanos_since_epoch() - setpoint.time_auto_set;
}

static void __stop_waypoint_counter()
{
    // If the system is disarmed or out of auto reset the auto_armed_set flag
    // and change the current waytpoint to zero
    setpoint.auto_armed_set = 0;
    setpoint.cur_waypoint_num = 0;
}

static void __reset_waypoint_counter()
{
    __stop_waypoint_counter();
    __start_waypoint_counter();
}

static bool __waypoint_has_nans(waypoint_t* wp)
{
    return isnan(wp->x) ||
           isnan(wp->y) ||
           isnan(wp->z) ||
           isnan(wp->xd) ||
           isnan(wp->yd) ||
           isnan(wp->zd) ||
           isnan(wp->roll) ||
           isnan(wp->pitch) ||
           isnan(wp->yaw) ||
           isnan(wp->p) ||
           isnan(wp->q) ||
           isnan(wp->r);
}

void setpoint_update_XYZ_bumpless() 
{
    setpoint.X = state_estimate.X;
    setpoint.Y = state_estimate.Y;
    setpoint.Z = state_estimate.Z;
    setpoint.yaw = state_estimate.continuous_yaw;

    // Set Feed-Forward roll and pitch angles (xy controller will add to it later) 
    setpoint.roll_ff = 0;
    setpoint.pitch_ff = 0;
}

void setpoint_update_loiter()
{
    setpoint.X_dot_ff = -deadzone(user_input.pitch_stick,LOITER_DEADZONE) * MAX_XY_VELOCITY;
    setpoint.Y_dot_ff =  deadzone(user_input.roll_stick,LOITER_DEADZONE)  * MAX_XY_VELOCITY;
    setpoint_update_XY_pos();
    setpoint_update_Z();
    setpoint_update_yaw();
}

bool just_transitioned_flight_mode()
{
    // Condition #1: Just transitioned
    bool just_transitioned = (user_input.prev_flight_mode != user_input.flight_mode);

    // Condition #2: Just armed the system
    bool just_armed = (fstate.loop_index == 0);

    return just_transitioned || just_armed;
}
