#include <math.h>
#include <stdio.h>
#include <inttypes.h>

#include <controller.h>
#include <rc/math.h>
#include <rc/time.h>
#include <settings.h>


// filters
static rc_filter_t D_Xdot_pd = RC_FILTER_INITIALIZER;
static rc_filter_t D_Xdot_i =  RC_FILTER_INITIALIZER;
static rc_filter_t D_Ydot_pd = RC_FILTER_INITIALIZER;
static rc_filter_t D_Ydot_i =  RC_FILTER_INITIALIZER;
static rc_filter_t D_Zdot_pd = RC_FILTER_INITIALIZER;
static rc_filter_t D_Zdot_i =  RC_FILTER_INITIALIZER;
static rc_filter_t D_X_pd =    RC_FILTER_INITIALIZER;
static rc_filter_t D_X_i =     RC_FILTER_INITIALIZER;
static rc_filter_t D_Y_pd =    RC_FILTER_INITIALIZER;
static rc_filter_t D_Y_i =     RC_FILTER_INITIALIZER;
static rc_filter_t D_Z_pd =    RC_FILTER_INITIALIZER;
static rc_filter_t D_Z_i =     RC_FILTER_INITIALIZER;

static void __xyz_init(void)
{
    rc_filter_duplicate(&D_Xdot_pd, settings.horiz_vel_ctrl_pd);
    rc_filter_duplicate(&D_Xdot_i,  settings.horiz_vel_ctrl_i);

    rc_filter_duplicate(&D_Ydot_pd, settings.horiz_vel_ctrl_pd);
    rc_filter_duplicate(&D_Ydot_i,  settings.horiz_vel_ctrl_i);

    rc_filter_duplicate(&D_Zdot_pd, settings.altitude_rate_controller_pd);
    rc_filter_duplicate(&D_Zdot_i,  settings.altitude_rate_controller_i);
   
    rc_filter_duplicate(&D_X_pd, settings.horiz_pos_ctrl_pd);
    rc_filter_duplicate(&D_X_i, settings.horiz_pos_ctrl_i);

    rc_filter_duplicate(&D_Y_pd, settings.horiz_pos_ctrl_pd);
    rc_filter_duplicate(&D_Y_i, settings.horiz_pos_ctrl_i);

    rc_filter_duplicate(&D_Z_pd, settings.altitude_controller_pd);
    rc_filter_duplicate(&D_Z_i,  settings.altitude_controller_i);
}

static void __zero_out_feedforward_terms()
{
    setpoint.roll_dot_ff = 0;
    setpoint.pitch_dot_ff = 0;
    setpoint.yaw_dot_ff = 0;
    setpoint.roll_ff = 0;
    setpoint.pitch_ff = 0;
    setpoint.Z_dot_ff = 0;
    setpoint.X_dot_ff = 0; 
    setpoint.Y_dot_ff = 0; 
}

static void __assign_setpoints_and_enable_loops()
{
    // Zero out feedforward terms so unexpected things don't happen
    __zero_out_feedforward_terms();

    // 1) Enable PID Loops based on flight mode
    setpoint.en_Z_ctrl = 1;
    setpoint.en_XY_ctrl = 1;

    // 2) Assign Setpoints
    //TODO: Add in setpoint assignment

    setpoint.en_rpy_rate_ctrl = 0;
    setpoint.en_rpy_ctrl = 0;
    setpoint.en_Z_ctrl = 0;
    setpoint.en_XY_ctrl = 0;
    setpoint.Z_throttle = 0;
    setpoint.roll_throttle = 0;
    setpoint.pitch_throttle = 0;

}

#ifndef OFFBOARD_TEST
static void __run_Z_controller()
{
    // 1) Position -> Velocity
    setpoint.Z_dot = rc_filter_march(&D_Z_pd, setpoint.Z - delta_location.Z)
                   + rc_filter_march(&D_Z_i,  setpoint.Z - delta_location.Z)
                   + setpoint.Z_dot_ff;
    rc_saturate_double(&setpoint.Z_dot, -MAX_Z_VELOCITY, MAX_Z_VELOCITY);

    
    if (user_input.flight_mode != TEST_BENCH_DIRECT_Z_ACC)
    {
        // 2) Velocity -> Acceleration
        setpoint.Z_ddot = rc_filter_march(&D_Zdot_pd, setpoint.Z_dot - state_estimate.Z_dot)
                        + rc_filter_march(&D_Zdot_i,  setpoint.Z_dot - state_estimate.Z_dot);
        rc_saturate_double(&setpoint.Z_ddot, -MAX_Z_ACCELERATION, MAX_Z_ACCELERATION);
    }

    // 3) Acceleration -> Throttle
    setpoint.Z_throttle = settings.hover_throttle + setpoint.Z_ddot;
    setpoint.Z_throttle = setpoint.Z_throttle / (cos(state_estimate.roll) * cos(state_estimate.pitch));
}

static void __run_XY_controller()
{
    // 1) Position -> Velocity
    setpoint.X_dot = rc_filter_march(&D_X_pd, setpoint.X - delta_location.X)
                   + rc_filter_march(&D_X_i, setpoint.X - delta_location.X)
                   + setpoint.X_dot_ff;
    setpoint.Y_dot = rc_filter_march(&D_Y_pd, setpoint.Y - delta_location.Y)
                   + rc_filter_march(&D_Y_i, setpoint.Y - delta_location.Y)
                   + setpoint.Y_dot_ff;
    rc_saturate_double(&setpoint.X_dot, -MAX_XY_VELOCITY, MAX_XY_VELOCITY);
    rc_saturate_double(&setpoint.Y_dot, -MAX_XY_VELOCITY, MAX_XY_VELOCITY);

    // 2) Velocity -> Acceleration
    setpoint.X_ddot = rc_filter_march(&D_Xdot_pd, setpoint.X_dot - state_estimate.X_dot)
                    + rc_filter_march(&D_Xdot_i,  setpoint.X_dot - state_estimate.X_dot);
    setpoint.Y_ddot = rc_filter_march(&D_Ydot_pd, setpoint.Y_dot - state_estimate.Y_dot)
                    + rc_filter_march(&D_Ydot_i,  setpoint.Y_dot - state_estimate.Y_dot);
    rc_saturate_double(&setpoint.X_ddot, -MAX_XY_ACCELERATION, MAX_XY_ACCELERATION);
    rc_saturate_double(&setpoint.Y_ddot, -MAX_XY_ACCELERATION, MAX_XY_ACCELERATION);
    
    // 3) Acceleration -> Lean Angles
    setpoint.roll = ((-sin(state_estimate.continuous_yaw) * setpoint.X_ddot 
                      +cos(state_estimate.continuous_yaw) * setpoint.Y_ddot)
                      / GRAVITY)
                    + setpoint.roll_ff;              
    setpoint.pitch = ((-cos(state_estimate.continuous_yaw) * setpoint.X_ddot 
                       -sin(state_estimate.continuous_yaw) * setpoint.Y_ddot)
                      / GRAVITY)
                    + setpoint.pitch_ff;
    rc_saturate_double(&setpoint.roll, -MAX_ROLL_SETPOINT, MAX_ROLL_SETPOINT);
    rc_saturate_double(&setpoint.pitch, -MAX_PITCH_SETPOINT, MAX_PITCH_SETPOINT);
}

static void __run_controller(double* u, double* mot)
{
    // 1) Z Controller
    if (setpoint.en_Z_ctrl) __run_Z_controller();

    // 2) XY Controller
    if (setpoint.en_XY_ctrl) __run_XY_controller();
}
#endif /* OFFBOARD_TEST */


///////////////////////////////////////////////////////////////////////////////
//                                                                           //
//               Functions Called from Outside "Controller"                  //
//                                                                           //
///////////////////////////////////////////////////////////////////////////////


void feedback_controller(double* u, double* mot)
{
    __assign_setpoints_and_enable_loops();

#ifndef OFFBOARD_TEST
    __run_controller(u, mot);
#else
    u = mot;
    mot = u;
#endif
}

int controller_init()
{
    __xyz_init();

    return 0;
}

int controller_reset()
{
    rc_filter_reset(&D_Xdot_pd);
    rc_filter_reset(&D_Xdot_i);
    rc_filter_reset(&D_Ydot_pd);
    rc_filter_reset(&D_Ydot_i);
    rc_filter_reset(&D_Zdot_pd);
    rc_filter_reset(&D_Zdot_i);

    rc_filter_reset(&D_X_pd);
    rc_filter_reset(&D_X_i);
    rc_filter_reset(&D_Y_pd);
    rc_filter_reset(&D_Y_i);
    rc_filter_reset(&D_Z_pd);
    rc_filter_reset(&D_Z_i);

    // prefill filters with zero error (only those with D terms)
    rc_filter_prefill_inputs(&D_roll_rate_pd, 0);
    rc_filter_prefill_inputs(&D_pitch_rate_pd, 0);
    rc_filter_prefill_inputs(&D_yaw_rate_pd, 0);

    rc_filter_prefill_inputs(&D_Xdot_pd, 0);
    rc_filter_prefill_inputs(&D_Ydot_pd, 0);
    rc_filter_prefill_inputs(&D_Zdot_pd, 0);

    return 0;
}

