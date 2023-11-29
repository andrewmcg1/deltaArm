/**
 * <feedback.h>
 *
 * @brief   Control module for the vehicle
 *
 * Here lies the heart and soul of the operation. feedback_init(void) pulls
 * in the control constants from settings module and sets up the discrete
 * controllers. From then on out, feedback_march(void) should be called by the
 * IMU interrupt at feedback_hz until the program is shut down.
 * feedback_march(void) will monitor the setpoint which is constantly being
 * changed by setpoint_manager(void).
 *
 * @addtogroup DeltaArm
 * @{
 */

#ifndef _DELTA_ARM_H
#define _DELTA_ARM_H


#include <stdio.h>
#include <math.h>
#include <servo.h>

/**
 * @brief   Condition of each of the feedback loop's states
 *
 * Reported by the feedback controller. Should only be written to by the
 * feedback controller after initialization.
 * This is the state of the feedback loop. contains most recent values
 */
typedef struct feedback_state_t
{
    int initialized;          ///< set to 1 after feedback_init(void)
    arm_state_t arm_state;    ///< actual arm state as reported by feedback controller
    fail_state_t fail_state;  ///< fail state as reported by the feedback controller
    uint64_t arm_time_ns;     ///< time since boot when controller was armed
    uint64_t loop_index;      ///< increases every time feedback loop runs
    uint64_t last_step_ns;    ///< last time controller has finished a step

    double u[4];  ///< siso controller outputs
    double m[4];  ///< signals sent to motors after mapping
} feedback_state_t;
extern feedback_state_t fstate;

/**
 * @brief   Point in 3D space
 *
 * Stores X Y and Z values of a point in 3D space, 
 * as well as the angles of each arm to reach that point
 */
typedef struct {
    double x;               ///< x position
    double y;               ///< y position
    double z;               ///< z position

    double theta1;          ///< angle of arm 1
    double theta2;          ///< angle of arm 2
    double theta3;          ///< angle of arm 3

    int claw;               ///< claw state (open or closed)
    float time;             
} point_t;


/**
 * @brief   Initializes the feedback controller.
 *
 * Usage: main.c, line ----
 *
 * @return  0 on success, -1 on failure
 */
int delta_init();

/**
 * @brief   Calculates forward kinematics of delta arm. Inputes are the three theta values
 *
 * Usage: ----, line ----
 *
 * @return  0 on success, -1 on non-existanting position
 */
int delta_calcForward(point_t* location);

/**
 * @brief   Helper function that calculates the angle of the servo for a given arm
 *
 * Usage: ----, line ----
 *
 * @return  0 on success, -1 on non-existing point
 */
int delta_calcAngleYZ(double x0, double y0, double z0, double *theta);

/**
 * @brief   Calculates inverse kinematics of delta arm. Inputs are three position values
 *
 * Usage: ----, line ----
 *
 * @return  0 on success, -1 on non-existing position
 */
int delta_calcInverse(point_t* location);

/**
 * @brief   helper function to Update the current angles of the delta arm
 *
 * Usage: ----, line ----
 *
 * @return  0 on success, -1 on non-existing position
 */
void delta_update_angles(point_t* location);

/**
 * @brief   Updates setpoint of delta arm to enter the standby position
 *
 * Usage: ----, line ----
 *
 * @return  0
 */
int delta_standby();

/**
 * @brief   Updates setpoint of delta arm to open the claw
 *
 * Usage: ----, line ----
 *
 * @return  1
 */
int delta_open_claw();

/**
 * @brief   Updates setpoint of delta arm to close the claw
 *
 * Usage: ----, line ----
 *
 * @return  1
 */
int delta_close_claw();

/**
 * @brief   NOT IMPLEMENTED:
 *  Updates setpoint of delta arm to grab an object
 *
 * Usage: ----, line ----
 *
 * @return  1
 */
int delta_grab(point_t* location);

/**
 * @brief   Calculates the angles for a given point and moves the delta arm to that point if the position is valid
 *
 * Usage: ----, line ----
 *
 * @return  1
 */
int delta_move(point_t* location);

/**
 * @brief   DEPRECATED:
 *  Checks barrel jack input voltage
 *
 * Usage: ----, line ----
 *
 * @return  1
 */
int delta_check_battery(float voltage);

/**
 * @brief   DEPRECATED:
 *  Test main function
 *
 * Usage: ----, line ----
 *
 * @return  1
 */
int delta_main();


#endif