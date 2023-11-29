/**
 * <setpoint_manager.h>
 *
 * @brief   Guidance module for the vehicle
 *
 * Setpoint manager runs at the same rate as the feedback controller
 * and is the interface between the user inputs (input manager) and
 * the feedback controller setpoint. currently it contains very
 * simply logic and runs very quickly which is why it's okay to run
 * in the feedback ISR right before the feedback controller. In the
 * future this is where go-home and other higher level autonomy will
 * live.
 *
 * This serves to allow the feedback controller to be as simple and
 * clean as possible by putting all high-level manipulation of the
 * setpoints here. Then feedback-controller only needs to march the
 * filters and zero them out when arming or enabling controllers
 *
 * @addtogroup SetpointManager
 * @{
 *
 */

#ifndef __SETPOINT_MANAGER__
#define __SETPOINT_MANAGER__

#include <rc_pilot_defs.h>
#include <stdbool.h>

 /**
  * Setpoint for the feedback controllers.
  *
  * This is written by setpoint_manager and primarily read in by fly_controller.
  * May also be read by printf_manager and log_manager for telemetry.
  */
typedef struct setpoint_t
{
    /** @name general */
    ///< @{
    int initialized;  ///< Incdicates setpoint manager initialization. 1 - Initialized, 0 - Not initialized.

    /** @name 3D Position Setpoint */
    ///< @{
    double X;
    double Y;
    double Z;
    ///< @}

    /** @name 3D Velocity Setpoint*/
    ///< @{
    double X_dot;   ///< x velocity (m/s)
    double Y_dot;   ///< y velocity (m/s)
    double Z_dot;   ///< z velocity (m/s)
    ///< @}

    /** @name Feedforward Velocity*/
    ///< @{
    double X_dot_ff;   ///< x velocity (m/s)
    double Y_dot_ff;   ///< y velocity (m/s)
    double Z_dot_ff;   ///< z velocity (m/s)
    ///< @}

    /** @name Acceleration setpoint */
    ///< @{
    double X_ddot;
    double Y_ddot;
    double Z_ddot;
    ///< @}

} setpoint_t;

extern setpoint_t setpoint;


/**
 * @brief   Initializes the setpoint manager.
 *
 * Usage: main.c, line 350
 *
 * @return  0 on success, -1 on failure
 */
int setpoint_manager_init(void);

/**
 * @brief   Updates the setpoint manager.
 *
 * Should be called before feedback loop.
 *
 * Usage: main.c, line 193
 *
 * @return  0 on success, -1 on failure
 */
int setpoint_manager_update(void);

/**
 * @brief   cleans up the setpoint manager.
 *
 *  Not really necessary but here for completeness
 *
 * Usage: main.c, line 654
 *
 * @return  0 on clean exit, -1 if exit timed out
 */
int setpoint_manager_cleanup(void);

/**
 * @brief   Update x y and z position setpoints
 *
 *  x y and z are updated from the input setpoint
 *
 * Usage: 
 *
 * @return  0 on success, -1 on failure
 */
void setpoint_update_XYZ(void);

#endif /* __SETPOINT_MANAGER__ */

/* @} end group SetpointManager */