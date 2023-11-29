/**
 * <state_estimator.h>
 *
 * @brief      Navigation module for the vehicle
 *
 * This runs at the same rate as the feedback controller.
 * state_estimator_march() is called immediately before  feedback_march() in the
 * IMU interrupt service routine.
 *
 * @addtogroup StateEstimator
 * @{
 */

#ifndef __STATE_ESTIMATOR__
#define __STATE_ESTIMATOR__

#include <rc/mpu.h>
#include <rc_pilot_defs.h>
#include <stdint.h>  // for uint64_t

 /**
  * This is the output from the state estimator. It contains raw sensor values
  * and the outputs of filters. Everything is in NED coordinates defined as:
  *
  * - X pointing Forward
  * - Y pointing Right
  * - Z pointing Down
  *
  * right hand rule applies for angular values such as tait bryan angles and gyro
  * - Positive Roll to the right about X
  * - Positive Pitch back about Y
  * - Positive Yaw right about Z
  */
typedef struct state_estimate_t {
    int initialized;
    uint64_t imu_time_ns;
    uint64_t bmp_time_ns;

    /** @name selected values for feedback
     * these are copies of other values in this state estimate used for feedback
     * this is done so we can easily chose which source to get feedback from (mag or no mag)
     */
     ///@{
    double X;
    double Y;
    double Z;
    double X_dot;
    double Y_dot;
    double Z_dot;
    double Z_ddot; // transformed z accel
    ///@}

    /** @name Motion Capture data
     * As mocap drop in and out the mocap_running flag will turn on and off.
     * Old values will remain readable after mocap drops out.
     */
     ///@{
    int mocap_running;            ///< 1 if motion capture data is recent and valid
    uint64_t mocap_timestamp_ns;  ///< timestamp of last received packet in nanoseconds since boot
    uint64_t xbee_time_received_ns;  ///< timestamp of xbee message received
    double pos_mocap[3];             ///< end effector position in mocap frame, converted to NED if necessary
    double pos_mocap_lp[3];          ///< end effector position in mocap frame, converted to NED if necessary with low pass filter
    ///@}

    /** @name Other */
    ///@{
    double v_batt_raw;  ///< main battery pack voltage (v)
    double v_batt_lp;   ///< main battery pack voltage with low pass filter (v)
    double bmp_temp;    ///< temperature of barometer
    ///@}

} state_estimate_t;

extern state_estimate_t state_estimate;