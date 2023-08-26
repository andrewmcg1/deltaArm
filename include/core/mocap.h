/**
 * @file mocap.h
 * @brief 
 * @date 2023-08-26
 * 
 */

#include <rc/math/filter.h>
#include <rc/math/kalman.h>

#include <settings.h>

typedef struct state_estimate_t {
    int initialized;
    uint64_t imu_time_ns;
    uint64_t bmp_time_ns;

    /** @name selected values for feedback
     * these are copoies of other values in this state estimate used for feedback
     * this is done so we can easily chose which source to get feedback from (mag or no mag)
     */
     ///@{
    double roll;
    double pitch;
    double yaw;
    double roll_dot;
    double pitch_dot;
    double yaw_dot;
    double continuous_yaw;  ///<  keeps increasing/decreasing aboce +-2pi
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
    double pos_mocap[3];             ///< position in mocap frame, converted to NED if necessary
    double quat_mocap[4];            ///< UAV orientation according to mocap
    double tb_mocap[3];              ///< Tait-Bryan angles according to mocap
    int is_active;                   ///< TODO used by mavlink manager, purpose unclear... (pg)
    ///@}

/** @name Global Position Estimate
 * This is the global estimated position, velocity, and acceleration
 * output of a kalman filter which filters accelerometer, DMP attitude,
 * and mocap data. If mocap is not available, barometer will be used.
 *
 * global values are in the mocap's frame for position control.
 * relative values are in a frame who's origin is at the position where
 * the feedback controller is armed. Without mocap data the filter will
 * assume altitude from the barometer and accelerometer, and position
 * estimate will have steady state gain of zero to prevent runaway.
 */
 ///@{
    double pos_global[3];
    double vel_global[3];
    double accel_global[3];
    double pos_relative[3];
    double vel_relative[3];
    double accel_relative[3];
    ///@}

    /** @name Other */
    ///@{
    double v_batt_raw;  ///< main battery pack voltage (v)
    double v_batt_lp;   ///< main battery pack voltage with low pass filter (v)
    ///@}

} state_estimate_t;