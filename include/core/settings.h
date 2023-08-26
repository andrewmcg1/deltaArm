/**
 * <settings.h>
 *
 * @brief   Functions to read the json settings file
 *
 * @addtogroup Settings
 * @{
 */

#ifndef __SETTINGS__
#define __SETTINGS__

#include <rc/math/filter.h>
#include <rc/mpu.h>

#include <flight_mode.h>
#include <input_manager.h>
#include <mix.h>
#include <rc_pilot_defs.h>
#include <thrust_map.h>

 /**
  * Configuration settings read from the json settings file and passed to most
  * threads as they initialize.
  */
typedef struct settings_t
{
    /** @name File details */
    char name[128];  ///< string declaring the name of the settings file
    ///@}

    /**@name warings */
    ///@{
    int warnings_en;
    ///@}

    //True if you want to automatically enter OPEN_LOOP_DESCENT if using a mode that
    //requires MOCAP and mocap has been unavailable for 'mocap_dropout_timeout_ms' ms
    //OPEN_LOOP_DESCENT commands 0 roll, 0 pitch, and throttle of 'dropout_z_throttle'
    //One can exit this mode by switching to a controller mode that doesn't require MOCAP
    int enable_mocap_dropout_emergency_standby;
    double mocap_dropout_timeout_ms;

    /** @name printf settings */
    ///@{
    int printf_arm;
    int printf_battery;
    int printf_position;
    int printf_setpoint;
    int printf_angles;
    int printf_mode;
    int printf_xbee;
    int printf_xbee_velocities;
    int printf_tracking;
    ///@}

    /** @name log settings */
    ///@{
    int enable_logging;
    int log_only_while_armed;
    int log_battery;
    int log_state;
    int log_xbee;
    int log_position_setpoint;
    int log_angles;
    int log_flight_mode;
    int log_benchmark;
    ///@}

    /** @name feedback controllers */
    ///@{
    rc_filter_t roll_rate_controller_pd;
    rc_filter_t roll_rate_controller_i;
    rc_filter_t pitch_rate_controller_pd;
    rc_filter_t pitch_rate_controller_i;
    rc_filter_t yaw_rate_controller_pd;
    rc_filter_t yaw_rate_controller_i;
    rc_filter_t roll_controller_pd;
    rc_filter_t roll_controller_i;
    rc_filter_t pitch_controller;
    rc_filter_t yaw_controller;
    rc_filter_t altitude_rate_controller_pd;
    rc_filter_t altitude_rate_controller_i;
    rc_filter_t altitude_controller_pd;
    rc_filter_t altitude_controller_i;
    rc_filter_t horiz_vel_ctrl_pd;
    rc_filter_t horiz_vel_ctrl_i;
    rc_filter_t horiz_pos_ctrl_pd;
    rc_filter_t horiz_pos_ctrl_i;

    ///@}

    /** @name waypoint folder name and path*/
    ///@{
    char wp_folder[200];
    ///@}

    /** @name waypoint filenames for takeoff, guided, and landing */
    ///@{
    char wp_takeoff_filename[100];
    char wp_guided_filename[100];
    char wp_landing_filename[100];
    ///@}

    /** @name serial port the Xbee is connected to */
    ///@{
    char rc_pilot_serial_port[50];
    int rc_pilot_packet_version;
    ///@}

} settings_t;

/**
 * settings are external, so just include this header and read from it
 */
extern settings_t settings;

/**
 * @brief   Populates the settings and controller structs with the settings file.
 *
 * Usage: main.c, line 287
 *
 * @return  0 on success, -1 on failure
 */
int settings_load_from_file(const char* path);

/**
 * @brief   Only used in debug mode. Prints settings to console
 *
 * Usage: settings.c, line 654
 *
 * @return  0 on success, -1 on failure
 */
int settings_print(void);

#endif /* __SETTINGS__ */

/* @} end group Settings */