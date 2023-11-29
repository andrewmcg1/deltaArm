/**
 * <settings.h>
 * @date 2023-11-04
 * 
 * @brief Configuration settings read from the json
 * 
 * Configuration settings read from the json settings file and passed to most
 * threads as they initialize.
 * 
 * @addtogroup Settings
 * @{
 */

#ifndef SETTINGS_H
#define SETTINGS_H

#include <rc/math/filter.h>

/**
 * Configuration settings read from the json settings file and passed to most
 * threads as they initialize.
 */
typedef struct settings_t
{
    /** @name File details*/
    char name[128];

    /** @name physical parameters*/
    float base_side_length;
    float end_effector_side_length;
    float top_link_length;
    float bottom_link_length;

    char flight_controller_uart[128];

     /** @name gyroscope low pass filter time constant multiplier */
    ///@{
    double moc_lpf_timeConst_multiplier;
    ///@}

    /** @name feedback controllers */
    ///@{
    rc_filter_t horiz_vel_ctrl_pd;
    rc_filter_t horiz_vel_ctrl_i;
    rc_filter_t horiz_pos_ctrl_pd;
    rc_filter_t horiz_pos_ctrl_i;

    ///@}

} settings_t;

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