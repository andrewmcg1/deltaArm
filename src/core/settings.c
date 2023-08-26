/**
 * @file settings.c
 *
 * contains all the functions for io to the settings file, including default
 * values that can be written to disk if no file is present.
 **/

#include <fcntl.h>  // for F_OK
#include <stdio.h>
#include <string.h>  // FOR str_cmp()
#include <unistd.h>  // for access()

#include <json-c/json.h>
#include <rc/math/filter.h>

#include <rc_pilot_defs.h>
#include <settings.h>

 // json object respresentation of the whole settings file
static json_object* jobj;

// primary settings struct declared as extern in header is defined ONCE here
settings_t settings;

// if anything goes wrong set this flag back to 0
static int was_load_successful = 0;

////////////////////////////////////////////////////////////////////////////////
/// MACROS FOR PARSING JSON TYPES
////////////////////////////////////////////////////////////////////////////////

// macro for reading a boolean
#define PARSE_BOOL(name)                                                                 \
    if (json_object_object_get_ex(jobj, #name, &tmp) == 0)                               \
    {                                                                                    \
        fprintf(stderr, "ERROR parsing settings file, can't find " #name "\n");          \
        return -1;                                                                       \
    }                                                                                    \
    if (json_object_is_type(tmp, json_type_boolean) == 0)                                \
    {                                                                                    \
        fprintf(stderr, "ERROR parsing settings file, " #name " should be a boolean\n"); \
        return -1;                                                                       \
    }                                                                                    \
    settings.name = json_object_get_boolean(tmp);

// macro for reading an integer
#define PARSE_INT(name)                                                               \
    if (json_object_object_get_ex(jobj, #name, &tmp) == 0)                            \
    {                                                                                 \
        fprintf(stderr, "ERROR parsing settings file, can't find " #name "\n");       \
        return -1;                                                                    \
    }                                                                                 \
    if (json_object_is_type(tmp, json_type_int) == 0)                                 \
    {                                                                                 \
        fprintf(stderr, "ERROR parsing settings file, " #name " should be an int\n"); \
        return -1;                                                                    \
    }                                                                                 \
    settings.name = json_object_get_int(tmp);

// macro for reading a bound integer
#define PARSE_INT_MIN_MAX(name, min, max)                                                          \
    if (json_object_object_get_ex(jobj, #name, &tmp) == 0)                                         \
    {                                                                                              \
        fprintf(stderr, "ERROR parsing settings file, can't find " #name "\n");                    \
        return -1;                                                                                 \
    }                                                                                              \
    if (json_object_is_type(tmp, json_type_int) == 0)                                              \
    {                                                                                              \
        fprintf(stderr, "ERROR parsing settings file, " #name " should be an int\n");              \
        return -1;                                                                                 \
    }                                                                                              \
    settings.name = json_object_get_int(tmp);                                                      \
    if (settings.name < min || settings.name > max)                                                \
    {                                                                                              \
        fprintf(stderr, "ERROR parsing settings file, " #name " should be between min and max\n"); \
        return -1;                                                                                 \
    }

// macro for reading a polarity which should be +-1
#define PARSE_POLARITY(name)                                                           \
    if (json_object_object_get_ex(jobj, #name, &tmp) == 0)                             \
    {                                                                                  \
        fprintf(stderr, "ERROR parsing settings file, can't find " #name "\n");        \
        return -1;                                                                     \
    }                                                                                  \
    if (json_object_is_type(tmp, json_type_int) == 0)                                  \
    {                                                                                  \
        fprintf(stderr, "ERROR parsing settings file, " #name " should be an int\n");  \
        return -1;                                                                     \
    }                                                                                  \
    settings.name = json_object_get_int(tmp);                                          \
    if (settings.name != -1 && settings.name != 1)                                     \
    {                                                                                  \
        fprintf(stderr, "ERROR parsing settings file, " #name " should be -1 or 1\n"); \
        return -1;                                                                     \
    }

// macro for reading a floating point number
#define PARSE_DOUBLE_MIN_MAX(name, min, max)                                \
    if (json_object_object_get_ex(jobj, #name, &tmp) == 0)                  \
    {                                                                       \
        fprintf(stderr, "ERROR can't find " #name " in settings file\n");   \
        return -1;                                                          \
    }                                                                       \
    if (json_object_is_type(tmp, json_type_double) == 0)                    \
    {                                                                       \
        fprintf(stderr, "ERROR " #name " should be a double\n");            \
        return -1;                                                          \
    }                                                                       \
    settings.name = json_object_get_double(tmp);                            \
    if (settings.name < min || settings.name > max)                         \
    {                                                                       \
        fprintf(stderr, "ERROR " #name " should be between min and max\n"); \
        return -1;                                                          \
    }

// macro for reading a string
#define PARSE_STRING(name)                                                              \
    if (json_object_object_get_ex(jobj, #name, &tmp) == 0)                              \
    {                                                                                   \
        fprintf(stderr, "ERROR parsing settings file, can't find " #name "\n");         \
        return -1;                                                                      \
    }                                                                                   \
    if (json_object_is_type(tmp, json_type_string) == 0)                                \
    {                                                                                   \
        fprintf(stderr, "ERROR parsing settings file, " #name " should be a string\n"); \
        return -1;                                                                      \
    }                                                                                   \
    strcpy(settings.name, json_object_get_string(tmp));

// macro for reading feedback controller
#define PARSE_CONTROLLER(name)                                             \
    if (json_object_object_get_ex(jobj, #name, &tmp) == 0)                 \
    {                                                                      \
        fprintf(stderr, "ERROR: can't find " #name " in settings file\n"); \
        return -1;                                                         \
    }                                                                      \
    if (__parse_controller(tmp, &settings.name) == -1)                           \
    {                                                                      \
        fprintf(stderr, "ERROR: could not parse " #name "\n");             \
        return -1;                                                         \
    }

////////////////////////////////////////////////////////////////////////////////
/// functions for parsing enums
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      pulls rotor layout out of json object into settings struct
 *
 * @return     0 on success, -1 on failure
 */
static int __parse_layout(void)
{
    struct json_object* tmp = NULL;
    char* tmp_str = NULL;
    if (json_object_object_get_ex(jobj, "layout", &tmp) == 0)
    {
        fprintf(stderr, "ERROR: can't find layout in settings file\n");
        return -1;
    }
    if (json_object_is_type(tmp, json_type_string) == 0)
    {
        fprintf(stderr, "ERROR: layout should be a string\n");
        return -1;
    }
    tmp_str = (char*)json_object_get_string(tmp);
    if (strcmp(tmp_str, "LAYOUT_6DOF_ROTORBITS") == 0)
    {
        settings.num_rotors = 6;
        settings.layout = LAYOUT_6DOF_ROTORBITS;
        settings.dof = 6;
    }
    else if (strcmp(tmp_str, "LAYOUT_4X") == 0)
    {
        settings.num_rotors = 4;
        settings.layout = LAYOUT_4X;
        settings.dof = 4;
    }
    else if (strcmp(tmp_str, "LAYOUT_4X2") == 0)
    {
        settings.num_rotors = 4;
        settings.layout = LAYOUT_4X2;
        settings.dof = 4;
    }
    else if (strcmp(tmp_str, "LAYOUT_4PLUS") == 0)
    {
        settings.num_rotors = 4;
        settings.layout = LAYOUT_4PLUS;
        settings.dof = 4;
    }
    else if (strcmp(tmp_str, "LAYOUT_6X") == 0)
    {
        settings.num_rotors = 6;
        settings.layout = LAYOUT_6X;
        settings.dof = 4;
    }
    else if (strcmp(tmp_str, "LAYOUT_6X2") == 0)
    {
        settings.num_rotors = 6;
        settings.layout = LAYOUT_6X2;
        settings.dof = 4;
    }
    else if (strcmp(tmp_str, "LAYOUT_8X") == 0)
    {
        settings.num_rotors = 8;
        settings.layout = LAYOUT_8X;
        settings.dof = 4;
    }
    else
    {
        fprintf(stderr, "ERROR: invalid layout string\n");
        return -1;
    }
    return 0;
}

/**
 * @ brief     parses a json_object and sets up a new controller
 *
 * @param      jobj         The jobj to parse
 * @param      filter       pointer to write the new filter to
 *
 * @return     0 on success, -1 on failure
 */
static int __parse_controller(json_object* jobj_ctl, rc_filter_t* filter)
{
    struct json_object* array = NULL;  // to hold num & den arrays
    struct json_object* tmp = NULL;    // temp object
    char* tmp_str = NULL;
    double tmp_flt, tmp_kp, tmp_ki, tmp_kd, tmp_imax;
    int i, num_len, den_len;
    rc_vector_t num_vec = RC_VECTOR_INITIALIZER;
    rc_vector_t den_vec = RC_VECTOR_INITIALIZER;

    // destroy old memory in case the order changes
    rc_filter_free(filter);

    // check if PID gains or transfer function coefficients
    if (json_object_object_get_ex(jobj_ctl, "TF_or_PID", &tmp) == 0)
    {
        fprintf(stderr, "ERROR: can't find TF_or_PID in settings file\n");
        return -1;
    }
    if (json_object_is_type(tmp, json_type_string) == 0)
    {
        fprintf(stderr, "ERROR: TF_or_PID should be a string\n");
        return -1;
    }
    tmp_str = (char*)json_object_get_string(tmp);

    if (strcmp(tmp_str, "TF") == 0)
    {
        // pull out gain
        if (json_object_object_get_ex(jobj_ctl, "gain", &tmp) == 0)
        {
            fprintf(stderr, "ERROR: can't find controller gain in settings file\n");
            return -1;
        }
        if (json_object_is_type(tmp, json_type_double) == 0)
        {
            fprintf(stderr, "ERROR: controller gain should be a double\n");
            return -1;
        }
        tmp_flt = json_object_get_double(tmp);

        // pull out numerator
        if (json_object_object_get_ex(jobj_ctl, "numerator", &array) == 0)
        {
            fprintf(stderr, "ERROR: can't find controller numerator in settings file\n");
            return -1;
        }
        if (json_object_is_type(array, json_type_array) == 0)
        {
            fprintf(stderr, "ERROR: controller numerator should be an array\n");
            return -1;
        }
        num_len = json_object_array_length(array);
        if (num_len < 1)
        {
            fprintf(stderr, "ERROR, numerator must have at least 1 entry\n");
            return -1;
        }
        rc_vector_alloc(&num_vec, num_len);
        for (i = 0; i < num_len; i++)
        {
            tmp = json_object_array_get_idx(array, i);
            if (json_object_is_type(tmp, json_type_double) == 0)
            {
                fprintf(stderr, "ERROR: numerator array entries should be a doubles\n");
                return -1;
            }
            tmp_flt = json_object_get_double(tmp);
            num_vec.d[i] = tmp_flt;
        }

        // pull out denominator
        if (json_object_object_get_ex(jobj_ctl, "denominator", &array) == 0)
        {
            fprintf(stderr, "ERROR: can't find controller denominator in settings file\n");
            return -1;
        }
        if (json_object_is_type(array, json_type_array) == 0)
        {
            fprintf(stderr, "ERROR: controller denominator should be an array\n");
            return -1;
        }
        den_len = json_object_array_length(array);
        if (den_len < 1)
        {
            fprintf(stderr, "ERROR, denominator must have at least 1 entry\n");
            return -1;
        }
        rc_vector_alloc(&den_vec, den_len);
        for (i = 0; i < den_len; i++)
        {
            tmp = json_object_array_get_idx(array, i);
            if (json_object_is_type(tmp, json_type_double) == 0)
            {
                fprintf(stderr, "ERROR: denominator array entries should be a doubles\n");
                return -1;
            }
            tmp_flt = json_object_get_double(tmp);
            den_vec.d[i] = tmp_flt;
        }

        // check for improper TF
        if (num_len > den_len)
        {
            fprintf(stderr, "ERROR: improper transfer function\n");
            rc_vector_free(&num_vec);
            rc_vector_free(&den_vec);
            return -1;
        }

        // check CT continuous time or DT discrete time
        if (json_object_object_get_ex(jobj_ctl, "CT_or_DT", &tmp) == 0)
        {
            fprintf(stderr, "ERROR: can't find CT_or_DT in settings file\n");
            return -1;
        }
        if (json_object_is_type(tmp, json_type_string) == 0)
        {
            fprintf(stderr, "ERROR: CT_or_DT should be a string\n");
            return -1;
        }
        tmp_str = (char*)json_object_get_string(tmp);

        // if CT, use tustin's approx to get to DT
        if (strcmp(tmp_str, "CT") == 0)
        {
            // get the crossover frequency
            if (json_object_object_get_ex(jobj_ctl, "crossover_freq_rad_per_sec", &tmp) == 0)
            {
                fprintf(stderr, "ERROR: can't find crossover frequency in settings file\n");
                return -1;
            }
            if (json_object_is_type(tmp, json_type_double) == 0)
            {
                fprintf(stderr, "ERROR: crossover frequency should be a double\n");
                return -1;
            }
            tmp_flt = json_object_get_double(tmp);
            if (rc_filter_c2d_tustin(filter, DT, num_vec, den_vec, tmp_flt))
            {
                fprintf(stderr, "ERROR: failed to c2dtustin while parsing json\n");
                return -1;
            }
        }

        // if DT, much easier, just construct filter
        else if (strcmp(tmp_str, "DT") == 0)
        {
            if (rc_filter_alloc(filter, num_vec, den_vec, DT))
            {
                fprintf(stderr, "ERROR: failed to alloc filter in __parse_controller()");
                return -1;
            }
        }

        // wrong value for CT_or_DT
        else
        {
            fprintf(stderr, "ERROR: CT_or_DT must be 'CT' or 'DT'\n");
            printf("instead got :%s\n", tmp_str);
            return -1;
        }
    }

    else if (strcmp(tmp_str, "P") == 0)
    {
        // pull out gains
        if (json_object_object_get_ex(jobj_ctl, "kp", &tmp) == 0)
        {
            fprintf(stderr, "ERROR: can't find kp in settings file\n");
            return -1;
        }
        tmp_kp = json_object_get_double(tmp);
        tmp_ki = 0.0;
        tmp_kd = 0.0;

        // Not used in rc_filter_pid for pure P, but (1/tmp_flt) must be >2*DT
        tmp_flt = 62.83;

        if (rc_filter_pid(filter, tmp_kp, tmp_ki, tmp_kd, 1.0 / tmp_flt, DT))
        {
            fprintf(stderr, "ERROR: failed to alloc pid filter in __parse_controller()");
            return -1;
        }
    }

    else if (strcmp(tmp_str, "PD") == 0)
    {
        // pull out gains
        if (json_object_object_get_ex(jobj_ctl, "kp", &tmp) == 0)
        {
            fprintf(stderr, "ERROR: can't find kp in settings file\n");
            return -1;
        }
        tmp_kp = json_object_get_double(tmp);
        tmp_ki = 0.0;
        if (json_object_object_get_ex(jobj_ctl, "kd", &tmp) == 0)
        {
            fprintf(stderr, "ERROR: can't find ki in settings file\n");
            return -1;
        }
        tmp_kd = json_object_get_double(tmp);
        // get the crossover frequency
        if (json_object_object_get_ex(jobj_ctl, "crossover_freq_rad_per_sec", &tmp) == 0)
        {
            fprintf(stderr, "ERROR: can't find crossover frequency in settings file\n");
            return -1;
        }
        if (json_object_is_type(tmp, json_type_double) == 0)
        {
            fprintf(stderr, "ERROR: crossover frequency should be a double\n");
            return -1;
        }
        tmp_flt = json_object_get_double(tmp);
        if (rc_filter_pid(filter, tmp_kp, tmp_ki, tmp_kd, 1.0 / tmp_flt, DT))
        {
            fprintf(stderr, "ERROR: failed to alloc pid filter in __parse_controller()");
            return -1;
        }
    }

    else if (strcmp(tmp_str, "I") == 0)
    {
        if (json_object_object_get_ex(jobj_ctl, "ki", &tmp) == 0)
        {
            fprintf(stderr, "ERROR: can't find ki in settings file\n");
            return -1;
        }
        tmp_ki = json_object_get_double(tmp);

        if (json_object_object_get_ex(jobj_ctl, "imax", &tmp) == 0)
        {
            fprintf(stderr, "ERROR: can't find imax in settings file\n");
            return -1;
        }
        tmp_imax = json_object_get_double(tmp);

        if (rc_filter_integrator(filter, DT))
        {
            fprintf(stderr, "ERROR: failed to alloc i filter in __parse_controller()");
            return -1;
        }

        rc_filter_enable_saturation(filter, -tmp_imax, tmp_imax);
        filter->gain = tmp_ki;
    }

#ifdef DEBUG
    rc_filter_print(*filter);
#endif

    rc_vector_free(&num_vec);
    rc_vector_free(&den_vec);

    return 0;
}

int settings_load_from_file(const char* path)
{
    struct json_object* tmp = NULL;  // temp object

    was_load_successful = 0;

#ifdef DEBUG
    fprintf(stderr, "beginning of load_settings_from_file\n");
    fprintf(stderr, "about to check access of fly settings file\n");
#endif

    // read in file contents
    if (access(path, F_OK) != 0)
    {
        fprintf(stderr, "ERROR: settings file missing\n");
        return -1;
    }
    else
    {
#ifdef DEBUG
        printf("about to read json from file\n");
#endif
        jobj = json_object_from_file(path);
        if (jobj == NULL)
        {
            fprintf(stderr, "ERROR, failed to read settings from disk\n");
            return -1;
        }
    }

#ifdef DEBUG
    settings_print();
#endif

    // START PARSING

    PARSE_STRING(name)
#ifdef DEBUG
        fprintf(stderr, "name: %s\n", settings.name);
#endif
    PARSE_BOOL(warnings_en)
#ifdef DEBUG
        fprintf(stderr, "warnings: %d\n", settings.warnings_en);
#endif
    // EMERGENCY LANDING SETTINGS
    PARSE_BOOL(enable_mocap_dropout_emergency_standby)
    PARSE_DOUBLE_MIN_MAX(mocap_dropout_timeout_ms, 0, 10000)
    if (settings.enable_mocap_dropout_emergency_land)
    {
        printf("Mocap dropout emergency landing ENABLED.\tDropout timeout: %0.1lfms.\tThrottle: %0.3lf\n",
            settings.mocap_dropout_timeout_ms, settings.dropout_z_throttle);
    }
#ifdef DEBUG
    fprintf(stderr, "enable_mocap_dropout_emergency_land: %d\n", settings.enable_mocap_dropout_emergency_land);
    fprintf(stderr, "mocap_dropout_timeout_ms: %lf\n", settings.mocap_dropout_timeout_ms);
#endif
    printf("---\n"); // Just a visual break between above settings and the ones below

    // PRINTF OPTIONS
    PARSE_BOOL(printf_arm)
    PARSE_BOOL(printf_battery)
    PARSE_BOOL(printf_position)
    PARSE_BOOL(printf_setpoint)
    PARSE_BOOL(printf_angles)
    PARSE_BOOL(printf_mode)
    PARSE_BOOL(printf_xbee)
    PARSE_BOOL(printf_xbee_velocities)
    PARSE_BOOL(printf_tracking)


    // LOGGING
    PARSE_BOOL(enable_logging)
    PARSE_BOOL(log_only_while_armed)
    PARSE_BOOL(log_battery)
    PARSE_BOOL(log_state)
    PARSE_BOOL(log_xbee)
    PARSE_BOOL(log_position_setpoint)
    PARSE_BOOL(log_angles)
    PARSE_BOOL(log_flight_mode)
    PARSE_BOOL(log_benchmark)

    // WAYPOINT FILES
    PARSE_BOOL(enable_wp_linear_interpolation)
    PARSE_STRING(wp_folder)
    PARSE_STRING(wp_takeoff_filename)
    PARSE_STRING(wp_guided_filename)
    PARSE_STRING(wp_landing_filename)

    // XBEE SERIAL PORT
    PARSE_STRING(rc_pilot_serial_port)
    PARSE_INT(rc_pilot_packet_version)

    // FEEDBACK CONTROLLERS
    PARSE_CONTROLLER(roll_rate_controller_pd)
    PARSE_CONTROLLER(roll_rate_controller_i)
    PARSE_CONTROLLER(pitch_rate_controller_pd)
    PARSE_CONTROLLER(pitch_rate_controller_i)
    PARSE_CONTROLLER(yaw_rate_controller_pd)
    PARSE_CONTROLLER(yaw_rate_controller_i)
    PARSE_CONTROLLER(roll_controller_pd)
    PARSE_CONTROLLER(roll_controller_i)
    PARSE_CONTROLLER(pitch_controller)
    PARSE_CONTROLLER(yaw_controller)
    PARSE_CONTROLLER(altitude_rate_controller_pd)
    PARSE_CONTROLLER(altitude_rate_controller_i)
    PARSE_CONTROLLER(altitude_controller_pd)
    PARSE_CONTROLLER(altitude_controller_i)
    PARSE_CONTROLLER(horiz_vel_ctrl_pd)
    PARSE_CONTROLLER(horiz_vel_ctrl_i)
    PARSE_CONTROLLER(horiz_pos_ctrl_pd)
    PARSE_CONTROLLER(horiz_pos_ctrl_i)

    json_object_put(jobj);  // free memory
    was_load_successful = 1;
    return 0;
}

int settings_print(void)
{
    if (jobj == NULL)
    {
        fprintf(stderr, "ERROR: NULL object passed to settings_print\n");
        return -1;
    }
    printf("settings:\n\n");
    printf("%s",
        json_object_to_json_string_ext(jobj, JSON_C_TO_STRING_SPACED | JSON_C_TO_STRING_PRETTY));
    printf("\n");
    return 0;
}
