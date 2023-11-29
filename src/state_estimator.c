/**
 * @file state_estimator.c
 *
 */

#include <math.h>
#include <stdio.h>

#include <rc/adc.h>
#include <rc/bmp.h>
#include <rc/led.h>
#include <rc/math/filter.h>
#include <rc/math/kalman.h>
#include <rc/math/matrix.h>
#include <rc/math/other.h>
#include <rc/math/quaternion.h>
#include <rc/mpu.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#include <rc_pilot_defs.h>
#include <settings.h>
#include <state_estimator.h>
#include <rc_pilot_serial.h>

#define TWO_PI (M_PI * 2.0)

state_estimate_t state_estimate;  // extern variable in state_estimator.h

// pos filter
static rc_filter_t x_lpf = RC_FILTER_INITIALIZER;
static rc_filter_t y_lpf = RC_FILTER_INITIALIZER;
static rc_filter_t z_lpf = RC_FILTER_INITIALIZER;

// battery filter
static rc_filter_t batt_lpf = RC_FILTER_INITIALIZER;

static void __batt_init(void)
{
    // init the battery low pass filter
    rc_filter_moving_average(&batt_lp, 20, DT);
    double tmp = rc_adc_dc_jack();
    if (tmp < 3.0)
    {
        tmp = settings.v_nominal;
        if (settings.warnings_en)
        {
            fprintf(stderr, "WARNING: ADC read %0.1fV on the barrel jack. Please connect\n", tmp);
            fprintf(stderr, "battery to barrel jack, assuming nominal voltage for now.\n");
        }
    }
    rc_filter_prefill_inputs(&batt_lp, tmp);
    rc_filter_prefill_outputs(&batt_lp, tmp);
    return;
}

static void __batt_march(void)
{
    double tmp = rc_adc_dc_jack();
    if (tmp < 3.0) tmp = settings.v_nominal;
    state_estimate.v_batt_raw = tmp;
    state_estimate.v_batt_lp = rc_filter_march(&batt_lp, tmp);
    return;
}

static void __batt_cleanup(void)
{
    rc_filter_free(&batt_lp);
    return;
}

static void __pos_init(void)
{
    // initialize position filters
    rc_filter_dfirst_order_lowpass(&x_lpf, DT, 300*DT);
    rc_filter_dfirst_order_lowpass(&y_lpf, DT, 300*DT);
    rc_filter_dfirst_order_lowpass(&z_lpf, DT, 300*DT);
    return;
}

static void __pos_march(void)
{
    // march position filters
    state_estimate.X = rc_filter_march(&x_lpf, data_packet.x_d);
    state_estimate.Y = rc_filter_march(&y_lpf, data_packet.y_d);
    state_estimate.Z = rc_filter_march(&z_lpf, data_packet.z_d);
    return;
}

static void __mocap_check_timeout(void)
{
    if (state_estimate.mocap_running)
    {
        uint64_t current_time = rc_nanos_since_epoch();
        // check if mocap data is > 3 steps old
        if ((current_time - state_estimate.mocap_timestamp_ns) > (3 * 1E7))
        {
            state_estimate.mocap_running = 0;
            if (settings.warnings_en)
            {
                fprintf(stderr, "WARNING, MOCAP LOST VISUAL\n");
            }
        }
    }
    return;
}

int state_estimator_init(void)
{
    __batt_init();
    __pos_init();
    state_estimate.initialized = 1;
    return 0;
}

int state_estimator_march(void)
{
    if (state_estimate.initialized == 0)
    {
        fprintf(stderr, "ERROR in state_estimator_march, estimator not initialized\n");
        return -1;
    }

    // populate state_estimate struct one setion at a time, top to bottom
    __batt_march();
    __gyro_march();
    __pos_march();
    __feedback_select();
    __mocap_check_timeout();
    return 0;
}

int state_estimator_cleanup(void)
{
    __batt_cleanup();
    __pos_cleanup();
    return 0;
}