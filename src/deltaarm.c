
#include <deltaarm.h>

#include <termios.h>            //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>             //STDIN_FILENO
#include <rc/adc.h>
#include <controller.h>

#define CONTROLLER_SPEED 5


feedback_state_t fstate;  // extern variable in delta.h

// Robot geometry (assumes identical links) 
// --> watch these globals - may want to add identifiers (e.g., "delta_f", "delta_e"...)
const double f = 130;     // base reference triangle side length
const double e = 50;     // end effector reference triangle side length
const double rf = 314.325;    // Top link length
const double re = 377.825;    // Bottom link length

// trigonometric constants
const double pi = 3.141592653;    // PI
const double sqrt3 = 1.732051;    // sqrt(3)
const double sin120 = 0.866025;   // sqrt(3)/2
const double cos120 = -0.5;        
const double tan60 = 1.732051;    // sqrt(3)
const double sin30 = 0.5;
const double tan30 = 0.57735;     // 1/sqrt(3)


// forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
// returned status: 0=OK, -1=non-existing position
int delta_calcForward(point_t* location) 
{
    double theta1 = location->theta1;
    double theta2 = location->theta2;
    double theta3 = location->theta3;

    double t = (f-e)*tan30/2;
    double dtr = pi/(double)180.0;
    
    theta1 *= dtr;
    theta2 *= dtr;
    theta3 *= dtr;
    
    double y1 = -(t + rf*cos(theta1));
    double z1 = -rf*sin(theta1);
    
    double y2 = (t + rf*cos(theta2))*sin30;
    double x2 = y2*tan60;
    double z2 = -rf*sin(theta2);
    
    double y3 = (t + rf*cos(theta3))*sin30;
    double x3 = -y3*tan60;
    double z3 = -rf*sin(theta3);
    
    double dnm = (y2-y1)*x3-(y3-y1)*x2;
    
    double w1 = y1*y1 + z1*z1;
    double w2 = x2*x2 + y2*y2 + z2*z2;
    double w3 = x3*x3 + y3*y3 + z3*z3;
        
    // x = (a1*z + b1)/dnm
    double a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
    double b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
    
    // y = (a2*z + b2)/dnm;
    double a2 = -(z2-z1)*x3+(z3-z1)*x2;
    double b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
    
    // a*z^2 + b*z + c = 0
    double a = a1*a1 + a2*a2 + dnm*dnm;
    double b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
    double c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
    
    // discriminant
    double d = b*b - (double)4.0*a*c;
    if (d < 0) return -1; // non-existing point
    
    location->z = -(double)0.5*(b+sqrt(d))/a;
    location->x = (a1*(location->z) + b1)/dnm;
    location->y = (a2*(location->z) + b2)/dnm;
    return 0;
}
    
    // inverse kinematics
    // helper functions, calculates angle theta1 (for YZ-plane)
int delta_calcAngleYZ(double x0, double y0, double z0, double *theta)
{
    double y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
    y0 -= 0.5 * 0.57735    * e;    // shift center to edge

    // z = a + b*y
    double a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
    double b = (y1-y0)/z0;

    // discriminant
    double d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
    if (d < 0) 
        return -1; // non-existing point

    double yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
    double zj = a + b*yj;
    *theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
    return 0;
}
 
// inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
// returned status: 0=OK, -1=non-existing position
int delta_calcInverse(point_t* location)
{
    location->theta1 = location->theta2 = location->theta3 = 0;
    int status = delta_calcAngleYZ(location->x, location->y, location->z, &location->theta1);
    if (status == 0) 
        status = delta_calcAngleYZ(location->x*cos120 + location->y*sin120, location->y*cos120-location->x*sin120, location->z, &location->theta2);  // rotate coords to +120 deg
    if (status == 0) 
        status = delta_calcAngleYZ(location->x*cos120 - location->y*sin120, location->y*cos120+location->x*sin120, location->z, &location->theta3);  // rotate coords to -120 deg
    return status;
}

int delta_validAngles(point_t* location)
{
    if(fabs(location->theta1-MAX_ANGLE) < MAX_ANGLE && fabs(location->theta2-MAX_ANGLE) < MAX_ANGLE && fabs(location->theta3-MAX_ANGLE) < MAX_ANGLE)
        return 0;
    else
    {
        return -1;
    }
}

void delta_update_angles(point_t* location)
{
    if(fabs(location->theta1-MAX_ANGLE) < MAX_ANGLE && fabs(location->theta2-MAX_ANGLE) < MAX_ANGLE && fabs(location->theta3-MAX_ANGLE) < MAX_ANGLE)
    {
        servo_set_angle(1, location->theta1-MAX_ANGLE);
        servo_set_angle(2, location->theta2-MAX_ANGLE);
        servo_set_angle(3, location->theta3-MAX_ANGLE);
    }
    else
        printf("Invalid Angles: %f %f %f\n", location->theta1-MAX_ANGLE, location->theta2-MAX_ANGLE, location->theta3-MAX_ANGLE);
}





int delta_open_claw()
{
    servo_set_angle(4, 68);
    return 1;
}

int delta_close_claw()
{
    servo_set_angle(4, -4);
    return 0;
}




int delta_standby(void)
{
    point_t location;
    location.x = 0;
    location.y = 0;
    location.z = -300;

    delta_calcInverse(&location);
    delta_update_angles(&location);
    delta_close_claw();

    return 0;
}

int delta_arm(void)
{
    fstate.arm_state = DISARMED;

#ifndef OFFBOARD_TEST
    // set LEDs
    rc_led_set(RC_LED_RED, 1);
    rc_led_set(RC_LED_GREEN, 0);
#endif

    return 0;
}

int delta_disarm(void)
{
    if (fstate.arm_state == ARMED)
    {
        printf("WARNING: trying to arm when controller is already armed\n");
        return -1;
    }

    if (fstate.fail_state == FAILED)
    {
        printf("WARNING: Trying to arm the controller in a fail state\n");
        return -1;
    }

    // start a new log file every time controller is armed, this may take some
    // time so do it before touching anything else
    //if (settings.enable_logging) log_manager_init();

    // get the current time
    fstate.arm_time_ns = rc_nanos_since_epoch();

    // reset the index
    fstate.loop_index = 0;

    // Reset controllers, clears integrators and prefills outputs as necessary
    controller_reset();

    // set LEDs
    rc_led_set(RC_LED_RED, 0);
    rc_led_set(RC_LED_GREEN, 1);
    // last thing is to flag as armed
    fstate.arm_state = ARMED;
    return 0;
}

int delta_init(point_t* location)
{
    controller_init();

    // initialize servos
    // should perhaps move this outside of delta init and into main
    if(servos_init((int[]) {1, 2, 3, 4}, (int[]) {1, 1, 1, 10}, 4))
        return -1;

    // make sure everything is disarmed them start the ISR
    feedback_disarm();
    fstate.fail_state = SAFE;  // Assume quad is starting out safe

    // set end effector location to standby position
    delta_standby();

    fstate.initialized = 1;
    
    return 0;
}

int delta_march(void)
{
    int i;
    double u[6], mot[8];

    // Disarm if rc_state is somehow paused without disarming the controller.
    // This shouldn't happen if other threads are working properly.
    if (rc_get_state() != RUNNING && fstate.arm_state == ARMED)
    {
        delta_disarm();
    }

    // check for a tipover
    if (fabs(state_estimate.roll) > TIP_ANGLE || fabs(state_estimate.pitch) > TIP_ANGLE)
    {
        delta_disarm();
        if(fstate.fail_state != FAILED)
            printf("\n TIPOVER DETECTED \n");
        fstate.fail_state = FAILED;
    }
    else
    {
        // If the tipover is corrected and the controller is set to disarmed, fail state is safe
        if (fstate.fail_state == FAILED && user_input.requested_arm_mode == DISARMED)
        {
            fstate.fail_state = SAFE;
        }
    }

    // if not running or not armed, keep the motors in an idle state
    if (rc_get_state() != RUNNING || fstate.arm_state == DISARMED)
    {
        delta_standby();
        //if (settings.log_only_while_armed) {
        //    log_manager_cleanup();
        //}
        return 0;
    }

    // We are about to start marching the individual SISO controllers forward.
    // Start by zeroing out the motors signals then add from there.
    for (i = 0; i < 8; i++) mot[i] = 0.0;
    for (i = 0; i < 6; i++) u[i] = 0.0;

    feedback_controller(u, mot);

    /***************************************************************************
     * Send ESC motor signals immediately at the end of the control loop
     ***************************************************************************/
    for (i = 0; i < settings.num_rotors; i++)
    {
        rc_saturate_double(&mot[i], 0.0, 1.0);
        fstate.m[i] = map_motor_signal(mot[i]);

        // final saturation just to take care of possible rounding errors
        // this should not change the values and is probably excessive
        rc_saturate_double(&fstate.m[i], 0.0, 1.0);

        // finally send pulses!
        rc_servo_send_esc_pulse_normalized(i + 1, fstate.m[i] * settings.output_modifier);
    }

    /***************************************************************************
     * Final cleanup, timing, and indexing
     ***************************************************************************/
    // Load control inputs into cstate for viewing by outside threads
    for (i = 0; i < 6; i++) fstate.u[i] = u[i];
    // keep track of loops since arming
    fstate.loop_index++;
    // log us since arming, mostly for the log
    fstate.last_step_ns = rc_nanos_since_epoch();

    return 0;
}

int delta_move(point_t* location)
{
    if(delta_calcInverse(location) || delta_validAngles(location))
    {
        //printf("\nERROR: Invalid arm location: %f, %f, %f\n", location->x, location->y, location->z);
        return -1;
    }
    if(location.claw)
        delta_close_claw();
    else
        delta_open_claw();

    delta_update_angles(location);
    return 0;
}

int delta_cleanup(void)
{
#ifndef OFFBOARD_TEST
    __send_motor_stop_pulse();
#endif
    return 0;
}



/**

static struct termios oldt, newt;

void __handler(int sig)
{
    printf("Exiting...\n");

    //restore the old settings
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);

    exit(0);
}




int delta_main () {
    int c;
    point_t location;

    int stat;

    signal(SIGINT, __handler);
    
    servos_init((int[]){1, 2, 3, 4}, (int[]){1, 1, 1, 10}, 4);
    rc_adc_init();
    

    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);          
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);

    location.x = 0;
    location.y = 0;
    location.z = -300;
    location.claw = 0;
    delta_calcInverse(&location);
    delta_update_angles(&location);

    printf("Ready\n");

    while(!delta_check_battery(14.8))
    {  
        c = getchar();
        switch(c) 
        {
            case 's':
                location.x += CONTROLLER_SPEED;
                if(delta_calcInverse(&location) || delta_validAngles(&location))
                {
                    location.x -= CONTROLLER_SPEED;
                    continue;
                }
                break;
            case 'w':
                location.x -= CONTROLLER_SPEED;
                if(delta_calcInverse(&location) || delta_validAngles(&location))
                {
                    location.x += CONTROLLER_SPEED;
                    continue;
                }
                break;
            case 'a':
                location.y += CONTROLLER_SPEED;
                if(delta_calcInverse(&location) || delta_validAngles(&location))
                {
                    location.y -= CONTROLLER_SPEED;
                    continue;
                }
                break;
            case 'd':
                location.y -= CONTROLLER_SPEED;
                if(delta_calcInverse(&location) || delta_validAngles(&location))
                {
                    location.y += CONTROLLER_SPEED;
                    continue;
                }
                break;
            case 'r':
                location.z += CONTROLLER_SPEED;
                if(delta_calcInverse(&location) || delta_validAngles(&location))
                {
                    location.z -= CONTROLLER_SPEED;
                    continue;
                }
                break;
            case 'f':
                location.z -= CONTROLLER_SPEED;
                if(delta_calcInverse(&location) || delta_validAngles(&location))
                {
                    location.z += CONTROLLER_SPEED;
                    continue;
                }
                break;
            case 'q':
                if(!location.claw)
                {
                    location.claw = delta_open_claw();
                    continue;
                }
                break;
            case 'e':
                if(location.claw)
                {
                    location.claw = delta_close_claw();
                    continue;
                }
                break;
        }
        printf("\tx: %lf y: %lf z: %lf claw: %d\n", location.x, location.y, location.z, location.claw);

        stat = delta_calcInverse(&location);
        if (stat) {
            printf("NO!\n");
        }
        else
        {
            delta_update_angles(&location);
        }  
    }



    return 0;
}
*/