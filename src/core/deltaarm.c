
#include "deltaarm.h"

#include <termios.h>            //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>             //STDIN_FILENO
#include <rc/adc.h>

#define CONTROLLER_SPEED 5


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

    double t = (BASE_SIDE_LEN - END_EFF_SIDE_LEN) * tan30 / 2;
    double dtr = pi / (double)180.0;
    
    theta1 *= dtr;
    theta2 *= dtr;
    theta3 *= dtr;
    
    double y1 = -(t + TOP_LINK_LEN * cos(theta1));
    double z1 = -TOP_LINK_LEN * sin(theta1);
    
    double y2 = (t + TOP_LINK_LEN * cos(theta2))*sin30;
    double x2 = y2 * tan60;
    double z2 = -TOP_LINK_LEN * sin(theta2);
    
    double y3 = (t + TOP_LINK_LEN * cos(theta3))*sin30;
    double x3 = -y3 * tan60;
    double z3 = -TOP_LINK_LEN * sin(theta3);
    
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
    double c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - BOT_LINK_LEN*BOT_LINK_LEN);
    
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
    double y1 = -0.5 * 0.57735 * BASE_SIDE_LEN; // BASE_SIDE_LEN/2 * tg 30
    y0 -= 0.5 * 0.57735 * END_EFF_SIDE_LEN;    // shift center to edge

    // z = a + b*y
    double a = (x0*x0 + y0*y0 + z0*z0 + TOP_LINK_LEN*TOP_LINK_LEN - BOT_LINK_LEN*BOT_LINK_LEN - y1*y1)/(2*z0);
    double b = (y1-y0)/z0;

    // discriminant
    double d = -(a+b*y1)*(a+b*y1)+ TOP_LINK_LEN *(b*b * TOP_LINK_LEN + TOP_LINK_LEN ); 
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

int delta_check_battery(float low_voltage)
{
    return (rc_adc_dc_jack() > low_voltage) ? 1 : 0;
}

int delta_init(point_t* location)
{
    if(rc_adc_init())
    {
        printf("ERROR: Failed to run enable ADC\n");
        return -1;
    }
    if(servos_init((int[]) {1, 2, 3, 4}, (int[]) {1, 1, 1, 10}, 4))
        return -1;

    location->x = 0;
    location->y = 0;
    location->z = -300;
    location->claw = 0;
    delta_calcInverse(location);
    delta_update_angles(location);
    delta_close_claw();
    
    return 0;
}

int delta_move(point_t* location)
{
    if(delta_calcInverse(location) || delta_validAngles(location))
    {
        //printf("\nERROR: Invalid arm location: %f, %f, %f\n", location->x, location->y, location->z);
        return -1;
    }
    if(location->claw)
        delta_close_claw();
    else
        delta_open_claw();

    delta_update_angles(location);
    return 0;
}


int delta_standby()
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

int delta_grab(point_t* location)
{
    delta_open_claw();
    location->z -= 50;
    if(delta_move(location));
    {
        printf("ERROR: Unable to grab\n");
        return -1;
    }
    delta_close_claw();
    location->z += 50;
    delta_move(location);

    return 0;
}


static struct termios oldt, newt;

void __handler(int sig)
{
    printf("Exiting...\n");

    /*restore the old settings*/
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
    

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);          
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

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
