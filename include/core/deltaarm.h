#ifndef _DELTA_ARM_H
#define _DELTA_ARM_H


#include <stdio.h>
#include <math.h>
#include <delta_defs.h>
#include <setpoint_manager.h>

#include "servo.h"

typedef struct {
    double X;
    double Y;
    double Z;

    double X_dot;
    double Y_dot;
    double Z_dot;

    double X_ddot;
    double Y_ddot;
    double Z_ddot;

    double X_dot_ff;
    double Y_dot_ff;
    double Z_dot_ff;

    double theta1;
    double theta2;
    double theta3;

    int claw;
} point_t;



// Delta arm function declarations
int delta_init();
int delta_calcForward(setpoint_t* location);
int delta_calcAngleYZ(double x0, double y0, double z0, double *theta);
int delta_calcInverse(setpoint_t* location);
void delta_update_angles(setpoint_t* location);
int delta_standby();
int delta_open_claw();
int delta_close_claw();
int delta_grab(setpoint_t* location);
int delta_move(setpoint_t* location);
int delta_check_battery(float voltage);

int delta_main();


#endif