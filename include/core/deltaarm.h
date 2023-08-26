#ifndef _DELTA_ARM_H
#define _DELTA_ARM_H


#include <stdio.h>
#include <math.h>
#include <delta_defs.h>

#include "servo.h"

typedef struct {
    double x;
    double y;
    double z;

    double theta1;
    double theta2;
    double theta3;

    int claw;
} point_t;


// Delta arm function declarations
int delta_init();
int delta_calcForward(point_t* location);
int delta_calcAngleYZ(double x0, double y0, double z0, double *theta);
int delta_calcInverse(point_t* location);
void delta_update_angles(point_t* location);
int delta_standby();
int delta_open_claw();
int delta_close_claw();
int delta_grab(point_t* location);
int delta_move(point_t* location);
int delta_check_battery(float voltage);

int delta_main();


#endif