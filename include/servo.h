#ifndef SERVO_H
#define SERVO_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <signal.h>

#include <rc/servo.h>
#include <rc/pthread.h>
#include <rc/time.h>


#define MAX_ANGLE 68

typedef struct {
    int initialized;
    pthread_t send_pulse_thread;

    int running[8];
    float speed[8];
    
    int width_goal[8];
    float width[8];
} servo_t;
servo_t servos;


void servo_set_angle(int ch, double angle);
int servos_init(int* ch, int* speeds, int num_ch);

#endif