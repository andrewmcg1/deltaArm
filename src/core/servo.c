#include "servo.h"

#define START_LOC 0

#define US_MAX 2100
#define US_MIN 900
#define SPEED .5



void* __send_pulses(void* servo_);


void servos_exit(int k)
{
    for(int i = 0; i < 8; i++)
        servos.running[i] = 0;
    rc_pthread_timed_join(servos.send_pulse_thread, NULL, 1.0); // wait for it to stop
    rc_servo_cleanup();
    exit(k);
}

int servos_init(int* ch, int* speeds, int num_ch)
{
    if(!servos.initialized)
    {
        for(int i = 0; i < num_ch; i++)
        {
            servos.running[ch[i]-1] = 1;
            servos.speed[ch[i]-1] = speeds[i];
            servos.width_goal[ch[i]-1] = (((US_MIN + US_MAX) / 2) + START_LOC * (US_MAX - US_MIN) / (MAX_ANGLE * 2));
            servos.width[ch[i]-1] = servos.width_goal[ch[i]-1];
        }
        if(!rc_servo_init())
        {    
            rc_pthread_create(&(servos.send_pulse_thread), __send_pulses, (void*)&servos, 0, 0);
            rc_usleep(1000000);
            return 0;
        }
        else
        {
            printf("ERROR: Failed to initialize servos\n");
            return -1;
        }
    }
    else
    {
        printf("ERROR: Servos already initialized\n");
        return -1;
    }
}

void* __send_pulses(void* servo_)
{
    signal(SIGINT, servos_exit);

    int pulse_success = 0;
    servo_t* servos_ = (servo_t*)servo_;
    while(1)
    {    
        for(int i = 0; i < 8; i++)
        {
            pulse_success = rc_servo_send_pulse_us(i+1, (int)servos_->width[i]);
            if(pulse_success)
            {
                printf("ERROR: Failed to send pulse\n");
                servos_exit(1);
            }
            else if(servos_->running[i] == 1 && (int)servos_->width[i] != servos_->width_goal[i])
            {
                servos_->width[i] += (servos_->width[i] < servos_->width_goal[i]) ? servos_->speed[i] : -1 * servos_->speed[i];
            }   
        }
        rc_usleep(2000);   // 5-hundo Hz
    }
}

void servo_set_angle(int ch, double angle)
{
    if (fabs(angle) > MAX_ANGLE)
    {
        printf("ERROR: Invalid angle: %lf\n", angle);        
        servos.running[ch-1] = 0;
        rc_pthread_timed_join(servos.send_pulse_thread, NULL, 1.0); // wait for it to stop
        rc_servo_cleanup();
        exit(1);
    }
    servos.width_goal[ch-1] = (((US_MIN + US_MAX) / 2) + angle * (US_MAX - US_MIN) / (MAX_ANGLE * 2));
    rc_usleep(10000);
}