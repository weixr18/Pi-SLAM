#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <wiringPi.h>
#include <softPwm.h>

#include "../utils/pins.h"

bool continue_loop;

unsigned int keyboard_interrupt(void* param)
{
    while(continue_loop){
        if(getchar()=='b'){
            continue_loop = false;
        }
        delay(1000);
    }
    return 0;
}

void setup(void){

    printf("Starting up ...\n");

    // pin initialization
    pinMode(GPIO_move_direction_a, OUTPUT); 
    pinMode(GPIO_move_direction_b, OUTPUT); 
    pinMode(GPIO_pwm_front_left, PWM_OUTPUT); 
    pinMode(GPIO_pwm_front_right, PWM_OUTPUT); 
    pinMode(GPIO_pwm_back_left, PWM_OUTPUT); 
    pinMode(GPIO_pwm_back_right, PWM_OUTPUT); 
    softPwmCreate(GPIO_pwm_front_left, 0, 100);
    softPwmCreate(GPIO_pwm_front_right, 0, 100);
    softPwmCreate(GPIO_pwm_back_left, 0, 100);
    softPwmCreate(GPIO_pwm_back_right, 0, 100);

}

int main(void)
{
    continue_loop = true;
    pthread_t id;
    int res = pthread_create(&id, NULL, (unsigned int *)keyboard_interrupt, NULL);

    if(wiringPiSetup() == -1)
    {
        printf("wiringPi Init error\n");
        exit(1);
    }
    setup();

    // loop
    digitalWrite(GPIO_move_direction_a, HIGH);
    digitalWrite(GPIO_move_direction_b, LOW);
    while(continue_loop)
    {        
        softPwmWrite(GPIO_pwm_front_left, 10);
        delay(1000);
    }

    softPwmWrite(GPIO_pwm_front_left, 0);
    pthread_join(id, NULL);
    return 0;
}