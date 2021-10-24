#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <wiringPi.h>
#include <softPwm.h>

#include "../utils/pins.h"

bool continue_loop;

void* keyboard_interrupt(void* param)
{
    while(continue_loop){
        if(getchar()=='b'){
            continue_loop = false;
        }
        delay(100);
    }
    return NULL;
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

void exit_program(void){
    digitalWrite(GPIO_move_direction_a, LOW);
    digitalWrite(GPIO_move_direction_b, LOW);
    softPwmWrite(GPIO_pwm_front_left, 0);
    softPwmWrite(GPIO_pwm_front_right, 0);
    softPwmWrite(GPIO_pwm_back_left, 0);
    softPwmWrite(GPIO_pwm_back_right, 0);
}

int main(void)
{
    continue_loop = true;
    pthread_t id;
    // int res = pthread_create(&id, NULL, (void* (*)(void*))keyboard_interrupt, NULL);

    if(wiringPiSetup() == -1)
    {
        printf("wiringPi Init error\n");
        exit(1);
    }
    setup();

    // loop
    digitalWrite(GPIO_move_direction_a, HIGH);
    digitalWrite(GPIO_move_direction_b, LOW);
    for(int i = 0; continue_loop; i++)
    {
        printf("i = %d\n", i);
        if(i % 5 == 0){
            softPwmWrite(GPIO_pwm_front_left, 20);
            softPwmWrite(GPIO_pwm_front_right, 0);
            softPwmWrite(GPIO_pwm_back_left, 0);
            softPwmWrite(GPIO_pwm_back_right, 0);
        }
        else if(i % 5 == 1){
            softPwmWrite(GPIO_pwm_front_left, 0);
            softPwmWrite(GPIO_pwm_front_right, 20);
            softPwmWrite(GPIO_pwm_back_left, 0);
            softPwmWrite(GPIO_pwm_back_right, 0);
        }
        else if(i % 5 == 2){
            softPwmWrite(GPIO_pwm_front_left, 0);
            softPwmWrite(GPIO_pwm_front_right, 0);
            softPwmWrite(GPIO_pwm_back_left, 20);
            softPwmWrite(GPIO_pwm_back_right, 0);
        }
        else if(i % 5 == 3){
            softPwmWrite(GPIO_pwm_front_left, 0);
            softPwmWrite(GPIO_pwm_front_right, 0);
            softPwmWrite(GPIO_pwm_back_left, 0);
            softPwmWrite(GPIO_pwm_back_right, 20);
        }
        else if(i % 5 == 4){
            softPwmWrite(GPIO_pwm_front_left, 20);
            softPwmWrite(GPIO_pwm_front_right, 20);
            softPwmWrite(GPIO_pwm_back_left, 20);
            softPwmWrite(GPIO_pwm_back_right, 20);
        }
        delay(1000);
    }

    exit_program();
    // pthread_join(id, NULL);
    return 0;
}