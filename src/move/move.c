#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>

#include "../utils/pins.h"

int main(void)
{

    if(wiringPiSetup() == -1)
    {
    	printf("wiringPi Init error\n");
		exit(1);
	}

    printf("Starting up ...\n")
    // pin initialization
    pinMode(GPIO_pwm_front_left, PWM_OUTPUT); 
    softPwmCreate(GPIO_pwm_front_left, 0, 100); 

    // loop
    while(true)
    {
        softPwmWrite(GPIO_pwm_front_left, 50);
        delay(1000);
    }

    return 0;
}