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

    printf("Starting up ...\n");
    // pin initialization
    pinMode(GPIO_move_direction_a, OUTPUT); 
    pinMode(GPIO_move_direction_b, OUTPUT); 
    pinMode(GPIO_pwm_front_left, PWM_OUTPUT); 

    // loop
    int i = 0;
    while(true)
    {
        char c = getchar();
        if(c == "b"){
            break;
        }

        if(i % 2 == 0){
            digitalWrite(GPIO_move_direction_a, HIGH);
            digitalWrite(GPIO_move_direction_b, LOW);
        }
        else{
            digitalWrite(GPIO_move_direction_a, LOW);
            digitalWrite(GPIO_move_direction_b, HIGH);
        }
        
        softPwmWrite(GPIO_pwm_front_left, 20);
        delay(5000);
        i++;
    }
    return 0;
}