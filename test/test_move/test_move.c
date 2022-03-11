#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <wiringPi.h>
#include <softPwm.h>

#include "pins.h"
#define START_SPEED 25

bool continue_loop;

enum Direction
{
    Stop = 0,
    Forward = 1,
    Backward = 2,
    Left = 3,
    Right = 4,
    TurnLeft = 5,
    TurnRight = 6,
};

void set_direction(enum Direction d)
{
    switch (d)
    {
    case Stop:
        digitalWrite(GPIO_move_direction_FL_a, HIGH);
        digitalWrite(GPIO_move_direction_FL_b, HIGH);
        digitalWrite(GPIO_move_direction_FR_a, HIGH);
        digitalWrite(GPIO_move_direction_FR_b, HIGH);
        digitalWrite(GPIO_move_direction_BL_a, HIGH);
        digitalWrite(GPIO_move_direction_BL_b, HIGH);
        digitalWrite(GPIO_move_direction_BR_a, HIGH);
        digitalWrite(GPIO_move_direction_BR_b, HIGH);
        break;
    case Forward:
        digitalWrite(GPIO_move_direction_FL_a, HIGH);
        digitalWrite(GPIO_move_direction_FL_b, LOW);
        digitalWrite(GPIO_move_direction_FR_a, HIGH);
        digitalWrite(GPIO_move_direction_FR_b, LOW);
        digitalWrite(GPIO_move_direction_BL_a, HIGH);
        digitalWrite(GPIO_move_direction_BL_b, LOW);
        digitalWrite(GPIO_move_direction_BR_a, HIGH);
        digitalWrite(GPIO_move_direction_BR_b, LOW);
        break;
    case Backward:
        digitalWrite(GPIO_move_direction_FL_a, LOW);
        digitalWrite(GPIO_move_direction_FL_b, HIGH);
        digitalWrite(GPIO_move_direction_FR_a, LOW);
        digitalWrite(GPIO_move_direction_FR_b, HIGH);
        digitalWrite(GPIO_move_direction_BL_a, LOW);
        digitalWrite(GPIO_move_direction_BL_b, HIGH);
        digitalWrite(GPIO_move_direction_BR_a, LOW);
        digitalWrite(GPIO_move_direction_BR_b, HIGH);
        break;
    case Left:
        digitalWrite(GPIO_move_direction_FL_a, LOW);
        digitalWrite(GPIO_move_direction_FL_b, HIGH);
        digitalWrite(GPIO_move_direction_FR_a, HIGH);
        digitalWrite(GPIO_move_direction_FR_b, LOW);
        digitalWrite(GPIO_move_direction_BL_a, HIGH);
        digitalWrite(GPIO_move_direction_BL_b, LOW);
        digitalWrite(GPIO_move_direction_BR_a, LOW);
        digitalWrite(GPIO_move_direction_BR_b, HIGH);
        break;
    case Right:
        digitalWrite(GPIO_move_direction_FL_a, HIGH);
        digitalWrite(GPIO_move_direction_FL_b, LOW);
        digitalWrite(GPIO_move_direction_FR_a, LOW);
        digitalWrite(GPIO_move_direction_FR_b, HIGH);
        digitalWrite(GPIO_move_direction_BL_a, LOW);
        digitalWrite(GPIO_move_direction_BL_b, HIGH);
        digitalWrite(GPIO_move_direction_BR_a, HIGH);
        digitalWrite(GPIO_move_direction_BR_b, LOW);
        break;
    case TurnLeft:
        digitalWrite(GPIO_move_direction_FL_a, LOW);
        digitalWrite(GPIO_move_direction_FL_b, HIGH);
        digitalWrite(GPIO_move_direction_FR_a, HIGH);
        digitalWrite(GPIO_move_direction_FR_b, LOW);
        digitalWrite(GPIO_move_direction_BL_a, LOW);
        digitalWrite(GPIO_move_direction_BL_b, HIGH);
        digitalWrite(GPIO_move_direction_BR_a, HIGH);
        digitalWrite(GPIO_move_direction_BR_b, LOW);
        break;
    case TurnRight:
        digitalWrite(GPIO_move_direction_FL_a, HIGH);
        digitalWrite(GPIO_move_direction_FL_b, LOW);
        digitalWrite(GPIO_move_direction_FR_a, LOW);
        digitalWrite(GPIO_move_direction_FR_b, HIGH);
        digitalWrite(GPIO_move_direction_BL_a, HIGH);
        digitalWrite(GPIO_move_direction_BL_b, LOW);
        digitalWrite(GPIO_move_direction_BR_a, LOW);
        digitalWrite(GPIO_move_direction_BR_b, HIGH);
        break;
    }
}


void *keyboard_interrupt(void *param)
{
    while (continue_loop)
    {
        char c = getchar();
        if (c == 'b' or c == 'q'){
            continue_loop = false;
        }
        else if (c == ' '){
            set_direction(Stop);
        }
        else if (c == 'w'){
            set_direction(Forward);
        }
        else if (c == 's'){
            set_direction(Backward);
        }
        else if (c == 'a'){
            set_direction(Left);
        }
        else if (c == 'd'){
            set_direction(Right);
        }
        else if (c == 'j'){
            set_direction(TurnLeft);
        }
        else if (c == 'k'){
            set_direction(TurnRight);
        }
        else{
            set_direction(Stop);
        }
        delay(100);
    }
    return NULL;
}

void setup(void)
{
    printf("Starting up ...\n");

    // pin initialization
    pinMode(GPIO_move_direction_FL_a, OUTPUT);
    pinMode(GPIO_move_direction_FL_b, OUTPUT);
    pinMode(GPIO_move_direction_FR_a, OUTPUT);
    pinMode(GPIO_move_direction_FR_b, OUTPUT);
    pinMode(GPIO_move_direction_BL_a, OUTPUT);
    pinMode(GPIO_move_direction_BL_b, OUTPUT);
    pinMode(GPIO_move_direction_BR_a, OUTPUT);
    pinMode(GPIO_move_direction_BR_b, OUTPUT);

    pinMode(GPIO_pwm_front_left, PWM_OUTPUT);
    pinMode(GPIO_pwm_front_right, PWM_OUTPUT);
    pinMode(GPIO_pwm_back_left, PWM_OUTPUT);
    pinMode(GPIO_pwm_back_right, PWM_OUTPUT);

    softPwmCreate(GPIO_pwm_front_left, 0, 100);
    softPwmCreate(GPIO_pwm_front_right, 0, 100);
    softPwmCreate(GPIO_pwm_back_left, 0, 100);
    softPwmCreate(GPIO_pwm_back_right, 0, 100);

    delay(300);
}

void exit_program(void)
{
    // all high to lock the wheels
    digitalWrite(GPIO_move_direction_FL_a, HIGH);
    digitalWrite(GPIO_move_direction_FL_b, HIGH);
    digitalWrite(GPIO_move_direction_FR_a, HIGH);
    digitalWrite(GPIO_move_direction_FR_b, HIGH);
    digitalWrite(GPIO_move_direction_BL_a, HIGH);
    digitalWrite(GPIO_move_direction_BL_b, HIGH);
    digitalWrite(GPIO_move_direction_BR_a, HIGH);
    digitalWrite(GPIO_move_direction_BR_b, HIGH);

    softPwmWrite(GPIO_pwm_front_left, 0);
    softPwmWrite(GPIO_pwm_front_right, 0);
    softPwmWrite(GPIO_pwm_back_left, 0);
    softPwmWrite(GPIO_pwm_back_right, 0);
}

int main(void)
{
    continue_loop = true;
    pthread_t id;
    int res = pthread_create(&id, NULL, (void *(*)(void *))keyboard_interrupt, NULL);

    if (wiringPiSetup() == -1)
    {
        printf("wiringPi Init error\n");
        exit(1);
    }
    setup();

    // loop
    for (int i = 0; continue_loop; i++)
    {
        softPwmWrite(GPIO_pwm_front_left, START_SPEED);
        softPwmWrite(GPIO_pwm_front_right, START_SPEED);
        softPwmWrite(GPIO_pwm_back_left, START_SPEED);
        softPwmWrite(GPIO_pwm_back_right, START_SPEED);
        delay(1000);
    }
    exit_program();
    pthread_join(id, NULL);
    return 0;
}