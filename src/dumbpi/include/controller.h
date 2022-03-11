#include "ros/ros.h"
#include "std_msgs/String.h"

#include <wiringPi.h>
#include <softPwm.h>

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

int GPIO_pwm_front_left = 24; // physical 35 orange
int GPIO_pwm_front_right = 25; // physical 37 yellow
int GPIO_pwm_back_left = 27; // physical 36 blue
int GPIO_pwm_back_right = 28; // physical 38 green
int GPIO_move_direction_FL_a = 22; // physical 31 purple
int GPIO_move_direction_FL_b = 26; // physical 32 gray

int GPIO_move_direction_FR_a = 0; // physical 11 purple
int GPIO_move_direction_FR_b = 1; // physical 12 gray
int GPIO_move_direction_BL_a = 2; // physical 13 white
int GPIO_move_direction_BL_b = 3; // physical 15 black
int GPIO_move_direction_BR_a = 4; // physical 16 brown
int GPIO_move_direction_BR_b = 5; // physical 18 red
