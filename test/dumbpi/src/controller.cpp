#include <iostream>
#include <string>

#include "controller.h"

int RUN_SPEED = 22;


void set_movement(enum Direction d)
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
        softPwmWrite(GPIO_pwm_front_left, 0);
        softPwmWrite(GPIO_pwm_front_right, 0);
        softPwmWrite(GPIO_pwm_back_left, 0);
        softPwmWrite(GPIO_pwm_back_right, 0);
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
        softPwmWrite(GPIO_pwm_front_left, RUN_SPEED);
        softPwmWrite(GPIO_pwm_front_right, RUN_SPEED);
        softPwmWrite(GPIO_pwm_back_left, RUN_SPEED);
        softPwmWrite(GPIO_pwm_back_right, RUN_SPEED);
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
        softPwmWrite(GPIO_pwm_front_left, RUN_SPEED);
        softPwmWrite(GPIO_pwm_front_right, RUN_SPEED);
        softPwmWrite(GPIO_pwm_back_left, RUN_SPEED);
        softPwmWrite(GPIO_pwm_back_right, RUN_SPEED);
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
        softPwmWrite(GPIO_pwm_front_left, RUN_SPEED);
        softPwmWrite(GPIO_pwm_front_right, RUN_SPEED);
        softPwmWrite(GPIO_pwm_back_left, RUN_SPEED);
        softPwmWrite(GPIO_pwm_back_right, RUN_SPEED);
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
        softPwmWrite(GPIO_pwm_front_left, RUN_SPEED);
        softPwmWrite(GPIO_pwm_front_right, RUN_SPEED);
        softPwmWrite(GPIO_pwm_back_left, RUN_SPEED);
        softPwmWrite(GPIO_pwm_back_right, RUN_SPEED);
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
        softPwmWrite(GPIO_pwm_front_left, RUN_SPEED);
        softPwmWrite(GPIO_pwm_front_right, RUN_SPEED);
        softPwmWrite(GPIO_pwm_back_left, RUN_SPEED);
        softPwmWrite(GPIO_pwm_back_right, RUN_SPEED);
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
        softPwmWrite(GPIO_pwm_front_left, RUN_SPEED);
        softPwmWrite(GPIO_pwm_front_right, RUN_SPEED);
        softPwmWrite(GPIO_pwm_back_left, RUN_SPEED);
        softPwmWrite(GPIO_pwm_back_right, RUN_SPEED);
        break;
    }
}

void setup(void)
{
    printf("Starting up ...\n");
    
    if (wiringPiSetup() == -1)
    {
        printf("wiringPi Init error\n");
        exit(1);
    }

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


void keyboard_info(const std_msgs::String::ConstPtr &msg)
{
    std::string cmd = std::string(msg->data.c_str());
    ROS_INFO("Controller receive: [%s]", cmd.c_str());
    char c = cmd[0];
    if (c == 'b' or c == 'q'){
        // TODO: exit 
    }
    else if (c == ' '){
        set_movement(Stop);
    }
    else if (c == 'w'){
        set_movement(Forward);
    }
    else if (c == 's'){
        set_movement(Backward);
    }
    else if (c == 'a'){
        set_movement(Left);
    }
    else if (c == 'd'){
        set_movement(Right);
    }
    else if (c == 'j'){
        set_movement(TurnLeft);
    }
    else if (c == 'k'){
        set_movement(TurnRight);
    }
    else{
        set_movement(Stop);
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "dumbpi_controller");
    ros::NodeHandle n;
    setup();

    ros::Subscriber sub = n.subscribe("dumbpi_cmd", 1000, keyboard_info);
    
    ros::spin();
    exit_program();
    return 0;
}
