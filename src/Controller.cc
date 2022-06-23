/**
 * This file is part of project Active-ORB-SLAM. 
 * The author is Xinran Wei from Tsinghua University.
 * This project is under GPLv3 lisence.
 * */


#include "Controller.h"

#include <mutex>
#include <iostream>
#include <unistd.h>
#include <opencv2/core.hpp>


namespace ORB_SLAM2
{

void Controller::SetLoopCloser(LoopClosing* pLoopCloser){
    mpLoopCloser = pLoopCloser;
}

void Controller::SetTracker(Tracking* pTracker){
    mpTracker = pTracker;
    mpMap2d->SetTracker(pTracker);
}

void Controller::SetLocalMapper(LocalMapping* pLocalMapper){
    mpLocalMapper = pLocalMapper;
    mpMap = pLocalMapper->getMap();
}

void Controller::SetViewer(Viewer* pViewer){
    mpViewer = pViewer;
    mpFrameDrawer =  pViewer->getFrameDrawer();
}


void msleep(unsigned long n){
    usleep(n*1000);
}


Controller::Controller(){
    
    mpMap2d = new Map2d();
    
    // setup wiringpi
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
    
    digitalWrite(GPIO_move_direction_FL_a, HIGH);
    digitalWrite(GPIO_move_direction_FL_b, HIGH);
    digitalWrite(GPIO_move_direction_FR_a, HIGH);
    digitalWrite(GPIO_move_direction_FR_b, HIGH);
    digitalWrite(GPIO_move_direction_BL_a, HIGH);
    digitalWrite(GPIO_move_direction_BL_b, HIGH);
    digitalWrite(GPIO_move_direction_BR_a, HIGH);
    digitalWrite(GPIO_move_direction_BR_b, HIGH);
    msleep(300);
    
    softPwmWrite(GPIO_pwm_front_left, RUN_SPEED);
    softPwmWrite(GPIO_pwm_front_right, RUN_SPEED);
    softPwmWrite(GPIO_pwm_back_left, RUN_SPEED);
    softPwmWrite(GPIO_pwm_back_right, RUN_SPEED);
    msleep(300);
    
    std::cout << "Controller constructed!" << std::endl;
}

Controller::~Controller(){
    
    delete mpMap2d;

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



// move function
void Controller::Move(MoveDirection d, int nTimePeriods){
    
    std::cout << "[Move] " << "direction: " << d << ", time:" << nTimePeriods << std::endl;
    
    for(int i = 0; i < nTimePeriods; i++){
        /*
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
        } // end switch
        */ 
        
        msleep(100); // 100 ms for a period
    } // end for
    
    // Important!! Stop the vehicle after moving!!!
    digitalWrite(GPIO_move_direction_FL_a, HIGH);
    digitalWrite(GPIO_move_direction_FL_b, HIGH);
    digitalWrite(GPIO_move_direction_FR_a, HIGH);
    digitalWrite(GPIO_move_direction_FR_b, HIGH);
    digitalWrite(GPIO_move_direction_BL_a, HIGH);
    digitalWrite(GPIO_move_direction_BL_b, HIGH);
    digitalWrite(GPIO_move_direction_BR_a, HIGH);
    digitalWrite(GPIO_move_direction_BR_b, HIGH);
}

void Controller::ActiveMapping(){
    
    const int DISTANCE_NEAR = 25;
    const int DISTANCE_FAR = 50;
    const int KEY_POINT_DENSE = 300;
    const int KEY_POINT_SPARSE = 100;
    
    
    // get min distance
    float obstacleDistance;
    Map2d::Pos2dScale mapScale;
    obstacleDistance = mpMap2d->GetObstacleDistance();
    std::cout << "[ActiveMapping] " << "minDistance: " << obstacleDistance << std::endl;
    mapScale = mpMap2d->GetMapSize();
 
    int mapSize = std::max(mapScale.xScale, mapScale.zScale);
    if ( abs(obstacleDistance - 1.414*mapSize) < 5){
        std::cout << "[ActiveMapping] " << "minDistance error, try to reget..." << std::endl;
        sleep(8);
        
        obstacleDistance = mpMap2d->GetObstacleDistance();
        std::cout << "[ActiveMapping] " << "minDistance: " << obstacleDistance << std::endl;
        if ( abs(obstacleDistance - 1.414*mapSize) < 5){
            std::cout << "[ActiveMapping] " << "Tracking lost." << std::endl;
            mstate = Tracking::LOST;
            return;
        }
    }
    // judge obstacle
    bool bObstacleNear = obstacleDistance < DISTANCE_NEAR;
    bool bObstacleFar = obstacleDistance > DISTANCE_FAR;
    
    // judge key dense
    int nKeysInCurrentFrame = mpFrameDrawer->getCurrentKeyNumber();
    std::cout << "[ActiveMapping] " << "nKeyPoints: " << nKeysInCurrentFrame << std::endl;
    bool bKeysDense = nKeysInCurrentFrame > KEY_POINT_DENSE;
    bool bKeysSparse = nKeysInCurrentFrame < KEY_POINT_SPARSE;
    int obstacleState = bObstacleNear ? 1 : (bObstacleFar ? 3 : 2);
    int keysState = bKeysSparse ? 1 : (bKeysDense ? 3 : 2);
    std::cout << "[ActiveMapping] " << "obstacleState: " << obstacleState
                << ", keysState: " << keysState << std::endl;
            
    if(bObstacleNear){
        if (bKeysDense){
            Move(MoveDirection::Backward, 3);
        }
        else if (!bKeysSparse){
            Move(MoveDirection::Backward, 2);
        }
        else // bKeysSparse
        {
            Move(MoveDirection::Right, 4);
        }
    } // bObstacleNear

    else if (!bObstacleFar) // middle distance
    {
        if (bKeysDense){
            Move(MoveDirection::Backward, 2);
        }
        else if (!bKeysSparse){
            Move(MoveDirection::Right, 4);
        }
        else // bKeysSparse
        {
            Move(MoveDirection::Forward, 2);
        }
    }
    
    else // bObstacleFar
    {
        if (bKeysDense){
            Move(MoveDirection::TurnRight, 1);
        }
        else if (!bKeysSparse){
            Move(MoveDirection::Forward, 2);
        }
        else {
            Move(MoveDirection::Forward, 3);
        }
    } // end bObstacleFar
    return;
}

void Controller::ActiveRevisiting(){
    std::cout << "[ActiveRevisiting] " << "working!!" << std::endl; 
}


void Controller::SetLoopClosed(EdgeContainer& edges){
    std::cout << "[Controller] " << "SetLoopClosed." << std::endl; 
    mbLoopClosed = true;
    mpMap2d->UpdateErrorWholeMap(edges);
}

// Main function
void Controller::Run(){
    
    const int ACTIVE_MAPPING_PERIOD = 20;
    int count = 0;
    
    
    while(true){
        count++;
        
        // not ready
	    if(nullptr == mpTracker){
            sleep(3);
            continue;
	    }
        mstate = mpTracker->mState;    
        if(mstate == Tracking::SYSTEM_NOT_READY ||
            mstate == Tracking::NO_IMAGES_YET){
            sleep(3);
            continue;
        }
        
        // active initialization
        else if(mstate==Tracking::NOT_INITIALIZED)
        {
	        Move(MoveDirection::Right, 3);
            sleep(3);
            continue;
        }
        
        // active mapping
        else if(mstate==Tracking::OK)
        {   
            mpMap2d->UpdateType(mpMap); // update 2dmap
            msleep(1000);    // sleep 200ms
            if(!mbLoopClosed){
                if(count % ACTIVE_MAPPING_PERIOD == 0){
                    ActiveMapping(); // active mapping                    
                }
            }
            else{
                ActiveRevisiting(); // active revisiting
            }
        }
        
        // active relocalization
        else if(mstate==Tracking::LOST)
        {
            // TODO: relocalize
            std::cout << "[Controller] " << "Active Relocalization." << std::endl;
            sleep(3);
            continue;
        }    
    }
    
}

} // namespace ORB_SLAM2
