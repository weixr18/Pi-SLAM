/**
 * This file is part of project Active-ORB-SLAM. 
 * The author is Xinran Wei from Tsinghua University.
 * This project is under GPLv3 lisence.
 * */
 
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <wiringPi.h>
#include <softPwm.h>

#include "LoopClosing.h"
#include "Tracking.h"
#include "LocalMapping.h"
#include "Viewer.h"
#include "Map2d.h"

#include "Thirdparty/g2o/g2o/core/optimizable_graph.h"



namespace ORB_SLAM2
{
typedef g2o::OptimizableGraph::EdgeContainer EdgeContainer;

class Tracking;
class LocalMapping;
class LoopClosing;
class Viewer;
class FrameDrawer;

enum MoveDirection {
    Stop = 0,
    Forward = 1,
    Backward = 2,
    Left = 3,
    Right = 4,
    TurnLeft = 5,
    TurnRight = 6,
};

class Controller
{
protected:

    static const int RUN_SPEED = 22;

    static const int GPIO_pwm_front_left = 24; // physical 35 orange
    static const int GPIO_pwm_front_right = 25; // physical 37 yellow
    static const int GPIO_pwm_back_left = 27; // physical 36 blue
    static const int GPIO_pwm_back_right = 28; // physical 38 green
    static const int GPIO_move_direction_FL_a = 22; // physical 31 purple
    static const int GPIO_move_direction_FL_b = 26; // physical 32 gray

    static const int GPIO_move_direction_FR_a = 0; // physical 11 purple
    static const int GPIO_move_direction_FR_b = 1; // physical 12 gray
    static const int GPIO_move_direction_BL_a = 2; // physical 13 white
    static const int GPIO_move_direction_BL_b = 3; // physical 15 black
    static const int GPIO_move_direction_BR_a = 4; // physical 16 brown
    static const int GPIO_move_direction_BR_b = 5; // physical 18 red

public:
    Controller();
    ~Controller();

    void SetLoopCloser(LoopClosing* pLoopCloser);
    void SetTracker(Tracking* pTracker);
    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetViewer(Viewer* pViewer);

    // Main function
    void Run();
    
    // move function
    void Move(MoveDirection d, int nTimePeriods);
    
    // Active mapping
    void ActiveMapping();
    
    // Active revisiting
    void ActiveRevisiting();
    // set loop closed
    void SetLoopClosed(EdgeContainer& edges);

public:

    Map2d* mpMap2d = nullptr;   
    bool mbLoopClosed = false; 

protected:

    int mstate;
    Map* mpMap = nullptr;
    FrameDrawer* mpFrameDrawer = nullptr;
    
    LoopClosing* mpLoopCloser = nullptr;
    Tracking* mpTracker = nullptr;
    LocalMapping* mpLocalMapper = nullptr;
    Viewer* mpViewer = nullptr;
};

}

#endif //CONTROLLER_H
