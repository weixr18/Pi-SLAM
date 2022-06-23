/**
 * This file is part of project Active-ORB-SLAM. 
 * The author is Xinran Wei from Tsinghua University.
 * This project is under GPLv3 lisence.
 * */


#ifndef MAP2D_H
#define MAP2D_H

#include <mutex>

#include "Eigen/Core"
#include "Thirdparty/g2o/g2o/core/optimizable_graph.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/types/types_sba.h"

#include "Map.h"

namespace ORB_SLAM2
{

typedef g2o::OptimizableGraph::EdgeContainer EdgeContainer;
typedef const g2o::VertexSBAPointXYZ* ConstVtxMapPoint;
    
void ShowImage(cv::Mat& image, const char* title);

class Map;
class Tracking;

class Map2d
{
public:

    Map2d();
    ~Map2d();
    void SetTracker(Tracking* pTracker);
    
    enum GridType{
        GRID_UNKNOWN,
        GRID_OCCUPIED,
        GRID_FREE,
        GRID_CAMERA,
    };
    struct MapGrid{
        GridType type;
        double baError; 
    };
    
    struct Pos3dScale{
        float fxMin;
        float fxMax;
        float fzMin;
        float fzMax;
    }; 
    struct Pos2dScale{
        int xScale = 0;
        int zScale = 0;
    }; 
    
    /*** map data allocations ***/
    void UpdateMap(int xScale, int zScale, Pos3dScale& newScalef);
    void ClearMap();
    Pos2dScale GetMapSize();
    bool IsMapEmpty(){
        return (m2dScale.xScale == 0 || m2dScale.zScale == 0);
    }
    
    /*** pos transfer ***/
    Eigen::Vector2d Get2dPos(Eigen::Vector3d pos3d);
    Eigen::Vector2d Get2dPos(float fx, float fz);
    
    /*** occupy map ***/
    // obstacle filter
    void ObstacleFilter();
    // calculate nearest obstacle distance
    float GetObstacleDistance();
    // update grid type
    void UpdateType(Map* map, bool fromLoopClose = false);
    
    
    /*** error map ***/
    // update map error
    void UpdateError(EdgeContainer& activeEdges);
    void UpdateErrorWholeMap(EdgeContainer& activeEdges);
    
    /*** graphicization ***/
    // transfer to uchar occupy image
    cv::Mat* GetOccupyMapImage();
    // transfer to uchar error image
    cv::Mat* GetErrorMapImage();
    // transfer to color error image
    cv::Mat* GetErrorMapImageColor(cv::Mat* errorMap);
    
    /*** show image ***/
    void ShowOccupy1Map();
    void ShowColor2dMap();
    
public:
    // update mutex
    std::mutex mMutexMapUpdate;
	
protected:
    
    // building map
    const float MIN_DELTA_POS = 0.01;
    Pos3dScale m3dScalef;
    Pos2dScale m2dScale;
    
    // camera
    Eigen::Vector2d mCamPosf;
    Eigen::Vector2d mCamDirectionf;
    
    // data
    MapGrid** mMapData = nullptr;
    
    // error
    double mMaxError = 0.0001;
    
    Tracking* mpTracker;
    
    // for debug
    int tmp_count = 0;
};

} //namespace ORB_SLAM2

#endif // MAP2D_H
