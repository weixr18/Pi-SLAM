
/**
 * This file is part of project Active-ORB-SLAM. 
 * The author is Xinran Wei from Tsinghua University.
 * This project is under GPLv3 lisence.
 * */


#include "Map2d.h"
#include "Tracking.h"

#include <iostream>
#include <vector>
#include <unordered_map>
#include <unistd.h>
#include <math.h>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <cmath>

#define DEBUG_MAP2D

namespace ORB_SLAM2
{


Map2d::Map2d(){
    
}

Map2d::~Map2d(){
    
}

void Map2d::SetTracker(Tracking* pTracker){
    mpTracker = pTracker;
}




/*** map data allocations ***/


void Map2d::UpdateMap(int newXScale, int newZScale, Pos3dScale& newScalef){
    
    int xScale = m2dScale.xScale;
    int zScale = m2dScale.zScale;
    if(newZScale <= zScale && newXScale <= xScale){
        return; // no need to reallocate.
    }
    
    // allocate new space
    MapGrid** newMapData = new MapGrid*[newXScale];
    for(int i = 0; i < newXScale; i++){
        newMapData[i] = new MapGrid[newZScale];
    }
    for(int i = 0; i < newXScale; i++){
        for(int j = 0; j < newZScale; j++){
            newMapData[i][j].type = GridType::GRID_UNKNOWN;
            newMapData[i][j].baError = 0.0;
        }
    }
    
    if(nullptr != mMapData){
        // transfer error data
        int x_offset = (int) ((m3dScalef.fxMin - newScalef.fxMin) / MIN_DELTA_POS);
        int z_offset = (int) ((m3dScalef.fzMin - newScalef.fzMin) / MIN_DELTA_POS);
        for(int i = 0; i < xScale; i++){
            for(int j = 0; j < zScale; j++){
                newMapData[i+x_offset][j+z_offset].baError = mMapData[i][j].baError;
            }
        }
        
        // delete old space
        if(mMapData != nullptr && xScale > 0){
            for(int i = 0; i < xScale; i++){
                delete mMapData[i];
            }
            delete mMapData;
        }
    }
    
    // data assign
    mMapData = newMapData;
    // scale assign
    m2dScale.xScale = newXScale;
    m2dScale.zScale = newZScale;
    m3dScalef = newScalef;
}

void Map2d::ClearMap(){
    for(int i = 0; i < m2dScale.xScale; i++){
        for(int j = 0; j < m2dScale.zScale; j++){
            mMapData[i][j].type = GridType::GRID_UNKNOWN;
            mMapData[i][j].baError = 0.0;
        }
    }
}

Map2d::Pos2dScale Map2d::GetMapSize(){
    return m2dScale;
}



/*** pos transfer ***/


Eigen::Vector2d Map2d::Get2dPos(Eigen::Vector3d pos3d){
    return Get2dPos(pos3d(0), pos3d(2));
}

Eigen::Vector2d Map2d::Get2dPos(float fx, float fz){
    Eigen::Vector2d pos2d;
    pos2d(0) = (int) ((fx - m3dScalef.fxMin) / MIN_DELTA_POS);
    pos2d(1) = (int) ((fz - m3dScalef.fzMin) / MIN_DELTA_POS);
    return pos2d;
}



/*** occupy map ***/


void Map2d::ObstacleFilter(){
    
    int AREA_THRESHOLD = 30;
    int xScale = m2dScale.xScale;
    int zScale = m2dScale.zScale;
    
    // get binary map
    cv::Mat* binMap = GetOccupyMapImage();
    
    // dilate
    cv::Mat tmp0;
    cv::Mat kernelDilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::dilate(*binMap, tmp0, kernelDilate);
    
    // find contours
    cv::Mat contourLabels, contourStats, contourCentriods;
    int contourNum = cv::connectedComponentsWithStats(
        tmp0, contourLabels, contourStats, contourCentriods, 4, CV_32S // connectivity = 4
    ); 
    /*
        contourLabels: CV_32SC1, labels. 
        countourStats: [N, 5], CV_32SC1, surrounding rectangle's x,y,width,height and square.
        contourCentriods: [N, 2], CV_64F, centriod coordinates.
    */
    
    // select big countors
    bool* bSelect = new bool[contourNum];
    for(int i = 0; i < contourNum; i++){
        int area = contourStats.at<int>(i, cv::CC_STAT_AREA);
        bSelect[i] = (area > AREA_THRESHOLD);
        int width = contourStats.at<int>(i, 2);
        int height = contourStats.at<int>(i, 3);
        if(width > 0.98*xScale || height > 0.98*zScale){
            bSelect[i] = false;
        }
    }
    cv::Mat tmp1 = cv::Mat::zeros(zScale, xScale, 0); // cv::CV_8UC1
    for(int i = 0; i < xScale; i++){
        for(int j = 0; j < zScale; j++){
            int label = contourLabels.at<int>(zScale-j-1, i);
            if(bSelect[label]){
                tmp1.at<uchar>(zScale-j-1, i) = 255;
            }
        }
    }
    
    // erode
    cv::Mat tmp2;
    cv::erode(tmp1, tmp2, kernelDilate);
    
    // reset map type
    for(int i = 0; i < xScale; i++){
        for(int j = 0; j < zScale; j++){
            bool bObstacle = tmp2.at<uchar>(zScale-j-1, i) == 255;
            if(bObstacle){
                mMapData[i][j].type = GridType::GRID_OCCUPIED;
            }
            else{
                mMapData[i][j].type = GridType::GRID_UNKNOWN;
            }
        }
    }
}

float Map2d::GetObstacleDistance(){
    int xScale = m2dScale.xScale;
    int zScale = m2dScale.zScale;
    
    float fxCam = mCamPosf(0);
    float fzCam = mCamPosf(1);
    Eigen::Vector2d cam2dPos = Get2dPos(fxCam, fzCam);
    unsigned int xCam = cam2dPos(0);
    unsigned int zCam = cam2dPos(1);
    
    float fxDir = mCamDirectionf(0);
    float fzDir = mCamDirectionf(1);
    Eigen::Vector2d dir2dPos = Get2dPos(fxDir, fzDir);
    unsigned int xDir = dir2dPos(0);
    unsigned int zDir = dir2dPos(1);
    
    float THETA_THRESHOLD = 0.866; //cos(30 degree)
    float minDistance = 1.414 * std::max(xScale, zScale);
    for(int i = 0; i < xScale; i++){
        for(int j = 0; j < zScale; j++){
            if(mMapData[i][j].type != GridType::GRID_OCCUPIED){
                continue;
            }
            int deltaX = i - xCam;
            int deltaZ = j - zCam;
            float lenthDelta = sqrt(deltaX*deltaX+deltaZ*deltaZ);
            float lengthDir = sqrt(xDir*xDir+zDir*zDir);
            float cosTheta = (float)(deltaX*xDir+deltaZ*zDir) / (lenthDelta*lengthDir);
            if(cosTheta > THETA_THRESHOLD){
                if(minDistance > lenthDelta){
                    minDistance = lenthDelta;
                }
            }
        }
    }
    return minDistance;
}

void Map2d::UpdateType(Map* map, bool fromLoopClose){
    
    // lock
    std::unique_lock<std::mutex> lock(this->mMutexMapUpdate);
    
    // get map points
    cv::Mat groundPoints;
    int nGroundPoints;
    std::vector<MapPoint*> vpPoints; 
    int n;
    try{
    // get ground point poses
    vpPoints = map->GetAllMapPoints(); 
    n = vpPoints.size();
    cv::Mat points(3, n, 5);
    nGroundPoints = 0;
    for(int i = 0; i < n; i++){
        cv::Mat pos = (vpPoints[i])->GetWorldPos(); // CV_32FC1
        pos.col(0).copyTo(points.col(nGroundPoints));
        nGroundPoints++;
    }
    groundPoints = points(cv::Range::all(), cv::Range(0, nGroundPoints));
    }
    catch (std::exception &e){
        std::cout << "[Exception] " << "Catch!!" << "111111" << std::endl;
    }

    // get new map size
    cv::Mat xs;
    cv::Mat zs;
    double lfxMin, lfxMax, lfzMin, lfzMax;
    try{
    xs = groundPoints(cv::Range(0, 1), cv::Range::all());
    zs = groundPoints(cv::Range(2, 3), cv::Range::all());
    cv::minMaxIdx(xs, &lfxMin, &lfxMax, nullptr, nullptr);
    cv::minMaxIdx(zs, &lfzMin, &lfzMax, nullptr, nullptr);
    }
    catch (std::exception &e){
        std::cout << "[Exception] " << "Catch!!" << "222222" << std::endl;
    }

    
    Pos3dScale newScalef;
    newScalef.fxMin = (float)lfxMin;
    newScalef.fxMax = (float)lfxMax;
    newScalef.fzMin = (float)lfzMin;
    newScalef.fzMax = (float)lfzMax;

    // update camera position
    float fxCam, fzCam;
    bool bTracking = mpTracker->mState==Tracking::OK;
    try{
    if(bTracking){
        // if not tracking, don't update
        cv::Mat Tcw, Rwc, twc;
        Tcw = mpTracker->mCurrentFrame.mTcw.clone();
        if(Tcw.rows >=3  && Tcw.cols >=3 ){
            // if Tcw has appropriate shape, update camera position.
            Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*Tcw.rowRange(0,3).col(3);
            mCamPosf(0) = twc.at<float>(0, 0);
            mCamPosf(1) = twc.at<float>(2, 0);
            cv::Mat zAxis = (cv::Mat_<float>(3, 1) << 0, 0, 0.2);
            cv::Mat camDir= Rwc*zAxis;
            mCamDirectionf(0) = camDir.at<float>(0, 0);
            mCamDirectionf(1) = camDir.at<float>(2, 0);
        }

        // update scale. camera maybe at the edge of map
        fxCam = mCamPosf(0);
        fzCam = mCamPosf(1);
        newScalef.fxMin = min(newScalef.fxMin, (float)(fxCam-0.1));
        newScalef.fzMin = min(newScalef.fzMin, (float)(fzCam-0.1));
        newScalef.fxMax = max(newScalef.fxMax, (float)(fxCam+0.1));
        newScalef.fzMax = max(newScalef.fzMax, (float)(fzCam+0.1));
    }
    }
    catch (std::exception &e){
        std::cout << "[Exception] " << "Catch!!" << "3333333" << std::endl;
    }

    
    // update map
    // select most value of old and new scale
    newScalef.fxMin = min(newScalef.fxMin, m3dScalef.fxMin);
    newScalef.fzMin = min(newScalef.fzMin, m3dScalef.fzMin);
    newScalef.fxMax = max(newScalef.fxMax, m3dScalef.fxMax);
    newScalef.fzMax = max(newScalef.fzMax, m3dScalef.fzMax);  
    int newXScale = (int) ((newScalef.fxMax - newScalef.fxMin) / MIN_DELTA_POS) + 10;
    int newZScale = (int) ((newScalef.fzMax - newScalef.fzMin) / MIN_DELTA_POS) + 10;
    // rigion error copy
    try{
    UpdateMap(newXScale, newZScale, newScalef);
    }
    catch (std::exception &e){
        std::cout << "[Exception] " << "Catch!!" << "4444444" << std::endl;
    }


    // set occupations
    for(int i = 0; i < nGroundPoints; i++){
        Eigen::Vector2d point2dPos = Get2dPos(xs.at<float>(0, i), zs.at<float>(0, i));
        unsigned int x = point2dPos(0);
        unsigned int z = point2dPos(1);
        mMapData[x][z].type = GridType::GRID_OCCUPIED;
    }
    try{
    ObstacleFilter(); // filter obstacle
    }
    catch (std::exception &e){
        std::cout << "[Exception] " << "Catch!!" << "555555" << std::endl;
    }
    if(fromLoopClose){
        std::cout << "[UpdateType] " << "55555" << std::endl;
    }

    // set camera
    unsigned int xCam, zCam;
    if(bTracking){
        Eigen::Vector2d cam2dPos = Get2dPos(fxCam, fzCam);
        xCam = cam2dPos(0);
        zCam = cam2dPos(1);
        mMapData[xCam][zCam].type = GridType::GRID_CAMERA;
    }
    // show map data
    // ShowOccupy1Map();
}



/*** error map ***/
typedef std::unordered_map<g2o::OptimizableGraph::Edge*, Eigen::Vector2d> PosMap;
typedef std::pair<g2o::OptimizableGraph::Edge*, Eigen::Vector2d> PosMapItem;

void Map2d::UpdateError(EdgeContainer& activeEdges){
    
    int xScale = m2dScale.xScale;
    int zScale = m2dScale.zScale;
    // get updating (x,z) range
    int xmin = 9999;
    int xmax = -9999;
    int zmin = 9999;
    int zmax = -9999;
    PosMap posMap;
    for (int k = 0; k < static_cast<int>(activeEdges.size()); ++k) {
        g2o::OptimizableGraph::Edge* e = activeEdges[k];
        if(e == nullptr){
            continue;
        }
        ConstVtxMapPoint vMapPoint = static_cast<ConstVtxMapPoint>(e->vertices()[0]);
        Eigen::Vector3d pos3d = vMapPoint->estimate();
        Eigen::Vector2d pos2d = Get2dPos(pos3d);
        int x = (int)pos2d(0);
        int z = (int)pos2d(1);
        if(x < 0 || x > xScale){
            continue;
        }
        if(z < 0 || z > zScale){
            continue;
        }
        if(x > xmax) xmax = x; 
        if(x < xmin) xmin = x;
        if(z > zmax) zmax = z; 
        if(z < zmin) zmin = z; 

        PosMapItem item(e, pos2d);
        posMap.insert(item);
    }
    if(xmin > xmax || zmin > zmax){
        std::cout << "[UpdateError] " << "Error: no edge in optimize graph." << std::endl;
        return;
    }
    //std::cout << "[UpdateError] " << "xmax: " << xmax << ", xmin: " << xmin \
    //    << ", zmax: " << zmax << ", zmin: " << zmin << std::endl;
    
    
    // fill temp error map
    int xRange = xmax + 1 - xmin;
    int zRange = zmax + 1 - zmin;
    cv::Mat* tmpErrorMap = new cv::Mat(cv::Mat::zeros(xRange, zRange, 6)); // CV_64FC1
    cv::Mat* tmpErrorCount = new cv::Mat(cv::Mat::zeros(xRange, zRange, 0)); // CV_8UC1
    for (int k = 0; k < static_cast<int>(activeEdges.size()); ++k) {
        // get error
        g2o::OptimizableGraph::Edge* e = activeEdges[k];
        if(e == nullptr){
            continue;
        }
        double errorSqr;
        if (e->robustKernel()) {
            Eigen::Vector3d tmp;
            e->robustKernel()->robustify(e->chi2(), tmp);
            errorSqr = tmp[0];
        }
        else{
            errorSqr = e->chi2();
        }
        
        if(posMap.find(e) == posMap.end()){
            continue;
        }
        Eigen::Vector2d pos2d = posMap[e];
        int tmpX = pos2d(0) - xmin;
        int tmpZ = pos2d(1) - zmin;
        tmpErrorMap->at<double>(tmpX, tmpZ) += std::sqrt(errorSqr);
        tmpErrorCount->at<uchar>(tmpX, tmpZ) += 1;
    }
    
    // blur
    cv::Mat* blurredErrorMap = new cv::Mat(cv::Mat::zeros(zScale, xScale, 6)); // CV_64FC1
    GaussianBlur(*tmpErrorMap, *blurredErrorMap, cv::Size(11, 11), 0, 0);
    
    // update map data
    std::unique_lock<std::mutex> mapUpdateLock(mMutexMapUpdate); // add lock
    for(int i = 0; i < xRange; i++){
        for(int j = 0; j < zRange; j++){
            unsigned char count = tmpErrorCount->at<uchar>(i, j);
            if(count > 0){
                double tmp = blurredErrorMap->at<double>(i, j);
                mMapData[xmin+i][zmin+j].baError = tmp / (double)count;
            }
        }
    }
    delete tmpErrorMap;
    delete blurredErrorMap;
    
    // show map data
    ShowColor2dMap();
}


void Map2d::UpdateErrorWholeMap(EdgeContainer& activeEdges){
    
    std::cout << "[UpdateError] " << "Updating WHOLE Map !!!" << std::endl;
    int xScale = m2dScale.xScale;
    int zScale = m2dScale.zScale;

    // fill temp error map
    cv::Mat* tmpErrorMap = new cv::Mat(cv::Mat::zeros(xScale, zScale, 6)); // CV_64FC1
    for (int k = 0; k < static_cast<int>(activeEdges.size()); ++k) {
        // get edge
        g2o::OptimizableGraph::Edge* e = activeEdges[k];
        if(e == nullptr){
            continue;
        }
        
        // get coordinate
        ConstVtxMapPoint vMapPoint = static_cast<ConstVtxMapPoint>(e->vertices()[0]);
        Eigen::Vector3d pos3d = vMapPoint->estimate();
        Eigen::Vector2d pos2d = Get2dPos(pos3d);
        int x = (int)pos2d(0);
        int z = (int)pos2d(1);
        if(x < 0 || x > xScale){
            continue;
        }
        if(z < 0 || z > zScale){
            continue;
        }
        
        // get error
        double errorSqr;
        if (e->robustKernel()) {
            Eigen::Vector3d tmp;
            e->robustKernel()->robustify(e->chi2(), tmp);
            errorSqr = tmp[0];
        }
        else{
            errorSqr = e->chi2();
        }
        
        // assign
        tmpErrorMap->at<double>(x, z) += std::sqrt(errorSqr);
    }
    
    // blur
    cv::Mat* blurredErrorMap = new cv::Mat(cv::Mat::zeros(zScale, xScale, 6)); // CV_64FC1
    GaussianBlur(*tmpErrorMap, *blurredErrorMap, cv::Size(11, 11), 0, 0);
    
    // update map data
    std::unique_lock<std::mutex> mapUpdateLock(mMutexMapUpdate); // add lock
    for(int i = 0; i < xScale; i++){
        for(int j = 0; j < zScale; j++){
            double tmp = blurredErrorMap->at<double>(i, j);
            mMapData[i][j].baError = tmp;
        }
    }
    delete tmpErrorMap;
    delete blurredErrorMap;
    std::cout << "[UpdateError] " << "WHOLE Map updated!!!" << std::endl;
    
    // show map data
    ShowColor2dMap();
}



/*** graphicization ***/

cv::Mat* Map2d::GetOccupyMapImage(){
    int xScale = m2dScale.xScale;
    int zScale = m2dScale.zScale;
    cv::Mat* binMap = new cv::Mat(cv::Mat::zeros(zScale, xScale, 0)); // cv::CV_8UC1
    for(int i = 0; i < xScale; i++){
        for(int j = 0; j < zScale; j++){
            GridType tmp = mMapData[i][j].type;
            if(GridType::GRID_OCCUPIED == tmp){
                binMap->at<uchar>(zScale-j-1, i) = 255;
            }
            else{
                binMap->at<uchar>(zScale-j-1, i) = 0;
            }
        }
    }
    return binMap;
}

cv::Mat* Map2d::GetErrorMapImage(){
    // get max error
    int xScale = m2dScale.xScale;
    int zScale = m2dScale.zScale;
    double maxError = -1;
    for(int i = 0; i < xScale; i++){
        for(int j = 0; j < zScale; j++){
            double tmp = mMapData[i][j].baError;
            if(tmp > maxError){
                maxError = tmp;
            }
        }
    }
    if(maxError <= 0){
        throw "Fatal error: no error on map.";
    }
    mMaxError = maxError;
    std::cout << "[ErrorMap] " << "Max error: " << maxError << std::endl;
    
    // generate map
    cv::Mat* errorMapB = new cv::Mat(cv::Mat::zeros(zScale, xScale, 0)); // cv::CV_8UC1
    for(int i = 0; i < xScale; i++){
        for(int j = 0; j < zScale; j++){
            double tmp = mMapData[i][j].baError;
            if(tmp > 0){
                tmp = tmp / mMaxError;
                errorMapB->at<uchar>(zScale-j-1, i) = (int) (tmp * 255);
            }
            else{
                errorMapB->at<uchar>(zScale-j-1, i) = 0;
            }
        }
    }
    return errorMapB;
}

cv::Mat* Map2d::GetErrorMapImageColor(cv::Mat* errorMapB){
    int xScale = m2dScale.xScale;
    int zScale = m2dScale.zScale;
    if(errorMapB == nullptr){
        errorMapB = GetErrorMapImage();
    }
    
    // apply color
    cv::Mat* colorMap3B = new cv::Mat(cv::Mat::zeros(zScale, xScale, 16)); // cv::CV_8UC3
    applyColorMap(*errorMapB, *colorMap3B, cv::COLORMAP_JET);
    
    /*
    // set obstacle to white
    for(int i = 0; i < xScale; i++){
        for(int j = 0; j < zScale; j++){
            GridType tmp = mMapData[i][j].type;
            if(GridType::GRID_OCCUPIED == tmp){
                colorMap3B->at<cv::Vec3b>(zScale-j-1, i) = cv::Vec3b(255, 255, 255);
            }
        }
    }
    */
    return colorMap3B;
}



/*** show image ***/

void Map2d::ShowOccupy1Map(){
    
    // get obstacle scale
    int xScale = m2dScale.xScale;
    int zScale = m2dScale.zScale;
    int maxOccupyX = -1;
    int maxOccupyZ = -1;
    int minOccupyX = xScale;
    int minOccupyZ = zScale;
    double maxError = -1;
    for(int i = 0; i < xScale; i++){
        for(int j = 0; j < zScale; j++){
            GridType type = mMapData[i][j].type;
            if(type == GridType::GRID_OCCUPIED ||
                type == GridType::GRID_CAMERA){
                if(i > maxOccupyX) maxOccupyX = i;
                if(i < minOccupyX) minOccupyX = i;
                if(j > maxOccupyZ) maxOccupyZ = j;
                if(j < minOccupyZ) minOccupyZ = j;
            }
        }
    }

    cv::Mat* occupyMap = GetOccupyMapImage();
    
    // draw camera
    Eigen::Vector2d cam2dPos = Get2dPos(mCamPosf(0), mCamPosf(1));
    int xCam = cam2dPos(0);
    int zCam = cam2dPos(1);
    Eigen::Vector2d arrowHead = mCamDirectionf + mCamPosf;
    Eigen::Vector2d arrowHead2dPos = Get2dPos(arrowHead(0), arrowHead(1));
    int xArrowHead = arrowHead2dPos(0);
    int zArrowHead = arrowHead2dPos(1);
    cv::Point start(xCam, zScale-zCam-1); // column first !!!! OpenCV SBSBSBSBSBSBSB !!!!
    cv::Point end(xArrowHead, zScale-zArrowHead-1); // column first !!!! 
    cv::arrowedLine(*occupyMap, start, end, 255, 2, 8, 0, 0.2);
      // src, pt1(start), pt2(end), color, thickness, line_type， shift, tipLength
    minOccupyX = min(minOccupyX, min(xCam, xArrowHead));
    minOccupyZ = min(minOccupyZ, min(zCam, zArrowHead));
    maxOccupyX = max(maxOccupyX, max(xCam, xArrowHead));
    maxOccupyZ = max(maxOccupyZ, max(zCam, zArrowHead));
    
    // crop
    minOccupyX = std::max(minOccupyX, 0);
    maxOccupyX = std::min(maxOccupyX, occupyMap->cols);
    minOccupyZ = std::max(minOccupyZ, zScale+1-occupyMap->rows);
    maxOccupyZ = std::min(maxOccupyZ, zScale+1);
    cv::Mat cropOccupyMap = (*occupyMap)(
        cv::Range(zScale-maxOccupyZ+1, zScale-minOccupyZ+1),
        cv::Range(minOccupyX, maxOccupyX)
    );
     
    //cv::Mat cropOccupyMap = *occupyMap;
    ShowImage(cropOccupyMap, "Occupation map");
    delete occupyMap;
}

void Map2d::ShowColor2dMap(){
    
    // get obstacle scale
    int xScale = m2dScale.xScale;
    int zScale = m2dScale.zScale;
    int maxOccupyX = -1;
    int maxOccupyZ = -1;
    int minOccupyX = xScale;
    int minOccupyZ = zScale;
    double maxError = -1;
    for(int i = 0; i < xScale; i++){
        for(int j = 0; j < zScale; j++){
            GridType type = mMapData[i][j].type;
            if(type == GridType::GRID_OCCUPIED ||
                type == GridType::GRID_CAMERA){
                if(i > maxOccupyX) maxOccupyX = i;
                if(i < minOccupyX) minOccupyX = i;
                if(j > maxOccupyZ) maxOccupyZ = j;
                if(j < minOccupyZ) minOccupyZ = j;
            }
        }
    }
    
    // get error map
    cv::Mat* errMapB = nullptr;
    cv::Mat* colorErrMap = GetErrorMapImageColor(errMapB);
    
    // draw camera
    Eigen::Vector2d cam2dPos = Get2dPos(mCamPosf(0), mCamPosf(1));
    int xCam = cam2dPos(0);
    int zCam = cam2dPos(1);
    Eigen::Vector2d arrowHead = mCamDirectionf + mCamPosf;
    Eigen::Vector2d arrowHead2dPos = Get2dPos(arrowHead(0), arrowHead(1));
    int xArrowHead = arrowHead2dPos(0);
    int zArrowHead = arrowHead2dPos(1);
    cv::Point start(xCam, zScale-zCam-1); // column first !!!! OpenCV SBSBSBSBSBSBSB !!!!
    cv::Point end(xArrowHead, zScale-zArrowHead-1); // column first !!!! 
    cv::arrowedLine(*colorErrMap, start, end, cv::Scalar(0,255,0), 2, 8, 0, 0.2);
      // src, pt1(start), pt2(end), color, thickness, line_type， shift, tipLength
    minOccupyX = min(minOccupyX, min(xCam, xArrowHead));
    minOccupyZ = min(minOccupyZ, min(zCam, zArrowHead));
    maxOccupyX = max(maxOccupyX, max(xCam, xArrowHead));
    maxOccupyZ = max(maxOccupyZ, max(zCam, zArrowHead));
    
    // crop
    minOccupyX = std::max(minOccupyX, 0);
    maxOccupyX = std::min(maxOccupyX, colorErrMap->cols);
    minOccupyZ = std::max(minOccupyZ, zScale+1-colorErrMap->rows);
    maxOccupyZ = std::min(maxOccupyZ, zScale+1);
    cv::Mat cropColorMap = (*colorErrMap)(
        cv::Range(zScale-maxOccupyZ+1, zScale-minOccupyZ+1),
        cv::Range(minOccupyX, maxOccupyX)
    );
    // show
    ShowImage(cropColorMap, "Color Error Map");
    delete errMapB;
    delete colorErrMap;
}

void ShowImage(cv::Mat& image, const char* title){
    cv::imshow(title, image);
}



} // namespace ORB_SLAM2
