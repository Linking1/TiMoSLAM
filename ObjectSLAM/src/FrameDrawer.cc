/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    // cout << "draw state: " << state << endl;
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                // cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                //         cv::Scalar(0,255,0));
            }
        }
        // std::cout << "mvpObj2Ds size: " << mvpObj2Ds.size() <<std::endl;

        for(int bbi=0; bbi<(int)mvpObj2Ds.size();bbi++)
        {
            // [QUA SLAM]绘制目标检测框
            cv::rectangle(im, mvpObj2Ds[bbi]->GetRect(), (0,0,255));
            
            Eigen::Matrix<double, 12, 3> cubePoints;
            mvpObj2Ds[bbi]->pObj3D->GetCubePoints(cubePoints);
            vector<cv::Point2f> frameCubePoints;
            for(int i=0; i<12; i++)
            {
                cv::Point2f pt1(cubePoints(i, 0)/cubePoints(i, 2), cubePoints(i, 1)/cubePoints(i, 2));
                frameCubePoints.push_back(pt1);
            }

            // 绘制顶部矩形(0, 1, 2, 3)
            cv::Scalar cube_color1(255,0,0);
            cv::Scalar cube_color2(0,255,0);
            cv::Scalar cube_color3(0,0,255);
            cv::Scalar cube_color4(255,255,0);
            int cube_thickness = 3;
            cv::line(im, frameCubePoints[0], frameCubePoints[1], cube_color1, cube_thickness);
            cv::line(im, frameCubePoints[1], frameCubePoints[2], cube_color1, cube_thickness);
            cv::line(im, frameCubePoints[2], frameCubePoints[3], cube_color1, cube_thickness);
            cv::line(im, frameCubePoints[3], frameCubePoints[0], cube_color1, cube_thickness);

            // 绘制底部矩形(4, 5, 6, 7)
            cv::line(im, frameCubePoints[4], frameCubePoints[5], cube_color2, cube_thickness);
            cv::line(im, frameCubePoints[5], frameCubePoints[6], cube_color2, cube_thickness);
            cv::line(im, frameCubePoints[6], frameCubePoints[7], cube_color2, cube_thickness);
            cv::line(im, frameCubePoints[7], frameCubePoints[4], cube_color2, cube_thickness);

            // 绘制左侧矩形(0, 3, 7, 4)
            cv::line(im, frameCubePoints[0], frameCubePoints[3], cube_color3, cube_thickness);
            cv::line(im, frameCubePoints[3], frameCubePoints[7], cube_color3, cube_thickness);
            cv::line(im, frameCubePoints[7], frameCubePoints[4], cube_color3, cube_thickness);
            cv::line(im, frameCubePoints[4], frameCubePoints[0], cube_color3, cube_thickness);

            // 绘制右侧矩形(5, 1, 2, 6)
            cv::line(im, frameCubePoints[5], frameCubePoints[1], cube_color4, cube_thickness);
            cv::line(im, frameCubePoints[1], frameCubePoints[2], cube_color4, cube_thickness);
            cv::line(im, frameCubePoints[2], frameCubePoints[6], cube_color4, cube_thickness);
            cv::line(im, frameCubePoints[6], frameCubePoints[5], cube_color4, cube_thickness);

            // 绘制坐标轴
            // cv::line(im, frameCubePoints[8], frameCubePoints[9], cv::Scalar(255, 255, 0), cube_thickness);
            // cv::line(im, frameCubePoints[8], frameCubePoints[10], cv::Scalar(0, 255, 0), cube_thickness);
            // cv::line(im, frameCubePoints[8], frameCubePoints[11], cv::Scalar(0, 0, 255), cube_thickness);


        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }

        for(int bbi=0; bbi<(int)mvpObj2Ds.size();bbi++)
        {
            // [QUA SLAM]绘制目标检测框
            // [QUA SLAM]绘制目标检测线
            vector<vector<cv::Point2f>> emvLines = mvpObj2Ds[bbi]->GetLines();

            Eigen::Matrix<double, 12, 3> cubePoints;
            mvpObj2Ds[bbi]->pObj3D->GetCubePoints(cubePoints);
            
            vector<cv::Point2f> frameCubePoints;
            for(int i=0; i<12; i++)
            {
                cv::Point2f pt1(cubePoints(i, 0)/cubePoints(i, 2), cubePoints(i, 1)/cubePoints(i, 2));
                frameCubePoints.push_back(pt1);
            }


            // 坐标轴点
            vector<cv::Point2f> frameDire;
            cv::Point2f pto();
            
            // 绘制顶部矩形(0, 1, 2, 3)
            cv::Scalar cube_color1(255,0,0);
            cv::Scalar cube_color2(0,255,0);
            cv::Scalar cube_color3(0,0,255);
            cv::Scalar cube_color4(255,255,0);
            int cube_thickness = 3;
            cv::line(im, frameCubePoints[0], frameCubePoints[1], cube_color1, cube_thickness);
            cv::line(im, frameCubePoints[1], frameCubePoints[2], cube_color1, cube_thickness);
            cv::line(im, frameCubePoints[2], frameCubePoints[3], cube_color1, cube_thickness);
            cv::line(im, frameCubePoints[3], frameCubePoints[0], cube_color1, cube_thickness);

            // 绘制底部矩形(4, 5, 6, 7)
            cv::line(im, frameCubePoints[4], frameCubePoints[5], cube_color2, cube_thickness);
            cv::line(im, frameCubePoints[5], frameCubePoints[6], cube_color2, cube_thickness);
            cv::line(im, frameCubePoints[6], frameCubePoints[7], cube_color2, cube_thickness);
            cv::line(im, frameCubePoints[7], frameCubePoints[4], cube_color2, cube_thickness);

            // 绘制左侧矩形(0, 3, 7, 4)
            cv::line(im, frameCubePoints[0], frameCubePoints[3], cube_color3, cube_thickness);
            cv::line(im, frameCubePoints[3], frameCubePoints[7], cube_color3, cube_thickness);
            cv::line(im, frameCubePoints[7], frameCubePoints[4], cube_color3, cube_thickness);
            cv::line(im, frameCubePoints[4], frameCubePoints[0], cube_color3, cube_thickness);

            // 绘制右侧矩形(5, 1, 2, 6)
            cv::line(im, frameCubePoints[5], frameCubePoints[1], cube_color4, cube_thickness);
            cv::line(im, frameCubePoints[1], frameCubePoints[2], cube_color4, cube_thickness);
            cv::line(im, frameCubePoints[2], frameCubePoints[6], cube_color4, cube_thickness);
            cv::line(im, frameCubePoints[6], frameCubePoints[5], cube_color4, cube_thickness);
            
            // 绘制优化后的3D目标的投影结果
            Eigen::Matrix<double, 8, 2> frameCubePointsOpObj3D;
            frameCubePointsOpObj3D = mvpObj2Ds[bbi]->pObj3D->framePointsx;
            // cout << "frameCubePointsOpObj3D: " << frameCubePointsOpObj3D << endl;
            vector<cv::Point2f> frameCubePointsOp;
            for(int i=0; i<8; i++)
            {
                frameCubePointsOp.push_back(cv::Point2f(frameCubePointsOpObj3D(i,0), frameCubePointsOpObj3D(i,1)));
            }

        }
        // [QUA SLAM]绘制目标3D模型
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);
    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    timestamp = pTracker->mCurrentFrame.mTimeStamp;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;
    mvpObj2Ds = pTracker->mCurrentFrame.mvpObj2Ds;


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
