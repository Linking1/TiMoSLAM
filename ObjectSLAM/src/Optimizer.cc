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

#include "Optimizer.h"
#include <math.h>

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
// #include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

#include<Eigen/StdVector>

#include "Converter.h"

#include<mutex>

namespace ORB_SLAM2
{

bool Optimizer::LSDTransGBundleAdjustemnt(Map* pMap, Frame &CurFrame, bool tosave, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<KeyFrame*> vpKFs1 = pMap->GetAllKeyFrames();
    vector<KeyFrame*> vpKFs;
    int WLength = 50;
    if((int)vpKFs1.size() > WLength)
    {
        vpKFs.assign(vpKFs1.end()-WLength, vpKFs1.end());
    }
    else
    {
        vpKFs.assign(vpKFs1.begin(), vpKFs1.end());
    }
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    vector<Obj3D*> vpObj = pMap->GetAllObj3D();
    return LSDTransBundleAdjustment(vpKFs,vpMP,vpObj, CurFrame, tosave, nIterations, pbStopFlag, nLoopKF, bRobust);
}

bool Optimizer::LSDTransBundleAdjustment(const std::vector<KeyFrame*> &vpKFs, const std::vector<MapPoint*> &vpMP, const vector<Obj3D*> &vpObj, Frame &CurFrame, bool tosave,
                                 int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    g2o::SparseOptimizer optimizer;

    g2o::BlockSolver<g2o::BlockSolverTraits<3,6>>::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver<g2o::BlockSolverTraits<3,6>>::PoseMatrixType>();

    g2o::BlockSolver<g2o::BlockSolverTraits<3,6>> * solver_ptr = new g2o::BlockSolver<g2o::BlockSolverTraits<3,6>>(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        VertexCam *vCam = new VertexCam();

        cv::Mat pKFR = pKF->GetRotation();
        for(int ri=0; ri<3; ri++)
        {
            for(int rj=0; rj<3; rj++)
            {
                vCam->Rcw(ri, rj) = pKFR.at<float>(ri,rj);
            }
        }
        for(int ri=0; ri<4; ri++)
        {
            for(int rj=0; rj<6; rj++)
            {
                vCam->LineConstraint(ri, rj) = pKF->mvpObj2Ds[0]->pObj3D->LineConstraint(ri, rj);
            }
        }
        vCam->setEstimate(Converter::toVector3d(pKF->GetTranslation()));
        vCam->setId(pKF->mnId);
        vCam->setFixed(pKF->mnId==0);
        optimizer.addVertex(vCam);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    // 创建当前帧顶点

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);
    const float thHuberObject = sqrt(900);

    // Set MapPoint vertices
    long unsigned int maxMPid = 0;
    

    // 设置目标节点
    for(size_t i=0; i<vpObj.size(); i++)
    {
        Obj3D* _pObj = vpObj[i];
        
        VertexCuboid* vObj = new VertexCuboid();
        cuboidS cuboidTemp;

        cuboidTemp.setRotation(_pObj->R_3D);
        
        cuboidTemp.setTranslation(_pObj->Center);
        cuboidTemp.setScale(_pObj->Axes);
        vObj->setEstimate(cuboidTemp);
        const int id = _pObj->mnId+maxKFid+maxMPid+2+1;
        vObj->setFixed(false);
        vObj->setId(id);
        vObj->setMarginalized(true);
        optimizer.addVertex(vObj);
        // 构建边
        for(size_t kf_i=0; kf_i<vpKFs.size(); kf_i++)
        {
            KeyFrame* _pKFi = vpKFs[kf_i];
            EdgeLSDCuboidProj* e = new EdgeLSDCuboidProj();
            e->Kalib = Converter::toMatrix3d(_pKFi->mK);
            e->getCubePoints(_pKFi->mvpObj2Ds[0]->pObj3D->LineConstraint);
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(_pKFi->mnId)));
            
            e->setMeasurement(Vector6d::Zero());
            // 信息矩阵的维数与观测的维数一致
            e->setInformation(Eigen::Matrix<double, 6, 6>::Identity()*1.0);
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber2D);

            optimizer.addEdge(e);
        }

        
    }
    
    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(40);

    // 计算误差的边
    EdgeLSDCuboidProj* e = new EdgeLSDCuboidProj();
    e->Kalib = Converter::toMatrix3d(CurFrame.mK);
    e->getCubePoints(CurFrame.mvpObj2Ds[0]->pObj3D->LineConstraint);
    int id = vpObj[0]->mnId+maxKFid+maxMPid+2+1;
    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
    // e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(maxKFid+1)));
    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(maxKFid)));
    // e->setMeasurement(_pKFi->mvpObj2Ds[0]->evecCenter);
    e->setMeasurement(Vector6d::Zero());
    // const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
    e->setInformation(Eigen::Matrix<double, 6, 6>::Identity()*1.0);
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);
    rk->setDelta(thHuber3D);

    if(e->computeError1()>10 && tosave)
    {
        cout << "tosave: " << tosave << endl;
        return false;
    }

    // 将当前的优化结果写入类对象中

    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        VertexCam* vSE3Cur = static_cast<VertexCam*>(optimizer.vertex(pKF->mnId));
        Eigen::Vector3d vSE3Trans = vSE3Cur->estimate();
        cv::Mat tcw_ = Converter::toCvMat(vSE3Trans);
        pKF->SetTranslation(tcw_);
    }

    VertexCam* vSE3Cur = static_cast<VertexCam*>(optimizer.vertex(maxKFid));


    Eigen::Vector3d vSE3Trans = vSE3Cur->estimate();
    cv::Mat tcw_ = Converter::toCvMat(vSE3Trans);
    // 当前帧位姿
    tcw_.copyTo(CurFrame.mtcw);
    tcw_.copyTo(CurFrame.mTcw.rowRange(0,3).col(3));
    cv::Mat Rcw = CurFrame.mTcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = CurFrame.mTcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat Ow = -Rwc*tcw;
    cout << "CurFrame.mTcw: " << CurFrame.mTcw << endl;

    // 根据目标的平移和尺寸更新目标在世界坐标系下的八个顶点位置
    // Objects
    for(size_t i=0; i<vpObj.size(); i++)
    {
        Obj3D* _pObj = vpObj[i];

        VertexCuboid* vObj = static_cast<VertexCuboid*>(optimizer.vertex(_pObj->mnId+maxKFid+maxMPid+2+1));
        cuboidS cuboidTemp = vObj->estimate();
        // _pObj->R_3D = cuboidTemp.mRow;
        _pObj->Center = cuboidTemp.mtow;
        _pObj->Axes = cuboidTemp.scale;
        // 获取世界坐标系下的八个顶点坐标
        Eigen::Matrix<double, 8, 3> cubePoints1 = cuboidTemp.getPoints();
        _pObj->cubePoints = cubePoints1;
        Eigen::Matrix<double, 8, 4> cubePoints;
        cubePoints.setZero();
        cubePoints.block(0,0,8,3) = cuboidTemp.getPoints1();
        for(int fi=0; fi<8; fi++)
        {
            cubePoints(fi,3) = 1;
        }
        
        // 将优化后的目标投影到当前图像帧
        cv::Mat Tcw_;
        CurFrame.mTcw.copyTo(Tcw_);
        Eigen::Matrix4d Cur_Tcw;
        Cur_Tcw <<  Tcw_.at<float>(0,0), Tcw_.at<float>(0,1), Tcw_.at<float>(0,2), Tcw_.at<float>(0,3),
                    Tcw_.at<float>(1,0), Tcw_.at<float>(1,1), Tcw_.at<float>(1,2), Tcw_.at<float>(1,3),
                    Tcw_.at<float>(2,0), Tcw_.at<float>(2,1), Tcw_.at<float>(2,2), Tcw_.at<float>(2,3),
                    Tcw_.at<float>(3,0), Tcw_.at<float>(3,1), Tcw_.at<float>(3,2), Tcw_.at<float>(3,3);
        Eigen::Matrix<double, 3, 8> cubePointsInCurCam = Cur_Tcw.block(0,0,3,4)*cubePoints.transpose();
        // 投影到图像帧
        Eigen::Matrix3d Cur_K;
        Cur_K << CurFrame.mK.at<float>(0,0), CurFrame.mK.at<float>(0,1), CurFrame.mK.at<float>(0,2),
                 CurFrame.mK.at<float>(1,0), CurFrame.mK.at<float>(1,1), CurFrame.mK.at<float>(1,2),
                 CurFrame.mK.at<float>(2,0), CurFrame.mK.at<float>(2,1), CurFrame.mK.at<float>(2,2);
        Eigen::Matrix<double, 3, 8> cubePointsx =  Cur_K * cubePointsInCurCam;
        Eigen::Matrix<double, 2, 8> framePointsx;
        // Eigen::Matrix<double, 2, 8> framePointsx1 = CurFrame.mvpObj2Ds[0]->pObj3D->framePointsx.transpose();
        for(int fi=0; fi<8; fi++)
        {
            framePointsx(0,fi) = cubePointsx(0, fi)/cubePointsx(2, fi);
            framePointsx(1,fi) = cubePointsx(1, fi)/cubePointsx(2, fi);
        }

        // 显示与误差计算之间的索引对应关系
        int displayAndErrorID[8] = {4,1,0,7,5,2,3,6};
        for(int fi=0; fi<8; fi++)
        {
            CurFrame.mvpObj2Ds[0]->pObj3D->framePointsx.block(fi,0,1,2) = framePointsx.block(0,displayAndErrorID[fi],2,1).transpose();
        }
        // // 在当前帧中计算误差
        Eigen::Matrix<double, 4, 6> LineConstraint = CurFrame.mvpObj2Ds[0]->pObj3D->LineConstraint;
		for(int fi=0; fi<6; fi++)
		{
			double LineConstraintMo = pow(pow(LineConstraint(0,fi),2)+pow(LineConstraint(1,fi),2), 0.5);
			for(int fj=0; fj<3; fj++)
			{
				LineConstraint(fj,fi) /= LineConstraintMo;
			}
		}
		Vector6d _error1;
		_error1.setZero();

		for(int fi=0; fi<6; fi++)
		{
			double LineConstraintMo = pow(pow(LineConstraint(0,fi),2)+pow(LineConstraint(1,fi),2), 0.5);
			_error1(fi) = (framePointsx(0, LineConstraint(3,fi))*LineConstraint(0,fi)+framePointsx(1, LineConstraint(3,fi))*LineConstraint(1,fi)+LineConstraint(2,fi))/LineConstraintMo;
		}

        cout << "framePointsx 3D: \n" << framePointsx << endl;
		cout << "LineConstraint 3D: \n" << LineConstraint << endl;

        cout << "error1: \n" << _error1 << endl;
        
    }
    return true;//cubeProError<10000;
}







bool Optimizer::SE3TransGBundleAdjustemnt1(Map* pMap, Frame &CurFrame, bool tosave, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<KeyFrame*> vpKFs1 = pMap->GetAllKeyFrames();
    vector<KeyFrame*> vpKFs;
    int WLength = 800;
    if((int)vpKFs1.size() > WLength)
    {
        vpKFs.assign(vpKFs1.end()-WLength, vpKFs1.end());
    }
    else
    {
        vpKFs.assign(vpKFs1.begin(), vpKFs1.end());
    }
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    vector<Obj3D*> vpObj = pMap->GetAllObj3D();
    return SE3TransBundleAdjustment(vpKFs,vpMP,vpObj, CurFrame, tosave, nIterations, pbStopFlag, nLoopKF, bRobust);
    return true;
}

bool Optimizer::SE3TransBundleAdjustment(const std::vector<KeyFrame*> &vpKFs, const std::vector<MapPoint*> &vpMP, const vector<Obj3D*> &vpObj, Frame &CurFrame, bool tosave,
                                 int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    g2o::SparseOptimizer graph;
	g2o::BlockSolverX::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	graph.setAlgorithm(solver);
	graph.setVerbose(false);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices    
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        cout << "pKF id: " << pKF->mnId << endl;
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setId(pKF->mnId);
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setFixed(pKF->mnId==0);
        graph.addVertex(vSE3);
        if(pKF->mnId > maxKFid)
        {
            maxKFid = pKF->mnId;
        }
    }
    
    // 设置相机与相机之间的边
    for(int i=0; i<(int)vpKFs.size()-1; i++)
    {
        KeyFrame* pKF1;
        KeyFrame* pKF2;
        // 寻找两个关键帧
        for(int j=0; j<(int)vpKFs.size(); j++)
        {
            if(vpKFs[j]->mnId == i)
            {
                pKF1 = vpKFs[j];
            }
            if(vpKFs[j]->mnId == i+1)
            {
                pKF2 = vpKFs[j];
            }
        }
        g2o::SE3Quat odom_val;
        if(i > 1)
        {
			g2o::SE3Quat prev_pose_Tcw = Converter::toSE3Quat(pKF1->GetPose());
            g2o::SE3Quat prev_prev_pose_Tcw = Converter::toSE3Quat(pKF2->GetPose());
            odom_val = prev_pose_Tcw * prev_prev_pose_Tcw.inverse();
        }
        
        EdgeSE3Expmap1 *e = new EdgeSE3Expmap1();
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(graph.vertex(pKF1->mnId)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(graph.vertex(pKF2->mnId)));
        e->setMeasurement(odom_val);
        e->setId(maxKFid+i);
        Vector6d inv_sigma;
        inv_sigma << 1, 1, 1, 1, 1, 1;
        inv_sigma = inv_sigma * 1.0;
        Eigen::Matrix<double, 6, 6> info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
        e->setInformation(info);
        graph.addEdge(e);
    }

    // Optimize!
    graph.initializeOptimization();
    graph.optimize(5);
    // 修改优化结果
    for (int i = 0; i < (int)vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(graph.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }
    return true;
}


bool Optimizer::CurFrameGBundleAdjustemnt(Map* pMap, Frame &CurFrame, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<KeyFrame*> vpKFs1 = pMap->GetAllKeyFrames();
    vector<KeyFrame*> vpKFs;
    int WLength = 30;

        vpKFs.assign(vpKFs1.begin(), vpKFs1.end());
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    vector<Obj3D*> vpObj = pMap->GetAllObj3D();
    return CurFrameBundleAdjustment(vpKFs,vpMP,vpObj, CurFrame, nIterations,pbStopFlag, nLoopKF, bRobust);
}

bool Optimizer::CurFrameBundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP, const vector<Obj3D*> &vpObj, Frame &CurFrame,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    // 创建当前帧顶点
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(CurFrame.mTcw));
    vSE3->setId(maxKFid+1);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);
    const float thHuberObject = sqrt(900);

    // Set MapPoint vertices
    long unsigned int maxMPid = 0;

    // 设置目标节点
    // vector<EdgeSE3CuboidProj*> vpEdgesCameraObject;
    for(size_t i=0; i<vpObj.size(); i++)
    {

        Obj3D* _pObj = vpObj[i];
        for(int cpi=0; cpi<8; cpi++)
        {
            g2o::VertexSBAPointXYZ* vObj = new g2o::VertexSBAPointXYZ();
            // vObj->setEstimate(_pObj->Center);
            vObj->setEstimate(_pObj->cubePoints.block(cpi,0,1,3).transpose());
            const int id = _pObj->mnId*8+cpi+maxKFid+maxMPid+2+1;
            vObj->setFixed(true);
            vObj->setId(id);
            vObj->setMarginalized(true);
            optimizer.addVertex(vObj);
            for(size_t kf_i=0; kf_i<vpKFs.size(); kf_i++)
            {
                KeyFrame* _pKFi = vpKFs[kf_i];
                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(_pKFi->mnId)));
                e->setMeasurement(_pKFi->mvpObj2Ds[0]->pObj3D->framePointsx.block(cpi,0,1,2).transpose());
                e->setInformation(Eigen::Matrix2d::Identity()*1.0);
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuber2D);

                e->fx = _pKFi->fx;
                e->fy = _pKFi->fy;
                e->cx = _pKFi->cx;
                e->cy = _pKFi->cy;

                optimizer.addEdge(e);
            }
            // 添加与当前帧之间的边
            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(maxKFid+1)));
            e->setMeasurement(CurFrame.mvpObj2Ds[0]->pObj3D->framePointsx.block(cpi,0,1,2).transpose());
            e->setInformation(Eigen::Matrix2d::Identity()*1.0);
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber2D);

            e->fx = CurFrame.fx;
            e->fy = CurFrame.fy;
            e->cx = CurFrame.cx;
            e->cy = CurFrame.cy;

            optimizer.addEdge(e);
        }
        
    }
    

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    // 评价当前帧的误差

    //Keyframes
    g2o::VertexSE3Expmap* vSE3Cur = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(maxKFid+1));
    g2o::SE3Quat SE3quat = vSE3Cur->estimate();
    cv::Mat Tcw_ = Converter::toCvMat(SE3quat);
    // 当前帧位姿
    Tcw_.copyTo(CurFrame.mTcw);
    cv::Mat Rcw = CurFrame.mTcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = CurFrame.mTcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat Ow = -Rwc*tcw;
    Eigen::Matrix4d Cur_Tcw;
    Cur_Tcw <<  Tcw_.at<float>(0,0), Tcw_.at<float>(0,1), Tcw_.at<float>(0,2), Tcw_.at<float>(0,3),
                Tcw_.at<float>(1,0), Tcw_.at<float>(1,1), Tcw_.at<float>(1,2), Tcw_.at<float>(1,3),
                Tcw_.at<float>(2,0), Tcw_.at<float>(2,1), Tcw_.at<float>(2,2), Tcw_.at<float>(2,3),
                Tcw_.at<float>(3,0), Tcw_.at<float>(3,1), Tcw_.at<float>(3,2), Tcw_.at<float>(3,3);
    Eigen::Matrix3d Cur_K;
    Cur_K << CurFrame.mK.at<float>(0,0), CurFrame.mK.at<float>(0,1), CurFrame.mK.at<float>(0,2),
             CurFrame.mK.at<float>(1,0), CurFrame.mK.at<float>(1,1), CurFrame.mK.at<float>(1,2),
             CurFrame.mK.at<float>(2,0), CurFrame.mK.at<float>(2,1), CurFrame.mK.at<float>(2,2);
    

    // 目标顶点的误差
    Obj3D* _pObj = vpObj[0];

    // 重投影误差
    double cubeProError = 0.0;
    // 将地图目标顶点投影到当前相机坐标系
    // _pObj->cubePoints.block(cpi,0,1,3);
    Eigen::Matrix<double, 3, 8> cubePointsInCurCam = Cur_Tcw.block(0,0,3,4)*_pObj->cubePoints.transpose();
    // 投影到图像帧
    Eigen::Matrix<double, 3, 8> cubePointsx =  Cur_K * cubePointsInCurCam;
    Eigen::Matrix<double, 2, 8> framePointsx;
    Eigen::Matrix<double, 2, 8> framePointsx1 = CurFrame.mvpObj2Ds[0]->pObj3D->framePointsx.transpose();
    for(int i=0; i<8; i++)
    {
        framePointsx(0,i) = cubePointsx(0, i)/cubePointsx(2, i);
        framePointsx(1,i) = cubePointsx(0, i)/cubePointsx(2, i);
        double curCubeError = pow(pow(framePointsx(0,i)-framePointsx1(0,i), 2)+pow(framePointsx(1,i)-framePointsx1(1,i), 2),0.5);
        cubeProError += curCubeError;
    }
    return true;//cubeProError<10000;
}


void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<KeyFrame*> vpKFs1 = pMap->GetAllKeyFrames();
    vector<KeyFrame*> vpKFs;
    int WLength = 200;
    if((int)vpKFs1.size() > WLength)
    // if((int)vpKFs1.size() > WLength)
    {
        vpKFs.assign(vpKFs1.end()-WLength, vpKFs1.end());
        vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
        vector<Obj3D*> vpObj = pMap->GetAllObj3D();
        // vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
        BundleAdjustmentNoCube(vpKFs,vpMP,vpObj, nIterations,pbStopFlag, nLoopKF, bRobust);
    }
    else
    {
        vpKFs.assign(vpKFs1.begin(), vpKFs1.end());
        vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
        vector<Obj3D*> vpObj = pMap->GetAllObj3D();
        // vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
        BundleAdjustment(vpKFs,vpMP,vpObj, nIterations,pbStopFlag, nLoopKF, bRobust);
    }
    
}

void Optimizer::BundleAdjustmentNoCube(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP, const vector<Obj3D*> &vpObj,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);
    const float thHuberObject = sqrt(900);

    // Set MapPoint vertices
    long unsigned int maxMPid = 0;

    // 设置目标节点
    for(size_t i=0; i<vpObj.size(); i++)
    {
        Obj3D* _pObj = vpObj[i];
        for(int cpi=0; cpi<8; cpi++)
        {
            g2o::VertexSBAPointXYZ* vObj = new g2o::VertexSBAPointXYZ();
            // vObj->setEstimate(_pObj->Center);
            vObj->setEstimate(_pObj->cubePoints.block(cpi,0,1,3).transpose());
            const int id = _pObj->mnId*8+cpi+maxKFid+maxMPid+2;
            vObj->setFixed(true);
            vObj->setId(id);
            vObj->setMarginalized(true);
            optimizer.addVertex(vObj);
            for(size_t kf_i=0; kf_i<vpKFs.size(); kf_i++)
            {
                KeyFrame* _pKFi = vpKFs[kf_i];
                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(_pKFi->mnId)));
                // e->setMeasurement(_pKFi->mvpObj2Ds[0]->evecCenter);
                e->setMeasurement(_pKFi->mvpObj2Ds[0]->pObj3D->framePointsx.block(cpi,0,1,2).transpose());
                // const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*1.0);
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuber2D);

                e->fx = _pKFi->fx;
                e->fy = _pKFi->fy;
                e->cx = _pKFi->cx;
                e->cy = _pKFi->cy;

                optimizer.addEdge(e);
            }
        }
        
    }
    

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    // Recover optimized data

    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
            pKF->SetPose(Converter::toCvMat(SE3quat));

    }
    
}


void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP, const vector<Obj3D*> &vpObj,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);
    const float thHuberObject = sqrt(900);

    // Set MapPoint vertices
    long unsigned int maxMPid = 0;

    // 设置目标节点
    for(size_t i=0; i<vpObj.size(); i++)
    {

        Obj3D* _pObj = vpObj[i];
        for(int cpi=0; cpi<8; cpi++)
        {
            g2o::VertexSBAPointXYZ* vObj = new g2o::VertexSBAPointXYZ();
            // vObj->setEstimate(_pObj->Center);
            vObj->setEstimate(_pObj->cubePoints.block(cpi,0,1,3).transpose());
            const int id = _pObj->mnId*8+cpi+maxKFid+maxMPid+2;
            vObj->setId(id);
            vObj->setMarginalized(true);
            optimizer.addVertex(vObj);
            for(size_t kf_i=0; kf_i<vpKFs.size(); kf_i++)
            {
                KeyFrame* _pKFi = vpKFs[kf_i];
                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(_pKFi->mnId)));
                // e->setMeasurement(_pKFi->mvpObj2Ds[0]->evecCenter);
                e->setMeasurement(_pKFi->mvpObj2Ds[0]->pObj3D->framePointsx.block(cpi,0,1,2).transpose());
                // const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*1.0);
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuber2D);

                e->fx = _pKFi->fx;
                e->fy = _pKFi->fy;
                e->cx = _pKFi->cx;
                e->cy = _pKFi->cy;

                optimizer.addEdge(e);
            }
        }
        
    }

    // 添加cam-cam节点
    for(int i=0; i<(int)vpKFs.size()-1; i++)
    {
        KeyFrame* pKF1;
        KeyFrame* pKF2;
        // 寻找两个关键帧
        for(int j=0; j<(int)vpKFs.size(); j++)
        {
            if(vpKFs[j]->mnId == i)
            {
                pKF1 = vpKFs[j];
            }
            if(vpKFs[j]->mnId == i+1)
            {
                pKF2 = vpKFs[j];
            }
        }
        g2o::SE3Quat odom_val;
        if(i > 1)
        {
			g2o::SE3Quat prev_pose_Tcw = Converter::toSE3Quat(pKF1->GetPose());
            g2o::SE3Quat prev_prev_pose_Tcw = Converter::toSE3Quat(pKF2->GetPose());
            odom_val = prev_pose_Tcw * prev_prev_pose_Tcw.inverse();
        }
        EdgeSE3Expmap1 *e = new EdgeSE3Expmap1();
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF1->mnId)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF2->mnId)));
        e->setMeasurement(odom_val);
        e->setId(maxKFid+i);
        Vector6d inv_sigma;
        inv_sigma << 1, 1, 1, 1, 1, 1;
        inv_sigma = inv_sigma * 2.0;
        Eigen::Matrix<double, 6, 6> info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
        e->setInformation(info);
        optimizer.addEdge(e);
    }
    

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    // Recover optimized data

    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
            pKF->SetPose(Converter::toCvMat(SE3quat));

    }

    // Objects
    for(size_t i=0; i<vpObj.size(); i++)
    {
        Obj3D* _pObj = vpObj[i];
        for(int cpi=0; cpi<8; cpi++)
        {
            g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(_pObj->mnId*8+cpi+maxKFid+maxMPid+2));
            _pObj->cubePoints.block(cpi,0,1,3) = vPoint->estimate().transpose();
        }

        KeyFrame* pKF;
        pKF = vpKFs[0];
        for(int j=0; j<vpKFs.size(); j++)
        {
            if(vpKFs[j]->mnId == maxKFid)
            {
                pKF = vpKFs[j];
            }
        }
        cv::Mat Tcw_ = pKF->GetPose();
        Eigen::Matrix4d Cur_Tcw;
        Cur_Tcw <<  Tcw_.at<float>(0,0), Tcw_.at<float>(0,1), Tcw_.at<float>(0,2), Tcw_.at<float>(0,3),
                    Tcw_.at<float>(1,0), Tcw_.at<float>(1,1), Tcw_.at<float>(1,2), Tcw_.at<float>(1,3),
                    Tcw_.at<float>(2,0), Tcw_.at<float>(2,1), Tcw_.at<float>(2,2), Tcw_.at<float>(2,3),
                    Tcw_.at<float>(3,0), Tcw_.at<float>(3,1), Tcw_.at<float>(3,2), Tcw_.at<float>(3,3);
        Eigen::Matrix3d Cur_K;
        Cur_K << pKF->mK.at<float>(0,0), pKF->mK.at<float>(0,1), pKF->mK.at<float>(0,2),
                 pKF->mK.at<float>(1,0), pKF->mK.at<float>(1,1), pKF->mK.at<float>(1,2),
                 pKF->mK.at<float>(2,0), pKF->mK.at<float>(2,1), pKF->mK.at<float>(2,2);
        

        // 目标顶点的误差

        // 重投影误差
        // 将地图目标顶点投影到当前相机坐标系
        Eigen::Matrix<double, 3, 8> cubePointsInCurCam = Cur_Tcw.block(0,0,3,4)*_pObj->cubePoints.transpose();
        // 投影到图像帧
        Eigen::Matrix<double, 3, 8> cubePointsx =  Cur_K * cubePointsInCurCam;
        Eigen::Matrix<double, 2, 8> framePointsx;
        framePointsx.setZero();
        for(int fi=0; fi<8; fi++)
        {
            framePointsx(0,fi) = cubePointsx(0, fi)/cubePointsx(2, fi);
            framePointsx(1,fi) = cubePointsx(1, fi)/cubePointsx(2, fi);

            if(abs(cubePointsx(2, fi)) < 0.001)
            {
                pKF->framePointsxOp(fi,0) = 0;
                pKF->framePointsxOp(fi,1) = 0;
            }
            else
            {
                pKF->framePointsxOp(fi,0) = framePointsx(0,fi);
                pKF->framePointsxOp(fi,1) = framePointsx(1,fi);
            }
            
        }
        
    }
    
}

// [QUA SLAM]该优化中要添加有关目标的操作，利用目标位姿进行优化
int Optimizer::PoseOptimization(Frame *pFrame)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = pFrame->N;

    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);


    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            // Monocular observation
            if(pFrame->mvuRight[i]<0)
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
            }
            else  // Stereo observation
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                //SET EDGE
                Eigen::Matrix<double,3,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                const float &kp_ur = pFrame->mvuRight[i];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaStereo);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                e->bf = pFrame->mbf;
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vnIndexEdgeStereo.push_back(i);
            }
        }

    }
    }


    if(nInitialCorrespondences<3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={10,10,10,10};    

    int nBad=0;
    for(size_t it=0; it<4; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {                
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {                
                e->setLevel(0);
                pFrame->mvbOutlier[idx]=false;
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        if(optimizer.edges().size()<10)
            break;
    }    

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pFrame->SetPose(pose);

    return nInitialCorrespondences-nBad;
}

void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap)
{    
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {                
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        //Set edges
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad())
            {                
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if(pKFi->mvuRight[mit->second]<0)
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = pKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {

    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    // Optimize again without the outliers

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    }

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations       
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }
}


void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, const bool &bFixScale)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);

    const int minFeat = 100;

    // Set KeyFrame vertices
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKF->mnId;

        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

        if(it!=CorrectedSim3.end())
        {
            vScw[nIDi] = it->second;
            VSim3->setEstimate(it->second);
        }
        else
        {
            Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
            Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if(pKF==pLoopKF)
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;
    }


    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    // Set Loop edges
    for(map<KeyFrame *, set<KeyFrame *> >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        const set<KeyFrame*> &spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi];
        const g2o::Sim3 Swi = Siw.inverse();

        for(set<KeyFrame*>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnId;
            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(*sit)<minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // Set normal edges
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Swi;

        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

        if(iti!=NonCorrectedSim3.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[nIDi].inverse();

        KeyFrame* pParentKF = pKF->GetParent();

        // Spanning tree edge
        if(pParentKF)
        {
            int nIDj = pParentKF->mnId;

            g2o::Sim3 Sjw;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

            if(itj!=NonCorrectedSim3.end())
                Sjw = itj->second;
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        const set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
        for(set<KeyFrame*>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFrame* pLKF = *sit;
            if(pLKF->mnId<pKF->mnId)
            {
                g2o::Sim3 Slw;

                LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                if(itl!=NonCorrectedSim3.end())
                    Slw = itl->second;
                else
                    Slw = vScw[pLKF->mnId];

                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3* el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        const vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFrame*>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFrame* pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnId,pKFn->mnId),max(pKF->mnId,pKFn->mnId))))
                        continue;

                    g2o::Sim3 Snw;

                    LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                    if(itn!=NonCorrectedSim3.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[pKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3* en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat Tiw = Converter::toCvSE3(eigR,eigt);

        pKFi->SetPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        int nIDr;
        if(pMP->mnCorrectedByKF==pCurKF->mnId)
        {
            nIDr = pMP->mnCorrectedReference;
        }
        else
        {
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mnId;
        }


        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        cv::Mat P3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMP->SetWorldPos(cvCorrectedP3Dw);

        pMP->UpdateNormalAndDepth();
    }
}

int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Calibration
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // Camera poses
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();

    // Set Sim3 vertex
    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();    
    vSim3->_fix_scale=bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0,2);
    vSim3->_principle_point1[1] = K1.at<float>(1,2);
    vSim3->_focal_length1[0] = K1.at<float>(0,0);
    vSim3->_focal_length1[1] = K1.at<float>(1,1);
    vSim3->_principle_point2[0] = K2.at<float>(0,2);
    vSim3->_principle_point2[1] = K2.at<float>(1,2);
    vSim3->_focal_length2[0] = K2.at<float>(0,0);
    vSim3->_focal_length2[1] = K2.at<float>(1,1);
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
    vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;

    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])
            continue;

        MapPoint* pMP1 = vpMapPoints1[i];
        MapPoint* pMP2 = vpMatches1[i];

        const int id1 = 2*i+1;
        const int id2 = 2*(i+1);

        const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
            {
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D1w = pMP1->GetWorldPos();
                cv::Mat P3D1c = R1w*P3D1w + t1w;
                vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D2w = pMP2->GetWorldPos();
                cv::Mat P3D2c = R2w*P3D2w + t2w;
                vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double,2,1> obs2;
        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    int nBad=0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges21[i]=static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
            nBad++;
        }
    }

    int nMoreIterations;
    if(nBad>0)
        nMoreIterations=10;
    else
        nMoreIterations=5;

    if(nCorrespondences-nBad<10)
        return 0;

    // Optimize again only with inliers

    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
        }
        else
            nIn++;
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();

    return nIn;
}


} //namespace ORB_SLAM
