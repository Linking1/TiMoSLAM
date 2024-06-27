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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "Object.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

namespace ORB_SLAM2
{

class LoopClosing;
class Obj3D;
class cuboid;
class cuboidS;

class Optimizer
{
public:
	void static BundleAdjustment(const std::vector<KeyFrame *> &vpKF, const std::vector<MapPoint *> &vpMP, const vector<Obj3D *> &vpObj,
									int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
									const bool bRobust = true);
	void static BundleAdjustmentNoCube(const std::vector<KeyFrame *> &vpKF, const std::vector<MapPoint *> &vpMP, const vector<Obj3D *> &vpObj,
									int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
									const bool bRobust = true);
									
	void static GlobalBundleAdjustemnt(Map *pMap, int nIterations = 5, bool *pbStopFlag = NULL,
										const unsigned long nLoopKF = 0, const bool bRobust = true);
	bool static CurFrameGBundleAdjustemnt(Map *pMap, Frame &CurFrame, int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0, const bool bRobust = true);
	bool static CurFrameBundleAdjustment(const std::vector<KeyFrame *> &vpKF, const std::vector<MapPoint *> &vpMP, const vector<Obj3D *> &vpObj, Frame &CurFrame,
											int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
											const bool bRobust = true);
	void static LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap);
	int static PoseOptimization(Frame *pFrame);

	bool static LSDTransGBundleAdjustemnt(Map *pMap, Frame &CurFrame, bool tosave, int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0, const bool bRobust = true);
	bool static LSDTransBundleAdjustment(const std::vector<KeyFrame *> &vpKF, const std::vector<MapPoint *> &vpMP, const vector<Obj3D *> &vpObj, Frame &CurFrame, bool tosave,
											int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
											const bool bRobust = true);

	bool static SE3TransGBundleAdjustemnt1(Map *pMap, Frame &CurFrame, bool tosave, int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0, const bool bRobust = true);
	bool static SE3TransBundleAdjustment(const std::vector<KeyFrame *> &vpKF, const std::vector<MapPoint *> &vpMP, const vector<Obj3D *> &vpObj, Frame &CurFrame, bool tosave,
											int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
											const bool bRobust = true);

	// if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
	void static OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
										const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
										const LoopClosing::KeyFrameAndPose &CorrectedSim3,
										const map<KeyFrame *, set<KeyFrame *>> &LoopConnections,
										const bool &bFixScale);

	// if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
	static int OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches1,
							g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
};

// 相机顶点
class VertexCam : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	VertexCam() : BaseVertex(){
		Rcw.setZero();
		tcw.setZero();
		LineConstraint.setZero();
	}

	// 重置
	virtual void setToOriginImpl() override
	{
		_estimate << 0, 0, 0;
	}

	// 更新
	virtual void oplusImpl(const double *update) override
	{
		_estimate += Eigen::Vector3d(update);
	}

	// 存盘和读盘：留空
	virtual bool read(istream &in) {}
	virtual bool write(ostream &out) const {}

public:
	// 相机旋转矩阵
	Eigen::Matrix3d Rcw;
	// 相机平移矩阵
	Eigen::Matrix4d tcw;
	Eigen::Matrix<double, 4, 6> LineConstraint;
};

// 目标顶点，优化变量为平移和尺寸， scale和mtow
class VertexCuboid : public g2o::BaseVertex<6, cuboidS>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	// 构造函数, 需要：
	// 1. 将顶点从目标坐标系变换到世界坐标系的旋转矩阵
	// 2. 将顶点从目标坐标系变换到世界坐标系的平移向量
	VertexCuboid() : BaseVertex()
	{
	};

	// 重置，怎么设置初值？——设置初值采用setEstimate
	virtual void setToOriginImpl()
	{
		_estimate = cuboidS();
	};

	// 更新
	virtual void oplusImpl(const double *update_)
	{
		Eigen::Map<const Vector6d> update(update_);

		cuboidS newcube;
		newcube.setTranslation(_estimate.mtow+update.head(3));
		newcube.scale = _estimate.scale + update.tail<3>();
		newcube.mRow = _estimate.mRow;
		newcube.mTow = _estimate.mTow;

		setEstimate(newcube);
	}

	// 存盘和读盘：留空
	virtual bool read(std::istream &is) { return true; };
	virtual bool write(std::ostream &os) const { return os.good(); };

};

//                                    测量值维度，       测量值类型，连接的顶点1，   连接的顶点2
class EdgeLSDCuboidProj : public g2o::BaseBinaryEdge<6, Vector6d, VertexCuboid, VertexCam>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	// 构造函数
	EdgeLSDCuboidProj(){
		cubeCoeff.setZero();
	};

	// 1. 将相应的点按照6x1的顺序投影到图像平面上，并根据线段方程计算距离，存储下来
	// 2. 初始化时挑选出相应的点
	void getCubePoints(Eigen::Matrix<double, 4, 6> LineConstraint)
	{
		Eigen::Matrix<double, 8, 3> coeffMatrix;


		coeffMatrix <<   1,  1,  1,
                        -1,  1,  1,
                        -1,  1, -1,
                         1,  1, -1,
                        -1, -1,  1,
                        -1, -1, -1,
                         1, -1, -1,
                         1, -1,  1;

        // 根据LineConstraint的编号选择适合的系数
		for(int i=0; i<6; i++)
		{
			int coeffId = LineConstraint(3,i);
			cubeCoeff(0, i) = coeffMatrix(coeffId, 0);
			cubeCoeff(1, i) = coeffMatrix(coeffId, 1);
			cubeCoeff(2, i) = coeffMatrix(coeffId, 2);
		}
	}

	virtual bool read(std::istream &is) { return true; };
	virtual bool write(std::ostream &os) const { return os.good(); };

	virtual void computeError() override
	{
		// 获取两个节点
		const VertexCuboid *vCuboid = static_cast<const VertexCuboid *> (_vertices[0]);
		const VertexCam *vCam = static_cast<const VertexCam *> (_vertices[1]);

		// 计算误差
		// 先根据尺寸获取目标坐标系下的六个顶点
		cuboidS cubeTemp = vCuboid->estimate();
		Eigen::Matrix<double, 3, 6> cubePointsObj;
		cubePointsObj.setZero();
		
		for(int i=0; i<6; i++)
		{
			cubePointsObj(0,i) = cubeCoeff(0,i)*cubeTemp.scale(0);
			cubePointsObj(1,i) = cubeCoeff(1,i)*cubeTemp.scale(1);
			cubePointsObj(2,i) = cubeCoeff(2,i)*cubeTemp.scale(2);
		}
		// 将顶点投影到世界坐标系
		// 旋转
		Eigen::Matrix<double, 3, 6> cubePointsWorld = cubeTemp.mRow*cubePointsObj;
		// 平移
		for(int i=0; i<6; i++)
		{
			cubePointsWorld(0,i) += cubeTemp.mtow(0);
			cubePointsWorld(1,i) += cubeTemp.mtow(1);
			cubePointsWorld(2,i) += cubeTemp.mtow(2);
		}
		// 将顶点投影到当前相机坐标系
		// 相机的平移估计
		Eigen::Vector3d CamTranslate = vCam->estimate();

		Eigen::Matrix<double, 3, 6> cubePointsCam = vCam->Rcw*cubePointsWorld;
		for(int i=0; i<6; i++)
		{
			cubePointsCam(0, i) += CamTranslate(0);
			cubePointsCam(1, i) += CamTranslate(1);
			cubePointsCam(2, i) += CamTranslate(2);
		}
		// 将顶点投影到图像归一化平面上
		Eigen::Matrix<double, 3, 6> cubePointsImg = Kalib*cubePointsCam;

		// 获得像素坐标
		for(int i=0; i<6; i++)
		{
			cubePointsImg(0, i) /= cubePointsCam(2, i);
			cubePointsImg(1, i) /= cubePointsCam(2, i);
			cubePointsImg(2, i) /= cubePointsCam(2, i);
		}

		// 放到LineConstriant中计算误差
		Eigen::Matrix<double, 4, 6> LineConstraint = vCam->LineConstraint;
		for(int i=0; i<6; i++)
		{
			double LineConstraintMo = pow(pow(LineConstraint(0,i),2)+pow(LineConstraint(1,i),2), 0.5);
			for(int j=0; j<3; j++)
			{
				LineConstraint(j,i) /= LineConstraintMo;
			}
		}
		Eigen::Matrix<double, 6, 3> cubePointsImg1 = cubePointsImg.transpose();
		Vector6d _error1;
		_error1.setZero();

		for(int i=0; i<6; i++)
		{
			double LineConstraintMo = pow(pow(LineConstraint(0,i),2)+pow(LineConstraint(1,i),2), 0.5);
			_error1(i) = (cubePointsImg1(i,0)*LineConstraint(0,i)+cubePointsImg1(i,1)*LineConstraint(1,i)+cubePointsImg1(i,2)*LineConstraint(2,i))/LineConstraintMo;
		}
		_error = _measurement - _error1;

	}

	double computeError1()
	{
		// 获取两个节点
		const VertexCuboid *vCuboid = static_cast<const VertexCuboid *> (_vertices[0]);
		const VertexCam *vCam = static_cast<const VertexCam *> (_vertices[1]);

		// 计算误差
		// 先根据尺寸获取目标坐标系下的六个顶点
		cuboidS cubeTemp = vCuboid->estimate();
		Eigen::Matrix<double, 3, 6> cubePointsObj;
		cubePointsObj.setZero();

		
		for(int i=0; i<6; i++)
		{
			cubePointsObj(0,i) = cubeCoeff(0,i)*cubeTemp.scale(0);
			cubePointsObj(1,i) = cubeCoeff(1,i)*cubeTemp.scale(1);
			cubePointsObj(2,i) = cubeCoeff(2,i)*cubeTemp.scale(2);
		}
		// 将顶点投影到世界坐标系
		// 旋转
		Eigen::Matrix<double, 3, 6> cubePointsWorld = cubeTemp.mRow*cubePointsObj;
		// 平移
		for(int i=0; i<6; i++)
		{
			cubePointsWorld(0,i) += cubeTemp.mtow(0);
			cubePointsWorld(1,i) += cubeTemp.mtow(1);
			cubePointsWorld(2,i) += cubeTemp.mtow(2);
		}
		// 将顶点投影到当前相机坐标系
		// 相机的平移估计
		Eigen::Vector3d CamTranslate = vCam->estimate();
		
		Eigen::Matrix<double, 3, 6> cubePointsCam = vCam->Rcw*cubePointsWorld;
		for(int i=0; i<6; i++)
		{
			cubePointsCam(0, i) += CamTranslate(0);
			cubePointsCam(1, i) += CamTranslate(1);
			cubePointsCam(2, i) += CamTranslate(2);
		}

		cout << "cubePointsCam in error: " << cubePointsCam << endl;

		// 将顶点投影到图像归一化平面上
		Eigen::Matrix<double, 3, 6> cubePointsImg = Kalib*cubePointsCam;

		// 获得像素坐标
		for(int i=0; i<6; i++)
		{
			cubePointsImg(0, i) /= cubePointsCam(2, i);
			cubePointsImg(1, i) /= cubePointsCam(2, i);
			cubePointsImg(2, i) /= cubePointsCam(2, i);
		}

		// 放到LineConstriant中计算误差
		Eigen::Matrix<double, 4, 6> LineConstraint = vCam->LineConstraint;
		for(int i=0; i<6; i++)
		{
			double LineConstraintMo = pow(pow(LineConstraint(0,i),2)+pow(LineConstraint(1,i),2), 0.5);
			for(int j=0; j<3; j++)
			{
				LineConstraint(j,i) /= LineConstraintMo;
			}
		}
		Eigen::Matrix<double, 6, 3> cubePointsImg1 = cubePointsImg.transpose();
		Vector6d _error1;
		_error1.setZero();

		for(int i=0; i<6; i++)
		{
			double LineConstraintMo = pow(pow(LineConstraint(0,i),2)+pow(LineConstraint(1,i),2), 0.5);
			_error1(i) = (cubePointsImg1(i,0)*LineConstraint(0,i)+cubePointsImg1(i,1)*LineConstraint(1,i)+cubePointsImg1(i,2)*LineConstraint(2,i))/LineConstraintMo;
		}
		_error = _measurement - _error1;

		double Error = 0.0;
		for(int i=0; i<6; i++)
		{
			Error += abs(_error(i));
		}
		return Error;
	}

	Vector6d get_error()
	{
		return _error;
	}

	Eigen::Vector3d get_scale()
	{
		const VertexCuboid *vCuboid = static_cast<const VertexCuboid *> (_vertices[0]);

		// 计算误差
		// 先根据尺寸获取目标坐标系下的六个顶点
		cuboidS cubeTemp = vCuboid->estimate();	
		return cubeTemp.scale;
	}


public:
	Eigen::Matrix3d Kalib;
	// 挑选后的目标顶点坐标
	Eigen::Matrix<double, 3, 6> cubeCoeff;
};

/**
* \brief 6D edge between two Vertex6
*/
class EdgeSE3Expmap1 : public g2o::BaseBinaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeSE3Expmap1(){};
  virtual bool read(std::istream& is) { return true; };
  virtual bool write(std::ostream& os) const { return os.good(); };
  virtual void computeError() override
  {
    const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    const g2o::VertexSE3Expmap* v2 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);

    g2o::SE3Quat C(_measurement);
    g2o::SE3Quat error_= C*v1->estimate()*v2->estimate().inverse();  // from sim3 orbslam   vertex 保存的是 world to cam. measurent 是 from 1 to 2
//     SE3Quat error_=C*v2->estimate().inverse()*v1->estimate();     // similar as LSD    
    _error = error_.log();
  }
  
  g2o::Vector6d showError()
  {
      const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
      const g2o::VertexSE3Expmap* v2 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
      
      g2o::SE3Quat C(_measurement);
      g2o::SE3Quat error_= C*v1->estimate()*v2->estimate().inverse();  // from sim3 orbslam
//       SE3Quat error_=C*v2->estimate().inverse()*v1->estimate();     // similar as LSD
      
      std::cout<<"error_ "<<error_.toVector().transpose()<<std::endl;
      
      return error_.log();
  }

  double get_error_norm()
  {
    const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    const g2o::VertexSE3Expmap* v2 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);

    g2o::SE3Quat C(_measurement);
    g2o::SE3Quat error_= C*v1->estimate()*v2->estimate().inverse(); // from sim3 orbslam
//     SE3Quat error_=C*v2->estimate().inverse()*v1->estimate(); // similar as LSD
    return error_.log().norm();
  }
  
  virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& , g2o::OptimizableGraph::Vertex* ) { return 1.;}
  virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* /*to*/)
  {
    g2o::VertexSE3Expmap* v1 = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
    g2o::VertexSE3Expmap* v2 = static_cast<g2o::VertexSE3Expmap*>(_vertices[1]);
    if (from.count(v1) > 0)
	v2->setEstimate(measurement()*v1->estimate());
// 	v2->setEstimate(v1->estimate()*measurement());
    else
	v1->setEstimate(measurement().inverse()*v2->estimate());
// 	v1->setEstimate(v2->estimate()*measurement().inverse());
  }
  void setMeasurement(const g2o::SE3Quat& m){
    _measurement = m;
  }
//   void linearizeOplus();   // compute exact jacobian  LSD and raw g2o has this.
};

} // namespace ORB_SLAM2

#endif // OPTIMIZER_H
