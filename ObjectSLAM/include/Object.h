/**
 * The Function about Objects
 **/

#ifndef OBJECT_H
#define OBJECT_H

#include<opencv2/core/core.hpp>
#include<Eigen/Dense>
#include<math.h>
#include<Eigen/StdVector>

#include "Frame.h"
#include "Map.h"
#include "Converter.h"
// #include "Optimizer.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class Frame;
class Map;
class Obj3D;
// class cuboid;

class cuboidS
{
public:
	Eigen::Vector3d scale; // [length, width, height]  half! x,y,z方向上的尺寸
    Eigen::Matrix3d mRow;  // 目标坐标系的点向世界坐标系变换的旋转矩阵
    Eigen::Vector3d mtow;  // 目标坐标系的点向世界坐标系变换的平移矩阵
    Eigen::Matrix4d mTow;  // 目标坐标系的点向世界坐标系变换的变换矩阵

	cuboidS()
	{
		scale.setZero();
        // mRow.setIdentity();
        mtow.setZero();
        // mTow.setIdentity();
        mTow.block(0,3,3,1) = mtow;
	}

	inline void setTranslation(const Eigen::Vector3d &t_) { mtow = t_; mTow.block(0,3,3,1)=t_; }
	inline void setRotation(const Eigen::Matrix3d &R) { mRow = R; mTow.block(0,0,3,3)=R;}
	inline void setTransform(const Eigen::Matrix4d &T_) { mTow = T_; }
	inline void setScale(const Eigen::Vector3d &scale_) { scale = scale_; }

    // 返回八个顶点的世界坐标系坐标
    Eigen::Matrix<double, 8, 3> getPoints()
    {
        Eigen::Matrix<double, 8, 3> cubePoints;
        cubePoints <<   -scale(0), -scale(1),  scale(2),
                        -scale(0),  scale(1),  scale(2),
                         scale(0),  scale(1),  scale(2),
                         scale(0), -scale(1),  scale(2),
                        -scale(0), -scale(1), -scale(2),
                        -scale(0),  scale(1), -scale(2),
                         scale(0),  scale(1), -scale(2),
                         scale(0), -scale(1), -scale(2);

        // 旋转八个点
        Eigen::Matrix<double, 3, 8> cubePoints1 = mRow*cubePoints.transpose();

        for(int i=0; i<8; i++)
        {
            cubePoints1(0,i) = cubePoints1(0,i)+mtow(0);
            cubePoints1(1,i) = cubePoints1(1,i)+mtow(1);
            cubePoints1(2,i) = cubePoints1(2,i)+mtow(2);
        }
        
        // 相机坐标系下的点
        cubePoints = cubePoints1.transpose();
        return cubePoints;
    }

    Eigen::Matrix<double, 8, 3> getPoints1()
    {
        Eigen::Matrix<double, 8, 3> cubePoints;
        
        cubePoints <<    scale(0),  scale(1),  scale(2),
                        -scale(0),  scale(1),  scale(2),
                        -scale(0),  scale(1), -scale(2),
                         scale(0),  scale(1), -scale(2),
                        -scale(0), -scale(1),  scale(2),
                        -scale(0), -scale(1), -scale(2),
                         scale(0), -scale(1), -scale(2),
                         scale(0), -scale(1),  scale(2);

        // 旋转八个点
        Eigen::Matrix<double, 3, 8> cubePoints1 = mRow*cubePoints.transpose();

        for(int i=0; i<8; i++)
        {
            cubePoints1(0,i) = cubePoints1(0,i)+mtow(0);
            cubePoints1(1,i) = cubePoints1(1,i)+mtow(1);
            cubePoints1(2,i) = cubePoints1(2,i)+mtow(2);
        }
        
        // 相机坐标系下的点
        cubePoints = cubePoints1.transpose();
        return cubePoints;
    }

};

class cuboid
{
public:
	g2o::SE3Quat pose;   // 6 dof for object, object to world by default
	Eigen::Vector3d scale; // [length, width, height]  half! x,y,z方向上的尺寸

	cuboid()
	{
		pose = g2o::SE3Quat();
		scale.setZero();
	}

    inline const Eigen::Vector3d &translation() const { return pose.translation(); }
	inline void setTranslation(const Eigen::Vector3d &t_) { pose.setTranslation(t_); }
	inline void setRotation(const Eigen::Quaterniond &r_) { pose.setRotation(r_); }
	inline void setRotation(const Eigen::Matrix3d &R) { pose.setRotation(Eigen::Quaterniond(R)); }
	inline void setScale(const Eigen::Vector3d &scale_) { scale = scale_; }

    // transform a local cuboid to global cuboid  Twc is camera pose. from camera to world
    cuboid transform_from(const g2o::SE3Quat &Twc) const
    {
        cuboid res;
        res.pose = Twc * this->pose;
        res.scale = this->scale;
        return res;
    }

    // actual error between two cuboids.
    Eigen::Matrix<double, 9, 1> cube_log_error(const cuboid &newone) const
    {
        cout<<"this->pose: " << endl;
        cout << this->pose << endl;
        Eigen::Matrix<double, 9, 1> res;
        res.setZero();
        g2o::SE3Quat pose_diff = newone.pose.inverse() * this->pose;
        res.head<6>() = pose_diff.log(); //treat as se3 log error. could also just use yaw error
        // res.tail<3>() = this->scale - newone.scale;
        cout<<"res: "<< res << endl;
        return res;
    }

    void CuboidNormalize()
    {
        Eigen::Matrix4d res = pose.to_homogeneous_matrix();
        Eigen::Vector3d _t = res.block(0,3,3,1);
        double _s = _t(2);
        _t = _t/_s;
        pose.setTranslation(_t);
        scale = scale/_s;
    }
};


// 2D目标类
class Obj2D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /* data */
    Obj3D* pObj3D;
    // 2D目标框所在的Frame
    Frame* pFrame;
    // 上一帧3D目标旋转
    Obj3D* pLastObj3D;
    bool hasLastObj;
    // Eigen::Matrix3d R_Last3D;
    // 上上帧3D目标
    // Obj3D* pLastLastObj3D;
    // Eigen::Matrix3d R_LastLast3D;
    // 中心点坐标
    Eigen::Vector2d evecCenter;
    // 左上角坐标
    Eigen::Vector2d evecLeftUp;
    // 右下角坐标
    Eigen::Vector2d evecRightDown;
    double width, height;
    // cv Rect对象
    cv::Rect objRect;
    // 类别标签
    int iLabel;
    // confident
    double dConf;
    // 匹配的3D目标的ID
    int iObj3DMatchedID;
    // 判断是否保留线段的阈值
    double lineWidTh;
    // 包含的线段，左侧点坐标，右侧点坐标，长度
    vector<Eigen::Matrix<double, 1, 5>, Eigen::aligned_allocator<Eigen::Matrix<double, 1, 5>>> emvLines;
    // 包含的平行线段，paraline1-line1，paraline1-line2；paraline2-line1，paraline2-line2
    vector<Eigen::Matrix<double, 2, 5>, Eigen::aligned_allocator<Eigen::Matrix<double, 2, 5>>> emvParaLines;

    // 构造函数
    Obj2D(Frame* pFrame, double x1, double y1, double x2, double y2, int label, double conf, const vector<vector<double>> &vLines);
    ~Obj2D();
    // 获取二维线段的参数
    void GetLineParameters(const Eigen::Matrix<double,1,5> &inLine, Eigen::Vector3d &LineParameter);
    // 寻找在边界框内的线段
    void GetLineInBBox(const vector<vector<double>> &vLines);
    // 合并线段
    void MergeLine();
    // 寻找平行线
    void GetParaLines();
    // 获取Rect对象
    cv::Rect GetRect();
    // 输出Line
    vector<vector<cv::Point2f>> GetLines();
    // 输出paraLine
    vector<vector<cv::Point2f>> PopParaLines();
};

// 3D目标类
class Obj3D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 相机内参
    Eigen::Matrix3d K;
    // 是否生成了3D目标
    bool has3DObj;
    // 3D目标的中心点坐标
    Eigen::Vector3d Center;
    Eigen::Vector3d Axes;
    Eigen::Matrix3d R_3D;
    // 八个顶点在相机坐标系下的坐标
    //      0————————1
    //     /|       /|
    //    / |      / |
    //   3————————2  |
    //   |  4—— ——|——5
    //   | /      | /
    //   7————————6
    Eigen::Matrix<double, 8, 3> cubePoints;
    Eigen::Matrix<double, 8, 2> framePointsx;
    Eigen::Matrix<double, 4, 3> direPoints;
    // 椭球结果
    Eigen::Matrix4d Quadric;
    // 用于计算检测线与目标和相机位姿之间的误差
    // 数据格式：4x6
    // a0, a1, ..., a5,   ----图像平面的检测线的参数方程
    // b0, b1, ..., b5,   ----图像平面的检测线的参数方程
    // c0, c1, ..., c5,   ----图像平面的检测线的参数方程
    // i0, i1, ..., i5,   ----与该线段相交的点的id
    Eigen::Matrix<double, 4, 6> LineConstraint;
    // 标签
    int label;
    // 置信度
    double conf;
    Obj2D* pObj2D;
    double directSim;
    double sizeSim;
    int mnId;
    g2o::SE3Quat pose;
	Eigen::Vector3d scale;


    // 构造函数
    Obj3D(Eigen::Matrix<double, 8, 3> &cubePoints);
    Obj3D(Obj2D* pObj2D, cv::Mat mK);
    ~Obj3D();
    // 生成3D目标
    void Get3DObj();
    // 从3D模型中得到目标的信息
    void Get3DObjInfo(Eigen::Matrix3d &R);
    void Get3DObjInfoTest(Eigen::Matrix3d &R);

    // 将cubePoints投影到frame中
    void ProjCube2Frame();
    void GetDirection(Eigen::Matrix3d &R);
    void GetDirection(Eigen::Matrix3d &R, Eigen::Matrix3d &R_Last);
    void GetEllipsoid(Eigen::Matrix3d &R, Eigen::Matrix4d &estQ);
    void GetEllipsoidFromLast(Eigen::Matrix3d &R, Eigen::Matrix4d &estQ);
    void GetCubePoints(Eigen::Matrix<double, 12, 3> &_cubePoints);
    void SetWorldPos(const cuboid &Pos);
    void Get3DObjInfo();
    cuboid getCuboid();
};


class DetectObj3D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static void getDirectConstraint(Eigen::Matrix<double, 2, 5> &paraLine, Eigen::Matrix<double, 2, 10> &DirectionCons, Eigen::Vector3d &direction, Eigen::Matrix<double, 4,3> &Pst);
    static void get_line_parameters(Eigen::Matrix<double, 2, 3> &line_para, Eigen::Matrix<double, 2, 5> &paraLine);
    static void dual_quadric_to_ellipsoid_parameters(Eigen::Vector3d &center, Eigen::Vector3d &axes, Eigen::Matrix3d &R, Eigen::Matrix4d &Ellipsoid);
    static void sort_vec(const Eigen::Vector3d &vec, Eigen::Vector3d &sorted_vec, Eigen::Vector3i &ind);
    static void get_plane_constraint_box(const Eigen::Matrix<double, 4, 1> &objBBS, const Eigen::Matrix<double, 4, 3> &Pst, const Eigen::Matrix<double, 4, 3> &Mst, Eigen::Matrix<double, 4, 10> &plane_cons, Eigen::Matrix<double, 4, 4> &plane_dire);
    static void get_bbline_parameters(Eigen::Matrix4d &bb_lines, Eigen::Matrix<double, 4, 3> &line_para);
    static void get_plane_constraint_detect(vector<Eigen::Matrix<double, 1, 5>, Eigen::aligned_allocator<Eigen::Matrix<double, 1, 5>>> &objLines, const Eigen::Matrix<double, 4, 3> &Pst, Eigen::MatrixXd &plane_cons_det);
    static void get_bbline_parameters1(Eigen::Matrix<double, 1, 5> &bb_lines, Eigen::Matrix<double, 1, 3> &line_para);
};





}


#endif