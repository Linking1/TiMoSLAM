#ifndef OBJ3D_H
#define OBJ3D_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<Eigen/Dense>
#include<Eigen/StdVector>

#include<math.h>
#include<time.h>

#include "obj2d.h"
#include "line_seg.h"
// #include "Hungarian.h"
#include "ITNode.h"
#include "SRG.h"

using namespace std;

namespace OBJECT
{

class Obj2D;
class Line_seg;
class Direction;
class IT;
class SRG;

// 两个作用
// 1.表示单帧中的3D目标观测
// 2.表示地图中的3D目标
class Obj3D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 观测目标属性
    // 3D目标中心点坐标
    // Cuboid Model
    Eigen::Vector3d Center;
    Eigen::Vector3d Axes;
    Eigen::Matrix3d R;
    // 标签
    int label;
    // 地图目标id
    int id;
    // 置信度
    double conf;
    double IOU;
    // 所属的2D目标
    // Obj2D* pObj2D;
    // have rotation matrix
    bool hasRot;
    // have translation information
    bool hasTrans;
    // the eight point in the cuboid
    Eigen::Matrix<double, 4, 8> eightPointsInCuboid;
    Eigen::Matrix<double, 4, 8> eightPointsInCamera;
    // line features with points
    Eigen::Matrix<int, 2, 12> linePointsID;
    // plane features with points
    Eigen::Matrix<int, 4, 6> planePointsID;
    // direction with line ID
    Eigen::Matrix<int, 6, 3> directionFeaID;

    double maxScale;
    double tranConf;
    double rotConf;
    
    // 八个顶点在相机坐标系下的坐标
    //      0————————1      ^  Vz
    //     /|       /|      |
    //    / |      / |      |
    //   3————————2  |      O-----> Vy
    //   |  4—— ——|——5     /
    //   | /      | /     v  Vx
    //   7————————6 
    // Vx: l12, l03, l56, l47, p0154, p3267
    // Vy: l01, l23, l45, l67, p0374, p1265
    // Vz: l04, l15, l26, l37, p0123, p4567
    // 不再存储顶点坐标，每次调用函数计算
    void getEightPointsInCuboid();
    void getEightPointsInCamera();

    // 函数
    // 构造函数
    Obj3D();
    Obj3D(Obj3D* _pObj3D);
    Obj3D(Eigen::Matrix<double, 8, 3> &cubePoints);
    Obj3D(Obj2D* pObj2D, Eigen::Matrix3d K);
    ~Obj3D();
    // generate rotation matrix
    void getRotationMat();
    // estimate the translation and scale of the cuboid
    void getTrans();
    // test the matching consistency
    bool testMatchingCon();
    // 在已有先验目标的情况下获得3D目标
    // void Get3DObj(Eigen::Matrix3d R, Eigen::Vector3d Center, Eigen::Vector3d Axes, const cv::Mat &imGray);
    // 根据两个线段得到对应的方向和约束
    // void GetDirectConstraint(Line_seg* line1, Line_seg* line2, Eigen::Matrix<double, 2, 10> &DirectionCons, Eigen::Vector3d &direction);
    // 得到每一对平行线的方向和约束
    // void GetAllDirectConstraint(Eigen::MatrixXd &directions, vector<Direction*> &DirectionObjs);
    // 得到每一对平行线的置信度，即有效线段的数量
    // void GetParaConf();
    // 得到可能的采样
    // void GetPossiblePair(vector<Eigen::Vector4i> &PossiblePair);
};





}


#endif