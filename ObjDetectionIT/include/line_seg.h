#ifndef LINE_SEG_H
#define LINE_SEG_H

#include<Eigen/Dense>
#include<Eigen/StdVector>

#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<math.h>

namespace OBJECT
{

class Line_seg
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int id;
    // 左侧点
    Eigen::Vector2d left_point;
    // 右侧点
    Eigen::Vector2d right_point;
    // 长度
    double length;
    // 线段在图像平面坐标系下的参数
    Eigen::Vector3d parameter;
    // 线段在相机坐标系下的平面
    Eigen::Vector4d plane;
    // 线段在目标坐标系下的平面
    Eigen::Vector4d planeInCuboid;
    // 与u轴的夹角
    double angle;
    // 相机内参
    Eigen::Matrix3d K;
    // 目标框线对应的顶点
    int pointid;
    // 当前线段对应的距离图，在初始化目标之后应该删除距离图
    cv::Mat DistMap;


    // 构造函数
    Line_seg();
    Line_seg(cv::Vec4f _line, Eigen::Matrix3d _K);
    Line_seg(const Line_seg& _line_seg);
    // 析构函数
    ~Line_seg();

    // 判断线段是否在一个框内部
    bool ifInBB(double minx, double miny, double maxx, double maxy);
    // 计算线段在图像平面坐标系下的参数
    void getLineParameters();
    // 计算平面参数
    void getPlaneParameters();
    // 计算目标坐标系下的平面参数
    void getPlaneParametersInCuboid(Eigen::Matrix<double, 4, 3> Pst);
    void getPlaneParametersInCuboid1(Eigen::Matrix<double, 3, 3> Pst);
    // 计算目标框的每个线段对应的顶点
    void getPlaneObjPointID(Eigen::Matrix<double, 4, 3> Mst, int id);
    double getPLDist(Eigen::Vector2d point);
    void getDistMap(double minx, double miny, double maxx, double maxy, double im_width, double im_height);
};

}


#endif