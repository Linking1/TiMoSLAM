#include "line_seg.h"

namespace OBJECT
{

Line_seg::Line_seg()
{
    left_point.setZero();
    right_point.setZero();
    length = 0;
}

Line_seg::Line_seg(cv::Vec4f _line, Eigen::Matrix3d _K)
{
    if(_line[0]>_line[2])
    {
        left_point(0) = _line[2];
        left_point(1) = _line[3];
        right_point(0) = _line[0];
        right_point(1) = _line[1];
    }
    else
    {
        left_point(0) = _line[0];
        left_point(1) = _line[1];
        right_point(0) = _line[2];
        right_point(1) = _line[3];
    }
    length = pow(pow(_line(0)-_line(2),2.0)+pow(_line(1)-_line(3),2.0),0.5);
    getLineParameters();
    angle = atan2(right_point(1)-left_point(1), abs(right_point(0)-left_point(0)));
    K = _K;
    getPlaneParameters();
}

Line_seg::Line_seg(const Line_seg& _line_seg)
{
    left_point(0) = _line_seg.left_point(0);
    left_point(1) = _line_seg.left_point(1);
    right_point(0) = _line_seg.right_point(0);
    right_point(0) = _line_seg.right_point(0);
    length = _line_seg.length;
    parameter(0) = _line_seg.parameter(0);
    parameter(1) = _line_seg.parameter(1);
    parameter(2) = _line_seg.parameter(2);
    angle = _line_seg.angle;
}

Line_seg::~Line_seg()
{
}

bool Line_seg::ifInBB(double minx, double miny, double maxx, double maxy)
{
    if(left_point(0)>minx && right_point(0)<maxx && left_point(1)>miny && left_point(1)<maxy && right_point(1)>miny && right_point(1)<maxy)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Line_seg::getLineParameters()
{
    parameter(0) = left_point(1) - right_point(1);
    parameter(1) = right_point(0) - left_point(0);
    parameter(2) = -left_point(1)*parameter(1) - left_point(0)*parameter(0);
    double _mo = pow(pow(parameter(0), 2.0)+pow(parameter(1), 2.0), 0.5);
    parameter(0) = parameter(0)/_mo;
    parameter(1) = parameter(1)/_mo;
    parameter(2) = parameter(2)/_mo;
}

void Line_seg::getPlaneParameters()
{
    Eigen::Matrix4d ObjMst;
    ObjMst.setIdentity();
    Eigen::Matrix<double, 3, 4> Pst1 = K*ObjMst.block(0,0,3,4);
    Eigen::Matrix<double, 4, 3> Pst = Pst1.transpose();
    plane = Pst*parameter;
    plane(3) = 0;
    plane.normalize();
}

void Line_seg::getPlaneParametersInCuboid(Eigen::Matrix<double, 4, 3> Pst)
{
    planeInCuboid = Pst * parameter;
}

void Line_seg::getPlaneParametersInCuboid1(Eigen::Matrix<double, 3, 3> Pst)
{
    planeInCuboid.block(0,0,3,1) = Pst * plane.block(0,0,3,1);
    planeInCuboid(3) = 0;
}

void Line_seg::getPlaneObjPointID(Eigen::Matrix<double, 4, 3> Mst, int id)
{
    Eigen::Matrix<double, 4, 3> Pst = Mst*K.transpose();
    Eigen::Vector4d plane_dire;

    for(int i=0; i<4; i++)
    {
        plane_dire(i) = planeInCuboid(i);
    }
    
    // 获取平面在相机坐标系下的近似方向向量
    Eigen::Matrix4d plane_dire_appro;// = Eigen::Matrix4d::Zero();
    //                 上， 左， 下， 右
    plane_dire_appro << 0,  1,  0, -1,
                        1,  0, -1,  0,
                        0,  0,  0,  0,
                        0,  0,  0,  0;
    Eigen::Matrix<double, 4, 4> Mst1;
    Mst1.setIdentity();
    Mst1.block(0,0,4,3) = Mst;
    Eigen::Vector4d plane_dire_obj_appro = Mst1 * plane_dire_appro.block(0,id,4,1);
    // 将plane_dire_obj_appro变换为单位方向向量
    // 将plane_dire变换为单位方向向量
    plane_dire_obj_appro = plane_dire_obj_appro.normalized();
    plane_dire(3) = 0;
    plane_dire = plane_dire.normalized();

    // 判断plane_dire是否为内侧向量，不是的话反向
    Eigen::Matrix<double, 1, 1> _dot_res = plane_dire_obj_appro.transpose() * plane_dire;
    if(_dot_res(0,0) < 0)
    {
        plane_dire(0) = -plane_dire(0);
        plane_dire(1) = -plane_dire(1);
        plane_dire(2) = -plane_dire(2);
    }
    Eigen::Matrix<double, 8, 9> pointsInfo;
    pointsInfo <<   1,0,0, 0, 1,0, 0,0,-1,
                    1,0,0, 0,-1,0, 0,0,-1,
                   -1,0,0, 0,-1,0, 0,0,-1,
                   -1,0,0, 0, 1,0, 0,0,-1,
                    1,0,0, 0, 1,0, 0,0, 1,
                    1,0,0, 0,-1,0, 0,0, 1,
                   -1,0,0, 0,-1,0, 0,0, 1,
                   -1,0,0, 0, 1,0, 0,0, 1;

    // 然后计算与哪个顶点关联
    for(int i=0; i<8; i++)
    {
        // 判断上平面与当前点
        Eigen::Matrix<double, 1, 1> _up_dot_0= pointsInfo.block(i,0,1,3)*plane_dire;
        Eigen::Matrix<double, 1, 1> _up_dot_1= pointsInfo.block(i,3,1,3)*plane_dire;
        Eigen::Matrix<double, 1, 1> _up_dot_2= pointsInfo.block(i,6,1,3)*plane_dire;

        if((_up_dot_0(0,0)>=0 || abs(_up_dot_0(0,0))<0.001) && (_up_dot_1(0,0)>=0 || abs(_up_dot_1(0,0))<0.001) && (_up_dot_2(0,0)>=0||abs(_up_dot_2(0,0))<0.001))
        {
            pointid = i;
        }
    }
}

double Line_seg::getPLDist(Eigen::Vector2d point)
{
    double dist = parameter(0)*point(0)+parameter(1)*point(1)+parameter(2);
    dist = dist/pow(pow(parameter(0),2.0)+pow(parameter(1),2.0),0.5);
    std::cout << "-----------------------" << std::endl;
    std::cout << "parameter: \n" << parameter << std::endl;
    std::cout << "point: \n" << point << std::endl;
    std::cout << "dist: " << dist << std::endl;
    return abs(dist);
}


void Line_seg::getDistMap(double minx, double miny, double maxx, double maxy, double im_width, double im_height)
{
    cv::Mat Line_im = cv::Mat::zeros(cv::Size(im_width, im_height),CV_8UC1);
    // 将当前线段写入图像
    for(int i=int(left_point(0)); i<int(right_point(0)); i++)
    {
        int y = (-parameter(2)-parameter(0)*i)/parameter(1);
        if(y>miny&&y<maxy)
        {
            Line_im.at<uchar>(y,i) = 255;
        }
    }
    cv::distanceTransform(255 - Line_im, DistMap, CV_DIST_L2, 3);
}

}