#ifndef OBJ2D_H
#define OBJ2D_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<Eigen/Dense>
#include<Eigen/StdVector>

#include<math.h>

#include "obj3d.h"
#include "line_seg.h"
#include "ITNode.h"
#include "SRG.h"

using namespace std;

namespace OBJECT
{

class Obj3D;
class Line_seg;
class IT;
class SRG;

// 用于表示图像平面上检测到的2D目标，即3D空间目标的观测
class Obj2D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 当前观测在当前帧中的id
    int id;
    // 检测框信息, x1, y1, x2, y2
    Eigen::Vector4d BoundingBox;
    // 中心点坐标
    Eigen::Vector2d Center;
    // 扩展后的坐标信息
    double expand_x1, expand_y1, expand_x2, expand_y2;
    double width_expand_scale, height_expand_scale;
    // 宽和高
    double width, height;
    // 2D框的标签
    int label;
    // 得分
    double conf;
    double NodeRotTh, NodeTranTh;
    // 包含的线段，左侧点坐标，右侧点坐标，长度
    vector<Line_seg*> lines;
    vector<Line_seg*> BBlines;
    vector<Line_seg*> BBExpandLines;
    vector<int> selectedLineID;
    // 3D目标对象
    Obj3D* pObj3D;
    // 是否生成了3D目标
    bool has3D;
    Eigen::Matrix3d K;
    int im_width;
    int im_height;
    // the compound iterpretation tree
    IT* pIT;
    // semantic relation graph
    SRG* pSRG;
    // line number
    int lineNum;
    // Frame Rotation
    Eigen::Matrix3d FrameRot;

    // 构造函数
    Obj2D(int id, double x1, double y1, double x2, double y2, int label, double conf, std::vector<cv::Vec4f>& lines, const cv::Mat &imGray, Eigen::Matrix3d _K);
    // 析构函数
    ~Obj2D();
    // 寻找在边界框内部的线段
    void selectLinesInBB(std::vector<cv::Vec4f>& lines);
    // 寻找平行线
    void selectParaLines();
    // 得到两条线段之间的夹角
    // 生成3D目标
    void getObj3D(Eigen::Matrix3d K);
    void getObj3D1(Eigen::Matrix3d K);
    
    void DrawObj(cv::Mat &im);
    void DrawLines(cv::Mat &im);
};

}
#endif