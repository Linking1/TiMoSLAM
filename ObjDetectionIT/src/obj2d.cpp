#include "obj2d.h"

namespace OBJECT
{

// 2D model initialization
Obj2D::Obj2D(int _id, double x1, double y1, double x2, double y2, int _label, double _conf, std::vector<cv::Vec4f>& _lines, const cv::Mat &imGray, Eigen::Matrix3d _K)
{
    if(_label == -1)
    {
        label = _label;
        pObj3D = NULL;
        has3D = false;
        K = _K;
        im_width = imGray.cols;
        im_height = imGray.rows;

        for(int i=0; i<(int)_lines.size(); i++)
        {
            // 判断长度是否符合阈值要求
            // 但是opencv的LSD输出中没有线段宽度
            // 生成线段对象
            Line_seg* line_temp = new Line_seg(_lines[i], K);
            lines.push_back(line_temp);
        }
        // 按照线段长度进行排序
        sort(lines.begin(), lines.end(), [](Line_seg* _a, Line_seg* _b){
            if(_a->length > _b->length)
            {
                return true;
            }
            return false;
        });
        lineNum = (int)lines.size();
        int needNum = min(30, lineNum);
        for(int i=0; i<needNum; i++)
        {
            selectedLineID.push_back(i);
        }
        if(lineNum>needNum)
        {
            // initial the observation matrix
            Eigen::MatrixXi obMat = Eigen::MatrixXi::Zero(lineNum, lineNum);
            // get the angle between any two line segments
            vector<double> angles;
            for(int i=0; i<lineNum; i++)
            {
                angles.push_back(lines[i]->angle);
            }

            for(int i=0; i<lineNum-1; i++)
            {
                for(int j=i+1; j<lineNum; j++)
                {
                    // double angle = getAngle(angles[i], angles[j]);
                    double angle = getAngle(lines[i]->angle, lines[j]->angle);
                    // if the angle is small and the distance is large, the two line segments are maybe parallel
                    if(angle < M_PI/6)
                    {
                        // the distance need to be large
                        // double lineDist = abs(pObj2D->lines[i]->parameter(2)-pObj2D->lines[j]->parameter(2));
                        // if(lineDist > max(min(pObj2D->width, pObj2D->height)/10, 5.0))
                        {
                            // the two line segments maybe parallel
                            obMat(i,j)=0;
                            obMat(j,i)=0;
                        }
                        // 
                    }
                    else
                    {
                        obMat(i,j)=1;
                        obMat(j,i)=1;
                    }
                }
            }
            int notParaID = needNum-5;
            for(int i=needNum-4; i<lineNum; i++)
            {
                bool isPara = true;
                for(int j=0; j<needNum-4; j++)
                {
                    if(obMat(i,j)==1)
                    {
                        isPara = false;
                    }
                }
                if(!isPara)
                {
                    notParaID = notParaID+1;
                    selectedLineID[notParaID] = i;
                    if(notParaID>needNum-1)
                    {
                        break;
                    }
                }
            }
        }
        int needNum1 = 200;
        cout << "lineNum: " << lineNum << endl;
        cout << "needNum: " << needNum << endl;
        if(lineNum>needNum1)
        {
            for(int i=needNum1; i<lineNum;i++)
            {
                lines.pop_back();
            }
            lineNum = lines.size();
        }
    }
    else
    {
        id = _id;
        BoundingBox(0) = x1;
        BoundingBox(1) = y1;
        BoundingBox(2) = x2;
        BoundingBox(3) = y2;
        Center(0) = (x1+x2)/2;
        Center(1) = (y1+y2)/2;
        width = abs(x2-x1);
        height = abs(y2-y1);

        NodeRotTh = pow(width*height,0.5)*4;
        NodeTranTh = pow(width*height,0.5)*2;

        label = _label;
        conf = _conf;
        pObj3D = NULL;
        has3D = false;
        K = _K;
        im_width = imGray.cols;
        im_height = imGray.rows;
        // 寻找在边界框内部的线段
        selectLinesInBB(_lines);

        width_expand_scale = 30;
        height_expand_scale = 30;

        expand_x1 = max(0.0, x1-width/width_expand_scale);
        expand_y1 = max(0.0, y1-height/height_expand_scale);
        expand_x2 = min(double(im_width), x2+width/width_expand_scale);
        expand_y2 = min(double(im_height), y2+height/height_expand_scale);

        // 对四个边界线生成Line_seg对象
        BBlines.push_back(new Line_seg(cv::Vec4f(x1,y1,x2,y1), K));
        BBlines.push_back(new Line_seg(cv::Vec4f(x1,y1,x1,y2), K));
        BBlines.push_back(new Line_seg(cv::Vec4f(x1,y2,x2,y2), K));
        BBlines.push_back(new Line_seg(cv::Vec4f(x2,y1,x2,y2), K));

        BBExpandLines.push_back(new Line_seg(cv::Vec4f(expand_x1,expand_y1,expand_x2,expand_y1), K));
        BBExpandLines.push_back(new Line_seg(cv::Vec4f(expand_x1,expand_y1,expand_x1,expand_y2), K));
        BBExpandLines.push_back(new Line_seg(cv::Vec4f(expand_x1,expand_y2,expand_x2,expand_y2), K));
        BBExpandLines.push_back(new Line_seg(cv::Vec4f(expand_x2,expand_y1,expand_x2,expand_y2), K));
    }

}

Obj2D::~Obj2D()
{
    for(int i=0; i<(int)lines.size(); i++)
    {
        if(lines[i])
        {
            delete lines[i];
            lines[i] = NULL;
        }
    }
    for(int i=0; i<(int)BBlines.size(); i++)
    {
        if(BBlines[i])
        {
            delete BBlines[i];
            BBlines[i] = NULL;
        }
    }
    if(pIT)
    {
        delete pIT;
        pIT = NULL;
    }
}

// select the line segments in the bounding box
void Obj2D::selectLinesInBB(std::vector<cv::Vec4f>& _lines)
{   
    lines.clear();
    // 扩展边界框
    cout << "width/5: " << width/10 << endl;
    cout << "height/5: " << height/10 << endl;
    double minx = BoundingBox(0) - min(10.0, width/10);
    double miny = BoundingBox(1) - min(10.0, height/10);
    double maxx = BoundingBox(2) + min(10.0, width/10);
    double maxy = BoundingBox(3) + min(10.0, height/10);

    // 遍历线段，判断是不是在BB内部；
    for(int i=0; i<(int)_lines.size(); i++)
    {
        // 判断长度是否符合阈值要求
        // 但是opencv的LSD输出中没有线段宽度
        // 生成线段对象
        Line_seg* line_temp = new Line_seg(_lines[i], K);
        // 判断line_temp是否在目标框内
        // if(line_temp->ifInBB(minx, miny, maxx, maxy) && line_temp->length > 1)
        // cout << "line length: " << line_temp->length << endl;
        if(line_temp->ifInBB(minx, miny, maxx, maxy))
        {
            // get a dist map for every line segment?
            // line_temp->getDistMap(minx, miny, maxx, maxy, im_width, im_height);
            // 添加到vector中
            lines.push_back(line_temp);
        }
        // 如果不在
        else
        {
            // 删除该线段
            delete line_temp;
        }
    }

    // 按照线段长度进行排序
    sort(lines.begin(), lines.end(), [](Line_seg* _a, Line_seg* _b){
        if(_a->length > _b->length)
        {
            return true;
        }
        return false;
    });
    lineNum = (int)lines.size();
    int needNum = min(10, lineNum);
    for(int i=0; i<needNum; i++)
    {
        selectedLineID.push_back(i);
    }
    if(lineNum>needNum)
    {
        // initial the observation matrix
        Eigen::MatrixXi obMat = Eigen::MatrixXi::Zero(lineNum, lineNum);
        // get the angle between any two line segments
        vector<double> angles;
        for(int i=0; i<lineNum; i++)
        {
            angles.push_back(lines[i]->angle);
        }

        for(int i=0; i<lineNum-1; i++)
        {
            for(int j=i+1; j<lineNum; j++)
            {
                // double angle = getAngle(angles[i], angles[j]);
                double angle = getAngle(lines[i]->angle, lines[j]->angle);
                // if the angle is small and the distance is large, the two line segments are maybe parallel
                if(angle < M_PI/6)
                {
                    // the distance need to be large
                    // double lineDist = abs(pObj2D->lines[i]->parameter(2)-pObj2D->lines[j]->parameter(2));
                    // if(lineDist > max(min(pObj2D->width, pObj2D->height)/10, 5.0))
                    {
                        // the two line segments maybe parallel
                        obMat(i,j)=0;
                        obMat(j,i)=0;
                    }
                    // 
                }
                else
                {
                    obMat(i,j)=1;
                    obMat(j,i)=1;
                }
            }
        }
        int notParaID = needNum-3;
        for(int i=needNum-2; i<lineNum; i++)
        {
            bool isPara = true;
            for(int j=0; j<needNum-2; j++)
            {
                if(obMat(i,j)==1)
                {
                    isPara = false;
                }
            }
            if(!isPara)
            {
                notParaID = notParaID+1;
                selectedLineID[notParaID] = i;
                if(notParaID>needNum-1)
                {
                    break;
                }
            }
        }
    }
    int needNum1 = 15;
    if(lineNum>needNum1)
    {
        for(int i=needNum1; i<lineNum;i++)
        {
            lines.pop_back();
        }
        lineNum = lines.size();
    }
}

void Obj2D::getObj3D(Eigen::Matrix3d K)
{
    // initial the IT
    pIT = new IT(this);

    pIT->constructIT(pIT->root);
    pIT->getObj3D(pObj3D, pSRG);
}


void Obj2D::DrawObj(cv::Mat &im)
{
    if(pObj3D == NULL)
    {
        cout << "pObj3D NULL" << endl;
        return;
    }

    if(label == -1)
    {
        pObj3D->Center(0) = 0;
        pObj3D->Center(1) = 0;
        pObj3D->Center(2) = 10;
        pObj3D->Axes(0) = 1;
        pObj3D->Axes(1) = 1;
        pObj3D->Axes(2) = 1;
        pObj3D->hasTrans = true;
    }

    if(!pObj3D->hasTrans)
    {
        cout << "hasTrans NULL" << endl;
        return;
    }

    pObj3D->getEightPointsInCuboid();


    pObj3D->getEightPointsInCamera();

    // the fast point ID in the eight points
    int fastPID = 0;
    double fastDist = 0;
    // compute the eightPoint in the image plane;
    Eigen::Matrix4d MstTemp;
    MstTemp.setIdentity();
    Eigen::Matrix<double, 3, 8> eightPointsInImg = K*MstTemp.block(0,0,3,4)*pObj3D->eightPointsInCamera;
    // normalize the points in img
    vector<cv::Point2f> frameCubePointsOp;
    for(int i=0; i<8; i++)
    {
        frameCubePointsOp.push_back(cv::Point2f(eightPointsInImg(0,i)/eightPointsInImg(2,i), eightPointsInImg(1,i)/eightPointsInImg(2,i)));
        if(pObj3D->eightPointsInCamera(2,i)>fastDist)
        {
            fastPID = i;
            fastDist = pObj3D->eightPointsInCamera(2,i);
        }
    }


    Eigen::Vector4d ProBB;
    ProBB.setZero();
    ProBB(0) = frameCubePointsOp[0].x;
    ProBB(1) = frameCubePointsOp[0].y;
    ProBB(2) = frameCubePointsOp[0].x;
    ProBB(3) = frameCubePointsOp[0].y;

    for(int fcp_id=0; fcp_id<8; fcp_id++)
    {
        if(frameCubePointsOp[fcp_id].x<ProBB(0))
        {
            ProBB(0) = frameCubePointsOp[fcp_id].x;
        }
        if(frameCubePointsOp[fcp_id].x>ProBB(2))
        {
            ProBB(2) = frameCubePointsOp[fcp_id].x;
        }
        if(frameCubePointsOp[fcp_id].y<ProBB(1))
        {
            ProBB(1) = frameCubePointsOp[fcp_id].y;
        }
        if(frameCubePointsOp[fcp_id].y>ProBB(3))
        {
            ProBB(3) = frameCubePointsOp[fcp_id].y;
        }
    }

    // cout << "ProBB: " << ProBB << endl;
    // 计算投影框与原始框的交并比
    double xmin = max(double(BoundingBox(0)), ProBB(0));
    double ymin = max(double(BoundingBox(1)), ProBB(1));
    // 求交集的右下角的点
    double xmax = min(double(BoundingBox(2)), ProBB(2));
    double ymax = min(double(BoundingBox(3)), ProBB(3));
    // 计算当前帧目标的面积
    double S1 = (BoundingBox(2)-BoundingBox(0))*(BoundingBox(3)-BoundingBox(1));
    // 计算上一帧目标的面积
    double S2 = (ProBB(2)-ProBB(0))*(ProBB(3)-ProBB(1));
    // 计算交部分面积
    double IOU = 0.0;
    if (xmax-xmin<=0||ymax-ymin<=0)
    {
        IOU = 0.0;
    }
    else
    {
        double SIn = (xmax-xmin)*(ymax-ymin);
        // 计算IOU
        IOU = SIn/(S1+S2-SIn);
    }
    
    // std::cout << "IOU in draw: " << IOU << std::endl;

    // std::cout << "ProBB: " << ProBB << std::endl;
    // std::cout << "BoundingBox: " << BoundingBox << std::endl;


    cv::Scalar cube_color11(121,0,0);
    cv::Scalar cube_color21(0,121,0);
    cv::Scalar cube_color31(0,0,121);
    cv::Scalar cube_color41(121,121,0);
    int cube_thickness1 = 4;
    int cube_thickness2 = 6;
    vector<Eigen::Vector2i> drawPairs;
    vector<cv::Scalar> cube_colors;
    vector<int> cube_thicknesses;

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

    // 顶部
    drawPairs.push_back(Eigen::Vector2i(0,1));
    drawPairs.push_back(Eigen::Vector2i(1,2));
    drawPairs.push_back(Eigen::Vector2i(2,3));
    drawPairs.push_back(Eigen::Vector2i(3,0));
    cube_colors.push_back(cv::Scalar(255, 0, 0));
    cube_thicknesses.push_back(4);

    // 底部
    drawPairs.push_back(Eigen::Vector2i(4,5));
    drawPairs.push_back(Eigen::Vector2i(5,6));
    drawPairs.push_back(Eigen::Vector2i(6,7));
    drawPairs.push_back(Eigen::Vector2i(7,4));
    cube_colors.push_back(cv::Scalar(0, 255, 0));
    cube_thicknesses.push_back(4);

    // 侧面
    drawPairs.push_back(Eigen::Vector2i(0,4));
    drawPairs.push_back(Eigen::Vector2i(1,5));
    drawPairs.push_back(Eigen::Vector2i(2,6));
    drawPairs.push_back(Eigen::Vector2i(3,7));
    cube_colors.push_back(cv::Scalar(0, 0, 255));
    cube_thicknesses.push_back(4);


    for(int i=0; i<3; i++)
    {
        for(int j=0; j<4; j++)
        {
            if(drawPairs[i*4+j][0]==fastPID||drawPairs[i*4+j][1]==fastPID)
            {
                cv::line(im, frameCubePointsOp[drawPairs[i*4+j][0]], frameCubePointsOp[drawPairs[i*4+j][1]], cube_colors[i], 1);
            }
        }
    }

    for(int i=0; i<3; i++)
    {
        for(int j=0; j<4; j++)
        {
            if(drawPairs[i*4+j][0]==fastPID||drawPairs[i*4+j][1]==fastPID)
            {
                continue;
            }
            else
            {
                cv::line(im, frameCubePointsOp[drawPairs[i*4+j][0]], frameCubePointsOp[drawPairs[i*4+j][1]], cube_colors[i], cube_thicknesses[i]);
            }
            
        }
    }
}


void Obj2D::DrawLines(cv::Mat &im)
{
    // 绘制其它线段
    // for(int i=0; i<min(int(lines.size()),10); i++)
    // {
    //     // cv::line(im, cv::Point2f(lines[i]->left_point(0), lines[i]->left_point(1)), cv::Point2f(lines[i]->right_point(0), lines[i]->right_point(1)), (120,120,120), 3);
    //     cout << "line length " << i << ": " << lines[i]->length << endl;

    // }
    for(int i=0; i<(int)selectedLineID.size(); i++)
    {
        cv::line(im, cv::Point2f(lines[selectedLineID[i]]->left_point(0), lines[selectedLineID[i]]->left_point(1)), cv::Point2f(lines[selectedLineID[i]]->right_point(0), lines[selectedLineID[i]]->right_point(1)), (120,120,120), 3);

    }

}

}