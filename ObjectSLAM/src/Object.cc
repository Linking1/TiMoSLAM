#include "Object.h"
#include<Eigen/StdVector>

#define M_PI 3.14159265358979323846

namespace ORB_SLAM2
{

// 构造函数
Obj2D::Obj2D(Frame* _pFrame, double x1, double y1, double x2, double y2, int label, double conf, const vector<vector<double>> &vLines)
{
    lineWidTh = 0;
    pFrame = _pFrame;
    evecCenter << (x1+x2)/2, (y1+y2)/2;
    evecLeftUp << x1, y1;
    evecRightDown << x2, y2;
    iLabel = label;
    dConf = conf;
    width = abs(x2-x1);
    height = abs(y2-y1);
    iObj3DMatchedID = -1;
    objRect = cv::Rect(evecLeftUp[0], evecLeftUp[1], width, height);
    GetLineInBBox(vLines);
    GetParaLines();
    hasLastObj = false;
    if((int)_pFrame->mvpLastObj2Ds.size()>0)
    {
        pLastObj3D = _pFrame->mvpLastObj2Ds[0]->pObj3D;
        hasLastObj = true;
    }
    
    pObj3D = new Obj3D(this, _pFrame->mK);
}

// 获取二维线段的参数
void Obj2D::GetLineParameters(const Eigen::Matrix<double,1,5> &inLine, Eigen::Vector3d &LineParameter)
{
    LineParameter(0) = inLine(2) - inLine(3);
    LineParameter(1) = inLine(1) - inLine(0);
    LineParameter(2) = -inLine(2)*LineParameter(1) - inLine(0)*LineParameter(0);
    double _mo = pow(pow(LineParameter(0), 2.0)+pow(LineParameter(1), 2.0), 0.5);
    LineParameter(0) = LineParameter(0)/_mo;
    LineParameter(1) = LineParameter(1)/_mo;
    LineParameter(2) = LineParameter(2)/_mo;
}
// 寻找在边界框内的线段
void Obj2D::GetLineInBBox(const vector<vector<double>> &vLines)
{
    /** 该函数完成两个功能
     * 1. 保留在目标框内部的线
     * 2. 按照线段长度对线段进行排序，由长到短
     **/
    // 扩展边界框
    double minx = evecLeftUp[0] - min(5.0, width/5);
    double miny = evecLeftUp[1] - min(5.0, height/5);
    double maxx = evecRightDown[0] + min(5.0, width/5);
    double maxy = evecRightDown[1] + max(5.0, height/5);
    // 遍历线段
    for(int i=0; i<(int)vLines.size(); i++)
    {
        if(vLines[i][4] < lineWidTh)
        {
            continue;
        }
        Eigen::Matrix<double, 1, 5> lineCur;
        double lineLength = pow(pow(vLines[i][0]-vLines[i][1], 2.0) + pow(vLines[i][2]-vLines[i][3], 2.0), 0.5);
        // 如果线段太短，则删除
        if(lineLength < 1)
        {
            continue;
        }
        // 判断线段是否在bbox内部
        if(vLines[i][0] > minx && vLines[i][0] < maxx && vLines[i][1] > minx && vLines[i][1] < maxx && vLines[i][2] > miny && vLines[i][2] < maxy && vLines[i][3] > miny && vLines[i][3] < maxy)
        {
            // 将线段修改为从左指向右
            if(vLines[i][0] > vLines[i][1])
            {
                lineCur << vLines[i][1], vLines[i][0], vLines[i][3], vLines[i][2], lineLength;
                emvLines.push_back(lineCur);
            }
            else
            {
                lineCur << vLines[i][0], vLines[i][1], vLines[i][2], vLines[i][3], lineLength;
                emvLines.push_back(lineCur);
            }
        }
    }
    // 按照线段长度进行排序
    sort(emvLines.begin(), emvLines.end(), [](Eigen::Matrix<double, 1, 5> i, Eigen::Matrix<double, 1, 5> j){
        if (i[4] > j[4])
        {
            return true;
        }
        return false;
    });
}
// 合并线段
void Obj2D::MergeLine()
{

}
// 寻找平行线
void Obj2D::GetParaLines()
{
    // vector<Eigen::Matrix<double, 2, 5>> emvParaLines;
    emvParaLines.clear();
    int linesNum = emvLines.size();
    vector<double> angles;
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> LineParameters;
    for(int i=0; i<linesNum; i++)
    {
        angles.push_back(atan2(emvLines[i][3]-emvLines[i][2], abs(emvLines[i][1]-emvLines[i][0])));
        Eigen::Vector3d line1Par;
        GetLineParameters(emvLines[i], line1Par);
        LineParameters.push_back(line1Par);
        // cout << "angle: " << angles[i] << endl;

    }
    double disMin_th = min(width/10, height/10);
    int _index = 0;
    for(int i=0; i<linesNum; i++)
    {
        double line1Dist = LineParameters[i][2];
        for(int j=i+1; j<linesNum; j++)
        {
            double angleSum = 0;
            if(angles[i]*angles[j] < 0)
            {
                angleSum = abs(angles[i])+abs(angles[j]);
            }
            else
            {
                angleSum = abs(angles[i] - angles[j]);
            }

            if(angleSum > M_PI/2)
            {
                angleSum = M_PI - angleSum;
            }
            
            if(angleSum < M_PI/3)
            {
                double line2Dist = LineParameters[j][2];
                double lineDist = abs(line1Dist - line2Dist);
                if(j != i && lineDist > disMin_th)
                {
                    _index++;
                    Eigen::Matrix<double, 2, 5> paraLineCur;
                    paraLineCur << emvLines[i], emvLines[j];
                    emvParaLines.push_back(paraLineCur);
                }
            }
        }
    }
}

cv::Rect Obj2D::GetRect()
{
    return objRect;
}

vector<vector<cv::Point2f>> Obj2D::GetLines()
{
    vector<vector<cv::Point2f>> outLines;
    for(int i=0; i<(int)emvLines.size(); i++)
    {
        cv::Point2f pt1(emvLines[i][0], emvLines[i][2]);
        cv::Point2f pt2(emvLines[i][1], emvLines[i][3]);
        vector<cv::Point2f> lineCur {pt1, pt2};
        outLines.push_back(lineCur);
    }
    return outLines;
}

vector<vector<cv::Point2f>> Obj2D::PopParaLines()
{
    vector<vector<cv::Point2f>> outLines;
    for(int i=0; i<(int)emvParaLines.size(); i++)
    {
        cv::Point2f pt1(emvParaLines[i](0,0), emvParaLines[i](0,2));
        cv::Point2f pt2(emvParaLines[i](0,1), emvParaLines[i](0,3));
        cv::Point2f pt3(emvParaLines[i](1,0), emvParaLines[i](1,2));
        cv::Point2f pt4(emvParaLines[i](1,1), emvParaLines[i](1,3));
        vector<cv::Point2f> lineCur {pt1, pt2, pt3, pt4};
        outLines.push_back(lineCur);
    }
    return outLines;
}

Obj3D::Obj3D(Eigen::Matrix<double, 8, 3> &_cubePoints)
{
    cubePoints = Eigen::Matrix<double, 8, 3>::Zero();
    for(int i=0;i<8;i++)
    {
        for(int j=0;j<3;j++)
        {
            cubePoints(i,j) = _cubePoints(i,j);
        }
    }
}

Obj3D::Obj3D(Obj2D* _pObj2D, cv::Mat mK)
{

    directSim = 0;
    sizeSim = 0;
    pObj2D = _pObj2D;
    K = Converter::toMatrix3d(mK);
    has3DObj = false;
    label = 0;
    conf = 1.0;
    framePointsx.setZero();
    Get3DObj();

    Eigen::Matrix<double, 12, 3> cubePointsx;
    GetCubePoints(cubePointsx);
    for(int i=0; i<8; i++)
    {
        framePointsx(i,0) = cubePointsx(i, 0)/cubePointsx(i, 2);
        framePointsx(i,1) = cubePointsx(i, 1)/cubePointsx(i, 2);
    }
}

void Obj3D::Get3DObj()
{
    mnId = 0;
    Eigen::Matrix3d R;
    // cout << "======================" << endl;
    Eigen::Matrix3d R_Last;
    // 判断是否有指导目标
    if(pObj2D->hasLastObj)
    {
        double lastDirectSim = pObj2D->pLastObj3D->directSim;
        if(lastDirectSim > 2.4)
        {
            // 如果有指导目标，则根据指导检测目标
            R_Last = pObj2D->pLastObj3D->R_3D;
            // GetDirection(R, R_Last);
            GetDirection(R);
        }
        else
        {
            // 没有指导目标，则重新进行检测
            // R_Last = Eigen::Matrix3d::Identity();
            R_Last = pObj2D->pLastObj3D->R_3D;

            // GetDirection(R, R_Last);
            GetDirection(R);


        }
        
    }
    else
    {
        // 没有指导目标，则重新进行检测
        R_Last = Eigen::Matrix3d::Identity();
        GetDirection(R);
    }
    

    // 对R进行方向校准，尽量与相机坐标系类似
    Eigen::Matrix<double, 3, 12> x_R;
    Eigen::Matrix<double, 3, 12> y_R;
    Eigen::Matrix<double, 3, 12> z_R;
    Eigen::Matrix3d testDire;
    x_R <<  1, 0, 0,   1, 0, 0,   1, 0, 0,   1, 0, 0,
            0, 1, 0,   0, 0,-1,   0,-1, 0,   0, 0, 1,
            0, 0, 1,   0, 1, 0,   0, 0,-1,   0,-1, 0;
    
    y_R <<  1, 0, 0,   0, 0, 1,  -1, 0, 0,   0, 0,-1,
            0, 1, 0,   0, 1, 0,   0, 1, 0,   0, 1, 0,
            0, 0, 1,  -1, 0, 0,   0, 0,-1,   1, 0, 0;

    z_R <<  1, 0, 0,   0,-1, 0,  -1, 0, 0,   0, 1, 0,
            0, 1, 0,   1, 0, 0,   0,-1, 0,  -1, 0, 0,
            0, 0, 1,   0, 0, 1,   0, 0, 1,   0, 0, 1;
    
    testDire << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;
    
    Eigen::Matrix3d outDire = R*testDire;
    Eigen::Matrix3d simDire = outDire.transpose()*testDire;
    
    Eigen::Matrix3d R_Last_in = R_Last.inverse();
    // Eigen::Matrix3d R_Last_in = pObj2D->R_Last3D;

    Eigen::Matrix3d R_jiao;
    Eigen::Matrix3d R_orig;
    R_orig << R(0,0), R(0,1), R(0,2),
              R(1,0), R(1,1), R(1,2),
              R(2,0), R(2,1), R(2,2);
    double minSimDireSum = 0;
    for(int i=0; i<4; i++)
    {
        R_jiao = R_orig*x_R.block(0,i*3,3,3);
        for(int j=0; j<4; j++)
        {
            R_jiao = R_jiao*y_R.block(0,j*3,3,3);
            for(int _m=0; _m<4; _m++)
            {
                R_jiao = R_jiao*z_R.block(0,_m*3,3,3);
                Eigen::Matrix3d outDire = R_Last_in*R_jiao*testDire;
                Eigen::Matrix3d simDire = outDire.transpose()*testDire;
                double simDireSum = simDire(0,0)+simDire(1,1)+abs(simDire(2,2));
                // if(simDire(0,0)>=0&&simDire(1,1)>=0&&simDire(2,2)>=0&&simDireSum>minSimDireSum)
                if(simDire(1,1)>=0&&simDire(1,1)>minSimDireSum)
                {
                    // cout << "======================get jiao" << endl;
                    // R = R_jiao;
                    for(int R_i=0; R_i<3; R_i++)
                    {
                        for(int R_j=0; R_j<3; R_j++)
                        {
                            R(R_i, R_j) = R_jiao(R_i, R_j);
                        }
                    }
                    minSimDireSum = simDire(1,1);
                    // minSimDireSum = simDireSum;
                    // break;
                }
            }
        }
    }

    // R_jiao = R;
    minSimDireSum = 0;
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            R_orig(i,j) = R(i,j);
        }
    }
    for(int i=0; i<4; i++)
    {
        R_jiao = R_orig*y_R.block(0,i*3,3,3);
        Eigen::Matrix3d outDire = R_Last_in*R_jiao*testDire;
        Eigen::Matrix3d simDire = outDire.transpose()*testDire;
        double simDireSum = simDire(0,0)+simDire(2,2);
        // if(simDire(0,0)>=0&&simDire(1,1)>=0&&simDire(2,2)>=0&&simDireSum<minSimDireSum)
        // cout << "simDire: " << simDire(0,0) << endl;
        if(simDire(0,0)>=0&&simDire(0,0)>minSimDireSum)
        {
            // cout << "======================get jiao" << endl;
            // R = R_jiao;
            for(int R_i=0; R_i<3; R_i++)
            {
                for(int R_j=0; R_j<3; R_j++)
                {
                    R(R_i, R_j) = R_jiao(R_i, R_j);
                }
            }
            if(simDire(2,2)<0)
            {
                for(int R_i=0; R_i<3; R_i++)
                {
                    R(R_i, 2) = -R_jiao(R_i, 2);
                }
            }
            minSimDireSum = simDire(0,0);
            // break;
        }
    }

    for(int R_i=0; R_i<3; R_i++)
    {
        for(int R_j=0; R_j<3; R_j++)
        {
            R_3D(R_i, R_j) = R(R_i, R_j);
        }
    }

    outDire = R_Last_in*R*testDire;
    simDire = outDire.transpose()*testDire;

    Eigen::Matrix4d Mst;
    Mst.setIdentity();
    Mst.block(0,0,3,3) = R;
    Eigen::Matrix4d estQ = Eigen::Matrix4d::Zero();
    // estQ << 1.0, 0.0, 0.0, 0.0,
    //         0.0, 2.0, 0.0, 0.0,
    //         0.0, 0.0, 3.0, 0.0,
    //         0.0, 0.0, 0.0,-1.0;
    // Eigen::Matrix4d T;
    // T.setIdentity();
    // T(2,3) = 10;
    // estQ = Mst*estQ*Mst.transpose();
    // estQ = T*estQ*T.transpose();
    
    if(pObj2D->hasLastObj)
    {
        double lastSizeSim = pObj2D->pLastObj3D->sizeSim;
        if(lastSizeSim < 10)
            // GetEllipsoidFromLast(R, estQ);
        {
            // GetEllipsoid(R, estQ);
            GetEllipsoidFromLast(R, estQ);
        }
        else
        {
            // GetEllipsoid(R, estQ);
            GetEllipsoidFromLast(R, estQ);

        }
    }
    else
    {
        // GetEllipsoid(R, estQ);
        GetEllipsoidFromLast(R, estQ);
    }
    

    estQ = Mst*estQ*Mst.transpose();

    // 相机坐标系下的椭球方程
    Quadric << estQ(0,0), estQ(0,1), estQ(0,2), estQ(0,3),
                estQ(1,0), estQ(1,1), estQ(1,2), estQ(1,3),
                estQ(2,0), estQ(2,1), estQ(2,2), estQ(2,3),
                estQ(3,0), estQ(3,1), estQ(3,2), estQ(3,3);

    Get3DObjInfo(R);
}

void Obj3D::SetWorldPos(const cuboid &Pos)
{
    pose = Pos.pose;
    scale = Pos.scale;
    // 获取cube八个点的模型参数
    Eigen::Matrix4d res = pose.to_homogeneous_matrix();
    R_3D = res.block(0,0,3,3);
    Center(0) = res(0,3);
    Center(1) = res(1,3);
    Center(2) = res(2,3);
    Axes(0) = scale(0);
    Axes(1) = scale(1);
    Axes(2) = scale(2);
}

cuboid Obj3D::getCuboid()
{
    cuboid newCuboid;
    newCuboid.pose.setRotation(Eigen::Quaterniond(R_3D));
    newCuboid.pose.setTranslation(Center);
    newCuboid.scale = Axes;
    return newCuboid;
}

void Obj3D::Get3DObjInfo()
{

    cubePoints <<   -Axes(0), -Axes(1),  Axes(2),
                    -Axes(0),  Axes(1),  Axes(2),
                     Axes(0),  Axes(1),  Axes(2),
                     Axes(0), -Axes(1),  Axes(2),
                    -Axes(0), -Axes(1), -Axes(2),
                    -Axes(0),  Axes(1), -Axes(2),
                     Axes(0),  Axes(1), -Axes(2),
                     Axes(0), -Axes(1), -Axes(2);
    
    direPoints <<    0,   0,   0,
                   100,   0,   0,
                     0, 100,   0,
                     0,   0, 100;
    // 旋转八个点
    Eigen::Matrix<double, 3, 8> cubePoints1 = R_3D*cubePoints.transpose();
    Eigen::Matrix<double, 3, 4> direPoints1 = R_3D*direPoints.transpose();
    for(int i=0; i<8; i++)
    {
        cubePoints1(0,i) = cubePoints1(0,i)+Center(0);
        cubePoints1(1,i) = cubePoints1(1,i)+Center(1);
        cubePoints1(2,i) = cubePoints1(2,i)+Center(2);
    }
    for(int i=0; i<4; i++)
    {
        direPoints1(0,i) = direPoints1(0,i)+Center(0);
        direPoints1(1,i) = direPoints1(1,i)+Center(1);
        direPoints1(2,i) = direPoints1(2,i)+Center(2);
    }
    // 相机坐标系下的点
    cubePoints = cubePoints1.transpose();
    direPoints = direPoints1.transpose();
}

void Obj3D::Get3DObjInfo(Eigen::Matrix3d &_R)
{
    Eigen::Vector3d center;
    Eigen::Vector3d axes;
    Eigen::Matrix3d R;
    DetectObj3D::dual_quadric_to_ellipsoid_parameters(center, axes, R, Quadric);
    Center(0) = center(0);
    Center(1) = center(1);
    Center(2) = center(2);
    axes(0) = Axes(0);
    axes(1) = Axes(1);
    axes(2) = Axes(2);

    cubePoints <<   -axes(0), -axes(1),  axes(2),
                    -axes(0),  axes(1),  axes(2),
                     axes(0),  axes(1),  axes(2),
                     axes(0), -axes(1),  axes(2),
                    -axes(0), -axes(1), -axes(2),
                    -axes(0),  axes(1), -axes(2),
                     axes(0),  axes(1), -axes(2),
                     axes(0), -axes(1), -axes(2);
    
    direPoints <<    0,   0,   0,
                   100,   0,   0,
                     0, 100,   0,
                     0,   0, 100;
    // 旋转八个点
    Eigen::Matrix<double, 3, 8> cubePoints1 = _R*cubePoints.transpose();
    Eigen::Matrix<double, 3, 4> direPoints1 = _R*direPoints.transpose();
    for(int i=0; i<8; i++)
    {
        cubePoints1(0,i) = cubePoints1(0,i)+Center(0);
        cubePoints1(1,i) = cubePoints1(1,i)+Center(1);
        cubePoints1(2,i) = cubePoints1(2,i)+Center(2);
    }
    for(int i=0; i<4; i++)
    {
        direPoints1(0,i) = direPoints1(0,i)+Center(0);
        direPoints1(1,i) = direPoints1(1,i)+Center(1);
        direPoints1(2,i) = direPoints1(2,i)+Center(2);
    }
    // 相机坐标系下的点
    cubePoints = cubePoints1.transpose();
    direPoints = direPoints1.transpose();
    // cout << "Center in quadric: \n" << Center << endl;

}

void Obj3D::GetCubePoints(Eigen::Matrix<double, 12, 3> &frameCubePoints)
{
    // Eigen::Matrix4d transformMatrixInv;
    // transformMatrixInv.setIdentity();
    Eigen::MatrixXd cubePoints1(12, 4);
    for(int i=0; i<12; i++)
    {
        cubePoints1(i,3) = 1;
    }
    cubePoints1.block(0,0,8,3) = cubePoints;
    cubePoints1.block(8,0,4,3) = direPoints;
    Eigen::Matrix<double, 4, 12> cameraCubePoints = Eigen::Matrix4d::Identity()*cubePoints1.transpose();
    Eigen::Matrix<double, 3, 12> frameCubePoints1 = K*cameraCubePoints.block(0,0,3,12);
    frameCubePoints = frameCubePoints1.transpose();
}

void Obj3D::GetDirection(Eigen::Matrix3d &R)
{
    vector<Eigen::Matrix<double, 2, 5>, Eigen::aligned_allocator<Eigen::Matrix<double, 2, 5>>> emvParaLines = pObj2D->emvParaLines;
    int ParaLinesNum = (int)emvParaLines.size();
    Eigen::Matrix4d ObjMst;
    ObjMst.setIdentity();
    Eigen::Matrix<double, 3, 4> Pst1 = K*ObjMst.block(0,0,3,4);
    Eigen::Matrix<double, 4, 3> Pst = Pst1.transpose();
    // 声明约束矩阵
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(ParaLinesNum*2, 10);
    Eigen::MatrixXd directions = Eigen::MatrixXd::Zero(ParaLinesNum+1, 3);
    // cout << "ParaLinesNum: " << ParaLinesNum << endl;
    // 求解每一对平行线构成的约束和方向
    for(int i=0; i<ParaLinesNum; i++)
    {
        Eigen::Matrix<double, 2, 10> DirectionCons = Eigen::Matrix<double, 2, 10>::Zero();
        Eigen::Vector3d direction;
        direction << 0.0,0.0,0.0;
        DetectObj3D::getDirectConstraint(emvParaLines[i], DirectionCons, direction, Pst);
        M.block(i*2,0,2,10) = DirectionCons;
        directions.block(i,0,1,3) = direction.transpose();
    }

    Eigen::Vector3d _z_dire;
    // _z_dire = R_Last.block(0,1,3,1);
    _z_dire << -0.0175217, 0.748902, 0.662449;
    // cout << "R_Last: " << R_Last << endl; 
    directions.block(ParaLinesNum,0,1,3) = _z_dire.transpose();
    ParaLinesNum += 1;

    // 寻找互相垂直的平行线对
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> ver_id_list;
    Eigen::MatrixXd dire_angle_mat = directions*(directions.transpose());
    dire_angle_mat = dire_angle_mat.cwiseAbs();
    // cout << "dire_angle_mat: \n" << dire_angle_mat << endl;
    for(int i=0; i<ParaLinesNum; i++)
    {
        double para_dist_i = min(emvParaLines[i](0,4), emvParaLines[i](1,4));

        for(int j=i+1; j<ParaLinesNum; j++)
        {
            double angle = dire_angle_mat(i,j);
            double para_dist_j = min(emvParaLines[j](0,4), emvParaLines[j](1,4));
            double para_dist = min(para_dist_i, para_dist_j);
            if(angle < 0.2)
            {
                Eigen::Vector3d ver_id_cur;
                ver_id_cur << double(i), double(j), para_dist;
                ver_id_list.push_back(ver_id_cur);
            }
        }
    }
    // 按照长度排序
    sort(ver_id_list.begin(), ver_id_list.end(), [](Eigen::Vector3d i, Eigen::Vector3d j){
        if (i[2] > j[2])
        {
            return true;
        }
        return false;
    });
    // 找出前n对
    // cout <<"ver_id_list size: " << (int)ver_id_list.size() << endl;
    int iterNum = min(40, (int)ver_id_list.size());
    double maxNum = 0.0;

    Eigen::Vector3d minRes = Eigen::Vector3d::Zero();
    double minError = 10000000000000000000000000000.0;


    
    vector<Eigen::Matrix<double, 1, 5>, Eigen::aligned_allocator<Eigen::Matrix<double, 1, 5>>> objLines = pObj2D->emvLines;
    int lineNum = (int)objLines.size();
    Eigen::MatrixXd line_direct = Eigen::MatrixXd::Zero(lineNum, 10);
    Eigen::Matrix3d possible_dire;
    possible_dire << 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1;


    int max_id1 = -1;
    int max_id2 = -1;
    for(int i=0; i<iterNum; i++)//iterNum
    {
        int para1ID = int(ver_id_list[i](0));
        int para2ID = int(ver_id_list[i](1));
        
        // 此次迭代的约束矩阵
        Eigen::Matrix<double, 4, 10> M1;
        Eigen::Matrix<double, 4, 1> B;
        M1 << M.block(para1ID*2, 0, 2, 10), M.block(para2ID*2, 0, 2, 10);
        B = -M1.block(0,3,4,1) - 160000*M1.block(0,4,4,1) - 8100000000*M1.block(0,5,4,1);
        Eigen::Vector3d X_temp = M1.block(0,0,4,3).fullPivHouseholderQr().solve(B);
        // Eigen::Matrix<double, 4, 1> error = M1.block(0,0,4,3)*X_temp-B;
        Eigen::MatrixXd error = M.block(0,0,ParaLinesNum*2,3)*X_temp+M.block(0,3,ParaLinesNum*2,1) + 160000*M1.block(0,4,ParaLinesNum*2,1) + 8100000000*M1.block(0,5,ParaLinesNum*2,1);
        error = error.cwiseAbs();
        // cout << "error: " << error(para1ID*2)+error(para1ID*2+1)+error(para2ID*2)+error(para2ID*2+1) << endl;
        double error_sum = error.sum();
        double rightNum = 0.0;
        // if(error_sum < minError)
        // {
        //     minError = error_sum;
        //     minRes(0) = X_temp(0);
        //     minRes(1) = X_temp(1);
        //     minRes(2) = X_temp(2);
        // }

        Eigen::Matrix4d _Ellipsoid;
        _Ellipsoid <<            1, X_temp(0),  X_temp(1),  0,
                        X_temp(0),    160000,  X_temp(2),  0,
                        X_temp(1), X_temp(2), 8100000000,  0,
                                0,         0,          0, -1;
        Eigen::Vector3d _center;
        Eigen::Vector3d _axes;
        Eigen::Matrix3d _R;
        DetectObj3D::dual_quadric_to_ellipsoid_parameters(_center, _axes, _R, _Ellipsoid);

        Eigen::Matrix<double, 4, 3> _Mst;
        _Mst.setIdentity();
        _Mst.block(0,0,3,3) = _R.transpose();
        // 从图像平面到目标附体坐标系
        Eigen::Matrix<double, 4, 3> _Pst = _Mst*K.transpose();

        int lineNum = (int)objLines.size();
        for(int line_i=0; line_i<lineNum; line_i++)
        {
            Eigen::Matrix<double, 1, 3> line_para;
            DetectObj3D::get_bbline_parameters1(objLines[line_i], line_para);
            Eigen::Matrix<double, 4, 1> plane_para = _Pst*line_para.transpose();
            line_direct(line_i,0) = plane_para(0,0);
            line_direct(line_i,1) = plane_para(1,0);
            line_direct(line_i,2) = plane_para(2,0);
            line_direct.block(line_i,0,1,3) = line_direct.block(line_i,0,1,3).normalized();
            double min_dot = 100;
            int min_id = -1;
            for(int line_j=0; line_j<3; line_j++)
            {
                Eigen::Matrix<double, 1, 1> _dot = line_direct.block(line_i,0,1,3)*possible_dire.block(0, line_j, 3, 1);
                _dot = _dot.cwiseAbs();
                if(_dot(0,0) < min_dot)
                {
                    min_dot = _dot(0,0);
                    min_id = line_j;
                }
            }

            if(min_dot < 0.05)
            {
                rightNum += objLines[line_i](4);
            }
        }

        if (rightNum > maxNum)
        {
            max_id1 = para1ID;
            max_id2 = para2ID;
            maxNum = rightNum;
            minRes(0) = X_temp(0);
            minRes(1) = X_temp(1);
            minRes(2) = X_temp(2);
        }
    }
    Eigen::Matrix4d Ellipsoid;
    Ellipsoid <<            1, minRes(0),  minRes(1),  0,
                    minRes(0),    160000,  minRes(2),  0,
                    minRes(1), minRes(2), 8100000000,  0,
                            0,         0,          0, -1;
    
    Eigen::Vector3d center;
    Eigen::Vector3d axes;
    // Eigen::Matrix3d R;
    DetectObj3D::dual_quadric_to_ellipsoid_parameters(center, axes, R, Ellipsoid);



    for(int i=0; i<ParaLinesNum; i++)
    {
        double rightNum = 0.0;

        double angle = dire_angle_mat(ParaLinesNum-1,i);
        if(angle > 0.2)
        {
            continue;
        }
        Eigen::Vector3d directionx, directiony, directionz;
        directionx = directions.block(i,0,1,3).transpose();
        directiony = directions.block(ParaLinesNum-1,0,1,3).transpose();
        directionz = directionx.cross(directiony);
        
        Eigen::Matrix3d _R;
        _R.block(0,0,3,1) = directionx;
        _R.block(0,1,3,1) = directiony;
        _R.block(0,2,3,1) = directionz;


        Eigen::Matrix<double, 4, 3> _Mst;
        _Mst.setIdentity();
        _Mst.block(0,0,3,3) = _R.transpose();
        // 从图像平面到目标附体坐标系
        Eigen::Matrix<double, 4, 3> _Pst = _Mst*K.transpose();

        int lineNum = (int)objLines.size();
        for(int line_i=0; line_i<lineNum; line_i++)
        {
            Eigen::Matrix<double, 1, 3> line_para;
            DetectObj3D::get_bbline_parameters1(objLines[line_i], line_para);
            Eigen::Matrix<double, 4, 1> plane_para = _Pst*line_para.transpose();
            line_direct(line_i,0) = plane_para(0,0);
            line_direct(line_i,1) = plane_para(1,0);
            line_direct(line_i,2) = plane_para(2,0);
            line_direct.block(line_i,0,1,3) = line_direct.block(line_i,0,1,3).normalized();
            double min_dot = 100;
            int min_id = -1;
            for(int line_j=0; line_j<3; line_j++)
            {
                Eigen::Matrix<double, 1, 1> _dot = line_direct.block(line_i,0,1,3)*possible_dire.block(0, line_j, 3, 1);
                _dot = _dot.cwiseAbs();
                if(_dot(0,0) < min_dot)
                {
                    min_dot = _dot(0,0);
                    min_id = line_j;
                }
            }

            if(min_dot < 0.05)
            {
                rightNum += objLines[line_i](4);
            }
        }

        if (rightNum > maxNum)
        {
            maxNum = rightNum;
            R = _R;
        }
    }



    // 得到R之后，选择符合条件的平行线对
    std::vector<int> paraSelectedID;
    Eigen::Matrix3d estimate_direction = R*Eigen::Matrix3d::Identity();
    for(int i=0; i<3; i++)
    {
        estimate_direction.block(0,i,3,1) = estimate_direction.block(0,i,3,1).normalized();
    }
    for(int i=0; i<ParaLinesNum; i++)
    {
        double max_dot = 0;
        for(int j=0; j<3; j++)
        {
            Eigen::Matrix<double, 1, 1> _dot = directions.block(i,0,1,3)*estimate_direction.block(0,j,3,1);
            _dot = _dot.cwiseAbs();
            if(_dot(0,0)>max_dot)
            {
                max_dot = _dot(0,0);
            }
        }
        if(max_dot > 0.95)
        {
            paraSelectedID.push_back(i);
        }

    }
    int paraSelectedNum = paraSelectedID.size();
    // 构建约束矩阵
    Eigen::MatrixXd M2 = Eigen::MatrixXd::Zero(paraSelectedNum*2, 10);
    Eigen::MatrixXd B2 = Eigen::MatrixXd::Zero(paraSelectedNum*2, 1);
    for(int i=0; i<paraSelectedNum; i++)
    {
        M2.block(i*2, 0, 2, 10) = M.block(paraSelectedID[i]*2, 0, 2, 10);

    }
    B2 = -M2.block(0,3,paraSelectedNum*2,1) - 160000*M2.block(0,4,paraSelectedNum*2,1) - 8100000000*M2.block(0,5,paraSelectedNum*2,1);
    Eigen::Vector3d X_temp1 = M2.block(0,0,paraSelectedNum*2,3).fullPivHouseholderQr().solve(B2);

    Eigen::Matrix4d Ellipsoid1;
    Ellipsoid1 <<            1, X_temp1(0),  X_temp1(1),  0,
                    X_temp1(0),     160000,  X_temp1(2),  0,
                    X_temp1(1), X_temp1(2),  8100000000,  0,
                             0,          0,           0, -1;
    
    // DetectObj3D::dual_quadric_to_ellipsoid_parameters(center, axes, R, Ellipsoid);
    
}

void Obj3D::GetEllipsoid(Eigen::Matrix3d &R, Eigen::Matrix4d &estQ)
{
    // 获取线段对象
    vector<Eigen::Matrix<double, 1, 5>, Eigen::aligned_allocator<Eigen::Matrix<double, 1, 5>>> objLines = pObj2D->emvLines;
    // 获取边界框
    Eigen::Matrix<double, 4, 1> objBBS;
    objBBS << pObj2D->evecLeftUp(0), pObj2D->evecLeftUp(1), pObj2D->evecRightDown(0), pObj2D->evecRightDown(1);
    int lineNum = (int)objLines.size();

    // 在已知方向和线的情况下估计尺寸和平移
    Eigen::Matrix<double, 4, 6> planes_para = Eigen::Matrix<double, 4, 6>::Zero();
    // 约束矩阵
    Eigen::MatrixXd M_all = Eigen::MatrixXd::Zero(lineNum+5, 10);
    // 从相机坐标系到目标附体坐标系
    Eigen::Matrix<double, 4, 3> Mst;
    Mst.setIdentity();
    Mst.block(0,0,3,3) = R.transpose();
    // 从图像平面到目标附体坐标系
    Eigen::Matrix<double, 4, 3> Pst = Mst*K.transpose();

    // 获取四条边的约束
    Eigen::Matrix<double, 4, 10> plane_cons = Eigen::Matrix<double, 4, 10>::Zero();
    Eigen::Matrix<double, 4, 4> plane_dire = Eigen::Matrix<double, 4, 4>::Zero();
    DetectObj3D::get_plane_constraint_box(objBBS, Pst, Mst, plane_cons, plane_dire);



    // 图像平面上的线段参数
    Eigen::Matrix4d bb_lines;
    bb_lines << objBBS(0), objBBS(0), objBBS(0), objBBS(2),
                objBBS(2), objBBS(0), objBBS(2), objBBS(2),
                objBBS(1), objBBS(1), objBBS(3), objBBS(1),
                objBBS(1), objBBS(3), objBBS(3), objBBS(3);
    // 获取线段信息
    Eigen::Matrix<double, 4, 3> line_para;
    DetectObj3D::get_bbline_parameters(bb_lines, line_para);
    LineConstraint.block(0,0,3,4) = line_para.transpose();
    // 图像平面上LSD线段的参数
    for(int i=0; i<2; i++)
    {
        Eigen::Matrix<double, 1, 3> line_para_temp;
        DetectObj3D::get_bbline_parameters1(objLines[i], line_para_temp);
        LineConstraint.block(0,i+4,3,1) = line_para_temp.transpose();
    }

    M_all.block(0,0,4,10) = plane_cons;
    planes_para.block(0,0,4,4) = plane_dire;
    // 得到检测线段的约束
    Eigen::MatrixXd plane_cons_det = Eigen::MatrixXd::Zero(lineNum, 10);
    DetectObj3D::get_plane_constraint_detect(objLines, Pst, plane_cons_det);
    M_all.block(4,0,lineNum,10) = plane_cons_det;

    // 添加最后一个假设约束平面
    Eigen::Matrix<double, 4, 1> z_plane;
    z_plane << 0, 0, 1, -100;
    Eigen::Matrix4d Mst1;
    Mst1.setIdentity();
    Mst1.block(0,0,4,3) = Mst;
    Eigen::Matrix<double, 4, 1> z_plane_para = Mst1 * z_plane;
    M_all(lineNum+4, 0) = z_plane_para(0,0);
    M_all(lineNum+4, 1) = z_plane_para(1,0);
    M_all(lineNum+4, 2) = z_plane_para(2,0);
    M_all(lineNum+4, 3) = z_plane_para(0,0);
    M_all(lineNum+4, 4) = z_plane_para(1,0);
    M_all(lineNum+4, 5) = z_plane_para(2,0);
    M_all(lineNum+4, 6) = z_plane_para(3,0);

    // 构建可用的约束矩阵M和B
    Eigen::Matrix<double, 7, 6> M = Eigen::Matrix<double, 7, 6>::Zero();
    Eigen::Matrix<double, 7, 1> B = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 8, 6> signs;
    signs <<  1.0,  1.0,  1.0, 1.0, 1.0, 1.0,
             -1.0,  1.0,  1.0, 1.0, 1.0, 1.0,
             -1.0,  1.0, -1.0, 1.0, 1.0, 1.0,
              1.0,  1.0, -1.0, 1.0, 1.0, 1.0,
             -1.0, -1.0,  1.0, 1.0, 1.0, 1.0,
             -1.0, -1.0, -1.0, 1.0, 1.0, 1.0,
              1.0, -1.0, -1.0, 1.0, 1.0, 1.0,
              1.0, -1.0,  1.0, 1.0, 1.0, 1.0;
    
    Eigen::Matrix<double, 8, 9> pointsInfo;
    pointsInfo << -1, 0, 0, 0, 0,-1, 0,-1, 0,
                   0,-1, 0, 1, 0, 0, 0, 0,-1,
                   0, 0, 1, 1, 0, 0, 0,-1, 0,
                   0, 0, 1,-1, 0, 0, 0,-1, 0,
                   0, 0,-1, 0, 1, 0, 1, 0, 0,
                   0, 0, 1, 0, 1, 0, 1, 0, 0,
                   0, 0, 1,-1, 0, 0, 0, 1, 0,
                  -1, 0, 0, 0, 0,-1, 0, 1, 0;
    // 判断上、左、下、右四个平面分别与哪一个点相交
    for(int i=0; i<8; i++)
    {
        // 判断上平面与当前点
        Eigen::Matrix<double, 1, 1> _up_dot_0= pointsInfo.block(i,0,1,3)*planes_para.block(0,0,3,1);
        Eigen::Matrix<double, 1, 1> _up_dot_1= pointsInfo.block(i,3,1,3)*planes_para.block(0,0,3,1);
        Eigen::Matrix<double, 1, 1> _up_dot_2= pointsInfo.block(i,6,1,3)*planes_para.block(0,0,3,1);
        if(_up_dot_0(0,0) >= 0 && _up_dot_1(0,0) >= 0 && _up_dot_2(0,0) >= 0)
        {
            M.block(0, 0, 1, 6) = M_all.block(0, 0, 1, 6).array() * signs.block(i, 0, 1, 6).array();
            LineConstraint(3,0) = i;
        }

        // 判断左平面与当前点
        _up_dot_0= pointsInfo.block(i,0,1,3)*planes_para.block(0,1,3,1);
        _up_dot_1= pointsInfo.block(i,3,1,3)*planes_para.block(0,1,3,1);
        _up_dot_2= pointsInfo.block(i,6,1,3)*planes_para.block(0,1,3,1);
        if(_up_dot_0(0,0) >= 0 && _up_dot_1(0,0) >= 0 && _up_dot_2(0,0) >= 0)
        {
            M.block(1, 0, 1, 6) = M_all.block(1, 0, 1, 6).array() * signs.block(i, 0, 1, 6).array();
            LineConstraint(3,1) = i;
        }

        // 判断上平面与当前点
        _up_dot_0= pointsInfo.block(i,0,1,3)*planes_para.block(0,2,3,1);
        _up_dot_1= pointsInfo.block(i,3,1,3)*planes_para.block(0,2,3,1);
        _up_dot_2= pointsInfo.block(i,6,1,3)*planes_para.block(0,2,3,1);
        if(_up_dot_0(0,0) >= 0 && _up_dot_1(0,0) >= 0 && _up_dot_2(0,0) >= 0)
        {
            M.block(2, 0, 1, 6) = M_all.block(2, 0, 1, 6).array() * signs.block(i, 0, 1, 6).array();
            LineConstraint(3,2) = i;
        }

        // 判断上平面与当前点
        _up_dot_0= pointsInfo.block(i,0,1,3)*planes_para.block(0,3,3,1);
        _up_dot_1= pointsInfo.block(i,3,1,3)*planes_para.block(0,3,3,1);
        _up_dot_2= pointsInfo.block(i,6,1,3)*planes_para.block(0,3,3,1);
        if(_up_dot_0(0,0) >= 0 && _up_dot_1(0,0) >= 0 && _up_dot_2(0,0) >= 0)
        {
            M.block(3, 0, 1, 6) = M_all.block(3, 0, 1, 6).array() * signs.block(i, 0, 1, 6).array();
            LineConstraint(3,3) = i;
        }

    }
    for(int j=0; j<4; j++)
    {
        B(j,0) = -M_all(j, 6);
    }
    
    double minError = 10000000000000000.0;
    Eigen::Matrix<double, 6, 1> minRes = Eigen::Matrix<double, 6, 1>::Zero();
    // minRes = []
    // for(int i=0; i<lineNum; i++)
    // {
    //     if((int)possible_pointid[i].size()>0)
    //     {
    M.block(6,0,1,6) = M_all.block(lineNum+4, 0, 1, 6);
    B(6,0) = -M_all(lineNum+4,6);

    Eigen::MatrixXd M_all0 = M_all;
    Eigen::MatrixXd M_all1 = M_all;
    Eigen::MatrixXd M_all2 = M_all;
    Eigen::MatrixXd M_all3 = M_all;
    Eigen::MatrixXd M_all4 = M_all;
    Eigen::MatrixXd M_all5 = M_all;
    Eigen::MatrixXd M_all6 = M_all;
    Eigen::MatrixXd M_all7 = M_all;

    M_all0.block(0,0,lineNum+4,1) = M_all.block(0,0,lineNum+4,1)*signs(0,0);
    M_all0.block(0,1,lineNum+4,1) = M_all.block(0,1,lineNum+4,1)*signs(0,1);
    M_all0.block(0,2,lineNum+4,1) = M_all.block(0,2,lineNum+4,1)*signs(0,2);

    M_all1.block(0,0,lineNum+4,1) = M_all.block(0,0,lineNum+4,1)*signs(1,0);
    M_all1.block(0,1,lineNum+4,1) = M_all.block(0,1,lineNum+4,1)*signs(1,1);
    M_all1.block(0,2,lineNum+4,1) = M_all.block(0,2,lineNum+4,1)*signs(1,2);

    M_all2.block(0,0,lineNum+4,1) = M_all.block(0,0,lineNum+4,1)*signs(2,0);
    M_all2.block(0,1,lineNum+4,1) = M_all.block(0,1,lineNum+4,1)*signs(2,1);
    M_all2.block(0,2,lineNum+4,1) = M_all.block(0,2,lineNum+4,1)*signs(2,2);

    M_all3.block(0,0,lineNum+4,1) = M_all.block(0,0,lineNum+4,1)*signs(3,0);
    M_all3.block(0,1,lineNum+4,1) = M_all.block(0,1,lineNum+4,1)*signs(3,1);
    M_all3.block(0,2,lineNum+4,1) = M_all.block(0,2,lineNum+4,1)*signs(3,2);

    M_all4.block(0,0,lineNum+4,1) = M_all.block(0,0,lineNum+4,1)*signs(4,0);
    M_all4.block(0,1,lineNum+4,1) = M_all.block(0,1,lineNum+4,1)*signs(4,1);
    M_all4.block(0,2,lineNum+4,1) = M_all.block(0,2,lineNum+4,1)*signs(4,2);

    M_all5.block(0,0,lineNum+4,1) = M_all.block(0,0,lineNum+4,1)*signs(5,0);
    M_all5.block(0,1,lineNum+4,1) = M_all.block(0,1,lineNum+4,1)*signs(5,1);
    M_all5.block(0,2,lineNum+4,1) = M_all.block(0,2,lineNum+4,1)*signs(5,2);

    M_all6.block(0,0,lineNum+4,1) = M_all.block(0,0,lineNum+4,1)*signs(6,0);
    M_all6.block(0,1,lineNum+4,1) = M_all.block(0,1,lineNum+4,1)*signs(6,1);
    M_all6.block(0,2,lineNum+4,1) = M_all.block(0,2,lineNum+4,1)*signs(6,2);

    M_all7.block(0,0,lineNum+4,1) = M_all.block(0,0,lineNum+4,1)*signs(7,0);
    M_all7.block(0,1,lineNum+4,1) = M_all.block(0,1,lineNum+4,1)*signs(7,1);
    M_all7.block(0,2,lineNum+4,1) = M_all.block(0,2,lineNum+4,1)*signs(7,2);


    for(int id0=0; id0<8; id0++)
    {
        int i=0;
        M.block(4,0,1,6) = M_all.block(4+i,0,1,6).array()*signs.block(id0, 0, 1, 6).array();
        
        B(4,0) = -M_all(4+i,6);
        for(int id1=0; id1<8; id1++)
        {
            int j=1;
            // M.block(5,0,1,6) = M_all.block(4+j,0,1,6).array()*signs.block(possible_pointid[j][id1], 0, 1, 6).array();
            M.block(5,0,1,6) = M_all.block(4+j,0,1,6).array()*signs.block(id1, 0, 1, 6).array();
            B(5,0) = -M_all(4+j,6);
            Eigen::Matrix<double, 6, 1> X_temp = M.fullPivHouseholderQr().solve(B);

            

            bool hasNeg = false;
            for(int _j=0;_j<3;_j++)
            {
                if(X_temp(_j)<0)
                {
                    hasNeg = true;
                    break;
                }
            }
            // hasNeg = false;
            if(!hasNeg)
            {
                // 计算该结果与所有直线之间的误差
                // Eigen::Matrix<double, 7, 1> error = M*X_temp-B;
                Eigen::MatrixXd error_all = Eigen::MatrixXd::Zero(lineNum+5, 8);
                error_all.block(0,0,lineNum+5,1) = M_all0.block(0,0,lineNum+5,6)*X_temp+M_all0.block(0,6,lineNum+5,1);
                error_all.block(0,1,lineNum+5,1) = M_all1.block(0,0,lineNum+5,6)*X_temp+M_all1.block(0,6,lineNum+5,1);
                error_all.block(0,2,lineNum+5,1) = M_all2.block(0,0,lineNum+5,6)*X_temp+M_all2.block(0,6,lineNum+5,1);
                error_all.block(0,3,lineNum+5,1) = M_all3.block(0,0,lineNum+5,6)*X_temp+M_all3.block(0,6,lineNum+5,1);
                error_all.block(0,4,lineNum+5,1) = M_all4.block(0,0,lineNum+5,6)*X_temp+M_all4.block(0,6,lineNum+5,1);
                error_all.block(0,5,lineNum+5,1) = M_all5.block(0,0,lineNum+5,6)*X_temp+M_all5.block(0,6,lineNum+5,1);
                error_all.block(0,6,lineNum+5,1) = M_all6.block(0,0,lineNum+5,6)*X_temp+M_all6.block(0,6,lineNum+5,1);
                error_all.block(0,7,lineNum+5,1) = M_all7.block(0,0,lineNum+5,6)*X_temp+M_all7.block(0,6,lineNum+5,1);

                error_all = error_all.cwiseAbs();

                Eigen::MatrixXd error = Eigen::MatrixXd::Zero(lineNum+5, 1);
                for(int error_i=0; error_i<lineNum+5; error_i++)
                {
                    int __i, __j;
                    error(error_i,0) = error_all.block(error_i,0,1,8).minCoeff(&__i, &__j);
                }
                if(error.sum() < minError)
                {
                    minError = error.sum();
                    for(int _j=0; _j<6; _j++)
                    {
                        minRes(_j) = X_temp(_j);
                    }
                    LineConstraint(3,4) = id0;
                    LineConstraint(3,5) = id1;
                }
            }
        }

        
        
    }
    estQ(0,0) = minRes(0)*minRes(0);
    estQ(1,1) = minRes(1)*minRes(1);
    estQ(2,2) = minRes(2)*minRes(2);
    estQ(3,3) = -1.0;
    Eigen::Matrix4d _T;
    _T.setIdentity();
    _T(0,3) = minRes(3);
    _T(1,3) = minRes(4);
    _T(2,3) = minRes(5);

    Center(0) = minRes(3);
    Center(1) = minRes(4);
    Center(2) = minRes(5);
    Axes(0) = minRes(0);
    Axes(1) = minRes(1);
    Axes(2) = minRes(2);

    estQ = _T*estQ*_T.transpose();
    estQ = estQ / (-estQ(3,3));
}

void DetectObj3D::getDirectConstraint(Eigen::Matrix<double, 2, 5> &paraLine, Eigen::Matrix<double, 2, 10> &DirectionCons, Eigen::Vector3d &direction, Eigen::Matrix<double, 4,3> &Pst)
{
    Eigen::Matrix<double, 2, 3> line_para = Eigen::Matrix<double, 2, 3>::Zero();
    DetectObj3D::get_line_parameters(line_para, paraLine);
    // 得到平面
    Eigen::Matrix<double, 4, 2> plane_para = Pst*(line_para.transpose());
    // 叉乘得到方向
    direction(0) = plane_para(1,0)*plane_para(2,1)-plane_para(2,0)*plane_para(1,1);
    direction(1) = plane_para(2,0)*plane_para(0,1)-plane_para(0,0)*plane_para(2,1);
    direction(2) = plane_para(0,0)*plane_para(1,1)-plane_para(1,0)*plane_para(0,1);
    direction.normalize();
    Eigen::Matrix<double, 1, 10> DirectionCons1 = Eigen::Matrix<double, 1, 10>::Zero();
    Eigen::Matrix<double, 1, 10> DirectionCons2 = Eigen::Matrix<double, 1, 10>::Zero();

    DirectionCons1(0, 3) = plane_para(0, 0)*plane_para(1, 0)*plane_para(2,1) - plane_para(0, 0)*plane_para(2, 0)*plane_para(1,1);
    DirectionCons1(0, 0) = plane_para(0, 0)*plane_para(2, 0)*plane_para(0,1) - plane_para(0, 0)*plane_para(0, 0)*plane_para(2,1) + plane_para(1, 0)*plane_para(1, 0)*plane_para(2,1) - plane_para(1, 0)*plane_para(2, 0)*plane_para(1,1);
    DirectionCons1(0, 1) = plane_para(0, 0)*plane_para(0, 0)*plane_para(1,1) - plane_para(0, 0)*plane_para(1, 0)*plane_para(0,1) + plane_para(2, 0)*plane_para(1, 0)*plane_para(2,1) - plane_para(2, 0)*plane_para(2, 0)*plane_para(1,1);
    DirectionCons1(0, 4) = plane_para(1, 0)*plane_para(2, 0)*plane_para(0,1) - plane_para(1, 0)*plane_para(0, 0)*plane_para(2,1);
    DirectionCons1(0, 2) = plane_para(1, 0)*plane_para(0, 0)*plane_para(1,1) - plane_para(1, 0)*plane_para(1, 0)*plane_para(0,1) + plane_para(2, 0)*plane_para(2, 0)*plane_para(0,1) - plane_para(2, 0)*plane_para(0, 0)*plane_para(2,1);
    DirectionCons1(0, 5) = plane_para(2, 0)*plane_para(0, 0)*plane_para(1,1) - plane_para(2, 0)*plane_para(1, 0)*plane_para(0,1);

    DirectionCons2(1, 3) = plane_para(0, 1)*plane_para(1, 0)*plane_para(2,1) - plane_para(0, 1)*plane_para(2, 0)*plane_para(1,1);
    DirectionCons2(1, 0) = plane_para(0, 1)*plane_para(2, 0)*plane_para(0,1) - plane_para(0, 1)*plane_para(0, 0)*plane_para(2,1) + plane_para(1, 1)*plane_para(1, 0)*plane_para(2,1) - plane_para(1, 1)*plane_para(2, 0)*plane_para(1,1);
    DirectionCons2(1, 1) = plane_para(0, 1)*plane_para(0, 0)*plane_para(1,1) - plane_para(0, 1)*plane_para(1, 0)*plane_para(0,1) + plane_para(2, 1)*plane_para(1, 0)*plane_para(2,1) - plane_para(2, 1)*plane_para(2, 0)*plane_para(1,1);
    DirectionCons2(1, 4) = plane_para(1, 1)*plane_para(2, 0)*plane_para(0,1) - plane_para(1, 1)*plane_para(0, 0)*plane_para(2,1);
    DirectionCons2(1, 2) = plane_para(1, 1)*plane_para(0, 0)*plane_para(1,1) - plane_para(1, 1)*plane_para(1, 0)*plane_para(0,1) + plane_para(2, 1)*plane_para(2, 0)*plane_para(0,1) - plane_para(2, 1)*plane_para(0, 0)*plane_para(2,1);
    DirectionCons2(1, 5) = plane_para(2, 1)*plane_para(0, 0)*plane_para(1,1) - plane_para(2, 1)*plane_para(1, 0)*plane_para(0,1);

    DirectionCons1.normalize();
    DirectionCons2.normalize();
    DirectionCons.block(0,0,1,10) = DirectionCons1;
    DirectionCons.block(1,0,1,10) = DirectionCons2;
}

void DetectObj3D::get_line_parameters(Eigen::Matrix<double, 2, 3> &line_para, Eigen::Matrix<double, 2, 5> &paraLine)
{
    // Eigen::MatrixXd line_para1 = Eigen::MatrixXd::Zero(2,3);
    for(int i=0;i<2;i++)
    {
        line_para(i,0) = paraLine(i,2)-paraLine(i,3);
        line_para(i,1) = paraLine(i,1)-paraLine(i,0);
        line_para(i,2) = -paraLine(i,2)*line_para(i,1)-paraLine(i,0)*line_para(i,0);
    }
    // line_para=line_para1;
}

void DetectObj3D::dual_quadric_to_ellipsoid_parameters(Eigen::Vector3d &center, Eigen::Vector3d &axes, Eigen::Matrix3d &R, Eigen::Matrix4d &Q)
{
    Q = Q / (-Q(3,3));
    center(0) = -Q(0, 3);
    center(1) = -Q(1, 3);
    center(2) = -Q(2, 3);
    Eigen::Matrix4d T;
    T.setIdentity();
    T(0, 3) = -center(0);
    T(1, 3) = -center(1);
    T(2, 3) = -center(2);
    Eigen::Matrix4d Qcent = T*Q*T.transpose();

    Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver(Qcent.block(0,0,3,3));
    // Eigen::Vector3cd emD = eigen_solver.eigenvalues();
    // Eigen::MatrixXcd emV = eigen_solver.eigenvectors();
    // Eigen::VectorXcd emDSort;
    // Eigen::Matrix3cd emVSort;

    Eigen::Matrix3d emD = eigen_solver.pseudoEigenvalueMatrix();
    Eigen::Matrix3d emV = eigen_solver.pseudoEigenvectors();
    Eigen::Vector3d emDSort;
    Eigen::Matrix3d emVSort;

    Eigen::Vector3d evD;
    evD << emD(0, 0), emD(1, 1), emD(2, 2);
    // 根据特征值大小进行排序
    // 排序结果在原序列中的位置
    Eigen::Vector3i ind;
    DetectObj3D::sort_vec(evD, emDSort, ind);
    emVSort << emV.block(0, ind(0), 3, 1),
                emV.block(0, ind(1), 3, 1),
                emV.block(0, ind(2), 3, 1);
    
    axes << sqrt(abs(emDSort(0))),
                    sqrt(abs(emDSort(1))),
                    sqrt(abs(emDSort(2)));
    
    R = emVSort;
}

// 对向量进行排序，从大到小
void DetectObj3D::sort_vec(const Eigen::Vector3d &vec, Eigen::Vector3d &sorted_vec, Eigen::Vector3i &ind)
{
    // Inputs：
    // vec：待排序的向量
    // ====================
    // Outputs：
    // sorted_vec：排序结果
    // ind：排序结果中各个元素在原始向量的位置

    ind = Eigen::Vector3i::LinSpaced(vec.size(), 0, vec.size() - 1); //[0 1 2 3 ... N-1]
    auto rule = [vec](int i, int j) -> bool {
        return vec(i) > vec(j);
    }; //正则表达式，作为sort的谓词
    std::sort(ind.data(), ind.data() + ind.size(), rule);
    //data成员函数返回VectorXd的第一个元素的指针，类似于begin()
    sorted_vec.resize(vec.size());
    for (int i = 0; i < vec.size(); i++)
    {
        sorted_vec(i) = vec(ind(i));
    }
}

void DetectObj3D::get_plane_constraint_box(const Eigen::Matrix<double, 4, 1> &objBBS, const Eigen::Matrix<double, 4, 3> &Pst, const Eigen::Matrix<double, 4, 3> &Mst, Eigen::Matrix<double, 4, 10> &plane_cons, Eigen::Matrix<double, 4, 4> &plane_dire)
{
    Eigen::Matrix4d bb_lines;
    bb_lines << objBBS(0), objBBS(0), objBBS(0), objBBS(2),
                objBBS(2), objBBS(0), objBBS(2), objBBS(2),
                objBBS(1), objBBS(1), objBBS(3), objBBS(1),
                objBBS(1), objBBS(3), objBBS(3), objBBS(3);
    // 获取线段信息
    Eigen::Matrix<double, 4, 3> line_para;
    DetectObj3D::get_bbline_parameters(bb_lines, line_para);
    // 由线段参数获得平面参数
    Eigen::Matrix4d plane_para = Pst * line_para.transpose();
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            plane_dire(i,j) = plane_para(i,j);
        }
    }
    
    // 获取平面在相机坐标系下的近似方向向量
    Eigen::Matrix4d plane_dire_appro;// = Eigen::Matrix4d::Zero();
    plane_dire_appro << 0,  1,  0, -1,
                        1,  0, -1,  0,
                        0,  0,  0,  0,
                        0,  0,  0,  0;
    Eigen::Matrix<double, 4, 4> Mst1;
    Mst1.setIdentity();
    Mst1.block(0,0,4,3) = Mst;
    Eigen::Matrix4d plane_dire_obj_appro = Mst1 * plane_dire_appro;
    // 将plane_dire_obj_appro变换为单位方向向量
    // 将plane_dire变换为单位方向向量
    for(int i=0; i<4; i++)
    {
        plane_dire_obj_appro.block(0,i,4,1) = plane_dire_obj_appro.block(0,i,4,1).normalized();
        plane_dire(3,i) = 0;
        plane_dire.block(0,i,4,1) = plane_dire.block(0,i,4,1).normalized();
    }

    // 判断plane_dire是否为内侧向量，不是的话反向
    for(int i=0; i<4; i++)
    {
        Eigen::Matrix<double, 1, 1> _dot_res = plane_dire_obj_appro.block(0,i,4,1).transpose() * plane_dire.block(0,i,4,1);
        if(_dot_res(0,0) < 0)
        {
            plane_dire(0,i) = -plane_dire(0,i);
            plane_dire(1,i) = -plane_dire(1,i);
            plane_dire(2,i) = -plane_dire(2,i);
        }
    }
    // 构建约束向量
    for(int i=0; i<4; i++)
    {
        plane_cons(i, 0) = plane_para(0, i);
        plane_cons(i, 1) = plane_para(1, i);
        plane_cons(i, 2) = plane_para(2, i);
        plane_cons(i, 3) = plane_para(0, i);
        plane_cons(i, 4) = plane_para(1, i);
        plane_cons(i, 5) = plane_para(2, i);
        plane_cons(i, 6) = plane_para(3, i);

        plane_cons.block(i,0,1,10) = plane_cons.block(i,0,1,10).normalized();
    }
}

void DetectObj3D::get_bbline_parameters(Eigen::Matrix4d &bb_lines, Eigen::Matrix<double, 4, 3> &line_para)
{
    line_para = Eigen::Matrix<double, 4, 3>::Zero();
    for(int i=0; i<4; i++)
    {
        line_para(i,0) = bb_lines(2,i) - bb_lines(3,i);
        line_para(i,1) = bb_lines(1,i) - bb_lines(0,i);
        line_para(i,2) = -bb_lines(2,i)*line_para(i,1) - bb_lines(0,i)*line_para(i,0);
    }
}

void DetectObj3D::get_plane_constraint_detect(vector<Eigen::Matrix<double, 1, 5>, Eigen::aligned_allocator<Eigen::Matrix<double, 1, 5>>> &objLines, const Eigen::Matrix<double, 4, 3> &Pst, Eigen::MatrixXd &plane_cons_det)
{
    int lineNum = (int)objLines.size();
    for(int i=0; i<lineNum; i++)
    {
        Eigen::Matrix<double, 1, 3> line_para;
        DetectObj3D::get_bbline_parameters1(objLines[i], line_para);
        Eigen::Matrix<double, 4, 1> plane_para = Pst*line_para.transpose();
        plane_cons_det(i,0) = plane_para(0,0);
        plane_cons_det(i,1) = plane_para(1,0);
        plane_cons_det(i,2) = plane_para(2,0);
        plane_cons_det(i,3) = plane_para(0,0);
        plane_cons_det(i,4) = plane_para(1,0);
        plane_cons_det(i,5) = plane_para(2,0);
        plane_cons_det(i,6) = plane_para(3,0);
        plane_cons_det.block(i,0,1,10) = plane_cons_det.block(i,0,1,10).normalized();
    }
}

void DetectObj3D::get_bbline_parameters1(Eigen::Matrix<double, 1, 5> &bb_lines, Eigen::Matrix<double, 1, 3> &line_para)
{
    line_para = Eigen::Matrix<double, 1, 3>::Zero();
    line_para(0,0) = bb_lines(0,2) - bb_lines(0,3);
    line_para(0,1) = bb_lines(0,1) - bb_lines(0,0);
    line_para(0,2) = -bb_lines(0,2)*line_para(0,1) - bb_lines(0,0)*line_para(0,0);
}

void Obj3D::GetDirection(Eigen::Matrix3d &R, Eigen::Matrix3d &R_Last)
{
    // Eigen::Matrix3d R_Last_in = R_Last.inverse();
    
    vector<Eigen::Matrix<double, 2, 5>, Eigen::aligned_allocator<Eigen::Matrix<double, 2, 5>>> emvParaLines = pObj2D->emvParaLines;
    int ParaLinesNum = (int)emvParaLines.size();
    Eigen::Matrix4d ObjMst;
    ObjMst.setIdentity();
    Eigen::Matrix<double, 3, 4> Pst1 = K*ObjMst.block(0,0,3,4);
    Eigen::Matrix<double, 4, 3> Pst = Pst1.transpose();
    // 声明约束矩阵
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(ParaLinesNum*2, 10);
    Eigen::MatrixXd directions = Eigen::MatrixXd::Zero(ParaLinesNum+1, 3);
    // cout << "ParaLinesNum: " << ParaLinesNum << endl;
    // 求解每一对平行线构成的约束和方向
    for(int i=0; i<ParaLinesNum; i++)
    {
        Eigen::Matrix<double, 2, 10> DirectionCons = Eigen::Matrix<double, 2, 10>::Zero();
        Eigen::Vector3d direction;
        direction << 0.0,0.0,0.0;
        DetectObj3D::getDirectConstraint(emvParaLines[i], DirectionCons, direction, Pst);
        M.block(i*2,0,2,10) = DirectionCons;
        directions.block(i,0,1,3) = direction.transpose();
    }
    Eigen::Vector3d _z_dire;
    // _z_dire = R_Last.block(0,1,3,1);
    _z_dire << -0.0175217, 0.748902, 0.662449;
    // cout << "R_Last: " << R_Last << endl; 
    directions.block(ParaLinesNum,0,1,3) = _z_dire.transpose();
    ParaLinesNum += 1;

    // 寻找互相垂直的平行线对
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> ver_id_list;
    Eigen::MatrixXd dire_angle_mat = directions*(directions.transpose());
    dire_angle_mat = dire_angle_mat.cwiseAbs();

    for(int i=0; i<ParaLinesNum; i++)
    {
        double para_dist_i = min(emvParaLines[i](0,4), emvParaLines[i](1,4));

        for(int j=i+1; j<ParaLinesNum; j++)
        {
            double angle = dire_angle_mat(i,j);
            double para_dist_j = min(emvParaLines[j](0,4), emvParaLines[j](1,4));
            double para_dist = min(para_dist_i, para_dist_j);
            if(angle < 0.2)
            {
                Eigen::Vector3d ver_id_cur;
                ver_id_cur << double(i), double(j), para_dist;
                ver_id_list.push_back(ver_id_cur);
            }
        }
    }
    // 按照长度排序
    sort(ver_id_list.begin(), ver_id_list.end(), [](Eigen::Vector3d i, Eigen::Vector3d j){
        if (i[2] > j[2])
        {
            return true;
        }
        return false;
    });
    // 找出前n对
    // cout <<"ver_id_list size: " << (int)ver_id_list.size() << endl;
    int iterNum = min(40, (int)ver_id_list.size());
    double maxSim = 0.0;
    double minSim = 1000000000000000000000000.0;

    Eigen::Vector3d minRes = Eigen::Vector3d::Zero();
    double minError = 10000000000000000000000000000.0;


    
    vector<Eigen::Matrix<double, 1, 5>, Eigen::aligned_allocator<Eigen::Matrix<double, 1, 5>>> objLines = pObj2D->emvLines;
    int lineNum = (int)objLines.size();
    Eigen::MatrixXd line_direct = Eigen::MatrixXd::Zero(lineNum, 10);
    Eigen::Matrix3d possible_dire;
    possible_dire << 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1;

    Eigen::Matrix3d direInCameraLast = R_Last*possible_dire;

    int max_id1 = -1;
    int max_id2 = -1;
    for(int i=0; i<iterNum; i++)//iterNum
    {
        int para1ID = int(ver_id_list[i](0));
        int para2ID = int(ver_id_list[i](1));
        
        // 此次迭代的约束矩阵
        Eigen::Matrix<double, 4, 10> M1;
        Eigen::Matrix<double, 4, 1> B;
        M1 << M.block(para1ID*2, 0, 2, 10), M.block(para2ID*2, 0, 2, 10);
        B = -M1.block(0,3,4,1) - 160000*M1.block(0,4,4,1) - 8100000000*M1.block(0,5,4,1);
        Eigen::Vector3d X_temp = M1.block(0,0,4,3).fullPivHouseholderQr().solve(B);
        // Eigen::Matrix<double, 4, 1> error = M1.block(0,0,4,3)*X_temp-B;
        Eigen::MatrixXd error = M.block(0,0,ParaLinesNum*2,3)*X_temp+M.block(0,3,ParaLinesNum*2,1) + 160000*M1.block(0,4,ParaLinesNum*2,1) + 8100000000*M1.block(0,5,ParaLinesNum*2,1);
        error = error.cwiseAbs();
        double error_sum = error.sum();
        double rightNum = 0.0;
        
        Eigen::Matrix4d _Ellipsoid;
        _Ellipsoid <<            1, X_temp(0),  X_temp(1),  0,
                        X_temp(0),    160000,  X_temp(2),  0,
                        X_temp(1), X_temp(2), 8100000000,  0,
                                0,         0,          0, -1;
        Eigen::Vector3d _center;
        Eigen::Vector3d _axes;
        Eigen::Matrix3d _R;
        DetectObj3D::dual_quadric_to_ellipsoid_parameters(_center, _axes, _R, _Ellipsoid);

        

        // 利用上一帧旋转评价当前帧求解的旋转
        Eigen::Matrix3d direInCameraCur = _R*possible_dire;
        double cur_sim = 0;

        for(int cur_i=0; cur_i<3; cur_i++)
        {
            double min_dot = 1000000000000000.0;
            for(int last_i=0; last_i<3; last_i++)
            {
                Eigen::Vector3d direInCameraCur1 = direInCameraCur.block(0,cur_i,3,1);
                Eigen::Vector3d direInCameraLast1 = direInCameraLast.block(0,last_i,3,1);
                direInCameraLast1.normalize();
                direInCameraCur1.normalize();
                Eigen::MatrixXd _cross = direInCameraCur1.cross(direInCameraLast1);
                _cross = _cross.cwiseAbs();
                double _sinxita = _cross(0,0);
                if(_sinxita<min_dot)
                {
                    min_dot = _sinxita;
                }
            }
            cur_sim += min_dot;
        }
        if(cur_sim < minSim)
        {
            minSim = cur_sim;
            minRes(0) = X_temp(0);
            minRes(1) = X_temp(1);
            minRes(2) = X_temp(2);
            max_id1 = para1ID;
            max_id2 = para2ID;
        }
        
    }

    Eigen::Matrix4d Ellipsoid;
    Ellipsoid <<            1, minRes(0),  minRes(1),  0,
                    minRes(0),    160000,  minRes(2),  0,
                    minRes(1), minRes(2), 8100000000,  0,
                            0,         0,          0, -1;
    
    Eigen::Vector3d center;
    Eigen::Vector3d axes;
    // Eigen::Matrix3d R;
    DetectObj3D::dual_quadric_to_ellipsoid_parameters(center, axes, R, Ellipsoid);
    
    for(int i=0; i<ParaLinesNum; i++)
    {
        double angle = dire_angle_mat(ParaLinesNum-1,i);
        if(angle > 0.2)
        {
            continue;
        }
        Eigen::Vector3d directionx, directiony, directionz;
        directionx = directions.block(i,0,1,3).transpose();
        directiony = directions.block(ParaLinesNum-1,0,1,3).transpose();
        directionz = directionx.cross(directiony);
        
        Eigen::Matrix3d _R;
        _R.block(0,0,3,1) = directionx;
        _R.block(0,1,3,1) = directiony;
        _R.block(0,2,3,1) = directionz;

        Eigen::Matrix3d direInCameraCur = _R*possible_dire;
        double cur_sim = 0;

        for(int cur_i=0; cur_i<3; cur_i++)
        {
            double min_dot = 1000000000000000.0;
            for(int last_i=0; last_i<3; last_i++)
            {
                Eigen::Vector3d direInCameraCur1 = direInCameraCur.block(0,cur_i,3,1);
                Eigen::Vector3d direInCameraLast1 = direInCameraLast.block(0,last_i,3,1);
                direInCameraLast1.normalize();
                direInCameraCur1.normalize();
                Eigen::MatrixXd _cross = direInCameraCur1.cross(direInCameraLast1);
                _cross = _cross.cwiseAbs();
                double _sinxita = _cross(0,0);
                if(_sinxita<min_dot)
                {
                    min_dot = _sinxita;
                }
            }
            cur_sim += min_dot;
        }
        if(cur_sim < minSim)
        {
            minSim = cur_sim;
            R = _R;
        }
    }

    
    // 得到R之后，选择符合条件的平行线对
    std::vector<int> paraSelectedID;
    Eigen::Matrix3d estimate_direction = R_Last*Eigen::Matrix3d::Identity();
    for(int i=0; i<3; i++)
    {
        estimate_direction.block(0,i,3,1) = estimate_direction.block(0,i,3,1).normalized();
    }
    int directNum[3]={0,0,0};
    for(int i=0; i<ParaLinesNum; i++)
    {
        double max_dot = 0;
        int max_id = -1;
        for(int j=0; j<3; j++)
        {
            Eigen::Matrix<double, 1, 1> _dot = directions.block(i,0,1,3)*estimate_direction.block(0,j,3,1);
            _dot = _dot.cwiseAbs();
            if(_dot(0,0)>max_dot)
            {
                max_dot = _dot(0,0);
                max_id = j;
            }
        }
        if(max_dot > 0.95)
        {
            paraSelectedID.push_back(i);
            directNum[max_id]+=1;
        }

    }
    // 从其中找出中等数量
    int okNum = 0;
    for(int i=0; i<3; i++)
    {
        if(directNum[i]>2)
        {
            okNum++;
        }
    }
    if(okNum>=2)
    {
        
        
        int paraSelectedNum = paraSelectedID.size();
        // 构建约束矩阵
        Eigen::MatrixXd M2 = Eigen::MatrixXd::Zero(paraSelectedNum*2, 10);
        Eigen::MatrixXd B2 = Eigen::MatrixXd::Zero(paraSelectedNum*2, 1);
        for(int i=0; i<paraSelectedNum; i++)
        {
            M2.block(i*2, 0, 2, 10) = M.block(paraSelectedID[i]*2, 0, 2, 10);

        }
        B2 = -M2.block(0,3,paraSelectedNum*2,1) - 160000*M2.block(0,4,paraSelectedNum*2,1) - 8100000000*M2.block(0,5,paraSelectedNum*2,1);
        Eigen::Vector3d X_temp1 = M2.block(0,0,paraSelectedNum*2,3).fullPivHouseholderQr().solve(B2);

        Eigen::Matrix4d Ellipsoid1;
        Ellipsoid1 <<            1, X_temp1(0),  X_temp1(1),  0,
                        X_temp1(0),     160000,  X_temp1(2),  0,
                        X_temp1(1), X_temp1(2),  8100000000,  0,
                                0,          0,           0, -1;
        
    }
    else
    {
        R = R_Last;
    }
    cout << "minSim: " << minSim << endl;
    
}


void Obj3D::GetEllipsoidFromLast(Eigen::Matrix3d &R, Eigen::Matrix4d &estQ)
{
    // 获取线段对象
    vector<Eigen::Matrix<double, 1, 5>, Eigen::aligned_allocator<Eigen::Matrix<double, 1, 5>>> objLines = pObj2D->emvLines;
    // 获取边界框
    Eigen::Matrix<double, 4, 1> objBBS;
    objBBS << pObj2D->evecLeftUp(0), pObj2D->evecLeftUp(1), pObj2D->evecRightDown(0), pObj2D->evecRightDown(1);
    int lineNum = (int)objLines.size();

    // 在已知方向和线的情况下估计尺寸和平移
    Eigen::Matrix<double, 4, 6> planes_para = Eigen::Matrix<double, 4, 6>::Zero();
    // 约束矩阵
    Eigen::MatrixXd M_all = Eigen::MatrixXd::Zero(lineNum+5, 10);
    // 从相机坐标系到目标附体坐标系
    Eigen::Matrix<double, 4, 3> Mst;
    Mst.setIdentity();
    Mst.block(0,0,3,3) = R.transpose();
    // 从图像平面到目标附体坐标系
    Eigen::Matrix<double, 4, 3> Pst = Mst*K.transpose();

    // 获取四条边的约束
    Eigen::Matrix<double, 4, 10> plane_cons = Eigen::Matrix<double, 4, 10>::Zero();
    Eigen::Matrix<double, 4, 4> plane_dire = Eigen::Matrix<double, 4, 4>::Zero();
    DetectObj3D::get_plane_constraint_box(objBBS, Pst, Mst, plane_cons, plane_dire);



    // 图像平面上的线段参数
    Eigen::Matrix4d bb_lines;
    bb_lines << objBBS(0), objBBS(0), objBBS(0), objBBS(2),
                objBBS(2), objBBS(0), objBBS(2), objBBS(2),
                objBBS(1), objBBS(1), objBBS(3), objBBS(1),
                objBBS(1), objBBS(3), objBBS(3), objBBS(3);
    // 获取线段信息
    Eigen::Matrix<double, 4, 3> line_para;
    DetectObj3D::get_bbline_parameters(bb_lines, line_para);
    LineConstraint.block(0,0,3,4) = line_para.transpose();
    // 图像平面上LSD线段的参数
    for(int i=0; i<2; i++)
    {
        Eigen::Matrix<double, 1, 3> line_para_temp;
        DetectObj3D::get_bbline_parameters1(objLines[i], line_para_temp);
        LineConstraint.block(0,i+4,3,1) = line_para_temp.transpose();
    }





    M_all.block(0,0,4,10) = plane_cons;
    planes_para.block(0,0,4,4) = plane_dire;
    // 得到检测线段的约束
    Eigen::MatrixXd plane_cons_det = Eigen::MatrixXd::Zero(lineNum, 10);
    DetectObj3D::get_plane_constraint_detect(objLines, Pst, plane_cons_det);
    M_all.block(4,0,lineNum,10) = plane_cons_det;

    // 添加最后一个假设约束平面
    Eigen::Matrix<double, 4, 1> z_plane;
    z_plane << 0, 0, 1, -100;
    Eigen::Matrix4d Mst1;
    Mst1.setIdentity();
    Mst1.block(0,0,4,3) = Mst;
    Eigen::Matrix<double, 4, 1> z_plane_para = Mst1 * z_plane;
    M_all(lineNum+4, 0) = z_plane_para(0,0);
    M_all(lineNum+4, 1) = z_plane_para(1,0);
    M_all(lineNum+4, 2) = z_plane_para(2,0);
    M_all(lineNum+4, 3) = z_plane_para(0,0);
    M_all(lineNum+4, 4) = z_plane_para(1,0);
    M_all(lineNum+4, 5) = z_plane_para(2,0);
    M_all(lineNum+4, 6) = z_plane_para(3,0);

    // 构建可用的约束矩阵M和B
    Eigen::Matrix<double, 7, 6> M = Eigen::Matrix<double, 7, 6>::Zero();
    Eigen::Matrix<double, 7, 1> B = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 8, 6> signs;
    signs <<  1.0,  1.0,  1.0, 1.0, 1.0, 1.0,
             -1.0,  1.0,  1.0, 1.0, 1.0, 1.0,
             -1.0,  1.0, -1.0, 1.0, 1.0, 1.0,
              1.0,  1.0, -1.0, 1.0, 1.0, 1.0,
             -1.0, -1.0,  1.0, 1.0, 1.0, 1.0,
             -1.0, -1.0, -1.0, 1.0, 1.0, 1.0,
              1.0, -1.0, -1.0, 1.0, 1.0, 1.0,
              1.0, -1.0,  1.0, 1.0, 1.0, 1.0;
    
    Eigen::Matrix<double, 8, 9> pointsInfo;
    pointsInfo << -1, 0, 0, 0, 0,-1, 0,-1, 0,
                   0,-1, 0, 1, 0, 0, 0, 0,-1,
                   0, 0, 1, 1, 0, 0, 0,-1, 0,
                   0, 0, 1,-1, 0, 0, 0,-1, 0,
                   0, 0,-1, 0, 1, 0, 1, 0, 0,
                   0, 0, 1, 0, 1, 0, 1, 0, 0,
                   0, 0, 1,-1, 0, 0, 0, 1, 0,
                  -1, 0, 0, 0, 0,-1, 0, 1, 0;
    // 判断上、左、下、右四个平面分别与哪一个点相交
    for(int i=0; i<8; i++)
    {
        // 判断上平面与当前点
        Eigen::Matrix<double, 1, 1> _up_dot_0= pointsInfo.block(i,0,1,3)*planes_para.block(0,0,3,1);
        Eigen::Matrix<double, 1, 1> _up_dot_1= pointsInfo.block(i,3,1,3)*planes_para.block(0,0,3,1);
        Eigen::Matrix<double, 1, 1> _up_dot_2= pointsInfo.block(i,6,1,3)*planes_para.block(0,0,3,1);
        if(_up_dot_0(0,0) >= 0 && _up_dot_1(0,0) >= 0 && _up_dot_2(0,0) >= 0)
        {
            M.block(0, 0, 1, 6) = M_all.block(0, 0, 1, 6).array() * signs.block(i, 0, 1, 6).array();
            LineConstraint(3,0) = i;
        }

        // 判断左平面与当前点
        _up_dot_0= pointsInfo.block(i,0,1,3)*planes_para.block(0,1,3,1);
        _up_dot_1= pointsInfo.block(i,3,1,3)*planes_para.block(0,1,3,1);
        _up_dot_2= pointsInfo.block(i,6,1,3)*planes_para.block(0,1,3,1);
        if(_up_dot_0(0,0) >= 0 && _up_dot_1(0,0) >= 0 && _up_dot_2(0,0) >= 0)
        {
            M.block(1, 0, 1, 6) = M_all.block(1, 0, 1, 6).array() * signs.block(i, 0, 1, 6).array();
            LineConstraint(3,1) = i;
        }

        // 判断上平面与当前点
        _up_dot_0= pointsInfo.block(i,0,1,3)*planes_para.block(0,2,3,1);
        _up_dot_1= pointsInfo.block(i,3,1,3)*planes_para.block(0,2,3,1);
        _up_dot_2= pointsInfo.block(i,6,1,3)*planes_para.block(0,2,3,1);
        if(_up_dot_0(0,0) >= 0 && _up_dot_1(0,0) >= 0 && _up_dot_2(0,0) >= 0)
        {
            M.block(2, 0, 1, 6) = M_all.block(2, 0, 1, 6).array() * signs.block(i, 0, 1, 6).array();
            LineConstraint(3,2) = i;
        }

        // 判断上平面与当前点
        _up_dot_0= pointsInfo.block(i,0,1,3)*planes_para.block(0,3,3,1);
        _up_dot_1= pointsInfo.block(i,3,1,3)*planes_para.block(0,3,3,1);
        _up_dot_2= pointsInfo.block(i,6,1,3)*planes_para.block(0,3,3,1);
        if(_up_dot_0(0,0) >= 0 && _up_dot_1(0,0) >= 0 && _up_dot_2(0,0) >= 0)
        {
            M.block(3, 0, 1, 6) = M_all.block(3, 0, 1, 6).array() * signs.block(i, 0, 1, 6).array();
            LineConstraint(3,3) = i;
        }

    }

    for(int j=0; j<4; j++)
    {
        B(j,0) = -M_all(j, 6);
    }
    
    double minError = 10000000000000000.0;
    Eigen::Matrix<double, 6, 1> minRes = Eigen::Matrix<double, 6, 1>::Zero();
    // minRes = []
    // for(int i=0; i<lineNum; i++)
    // {
    //     if((int)possible_pointid[i].size()>0)
    //     {
    M.block(6,0,1,6) = M_all.block(lineNum+4, 0, 1, 6);
    B(6,0) = -M_all(lineNum+4,6);

    Eigen::MatrixXd M_all0 = M_all;
    Eigen::MatrixXd M_all1 = M_all;
    Eigen::MatrixXd M_all2 = M_all;
    Eigen::MatrixXd M_all3 = M_all;
    Eigen::MatrixXd M_all4 = M_all;
    Eigen::MatrixXd M_all5 = M_all;
    Eigen::MatrixXd M_all6 = M_all;
    Eigen::MatrixXd M_all7 = M_all;

    M_all0.block(0,0,lineNum+4,1) = M_all.block(0,0,lineNum+4,1)*signs(0,0);
    M_all0.block(0,1,lineNum+4,1) = M_all.block(0,1,lineNum+4,1)*signs(0,1);
    M_all0.block(0,2,lineNum+4,1) = M_all.block(0,2,lineNum+4,1)*signs(0,2);

    M_all1.block(0,0,lineNum+4,1) = M_all.block(0,0,lineNum+4,1)*signs(1,0);
    M_all1.block(0,1,lineNum+4,1) = M_all.block(0,1,lineNum+4,1)*signs(1,1);
    M_all1.block(0,2,lineNum+4,1) = M_all.block(0,2,lineNum+4,1)*signs(1,2);

    M_all2.block(0,0,lineNum+4,1) = M_all.block(0,0,lineNum+4,1)*signs(2,0);
    M_all2.block(0,1,lineNum+4,1) = M_all.block(0,1,lineNum+4,1)*signs(2,1);
    M_all2.block(0,2,lineNum+4,1) = M_all.block(0,2,lineNum+4,1)*signs(2,2);

    M_all3.block(0,0,lineNum+4,1) = M_all.block(0,0,lineNum+4,1)*signs(3,0);
    M_all3.block(0,1,lineNum+4,1) = M_all.block(0,1,lineNum+4,1)*signs(3,1);
    M_all3.block(0,2,lineNum+4,1) = M_all.block(0,2,lineNum+4,1)*signs(3,2);

    M_all4.block(0,0,lineNum+4,1) = M_all.block(0,0,lineNum+4,1)*signs(4,0);
    M_all4.block(0,1,lineNum+4,1) = M_all.block(0,1,lineNum+4,1)*signs(4,1);
    M_all4.block(0,2,lineNum+4,1) = M_all.block(0,2,lineNum+4,1)*signs(4,2);

    M_all5.block(0,0,lineNum+4,1) = M_all.block(0,0,lineNum+4,1)*signs(5,0);
    M_all5.block(0,1,lineNum+4,1) = M_all.block(0,1,lineNum+4,1)*signs(5,1);
    M_all5.block(0,2,lineNum+4,1) = M_all.block(0,2,lineNum+4,1)*signs(5,2);

    M_all6.block(0,0,lineNum+4,1) = M_all.block(0,0,lineNum+4,1)*signs(6,0);
    M_all6.block(0,1,lineNum+4,1) = M_all.block(0,1,lineNum+4,1)*signs(6,1);
    M_all6.block(0,2,lineNum+4,1) = M_all.block(0,2,lineNum+4,1)*signs(6,2);

    M_all7.block(0,0,lineNum+4,1) = M_all.block(0,0,lineNum+4,1)*signs(7,0);
    M_all7.block(0,1,lineNum+4,1) = M_all.block(0,1,lineNum+4,1)*signs(7,1);
    M_all7.block(0,2,lineNum+4,1) = M_all.block(0,2,lineNum+4,1)*signs(7,2);


    for(int id0=0; id0<8; id0++)
    {
        int i=0;
        M.block(4,0,1,6) = M_all.block(4+i,0,1,6).array()*signs.block(id0, 0, 1, 6).array();
        B(4,0) = -M_all(4+i,6);
        for(int id1=0; id1<8; id1++)
        {
            int j=1;
            M.block(5,0,1,6) = M_all.block(4+j,0,1,6).array()*signs.block(id1, 0, 1, 6).array();
            B(5,0) = -M_all(4+j,6);
            Eigen::Matrix<double, 6, 1> X_temp = M.fullPivHouseholderQr().solve(B);

            

            bool hasNeg = false;
            for(int _j=0;_j<3;_j++)
            {
                if(X_temp(_j)<0)
                {
                    hasNeg = true;
                    break;
                }
            }
            if(!hasNeg)
            {
                // 计算该结果与所有直线之间的误差
                Eigen::MatrixXd error_all = Eigen::MatrixXd::Zero(lineNum+5, 8);
                error_all.block(0,0,lineNum+5,1) = M_all0.block(0,0,lineNum+5,6)*X_temp+M_all0.block(0,6,lineNum+5,1);
                error_all.block(0,1,lineNum+5,1) = M_all1.block(0,0,lineNum+5,6)*X_temp+M_all1.block(0,6,lineNum+5,1);
                error_all.block(0,2,lineNum+5,1) = M_all2.block(0,0,lineNum+5,6)*X_temp+M_all2.block(0,6,lineNum+5,1);
                error_all.block(0,3,lineNum+5,1) = M_all3.block(0,0,lineNum+5,6)*X_temp+M_all3.block(0,6,lineNum+5,1);
                error_all.block(0,4,lineNum+5,1) = M_all4.block(0,0,lineNum+5,6)*X_temp+M_all4.block(0,6,lineNum+5,1);
                error_all.block(0,5,lineNum+5,1) = M_all5.block(0,0,lineNum+5,6)*X_temp+M_all5.block(0,6,lineNum+5,1);
                error_all.block(0,6,lineNum+5,1) = M_all6.block(0,0,lineNum+5,6)*X_temp+M_all6.block(0,6,lineNum+5,1);
                error_all.block(0,7,lineNum+5,1) = M_all7.block(0,0,lineNum+5,6)*X_temp+M_all7.block(0,6,lineNum+5,1);

                error_all = error_all.cwiseAbs();

                Eigen::MatrixXd error = Eigen::MatrixXd::Zero(lineNum+5, 1);
                for(int error_i=0; error_i<lineNum+5; error_i++)
                {
                    int __i, __j;
                    error(error_i,0) = error_all.block(error_i,0,1,8).minCoeff(&__i, &__j);
                }
                if(error.sum() < minError)
                {
                    minError = error.sum();
                    for(int _j=0; _j<6; _j++)
                    {
                        minRes(_j) = X_temp(_j);
                    }
                    LineConstraint(3,4) = id0;
                    LineConstraint(3,5) = id1;
                }
            }
        }

        
        
    }

    // 创建新的约束关系；
    Eigen::MatrixXd M3 = Eigen::MatrixXd::Zero(lineNum+5, 10);
    Eigen::MatrixXd B3 = Eigen::MatrixXd::Zero(lineNum+5, 1);
    Eigen::MatrixXd error_all = Eigen::MatrixXd::Zero(lineNum+5, 8);
    error_all.block(0,0,lineNum+5,1) = M_all0.block(0,0,lineNum+5,6)*minRes+M_all0.block(0,6,lineNum+5,1);
    error_all.block(0,1,lineNum+5,1) = M_all1.block(0,0,lineNum+5,6)*minRes+M_all1.block(0,6,lineNum+5,1);
    error_all.block(0,2,lineNum+5,1) = M_all2.block(0,0,lineNum+5,6)*minRes+M_all2.block(0,6,lineNum+5,1);
    error_all.block(0,3,lineNum+5,1) = M_all3.block(0,0,lineNum+5,6)*minRes+M_all3.block(0,6,lineNum+5,1);
    error_all.block(0,4,lineNum+5,1) = M_all4.block(0,0,lineNum+5,6)*minRes+M_all4.block(0,6,lineNum+5,1);
    error_all.block(0,5,lineNum+5,1) = M_all5.block(0,0,lineNum+5,6)*minRes+M_all5.block(0,6,lineNum+5,1);
    error_all.block(0,6,lineNum+5,1) = M_all6.block(0,0,lineNum+5,6)*minRes+M_all6.block(0,6,lineNum+5,1);
    error_all.block(0,7,lineNum+5,1) = M_all7.block(0,0,lineNum+5,6)*minRes+M_all7.block(0,6,lineNum+5,1);

    error_all = error_all.cwiseAbs();

    Eigen::MatrixXd error = Eigen::MatrixXd::Zero(lineNum+5, 1);
    for(int error_i=0; error_i<lineNum+4; error_i++)
    {
        int __i, __j;
        error(error_i,0) = error_all.block(error_i,0,1,8).minCoeff(&__i, &__j);
        if(error(error_i,0) > 1.5)
        {
            continue;
        }
        M3.block(error_i,0,1,10) = M_all.block(error_i, 0, 1, 10);
        M3(error_i,0) = M3(error_i,0)*signs(__j,0);
        M3(error_i,1) = M3(error_i,1)*signs(__j,1);
        M3(error_i,2) = M3(error_i,2)*signs(__j,2);
    }
    // cout << "error: " << error << endl;
    M3.block(lineNum+4,0,1,10) = M_all.block(lineNum+4, 0, 1, 10);
    B3 = -M3.block(0,6,lineNum+5,1);
    minRes = M3.block(0,0,lineNum+5,6).fullPivHouseholderQr().solve(B3);

    estQ(0,0) = minRes(0)*minRes(0);
    estQ(1,1) = minRes(1)*minRes(1);
    estQ(2,2) = minRes(2)*minRes(2);
    estQ(3,3) = -1.0;
    Eigen::Matrix4d _T;
    _T.setIdentity();
    _T(0,3) = minRes(3);
    _T(1,3) = minRes(4);
    _T(2,3) = minRes(5);

    Center(0) = minRes(3);
    Center(1) = minRes(4);
    Center(2) = minRes(5);
    Axes(0) = minRes(0);
    Axes(1) = minRes(1);
    Axes(2) = minRes(2);

    estQ = _T*estQ*_T.transpose();
    estQ = estQ / (-estQ(3,3));

}


}