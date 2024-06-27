#include "ITNode.h"
int nodeNum = 0;
namespace OBJECT
{
// generate the root node
Node::Node(Obj2D* _pObj2D)
{
    NodeRotTh = _pObj2D->NodeRotTh;
    NodeTranTh = _pObj2D->NodeTranTh;
    nodeNum = 0;
    fea1ID = -1;
    fea2ID = -1;
    kind = -1;
    pNode = NULL;
    cNodes.clear();
    levelNum = 0;
    validRotNum = 0;
    validTransNum = 0;
    rotCost = 0;
    transCost = 0;
    tranTh = 1.0/15.0;
    pObj3D = NULL;
    pObj2D = _pObj2D;
    relationMat = Eigen::MatrixXi::Zero(pObj2D->lineNum, pObj2D->lineNum);
    pSRG = new SRG();
    pRoot = this;
    isOK = false;
    constructObMat();

    for(int i=0; i<relationMat.rows(); i++)
    {
        for(int j=i+1; j<relationMat.cols(); j++)
        {
            relationMat(i,j) = 2;
            relationMat(j,i) = 2;
        }
    }
}

// generate the child node
Node::Node(Node* _pNode, int _imgFeaID, int _cubeFeaID, int _kind)
{
    NodeRotTh = _pNode->NodeRotTh;
    NodeTranTh = _pNode->NodeTranTh;
    fea1ID = _imgFeaID;
    fea2ID = _cubeFeaID;
    kind = _kind;
    pNode = _pNode;
    tranTh = 0.005;
    pObj3D = NULL;

    validRotNum = pNode->validRotNum;
    validTransNum = pNode->validTransNum;
    rotCost = pNode->rotCost;
    transCost = pNode->transCost;

    pObj2D = pNode->pObj2D;
    pRoot = pNode->pRoot;
    cNodes.clear();

    levelNum = pNode->levelNum+1;

    nodeNum++;
    

    // in the matching IT
    if(levelNum>3)
    {
        // a none node
        if(fea2ID == -1)
        {
            validRotNum = pNode->validRotNum;
            validTransNum = pNode->validTransNum;
            rotCost = pNode->rotCost;
            transCost = pNode->transCost;
        }
        // a valid node
        else
        {
            // if the level is a rotation level
            if(levelNum%2==0)
            {
                validRotNum += pObj2D->lines[fea1ID]->length;
            }
            // if the level is a trans level
            else
            {
                validTransNum += pObj2D->lines[fea1ID]->length;
            }
            rotCost = pNode->rotCost;
            transCost = pNode->transCost;
        }
    }
    
    relationMat = pNode->relationMat;
    pSRG = new SRG(pNode->pSRG);
    pObj2D = pNode->pObj2D;
    isOK = false;

    // useFrame or useGround
    if(fea1ID==-1)
    {
        isOK = true;
        return;
    }

    // judge this node with observation matrix
    if(!judgeObMat())
    {

        // if not satisfy the observation consistency
        // remove this node and return
        return;
    }

    // test the relation consistency
    if(!judgeRelationMat())
    {
        // if not satisfy the relation consistency
        // remove the node and return
        return;
    }
    // if satisfy the relation consistency
    // update the relation matrix
    // updateRelationMat();

    // if the level number < 3
    if(levelNum < 3)
    {
        // there is no 3D object and cant to estimate
        // go on
        updateRelationMat();

        isOK = true;
        return;
    }
    else if(levelNum == 3)
    {

        pObj3D = new Obj3D();

        // if the level num = 3, a rotation matrix should be estimated.
        estimateRotation();

        // update the relation matrix
        updateRelationMat();
        updateSRG();
        isOK = true;
        return;
    }
    // the level number > 3, in the matching IT
    else
    {
        // have rotation, so have valid rotation
        pObj3D = new Obj3D(pNode->pObj3D);

        if(fea2ID == -1)
        {
            updateSRG();
            isOK = true;
            return;
        }
        // judge the rotation level or the trans level
        // if this level is the rotation level
        if(levelNum%2 == 0)
        {
            // judge the cuboid model
            if(!judgeCuboidModel())
            {
                return;
            }
            else
            {
                updateRelationMat();
                updateCuboid();
                updateSRG();
                isOK = true;
                return;
            }
        }
        // this level is the tran level
        else
        {
            // judge if has trans now
            // if have trans now
            if(pObj3D->hasTrans)
            {
                // judge the cuboid model
                if(!judgeCuboidModel())
                {
                    return;
                }
                else
                {
                    updateRelationMat();
                    updateCuboid();
                    updateSRG();

                    isOK = true;
                    return;
                }
            }
            // if dont have trans now
            else
            {
                /// if not have trans
                // if the img feature match with one cuboid feature
                if(fea2ID != -1)
                {
                    // dont need to test this, have been judged before
                    if(fea2ID<3)
                    {
                        // cuboid feature is direction
                        // update the relation mat
                        updateRelationMat();
                        updateSRG();
                        isOK = true;
                    }
                    else
                    {
                        // cuboid feature is line or plane
                        // try to estiamte the trans
                        if(fea1ID > 10)
                        {
                            isOK = false;
                        }
                        // else if(estimateTrans())
                        if(estimateTrans())
                        {
                            updateSRG();
                            isOK = true;
                        }
                    }
                    
                }
                else
                {
                    updateSRG();
                    isOK = true;
                }
                return;
            }
            
        }
        
    }
    
}

Node::~Node()
{
    for(int i=0; i<(int)cNodes.size(); i++)
    {
        if(cNodes[i])
        {
            delete cNodes[i];
            cNodes[i] = NULL;
        }
    }

    if(pObj3D)
    {
        delete pObj3D;
        pObj3D = NULL;
    }
    // if(pObj2D)
    // {
    //     delete pObj2D;
    //     pObj2D = NULL;
    // }

    if(pSRG)
    {

        delete pSRG;

        pSRG = NULL;
    }
}

void Node::constructObMat()
{
    // img feature number
    int feaNum = (int)pObj2D->lines.size();

    // initial the observation matrix
    observationMat = Eigen::MatrixXi::Ones(feaNum, feaNum)*2;
    // observationMat = Eigen::MatrixXi::Zero(feaNum, feaNum);
    // get the angle between any two line segments
    vector<double> angles;
    for(int i=0; i<feaNum; i++)
    {
        angles.push_back(pObj2D->lines[i]->angle);
    }

    for(int i=0; i<feaNum-1; i++)
    {
        for(int j=i+1; j<feaNum; j++)
        {
            double angle = getAngle(pObj2D->lines[i]->angle, pObj2D->lines[j]->angle);
            // if the angle is small and the distance is large, the two line segments are maybe parallel
            if(angle < M_PI/4)
            {
                // the distance need to be large
                double lineDist = abs(pObj2D->lines[i]->parameter(2)-pObj2D->lines[j]->parameter(2));
                
                if(lineDist > max(min(pObj2D->width, pObj2D->height)/10, 5.0))
                {
                    // the two line segments maybe parallel
                    observationMat(i,j)=0;
                    observationMat(j,i)=0;
                }
                else
                {
                    observationMat(i,j)=-1;
                    observationMat(j,i)=-1;
                }
                
                // 
            }
            else
            {
                observationMat(i,j) = 1;
            }
            
            if(angle > M_PI/6)
            {
                if(observationMat(i,j)==-1)
                {
                    observationMat(i,j) = 1;
                }
                else if(observationMat(i,j) == 0)
                {
                    observationMat(i,j) = 2;
                }
            }
        }
    }

}

// judge observation consistency
bool Node::judgeObMat()
{
    // judge current relation
    if(levelNum<=3)
    {
        // it is the relation node
        // test the relation
        if(pRoot->observationMat(fea1ID, fea2ID) == kind||pRoot->observationMat(fea1ID, fea2ID)==2)
        {
            return true;
        }
        else
        {
            return false;
        }
        
    }
    else
    {
        // current node is matching
        if(fea2ID>=0 && fea2ID<=2)
        {
            for(int i=0; i<3; i++)
            {
                for(int j=0; j<pSRG->matchingEdge[i].size(); j++)
                {
                    int feaIDTemp = pSRG->matchingEdge[i][j];
                    if(i==fea2ID)
                    {
                        // need to be parallel
                        if(pRoot->observationMat(fea1ID, feaIDTemp) != 0 && pRoot->observationMat(fea1ID, feaIDTemp)!=2)
                        {
                            return false;
                        }
                    }
                    else
                    {
                        // need to be prependicular
                        // need to be parallel
                        if(pRoot->observationMat(fea1ID, feaIDTemp) != 1 && pRoot->observationMat(fea1ID, feaIDTemp)!=2)
                        {
                            return false;
                        }
                    }
                    
                }
            }
            return true;
        }
        else
        {
            return true;
        }
        
    }
}

bool Node::judgeRelationMat()
{
    return true;
    // to avoid the relation conflict
    // judge current relation
    if(levelNum<=3)
    {
        // it is the relation node
        // test the relation
        if(relationMat(fea1ID,fea2ID)==kind || relationMat(fea1ID,fea2ID)==2)
        {
            return true;
        }
        else
        {
            return false;
        }
        
    }
    else
    {
        // current node is matching
        if(fea2ID>=0 && fea2ID<=2)
        {
            for(int i=0; i<3; i++)
            {
                for(int j=0; j<pSRG->matchingEdge[i].size(); j++)
                {
                    int feaIDTemp = pSRG->matchingEdge[i][j];
                    if(i==fea2ID)
                    {
                        // need to be parallel
                        if(relationMat(fea1ID, feaIDTemp) != 0 || relationMat(fea1ID, feaIDTemp) != 2)
                        {
                            return false;
                        }
                    }
                    else
                    {
                        // need to be prependicular
                        // need to be parallel
                        if(relationMat(fea1ID, feaIDTemp) != 1 || relationMat(fea1ID, feaIDTemp) != 2)
                        {
                            return false;
                        }
                    }
                    
                }
            }
            return true;
        }
        else
        {
            return true;
        }
        
    }
}

bool Node::judgeCuboidModel()
{
    // the direction should be on the plane backprojected from the line
    if(fea2ID>=0 && fea2ID<3)
    {
        // compute the direction cost
        Eigen::Vector3d plane = pObj2D->lines[fea1ID]->plane.block(0,0,3,1);
        Eigen::Vector3d direction = pObj3D->R.block(0,fea2ID,3,1);
        Eigen::MatrixXd costTempMat = plane.transpose()*direction;
        double costTemp = abs(costTempMat(0,0));
        if(costTemp<0.05)
        {
            rotCost += costTemp;
            return true;
        }
        else
        {
            return false;
        }
    }
    else if(fea2ID>2&&fea2ID<15)
    {
        // compute the edge match cost
        Eigen::Vector4d plane = pObj2D->lines[fea1ID]->plane;
        // the point should in the plane
        Eigen::Vector4d Point1 = pObj3D->eightPointsInCamera.block(0,pObj3D->linePointsID(0,fea2ID-3),4,1);
        Eigen::Vector4d Point2 = pObj3D->eightPointsInCamera.block(0,pObj3D->linePointsID(1,fea2ID-3),4,1);
        // the two point should on the plane
        Eigen::MatrixXd dist1 = plane.transpose()*Point1;
        Eigen::MatrixXd dist2 = plane.transpose()*Point2;
        double dist = (abs(dist1(0,0))+abs(dist2(0,0)))/2;
        double distTh = tranTh; //pObj3D->Axes.sum()/10;
        // double distTh = 0.01;

        if(dist < distTh)
        {
            transCost += dist;
            return true;
        }
        else
        {

            return false;
        }
    }
    
}


void Node::estimateRotation()
{
    // the two parallel lines in the first level
    Eigen::Vector3d directionx;
    
    Line_seg* line1 = pObj2D->lines[pNode->pNode->fea1ID];

    Line_seg* line2 = pObj2D->lines[pNode->pNode->fea2ID];

    Line_seg* line3 = pObj2D->lines[pNode->fea2ID];

    directionx(0) = line1->plane(1)*line2->plane(2)-line1->plane(2)*line2->plane(1);
    directionx(1) = line1->plane(2)*line2->plane(0)-line1->plane(0)*line2->plane(2);
    directionx(2) = line1->plane(0)*line2->plane(1)-line1->plane(1)*line2->plane(0);
    directionx.normalize();
    // to get second direction y
    Eigen::Vector3d directiony;
    directiony(0) = directionx(1)*line3->plane(2)-directionx(2)*line3->plane(1);
    directiony(1) = directionx(2)*line3->plane(0)-directionx(0)*line3->plane(2);
    directiony(2) = directionx(0)*line3->plane(1)-directionx(1)*line3->plane(0);
    directiony.normalize();
    pObj3D->R.block(0,0,3,1) = directionx;
    pObj3D->R.block(0,1,3,1) = directiony;
    pObj3D->R.block(0,2,3,1) = directionx.cross(directiony);
    pObj3D->hasRot = true;
}


bool Node::estimateTrans()
{
    
    // with the rotation to estimate the translation and scale
    Eigen::Matrix<double, 4, 3> Mst;
    Mst.setIdentity();
    Mst.block(0,0,3,3) = pObj3D->R.transpose();
    // 从图像平面到目标附体坐标系
    Eigen::Matrix<double, 4, 3> Pst = Mst*pObj2D->K.transpose();

    // for(int lsdi=0; lsdi<pObj2D->lineNum; lsdi++)
    // {
        // pObj2D->lines[fea1ID]->getPlaneParametersInCuboid(Pst);
        pObj2D->lines[fea1ID]->getPlaneParametersInCuboid1(pObj3D->R.transpose());

    // }

    // 得到R之后计算检测框的每一个条线对应的目标顶点
    for(int bbi=0; bbi<4; bbi++)
    {
        // pObj2D->BBlines[bbi]->getPlaneParametersInCuboid(Pst);
        pObj2D->BBlines[bbi]->getPlaneParametersInCuboid1(pObj3D->R.transpose());
        pObj2D->BBlines[bbi]->getPlaneObjPointID(Mst, bbi);
    }

    // 构建约束矩阵
    Eigen::Matrix<double, 7, 7> M;
    Eigen::Matrix<double, 7, 1> B;
    M.setZero();
    B.setZero();

    // 检测框四条线的约束
    for(int bbi=0; bbi<4; bbi++)
    {
        M(bbi,0) = pObj2D->BBlines[bbi]->planeInCuboid(0);
        M(bbi,1) = pObj2D->BBlines[bbi]->planeInCuboid(1);
        M(bbi,2) = pObj2D->BBlines[bbi]->planeInCuboid(2);
        M(bbi,3) = pObj2D->BBlines[bbi]->plane(0);
        M(bbi,4) = pObj2D->BBlines[bbi]->plane(1);
        M(bbi,5) = pObj2D->BBlines[bbi]->plane(2);
        M(bbi,6) = pObj2D->BBlines[bbi]->planeInCuboid(3);
    }

    // 两个检测线构成四个约束
    for(int lsdi=0; lsdi<2; lsdi++)
    {
        M(4+lsdi,0) = pObj2D->lines[fea1ID]->planeInCuboid(0);
        M(4+lsdi,1) = pObj2D->lines[fea1ID]->planeInCuboid(1);
        M(4+lsdi,2) = pObj2D->lines[fea1ID]->planeInCuboid(2);
        M(4+lsdi,3) = pObj2D->lines[fea1ID]->plane(0);
        M(4+lsdi,4) = pObj2D->lines[fea1ID]->plane(1);
        M(4+lsdi,5) = pObj2D->lines[fea1ID]->plane(2);
        M(4+lsdi,6) = pObj2D->lines[fea1ID]->planeInCuboid(3);
    }

    // 最后一个平面决定尺度
    Eigen::Matrix<double, 4, 1> z_plane;
    z_plane << 0, 0, 1, -100;
    Eigen::Matrix4d Mst1;
    Mst1.setIdentity();
    Mst1.block(0,0,4,3) = Mst;
    Eigen::Matrix<double, 4, 1> z_plane_para = Mst1 * z_plane;
    M(6, 0) = z_plane_para(0,0);
    M(6, 1) = z_plane_para(1,0);
    M(6, 2) = z_plane_para(2,0);
    M(6, 3) = z_plane_para(0,0);
    M(6, 4) = z_plane_para(1,0);
    M(6, 5) = z_plane_para(2,0);
    M(6, 6) = z_plane_para(3,0);

    // 根据匹配的点的id，对M进行变号
    Eigen::Matrix<double, 8, 6> signs;
    signs <<    -1.0, -1.0, +1.0, 1.0, 1.0, 1.0,
                -1.0, +1.0, +1.0, 1.0, 1.0, 1.0,
                +1.0, +1.0, +1.0, 1.0, 1.0, 1.0,
                +1.0, -1.0, +1.0, 1.0, 1.0, 1.0,
                -1.0, -1.0, -1.0, 1.0, 1.0, 1.0,
                -1.0, +1.0, -1.0, 1.0, 1.0, 1.0,
                +1.0, +1.0, -1.0, 1.0, 1.0, 1.0,
                +1.0, -1.0, -1.0, 1.0, 1.0, 1.0;



    // 先是检测框的四条线段
    for(int bbi=0; bbi<4; bbi++)
    {
        // M.block(bbi,0,1,6) = M.block(bbi,0,1,6).array() * signs.block(pObj2D->BBlines[bbi]->pointid,0,1,6).array();
        for(int bbj=0; bbj<6; bbj++)
        {
            M(bbi,bbj) = M(bbi,bbj)*signs(pObj2D->BBlines[bbi]->pointid, bbj);


        }
    }

    // 然后是检测线的四个线段
    for(int lsdi=0; lsdi<2; lsdi++)
    {
        M.block(4+lsdi,0,1,6) = M.block(4+lsdi,0,1,6).array() * signs.block(pObj3D->linePointsID(lsdi, fea2ID-3),0,1,6).array();
    }
    for(int Bi=0; Bi<7; Bi++)
    {
        B(Bi) = -M(Bi,6)-M(Bi,5)*1;
    }

    // 计算3D目标
    Eigen::Matrix<double, 5, 1> X_temp = M.block(0,0,6,5).fullPivHouseholderQr().solve(B.block(0,0,6,1));

    // with scale
    if(false)
    {
        Eigen::MatrixXd M_temp = Eigen::MatrixXd::Zero(5, 10);
        Eigen::MatrixXd B_temp = Eigen::MatrixXd::Zero(5, 1);

        for(int mi=0; mi<4; mi++)
        {
            M_temp.block(mi,0,1,6) = M.block(mi,0,1,6);
            B_temp(mi) = B(mi);
        }
        M_temp.block(4,0,1,6) = M.block(6,0,1,6);
        B_temp(4) = B(6);

        Eigen::Vector3d scale_piror;
        scale_piror << 1, 1, 1;

        for(int mi=0; mi<5; mi++)
        {
            for(int mj=0; mj<3; mj++)
            {
                B_temp(mi) = B_temp(mi)-M_temp(mi, mj)*scale_piror(mj);
            }
        }

        Eigen::Matrix<double, 3, 1> X_temp_temp = M_temp.block(0,3,4,3).fullPivHouseholderQr().solve(B_temp.block(0,0,4,1));
        X_temp(0) = scale_piror(0);
        X_temp(1) = scale_piror(1);
        X_temp(2) = scale_piror(2);
        X_temp(3) = X_temp_temp(0);
        X_temp(4) = X_temp_temp(1);
        X_temp(5) = X_temp_temp(2);
    }

    Eigen::Vector3d CenterTemp;
    Eigen::Vector3d AxesTemp;
    AxesTemp(0) = abs(X_temp(0));
    AxesTemp(1) = abs(X_temp(1));
    AxesTemp(2) = abs(X_temp(2));

    CenterTemp(0) = X_temp(3);
    CenterTemp(1) = X_temp(4);
    CenterTemp(2) = 1;

    double centerScale = CenterTemp(2);
    for(int i=0; i<3; i++)
    {
        CenterTemp(i) = CenterTemp(i)/centerScale;
        AxesTemp(i) = AxesTemp(i)/centerScale;
    }
    
    // 计算当前目标的得分
    Eigen::Matrix<double, 8, 3> _cubePoints;
    _cubePoints <<  -AxesTemp(0), -AxesTemp(1),  AxesTemp(2),
                    -AxesTemp(0),  AxesTemp(1),  AxesTemp(2),
                     AxesTemp(0),  AxesTemp(1),  AxesTemp(2),
                     AxesTemp(0), -AxesTemp(1),  AxesTemp(2),
                    -AxesTemp(0), -AxesTemp(1), -AxesTemp(2),
                    -AxesTemp(0),  AxesTemp(1), -AxesTemp(2),
                     AxesTemp(0),  AxesTemp(1), -AxesTemp(2),
                     AxesTemp(0), -AxesTemp(1), -AxesTemp(2);
    
    // 旋转八个点，得到坐标系下的八个顶点
    Eigen::Matrix<double, 3, 8> _cubePoints1 = pObj3D->R*_cubePoints.transpose();
    // 寻找最远点id
    int _fastPID = 0;
    double fastPDist = _cubePoints1(2,0)+CenterTemp(2);
    for(int ci=0; ci<8; ci++)
    {
        _cubePoints1(0,ci) = _cubePoints1(0,ci)+CenterTemp(0);
        _cubePoints1(1,ci) = _cubePoints1(1,ci)+CenterTemp(1);
        _cubePoints1(2,ci) = _cubePoints1(2,ci)+CenterTemp(2);
        if(_cubePoints1(2,ci) > fastPDist)
        {
            fastPDist = _cubePoints1(2,ci);
            _fastPID = ci;
        }
    }

    // 投影到图像平面上
    Eigen::MatrixXd _cubePoints2(8, 4);
    for(int ci=0; ci<8; ci++)
    {
        _cubePoints2(ci,3) = 1;
    }
    _cubePoints2.block(0,0,8,3) = _cubePoints1.transpose();
    Eigen::Matrix<double, 4, 8> cameraCubePoints = Eigen::Matrix4d::Identity()*_cubePoints2.transpose();
    Eigen::Matrix<double, 3, 8> frameCubePoints1 = pObj2D->K*cameraCubePoints.block(0,0,3,8);

    Eigen::Matrix<double, 2, 8> frameCubePoints2d;
    // 对八个点进行归一化
    bool isNAN = false;
    for(int ci=0; ci<8; ci++)
    {
        frameCubePoints2d(0,ci) = frameCubePoints1(0,ci) / frameCubePoints1(2,ci);
        frameCubePoints2d(1,ci) = frameCubePoints1(1,ci) / frameCubePoints1(2,ci);
        if(isnan(frameCubePoints2d(0,ci))||isnan(frameCubePoints2d(1,ci)))
        {
            isNAN = true;
        }
    }

    if(isNAN)
    {
        return false;
    }

    // 计算投影得到的框
    Eigen::Vector4d ProBB;
    ProBB.setZero();
    ProBB(0) = frameCubePoints2d(0,0);
    ProBB(1) = frameCubePoints2d(1,0);
    ProBB(2) = frameCubePoints2d(0,0);
    ProBB(3) = frameCubePoints2d(1,0);

    for(int fcp_id=0; fcp_id<8; fcp_id++)
    {
        if(frameCubePoints2d(0,fcp_id)<ProBB(0))
        {
            ProBB(0) = frameCubePoints2d(0,fcp_id);
        }
        if(frameCubePoints2d(0,fcp_id)>ProBB(2))
        {
            ProBB(2) = frameCubePoints2d(0,fcp_id);
        }
        if(frameCubePoints2d(1,fcp_id)<ProBB(1))
        {
            ProBB(1) = frameCubePoints2d(1,fcp_id);
        }
        if(frameCubePoints2d(1,fcp_id)>ProBB(3))
        {
            ProBB(3) = frameCubePoints2d(1,fcp_id);
        }
    }

    // 计算投影框与原始框的交并比
    double xmin = max(double(pObj2D->BoundingBox(0)), ProBB(0));
    double ymin = max(double(pObj2D->BoundingBox(1)), ProBB(1));
    // 求交集的右下角的点
    double xmax = min(double(pObj2D->BoundingBox(2)), ProBB(2));
    double ymax = min(double(pObj2D->BoundingBox(3)), ProBB(3));
    // 计算当前帧目标的面积
    double S1 = (pObj2D->BoundingBox(2)-pObj2D->BoundingBox(0))*(pObj2D->BoundingBox(3)-pObj2D->BoundingBox(1));
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

    pObj3D->IOU = IOU;

    if (IOU<0.7)
    {
        return false;
    }
    
    // judge the valid lines
    Eigen::Vector4d plane = pObj2D->lines[fea1ID]->plane;
    // the point should in the plane
    Eigen::Vector4d Point1;
    Point1.setZero();
    Point1.block(0,0,3,1) = _cubePoints1.block(0,pObj3D->linePointsID(0,fea2ID-3),3,1);
    Eigen::Vector4d Point2;
    Point2.setZero();
    Point2.block(0,0,3,1) = _cubePoints1.block(0,pObj3D->linePointsID(1,fea2ID-3),3,1);
    // the two point should on the plane
    Eigen::MatrixXd dist1 = plane.transpose()*Point1;
    Eigen::MatrixXd dist2 = plane.transpose()*Point2;
    double dist = (abs(dist1(0,0))+abs(dist2(0,0)))/2;
    double distTh = tranTh; //pObj3D->Axes.sum()/10;

    for(int i=0; i<4; i++)
    {
        Eigen::Vector4d PointCur;
        PointCur.setZero();
        PointCur = _cubePoints1.block(0, pObj2D->BBlines[i]->pointid, 3, 1);

    }

    if(dist < distTh)
    {
        transCost += dist;

    }
    else
    {
        return false;
    }
    double maxAxes = max(max(abs(AxesTemp(0)), abs(AxesTemp(1))), abs(AxesTemp(2)));
    double minAxes = min(min(abs(AxesTemp(0)), abs(AxesTemp(1))), abs(AxesTemp(2)));

    if(maxAxes/minAxes>4)
    {
        validTransNum -= 200000;
    }

    pObj3D->hasTrans=true;
    pObj3D->Center = CenterTemp;
    pObj3D->Axes = AxesTemp;
    pObj3D->eightPointsInCamera.setZero();
    pObj3D->eightPointsInCamera.block(0,0,3,8) = _cubePoints1;
    pObj3D->eightPointsInCuboid.setZero();
    pObj3D->eightPointsInCuboid.block(0,0,3,8) = _cubePoints.transpose();
    return true;
}


void Node::updateCuboid()
{
    // when the new rotation or line information is added in
    // the rotation and trans should be update
    // not to update
}

void Node::updateRelationMat()
{
    // when a rotation matching is add in
    // the relation matrix should be update

    // 1. update in the first three levels
    if(levelNum<=3)
    {
        relationMat(fea1ID, fea2ID) = kind;
    }
    else
    {
        // current node is matching
        if(fea2ID>=0 && fea2ID<=2)
        {
            // if the node is a rotation match
            for(int i=0; i<3; i++)
            {
                for(int j=0; j<pSRG->matchingEdge[i].size(); j++)
                {
                    int feaIDTemp = pSRG->matchingEdge[i][j];
                    if(i==fea2ID)
                    {
                        // need to be parallel
                        relationMat(fea1ID, feaIDTemp) = 0;
                    }
                    else
                    {
                        // need to be prependicular
                        // need to be parallel
                        relationMat(fea1ID, feaIDTemp) = 1;
                    }
                    
                }
            }
        }
    }
}

void Node::updateSRG()
{
    if(levelNum < 3)
    {
        return;
    }
    else if(levelNum == 3)
    {
        /* code */
        pSRG->matchingEdge[0].push_back(pNode->pNode->fea1ID);
        pSRG->matchingEdge[0].push_back(pNode->pNode->fea2ID);
        pSRG->matchingEdge[1].push_back(pNode->fea2ID);
    }
    else
    {
        if(fea2ID!=-1)
        {
            pSRG->matchingEdge[fea2ID].push_back(fea1ID);
        }
    }
    
}



// =========IT function===========//
// constructor function
IT::IT(Obj2D* _pObj2D)
{
    NodeRotTh = _pObj2D->NodeRotTh;
    NodeTranTh = _pObj2D->NodeTranTh;

    Eigen::Matrix<double, 9, 3> scale_pirors;
    scale_pirors << 20, 1, 10,
                    1, 0.6, 1,
                    1, 1.5, 1,
                    1, 0.6, 1,
                    1, 1.8, 1,
                    10, 3, 10,
                    10, 5, 10,
                    1, 1.8, 1,
                    10, 8, 2;
    scale_piror = scale_pirors.block(_pObj2D->label, 0, 1, 3).transpose();
    useFrame = false;
    useGround = false;
    ground << 0, 1, 0;
    // construct the root node
    mostValidRotNumThr = 0;
    mostValidTransNumThr = 0;
    root = new Node(_pObj2D);
    pObj2D = _pObj2D;
    leafNodes.clear();
    // construct observation matrix
}

IT::~IT()
{
    if(root)
    {
        delete root;
        root = NULL;
    }
}

void IT::constructIT(Node* pNode)
{
    // img feature number
    int feaNum = pObj2D->lines.size();

    // the level num
    // if the node levelnum = 0, to generate the first level
    int imgFeaNum = min(feaNum,int(pObj2D->selectedLineID.size()));

    // int imgFeaNum = 6;
    if(pNode->levelNum == 0)
    {
        for(int i=0; i<imgFeaNum-2; i++)
        {
            for(int j=i+1; j<imgFeaNum-1; j++)
            {
                // generate children node for current node
                // i//j in the first level
                // Node* childNode = new Node(pNode, i, j, 0);
                Node* childNode = new Node(pNode, pObj2D->selectedLineID[i], pObj2D->selectedLineID[j], 0);
                processNode(pNode, childNode);
            }
        }
        // generate a node for frame
        if(useFrame)
        {
            Node* childNode = new Node(pNode, -1, -1, -2);
            childNode->levelNum = 3;
            childNode->pObj3D = new Obj3D();
            childNode->pObj3D->R = pObj2D->FrameRot;
            childNode->validRotNum = 1000000;//NodeRotTh;
            processNode(pNode, childNode);
        }
        if(useGround)
        {
            Node* childNode = new Node(pNode, -1, -1, -3);
            childNode->levelNum = 2;
            childNode->pObj3D = new Obj3D();
            childNode->pObj3D->R.block(0,0,3,1) = ground;
            processNode(pNode, childNode);
        }
    }
    // if the node levelnum=1, to generate the 2th level
    else if(pNode->levelNum == 1)
    {
        for(int i=pNode->fea2ID+1; i<imgFeaNum; i++)
        {
            // generate the new child node
            Node* childNode = new Node(pNode, pNode->fea1ID, pObj2D->selectedLineID[i], 1);
            processNode(pNode, childNode);
        }

    }
    else if(pNode->levelNum == 2)
    {
        Node* childNode = new Node(pNode, pNode->pNode->fea2ID, pNode->fea2ID, 1);
        processNode(pNode, childNode);
    }
    else
    {
        if(pObj2D->label == -1)
        {
            if(pNode->levelNum==3)
            {
                leafNodes.push_back(pNode);
            }
            return;
        }
        // for the matching IT
        // if have matched all img features
        // the last level
        if(pNode->levelNum-3>=feaNum*2)
        // if(pNode->levelNum-3>=2)
        {
            // IT has been constructed over
            // go on

            if(pNode->pObj3D->IOU > 0.4)
            {
                leafNodes.push_back(pNode);
            }
            return;
        }
        else
        {
            // to generate new node
            if((pNode->levelNum-3)%2==0)
            {
                // last level is trans match
                // current should be direction
                // in actually, the most sim direction should be selected
                // judge the min angle direction
                int fea1IDTemp = int((pNode->levelNum-3)/2);
                double minRotCost = 1;
                int minRotID = 0;
                for(int i=0; i<3; i++)
                {
                    int fea2IDTemp = i;
                    Eigen::Vector3d plane = pObj2D->lines[fea1IDTemp]->plane.block(0,0,3,1);
                    Eigen::Vector3d direction = pNode->pObj3D->R.block(0,fea2IDTemp,3,1);
                    Eigen::MatrixXd costTempMat = plane.transpose()*direction;
                    double costTemp = abs(costTempMat(0,0));

                    if(costTemp < 0.2)
                    {
                        Node* childNode = new Node(pNode, int((pNode->levelNum-3)/2), i, 2);
                        bool isDireOK = childNode->isOK;
                        processNode(pNode, childNode);
                    }
                    
                    
                    if(costTemp<minRotCost)
                    {
                        minRotCost = costTemp;
                        minRotID = i;
                    }
                }

                {
                    Node* childNode = new Node(pNode, int((pNode->levelNum-3)/2), -1, 2);

                    processNode(pNode, childNode);
                }
                
            }
            // else if((pNode->levelNum-3)==1)
            else
            {
                // last level is dire match
                // current shoule be trans
                // in actually, the most sim direction should be selected
                // compute the edge match cost
                if(pNode->fea2ID != -1)
                {

                    if(pNode->pObj3D->hasTrans)
                    {
                        int fea1IDTemp = pNode->fea1ID;
                        double minTransCost = 10000;
                        int minTransID = 0;
                        for(int i=0; i<4; i++)
                        {
                            int fea2IDTemp = pNode->pObj3D->directionFeaID(i, pNode->fea2ID)+3;
                            Eigen::Vector4d plane = pObj2D->lines[fea1IDTemp]->plane;
                            // the point should in the plane
                            Eigen::Vector4d Point1 = pNode->pObj3D->eightPointsInCamera.block(0,pNode->pObj3D->linePointsID(0,fea2IDTemp-3),4,1);
                            Eigen::Vector4d Point2 = pNode->pObj3D->eightPointsInCamera.block(0,pNode->pObj3D->linePointsID(1,fea2IDTemp-3),4,1);
                            // the two point should on the plane
                            Eigen::MatrixXd dist1 = plane.transpose()*Point1;
                            Eigen::MatrixXd dist2 = plane.transpose()*Point2;
                            double dist = (abs(dist1(0,0))+abs(dist2(0,0)))/2;
                            if(dist < minTransCost)
                            {
                                minTransCost = dist;
                                minTransID = i;
                            }
                            
                        }
                        int fea2ID = pNode->pObj3D->directionFeaID(minTransID, pNode->fea2ID)+3;
                        Node* childNode = new Node(pNode, pNode->fea1ID, fea2ID, 2);
                        processNode(pNode, childNode);
                    }
                    else
                    {
                        // for(int i=0; i<4; i++)
                        for(int i=0; i<4; i++)
                        {
                            pNode->fea2ID = 0;
                            int fea2ID = pNode->pObj3D->directionFeaID(i, pNode->fea2ID)+3;
                            Node* childNode = new Node(pNode, pNode->fea1ID, fea2ID, 2);
                            processNode(pNode, childNode);
                        }
                    }
                }

                Node* childNode = new Node(pNode, pNode->fea1ID, -1, 2);

                processNode(pNode, childNode);

            }
            
            
            
            if(pNode->cNodes.size()==0)
            {
                if(pNode->pObj3D->IOU > 0.4)
                {
                    leafNodes.push_back(pNode);
                }
            }
        }
    }
}

void IT::processNode(Node* pNode, Node* cNode)
{
    // test if the child node is valid
    // if(cNode->levelNum>pObj2D->lineNum*2+1)
    // {
    //     cNode->isOK = false;
    // }

    if(cNode->isOK)
    {
        // if it is valid
        // add it into the path

        pNode->cNodes.push_back(cNode);
        // construtIT for the child node

        constructIT(cNode);

    }
    else
    {
        // otherwise, the child node is invalid
        // it should not be added into the IT
        // delete it 
        delete cNode;
        // add the pNode into the leaf nodes vector
        
    }
}

void IT::estimateTrans(Obj3D* pObj3D, SRG* pSRG)
{
    // with the rotation to estimate the translation and scale

    Eigen::Vector3d cam_vy;
    cam_vy << 0, -1, 0;
    // 确保Vz轴指向上方
    double min_cos = 0;
    double min_id_cos = 0;
    int min_id = -1;
    for(int i=0; i<3; i++)
    {
        Eigen::MatrixXd costTempMat = pObj3D->R.block(0,i,3,1).transpose()*cam_vy;
        double costTemp = abs(costTempMat(0,0));
        if(costTemp > min_cos)
        {
            min_id = i;
            min_cos = costTemp;
            min_id_cos = costTempMat(0,0);
        }
    }
    Eigen::Matrix3d newR;
    newR.setIdentity();
    Eigen::Vector3d directionZ = pObj3D->R.block(0,min_id,3,1);
    Eigen::Vector3d directionX;
    newR.block(0,2,3,1) = directionZ;

    if(min_id<2)
    {
        directionX = pObj3D->R.block(0,min_id+1,3,1);
        newR.block(0,0,3,1) = directionX;
        newR.block(0,1,3,1) = directionZ.cross(directionX);
        pObj3D->R = newR;
    }

    Eigen::Matrix<double, 4, 3> Mst;
    Mst.setIdentity();
    Mst.block(0,0,3,3) = pObj3D->R.transpose();
    // 从图像平面到目标附体坐标系
    Eigen::Matrix<double, 4, 3> Pst = Mst*pObj2D->K.transpose();

    // 得到R之后计算检测框的每一个条线对应的目标顶点
    for(int bbi=0; bbi<4; bbi++)
    {
        pObj2D->BBExpandLines[bbi]->getPlaneParametersInCuboid(Pst);
        pObj2D->BBExpandLines[bbi]->getPlaneObjPointID(Mst, bbi);
    }

    // 构建约束矩阵
    Eigen::Matrix<double, 5, 7> M;
    Eigen::Matrix<double, 5, 1> B;
    M.setZero();
    B.setZero();

    // 检测框四条线的约束
    for(int bbi=0; bbi<4; bbi++)
    {
        M(bbi,0) = pObj2D->BBExpandLines[bbi]->planeInCuboid(0);
        M(bbi,1) = pObj2D->BBExpandLines[bbi]->planeInCuboid(1);
        M(bbi,2) = pObj2D->BBExpandLines[bbi]->planeInCuboid(2);
        M(bbi,3) = pObj2D->BBExpandLines[bbi]->planeInCuboid(0);
        M(bbi,4) = pObj2D->BBExpandLines[bbi]->planeInCuboid(1);
        M(bbi,5) = pObj2D->BBExpandLines[bbi]->planeInCuboid(2);
        M(bbi,6) = pObj2D->BBExpandLines[bbi]->planeInCuboid(3);
        // M.block(bbi,0,1,7).normalize();
    }

    // 最后一个平面决定尺度
    Eigen::Matrix<double, 4, 1> z_plane;
    z_plane << 0, 0, 1, -100;
    Eigen::Matrix4d Mst1;
    Mst1.setIdentity();
    Mst1.block(0,0,4,3) = Mst;
    Eigen::Matrix<double, 4, 1> z_plane_para = Mst1 * z_plane;
    M(4, 0) = z_plane_para(0,0);
    M(4, 1) = z_plane_para(1,0);
    M(4, 2) = z_plane_para(2,0);
    M(4, 3) = z_plane_para(0,0);
    M(4, 4) = z_plane_para(1,0);
    M(4, 5) = z_plane_para(2,0);
    M(4, 6) = z_plane_para(3,0);
    // M.block(6,0,1,7).normalize();

    // 根据匹配的点的id，对M进行变号
    Eigen::Matrix<double, 8, 6> signs;
    signs <<    -1.0, -1.0, +1.0, 1.0, 1.0, 1.0,
                -1.0, +1.0, +1.0, 1.0, 1.0, 1.0,
                +1.0, +1.0, +1.0, 1.0, 1.0, 1.0,
                +1.0, -1.0, +1.0, 1.0, 1.0, 1.0,
                -1.0, -1.0, -1.0, 1.0, 1.0, 1.0,
                -1.0, +1.0, -1.0, 1.0, 1.0, 1.0,
                +1.0, +1.0, -1.0, 1.0, 1.0, 1.0,
                +1.0, -1.0, -1.0, 1.0, 1.0, 1.0;

    // 先是检测框的四条线段
    for(int bbi=0; bbi<4; bbi++)
    {
        // M.block(bbi,0,1,6) = M.block(bbi,0,1,6).array() * signs.block(pObj2D->BBlines[bbi]->pointid,0,1,6).array();
        for(int bbj=0; bbj<6; bbj++)
        {
            M(bbi,bbj) = M(bbi,bbj)*signs(pObj2D->BBExpandLines[bbi]->pointid, bbj);

        }
    }

    for(int Bi=0; Bi<5; Bi++)
    {
        B(Bi) = -M(Bi,6);
    }

    

    for(int mi=0; mi<5; mi++)
    {
        for(int mj=0; mj<2; mj++)
        {
            // B(mi) = B(mi)-M(mi, mj)*scale_piror(mj);
            B(mi) = B(mi)-M(mi, mj);
        }
    }

    Eigen::Matrix<double, 4, 1> X_temp = M.block(0,2,4,4).fullPivHouseholderQr().solve(B.block(0,0,4,1));

    Eigen::Vector3d CenterTemp;
    Eigen::Vector3d AxesTemp;

    // AxesTemp(0) = scale_piror(0);
    // AxesTemp(1) = scale_piror(1);
    // AxesTemp(2) = scale_piror(2);
    AxesTemp(0) = 1;
    AxesTemp(1) = 1;
    AxesTemp(2) = abs(X_temp(0));
    CenterTemp(0) = X_temp(1);
    CenterTemp(1) = X_temp(2);
    CenterTemp(2) = X_temp(3);

    CenterTemp = pObj3D->R*CenterTemp;



    double centerScale = CenterTemp(2);
    for(int i=0; i<3; i++)
    {
        CenterTemp(i) = CenterTemp(i)/centerScale;
        AxesTemp(i) = AxesTemp(i)/centerScale;
    }

    // 计算当前目标的得分
    Eigen::Matrix<double, 8, 3> _cubePoints;
    _cubePoints <<  -AxesTemp(0), -AxesTemp(1),  AxesTemp(2),
                    -AxesTemp(0),  AxesTemp(1),  AxesTemp(2),
                     AxesTemp(0),  AxesTemp(1),  AxesTemp(2),
                     AxesTemp(0), -AxesTemp(1),  AxesTemp(2),
                    -AxesTemp(0), -AxesTemp(1), -AxesTemp(2),
                    -AxesTemp(0),  AxesTemp(1), -AxesTemp(2),
                     AxesTemp(0),  AxesTemp(1), -AxesTemp(2),
                     AxesTemp(0), -AxesTemp(1), -AxesTemp(2);
    
    // 旋转八个点，得到坐标系下的八个顶点

    Eigen::Matrix<double, 3, 8> _cubePoints1 = pObj3D->R*_cubePoints.transpose();


    // 寻找最远点id
    int _fastPID = 0;
    double fastPDist = _cubePoints1(2,0)+CenterTemp(2);
    for(int ci=0; ci<8; ci++)
    {
        _cubePoints1(0,ci) = _cubePoints1(0,ci)+CenterTemp(0);
        _cubePoints1(1,ci) = _cubePoints1(1,ci)+CenterTemp(1);
        _cubePoints1(2,ci) = _cubePoints1(2,ci)+CenterTemp(2);
        if(_cubePoints1(2,ci) > fastPDist)
        {
            fastPDist = _cubePoints1(2,ci);
            _fastPID = ci;
        }
    }

    // 投影到图像平面上
    Eigen::MatrixXd _cubePoints2(8, 4);
    for(int ci=0; ci<8; ci++)
    {
        _cubePoints2(ci,3) = 1;
    }


    _cubePoints2.block(0,0,8,3) = _cubePoints1.transpose();
    Eigen::Matrix<double, 4, 8> cameraCubePoints = Eigen::Matrix4d::Identity()*_cubePoints2.transpose();
    Eigen::Matrix<double, 3, 8> frameCubePoints1 = pObj2D->K*cameraCubePoints.block(0,0,3,8);


    Eigen::Matrix<double, 2, 8> frameCubePoints2d;
    // 对八个点进行归一化
    bool isNAN = false;
    for(int ci=0; ci<8; ci++)
    {
        frameCubePoints2d(0,ci) = frameCubePoints1(0,ci) / frameCubePoints1(2,ci);
        frameCubePoints2d(1,ci) = frameCubePoints1(1,ci) / frameCubePoints1(2,ci);
        if(isnan(frameCubePoints2d(0,ci))||isnan(frameCubePoints2d(1,ci)))
        {
            isNAN = true;
        }
    }

    if(isNAN)
    {
        cout << "NAN" << endl;
        return;
    }

    // 计算投影得到的框
    Eigen::Vector4d ProBB;
    ProBB.setZero();
    ProBB(0) = frameCubePoints2d(0,0);
    ProBB(1) = frameCubePoints2d(1,0);
    ProBB(2) = frameCubePoints2d(0,0);
    ProBB(3) = frameCubePoints2d(1,0);

    for(int fcp_id=0; fcp_id<8; fcp_id++)
    {
        if(frameCubePoints2d(0,fcp_id)<ProBB(0))
        {
            ProBB(0) = frameCubePoints2d(0,fcp_id);
        }
        if(frameCubePoints2d(0,fcp_id)>ProBB(2))
        {
            ProBB(2) = frameCubePoints2d(0,fcp_id);
        }
        if(frameCubePoints2d(1,fcp_id)<ProBB(1))
        {
            ProBB(1) = frameCubePoints2d(1,fcp_id);
        }
        if(frameCubePoints2d(1,fcp_id)>ProBB(3))
        {
            ProBB(3) = frameCubePoints2d(1,fcp_id);
        }
    }

    // 计算投影框与原始框的交并比
    double xmin = max(double(pObj2D->BoundingBox(0)), ProBB(0));
    double ymin = max(double(pObj2D->BoundingBox(1)), ProBB(1));
    // 求交集的右下角的点
    double xmax = min(double(pObj2D->BoundingBox(2)), ProBB(2));
    double ymax = min(double(pObj2D->BoundingBox(3)), ProBB(3));
    // 计算当前帧目标的面积
    double S1 = (pObj2D->BoundingBox(2)-pObj2D->BoundingBox(0))*(pObj2D->BoundingBox(3)-pObj2D->BoundingBox(1));
    // 计算上一帧目标的面积
    double S2 = (ProBB(2)-ProBB(0))*(ProBB(3)-ProBB(1));
    // 计算交部分面积
    // if (xmax-xmin<=0||ymax-ymin<0)
    // {
    //     continue;
    // }
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

    pObj3D->IOU = IOU;


    pObj3D->hasTrans=true;
    pObj3D->Center = CenterTemp;
    pObj3D->Axes = AxesTemp;
    pObj3D->eightPointsInCamera.setZero();
    pObj3D->eightPointsInCamera.block(0,0,3,8) = _cubePoints1;
    pObj3D->eightPointsInCuboid.setZero();
    pObj3D->eightPointsInCuboid.block(0,0,3,8) = _cubePoints.transpose();
}

void IT::getObj3D(Obj3D* &pObj3D, SRG* &pSRG)
{
    Obj3D* _pObj3D;
    SRG* _pSRG;
    double maxScaleThr = 5;


    if(leafNodes.size()==0 || leafNodes.size() > 1000000)
    {
        std::cout << "No leaf nodes" << std::endl;
        // there are no leaf node, node rotation or center/scale
        // the null obj should be used;
        _pObj3D = new Obj3D();
        _pSRG = new SRG();
        _pSRG->pObj2D = pObj2D;
        _pSRG->pObj3D = _pObj3D;
        _pObj3D->R = pObj2D->FrameRot;
        estimateTrans(_pObj3D, _pSRG);
        _pObj3D->hasTrans = true;
        // return _pObj3D;
    }
    // select the leaf node which has the most valid node;
    // 1. select the most number
    double mostValidRotNum = 0.0;
    double maxIOU = 0.0;
    int leafNum = int(leafNodes.size());

    for(int i=0; i<leafNum; i++)
    {


        if(leafNodes[i]->levelNum<5)
        {
            continue;
        }

        double _a = abs(leafNodes[i]->pObj3D->Axes(0));
        double _b = abs(leafNodes[i]->pObj3D->Axes(1));
        double _c = abs(leafNodes[i]->pObj3D->Axes(2));

        double maxScale = max(max(_a/_c,_c/_a), max(_b/_c,_c/_b));
        leafNodes[i]->pObj3D->maxScale = max(maxScale, max(_b/_a,_a/_b));

        if(leafNodes[i]->validRotNum > mostValidRotNum)
        {
            mostValidRotNum = leafNodes[i]->validRotNum;
            maxIOU = leafNodes[i]->pObj3D->IOU;
            _pObj3D = leafNodes[i]->pObj3D;
            _pSRG = leafNodes[i]->pSRG;
        }
    }

    double mostValidTransNum = -10000000000000000.0;
    maxIOU = 0.0;
    int maxID = -1;
    for(int i=0; i<leafNum; i++)
    {
        if(leafNodes[i]->levelNum<5)
        {
            continue;
        }
        if(leafNodes[i]->validRotNum == mostValidRotNum)
        {
            if(leafNodes[i]->validTransNum > mostValidTransNum)
            {
                mostValidTransNum = leafNodes[i]->validTransNum;
                maxIOU = leafNodes[i]->pObj3D->IOU;
                _pObj3D = leafNodes[i]->pObj3D;
                _pSRG = leafNodes[i]->pSRG;
                _pObj3D->tranConf = leafNodes[i]->validTransNum;
                _pObj3D->rotConf = leafNodes[i]->validRotNum;
                maxID = i;
                pObj3D = _pObj3D;
                pSRG = _pSRG;
            }
        }
    }

    
    pObj3D = _pObj3D;
    pSRG = _pSRG;
    
}

}