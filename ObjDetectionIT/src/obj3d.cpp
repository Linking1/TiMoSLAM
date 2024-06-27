#include "obj3d.h"

namespace OBJECT
{

Obj3D::Obj3D()
{
    maxScale = 0;
    Axes << 1, 1, 1;
    Center << 0, 0, 10;
    R.setIdentity();
    IOU = 0.0;
    label = 0;
    id = 0;
    conf = 1;
    hasRot = false;
    hasTrans = false;

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

    //             |----Vx----||----Vy---||----Vz----|
    //              3, 4, 5, 6, 7, 8, 9,10,11,12,13,14
    //              0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11
    linePointsID << 1, 0, 5, 4, 0, 2, 4, 6, 0, 1, 2, 3,
                    2, 3, 6, 7, 1, 3, 5, 7, 4, 5, 6, 7;
    //             |-Vx-||-Vy-||-Vz-|
    //             15,16,17,18,19,20
    //              0, 1, 2, 3, 4, 5
    planePointsID<< 0, 3, 0, 1, 0, 4,
                    1, 2, 3, 2, 1, 5,
                    5, 6, 7, 6, 2, 6,
                    4, 7, 4, 5, 3, 7;
    //              x, y, z
    directionFeaID<<0, 4, 8, 
                    1, 5, 9,
                    2, 6,10,
                    3, 7,11,
                    0, 2, 4,
                    1, 3, 5;
    eightPointsInCuboid.setZero();
    eightPointsInCamera.setZero();
}

Obj3D::Obj3D(Obj3D* _pObj3D)
{
    maxScale = _pObj3D->maxScale;
    IOU = _pObj3D->IOU;
    Center = _pObj3D->Center;
    Axes = _pObj3D->Axes;
    R = _pObj3D->R;
    label = _pObj3D->label;
    id = _pObj3D->id;
    conf = _pObj3D->conf;
    hasRot = _pObj3D->hasRot;
    hasTrans = _pObj3D->hasTrans;
    linePointsID = _pObj3D->linePointsID;
    planePointsID = _pObj3D->planePointsID;
    directionFeaID = _pObj3D->directionFeaID;
    eightPointsInCuboid = _pObj3D->eightPointsInCuboid;
    eightPointsInCamera = _pObj3D->eightPointsInCamera;
}

Obj3D::~Obj3D()
{
}

void Obj3D::getEightPointsInCuboid()
{
    Eigen::Matrix<double, 8, 4> points;

    points << -Axes(0), -Axes(1), +Axes(2), 1,
              -Axes(0), +Axes(1), +Axes(2), 1,
              +Axes(0), +Axes(1), +Axes(2), 1,
              +Axes(0), -Axes(1), +Axes(2), 1,
              -Axes(0), -Axes(1), -Axes(2), 1,
              -Axes(0), +Axes(1), -Axes(2), 1,
              +Axes(0), +Axes(1), -Axes(2), 1,
              +Axes(0), -Axes(1), -Axes(2), 1;

    eightPointsInCuboid = points.transpose();

}

void Obj3D::getEightPointsInCamera()
{
    Eigen::Matrix4d Transform;
    Transform.setIdentity();
    Transform.block(0,0,3,3) = R;
    Transform(0,3) = Center(0);
    Transform(1,3) = Center(1);
    Transform(2,3) = Center(2);
    eightPointsInCamera = Transform*eightPointsInCuboid;


}

}