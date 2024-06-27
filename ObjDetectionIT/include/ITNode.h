#ifndef IT_H
#define IT_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<Eigen/Dense>
#include<Eigen/StdVector>

#include<math.h>
#include<time.h>

#include "line_seg.h"
#include "obj2d.h"
#include "obj3d.h"
#include "util.h"
#include "SRG.h"

using namespace std;

namespace OBJECT
{

class Line_seg;
class Obj2D;
class Obj3D;
class SRG;

// construct the IT node class
class Node
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // the two feature pair
    int fea1ID;
    // -1: None feature
    //  0: Vx, 1:Vy, 2:Vz
    //  3~14: line
    // 15~20: plane
    int fea2ID;
    // kind, 0 is parallel, 1 is perpendicular, 2 is matching
    int kind;
    double NodeRotTh, NodeTranTh;

    // parent node
    Node* pNode;
    // root node
    Node* pRoot;
    // child node
    vector<Node*> cNodes;
    // levels num, the first three is relation IT, the rest is matching IT
    int levelNum;
    // the number of lines support the Rotation
    double validRotNum;
    // the number of lines support the Trans
    double validTransNum;
    // the Rotation cost
    double rotCost;
    // the Trans cost
    double transCost;
    // trans threshold
    double tranTh;
    // observation matrix
    // -1 is no relation
    //  0 is parallel
    //  1 is prependicular
    //  2 is parallel or prependicular
    Eigen::MatrixXi observationMat;
    // relation matrix
    Eigen::MatrixXi relationMat;
    // cuboid model
    Obj3D* pObj3D;
    // 2D object
    Obj2D* pObj2D;
    // cost
    // double rotationCost;
    // double transCost;
    // could be add into IT
    bool isOK;
    // SRG
    SRG* pSRG;

    // Function
    // initialization function
    Node(Obj2D* pObj2D);
    Node(Node* pNode, int _imgFeaID, int _cubeFeaID, int kind);
    ~Node();
    // judge observation consistency
    bool judgeObMat();
    // judge relation consistency
    bool judgeRelationMat();
    // judge cuboid model
    bool judgeCuboidModel();
    
    // estimate the rotation matrix
    void estimateRotation();
    // estimate the translation and scale
    bool estimateTrans();
    // update the cuboid model;
    void updateCuboid();
    // update the relation matrix;
    void updateRelationMat();
    // update the SRG relation
    void updateSRG();
    // constuct observation matrix
    void constructObMat();

};

class IT
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // root node
    Node* root;
    // the Obj2D pointer, to get the information about img feature
    Obj2D* pObj2D;

    double mostValidRotNumThr;
    double mostValidTransNumThr;
    Eigen::Vector3d scale_piror;
    bool useFrame;
    bool useGround;
    Eigen::Vector3d ground;
    
    // all the leaf node
    vector<Node*> leafNodes;

    // IT RotTh, TranTh
    double NodeRotTh, NodeTranTh;

    // initialization function
    IT(Obj2D* _pObj2D);
    ~IT();
    // iter function
    void constructIT(Node* pNode);
    
    // process the new child node
    void processNode(Node* pNode, Node* cNode);

    void getObj3D(Obj3D* &pObj3D, SRG* &pSRG);
    void getObj3D1(Obj3D* &pObj3D, SRG* &pSRG);
    void getFrame3D(Obj3D* &pObj3D, SRG* &pSRG);
    void estimateTrans(Obj3D* pObj3D, SRG* pSRG);
};

}


#endif