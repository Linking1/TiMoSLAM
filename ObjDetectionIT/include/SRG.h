#ifndef SRG_H
#define SRG_H

#include "obj2d.h"
#include "obj3d.h"

using namespace std;

namespace OBJECT
{

class Obj2D;
class Obj3D;

class SRG
{
public:
    // obj2D pointer
    Obj2D* pObj2D;
    Obj3D* pObj3D;

    // relation edge in SRG
    map<int, int> relationEdge;

    // matching edge in SRG
    // map<int, int> matchingEdge;
    vector<vector<int> > matchingEdge;

    // constructed function
    SRG();
    SRG(SRG* pSRG);
    ~SRG();
    // get the edge from path in interpretation
    void getEdges();

};

}

#endif