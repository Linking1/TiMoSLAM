#include "SRG.h"

using namespace std;

namespace OBJECT{

SRG::SRG()
{
    pObj2D = NULL;
    pObj3D = NULL;
    for(int i=0; i<21; i++)
    {
        vector<int> temp;
        matchingEdge.push_back(temp);
    }
}

SRG::SRG(SRG* pSRG)
{
    pObj2D = NULL;
    pObj3D = NULL;
    for(int i=0; i<21; i++)
    {
        vector<int> temp;
        for(int j=0; j<pSRG->matchingEdge[i].size(); j++)
        {
            temp.push_back(pSRG->matchingEdge[i][j]);
        }
        matchingEdge.push_back(temp);
    }
}

SRG::~SRG()
{
    if(pObj2D)
    {
        delete pObj2D;
        pObj2D = NULL;
    }

    if(pObj3D)
    {
        delete pObj3D;
        pObj3D = NULL;
    }
    return;

}

}