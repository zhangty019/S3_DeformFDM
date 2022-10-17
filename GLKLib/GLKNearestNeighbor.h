// GLKNearestNeighbor.h: interface for the GLKNearestNeighbor class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _GLKNEAREST_NEIGHBOR
#define _GLKNEAREST_NEIGHBOR

#include "GLKObList.h"

class GLKNearestNeighborNode : public GLKObject
{
public:
	float m_pos[3];
	int m_index;
	GLKPOSITION m_PosInGLKObList;
};

class GLKNearestNeighbor  
{
public:
	GLKNearestNeighbor(int pntNum, float* xpos, float *ypos, float *zpos);
	GLKNearestNeighbor(int pntNum, float** pnts);
	virtual ~GLKNearestNeighbor();

	void PntsInRange(float queryPnt[], float range, GLKArray* indexArray);
	void MovePnt(int pntIndex, float xNewPos, float yNewPos, float zNewPos);

private:
	int m_xNum,m_yNum,m_zNum;
	float m_size,m_xmin,m_ymin,m_zmin,m_xmax,m_ymax,m_zmax;
	GLKObList ****m_grids;

	int m_pntNum;
	GLKNearestNeighborNode **m_nodeArray;
};

#endif
