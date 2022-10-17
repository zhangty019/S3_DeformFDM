// GLKNearestNeighbor.cpp: implementation of the GLKNearestNeighbor class.
//
//////////////////////////////////////////////////////////////////////

#include <math.h>
#include "GLKNearestNeighbor.h"

#define GLKNN_GRIDNUM	50.0f

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

GLKNearestNeighbor::GLKNearestNeighbor(int pntNum, float** pnts)
{
	float xmin,ymin,zmin,xmax,ymax,zmax;
	int i,j,k,m;

	m_pntNum=0;
	if (pntNum<=0) return;
	m_pntNum=pntNum;

	xmin=1.0e+10;	ymin=1.0e+10;	zmin=1.0e+10;	
	xmax=-1.0e+10;	ymax=-1.0e+10;	zmax=-1.0e+10;	
	for(i=0;i<pntNum;i++) {
		if (pnts[i][0]>xmax) xmax=pnts[i][0];
		if (pnts[i][1]>ymax) ymax=pnts[i][1];
		if (pnts[i][2]>zmax) zmax=pnts[i][2];
		if (pnts[i][0]<xmin) xmin=pnts[i][0];
		if (pnts[i][1]<ymin) ymin=pnts[i][1];
		if (pnts[i][2]<zmin) zmin=pnts[i][2];
	}

	m_size=(float)(pow((double)(((xmax-xmin)*(ymax-ymin)*(zmax-zmin))/(GLKNN_GRIDNUM*GLKNN_GRIDNUM*GLKNN_GRIDNUM)),(double)(1.0/3.0)));

	m_xmin=xmin-m_size*0.5f;
	m_ymin=ymin-m_size*0.5f;
	m_zmin=zmin-m_size*0.5f;
	m_xNum=(int)((xmax-m_xmin)/m_size)+1;
	m_yNum=(int)((ymax-m_ymin)/m_size)+1;
	m_zNum=(int)((zmax-m_zmin)/m_size)+1;
	m_xmax=m_xmin+m_size*(float)m_xNum-m_size*0.001f;
	m_ymax=m_ymin+m_size*(float)m_yNum-m_size*0.001f;
	m_zmax=m_zmin+m_size*(float)m_zNum-m_size*0.001f;

	m_grids=(GLKObList ****)new long[m_xNum];
	for(i=0;i<m_xNum;i++) {
		m_grids[i]=(GLKObList ***)new long[m_yNum];
		for(j=0;j<m_yNum;j++) {
			m_grids[i][j]=(GLKObList **)new long[m_zNum];
			for(k=0;k<m_zNum;k++) {
				m_grids[i][j][k]=new GLKObList;
				m_grids[i][j][k]->RemoveAll();
			}
		}
	}

	m_nodeArray=(GLKNearestNeighborNode**)new long[pntNum];
	for(m=0;m<pntNum;m++) {
		GLKNearestNeighborNode *node;

		node=new GLKNearestNeighborNode;
		node->m_pos[0]=pnts[m][0];
		node->m_pos[1]=pnts[m][1];
		node->m_pos[2]=pnts[m][2];
		node->m_index=m;

		i=(int)((pnts[m][0]-m_xmin)/m_size);
		j=(int)((pnts[m][1]-m_ymin)/m_size);
		k=(int)((pnts[m][2]-m_zmin)/m_size);

		node->m_PosInGLKObList=m_grids[i][j][k]->AddTail(node);
		m_nodeArray[m]=node;
	}
}

GLKNearestNeighbor::GLKNearestNeighbor(int pntNum, float* xpos, float *ypos, float *zpos)
{
	float xmin,ymin,zmin,xmax,ymax,zmax;
	int i,j,k,m;

	m_pntNum=0;
	if (pntNum<=0) return;
	m_pntNum=pntNum;

	xmin=1.0e+10;	ymin=1.0e+10;	zmin=1.0e+10;	
	xmax=-1.0e+10;	ymax=-1.0e+10;	zmax=-1.0e+10;	
	for(i=0;i<pntNum;i++) {
		if (xpos[i]>xmax) xmax=xpos[i];
		if (ypos[i]>ymax) ymax=ypos[i];
		if (zpos[i]>zmax) zmax=zpos[i];
		if (xpos[i]<xmin) xmin=xpos[i];
		if (ypos[i]<ymin) ymin=ypos[i];
		if (zpos[i]<zmin) zmin=zpos[i];
	}

	m_size=(float)(pow((double)(((xmax-xmin)*(ymax-ymin)*(zmax-zmin))/(GLKNN_GRIDNUM*GLKNN_GRIDNUM*GLKNN_GRIDNUM)),(double)(1.0/3.0)));

	m_xmin=xmin-m_size*0.5f;
	m_ymin=ymin-m_size*0.5f;
	m_zmin=zmin-m_size*0.5f;
	m_xNum=(int)((xmax-m_xmin)/m_size)+1;
	m_yNum=(int)((ymax-m_ymin)/m_size)+1;
	m_zNum=(int)((zmax-m_zmin)/m_size)+1;
	m_xmax=m_xmin+m_size*(float)m_xNum-m_size*0.001f;
	m_ymax=m_ymin+m_size*(float)m_yNum-m_size*0.001f;
	m_zmax=m_zmin+m_size*(float)m_zNum-m_size*0.001f;

	m_grids=(GLKObList ****)new long[m_xNum];
	for(i=0;i<m_xNum;i++) {
		m_grids[i]=(GLKObList ***)new long[m_yNum];
		for(j=0;j<m_yNum;j++) {
			m_grids[i][j]=(GLKObList **)new long[m_zNum];
			for(k=0;k<m_zNum;k++) {
				m_grids[i][j][k]=new GLKObList;
				m_grids[i][j][k]->RemoveAll();
			}
		}
	}

	m_nodeArray=(GLKNearestNeighborNode**)new long[pntNum];
	for(m=0;m<pntNum;m++) {
		GLKNearestNeighborNode *node;

		node=new GLKNearestNeighborNode;
		node->m_pos[0]=xpos[m];
		node->m_pos[1]=ypos[m];
		node->m_pos[2]=zpos[m];
		node->m_index=m;

		i=(int)((xpos[m]-m_xmin)/m_size);
		j=(int)((ypos[m]-m_ymin)/m_size);
		k=(int)((zpos[m]-m_zmin)/m_size);

		node->m_PosInGLKObList=m_grids[i][j][k]->AddTail(node);
		m_nodeArray[m]=node;
	}
}

GLKNearestNeighbor::~GLKNearestNeighbor()
{
	int i,j,k;

	if (m_pntNum>0) {
		for(i=0;i<m_pntNum;i++) delete (GLKNearestNeighborNode*)(m_nodeArray[i]);
		delete [](GLKNearestNeighborNode**)m_nodeArray;
	}

	for(i=0;i<m_xNum;i++) {
		for(j=0;j<m_yNum;j++) {
			for(k=0;k<m_zNum;k++)	{
				delete (m_grids[i][j][k]);
			}
			delete [](GLKObList **)(m_grids[i][j]);
		}
		delete [](GLKObList ***)(m_grids[i]);
	}
	delete [](GLKObList ****)(m_grids);
}

//////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////
	
void GLKNearestNeighbor::MovePnt(int pntIndex, float xNewPos, float yNewPos, float zNewPos)
{
	int i,j,k;
	GLKNearestNeighborNode *node=m_nodeArray[pntIndex];

	i=(int)((node->m_pos[0]-m_xmin)/m_size);
	j=(int)((node->m_pos[1]-m_ymin)/m_size);
	k=(int)((node->m_pos[2]-m_zmin)/m_size);

	m_grids[i][j][k]->RemoveAt(node->m_PosInGLKObList);

	if (xNewPos<m_xmin) xNewPos=m_xmin;
	if (yNewPos<m_ymin) yNewPos=m_ymin;
	if (zNewPos<m_zmin) zNewPos=m_zmin;
	if (xNewPos>m_xmax) xNewPos=m_xmax;
	if (yNewPos>m_ymax) yNewPos=m_ymax;
	if (zNewPos>m_zmax) zNewPos=m_zmax;

	node->m_pos[0]=xNewPos; node->m_pos[1]=yNewPos; node->m_pos[2]=zNewPos;
	i=(int)((node->m_pos[0]-m_xmin)/m_size);
	j=(int)((node->m_pos[1]-m_ymin)/m_size);
	k=(int)((node->m_pos[2]-m_zmin)/m_size);
	node->m_PosInGLKObList=m_grids[i][j][k]->AddTail(node);
}

void GLKNearestNeighbor::PntsInRange(float queryPnt[], float range, GLKArray* indexArray)
{
	int i,j,k,ii,jj,kk,dd;
	float ll;
	GLKPOSITION Pos;

	dd=(int)(range/m_size)+1;
	ii=(int)((queryPnt[0]-m_xmin)/m_size);
	jj=(int)((queryPnt[1]-m_ymin)/m_size);
	kk=(int)((queryPnt[2]-m_zmin)/m_size);

	range=range*range;

	indexArray->RemoveAll();
	for(i=ii-dd;i<=ii+dd;i++) {
		if (i<0) continue;
		if (i>=m_xNum) continue;
		for(j=jj-dd;j<=jj+dd;j++) {
			if (j<0) continue;
			if (j>=m_yNum) continue;
			for(k=kk-dd;k<=kk+dd;k++) {
				if (k<0) continue;
				if (k>=m_zNum) continue;

                for(Pos=m_grids[i][j][k]->GetHeadPosition();Pos!=nullptr;) {
					GLKNearestNeighborNode *node=(GLKNearestNeighborNode *)(m_grids[i][j][k]->GetNext(Pos));

					ll=(node->m_pos[0]-queryPnt[0])*(node->m_pos[0]-queryPnt[0])
						+(node->m_pos[1]-queryPnt[1])*(node->m_pos[1]-queryPnt[1])
						+(node->m_pos[2]-queryPnt[2])*(node->m_pos[2]-queryPnt[2]);
					if (ll<=range) indexArray->Add(node->m_index);
				}
			}
		}
	}
}
