// QMeshFace.cpp: implementation of the QMeshFace class.
//
//////////////////////////////////////////////////////////////////////

#include <math.h>

#include "../GLKLib/GLKGeometry.h"

#include "QMeshFace.h"
#include "QMeshEdge.h"
#include "QMeshNode.h"
#include "QMeshTetra.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

QMeshFace::QMeshFace()
{
	int i;

	nSplittedNode=-1;
	for(i=0;i<8;i++) flags[i]=false;
	indexno=0;	edgeNum=0;
    meshSurface=nullptr;

	rgb[0]=0.0f;	rgb[1]=0.8f;	rgb[2]=0.8f;

	attachedList.RemoveAll();	

	m_nIdentifiedPatchIndex=-1;
	SetLeftTetra(NULL);
	SetRightTetra(NULL);
	for(i=0;i<MAX_EDGE_NUM;i++) nodeAngle[i]=-1.0e+32;
}

QMeshFace::~QMeshFace()
{

}

//////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////

bool QMeshFace::GetAttribFlag( const int whichBit )
{
	return flags[whichBit];
}

void QMeshFace::SetAttribFlag( const int whichBit, const bool toBe )
{
	flags[whichBit]=toBe;
}

int QMeshFace::GetIndexNo()		//from 1 to n
{
	return indexno;
}

void QMeshFace::SetIndexNo( const int _index )
{
	indexno=_index;
}

int QMeshFace::GetEdgeNum()
{
	return edgeNum;
}

void QMeshFace::SetEdgeNum(int num)
{
	edgeNum=num;
}

bool QMeshFace::IsNormalDirection( const int whichEdge )
{
	return edgeDir[whichEdge];
}

void QMeshFace::SetDirectionFlag( const int whichEdge, const bool toBe )
{
	edgeDir[whichEdge]=toBe;
}

QMeshEdge* QMeshFace::GetEdgeRecordPtr( const int whichEdge )
{
	return edges[whichEdge-1];
}

void QMeshFace::SetEdgeRecordPtr( const int whichEdge, QMeshEdge * _edge )
{
	edges[whichEdge]=_edge;
}

void QMeshFace::GetNodePos( const int whichNode, double &xx, double &yy, double &zz)
{
	GetNodeRecordPtr(whichNode)->GetCoord3D(xx,yy,zz);
}

QMeshNode* QMeshFace::GetNodeRecordPtr( const int whichNode )
{
	if (IsNormalDirection(whichNode))
	    return edges[whichNode]->GetStartPoint();
	else
	    return edges[whichNode]->GetEndPoint();
    return nullptr;
}

double QMeshFace::GetNodeAngle( const int whichNode)
{
	return nodeAngle[whichNode];
}

void QMeshFace::SetNodeAngle( const int whichNode, double angleInRadian)
{
	nodeAngle[whichNode]=angleInRadian;
}

void QMeshFace::SetColor(float r, float g, float b)
{
	rgb[0]=r;	rgb[1]=g;	rgb[2]=b;
}

void QMeshFace::GetColor(float &r, float &g, float &b)
{
	r=rgb[0];	g=rgb[1];	b=rgb[2];
}

void QMeshFace::SetHermiteData(double pos[], double normal[])
{
	m_HermitePos[0]=pos[0];	m_HermitePos[1]=pos[1];	m_HermitePos[2]=pos[2];
	m_HermiteNormal[0]=normal[0];	m_HermiteNormal[1]=normal[1];	m_HermiteNormal[2]=normal[2];
}

void QMeshFace::GetHermiteData(double pos[], double normal[])
{
	pos[0]=m_HermitePos[0];	pos[1]=m_HermitePos[1];	pos[2]=m_HermitePos[2];
	normal[0]=m_HermiteNormal[0];	normal[1]=m_HermiteNormal[1];	normal[2]=m_HermiteNormal[2];
}

void QMeshFace::GetPlaneEquation( double & A, double & B, double & C, double & D )
{
	A=abcd[0];	B=abcd[1];	C=abcd[2];	D=abcd[3];
}

double QMeshFace::CalArea2D()
{
	double cp[2],p1[2],p2[2];
	double area,x1,y1;
	int i;

	cp[0]=0.0;	cp[1]=0.0;
	for(i=0;i<edgeNum;i++) {
		GetNodeRecordPtr(i)->GetCoord2D(x1,y1);
		cp[0]+=x1;	cp[1]+=y1;
	}
	cp[0]=cp[0]/(double)edgeNum;	cp[1]=cp[1]/(double)edgeNum;

	area=0.0;
	GetNodeRecordPtr(0)->GetCoord2D(p1[0],p1[1]);
	for(i=0;i<edgeNum;i++) {
		GetNodeRecordPtr((i+1)%edgeNum)->GetCoord2D(p2[0],p2[1]);

		area+=0.5*((p1[0]-p2[0])*(p1[1]+p2[1])+(p2[0]-cp[0])*(p2[1]+cp[1])+(cp[0]-p1[0])*(cp[1]+p1[1]));

		p1[0]=p2[0]; p1[1]=p2[1];
	}

	m_area=area;

	return area;
}

double QMeshFace::CalArea()
{
	double cp[3],p1[3],p2[3];
	double area;
	double x1,y1,z1,x2,y2,z2;
	double ii,jj,kk;
	int i;

	CalCenterPos(cp[0],cp[1],cp[2]);	area=0.0;

	GetNodePos(0,p1[0],p1[1],p1[2]);
	for(i=0;i<edgeNum;i++) {
		GetNodePos(((i+1)%edgeNum),p2[0],p2[1],p2[2]);

		x1=p1[0]-cp[0];	y1=p1[1]-cp[1];	z1=p1[2]-cp[2];
		x2=p2[0]-cp[0];	y2=p2[1]-cp[1];	z2=p2[2]-cp[2];

		ii=y1*z2-z1*y2;
		jj=x2*z1-x1*z2;
		kk=x1*y2-x2*y1;

		area+=sqrt(ii*ii+jj*jj+kk*kk)/2.0;

		p1[0]=p2[0]; p1[1]=p2[1]; p1[2]=p2[2];
	}

	m_area=area;

	return area;
}

void QMeshFace::CalBoundingBox(double &xmin, double &ymin, double &zmin,
							   double &xmax, double &ymax, double &zmax)
{
	int pntNum=GetEdgeNum();
	double pp[3];

	GetNodePos(0,xmin,ymin,zmin);
	GetNodePos(0,xmax,ymax,zmax);
	for(int i=1;i<pntNum;i++) {
		GetNodePos(i,pp[0],pp[1],pp[2]);
		if (pp[0]<xmin) xmin=pp[0];
		if (pp[0]>xmax) xmax=pp[0];
		if (pp[1]<ymin) ymin=pp[1];
		if (pp[1]>ymax) ymax=pp[1];
		if (pp[2]<zmin) zmin=pp[2];
		if (pp[2]>zmax) zmax=pp[2];
	}
}

void QMeshFace::CalCenterPos()
{
	double xx, yy, zz;
	int pntNum = GetEdgeNum();
	double pp[3];

	xx = 0.0;	yy = 0.0;	zz = 0.0;
	for (int i = 0; i<pntNum; i++) {
		GetNodePos(i, pp[0], pp[1], pp[2]);
		xx = xx + pp[0];	yy = yy + pp[1];	zz = zz + pp[2];
	}
	xx = xx / (double)pntNum;	yy = yy / (double)pntNum;	zz = zz / (double)pntNum;
	center[0] = xx; center[1] = yy; center[2] = zz;
}

void QMeshFace::CalCenterPos(double &xx, double &yy, double &zz)
{
	int pntNum=GetEdgeNum();
	double pp[3];

	xx=0.0;	yy=0.0;	zz=0.0;
	for(int i=0;i<pntNum;i++) {
		GetNodePos(i,pp[0],pp[1],pp[2]);
		xx=xx+pp[0];	yy=yy+pp[1];	zz=zz+pp[2];
	}
	xx=xx/(double)pntNum;	yy=yy/(double)pntNum;	zz=zz/(double)pntNum;	
    center[0]=xx; center[1]=yy; center[2]=zz;
}

void QMeshFace::CalCenterPos_last(double &xx, double &yy, double &zz)
{
	int pntNum=GetEdgeNum();
	double pp[3];

	xx=0.0;	yy=0.0;	zz=0.0;
	for(int i=0;i<pntNum;i++) {
		GetNodeRecordPtr(i)->GetCoord3D_last(pp[0],pp[1],pp[2]);
		xx=xx+pp[0];	yy=yy+pp[1];	zz=zz+pp[2];
	}
	xx=xx/(double)pntNum;	yy=yy/(double)pntNum;	zz=zz/(double)pntNum;	
}

void QMeshFace::CalPlaneEquation( )
{
/*	if (edgeNum>3) 
	{
		double aa,bb,cc,dd;
		double nv[3],cp[3],v1[3],v2[3];		int i;

		aa=bb=cc=0.0;
		CalCenterPos(cp[0],cp[1],cp[2]);
		for(i=0;i<edgeNum;i++) {
			GetNodePos(i,v1[0],v1[1],v1[2]);
			GetNodePos((i+1)%edgeNum,v2[0],v2[1],v2[2]);

			v1[0]=v1[0]-cp[0];	v1[1]=v1[1]-cp[1];	v1[2]=v1[2]-cp[2];
			v2[0]=v2[0]-cp[0];	v2[1]=v2[1]-cp[1];	v2[2]=v2[2]-cp[2];

			nv[0]=v1[1]*v2[2]-v1[2]*v2[1];	nv[1]=v1[2]*v2[0]-v1[0]*v2[2];	nv[2]=v1[0]*v2[1]-v1[1]*v2[0];
			dd=nv[0]*nv[0]+nv[1]*nv[1]+nv[2]*nv[2];
			if (dd<1.0e-10) continue;
			dd=sqrt(dd);	nv[0]=nv[0]/dd;	nv[1]=nv[1]/dd;	nv[2]=nv[2]/dd;	
			//	without normalization - like the computation weighted by the area of triangle
			aa+=nv[0];	bb+=nv[1];	cc+=nv[2];
		}
		dd=aa*aa+bb*bb+cc*cc;	if (dd<1.0e-10) dd=1.0;
		dd=sqrt(dd);	
		abcd[0]=aa/dd;	abcd[1]=bb/dd;	abcd[2]=cc/dd;	abcd[3]=-(cp[0]*aa+cp[1]*bb+cp[2]*cc);
		return;
	}*/

	GLKGeometry geo;
	double p[3][3],nv[3],cp[3];	int i;
	double aa,bb,cc,dd;

	aa=bb=cc=0.0;
	CalCenterPos(cp[0],cp[1],cp[2]);
	for(i=0;i<edgeNum;i++) {
		GetNodePos(i,p[0][0],p[0][1],p[0][2]);
		GetNodePos((i+1)%edgeNum,p[1][0],p[1][1],p[1][2]);
		GetNodePos((i+2)%edgeNum,p[2][0],p[2][1],p[2][2]);
		geo.CalPlaneEquation(p[0],p[1],p[2],nv[0],nv[1],nv[2],dd);
		aa+=nv[0];	bb+=nv[1];	cc+=nv[2];
	}
	dd=aa*aa+bb*bb+cc*cc;	if (dd<1.0e-10) dd=1.0;
	dd=sqrt(dd);	
    abcd[0]=aa/dd;	abcd[1]=bb/dd;	abcd[2]=cc/dd;	//abcd[3]=-(cp[0]*aa+cp[1]*bb+cp[2]*cc);
    abcd[3]=-(cp[0]*abcd[0]+cp[1]*abcd[1]+cp[2]*abcd[2]);
}

void QMeshFace::SetMeshPatchPtr(QMeshPatch* _mesh)
{
	meshSurface=_mesh;
}

QMeshPatch* QMeshFace::GetMeshPatchPtr()
{
	return meshSurface;
}

//For volume mesh
QMeshTetra * QMeshFace::GetLeftTetra()
{
	return pLeftTetra;
}

void QMeshFace::SetLeftTetra(QMeshTetra * _pLeftTetra)
{	
	pLeftTetra = _pLeftTetra;
}

QMeshTetra * QMeshFace::GetRightTetra()
{
	return pRightTetra;
}

void QMeshFace::SetRightTetra(QMeshTetra * _pRightTetra)
{
	pRightTetra = _pRightTetra;
}

bool QMeshFace::isBoundaryFace() {

	if (edges[0]->IsBoundaryEdge() ||
		edges[1]->IsBoundaryEdge() || edges[2]->IsBoundaryEdge())
		return true;
	else return false;
}

int QMeshFace::getNodePtrNumber(QMeshNode* node)
{
	int index = -1;
	for (int i = 0; i < 3; i++) {
		if (node == GetNodeRecordPtr(i)) index = i;
	}
	return index;
}