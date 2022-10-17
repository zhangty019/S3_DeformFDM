// QMeshEdge.cpp: implementation of the QMeshEdge class.
//
//////////////////////////////////////////////////////////////////////

#include <math.h>

#include "../GLKLib/GLKObList.h"

#include "QMeshEdge.h"
#include "QMeshFace.h"
#include "QMeshNode.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

QMeshEdge::QMeshEdge()
{
	indexno=0;	m_sharpFactor=0;
    pLeftFace=nullptr;	pRightFace=nullptr;
	for(int i=0;i<8;i++) flags[i]=false;
	SetStartPoint(nullptr);
	SetEndPoint(nullptr);
	SetLeftFace(nullptr);
	SetRightFace(nullptr);

//	attrPntNum=0;	
    attachedPointer=nullptr;
}

QMeshEdge::~QMeshEdge()
{
	//if (attrPntNum>0)
	//{
	//	for(int i=0;i<attrPntNum;i++) delete [](float*)attrPnt[i];
	//	delete [](float**)attrPnt;
	//	delete attrLength;
	//}
}

//////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////

bool QMeshEdge::GetAttribFlag( const int whichBit )
{
	return flags[whichBit];
}

void QMeshEdge::SetAttribFlag( const int whichBit, const bool toBe )
{
	flags[whichBit]=toBe;
}

int QMeshEdge::GetIndexNo() 		//from 1 to n
{
	return indexno;
}

void QMeshEdge::SetIndexNo( const int _index )
{
	indexno=_index;
}

bool QMeshEdge::IsBoundaryEdge() 
{
    if ((pLeftFace==nullptr) || (pRightFace==nullptr))
		return true;
	return false;
}

QMeshNode* QMeshEdge::GetStartPoint() 
{
	return pStartPoint;
}

void QMeshEdge::SetStartPoint( QMeshNode * _pStartPoint )
{
	pStartPoint=_pStartPoint;
}

QMeshNode* QMeshEdge::GetEndPoint() 
{
	return pEndPoint;
}

void QMeshEdge::SetEndPoint( QMeshNode * _pEndPoint )
{
	pEndPoint=_pEndPoint;
}

double QMeshEdge::CalLength()
{
	double pp1[3],pp2[3],ll;

	pStartPoint->GetCoord3D(pp1[0],pp1[1],pp1[2]);
	pEndPoint->GetCoord3D(pp2[0],pp2[1],pp2[2]);
	ll=sqrt((pp1[0]-pp2[0])*(pp1[0]-pp2[0])
		+(pp1[1]-pp2[1])*(pp1[1]-pp2[1])
		+(pp1[2]-pp2[2])*(pp1[2]-pp2[2]));

	m_edgeLength=ll;

	return ll;
}

double QMeshEdge::Cal2DLength()
{
	double pp1[2],pp2[2],ll;

	pStartPoint->GetCoord2D(pp1[0],pp1[1]);
	pEndPoint->GetCoord2D(pp2[0],pp2[1]);
	ll=sqrt((pp1[0]-pp2[0])*(pp1[0]-pp2[0])
		+(pp1[1]-pp2[1])*(pp1[1]-pp2[1]));

	m_edge2DLength=ll;

	return ll;
}

void QMeshEdge::CalNormal(double normal[])
{
	double lv[3],rv[3],nv[3],dd;

	if (!(pLeftFace)) {pRightFace->GetPlaneEquation(normal[0],normal[1],normal[2],dd);return;}
	if (!(pRightFace)) {pLeftFace->GetPlaneEquation(normal[0],normal[1],normal[2],dd);return;}

	pLeftFace->GetPlaneEquation(lv[0],lv[1],lv[2],dd);
	pRightFace->GetPlaneEquation(rv[0],rv[1],rv[2],dd);
	
	nv[0]=(lv[0]+rv[0])/2.0; 
	nv[1]=(lv[1]+rv[1])/2.0; 
	nv[2]=(lv[2]+rv[2])/2.0;
	
	dd=sqrt(nv[0]*nv[0]+nv[1]*nv[1]+nv[2]*nv[2]);
	if (dd<1.0e-8) {
		normal[0]=0.0;	normal[1]=0.0;	normal[2]=0.0;
	}
	else {
		normal[0]=nv[0]/dd;	normal[1]=nv[1]/dd;	normal[2]=nv[2]/dd;
	}
}

void QMeshEdge::SetSharpFactor(int factor)
{
	m_sharpFactor=factor;
}

int QMeshEdge::GetSharpFactor()
{
	return m_sharpFactor;
}

QMeshFace* QMeshEdge::GetLeftFace() 
{
	return pLeftFace;
}

void QMeshEdge::SetLeftFace( QMeshFace * _pLeftFace )
{
	pLeftFace=_pLeftFace;
}

QMeshFace* QMeshEdge::GetRightFace() 
{
	return pRightFace;
}

void QMeshEdge::SetRightFace( QMeshFace * _pRightFace )
{
	pRightFace=_pRightFace;
}

void QMeshEdge::SetMeshPatchPtr(QMeshPatch* _mesh)
{
	meshSurface=_mesh;
}

QMeshPatch* QMeshEdge::GetMeshPatchPtr()
{
	return meshSurface;
}

GLKObList& QMeshEdge::GetFaceList()
{
	return FaceList;
}
