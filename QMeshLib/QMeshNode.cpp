// QMeshNode.cpp: implementation of the QMeshNode class.
//
//////////////////////////////////////////////////////////////////////

#include <math.h>
#include "QMeshPatch.h"
#include "QMeshTetra.h"
#include "QMeshFace.h"
#include "QMeshEdge.h"
#include "QMeshNode.h"
#include <iostream>

#define PI		3.141592654
#define DEGREE_TO_ROTATE(x)		0.0174532922222*x
#define ROTATE_TO_DEGREE(x)		57.295780490443*x


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

QMeshNode::QMeshNode()
{
    indexno=0;		m_trackingFace=nullptr;
    m_meanCurvatureNormalVector[0]=0.0;
    m_meanCurvatureNormalVector[1]=0.0;
    m_meanCurvatureNormalVector[2]=0.0;
    m_gaussianCurvature=0.0;	m_pMaxCurvature=0.0;	m_pMinCurvature=0.0;
    m_boundaryDist=0.0;
	faceList.RemoveAll();
	edgeList.RemoveAll();
	nodeList.RemoveAll();
	for(int i=0;i<8;i++) flags[i]=false;

    m_trackingPos[0]=0.0;
    m_trackingPos[1]=0.0;
    m_trackingPos[2]=0.0;
    attachedPointer=nullptr;
}

QMeshNode::~QMeshNode()
{
	faceList.RemoveAll();
	edgeList.RemoveAll();
	nodeList.RemoveAll();
}

//////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////

int QMeshNode::GetIndexNo() 
{
	return indexno;
}
	
void QMeshNode::SetIndexNo( const int _index )
{
	indexno=_index;
}

bool QMeshNode::GetAttribFlag( const int whichBit )
{
	return flags[whichBit];
}

void QMeshNode::SetAttribFlag( const int whichBit, const bool toBe )
{
	flags[whichBit]=toBe;
}

void QMeshNode::GetCoord2D( double &x, double &y )
{
	x=coord2D[0];	y=coord2D[1];
}

void QMeshNode::SetCoord2D( double x, double y )
{
	coord2D[0]=x;	coord2D[1]=y;
}

void QMeshNode::GetCoord3D( double &x, double &y, double &z )
{
	x=coord3D[0];	y=coord3D[1];	z=coord3D[2];
}

void QMeshNode::SetCoord3D( double x, double y, double z )
{
	coord3D[0]=x;	coord3D[1]=y;	coord3D[2]=z;
}

void QMeshNode::GetCoord3D_last( double &x, double &y, double &z )
{
	x=coord3D_last[0];	y=coord3D_last[1];	z=coord3D_last[2];
}

void QMeshNode::SetCoord3D_last( double x, double y, double z )
{
	coord3D_last[0]=x;	coord3D_last[1]=y;	coord3D_last[2]=z;
}

void QMeshNode::GetNormal_last(double& nx, double& ny, double& nz)
{
    nx = m_normal_last[0];	ny = m_normal_last[1];	nz = m_normal_last[2];
}

void QMeshNode::SetNormal_last(double nx, double ny, double nz)
{
    m_normal_last[0] = nx;	m_normal_last[1] = ny;	m_normal_last[2] = nz;
}


void QMeshNode::SetMeanCurvatureNormalVector(double kHx, double kHy, double kHz)
{
    m_meanCurvatureNormalVector[0]=kHx;
    m_meanCurvatureNormalVector[1]=kHy;
    m_meanCurvatureNormalVector[2]=kHz;
}

void QMeshNode::GetMeanCurvatureNormalVector(double &kHx, double &kHy, double &kHz)
{
    kHx=m_meanCurvatureNormalVector[0];
    kHy=m_meanCurvatureNormalVector[1];
    kHz=m_meanCurvatureNormalVector[2];
}

void QMeshNode::SetGaussianCurvature(double kG)
{
    m_gaussianCurvature=kG;
}

double QMeshNode::GetGaussianCurvature()
{
    return m_gaussianCurvature;
}

void QMeshNode::SetPMaxCurvature(double k1)
{
    m_pMaxCurvature=k1;
}

double QMeshNode::GetPMaxCurvature()
{
    return m_pMaxCurvature;
}

void QMeshNode::SetPMinCurvature(double k2)
{
    m_pMinCurvature=k2;
}

double QMeshNode::GetPMinCurvature()
{
    return m_pMinCurvature;
}

void QMeshNode::SetMinCurvatureVector(double vx, double vy, double vz)
{
    m_minCurvatureVector[0]=vx;	m_minCurvatureVector[1]=vy;	m_minCurvatureVector[2]=vz;
}

void QMeshNode::GetMinCurvatureVector(double &vx, double &vy, double &vz)
{
    vx=m_minCurvatureVector[0];	vy=m_minCurvatureVector[1];	vz=m_minCurvatureVector[2];
}

void QMeshNode::SetMaxCurvatureVector(double vx, double vy, double vz)
{
    m_maxCurvatureVector[0]=vx;	m_maxCurvatureVector[1]=vy;	m_maxCurvatureVector[2]=vz;
}

void QMeshNode::GetMaxCurvatureVector(double &vx, double &vy, double &vz)
{
    vx=m_maxCurvatureVector[0];	vy=m_maxCurvatureVector[1];	vz=m_maxCurvatureVector[2];
}

void QMeshNode::SetBoundaryDis(double dist)
{
    m_boundaryDist=dist;
}

double QMeshNode::GetBoundaryDis()
{
    return m_boundaryDist;
}

void QMeshNode::CalNormal()
{
    double nx, ny, nz, tt;
    nx=0.0;	ny=0.0;	nz=0.0;

    GLKPOSITION Pos;
    for(Pos=faceList.GetHeadPosition();Pos!=nullptr;)
    {
        double a,b,c,d;
        QMeshFace *temp=(QMeshFace *)(faceList.GetNext(Pos));
		if (temp->inner == true) continue;
        temp->GetPlaneEquation(a,b,c,d);
        nx+=a;	ny+=b;	nz+=c;
    }
    tt=nx*nx+ny*ny+nz*nz;
    tt=sqrt(tt);

    m_normal[0]=(double)(nx/tt);	m_normal[1]=(double)(ny/tt);	m_normal[2]=(double)(nz/tt);

	// this is used for debuging
	// the reason of NAN is inner flag of faces of layers should be initialized as false!;
	/*if (m_normal[0] != m_normal[0]) {

		int face_ind = 0;
		for (Pos = faceList.GetHeadPosition(); Pos != nullptr;)
		{
			double a, b, c, d;
			QMeshFace* temp = (QMeshFace*)(faceList.GetNext(Pos));
			if (temp->inner == true) continue;
			temp->GetPlaneEquation(a, b, c, d);
			std::printf("face_ind %d a %f b %f c %f d%f", face_ind, a, b, c, d);
			face_ind++;
		}


		std::cout << "tt = " << tt << std::endl;
		std::cout << " --node n: [" << nx << " " << ny << " " << nz << "]" << std::endl;
		std::cout << " --node Index: " << this->GetIndexNo()
			<< " --node normal: [" << m_normal[0] << " " << m_normal[1] << " " << m_normal[2] << "]" << std::endl;
	}*/
}

void QMeshNode::CalNormal(double normal[])
{
    double nx, ny, nz, tt;
	nx=0.0;	ny=0.0;	nz=0.0;

	GLKPOSITION Pos;
    for(Pos=faceList.GetHeadPosition();Pos!=nullptr;)
	{
		double a,b,c,d;
		QMeshFace *temp=(QMeshFace *)(faceList.GetNext(Pos));
		if (temp->inner == true) continue;
		temp->GetPlaneEquation(a,b,c,d);
		//std::cout << a << b << c << d << std::endl;
		nx+=a;	ny+=b;	nz+=c;
	}
	tt=nx*nx+ny*ny+nz*nz;
	tt=sqrt(tt);

    m_normal[0]=(double)(nx/tt);	m_normal[1]=(double)(ny/tt);	m_normal[2]=(double)(nz/tt);
    normal[0]=m_normal[0]; normal[1]=m_normal[1]; normal[2]=m_normal[2];
}

void QMeshNode::SetMeshPatchPtr(QMeshPatch* _mesh)
{
	meshSurface=_mesh;
}

QMeshPatch* QMeshNode::GetMeshPatchPtr()
{
	return meshSurface;
}

void QMeshNode::AddTetra(QMeshTetra *trglTetra)
{
	tetraList.AddTail(trglTetra);
}

int QMeshNode::GetTetraNumber()
{
	return tetraList.GetCount();
}

QMeshTetra* QMeshNode::GetTetraRecordPtr(int No) //from 1 to n
{
	if ((No < 1) || (No > tetraList.GetCount()))    return  NULL;
	return (QMeshTetra *)tetraList.GetAt(tetraList.FindIndex(No - 1));
}

GLKObList& QMeshNode::GetTetraList()
{
	return tetraList;
}

void QMeshNode::AddFace(QMeshFace *_face)
{
	faceList.AddTail(_face);
}

int QMeshNode::GetFaceNumber() 
{
	return faceList.GetCount();
}

QMeshFace* QMeshNode::GetFaceRecordPtr(int No) 	//from 1 to n
{
    if( (No < 1) || (No > faceList.GetCount()))    return  nullptr;
    return (QMeshFace *)faceList.GetAt(faceList.FindIndex(No-1));
}

GLKObList& QMeshNode::GetFaceList()
{
	return faceList;
}

void QMeshNode::AddEdge(QMeshEdge *_edge)
{
	edgeList.AddTail(_edge);
}

int QMeshNode::GetEdgeNumber() 
{
	return edgeList.GetCount();
}

QMeshEdge* QMeshNode::GetEdgeRecordPtr(int No) 	//from 1 to n
{
    if( (No < 1) || (No > edgeList.GetCount()))    return  nullptr;
    return (QMeshEdge *)edgeList.GetAt(edgeList.FindIndex(No-1));
}

GLKObList& QMeshNode::GetEdgeList()
{
	return edgeList;
}

void QMeshNode::AddNode(QMeshNode *_node)
{
	nodeList.AddTail(_node);
}

int QMeshNode::GetNodeNumber()
{
	return nodeList.GetCount();
}

bool QMeshNode::IsNodeInNodeList(QMeshNode *_node)
{
	GLKPOSITION Pos;

    for(Pos=nodeList.GetHeadPosition();Pos!=nullptr;) {
		QMeshNode *tempnode=(QMeshNode *)(nodeList.GetNext(Pos));
		if (tempnode==_node) return true;
	}

	return false;
}

QMeshNode* QMeshNode::GetNodeRecordPtr(int No)	//from 1 to n
{
    if ((No < 1) || (No > nodeList.GetCount())) return  nullptr;
    return (QMeshNode *)nodeList.GetAt(nodeList.FindIndex(No-1));
}

GLKObList& QMeshNode::GetNodeList()
{
	return nodeList;
}

void QMeshNode::GetCoord3D_FLP( double &x, double &y, double &z )
{
    x=coord3D_FLP[0];	y=coord3D_FLP[1];	z=coord3D_FLP[2];
}

void QMeshNode::SetCoord3D_FLP( double x, double y, double z )
{
   coord3D_FLP[0]=x;	coord3D_FLP[1]=y;	coord3D_FLP[2]=z;
}


//void QMeshNode::GetCoord3D(Eigen::Vector3f &printPos)
//{
//    printPos << m_printPos(0), m_printPos(1), m_printPos(2);
//}
//
//void QMeshNode::SetCoord3D(Eigen::Vector3f printPos)
//{
//    m_printPos << printPos(0), printPos(1), printPos(2);
//}
//
//void QMeshNode::GetNormal(Eigen::Vector3f& printNor)
//{
//    printNor << m_printNor(0), m_printNor(1), m_printNor(2);
//}
//
//void QMeshNode::SetNormal(Eigen::Vector3f printNor)
//{
//    m_printNor << printNor(0), printNor(1), printNor(2);
//}

double QMeshNode::dualArea() {
	double area = 0;
	GLKPOSITION Pos;
	for (Pos = faceList.GetHeadPosition(); Pos != nullptr;) {
		QMeshFace* connectFace = (QMeshFace*)(faceList.GetNext(Pos));
		area += connectFace->CalArea();
	}
	return area;
}