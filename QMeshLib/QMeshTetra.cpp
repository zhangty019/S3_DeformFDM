#include "StdAfx.h"
#include "QMeshTetra.h"
#include "QMeshFace.h"
#include "QMeshEdge.h"
#include "QMeshNode.h"

#include "../GLKLib/GLKGeometry.h"
#include <math.h>

QMeshTetra::QMeshTetra(void)
{
	//type = 4;
	meshSurface=NULL;
	indexno=0;
	selected=false; inner=true;
	m_nIdentifiedPatchIndex = -1;
	for (int i=0; i<4; i++) faces[i]=NULL;
	//for(int i=0;i<8;i++) flags[i]=FALSE;
	attachedList.RemoveAll();
	Pos=NULL;
}

QMeshTetra::~QMeshTetra(void)
{
}

int QMeshTetra::GetIndexNo(){
	return indexno;
}

void QMeshTetra::SetIndexNo( const int _index ){
	indexno = _index;
}

void QMeshTetra::SetMeshSurfacePtr(QMeshPatch* _mesh){
	meshSurface=_mesh;
}

QMeshPatch* QMeshTetra::GetMeshSurfacePtr(){
	return meshSurface;
}


QMeshFace * QMeshTetra::GetFaceRecordPtr( const int whichFace ){
	return faces[whichFace-1];
}

void QMeshTetra::SetFaceRecordPtr( const int whichFace, QMeshFace * _face ){
	faces[whichFace-1] = _face;
}

QMeshEdge * QMeshTetra::GetEdgeRecordPtr( const int whichEdge ){
	switch (whichEdge){
		case 1: case 2: case 3:
			return faces[0]->GetEdgeRecordPtr(whichEdge);
		case 4: case 5: case 6:
			{
				QMeshNode *n1 = this->GetNodeRecordPtr(whichEdge-3);
				QMeshNode *n4 = this->GetNodeRecordPtr(4);
				for (int k=1; k<=3; k++)
				for (int i=1; i<=3; i++){
					QMeshEdge *edge = faces[k]->GetEdgeRecordPtr(i);
					if ((edge->GetStartPoint()==n1 && edge->GetEndPoint()==n4) || (edge->GetStartPoint()==n4 && edge->GetEndPoint()==n1))
						return edge;
				}
			}
		default:
			return NULL;
	}
}

int QMeshTetra::GetFaceIndex(QMeshFace* face){
	for (int i=1; i<=4; i++)
		if (GetFaceRecordPtr(i)==face) return i;
	return -1;
}

int QMeshTetra::GetEdgeIndex(QMeshEdge* edge){
	for (int i=1; i<=6; i++)
		if (GetEdgeRecordPtr(i)==edge) return i;
	return -1;
}

bool QMeshTetra::IsFixed()
{
	if (GetNodeRecordPtr(1)->GetIndexNo() > -1 ||
		GetNodeRecordPtr(2)->GetIndexNo() > -1 ||
		GetNodeRecordPtr(3)->GetIndexNo() > -1 ||
		GetNodeRecordPtr(4)->GetIndexNo() > -1) return false;
	else return true;
}

bool QMeshTetra::IsInner()
{
	bool a = true;
	/*for (int i = 0; i < 6; i++) {
		if (GetEdgeRecordPtr(i + 1)->IsBoundaryEdge() == true) {
			a = false; break;
		}
	}*/

	for (int i = 0; i < 4; i++) {
		if (GetFaceRecordPtr(i + 1)->GetLeftTetra() == false || 
			GetFaceRecordPtr(i + 1)->GetRightTetra() == false) {
			a = false; break;
		}
	}

	return a;
}

//These are all computing with single Tetra
bool QMeshTetra::IsNormalDirection( const int whichFace )
{
	//ASSERT( (whichEdge==1) || (whichEdge==2) || (whichEdge==3) );
	int bitNumber = whichFace - 1;
	return isNormal[bitNumber];
}

void QMeshTetra::SetDirectionFlag( const int whichEdge, const int toBe )
{
	//ASSERT( (whichEdge==1) || (whichEdge==2) || (whichEdge==3) );
	int bitNumber = whichEdge - 1;
	isNormal[bitNumber]=toBe;
}

//BOOL QMeshTetra::GetAttribFlag( const int whichBit )
//{
//	return flags[whichBit];
//}
//
//void QMeshTetra::SetAttribFlag( const int whichBit, const BOOL toBe )
//{
//	flags[whichBit] = toBe;
//}

void QMeshTetra::CalCenterPos(double &xx, double &yy, double &zz)
{
	double pp[3];
	xx = 0.0; yy = 0.0; zz = 0.0;

	for (int i=0;i<4;i++) {
		GetNodePos(i+1,pp[0],pp[1],pp[2]);
		xx=xx+pp[0];	yy=yy+pp[1];	zz=zz+pp[2];
	}
	xx=xx/(double)4;	yy=yy/(double)4;	zz=zz/(double)4;	
}

double QMeshTetra::CalVolume(double t[4][3]){
	double a[3], b[3], c[3];
	for (int i=0; i<3; i++){
		a[i] = t[1][i]-t[0][i];
		b[i] = t[2][i]-t[0][i];
		c[i] = t[3][i]-t[0][i];
	}

	double area[3];
	CROSS(area, c, b);

	volume = DOT(a,area);

	if (volume<0){
		int x=0;
		volume = -volume;
	}

	return volume;
}

double QMeshTetra::CalVolume(){
	double t[4][3];
	for (int i=0; i<4; i++)
		GetNodePos(i+1, t[i][0], t[i][1], t[i][2]);
	return CalVolume(t);
}

double QMeshTetra::CalVolume_Last() {
	double t[4][3];
	for (int i = 0; i < 4; i++) {
		QMeshNode *Node = GetNodeRecordPtr(i+1);
		Node->GetCoord3D_last(t[i][0], t[i][1], t[i][2]);
	}
	return CalVolume(t);
}

double QMeshTetra::GetVolume(){
	return volume;
}

double QMeshTetra::CalSolidAngle(int whichNode){
	if (whichNode<=0 || whichNode>4) {printf("index range from 1 to 4!\n"); return -1;}
	double p[3][3];
	double pp[3];
	GetNodePos(whichNode, pp[0], pp[1], pp[2]);
	for (int i=1, j=0; i<=4; i++){
		if (i==whichNode) continue;
		GetNodePos(i, p[j][0], p[j][1], p[j][2]);
		j++;
	}

	return CalSolidAngle(p, pp);
}

double QMeshTetra::CalSolidAngle(double p[3][3], double pp[3]){
	
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			p[i][j]-=pp[j];

	double length[3]={0,0,0};
	for (int i=0; i<3; i++){
		length[i] = p[i][0]*p[i][0]+p[i][1]*p[i][1]+p[i][2]*p[i][2];
		length[i] = sqrt(length[i]);
	}

	//L'Huilier's theorem
	//double thetaA = acos(abs(DOT(p[0], p[1])/(length[0]*length[1])));
	//double thetaB = acos(abs(DOT(p[0], p[2])/(length[0]*length[2])));
	//double thetaC = acos(abs(DOT(p[2], p[1])/(length[2]*length[1])));

	//double thetaS = (thetaA+thetaB+thetaC)/2.0;

	//double O4 = sqrt(tan(thetaS/2.0)*tan((thetaS-thetaA)/2.0)*tan((thetaS-thetaB)/2.0)*tan((thetaS-thetaC)/2.0));
	//double Omega = 4.0*atan(O4);

/////////////////////////////////////////////////////////////////////////////////////////////
	double dest[3];
	CROSS(dest, p[1], p[2]);
	double det = fabs(DOT(p[0], dest));

	double div = length[0]*length[1]*length[2] + DOT(p[0], p[1])*length[2] + DOT(p[0], p[2])*length[1] + DOT(p[1], p[2])*length[0];

	double at = atan2(det, div);
	if (at<0) at += PI;

	double Omega = 2*at;
	//if (omega!=Omega){
	//	printf("omega!=Omega %.17f %.17f\n", omega, Omega);
	//}
/////////////////////////////////////////////////////////////////////////////////////////////

	if (Omega>2*PI)
		printf("ERROR: Omega=%.17f\n", Omega);

	return Omega;
}

bool QMeshTetra::CalTetraBarycentry(double p[3], double &_p, double &_q, double &_r, double &_s){
	double t[4][3];
	for (int i=0; i<4; i++)
		GetNodePos(i+1, t[i][0], t[i][1], t[i][2]);
	return CalTetraBarycentry(p, t, _p, _q, _r, _s);
}

bool QMeshTetra::CalTetraBarycentry(double p[3], double t[4][3], double &_p, double &_q, double &_r, double &_s){
	GLKGeometry geo;

	int index[5][4]={
		{1,3,2,4}, {1,3,2,4}, {1,4,3,2}, {1,2,4,3}, {2,3,4,1}
	};

	double d[5];
	for (int i=0; i<5; i++){
		double a[16];
		for (int m=0; m<4; m++){
			for (int n=0; n<3; n++)
				a[m*4+n]=t[index[i][m]-1][n];
			a[m*4+3]=1;
		}
		if (i>0){
			for (int n=0; n<3; n++)
				a[12+n]=p[n];
		}
		d[i]=geo.Determinant4(a);
	}
	if (d[0]==0) {
		return false;
	}

	for (int i=1; i<=4; i++){
		d[i]/=d[0];
	}

	_p = d[4]; _q = d[2]; _r = d[3]; _s = 1.0-_p-_q-_r;

	for (int i=1; i<=4; i++){
		if (d[i]<0){
			if (fabs(d[i])<1e-5) continue;
			return false;
		}
		if (d[i]>1+1e-5) return false;
	}

	if (_p>1) _p=1;
	if (_p<0) _p=0;

	if (_q>1) _q=1;
	if (_q<0) _q=0;

	if (_r>1) _r=1;
	if (_r<0) _r=0;

	_s = 1.0-_p-_q-_r;

	return true;
}

void QMeshTetra::BarycentryToPosition(double _p, double _q, double _r, double _s, double t[4][3], double p[3]){
	for (int i=0; i<3; i++)
		p[i] = _p * t[0][i] + _q * t[1][i] + _r * t[2][i] + _s * t[3][i];
}

void QMeshTetra::BarycentryToPosition(double _p, double _q, double _r, double _s, double p[3]){
	double t[4][3];
	for (int i=0; i<4; i++)
		GetNodePos(i+1, t[i][0], t[i][1], t[i][2]);
	BarycentryToPosition(_p, _q, _r, _s, t, p);
}

//These are all relationship between Tetra and Node
//void QMeshTetra::AddNode(QMeshNode *_node) {
//	nodeList.AddTail(_node);
//}

//int QMeshTetra::GetNodeNumber()
//{
//	return nodeList.GetCount();
//}

//QMeshNode* QMeshTetra::GetNodeRecordPtr(int No)	//from 1 to n
//{
//	return meshSurface->GetNodeRecordPtr(tet_node_index[No - 1] + 1);
//	/*if ((No < 1) || (No > nodeList.GetCount())) return  nullptr;
//	return (QMeshNode *)nodeList.GetAt(nodeList.FindIndex(No - 1));*/
//}

//index start from 1
QMeshNode * QMeshTetra::GetNodeRecordPtr(int whichNode) {
	switch (whichNode) {
	case 1: case 2: case 3:
		if (!isNormal[0]) whichNode = 4 - whichNode;
		return faces[0]->GetNodeRecordPtr(whichNode-1);
	case 4:
		for (int i = 1; i <= 3; i++) {
			bool exist = false;
			for (int j = 1; j <= 3; j++)
				if (faces[1]->GetNodeRecordPtr(i-1) == this->GetNodeRecordPtr(j))
					exist = true;
			if (!exist) return faces[1]->GetNodeRecordPtr(i-1);
		}
	default:
		printf("QMeshTetra::GetNodeRecordPtr( UINT whichNode ) ERROR: node not found %d\n", whichNode);
		return NULL;
	}
}

void QMeshTetra::GetNodePos(const int whichNode, double &xx, double &yy, double &zz) {
	this->GetNodeRecordPtr(whichNode)->GetCoord3D(xx, yy, zz);
}

int QMeshTetra::GetNodeIndex(QMeshNode* node) {
	for (int i = 1; i <= 4; i++)
		if (GetNodeRecordPtr(i) == node) return i;
	return -1;
}