///////////////////////////////////////////////////////////////
////// written by ToM, modified by Guoxin
///////////////////////////////////////////////////////////////
#pragma once

#ifndef DEF_TRGLTETRA
#define DEF_TRGLTETRA

#include "../GLKLib/GLKObList.h"

class QMeshEdge;
class QMeshPatch;
class QMeshNode;
class QMeshFace;

class QMeshTetra :
	public GLKObject
{
public:
	QMeshTetra(void);
	~QMeshTetra(void);

	//           1
	//			/\
	//		 e1/ |\e4
	//		  /e3| \
	//		 /_e5___\
	//		2 \  |  / 4
	//		 e2 \|/ e6
	//           3
	//	in anti-clockwise order
	//  face1: 1,2,3
	//	face2: 2,4,3
	//	face3: 3,4,1
	//	face4: 4,2,1


	bool selected;
	bool inner; //for cross section view
	int m_nIdentifiedPatchIndex;

	GLKPOSITION Pos;

private:
	int indexno;
	QMeshPatch* meshSurface;	// MESHSURFACE contain this triangle
	QMeshFace* faces[4];		//	4 faces
	bool isNormal[4];			// is normail direction

	double volume;

	GLKObList attachedList;
	//GLKObList nodeList;

	//BOOL flags[8];

public:
	int GetIndexNo();		//from 1 to n
	void SetIndexNo(const int _index = 1);

	void SetMeshSurfacePtr(QMeshPatch* _mesh);
	QMeshPatch* GetMeshSurfacePtr();

	QMeshFace* GetFaceRecordPtr(const int whichFace);
	//void SetFaceRecordPtr( const int whichFace, QMeshFace * _face = NULL);
	void SetFaceRecordPtr(const int whichFace, QMeshFace* _face);

	int GetFaceIndex(QMeshFace* face);

	QMeshEdge* GetEdgeRecordPtr(const int whichEdge);
	int GetEdgeIndex(QMeshEdge* edge);

	bool IsFixed();
	bool IsInner();
	bool IsNormalDirection(const int whichFace);
	void SetDirectionFlag(const int whichFace, const int toBe = true);

	//BOOL GetAttribFlag( const int whichBit );
	//void SetAttribFlag( const int whichBit, const BOOL toBe = TRUE );

	GLKObList& GetAttachedList() { return attachedList; };

	void CalCenterPos(double& xx, double& yy, double& zz);
	double CalVolume(double t[4][3]);
	double CalVolume();
	double CalVolume_Last();
	double GetVolume();

	double CalSolidAngle(const int whichNode);
	double CalSolidAngle(double p[3][3], double pp[3]);

	bool CalTetraBarycentry(double p[3], double t[4][3], double& _p, double& _q, double& _r, double& _s);
	bool CalTetraBarycentry(double p[3], double& _p, double& _q, double& _r, double& _s); //find barycentry in this tetra

	void BarycentryToPosition(double _p, double _q, double _r, double _s, double t[4][3], double p[3]);
	void BarycentryToPosition(double _p, double _q, double _r, double _s, double p[3]);

	//void AddNode(QMeshNode *_node);
	//int GetNodeNumber();
	QMeshNode* GetNodeRecordPtr(int whichNode);	//from 1 to n
	//GLKObList& GetNodeList() { return nodeList; };

	void GetNodePos(const int whichNode, double& xx, double& yy, double& zz);
	//QMeshNode * GetNodeRecordPtr(int whichNode);
	int GetNodeIndex(QMeshNode* node);

	int tet_node_index[4];
	bool isChamber[4] = { false,false,false,false };

	// for S^3 DeformFDM
public:
	// store neighbor ele
	std::vector<QMeshTetra*> neighborCell;
	bool isBottomTet = false;
	bool isOverhangTet = false;
	bool isTensileorCompressSelect = false;
	bool lessNeighborFlag = false;
	Eigen::Matrix3d R_estimate = Eigen::Matrix3d::Zero(3, 3);//initial SVD
	Eigen::Quaterniond R_quaternion = Eigen::Quaterniond::Identity();
	Eigen::Vector3d scaleValue_vector = Eigen::Vector3d::Zero();
	//Stress from Abaqus analysis
	Eigen::VectorXd eleStress;  //stress tensor in format
								//eleStress(1), eleStress(4), eleStress(5),
								//eleStress(4), eleStress(2), eleStress(6),
								//eleStress(5), eleStress(6), eleStress(3)
	double sigma_max, sigma_mid, sigma_min; // principle stress absolutely value
	Eigen::Vector3d tau_max, tau_mid, tau_min; // principle stress dir
	int stressIndex; //# from 0, used for stress sorting (critical region selection)
	bool containProtectSurface = false;
	bool show_innerTet_split = false;
	float principleStressColor[3];

	bool isCollisionTetra = false;
	Eigen::MatrixXd VolumeMatrix = Eigen::MatrixXd::Zero(4, 3); // matrix used in field computing
	Eigen::Vector3d vectorField_4voxelOrder = Eigen::Vector3d::Zero();
	Eigen::Vector3d vectorField_4voxelOrder_now = Eigen::Vector3d::Zero();

	Eigen::Vector3d vectorField = Eigen::Vector3d::Zero();
	bool isIn_tetSurfaceMesh = false;
	bool isVectorSource = false;

	//int supportIndex = -1;
	int surfaceKeepIndex = -1;
	int FieldIndexNumber = -1;

	double concaveAngle = 0.0;

	// for weighting determination of surface-Preservation case (global smooth)
	// 2--> largeCover 1--> vertical growing
	int select_coverLayer = 0; 

	bool isBoundaryTet = false;
};

#endif 