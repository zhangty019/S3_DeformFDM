
// QMeshFace.h: interface for the QMeshFace class
//
//////////////////////////////////////////////////////////////////////

#ifndef _QMESHFACE
#define _QMESHFACE
#include <stddef.h>

#include "../GLKLib/GLKObList.h"

#define MAX_EDGE_NUM 10

typedef enum face_type {
	KEEP,
	CUT_2,// 2+1- get two triangles;
	CUT_1,// 1+2- get one triangle;	
	DISCARD
};

// source_Node -> ... ->[(dir: treeNode_descend_Nor -->) target_Node: treeNode_coord3D]
typedef struct supportTreeNode {
	QMeshNode* souce_Node;
	Eigen::Vector3d belong_Face_Normal;
	Eigen::Vector3d treeNode_descend_Nor;
	Eigen::Vector3d treeNode_coord3D;
	bool isHostNode = false;
	bool isProcessed = false;
}SptTreeNode;

class QMeshPatch;
class QMeshEdge;
class QMeshNode;
class QMeshTetra;

class QMeshFace : public GLKObject
{
public:
	QMeshFace();
	virtual ~QMeshFace();

public:
	bool GetAttribFlag( const int whichBit );
	void SetAttribFlag( const int whichBit, const bool toBe = true );

	int GetIndexNo();		//from 1 to n
	void SetIndexNo( const int _index = 1 );

	bool IsNormalDirection( const int whichEdge );
	void SetDirectionFlag( const int whichEdge, const bool toBe = true );

	QMeshEdge * GetEdgeRecordPtr( const int whichEdge );
    void SetEdgeRecordPtr( const int whichEdge, QMeshEdge * _edge = nullptr );
	int GetEdgeNum();
	void SetEdgeNum(int num);
		
	void GetNodePos( const int whichNode, double &xx, double &yy, double &zz );
	QMeshNode * GetNodeRecordPtr( const int whichNode );

	double GetNodeAngle( const int whichNode );
	void SetNodeAngle( const int whichNode, double angleInRadian );

	void SetColor(float r, float g, float b);
	void GetColor(float &r, float &g, float &b);

	void SetHermiteData(double pos[], double normal[]);
	void GetHermiteData(double pos[], double normal[]);

	void GetPlaneEquation( double &A, double &B, double &C, double &D );
	void CalPlaneEquation( ); 
						// to calculate the plane equation parameter
	                    // Plane equation:  Ax + By + Cz + D = 0, and
                        // Vector(A,B,C) is positive unit normal vector of this plane

	void CalCenterPos(double &xx, double &yy, double &zz);
	void CalCenterPos();
    void GetCenterPos(double &xx, double &yy, double &zz) {xx=center[0];yy=center[1];zz=center[2];};
    void SetCenterPos(double xx, double yy, double zz) {center[0]=xx; center[1]=yy; center[2]=zz;};
	void CalCenterPos_last(double &xx, double &yy, double &zz);
	void CalBoundingBox(double &xmin, double &ymin, double &zmin,
						double &xmax, double &ymax, double &zmax);
	double CalArea();
	double GetArea() {return m_area;};
	double CalArea2D();

	void SetMeshPatchPtr(QMeshPatch* _mesh);
	QMeshPatch* GetMeshPatchPtr();

    GLKObList& GetAttachedList() {return attachedList;};
	int nSplittedNode;

	void SetNormal(double nx, double ny, double nz) {abcd[0]=nx;abcd[1]=ny;abcd[2]=nz;};
    void GetNormal(double &nx, double &ny, double &nz){nx=abcd[0]; ny=abcd[1]; nz=abcd[2];};
	void SetPlane(double nx, double ny, double nz, double dd) {abcd[0]=nx;abcd[1]=ny;abcd[2]=nz;abcd[3]=dd;};


    double m_GaussArea;

	int m_nIdentifiedPatchIndex;
    bool selected;
	bool inner = false;
	bool i_inner = false;
	bool isFixedDraw = false;
	bool isHandleDraw = false;
	double weight;

	void *attachedPointer;
    bool boundary;
    bool visited;
    double m_weight;
    bool m_centroid;
    double value1;
    double value2;
    double boundaryDis;
    bool m_bndface;
    bool IsMoreWeight;
    bool visible;

	bool isIntersetwithPlane = false;

    int identifiedIndex;
	GLKObList attachedList;	// a list of attached object

	int splitIndex = -1;// for split layer output

private:
	int indexno;
	bool flags[8];
		// 0 - for boundary faces
		// 1 - for sharp-feature region faces
		// 2 - for this face need to be subdivided from center
		// 3 - for temp use
	
		// 0, 1, 2, 3, ... edges construct an anti-clockwise closed loop
                                      //**********************************
                                      //             point1              *
                                      //              /\                 *
                                      //    1 edge  /    \ _             *
                                      //         |_       |\ 0 edge      *
                                      //        /            \           *
                                      //      /                \         *
                                      //  point2 ------>-------- point0  *
	                                  //              2 edge             *
	                                  //**********************************
	QMeshEdge * edges[MAX_EDGE_NUM];	//	edges
	bool edgeDir[MAX_EDGE_NUM];			//	edge directions
	int edgeNum;
	double nodeAngle[MAX_EDGE_NUM];

	QMeshPatch *meshSurface;	// MESHSURFACE contain this triangle
	double  abcd[4];	// plane equation
	float	rgb[3];		// the color of the face
	double m_area;
    double center[3];

	double	m_HermitePos[3],m_HermiteNormal[3];


	//for volume mesh
public:
	QMeshTetra * GetLeftTetra();
	void SetLeftTetra(QMeshTetra * _pLeftTetra = NULL);
	QMeshTetra * GetRightTetra();
	void SetRightTetra(QMeshTetra * _pRightTetra = NULL);

private:
	//for volume mesh
	QMeshTetra *pLeftTetra;
	QMeshTetra *pRightTetra;

// for S^3 DeformFDM
public:
	bool isOverhangFace = false; //used for deformation part
	QMeshEdge* installedIsoEdge = nullptr;
	bool isLocatedIsoEdge = false;
	bool isQualityProtectFace = false;

	bool show_innerFace_split = false;
	bool show_scale_value = false;
	double scale_value_onFace = 0.0;

	bool show_collision = false;
	bool is_concave = false;
	double concaveAngle = 0.0;

	bool needSupport = false; // used for support generation
	bool model_boundary = false;
	Eigen::Vector3d boundaryVector = Eigen::Vector3d::Zero();

	std::vector<QMeshNode*> support_treeNode_onFace;
	std::vector<SptTreeNode> support_treeNode_cell;

	face_type faceType;

	bool isBoundaryFace();
	int getNodePtrNumber(QMeshNode* node);

	int FieldIndexNumber = -1;

	Eigen::Vector3d printingDir = Eigen::Vector3d::Zero();

	Eigen::Vector3d normal_last = Eigen::Vector3d::Zero();

	//for plot large layer rotation OR vetical growing rotation 
	// of surface-preservation case // 2-large layer / 1 band rotation
	int select_coverLayer = 0;

	bool isSpecialShow = false;
	bool isDiscreteBoundary = false;

	//for stress dir toolpath generation
	bool stessFilament_flag = false;
	Eigen::Vector3d stessFilament_vector = Eigen::Vector3d::Zero();
	std::vector<QMeshFace*> neighFace;
};

#endif
