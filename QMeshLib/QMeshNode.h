// QMeshNode.h: interface for the QMeshNode class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _QMESHNODE
#define _QMESHNODE

#include "../GLKLib/GLKObList.h"
#include <vector>

class QMeshPatch;
class QMeshFace;
class QMeshEdge;
class QMeshTetra;

class QMeshNode : public GLKObject  
{
public:
	QMeshNode();
	virtual ~QMeshNode();

public:
	int GetIndexNo();		//from 1 to n
	void SetIndexNo( const int _index = 1 );

	bool GetAttribFlag( const int whichBit );
	void SetAttribFlag( const int whichBit, const bool toBe = true );

	void GetCoord2D( double &x, double &y );
	void SetCoord2D( double x, double y );

	void GetCoord3D( double &x, double &y, double &z );
	void SetCoord3D( double x, double y, double z );

    void GetCoord3D_last( double &x, double &y, double &z );
	void SetCoord3D_last( double x, double y, double z );

	void SetMeanCurvatureNormalVector(double kHx, double kHy, double kHz);
	void GetMeanCurvatureNormalVector(double &kHx, double &kHy, double &kHz);
	
	void SetGaussianCurvature(double kG);
	double GetGaussianCurvature();
	
	void SetPMaxCurvature(double k1);
	double GetPMaxCurvature();

	void SetPMinCurvature(double k2);
	double GetPMinCurvature();

    void CalNormal();
    void CalNormal(double normal[]);
    void GetNormal(double &nx, double &ny, double &nz) {nx=m_normal[0]; ny=m_normal[1]; nz=m_normal[2];};
    void SetNormal(double nx, double ny, double nz) {m_normal[0]=nx; m_normal[1]=ny; m_normal[2]=nz;};

    void SetBoundaryDis(double dist);
    double GetBoundaryDis();

    void SetDensityFuncValue(double density) {m_densityFuncValue=density;};
    double GetDensityFuncValue() {return m_densityFuncValue;};

	void SetMeshPatchPtr(QMeshPatch* _mesh);
	QMeshPatch* GetMeshPatchPtr();

	void AddTetra(QMeshTetra *trglTetra);
	int GetTetraNumber();
	QMeshTetra* GetTetraRecordPtr(int No);	//from 1 to n
	GLKObList& GetTetraList();

	void AddFace(QMeshFace *_face);
	int GetFaceNumber();
	QMeshFace* GetFaceRecordPtr(int No);	//from 1 to n
    GLKObList& GetFaceList();

	void AddEdge(QMeshEdge *_edge);
	int GetEdgeNumber();
	QMeshEdge* GetEdgeRecordPtr(int No);	//from 1 to n
    GLKObList& GetEdgeList();

	void AddNode(QMeshNode *_node);
	int GetNodeNumber();
	QMeshNode* GetNodeRecordPtr(int No);	//from 1 to n
    GLKObList& GetNodeList();
	bool IsNodeInNodeList(QMeshNode *_node);

    void SetMinCurvatureVector(double vx, double vy, double vz);
    void GetMinCurvatureVector(double &vx, double &vy, double &vz);

    void SetMaxCurvatureVector(double vx, double vy, double vz);
    void GetMaxCurvatureVector(double &vx, double &vy, double &vz);

    void SetWeight(double weight) {m_weight=weight;};
    double GetWeight() {return m_weight;};

    void SetColor(float r, float g, float b) {m_rgb[0]=r; m_rgb[1]=g; m_rgb[2]=b;};
    void GetColor(float &r, float &g, float &b) {r=m_rgb[0]; g=m_rgb[1]; b=m_rgb[2];};

	GLKObList attachedList;

    double m_trackingPos[3];	int m_collideConstraintIndex;
    QMeshFace* m_trackingFace;

    void *attachedPointer, *attachedPointer1;

    int m_nIdentifiedPatchIndex = -1;
	bool selected = false;
	bool selectedforEdgeSelection;
	bool selectedforFaceSelection;
    bool boundary;
    bool boundary1,boundary2;
    bool active;
    bool visited;
    int featurePt;
    bool m_sepFlag;
	bool isFixed = false;
	bool isHandle = false; // used for surface protection case

    double value1;
    double value2;

	double Alpha = 0; //For TopOpt Filter Variable

    double U,V,W;

    void GetCoord3D_FLP( double &x, double &y, double &z );
    void SetCoord3D_FLP( double x, double y, double z );

    void GetCoord3D_Laplacian( double &x, double &y, double &z ) {x=coord3D_Laplacian[0]; y=coord3D_Laplacian[1]; z=coord3D_Laplacian[2];};
    void SetCoord3D_Laplacian( double x, double y, double z ) {coord3D_Laplacian[0]=x; coord3D_Laplacian[1]=y; coord3D_Laplacian[2]=z;};

    void SetMixedArea(double area) {m_mixedArea=area;};

    int TempIndex; // for remeshing
    int identifiedIndex;
	bool inner, i_inner;

private:
	int indexno;
	bool flags[8];
				// bit 0 -- True for boundary points
				// bit 1 -- True for points on coarse mesh
				// bit 2 -- True for points on interpolation curve 
				// bit 3 -- True for points on hand (temp use) (or points adjacent to boundary face)
				// bit 4 -- True for points on arm (temp use) (or branch vertices)
				// bit 5 -- True for sharp-feature vertex (or vertex cannot be moved)
				// bit 6 -- True for sharp-feature-region vertex
				// bit 7 -- True for points moved (temp use or newly created)
	double coord2D[2];
				// 2D space coordinate
	double coord3D[3];
				// 3D space coordinate
	double coord3D_last[3];  // just for reset sewing operation for one step
                // 3D space coordinate in the last sewing step
    double coord3D_Laplacian[3];

    double m_meanCurvatureNormalVector[3], m_gaussianCurvature, m_pMaxCurvature, m_pMinCurvature;
    double m_minCurvatureVector[3], m_maxCurvatureVector[3];
    double m_boundaryDist, m_densityFuncValue;
    double m_mixedArea;

	QMeshPatch *meshSurface;		// QMesh contains this node

	GLKObList faceList;	// a list of faces contained this node (QMeshFace)
	GLKObList edgeList;	// a list of edges contained this node (QMeshEdge)
	GLKObList nodeList;	// a list of nodes coincident with this node (QMeshNode)
	GLKObList tetraList;	// a list of nodes coincident with this node (QMeshNode)

    double m_normal[3];
    double m_weight;
    double coord3D_FLP[3]; //For Keep the FLProcessData

    float m_rgb[3];
public:
	void GetNormal_last(double& nx, double& ny, double& nz);
	void SetNormal_last(double nx, double ny, double nz);

private:
	double m_normal_last[3];

// variables for S^3 DeformFDM
public:
	Eigen::Vector3d initial_coord3D = Eigen::Vector3d::Zero();
	Eigen::Vector3d deformed_coord3D = Eigen::Vector3d::Zero();
	bool isBottomNode = false;
	double scalarField = 0.0;//scalar field computing
	double scalarField_init = 0.0;
	double growingField_4voxelOrder = 0.0;
	QMeshEdge* relatedTetEdge = NULL;// for record the node of iso-layers on which tet edge;
	bool isoSurfaceBoundary = false; //curved layer slicing
	double nodePlaneDis; // parameter used in plane cutting
	bool planeCutNewNode = false;
	bool isSurfaceProtectNode = false;
	bool isSurfaceProtectNode_temp = false;
	int surfaceKeep_region_idx = 0; //this node belongs to which surfaceKeep region
	int surfaceKeepIndex; // for the equation construction of HardConstrain Ax=b

	bool isBlayerOffsetCoverAera = true;
	double layerThickDistance = 1.0;
	int cutNode_index = -1;

	int splitIndex;		 //detect which splited patch this node belongs to
	int splitPatchIndex; //the index number of this node in splited patch

	bool isCollisionHappenNode = false;

	// variables for support generation
	bool need_Support = false; //Node on init Layer needs support
	Eigen::Vector3d supportRay_Dir = { 0.0,0.0,0.0 };
	
	std::vector<Eigen::Vector3d> polyline_node;// store the polyline node for support generation
	bool isOringin = false;

	int Volume2Surface_Ind = 0;
	bool is_tetSupportNode = true;// keep the initial value as true;
	int hollow_supportTet_Ind = 0;
	bool model_boundary = false;	
	int supportIndex = -1;

	bool is_Host = false;
	bool is_Processed = false;
	bool is_virtual_Host = false;
	QMeshFace* treeNode_belonging_Face = NULL;
	Eigen::Vector3d descend_To_TreeNode_Dir = { 0.0,-1.0,0.0 };		//-->treeNode
	Eigen::Vector3d descend_From_TreeNode_Dir = { 0.0,-1.0,0.0 };	//treeNode-->
	int treeNode_before_branch_NUM = 0;

	// field value for cuting support layers with impelicity surface
	double implicitSurface_value = -INFINITY;
	// node index reorder for cutted support surface reconstruction
	double implicitSurface_cut_index = -1;
	// cutNode installed on which edge
	QMeshEdge* cutNode_related_LayerEdge = NULL; 

	double geoFieldValue; //for tool-path generation, used by heat method
	int heatmethodIndex;  //for tool-path generation, heat method computing
	double dualArea();
	double boundaryValue; //for tool-path generation
	QMeshEdge* relatedLayerEdge = NULL; //for tool-path generation, iso-Node on which layer edge
	bool connectTPathProcessed; //for tool-path connection, indicate has been linked
	double isoValue = -1.0;//for tool-path connection, indicate isoNode's isoValue
	bool resampleChecked = false; //for tool-path connection, indicate whether the Node will be selected after resampling

	double zigzagValue;   //for tool-path generation: zigzag distance field
	bool isZigzagNode = false;
	bool isZigzag_sideNode = false;

	//Gcode generation
	Eigen::Vector3d m_printNor = Eigen::Vector3d::Zero();
	Eigen::Vector3d m_printNor_4_robot = Eigen::Vector3d::Zero();
	Eigen::Vector3d m_printPos = Eigen::Vector3d::Zero();
	Eigen::Vector3d m_DHW = Eigen::Vector3d::Ones();
	Eigen::MatrixXd m_XYZBCE = Eigen::MatrixXd::Zero(1, 6);
	double m_F = 0.0;
	bool Jump_nextSecStart = false;	bool Jump_preSecEnd = false;
	bool is_PiFlip_JumpEnd = false;
	int  Jump_SecIndex; // NodeIndex in JumpPatch
	bool isSingularNode = false;// Singular node
	bool isSingularSecStartNode = false; bool isSingularSecEndNode = false;
	int solveSeclct = 0; // for solve choose // 1 -> solve 1 // 2 -> solve 2

	int insertNodesNum = 0; // indicate the inserted node Num and start point of Insert action
	std::vector<Eigen::MatrixXd> insertNodesInfo; // element Eigen::MatrixXd::Zero(1, 6);
	bool isCollision = false; bool iscollided = false;

	int FieldIndexNumber = -1;
	bool visited_forWeight = false;
	bool visited_forVol = false;
	bool isSource = false;
	bool isBoundary = false;
	double HeatFieldValue = -1.0;

	Eigen::Vector3d each_4nodes_newCoord3D = Eigen::Vector3d::Zero(); // for discrete tet plot
	bool isDiscreteBoundary = false;
	int discreteBoundary_ind = 0;

	double m_icpDist; // icp distance

	bool specialDraw = false;

};

#endif
