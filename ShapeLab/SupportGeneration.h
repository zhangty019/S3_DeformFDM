#pragma once

class PolygenMesh;
class QMeshPatch;
#include "../QMeshLib/PolygenMesh.h"

typedef struct qHullSet {
	int faceNum;	double* normalVec;	double* offset;
	int vertNum;	double* vertPos;
	unsigned int* faceTable;	//	Note that: the index starts from '1'
}QHULLSET;

class SupportGeneration {

public:
	SupportGeneration() {};
	~SupportGeneration() {};

	void initial(QMeshPatch* tetPatch, QMeshPatch* platform, PolygenMesh* supportModelSet);
	void initial_Guess_SupportEnvelope();
	void collect_Support_Polyline_fromTETsurface();
	void compute_initialGuess_EnvelopeHull();
	void outputMaterial_4_envelopHullGeneration();
	void outputMaterial_4_envelopHullGeneration_plus();

	void initial(QMeshPatch* tetPatch, QMeshPatch* tetSurface, QMeshPatch* tetSupport);
	void boundary_Detection();
	void transfer_ScalarField_2_supportSpace();
	void transfer_VectorField_2_supportSpace();
	void scalarField_4_supportSpace();

	void update_supportTet_2_hollowed(QMeshPatch* support_tetPatch_hollowed);
	void organize_compatibleLayer_Index(
		PolygenMesh* compatible_isoLayerSet, int compatibleLayer_NUM);
	void output_compatibleLayer_4_remesh(PolygenMesh* compatible_isoLayerSet);
	
	void update_inputMesh_4treeSkeleton_Generation(
		QMeshPatch* tetPatch, PolygenMesh* compatible_isoLayerSet, 
		QMeshPatch* platform, PolygenMesh* supportModelSet);
	void build_support_tree();
	void compute_Support_tree_Field();
	void build_tight_supportLayers();
	void output_slimSupport_inital_4_remesh(PolygenMesh* sSupport_init_isoLayerSet);

private:
	void _moveModel_up_4_initialGuess_generation(double upHeight);
	void _movePlatform_2_modelCenter();
	void _markSupportFace();
	bool _is_single_OverHangface(QMeshFace* face);
	void _update_bottomTet_flag(double threshold, QMeshPatch* tetMesh);
	void _cal_Ray_direction(QMeshPatch* tet_Model);
	bool _find_targetNode(Eigen::Vector3d& oringinNode, Eigen::Vector3d& step_Direction,
		QMeshNode* Node, bool decline_stepDir);
	QHULLSET* _mallocMemoryConvexHull(int faceNum, int vertNum);
	void _build_ConvexHull_mesh(QHULLSET* ConvexHULL, PolygenMesh* m_supportRaySet);
	void _freeMemoryConvexHull(QHULLSET*& pConvexHull);
	void _enlarge_convexHull(QMeshPatch* convexHull_patch);
	void _output_oneSurfaceMesh(QMeshPatch* surfaceMesh, std::string path);
	void _output_combined_offMesh(QMeshPatch* remeshed_CH, QMeshPatch* tetMesh_boundary, std::string path);
	void _output_tetraMesh_boundary(QMeshPatch* tetPatch, std::string path);
	void _output_support_polyline(QMeshPatch* supportPoly_patch, std::string path);
	bool _planeCutSurfaceMesh_delete(QMeshPatch* surfaceMesh, bool Yup, double cutPlaneHeight);

	void _index_initial(QMeshPatch* patch, bool is_TetMesh);
	bool _eleCenter_in_Mesh(QMeshPatch* surfaceMesh, double qx, double qy, double qz);
	bool IntersectTriangle(const Eigen::Vector3d& orig, const Eigen::Vector3d& dir,
		Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2);
	
	double _get_scalarfield_of_NearestNode(QMeshPatch* init_tetMesh, QMeshNode* inquireNode_supportSpace);
	Eigen::Vector3d _get_vectorfield_of_NearestFace(QMeshPatch* init_tetMesh, QMeshFace* inquireFace_supportSpace);
	void _vectorField_flooding_supportSpace(int loopTime);
	void _compTetMeshVolumeMatrix(QMeshPatch* tetSupport);

	void _hollow_tetSupport_generation(QMeshPatch* support_tetPatch, std::string path);
	void _get_compatibleLayer_list(std::vector<std::vector<QMeshPatch*>>& compatibleLayer_matrix);
	int _remove_allFile_in_Dir(std::string dirPath);
	bool _intersetTriangle(Eigen::Vector3d& orig, Eigen::Vector3d& dir,
		Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2, Eigen::Vector3d& insertP);
	void _compute_descend_Dir_hostNode(QMeshNode* hostNode, bool is_downWard);
	QMeshNode* _build_host_treeNodeEdge(QMeshNode* hostNode,
		const std::vector<QMeshPatch*>& largeLayer_vector_m_1,
		QMeshPatch* supportRay_patch, int layer_height);
	bool _compute_descend_Dir_followNode(QMeshNode* followNode, QMeshNode* dropNode_hostNode);
	void _build_follow_treeNodeEdge(QMeshNode* followNode, QMeshNode* dropNode_hostNode,
		const std::vector<QMeshPatch*>& largeLayer_vector_m_1, bool is_merge,
		QMeshPatch* supportRay_patch, int layer_height);

	void _input_remeshed_compatibleLayer(PolygenMesh* compatible_isoLayerSet);
	void _natSort(std::string dirctory, std::vector<std::string>& fileNameCell);
	double _implicitDistance_Func_tree(Eigen::Vector3d& queryPnt);
	bool _lineIntersectSphere(Eigen::Vector3d& O, Eigen::Vector3d& T,
		Eigen::Vector3d& Center, double R, double& mu1, double& mu2);

public:

private:
	QMeshPatch* m_tetPatch = NULL;
	QMeshPatch*	m_platform = NULL;
	PolygenMesh* m_supportModelSet = NULL;

	QMeshPatch* m_tetSurface = NULL;
	QMeshPatch* m_tetSupport = NULL;

	PolygenMesh* m_layerSet = NULL;
	
	double tau = 45.0;					//Tet surface normal detection angle
	double m_tetMoveUp_Height = 0.0;	//platform Height
	double detectRadius_ratio = 1.5;	//detection radius ratio
	double bottomHeight_threshold = 1.0;
	double C = 1.0;
	int m_compatibleLayer_Num = -1;
	double m_bottom_Threshold = 1.0;	//the height threshold of bottom

	bool hostNode_decent_alongNormal = false; //true -> vertical
};
