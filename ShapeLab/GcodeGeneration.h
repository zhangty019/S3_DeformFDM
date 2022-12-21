#pragma once
#include <iomanip>
#include <vector>

#include "../QMeshLib/PolygenMesh.h"
#include "../QMeshLib/QMeshPatch.h"

struct collision_Node {
	double B_value;
	double C_value;
	QMeshNode* waypoint_N_i;
};

class GcodeGeneration {

public:
	GcodeGeneration() {};
	~GcodeGeneration() {};

	void initial(PolygenMesh* Slices, PolygenMesh* Waypoints,
		PolygenMesh* CncPart, bool Yup2Zup, std::string Dir,
		double Xmove, double Ymove, double Zmove, double toolLength,
		std::string modelName);
	int read_layer_toolpath_cnc_files();
	void updateParameter(int FromIndex, int ToIndex, double lambdaValue,
		bool varyDistance_switch, bool varyHeight_switch,
		bool varyWidth_switch, bool outputDHW);
	void calDHW();
	void singularityOpt();
	void singularityOpt_newConfig();
	void cal_XYZBCE_test();
	void detectCollision_2();
	void graph_Search_Shortest_Path();
	void graph_Search_Shortest_Path_newConfig();
	void testXYZBCE(bool testXYZBC_E_switch);
	void feedrateOpt();
	void writeGcode(std::string GcodeDir);
	void writeGcode_continousPrint(std::string GcodeDir);
	void readGcodeFile(
		Eigen::MatrixXf& Gcode_Table, std::string FileName);
	void readGcodeFile_newConfig(
		Eigen::MatrixXf& Gcode_Table, std::string FileName);

private:
	void _getFileName_Set(std::string dirctory,
		std::vector<std::string>& fileNameCell);
	void _modifyCoord(QMeshPatch* patchFile, bool Yup2Zup);
	void _readWayPointData(std::string packName);
	void _readSliceData(std::string packName);
	void _readCncData(int input_Step);

	int _remove_allFile_in_Dir(std::string dirPath);
	void _cal_Dist();
	void _initialSmooth(int loopTime);
	void _cal_Height();
	void _cal_Width();
	void _output_DHW();

	std::vector<QMeshPatch*> _getJumpSection_patchSet(QMeshPatch* patch);
	double _safe_acos(double value);
	void _markSingularNode(QMeshPatch* patch);
	void _filterSingleSingularNode(QMeshPatch* patch);
	void _getSingularSec(QMeshPatch* patch, Eigen::MatrixXd& sectionTable);
	void _projectAnchorPoint(QMeshPatch* patch);
	void _getBCtable2(QMeshPatch* patch,
		Eigen::MatrixXd& B1C1table, Eigen::MatrixXd& B2C2table);
	void _getBCtable2_newConfig(QMeshPatch* patch,
		Eigen::MatrixXd& B1C1table, Eigen::MatrixXd& B2C2table);
	void _motionPlanning3(QMeshPatch* patch, 
		const Eigen::MatrixXd& sectionTable, const Eigen::MatrixXd& B1C1table,
		const Eigen::MatrixXd& B2C2table, Eigen::RowVector2d& prevBC);
	bool _chooseB1C1(const Eigen::RowVector2d& B1C1,
		const Eigen::RowVector2d& B2C2, Eigen::RowVector2d& prevBC);
	double _toLeft(const Eigen::RowVector2d& origin, const Eigen::RowVector2d& startPnt, const Eigen::RowVector2d& endPnt);
	void _getXYZ(QMeshPatch* patch);
	void _getXYZ_newConfig(QMeshPatch* patch);
	void _calDHW2E(QMeshPatch* patch, bool hysteresis_switch);
	void _optimizationC(QMeshPatch* patch);
	void _limit_C_range(QMeshPatch* patch);
	void _verifyPosNor();
	void _verifyPosNor_newConfig();
	double _getAngle3D(const Eigen::Vector3d& v1,
		const Eigen::Vector3d& v2, const bool in_degree);

	void _locate_EHead_printPos(QMeshPatch* eHeadPatch,
		Eigen::Vector3d nodePos, Eigen::Vector3d norm);
	bool _isPnt_in_mesh(QMeshPatch* surfaceMesh, Eigen::Vector3d node_coord3D);
	bool IntersectTriangle(const Eigen::Vector3d& orig, 
		const Eigen::Vector3d& dir,	Eigen::Vector3d& v0, 
		Eigen::Vector3d& v1, Eigen::Vector3d& v2);
	bool _isPnt_in_mesh2(QMeshPatch* surfaceMesh, Eigen::Vector3d node_coord3D);

	void _get_GraphNode_List(QMeshPatch* patch,
		std::vector<collision_Node>& graph_Node);
	void _get_GraphNode_List_newConfig(QMeshPatch* patch, 
		std::vector<collision_Node>& graph_Node);
	Eigen::Vector3d _calCandidateNormal(Eigen::Vector3d normal,
		double rad_ZrotateAngle, double rad_XtiltAngle);
	void _install_BC(Eigen::Vector3d temp_Normal,
		std::vector<collision_Node>& graph_Node, QMeshNode* sourceNode);
	void _install_BC_newConfig(Eigen::Vector3d temp_Normal,
		std::vector<collision_Node>& graph_Node, QMeshNode* sourceNode);
	void _build_Graph(QMeshPatch* patch, std::vector<collision_Node>& graph_Node);
	void _build_Graph_newConfig(QMeshPatch* patch, std::vector<collision_Node>& graph_Node);
	double _weight_calculation(double B_i, double C_i, double B_ii, double C_ii);

public:

private:
	PolygenMesh* m_Slices = NULL;
	PolygenMesh* m_Waypoints = NULL;
	PolygenMesh* m_CncPart = NULL;

	bool m_Yup2Zup = false;
	std::string m_Dir;
	double m_Xmove = 0.0;
	double m_Ymove = 0.0;
	double m_Zmove = 0.0;

	std::vector<std::string> wayPointFileCell;// Waypoints Dir Files
	std::vector<std::string> sliceSetFileCell;// Layers Dir Files

	int m_FromIndex = 0;
	int m_ToIndex = 0;

	double m_lambdaValue = 0.0;

	bool m_varyDistance_switch = true;
	bool m_varyHeight_switch = false;
	bool m_varyWidth_switch = false;
	bool m_outputDHW = false;

	int Core = 16;		// My CPU core NUM
	int layerNum = 40;	// The number of detected bottom layer for height calculation
	int layerDepth = 40; // Collision detecte depth // for calibration can choose 1
	int delta_Z = 30, delta_X = 10; // The delta of candidate collision-free Normals
	double h = 0.0;     // tool length
	double r = 63.75;   // C head rotation radius 

	double maxDeltaC = 5.0;   // the max angle of C change when use inserting method
	int toolpath_filter_threshold = 30; //skip little patch

	std::string m_modelName;

	//switch for the planar printing
	bool is_planar_printing = false; 
};