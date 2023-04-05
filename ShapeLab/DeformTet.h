#pragma once
#include "../QMeshLib/PolygenMesh.h"
#include <memory>
class DeformTet
{
public:
	DeformTet();
	~DeformTet();

	// general operation for all cases
	/*void initial(PolygenMesh* polygenMesh_TetModel, int loopTime, 
		double criticalTet_weight, double neighborScale_weight, 
		double regularScale_weight, double globalSmooth_weight, int caseType);*/
	void initial(PolygenMesh* polygenMesh_TetModel,
		int outer_loopTime, int inner_loopTime,
		double criticalTet_weight_SL, double criticalTet_weight_SR,
		double criticalTet_weight_SQC, double criticalTet_weight_SQV,
		double neighborScale_weight, double regularScale_weight, 
		double globalSmooth_weight,	int caseType);

	void initial(PolygenMesh* polygenMesh_TetModel);

	void update_quaternionSmooth_weight(
		double keepWeight_SL, double keepWeight_SR,
		double keepWeight_SQC, double keepWeight_SQV);

	void find_bestPrintingOrientation(bool is_Compute);
	// preProcess for each case
	void preProcess_4SupportLess(int initialGuess_case);
	bool preProcess_4StrengthReinforcement();
	bool preProcess_4SurfaceQuality(bool fucSwitch);
	// main function of ASAP
	void runASAP_SupportLess(bool initialGuess);
	void runASAP_SupportLess_test(bool initialGuess);
	void runASAP_SupportLess_test2(bool initialGuess);
	void runASAP_SupportLess_test3();
	void runASAP_StrengthReinforcement();
	void runASAP_StrengthReinforcement_test(); //--> for testing new method

	void runASAP_SurfaceQuality();
	void runASAP_SurfaceQuality_test();
	void runASAP_SurfaceQuality_test2();

	void runASAP_Hybrid_SL_SQ();
	void runASAP_Hybrid_SL_SQ_test();
	void runASAP_Hybrid_SL_SQ_test2();

	void runASAP_Hybrid_SR_SQ();
	void runASAP_Hybrid_SR_SQ_test();

	void runASAP_Hybrid_SL_SR();

	void runASAP_Hybrid_SL_SR_SQ();
	// post process after deformation
	void postProcess();

	//
	bool preProcess_4StrengthReinforcement_stressLine();
	void runASAP_StrengthReinforcement_stressLine();
	void delete_selected_ele_stress_line();

private:
	void _index_initial(QMeshPatch* patch, bool is_TetMesh);
	void _build_tetraSet_4SpeedUp(QMeshPatch* patch);
	void _moveModelup2ZeroHeight(QMeshPatch* patch);
	void _record_initial_coord3D();
	void _record_neighbor_Tet();
	void _detect_each_neighbor_Tet(std::vector< QMeshTetra* >& TetraSet, QMeshTetra* Tetra, bool neighb_type);
	void _record_initial_normal3D();
	void _compTetMeshVolumeMatrix(QMeshPatch* patch);
	void _calculate_edgeLength(QMeshPatch* patch);
	void _detectBottomTet(double threshold);
	void _input_VoxelOrder();
	void _input_HeatField();
	void _cal_heatMethod();
	void _cal_testScalarField();
	void _detectOverhangFace();
	void _detectOverhangFace2();
	void _calFabricationEnergy_SupportLess();
	void _get_energy_innerLoop_supportLess();
	void _calFabricationEnergy_StrengthReinforcement(const std::vector<Eigen::MatrixXd>& frame_Local_initial_inverse);
	void _get_energy_innerLoop_strengthReinforcement();
	void _calFabricationEnergy_SurfaceQuality();
	void _get_energy_innerLoop_surfaceQuality();
	void _globalQuaternionSmooth1(); //keep only the quaternion of critical tet
		void _globalQuaternionSmooth1_supportLess(); // only for supportLess case
		void _globalQuaternionSmooth1_surfaceQuality(); // only for surface quality
		void _globalQuaternionSmooth1_strengthReinforcement(); // only for strength reinforcement
	void _globalQuaternionSmooth2(); //keep all of the quaternion
	void _globalQuaternionSmooth3(); //consider the concavity of rotation
	void _globalQuaternionSmooth4(); //add the weighting of surface keep region (horizontal and vertical)
	void _globalQuaternionSmooth4_withoutBottom(); // without bottom constraints
	void _cal_virtual_Vfield();
	void _concavityDetection();
	bool _cal_2FaceAngle(QMeshTetra* hostTet, QMeshFace* eachFace,
		QMeshTetra* followTet, double& angle_2face);
	int _intersect3D_SegmentPlane(
		Eigen::Vector3d Segment_P0, Eigen::Vector3d Segment_P1,
		Eigen::Vector3d Plane_V0, Eigen::Vector3d Plane_Nor,
		Eigen::Vector3d& Intersect_P);
	void _moveModel2Center();
	bool _selectSurfaceQualityProtectRegion(QMeshPatch* patch);
	void _give_All_boundary_as_ProtectRegion(QMeshPatch* patch);

	void _check_parameters();
	Eigen::Vector3d _get_initial_overhang_faceNormal(QMeshTetra* Tetra);
	
	
	
	//local rotation (support-less)
	QMeshFace* _get_initial_overhang_face(QMeshTetra* Tetra);
	Eigen::Matrix3d _cal_rotationMatrix_supportLess(Eigen::Vector3d face_Normal_Dir, QMeshTetra* Tetra);
	//local rotation (surface-quality)
	QMeshFace* _get_initial_kept_face(QMeshTetra* Tetra);
	Eigen::Matrix3d _cal_rotationMatrix_surfaceQuality(Eigen::Vector3d normal, QMeshTetra* Tetra, QMeshFace* criticalFace);
	Eigen::Matrix3d _cal_rotationMatrix_surfaceQuality(
		Eigen::Vector3d normal, QMeshTetra* Tetra, QMeshFace* criticalFace, bool update_coverFlag);
	//local rotation (strength-reinforcement)
	Eigen::Matrix3d _cal_rotationMatrix_strengthReinforcement(Eigen::Vector3d principal_Stress_Dir);
	Eigen::Matrix3d _cal_rotationMatrix_strengthReinforcement_stressLine(
		Eigen::Vector3d principal_Stress_Dir, QMeshTetra* Tetra);
	//local rotation (support-less and surface-quality)
	Eigen::Matrix3d _cal_rotationMatrix_SL_SQ(Eigen::Vector3d overhangFace_normal_new, QMeshTetra* Tetra);
	//local ratation (strength-reinforcement and surface-quality)
	Eigen::Matrix3d _cal_rotationMatrix_SR_SQ(
		Eigen::Vector3d principal_Stress_Dir, Eigen::Vector3d keptFace_normal, QMeshTetra* Tetra);
	//local ratation (support-less and strength-reinforcement)
	Eigen::Matrix3d _cal_rotationMatrix_SL_SR(
		Eigen::Vector3d overhangFace_normal, Eigen::Vector3d principal_Stress_Dir, QMeshTetra* Tetra);
	Eigen::Matrix3d _cal_rotationMatrix_SL_SR_Iter(
		Eigen::Vector3d overhangFace_normal, Eigen::Vector3d principal_Stress_Dir,QMeshTetra* Tetra);
	Eigen::Matrix3d _cal_rotationMatrix_SL_SR_Iter(
		Eigen::Vector3d overhangFace_normal, Eigen::Vector3d principal_Stress_Dir,
		QMeshTetra* Tetra, int innerLoop);
	
	double _get_critcalTet_Weight(QMeshTetra* Tetra);
	double _get_globalSmooth_keepWeight(QMeshTetra* Tetra);

	bool _intersect3D_plane_plane(
		Eigen::Vector3d n1, double d1, Eigen::Vector3d n2, double d2,
		Eigen::Vector3d& node, Eigen::Vector3d& vec);
	bool _intersect3D_line_sphere(Eigen::Vector3d O, Eigen::Vector3d T,
		Eigen::Vector3d Center, double R, double& mu1, double& mu2);

	void _giveBoundary_OverhangTet_stressInfo();
	Eigen::Vector3d _projection_vector2Plane(
		Eigen::Vector3d vec_u, Eigen::Vector3d planeNormal_n);
	Eigen::Vector3d _rotate_vector_deg(Eigen::Vector3d pnt,
		Eigen::Vector3d rotate_axis, double angle);

public:
	double m_supportFreeAngle = 0.0; //supportLess case
	double m_tensileRegionRatio = 0.0; //strengthReinforcement
	double m_compressRegionRatio = 0.0; //strengthReinforcement

public:
	//xmin xmax ymin ymax zmin zmax
	double inital_range[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };


private:
	Eigen::Vector3d printDir = { 0.0 ,1.0 ,0.0 };

	QMeshPatch* tetPatch = NULL;
	std::vector<QMeshTetra*> tetraPatch_elementSet;
	std::string tetModel_Name;
	
	int m_loopTime = 0;
	int m_innerLoopTime = 0;

	double m_criticalTet_weight_SL = 0.0;
	double m_criticalTet_weight_SR = 0.0;
	double m_criticalTet_weight_SQ_cover = 0.0;
	double m_criticalTet_weight_SQ_vertical = 0.0;

	double m_neighborScale_weight = 0.0;
	double m_regularScale_weight = 0.0;
	double m_globalSmooth_weight = 0.0;

	double m_keep_SL = 1.0;
	double m_keep_SR = 1.0;
	double m_keep_SQ_cover = 1.0;
	double m_keep_SQ_vertical = 1.0;

	double m_bottom_height = 1.0;
	double m_bandWidth_SQ_rotation = 0.1;//0.4 for(SR_SQ)

	int m_caseType = -1;
};
