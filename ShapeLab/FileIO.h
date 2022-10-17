#pragma once
#include "../QMeshLib/PolygenMesh.h"

class FileIO
{
public:
	FileIO() {}
	~FileIO() {};

	void changeTet2Surface(PolygenMesh* io_Source, std::string path);
	void outputComparison_VectorField_vs_Objective(PolygenMesh* tet_Model, int caseType);
	void outputIsoSurface(PolygenMesh* isoSurface, bool isSplit);
	void output_toolpath(PolygenMesh* toolPath);
	void output_toolpath_compatible(PolygenMesh* toolPath);
	void output_userWaypoints(PolygenMesh* waypointSet, std::string path,
		double xMove, double yMove, double zMove);
	void output_stressField(PolygenMesh* tet_Model);
	void output_robotWaypoints(PolygenMesh* waypointSet, 
		std::string TOOLPATH_waypoint_dir);
	void output_Quaternion_VectorField(PolygenMesh* tet_Model);
	void output_ScalarOrHeight_field(PolygenMesh* tet_Model, bool output_scalarField);
	void output_TetMesh(QMeshPatch* tetMesh, std::string path);
	void output_discreteTet_beforeBlending(QMeshPatch* tetMesh, std::string path);
	void output_discreteTet_obj_beforeBlending(QMeshPatch* tetMesh, std::string path);
	void output_materialRotation_4CAE(PolygenMesh* tet_Model);

private:
	int _remove_allFile_in_Dir(std::string dirPath);
	bool _splitSingleSurfacePatch_detect(QMeshPatch* each_layer);
	void _splitSingleSurfacePatch_flooding(QMeshPatch* isoSurface, int index);
	void _splitSingleSurfacePatch_splitmesh(
		QMeshPatch* isoSurface, std::vector<QMeshPatch*>& isoSurfaceSet_splited);
	void _output_OneSurfaceMesh(QMeshPatch* eachLayer, std::string path);

	int _getSplitNumber_with_CompatibleLayer_ind(
		int compatibleLayer_ind, PolygenMesh* toolPath, bool isSupport);
	QMeshPatch* _getToolpath_with_CompatibleLayer_ind(
		int compatibleLayer_ind, PolygenMesh* toolPath, bool find_Support);
	Eigen::MatrixXd _read_boundaryFace_centerPosFile(QMeshPatch* tetMesh, std::string path);
	void _mark_boundaryFace(QMeshPatch* tetMesh, const Eigen::MatrixXd& boundaryFace_Set);

	std::vector<QMeshPatch*> _collect_patches_withSame_compatibleIndex(
		PolygenMesh* toolPath, int targetInd, bool find_Support);
};
