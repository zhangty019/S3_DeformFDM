#ifndef TOOLPATHGENERATION_H
#define TOOLPATHGENERATION_H

#include "QMeshPatch.h"
#include "PolygenMesh.h"
#include "QMeshNode.h"
#include "QMeshEdge.h"
#include "QMeshFace.h"
#include "../GLKLib/GLKObList.h"

class toolpathGeneration
{
public:
	toolpathGeneration(PolygenMesh* isoLayerSet, PolygenMesh* toolPathSet,
		double deltaWidth, double deltaDistance);
	~toolpathGeneration();

	void smoothingIsoSurface();
	void generate_all_toolPath();
	void generate_all_hybrid_toolPath();
	void output_toolpath(PolygenMesh* toolPath, bool onORoff);

private:
	void _input_remeshed_init_slimSupport_layers(PolygenMesh* m_isoLayerSet);

	QMeshPatch* generate_each_bundaryToolPath(QMeshPatch* surfaceMesh);
	void resampleToolpath(QMeshPatch* patch);
	int autoComputeTPathNum(QMeshPatch* surfaceMesh, bool boundaryORzigzag);
	void generateBoundaryIsoNode(QMeshPatch* singlePath, QMeshPatch* surfaceMesh, double isoValue);
	void linkEachIsoNode(QMeshPatch* singlePath, double startIsoValue);
	QMeshNode* findNextBoundaryToolPath(QMeshNode* sNode, QMeshPatch* singlePath);
	QMeshEdge* buildNewEdgetoQMeshPatch(QMeshPatch* patch, QMeshNode* startNode, QMeshNode* endNode);
	QMeshNode* findNextNearestPoint(QMeshNode* startNode, QMeshPatch* boundIsoPatch);
	bool detectAll_isoNode_Processed(QMeshPatch* singlePath);
	QMeshNode* link_OneRing_isoNode(QMeshPatch* singlePath, QMeshNode* sNode, QMeshNode* eNode);

	void generateZigzagIsoNode(QMeshPatch* singlePath,
		QMeshPatch* surfaceMesh, double isoValue, double boundaryField_threshold);
	void linkEachIsoNode_zigzag(QMeshPatch* singlePath, QMeshNode* last_boundaryNode);
	QMeshNode* findNextZigzagToolPath(QMeshNode* sNode, QMeshPatch* singlePath);
	QMeshNode* link_OneLine_isoNode(QMeshPatch* singlePath, QMeshNode* sNode, QMeshNode* eNode);
	QMeshNode* findNextNearest_sidePoint(QMeshNode* link_sNode, QMeshPatch* singlePath);

	QMeshPatch* _getToolpath_with_CompatibleLayer_ind(
		int compatibleLayer_ind, PolygenMesh* toolPath, bool find_Support);
	int toolpathGeneration::_getSplitNumber_with_CompatibleLayer_ind(
		int compatibleLayer_ind, PolygenMesh* toolPath, bool isSupport);
	int _remove_allFile_in_Dir(std::string dirPath);

	void _getFileName_Set(std::string dirctory,
		std::vector<std::string>& fileNameCell);

	void _importStressField4toolpathGeneration();
	Eigen::MatrixXd _read_stressField();
	void _findStressVector(QMeshFace* face, Eigen::MatrixXd& stressFieldSet);
	void _getFaceNeighbor(QMeshFace* face);

	//hybrid toolpath
	QMeshPatch* generate_each_hybrid_bundaryToolPath(QMeshPatch* surfaceMesh);

	PolygenMesh* m_isoLayerSet;
	PolygenMesh* m_toolPathSet;
	double toolpath_Width;
	double toolpath_Distance;
};

#endif // TOOLPATHGENERATION_H