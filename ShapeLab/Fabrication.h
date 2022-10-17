#pragma once
class PolygenMesh;
class QMeshPatch;
#include "../QMeshLib/PolygenMesh.h"

class Fabrication
{
public:
	Fabrication() {};
	~Fabrication() {};

	void initial(QMeshPatch* tetModel);
	void mode_RotMov_4fabrication(double rot_phi, double rot_theta, double modelHeight_y);

	void initial(QMeshPatch* tetModel, PolygenMesh* isoLayerSet, QMeshPatch* nozzle);
	bool collisionChecking();
	void collisionAware_flattening();

	void concaveVector_detection();
	void concaveVector_smooth();

private:
	void _rotModel(double rot_phi, double rot_theta);
	void _moveModel_center_up(double modelHeight_y);
	void _modify_scalarField_order();

	bool _detectCollision_tet();
	bool _checkSingleNodeCollision_withinItsLayer(QMeshPatch* layer, QMeshNode* checkNode);
	bool _node_in_nozzle(double xx, double yy, double zz);
	void _mark_collisionTet();
	void _smooth_vectorField_collisionTet();

	int _intersect3D_SegmentPlane(
		Eigen::Vector3d Segment_P0, Eigen::Vector3d Segment_P1,
		Eigen::Vector3d Plane_V0, Eigen::Vector3d Plane_Nor,
		Eigen::Vector3d& Intersect_P);

	void _concaveVector_mark(double threshold);

private:
	QMeshPatch* m_tetModel;
	PolygenMesh* m_isoLayerSet;
	QMeshPatch* m_nozzle;
};