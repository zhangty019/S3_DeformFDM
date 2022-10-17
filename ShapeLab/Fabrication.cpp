#include <Eigen/Eigen>
#include <Eigen/PardisoSupport>

#include "Fabrication.h"
#include "GLKGeometry.h"

void Fabrication::initial(QMeshPatch* tetModel, PolygenMesh* isoLayerSet, QMeshPatch* nozzle) {

	m_tetModel = tetModel;
	m_isoLayerSet = isoLayerSet;
	m_nozzle = nozzle;

	m_tetModel->drawOverhangSurface = false;
	m_nozzle->drawThisPatch = true;
	m_tetModel->drawStressField = false;
}

void Fabrication::initial(QMeshPatch* tetModel) {

	m_tetModel = tetModel;
}

void Fabrication::mode_RotMov_4fabrication(double rot_phi, double rot_theta, double modelHeight_y) {

	this->_rotModel(rot_phi, rot_theta);
	this->_moveModel_center_up(modelHeight_y);
	this->_modify_scalarField_order();
}

void Fabrication::_rotModel(double rot_phi, double rot_theta) {

	double theta = DEGREE_TO_ROTATE(rot_theta);
	double phi = DEGREE_TO_ROTATE(rot_phi);

	Eigen::Vector3d initNorm = { 0, 1.0, 0 };
	Eigen::Vector3d rotateDir = { cos(theta) * sin(phi), cos(phi),sin(theta) * sin(phi) };
	Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
	rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(initNorm, rotateDir);

	for (GLKPOSITION posMesh = m_tetModel->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshNode* node = (QMeshNode*)m_tetModel->GetNodeList().GetNext(posMesh);

		Eigen::Vector3d pp; node->GetCoord3D(pp(0), pp(1), pp(2));
		Eigen::Vector3d rotatedpp = rotationMatrix * pp;

		node->SetCoord3D(rotatedpp[0], rotatedpp[1], rotatedpp[2]);
		node->SetCoord3D_last(rotatedpp[0], rotatedpp[1], rotatedpp[2]);
	}
}

void Fabrication::_moveModel_center_up(double modelHeight_y) {

	double xmin, ymin, zmin, xmax, ymax, zmax;
	m_tetModel->ComputeBoundingBox(xmin, ymin, zmin, xmax, ymax, zmax);
	Eigen::Vector3d center = { (xmin + xmax) / 2.0,(ymin + ymax) / 2.0,(zmin + zmax) / 2.0 };

	double pp[3];
	for (GLKPOSITION Pos = m_tetModel->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetModel->GetNodeList().GetNext(Pos);
		Node->GetCoord3D_last(pp[0], pp[1], pp[2]);

		pp[0] -= center[0]; pp[1] -= ymin; pp[2] -= center[2];

		pp[1] += modelHeight_y;

		Node->SetCoord3D(pp[0], pp[1], pp[2]);
		Node->SetCoord3D_last(pp[0], pp[1], pp[2]);
	}
}

void Fabrication::_modify_scalarField_order() {

	double minHeight = 999999999.9;		double scalarValue_minHeight = 0.0;
	double maxHeight = -999999999.9;	double scalarValue_maxHeight = 0.0;
	double pp[3];
	for (GLKPOSITION Pos = m_tetModel->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetModel->GetNodeList().GetNext(Pos);

		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		if (pp[1] > maxHeight) {
			maxHeight = pp[1]; scalarValue_maxHeight = Node->scalarField;
		}
		if (pp[1] < minHeight) {
			minHeight = pp[1]; scalarValue_minHeight = Node->scalarField;
		}
	}

	// inverse scalar field
	if (scalarValue_minHeight > scalarValue_maxHeight) {
		for (GLKPOSITION Pos = m_tetModel->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)m_tetModel->GetNodeList().GetNext(Pos);
			Node->scalarField = 1.0 - Node->scalarField;

			if (Node->scalarField < 0.0) Node->scalarField = 0.0;
			if (Node->scalarField > 1.0) Node->scalarField = 1.0;
		}
	}

	// inverse scalar field initial
	double minScalarvalue_init = 999999999.9;
	double maxScalarvalue_init = -999999999.9;
	for (GLKPOSITION Pos = m_tetModel->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetModel->GetNodeList().GetNext(Pos);

		if (Node->scalarField_init > maxScalarvalue_init) maxScalarvalue_init = Node->scalarField_init;
		if (Node->scalarField_init < minScalarvalue_init) minScalarvalue_init = Node->scalarField_init;
	}

	if (scalarValue_minHeight > scalarValue_maxHeight) {
		for (GLKPOSITION Pos = m_tetModel->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)m_tetModel->GetNodeList().GetNext(Pos);

			//y = a+b-x; // a-x---b --> b-y---a
			Node->scalarField_init = maxScalarvalue_init + minScalarvalue_init - Node->scalarField_init;
		}
	}
}

bool Fabrication::collisionChecking() {

	bool existCollision = false;
	existCollision = this->_detectCollision_tet();
	this->_mark_collisionTet();

	return existCollision;
}

void Fabrication::collisionAware_flattening() {

	this->_smooth_vectorField_collisionTet();
}

bool Fabrication::_detectCollision_tet() {

	bool existCollision = false;

	printf("\nBEGIN run collision detection\n");

	Eigen::Vector3d nozzleInitNorm = { 0.0, 1.0, 0.0 };

	//clean the flag of node on the layers
	for (GLKPOSITION Pos = m_isoLayerSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)m_isoLayerSet->GetMeshList().GetNext(Pos);

		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);

			Node->isCollisionHappenNode = false;
			// the relatedTetEdge flag cannot cover all of the flags
			// 1. Tetra->isCollisionTetra 2. Face->show_collision
			//Node->relatedTetEdge->isCollision_EdgeOnTet = false;
		}
	}
	for (GLKPOSITION Pos = m_tetModel->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)m_tetModel->GetEdgeList().GetNext(Pos);

		Edge->isCollision_EdgeOnTet = false;
	}

	for (GLKPOSITION Pos = m_isoLayerSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)m_isoLayerSet->GetMeshList().GetNext(Pos);
		bool collisionHappen = false;

		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);

			Eigen::Vector3d nodePos, norm;
			Node->GetCoord3D(nodePos(0), nodePos(1), nodePos(2));
			Node->CalNormal(); Node->GetNormal(norm(0), norm(1), norm(2));
			norm.normalized();
			norm = -norm;

			// move and rotate the nozzle to align with the position and orientation of waypoint
			Eigen::Matrix3d rotationMatrix;
			rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(nozzleInitNorm, norm);

			for (GLKPOSITION Pos = m_nozzle->GetNodeList().GetHeadPosition(); Pos;) {
				QMeshNode* nozzleNode = (QMeshNode*)m_nozzle->GetNodeList().GetNext(Pos);
				Eigen::Vector3d nozzleNodePos;

				nozzleNode->GetCoord3D_last(nozzleNodePos(0), nozzleNodePos(1), nozzleNodePos(2));
				nozzleNodePos = rotationMatrix * nozzleNodePos + nodePos;
				nozzleNode->SetCoord3D(nozzleNodePos(0), nozzleNodePos(1), nozzleNodePos(2));
			}

			// detect collision
			if (this->_checkSingleNodeCollision_withinItsLayer(layer, Node)) {
				//std::cout << "collision node!" << std::endl;
				Node->isCollisionHappenNode = true;
				Node->relatedTetEdge->isCollision_EdgeOnTet = true;
				collisionHappen = true;
			}
		}
		if (collisionHappen) {
			std::cout << "#### No." << layer->GetIndexNo() << " contains collision nodes!" << std::endl;
			existCollision = true;
			//break;
		}
		else { 
			//std::cout << "No." << layer->GetIndexNo() << " is Collision-free!" << std::endl;
		}
	}

	return existCollision;
}

bool Fabrication::_checkSingleNodeCollision_withinItsLayer(QMeshPatch* layer, QMeshNode* checkNode) {

	std::vector<QMeshNode*> layernode_Set(layer->GetNodeNumber());
	int index = 0;
	for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* layerNode = (QMeshNode*)layer->GetNodeList().GetNext(Pos);

		layernode_Set[index] = layerNode;
		index++;
	}

	//get the Bounding Box of the nozzle
	double lowerB[3], upperB[3];
	m_nozzle->ComputeBoundingBox(lowerB[0], lowerB[1], lowerB[2], upperB[0], upperB[1], upperB[2]);
	double node_threshold = 0.5;
	//end

	std::vector<bool> flag_in_CheckNozzle(layer->GetNodeNumber());

#pragma omp parallel
	{
#pragma omp for 

		for (int i = 0; i < layernode_Set.size(); i++) {
			QMeshNode* layerNode = layernode_Set[i];

			if (checkNode == layerNode) continue;

			Eigen::Vector3d pp;
			layerNode->GetCoord3D(pp(0), pp(1), pp(2));

			//speed up with AABB box (not tree)
			bool overLap = true;
			double pp_lowerB[3], pp_upperB[3];
			for (int dimId = 0; dimId < 3; dimId++) {
				pp_lowerB[dimId] = pp[dimId] - node_threshold;
				pp_upperB[dimId] = pp[dimId] + node_threshold;
			}
			for (int dimId = 0; dimId < 3; dimId++) {
				if (pp_lowerB[dimId]> upperB[dimId] 
					|| pp_upperB[dimId] < lowerB[dimId]) {
					overLap = false;
					break;
				}
			}
			if (overLap == false) continue;
			//end
			flag_in_CheckNozzle[i] = this->_node_in_nozzle(pp(0), pp(1), pp(2));

		}
	}
	
	bool isCollision = false;
	for (int i = 0; i < layernode_Set.size(); i++) {
		if (flag_in_CheckNozzle[i]) {
			isCollision = true;
			break;
		}
	}
	return isCollision;

}

bool Fabrication::_node_in_nozzle(double xx, double yy, double zz) {

	bool is_nodeINnozzle = true;
	Eigen::Vector3d pp = { xx,yy,zz };

	for (GLKPOSITION Pos = m_nozzle->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* nozzleFace = (QMeshFace*)m_nozzle->GetFaceList().GetNext(Pos);

		Eigen::Vector3d n1, n2, n3;
		nozzleFace->GetNodeRecordPtr(0)->GetCoord3D(n1(0), n1(1), n1(2));
		nozzleFace->GetNodeRecordPtr(1)->GetCoord3D(n2(0), n2(1), n2(2));
		nozzleFace->GetNodeRecordPtr(2)->GetCoord3D(n3(0), n3(1), n3(2));

		Eigen::Vector3d planeNormal = (n2 - n1).cross(n3 - n1);
		planeNormal = planeNormal.normalized();
		//std::cout << planeNormal(0) << "," << planeNormal(1) << "," << planeNormal(2) << std::endl;

		double dd = -n3.dot(planeNormal);
		//std::cout << planeNormal(0) << "," << planeNormal(1) << "," << planeNormal(2) << "," << dd << "," << std::endl;
		if (pp.dot(planeNormal) + dd > -0.01) {
			//std::cout << "outside convex hull!" << std::endl;
			is_nodeINnozzle = false; 
			break;
		}
	}
	return is_nodeINnozzle;
}

void Fabrication::_mark_collisionTet() {

	// clean the flag isCollisionTetra
	for (GLKPOSITION Pos = m_tetModel->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)m_tetModel->GetTetraList().GetNext(Pos);

		Tetra->isCollisionTetra = false;
	}
	for (GLKPOSITION Pos = m_tetModel->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)m_tetModel->GetFaceList().GetNext(Pos);

		Face->show_collision = false;
	}

	for (GLKPOSITION Pos = m_tetModel->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)m_tetModel->GetEdgeList().GetNext(Pos);

		if (Edge->isCollision_EdgeOnTet) {

			for (GLKPOSITION Pos = Edge->GetFaceList().GetHeadPosition(); Pos;) {
				QMeshFace* Face = (QMeshFace*)Edge->GetFaceList().GetNext(Pos);

				Face->show_collision = true;

				if (Face->GetLeftTetra()) Face->GetLeftTetra()->isCollisionTetra = true;
				if (Face->GetRightTetra()) Face->GetRightTetra()->isCollisionTetra = true;
			}
		}
	}
}

void Fabrication::_smooth_vectorField_collisionTet() {

	for (GLKPOSITION Pos = m_tetModel->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)m_tetModel->GetTetraList().GetNext(Pos);

		Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
		Eigen::VectorXd fieldValue = Eigen::VectorXd::Zero(4);

		for (int i = 0; i < 4; i++) {
			fieldValue(i) = Tetra->GetNodeRecordPtr(i + 1)->scalarField;
			for (int j = 0; j < 3; j++) {
				gradient(j) += Tetra->VolumeMatrix(i, j) * fieldValue(i);
			}
		}
		Tetra->vectorField = gradient.normalized();
	}

	//conduct the smooth of collided tet elements
	int smoothLoop = 40;
	//clear the vector of collided tet elements
	for (GLKPOSITION Pos = m_tetModel->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* each_Tetra = (QMeshTetra*)m_tetModel->GetTetraList().GetNext(Pos);

		if (!each_Tetra->isCollisionTetra) continue;
		each_Tetra->vectorField << 0.0, 1.0, 0.0; //<- for ring(support-less) case
		//Eigen::Vector3d target_Vector = { each_Tetra->vectorField[0], 0.0, 0.0 }; //<- for bridgeSmall(support-less) case
		//Eigen::Vector3d target_Vector = { each_Tetra->vectorField[0], abs(each_Tetra->vectorField[0]), 0.0 }; //<- for bridgeSmall(support-less) case
		//each_Tetra->vectorField = target_Vector.normalized(); //<- for bridgeSmall(support-less) case
	}

	for (int loop = 0; loop < smoothLoop; loop++) {

		for (GLKPOSITION Pos = m_tetModel->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* each_Tetra = (QMeshTetra*)m_tetModel->GetTetraList().GetNext(Pos);

			// only considering the tetra with collision
			// if (!each_Tetra->isCollisionTetra ) continue;//<- for ring(support-less) case
			// only keep isTensileorCompressSelect tet element
			//if (each_Tetra->isTensileorCompressSelect 
			//	|| each_Tetra->isCollisionTetra ) continue;//<- for bridgeSmall(support-less) case

			for (int i = 0; i < 4; i++) {
				QMeshFace* thisFace = each_Tetra->GetFaceRecordPtr(i + 1);

				if (!thisFace->inner) continue;

				QMeshTetra* ConnectTetra = thisFace->GetRightTetra();
				if (each_Tetra == ConnectTetra) ConnectTetra = thisFace->GetLeftTetra();

				each_Tetra->vectorField += ConnectTetra->vectorField;
			}
			each_Tetra->vectorField.normalize();
		}
	}

	std::cout << "Collision-aware vector field smooth finished" << std::endl;
}

void Fabrication::concaveVector_detection() {

	//_build_tetraSet_4SpeedUp
	std::vector<QMeshTetra*> tetraSet(m_tetModel->GetTetraNumber());
	int index = 0;
	for (GLKPOSITION Pos = m_tetModel->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)m_tetModel->GetTetraList().GetNext(Pos);
		tetraSet[index] = Tetra; index++;
	}
	// protect operation
	if (m_tetModel->GetTetraNumber() != index)
		std::cout << "Error: please check the tet num of tet mesh!" << std::endl;

#pragma omp parallel   
	{
#pragma omp for 
	
		for (int i = 0; i < tetraSet.size(); i++) {
			QMeshTetra* each_Tetra = tetraSet[i];

			double sum_concave_angle = 0.0; int num_concave = 0;
			for (int i = 0; i < 4; i++) {
				// each face of tetra element
				QMeshFace* eachFace_tetEle = each_Tetra->GetFaceRecordPtr(i + 1);
				if (eachFace_tetEle->GetLeftTetra() == NULL || eachFace_tetEle->GetRightTetra() == NULL) continue;

				// get neighbor tet ele
				QMeshTetra* neigh_tet = eachFace_tetEle->GetLeftTetra();
				if (neigh_tet == each_Tetra) neigh_tet = eachFace_tetEle->GetRightTetra();

				std::vector<Eigen::Vector3d> intersect_Node_vector;
				Eigen::Vector3d Plane_V0 = Eigen::Vector3d::Zero();
				each_Tetra->CalCenterPos(Plane_V0[0], Plane_V0[1], Plane_V0[2]);

				for (int j = 0; j < 3; j++) {
					// each edge of face
					QMeshEdge* eachEdge_face = eachFace_tetEle->GetEdgeRecordPtr(j + 1);
					Eigen::Vector3d Segment_P0 = Eigen::Vector3d::Zero();
					eachEdge_face->GetStartPoint()->GetCoord3D(Segment_P0[0], Segment_P0[1], Segment_P0[2]);
					Eigen::Vector3d Segment_P1 = Eigen::Vector3d::Zero();
					eachEdge_face->GetEndPoint()->GetCoord3D(Segment_P1[0], Segment_P1[1], Segment_P1[2]);
					Eigen::Vector3d intersect_P = Eigen::Vector3d::Zero();

					//segment-plane intersection
					int intersect_case = _intersect3D_SegmentPlane // please make sure vector field towards up
					(Segment_P0, Segment_P1, Plane_V0, each_Tetra->vectorField, intersect_P);

					if (intersect_case == 1) {
						intersect_Node_vector.push_back(intersect_P);
					}
				}

				// get common edge of 2-face_Angle
				if (intersect_Node_vector.size() == 2) {
					Eigen::Vector3d e = intersect_Node_vector[1] - intersect_Node_vector[0];

					Eigen::Vector3d OE0 = intersect_Node_vector[0] - Plane_V0;
					Eigen::Vector3d OE1 = intersect_Node_vector[1] - Plane_V0;
					//decide the direction of common edge
					if (OE0.cross(OE1).dot(each_Tetra->vectorField) < 0) {
						e = -e;
					}

					//concave case
					if (each_Tetra->vectorField.cross(neigh_tet->vectorField).dot(e) < 0.0) {
						//std::cout << "concave pair" << std::endl;
						//get concave angle
						double angle_2Face = acos(each_Tetra->vectorField.dot(neigh_tet->vectorField));
						//std::cout << "the angle_2Face is " << ROTATE_TO_DEGREE(angle_2Face) << std::endl;
						num_concave++; sum_concave_angle += ROTATE_TO_DEGREE(angle_2Face);

					}
				}
			}
			double avg_concave_angle = sum_concave_angle / num_concave;
			each_Tetra->concaveAngle = avg_concave_angle;
			//std::cout << "\n####### the num_concave is " << num_concave
			//	<< ". The angle sum is " << sum_concave_angle << std::endl;
		}
	}

	//// tetra loop
	//for (GLKPOSITION Pos = m_tetModel->GetTetraList().GetHeadPosition(); Pos;) {
	//	QMeshTetra* each_Tetra = (QMeshTetra*)m_tetModel->GetTetraList().GetNext(Pos);	
	//}

	this->_concaveVector_mark(4.0);
	std::cout << "finish concave detection." << std::endl;
}

//===============================================================
#define SMALL_NUM  0.00000001 // anything that avoids division overflow
//===============================================================
// intersect3D_SegmentPlane():
//    Input:  S = a segment, and Pn = a plane = {Point V0; Vector n;}
//    Output: Intersect_P = the intersect point (when it exists)
//    Return: 0 = disjoint (no intersection)
//            1 = intersection in the unique point *I0
//            2 = the segment lies in the plane
int Fabrication::_intersect3D_SegmentPlane(
	Eigen::Vector3d Segment_P0, Eigen::Vector3d Segment_P1,
	Eigen::Vector3d Plane_V0, Eigen::Vector3d Plane_Nor,
	Eigen::Vector3d& Intersect_P
){
	Eigen::Vector3d    u = Segment_P1 - Segment_P0;
	Eigen::Vector3d    w = Segment_P0 - Plane_V0;

	double D = Plane_Nor.dot(u);
	double N = -Plane_Nor.dot(w);

	if (fabs(D) < SMALL_NUM) {          // Line segment and plane parallel
		if (N == 0)                     // segment is on the plane
			return 2;
		else
			return 0;                   // no intersection (parallel)
	}
	// they are not parallel
	// compute intersect param
	double sI = N / D;
	if (sI < 0 || sI > 1)
		return 0;                       // no intersection

	Intersect_P = Segment_P0 + sI * u;  // intersection node
	return 1;
}
//===============================================================

void Fabrication::_concaveVector_mark(double threshold) {

	for (GLKPOSITION Pos = m_tetModel->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* face = (QMeshFace*)m_tetModel->GetFaceList().GetNext(Pos);

		face->is_concave = false;
		if (face->GetLeftTetra() == NULL || face->GetRightTetra() == NULL) continue;
			
		if (face->GetLeftTetra()->concaveAngle + face->GetRightTetra()->concaveAngle > threshold) {
			face->is_concave = true;
		}
	}
}

void Fabrication::concaveVector_smooth() {

	for (GLKPOSITION Pos = m_tetModel->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* face = (QMeshFace*)m_tetModel->GetFaceList().GetNext(Pos);

		if (face->GetLeftTetra() == NULL || face->GetRightTetra() == NULL) continue;

		if (face->is_concave) {

			Eigen::Vector3d avgVectorField = (face->GetLeftTetra()->vectorField + face->GetRightTetra()->vectorField).normalized();
			face->GetLeftTetra()->vectorField = avgVectorField;
			face->GetRightTetra()->vectorField = avgVectorField;
		}
	}
	std::cout << "finish concave smooth." << std::endl;
}