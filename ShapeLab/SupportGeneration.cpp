#include <string>
#include <fstream>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/PardisoSupport>

#include "SupportGeneration.h"
#include "../GLKLib/GLKGeometry.h"
#include "QHullLib/qhull_a.h"
#include "PMBody.h"
#include "io.h"
#include "dirent.h"
#include "alphanum.hpp"

void SupportGeneration::initial(QMeshPatch* tetPatch, QMeshPatch* platform, PolygenMesh* supportModelSet) {

	m_tetPatch = tetPatch;
	m_platform = platform;
	m_supportModelSet = supportModelSet;
	tau = 45.0;	// support angle 25.0 (bunny and topo) 15.0 (yoga)
	m_tetMoveUp_Height = 0.0; // used for special case like bridgeSmall

	this->_moveModel_up_4_initialGuess_generation(m_tetMoveUp_Height);
	this->_movePlatform_2_modelCenter();

	m_tetPatch->drawOverhangSurface = true;
	m_platform->drawThisPatch = true;
	m_tetPatch->drawStressField = false;
}

// for compatible Layer generation
void SupportGeneration::initial(QMeshPatch* tetPatch, QMeshPatch* tetSurface, QMeshPatch* tetSupport) {

	m_tetPatch = tetPatch;
	m_tetSurface = tetSurface;
	m_tetSupport = tetSupport;

	this->_index_initial(m_tetSupport, true);

	m_tetPatch->drawOverhangSurface = false;
	m_tetPatch->drawStressField = false;
}

void SupportGeneration::_moveModel_up_4_initialGuess_generation(double upHeight) {

	double xx, yy, zz;
	for (GLKPOSITION Pos = m_tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetPatch->GetNodeList().GetNext(Pos);

		Node->GetCoord3D(xx, yy, zz);
		Node->SetCoord3D(xx, (yy + m_tetMoveUp_Height), zz);
	}
}

void SupportGeneration::_movePlatform_2_modelCenter() {

	double centerXx, centerYy, centerZz;
	double minYy = 9999.99;
	for (GLKPOSITION Pos = m_tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)m_tetPatch->GetNodeList().GetNext(Pos);

		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);
		if (minYy > yy) minYy = yy;
		centerXx += xx; centerYy += yy; centerZz += zz;
	}

	centerXx /= m_tetPatch->GetNodeNumber();
	centerZz /= m_tetPatch->GetNodeNumber();

	for (GLKPOSITION Pos = m_platform->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)m_platform->GetNodeList().GetNext(Pos);

		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);

		xx += centerXx;
		//yy += minYy; // this means the platform only move X Z value
		zz += centerZz;

		node->SetCoord3D(xx, yy, zz);
	}
}

//////////////////////////////////////////////////////////////////
// input:  TET model + vector field 
// output: convex hull (wide enough for including model)
void SupportGeneration::initial_Guess_SupportEnvelope() {

	// the threshold of self-support angle (tau) tuned in initial Function
	this->_markSupportFace(); 
	//calculate the direction of support Ray; by laplacian smoothness
	this->_cal_Ray_direction(m_tetPatch);

	// reconstruct the Node Array for omp usage
	std::vector<QMeshNode*> tet_surfaceNode_set;
	for (GLKPOSITION Pos = m_tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetPatch->GetNodeList().GetNext(Pos);
		// the inner node is not considered
		if (Node->inner) continue;
		if (Node->need_Support == false) continue;
		tet_surfaceNode_set.push_back(Node);
	}

#pragma omp parallel
	{
#pragma omp for  

		for (int i = 0; i < tet_surfaceNode_set.size(); i++) {
			QMeshNode* Node = tet_surfaceNode_set[i];

			// 1. get the source node position and decent direction
			Eigen::Vector3d oringinNode; Node->GetCoord3D(oringinNode[0], oringinNode[1], oringinNode[2]);
			Eigen::Vector3d step_Direction = Node->supportRay_Dir;

			// 2. iteratively find the drop points
			for (;;) {
				bool decline_stepDir = true;
				// the first decline along the orginal direction
				if (Node->polyline_node.size() == 0)decline_stepDir = false;
				bool stopFlag = _find_targetNode(oringinNode, step_Direction, Node, decline_stepDir);
				if (stopFlag) break;
			}
		}
	}
}

bool SupportGeneration::_find_targetNode(
	Eigen::Vector3d& oringinNode,
	Eigen::Vector3d& step_Direction,
	QMeshNode* Node,
	bool decline_stepDir
) {

	//parameter:
	double step_length = 1.0; //mm
	double descend_angle = 2.0; //deg

	/******************* hit the bottom platform ******************/
	if (oringinNode[1] < bottomHeight_threshold) {
		//if the node hight(Y) less than 1mm, we consider it hit the bottom(m_bellowHeight)
		Eigen::Vector3d target_bottom_node = { oringinNode[0],0.0,oringinNode[2] };
		// add one more node to link the platform (based on at least 1 support layer)
		Node->polyline_node.push_back(target_bottom_node);
		return true; //stopFlag
	}

	/***************** incline the step_direction *****************/
	if (decline_stepDir) {
		//incline a self-supporting angle to the Y direction
		Eigen::Vector3d descend_dir = { 0.0,-1.0,0.0 };
		Eigen::Vector3d rotateAxis = step_Direction.cross(descend_dir);

		//angle between _dir and _descend_dir [0,PI]
		double radian_angle = atan2(step_Direction.cross(descend_dir).norm(), step_Direction.transpose() * descend_dir);
		if (ROTATE_TO_DEGREE(radian_angle) < descend_angle) step_Direction = descend_dir; // in the range of support angle, directly downward
		else {
			// only rotate a support angle
			Eigen::AngleAxisd V1(DEGREE_TO_ROTATE(descend_angle), rotateAxis);//rotateAxis£¬rotate tau(deg)
			step_Direction = V1 * step_Direction;
		}
	}

	Eigen::Vector3d target_node = oringinNode + step_length * step_Direction;

	Node->polyline_node.push_back(target_node);
	oringinNode = target_node;

	return false;
}

// collect the support polyline for isoLayers based on tetSurface and vector field
void SupportGeneration::collect_Support_Polyline_fromTETsurface() {

	QMeshPatch* supportRay_patch = new QMeshPatch;
	supportRay_patch->SetIndexNo(m_supportModelSet->GetMeshList().GetCount()); //index begin from 0
	m_supportModelSet->GetMeshList().AddTail(supportRay_patch);
	supportRay_patch->drawThisPatch = true;

	int max_segment_NUM = 0;
	for (GLKPOSITION Pos = m_tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetPatch->GetNodeList().GetNext(Pos);

		if (Node->polyline_node.size() != 0) {

			//record the max segment NUM
			if (Node->polyline_node.size() > max_segment_NUM) { // size return unsigned Int
				max_segment_NUM = Node->polyline_node.size();
			}

			std::vector<QMeshNode*> polyLine_node_set(Node->polyline_node.size() + 1);

			//add the 1st node
			QMeshNode* orgin_Node = new QMeshNode;
			double xx, yy, zz;
			Node->GetCoord3D(xx, yy, zz);
			orgin_Node->SetCoord3D(xx, yy, zz);
			orgin_Node->SetMeshPatchPtr(supportRay_patch);
			orgin_Node->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
			supportRay_patch->GetNodeList().AddTail(orgin_Node);
			orgin_Node->isOringin = true;
			polyLine_node_set[0] = orgin_Node;

			//add rest polyline node
			for (int j = 0; j < Node->polyline_node.size(); j++) {

				QMeshNode* poly_Node = new QMeshNode;
				poly_Node->SetCoord3D(Node->polyline_node[j][0], Node->polyline_node[j][1], Node->polyline_node[j][2]);
				poly_Node->SetMeshPatchPtr(supportRay_patch);
				poly_Node->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
				supportRay_patch->GetNodeList().AddTail(poly_Node);
				polyLine_node_set[j + 1] = poly_Node;
			}

			//add polyline edge
			for (int k = 0; k < polyLine_node_set.size() - 1; k++) {

				QMeshEdge* rayEdge = new QMeshEdge;
				rayEdge->SetStartPoint(polyLine_node_set[k]);
				rayEdge->SetEndPoint(polyLine_node_set[k + 1]);
				rayEdge->SetMeshPatchPtr(supportRay_patch);
				rayEdge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0
				supportRay_patch->GetEdgeList().AddTail(rayEdge);
			}

		}
	}

	supportRay_patch->max_segment_NUM = max_segment_NUM;
	std::cout << "max segment num: " << supportRay_patch->max_segment_NUM << std::endl;
	std::cout << "polyline segments num: " << supportRay_patch->GetEdgeNumber() << std::endl;
}

///////////////////////////////////////////////////////////////////
//Envelope Hull compute
void SupportGeneration::compute_initialGuess_EnvelopeHull() {
	//build Convex-Hull (including supporing point(1) and surface of tetrahedral mesh(2))
	int pntNum = 0;
	QMeshPatch* supportNodePatch = (QMeshPatch*)m_supportModelSet->GetMeshList().GetHead();
	pntNum += supportNodePatch->GetNodeNumber();

	pntNum += m_tetPatch->GetNodeNumber();

	facetT* facet;		vertexT* vertex, ** vertexp;
	int i, index, num, stIndex;			float pos[3];
	double vec[3][3], dir[3], v1[3], v2[3], pp[3];
	QHULLSET* newConvexFront = NULL; // new convexhull

	//all the point use to compute convex hull
	double* pntArray = (double*)malloc(sizeof(double) * 3 * pntNum); 

	int nodeIndex = 0;
	for (GLKPOSITION posMesh = supportNodePatch->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshNode* node = (QMeshNode*)supportNodePatch->GetNodeList().GetNext(posMesh);
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		for (int i = 0; i < 3; i++)
			pntArray[nodeIndex * 3 + i] = pp[i];
		nodeIndex++;
	}
	for (GLKPOSITION posMesh = m_tetPatch->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshNode* node = (QMeshNode*)m_tetPatch->GetNodeList().GetNext(posMesh);
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		for (int i = 0; i < 3; i++)
			pntArray[nodeIndex * 3 + i] = pp[i];
		nodeIndex++;
	}

	//-------------------------------------------------------------------------------------
	//	Step 2: computaing the convex-hull
	qh_init_A(stdin, stdout, stderr, 0, NULL);
	qh_initflags("Qt Qx");
	qh_init_B(pntArray, pntNum, 3, false);
	qh_qhull();
	qh_check_output();
	qh_triangulate();
	if (qh VERIFYoutput && !qh STOPpoint && !qh STOPcone) qh_check_points();

	//-------------------------------------------------------------------------------------
	//	Step 3: output the results of convex-hull computation
	int nodeNum = 0, faceNum = 0;
	faceNum = qh_qh.num_facets;		nodeNum = qh_qh.num_vertices;
	//printf("Convex-Hull: %d faces with %d vertices\n",faceNum,nodeNum);
	if (faceNum > 0 && nodeNum > 0) {
		newConvexFront = _mallocMemoryConvexHull(faceNum, nodeNum);
		//---------------------------------------------------------------------------------
		index = 0;
		FORALLvertices{
			vertex->id = index;	// before this assignment, "vertex->id" contains the id of input vertices
		newConvexFront->vertPos[index * 3] = vertex->point[0];
		newConvexFront->vertPos[index * 3 + 1] = vertex->point[1];
		newConvexFront->vertPos[index * 3 + 2] = vertex->point[2];
		index++;
		}
			//---------------------------------------------------------------------------------
		index = 0;
		FORALLfacets{
			newConvexFront->normalVec[index * 3] = facet->normal[0];
		newConvexFront->normalVec[index * 3 + 1] = facet->normal[1];
		newConvexFront->normalVec[index * 3 + 2] = facet->normal[2];
		newConvexFront->offset[index] = facet->offset;
		//	It has been verified all normal[] vectors generated by qhull library are pointing outwards and are unit-vectors 
		//	(verified by the function -- QuadTrglMesh* convexHullGeneration(QuadTrglMesh* inputMesh)  ).

		int i = 0;
		FOREACHvertex_(facet->vertices) {
			newConvexFront->faceTable[index * 3 + i] = vertex->id + 1; //index start from 1;
																	   //newConvexFront->faceTable[index * 3 + i] = vertex->id; //index start from 0;

			SET(vec[i],vertex->point);
			i++;
			if (i >= 3) break; // Note that it could be a facet with more than 3 vertices if not applying "qh_triangulate();"
		}

		//-----------------------------------------------------------------------------
		//	Check if the vertices on this face is given in the anti-clockwise order
		SUB(v1,vec[1],vec[0]);
		SUB(v2,vec[2],vec[0]);
		CROSS(dir,v1,v2);
		if (DOT(dir,facet->normal) < 0) {
			unsigned int temp = newConvexFront->faceTable[index * 3];
			newConvexFront->faceTable[index * 3] = newConvexFront->faceTable[index * 3 + 2];
			newConvexFront->faceTable[index * 3 + 2] = temp;
		}

		index++;
		}
	}

	//-------------------------------------------------------------------------------------
	//	Step 4: free the memory
	int curlong, totlong;
	qh_freeqhull(false);
	qh_memfreeshort(&curlong, &totlong);
	if (curlong || totlong) 
		fprintf(stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
	//-------------------------------------------------------------------------------------
	free(pntArray);

	_build_ConvexHull_mesh(newConvexFront, m_supportModelSet); //(its mesh index = 1;)
}

QHULLSET* SupportGeneration::_mallocMemoryConvexHull(int faceNum, int vertNum) {

	QHULLSET* pConvexHull;

	pConvexHull = (QHULLSET*)malloc(sizeof(QHULLSET));
	pConvexHull->faceNum = faceNum;
	pConvexHull->normalVec = (double*)malloc(sizeof(double) * 3 * faceNum);
	pConvexHull->offset = (double*)malloc(sizeof(double) * faceNum);

	pConvexHull->faceTable = (unsigned int*)malloc(sizeof(unsigned int) * 3 * faceNum);

	pConvexHull->vertNum = vertNum;
	pConvexHull->vertPos = (double*)malloc(sizeof(double) * 3 * vertNum);

	return pConvexHull;
}

void SupportGeneration::_build_ConvexHull_mesh(QHULLSET* ConvexHULL, PolygenMesh* m_supportRaySet) {

	QMeshPatch* convexHullVisual = new QMeshPatch;
	convexHullVisual->drawThisPatch = true;
	convexHullVisual->SetIndexNo(m_supportRaySet->GetMeshList().GetCount()); //index begin from 0
	m_supportRaySet->GetMeshList().AddTail(convexHullVisual);

	float* nodeTable;
	nodeTable = (float*)malloc(sizeof(float) * ConvexHULL->vertNum * 3);
	for (int i = 0; i < ConvexHULL->vertNum * 3; i++)
		nodeTable[i] = (float)ConvexHULL->vertPos[i];
	unsigned int* faceTable;
	faceTable = (unsigned int*)malloc(sizeof(unsigned int) * ConvexHULL->faceNum * 3);
	for (int i = 0; i < ConvexHULL->faceNum * 3; i++)
		faceTable[i] = ConvexHULL->faceTable[i] - 1;

	convexHullVisual->constructionFromVerFaceTable
	(ConvexHULL->vertNum, nodeTable, ConvexHULL->faceNum, faceTable);
	_freeMemoryConvexHull(ConvexHULL);
	free(nodeTable);	free(faceTable);
}

void SupportGeneration::_freeMemoryConvexHull(QHULLSET*& pConvexHull) {
	free((pConvexHull->normalVec));
	free((pConvexHull->offset));
	free((pConvexHull->faceTable));
	free((pConvexHull->vertPos));
	free(pConvexHull);

	pConvexHull = NULL;
}

void SupportGeneration::outputMaterial_4_envelopHullGeneration() {

	QMeshPatch* convexHull_patch = NULL;
	QMeshPatch* supportPoly_patch = NULL;
	for (GLKPOSITION Pos = m_supportModelSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* supportRay_Patch = (QMeshPatch*)m_supportModelSet->GetMeshList().GetNext(Pos);
		// 0 - support ray; 1 - un-remeshed convexHull
		if (supportRay_Patch->GetIndexNo() == 1) { 
			supportRay_Patch->drawThisPatch = false; 
			convexHull_patch = supportRay_Patch;
		}
		if (supportRay_Patch->GetIndexNo() == 0) {
			supportRay_Patch->drawThisPatch = true;
			supportPoly_patch = supportRay_Patch;
		}
		else 
			supportRay_Patch->drawThisPatch = true;
	}
	this->_enlarge_convexHull(convexHull_patch);
	std::string supportConvex_dir = "../DataSet/support_convex/" + m_tetPatch->patchName + "_ConvexHull";
	this->_output_oneSurfaceMesh(convexHull_patch, supportConvex_dir);
	std::cout << "Output _convexHull_ of model for support generation." << std::endl;
	std::cout << supportConvex_dir << std::endl;

	std::string tetModel_surfac_dir = "../DataSet/support_convex/" + m_tetPatch->patchName + "_tetSurface";
	this->_output_tetraMesh_boundary(m_tetPatch, tetModel_surfac_dir);
	std::cout << "Output _boundary surface_ of Tet model for support generation." << std::endl;
	std::cout << tetModel_surfac_dir << std::endl;

	std::string tetModel_support_polyline_dir = "../DataSet/support_convex/" + m_tetPatch->patchName + "_support_polyline";
	this->_output_support_polyline(supportPoly_patch, tetModel_support_polyline_dir);
	std::cout << "Output support polyline for initial guess of support generation." << std::endl;
	std::cout << tetModel_support_polyline_dir << std::endl;
}

void SupportGeneration::outputMaterial_4_envelopHullGeneration_plus() {

	QMeshPatch* remeshed_CH = NULL;	QMeshPatch* tet_surface = NULL;
	for (GLKPOSITION Pos = m_supportModelSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* supportRay_Patch = (QMeshPatch*)m_supportModelSet->GetMeshList().GetNext(Pos);
		// 0 - support ray; 1 - un-remeshed convexHull; 2 - remeshed convexHull; 3 - tet_BoundarySurface;
		if (supportRay_Patch->GetIndexNo() == 2) { remeshed_CH = supportRay_Patch; }
		if (supportRay_Patch->GetIndexNo() == 3) { tet_surface = supportRay_Patch; }
	}

	//after doing the remesh in Meshlab
	//combine the convexHull (enlarged + remeshed) with original mesh
	std::string offMesh_dir = "../DataSet/support_convex/" + m_tetPatch->patchName + "_supportSpace";
	this->_output_combined_offMesh(tet_surface, remeshed_CH, offMesh_dir);
	std::cout << "Output combined_offMesh of model for support generation." << std::endl;
	std::cout << offMesh_dir << std::endl;
}

void SupportGeneration::_enlarge_convexHull(QMeshPatch* convexHull_patch) {

	Eigen::Vector3d coord3D_CoM = { 0.0,0.0,0.0 };
	for (GLKPOSITION posNode = convexHull_patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)convexHull_patch->GetNodeList().GetNext(posNode);

		Eigen::Vector3d pp; node->GetCoord3D(pp[0], pp[1], pp[2]);
		coord3D_CoM += pp;
	}
	coord3D_CoM /= convexHull_patch->GetNodeNumber();

	double expand_ratio = 1.2;
	for (GLKPOSITION posNode = convexHull_patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)convexHull_patch->GetNodeList().GetNext(posNode);

		Eigen::Vector3d pp; node->GetCoord3D(pp[0], pp[1], pp[2]);
		pp = (1.0 - expand_ratio) * coord3D_CoM + expand_ratio * pp;
		node->SetCoord3D(pp[0], pp[1], pp[2]);
	}

}

void SupportGeneration::_output_oneSurfaceMesh(QMeshPatch* isoSurface, std::string path) {

	double pp[3];
	path += ".obj";
	std::ofstream nodeSelection(path);

	int index = 0;
	for (GLKPOSITION posNode = isoSurface->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)isoSurface->GetNodeList().GetNext(posNode);
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		nodeSelection << "v " << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
		index++; node->SetIndexNo(index);
	}
	for (GLKPOSITION posFace = isoSurface->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)isoSurface->GetFaceList().GetNext(posFace);
		nodeSelection << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << std::endl;
	}
	nodeSelection.close();
}

void SupportGeneration::_output_tetraMesh_boundary(QMeshPatch* tetPatch, std::string path) {

	double pp[3];
	path += ".obj";
	std::ofstream str(path);

	int index = 0;
	for (GLKPOSITION posNode = tetPatch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(posNode);

		if (node->inner) continue;
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		str << "v " << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
		index++; node->Volume2Surface_Ind = index;
	}
	for (GLKPOSITION posFace = tetPatch->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(posFace);
		if (face->inner) continue;
		str << "f " << face->GetNodeRecordPtr(0)->Volume2Surface_Ind
			<< " " << face->GetNodeRecordPtr(1)->Volume2Surface_Ind
			<< " " << face->GetNodeRecordPtr(2)->Volume2Surface_Ind << std::endl;
	}
	str.close();
}

void SupportGeneration::_output_support_polyline(QMeshPatch* supportPoly_patch, std::string path) {

	double pp[3];
	path += ".txt";
	std::ofstream str(path);

	for (GLKPOSITION posNode = supportPoly_patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)supportPoly_patch->GetNodeList().GetNext(posNode);

		node->GetCoord3D(pp[0], pp[1], pp[2]);
		str << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
	}
	str.close();
}

void SupportGeneration::_output_combined_offMesh(
	QMeshPatch* remeshed_CH, QMeshPatch* tetMesh_boundary, std::string path) {

	/* combine two mesh together and output */
	double pp[3];
	path += ".off";

	std::ofstream outputFile(path);
	outputFile << "OFF" << std::endl;
	outputFile << remeshed_CH->GetNodeNumber() + tetMesh_boundary->GetNodeNumber()
		<< " " << remeshed_CH->GetFaceNumber() + tetMesh_boundary->GetFaceNumber() << " 0" << std::endl;


	int index = 0;
	for (GLKPOSITION posNode = remeshed_CH->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)remeshed_CH->GetNodeList().GetNext(posNode);
		node->GetCoord3D(pp[0],pp[1],pp[2]);
		outputFile << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
		index++; node->SetIndexNo(index);
	}
	for (GLKPOSITION posNode = tetMesh_boundary->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetMesh_boundary->GetNodeList().GetNext(posNode);
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		outputFile << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
		index++; node->SetIndexNo(index);
	}

	for (GLKPOSITION posFace = remeshed_CH->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)remeshed_CH->GetFaceList().GetNext(posFace);
		outputFile << "3 " << face->GetNodeRecordPtr(0)->GetIndexNo() - 1
			<< " " << face->GetNodeRecordPtr(1)->GetIndexNo() - 1
			<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() - 1 << std::endl;
	}

	for (GLKPOSITION posFace = tetMesh_boundary->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetMesh_boundary->GetFaceList().GetNext(posFace);
		outputFile << "3 " << face->GetNodeRecordPtr(0)->GetIndexNo() - 1
			<< " " << face->GetNodeRecordPtr(1)->GetIndexNo() - 1
			<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() - 1 << std::endl;
	}

	outputFile.close();

}

void SupportGeneration::_markSupportFace() {

	this->_update_bottomTet_flag(m_bottom_Threshold, m_tetPatch);

	int boundaryFace_ind = 0;
	for (GLKPOSITION PosFace = (m_tetPatch->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
		QMeshFace* face = (QMeshFace*)((m_tetPatch->GetFaceList()).GetNext(PosFace));

		face->isOverhangFace = false; //clear the flag isOverhangFace used for deformation
		face->needSupport = false;//reset
		if (face->inner == true) continue;// the inner face is not considered 

		//get TET element
		QMeshTetra* surface_tetElement = NULL;
		if (face->GetLeftTetra() == NULL) surface_tetElement = face->GetRightTetra();
		else surface_tetElement = face->GetLeftTetra();
		//protection code
		if (surface_tetElement == NULL) std::cout << "ERROR: no need TET element of surface\n";

		//get face normal (outward)
		Eigen::Vector3d n;
		face->CalPlaneEquation();
		face->GetNormal(n[0], n[1], n[2]);
		n = -n.normalized();

		//std::cout << surface_tetElement->vectorField << std::endl;
		//surface_tetElement->vectorField << 0, 0, 1;
		//mark risk face
		if ((n.dot(surface_tetElement->vectorField) + sin(DEGREE_TO_ROTATE(tau))) < 0) {

			face->needSupport = true;
		}
		boundaryFace_ind++;
	}

	// filter the single overhang faces
	std::vector<bool> needSupport_flagSet(boundaryFace_ind);
	boundaryFace_ind = 0;
	for (GLKPOSITION PosFace = (m_tetPatch->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
		QMeshFace* face = (QMeshFace*)((m_tetPatch->GetFaceList()).GetNext(PosFace));

		if (face->inner == true) continue;// the inner face is not considered

		if (_is_single_OverHangface(face)) {
			needSupport_flagSet[boundaryFace_ind] = false;
		}
		else {
			needSupport_flagSet[boundaryFace_ind] = true;
		}
		boundaryFace_ind++;
	}
	boundaryFace_ind = 0;
	for (GLKPOSITION PosFace = (m_tetPatch->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
		QMeshFace* face = (QMeshFace*)((m_tetPatch->GetFaceList()).GetNext(PosFace));

		if (face->inner == true) continue;// the inner face is not considered
		face->needSupport = needSupport_flagSet[boundaryFace_ind];
		boundaryFace_ind++;
	}

	// Bottom faces should not be overhang/needSupport ones
	for (GLKPOSITION pos = m_tetPatch->GetTetraList().GetHeadPosition(); pos != nullptr;) {
		QMeshTetra* tetra = (QMeshTetra*)m_tetPatch->GetTetraList().GetNext(pos);
		double xx, yy, zz;
		tetra->CalCenterPos(xx, yy, zz);
		//this is used for mark bottom face when model is moved up
		if (tetra->isBottomTet && (fabs(yy)< m_bottom_Threshold)) {
			for (int i = 0; i < 4; i++) {
				tetra->GetFaceRecordPtr(i + 1)->needSupport = false;
			}
		}
	}

	//record node which need support
	//reset flag (needsupport) of node 
	for (GLKPOSITION PosNode = (m_tetPatch->GetNodeList()).GetHeadPosition(); PosNode != NULL;) {
		QMeshNode* node = (QMeshNode*)((m_tetPatch->GetNodeList()).GetNext(PosNode));
		node->need_Support == false;
	}
	for (GLKPOSITION PosFace = (m_tetPatch->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
		QMeshFace* face = (QMeshFace*)((m_tetPatch->GetFaceList()).GetNext(PosFace));

		if (face->needSupport == true) {
			for (int i = 0; i < 3; i++)
				face->GetNodeRecordPtr(i)->need_Support = true;
		}
	}
}

bool SupportGeneration::_is_single_OverHangface(QMeshFace* face) {

	int neighbor_overhangFace_NUM = 0;
	for (int i = 0; i < 3; i++) {
		QMeshEdge* eachEdge = face->GetEdgeRecordPtr(i + 1);

		for (GLKPOSITION Pos = eachEdge->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)eachEdge->GetFaceList().GetNext(Pos);

			if (Face == face) continue;
			if (Face->inner) continue;

			if (Face->needSupport) neighbor_overhangFace_NUM++;

		}
	}

	if (neighbor_overhangFace_NUM < 2) return true; // 0 or 1 overhang face neighbor
	else if (neighbor_overhangFace_NUM < 4) return false; // 2 or 3 overhang face neighbors
	else { 
		std::cout << "Error case: the overhang face neighbor should not be more than 3, please check.\n";
		return false;
	}
}

// height bellow (1mm) will be considered as bottom
void SupportGeneration::_update_bottomTet_flag(double threshold, QMeshPatch* tetMesh) {

	//mark the bottom
	double minHight = 99999.0;
	for (GLKPOSITION pos = tetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(pos);
		double pp[3] = { 0 }; node->GetCoord3D(pp[0], pp[1], pp[2]);
		if (pp[1] < minHight) minHight = pp[1];
	}

	std::printf("The minume height: %f.\n", minHight);

	for (GLKPOSITION pos = tetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(pos);
		double pp[3] = { 0 }; node->GetCoord3D(pp[0], pp[1], pp[2]);
		if (pp[1] < minHight + threshold) node->isBottomNode = true;
	}
	// mark bottom tet element
	int bottom_Num = 0;
	for (GLKPOSITION pos = tetMesh->GetTetraList().GetHeadPosition(); pos != nullptr;) {
		QMeshTetra* tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(pos);
		tetra->isBottomTet = false;
		for (int i = 0; i < 4; i++) {
			if (tetra->GetNodeRecordPtr(i + 1)->isBottomNode) {
				tetra->isBottomTet = true;
				bottom_Num++;
				break;
			}
		}
	}
	std::printf("The bottom tet Num: %d.\n", bottom_Num);
	// clear isBottomNode flag
	for (GLKPOSITION pos = tetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(pos);
		node->isBottomNode = false;
	}
}

void SupportGeneration::_cal_Ray_direction(QMeshPatch* tet_Model) {

	for (GLKPOSITION Pos = tet_Model->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tet_Model->GetNodeList().GetNext(Pos);

		if (Node->inner) { continue; }	// the inner face is not considered 
		if (Node->need_Support == false) continue;

		// get average Vector field of support node
		Eigen::Vector3d averageField = { 0.0,0.0,1.0 };
		for (GLKPOSITION node_neighborTet_Pos = Node->GetTetraList().GetHeadPosition(); node_neighborTet_Pos;) {
			QMeshTetra* ConnectTetra = (QMeshTetra*)Node->GetTetraList().GetNext(node_neighborTet_Pos);

			averageField += ConnectTetra->vectorField;
		}
		Node->supportRay_Dir = -averageField.normalized();// reverse direction for projecting ray onto the platform
	}
}

/* Function for detecting the boundary in the supportTET */
void SupportGeneration::boundary_Detection() {

	//_build_tetraSet_4SpeedUp
	std::vector<QMeshTetra*> tetraSet_supportSpace(m_tetSupport->GetTetraNumber());
	int index = 0;
	for (GLKPOSITION Pos = m_tetSupport->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)m_tetSupport->GetTetraList().GetNext(Pos);
		tetraSet_supportSpace[index] = Tetra; index++;
	}
	// protect operation
	if (m_tetSupport->GetTetraNumber() != index)
		std::cout << "Error: please check the tet num of tet mesh(supportSpace)!" << std::endl;

	//detect the center of tet element of supportTET in/out the model surface
#pragma omp parallel   
	{
#pragma omp for 
		for (int i = 0; i < tetraSet_supportSpace.size(); i++) {
			QMeshTetra* each_Tetra = tetraSet_supportSpace[i];

			double xx, yy, zz;
			each_Tetra->CalCenterPos(xx, yy, zz);
			each_Tetra->isIn_tetSurfaceMesh = _eleCenter_in_Mesh(m_tetSurface, xx, yy, zz);
		}
	}

	//clear the is_tetSupportNode flag
	for (GLKPOSITION Pos = m_tetSupport->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetSupport->GetNodeList().GetNext(Pos);

		Node->is_tetSupportNode = false;
	}
	//mark the is_tetSupportNode
	for (GLKPOSITION Pos = m_tetSupport->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)m_tetSupport->GetTetraList().GetNext(Pos);

		if (Tetra->isIn_tetSurfaceMesh) continue;

		for (int nodeInd = 0; nodeInd < 4; nodeInd++) {
			Tetra->GetNodeRecordPtr(nodeInd + 1)->is_tetSupportNode = true;
		}
	}

	//generate and output hollowed support TET
	std::string hollowed_supportTet_dir = "../DataSet/TET_MODEL/" + m_tetPatch->patchName + "_supportTet_hollowed";
	this->_hollow_tetSupport_generation(m_tetSupport, hollowed_supportTet_dir);

	//for (GLKPOSITION Pos = m_tetSupport->GetFaceList().GetHeadPosition(); Pos;) {
	//	QMeshFace* Face = (QMeshFace*)m_tetSupport->GetFaceList().GetNext(Pos);
	//
	//	if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;
	//
	//	if ((Face->GetLeftTetra()->isIn_tetSurfaceMesh && !Face->GetRightTetra()->isIn_tetSurfaceMesh)
	//		|| (!Face->GetLeftTetra()->isIn_tetSurfaceMesh && Face->GetRightTetra()->isIn_tetSurfaceMesh)) {
	//
	//		Face->model_boundary = true;
	//		//transform the model_boundary flag to edges
	//		for (int edgeInd = 0; edgeInd < 3; edgeInd++) {
	//			Face->GetEdgeRecordPtr(edgeInd + 1)->model_boundary = true;
	//		}
	//		//transform the model_boundary flag to node
	//		for (int nodeInd = 0; nodeInd < 3; nodeInd++) {
	//			Face->GetNodeRecordPtr(nodeInd)->model_boundary = true;
	//		}
	//	}
	//	//std::cout << Face->convexHull_boundary << std::endl;		
	//}
	std::cout << "Finish boundary detection." << std::endl;
}

void SupportGeneration::_index_initial(QMeshPatch* patch, bool is_TetMesh) {

	//----initial the edge, node and face index, start from 0
	int index = 0;
	for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);
		Edge->SetIndexNo(index);
		index++;
	}
	index = 0;
	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
		Face->SetIndexNo(index);
		Face->CalPlaneEquation(); // pre-compute the normal of face
		index++;
	}
	index = 0;
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

		Node->SetIndexNo(index);
		index++;
	}
	// only tet mesh needs reorder the index of Tetra
	if (is_TetMesh) {
		index = 0;
		for (GLKPOSITION Pos = patch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)patch->GetTetraList().GetNext(Pos);

			Tetra->SetIndexNo(index);
			index++;
		}
	}
	std::cout << "Finish initializing mesh index." << std::endl;
}

bool SupportGeneration::_eleCenter_in_Mesh(QMeshPatch* surfaceMesh, double qx, double qy, double qz) {

	Eigen::Vector3d dir = { 1.0,0.0,0.0 };
	Eigen::Vector3d orig = { qx,qy,qz };
	int intersection_Time = 0;


	for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* each_face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);

		double xx, yy, zz;
		each_face->GetNodeRecordPtr(0)->GetCoord3D(xx, yy, zz);
		Eigen::Vector3d v0 = { xx,yy,zz };

		each_face->GetNodeRecordPtr(1)->GetCoord3D(xx, yy, zz);
		Eigen::Vector3d v1 = { xx,yy,zz };

		each_face->GetNodeRecordPtr(2)->GetCoord3D(xx, yy, zz);
		Eigen::Vector3d v2 = { xx,yy,zz };

		if (this->IntersectTriangle(orig, dir, v0, v1, v2))
			intersection_Time++;
	}
	//std::cout << "intersection Num " << intersection_Time << std::endl;
	if (intersection_Time % 2 != 0) {
		//std::cout << "in the mesh" << std::endl;
		return true;
	}
	else return false;
	//std::cout << "be out of mesh" << std::endl;
}

/* Determine whether a ray intersect with a triangle
/ Parameters
/ orig: origin of the ray
/ dir: direction of the ray
/ v0, v1, v2: vertices of triangle
/ t(out): weight of the intersection for the ray
/ u(out), v(out): barycentric coordinate of intersection */

bool SupportGeneration::IntersectTriangle(const Eigen::Vector3d& orig, const Eigen::Vector3d& dir,
	Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2)
{
	// E1
	Eigen::Vector3d E1 = v1 - v0;

	// E2
	Eigen::Vector3d E2 = v2 - v0;

	// P
	Eigen::Vector3d P = dir.cross(E2);

	// determinant
	float det = E1.dot(P);

	// keep det > 0, modify T accordingly
	Eigen::Vector3d T;
	if (det > 0)
	{
		T = orig - v0;
	}
	else
	{
		T = v0 - orig;
		det = -det;
	}

	// If determinant is near zero, ray lies in plane of triangle
	if (det < 0.0001f)
		return false;

	// Calculate u and make sure u <= 1
	double t, u, v;
	u = T.dot(P);
	if (u < 0.0f || u > det)
		return false;

	// Q
	Eigen::Vector3d Q = T.cross(E1);

	// Calculate v and make sure u + v <= 1
	v = dir.dot(Q);
	if (v < 0.0f || u + v > det)
		return false;

	// Calculate t, scale parameters, ray intersects triangle
	t = E2.dot(Q);
	if (t < 0) return false;

	float fInvDet = 1.0f / det;
	t *= fInvDet;
	u *= fInvDet;
	v *= fInvDet;

	return true;
}

void SupportGeneration::transfer_ScalarField_2_supportSpace() {

	//_build_nodeSet_4SpeedUp
	std::vector<QMeshNode*> nodeSet_supportSpace(m_tetSupport->GetNodeNumber());
	int index = 0;
	for (GLKPOSITION posNode = m_tetSupport->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)m_tetSupport->GetNodeList().GetNext(posNode);

		nodeSet_supportSpace[index] = node; index++;
	}
	// protect operation
	if (m_tetSupport->GetNodeNumber() != index) 
		std::cout << "Error: please check the node num of tet mesh (supportSpace)!" << std::endl;

#pragma omp parallel   
	{
#pragma omp for 
		for (int i = 0; i < nodeSet_supportSpace.size(); i++) {
			QMeshNode* each_node_supportSpace = nodeSet_supportSpace[i];

			if (each_node_supportSpace->inner) continue;

			each_node_supportSpace->scalarField_init =
				_get_scalarfield_of_NearestNode(m_tetPatch, each_node_supportSpace);

			//std::cout << "each_node_supportSpace->scalarField_init " 
			//	<< each_node_supportSpace->scalarField_init << std::endl;

			//std::cout << "each_node_supportSpace->scalarField "
			//	<< each_node_supportSpace->scalarField << std::endl;
		}
	}
	std::cout << "Finish boundary scalar field transformation." << std::endl;
}

double SupportGeneration::_get_scalarfield_of_NearestNode(QMeshPatch* init_tetMesh, QMeshNode* inquireNode_supportSpace ) {

	Eigen::Vector3d node_coord3D = Eigen::Vector3d::Zero();
	inquireNode_supportSpace->GetCoord3D(node_coord3D[0], node_coord3D[1], node_coord3D[2]);

	double minSquareDis = 9999999.9;
	double nearest_node_scalarValue = 0.0;

	for (GLKPOSITION Pos = init_tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)init_tetMesh->GetNodeList().GetNext(Pos);

		if (Node->inner) continue;

		Eigen::Vector3d nodeCoord3D_tetMesh_boundary = Eigen::Vector3d::Zero();
		Node->GetCoord3D(nodeCoord3D_tetMesh_boundary[0], nodeCoord3D_tetMesh_boundary[1], nodeCoord3D_tetMesh_boundary[2]);
		
		double squareDis = (nodeCoord3D_tetMesh_boundary - node_coord3D).squaredNorm();
		if (minSquareDis > squareDis) {
			minSquareDis = squareDis;
			nearest_node_scalarValue = Node->scalarField_init;
			/*nearest_node_scalarValue = Node->scalarField;*/
		}
	}
	if (minSquareDis < 0.0001) inquireNode_supportSpace->model_boundary = true;
	//std::cout << nearest_node_scalarValue << std::endl;
	return nearest_node_scalarValue;
}

void SupportGeneration::transfer_VectorField_2_supportSpace() {

	//_build_faceSet_4SpeedUp
	std::vector<QMeshFace*> faceSet_supportSpace(m_tetSupport->GetFaceNumber());
	int index = 0;
	for (GLKPOSITION posNode = m_tetSupport->GetFaceList().GetHeadPosition(); posNode != nullptr;) {
		QMeshFace* face = (QMeshFace*)m_tetSupport->GetFaceList().GetNext(posNode);

		faceSet_supportSpace[index] = face; index++;
	}
	// protect operation
	if (m_tetSupport->GetFaceNumber() != index)
		std::cout << "Error: please check the face num of tet mesh (supportSpace)!" << std::endl;

#pragma omp parallel   
	{
#pragma omp for 
		for (int i = 0; i < faceSet_supportSpace.size(); i++) {
			QMeshFace* each_face_supportSpace = faceSet_supportSpace[i];

			if (each_face_supportSpace->inner) continue;

			each_face_supportSpace->boundaryVector = _get_vectorfield_of_NearestFace(m_tetPatch, each_face_supportSpace);

			QMeshTetra* supportTet_ele_faceNeighbor = each_face_supportSpace->GetLeftTetra();
			if (supportTet_ele_faceNeighbor == NULL) 
				supportTet_ele_faceNeighbor = each_face_supportSpace->GetRightTetra();

			//protection code
			if (supportTet_ele_faceNeighbor == NULL)
				std::cerr << "This face doesn't have neighbor tet, please check." << std::endl;
			
			supportTet_ele_faceNeighbor->vectorField = each_face_supportSpace->boundaryVector;
			if(each_face_supportSpace->model_boundary)
				supportTet_ele_faceNeighbor->isVectorSource = true;
		}
	}

	//obtain the vectorField of the rest simple smooth 
	this->_vectorField_flooding_supportSpace(200);
	std::cout << "Finish boundary vector field transformation and vector field flooding." << std::endl;
}

Eigen::Vector3d SupportGeneration::_get_vectorfield_of_NearestFace(QMeshPatch* init_tetMesh, QMeshFace* inquireFace_supportSpace) {

	Eigen::Vector3d faceCenter_coord3D = Eigen::Vector3d::Zero();
	inquireFace_supportSpace->CalCenterPos(faceCenter_coord3D[0], faceCenter_coord3D[1], faceCenter_coord3D[2]);

	double minSquareDis = 9999999.9;
	Eigen::Vector3d nearest_face_boundaryVector = Eigen::Vector3d::Zero();

	for (GLKPOSITION Pos = init_tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)init_tetMesh->GetFaceList().GetNext(Pos);

		if (Face->inner) continue;

		Eigen::Vector3d initTet_faceCenter = Eigen::Vector3d::Zero();
		Face->GetCenterPos(initTet_faceCenter[0], initTet_faceCenter[1], initTet_faceCenter[2]);

		double squareDis = (initTet_faceCenter - faceCenter_coord3D).squaredNorm();
		if (minSquareDis > squareDis) {
			minSquareDis = squareDis;

			QMeshTetra* initTet_ele_faceNeighbor = Face->GetLeftTetra();
			if (initTet_ele_faceNeighbor == NULL) initTet_ele_faceNeighbor = Face->GetRightTetra();

			//protection code
			if (initTet_ele_faceNeighbor == NULL) 
				std::cerr << "This face doesn't have neighbor tet, please check." << std::endl;

			nearest_face_boundaryVector = initTet_ele_faceNeighbor->vectorField;
		}
	}
	if (minSquareDis < 0.0001) inquireFace_supportSpace->model_boundary = true;
	return nearest_face_boundaryVector;
}

void SupportGeneration::_vectorField_flooding_supportSpace(int loopTime) {

	double xmin, ymin, zmin, xmax, ymax, zmax;
	m_tetSupport->ComputeBoundingBox(xmin, ymin, zmin, xmax, ymax, zmax);
	this->_update_bottomTet_flag(m_bottom_Threshold, m_tetSupport);
	for (GLKPOSITION Pos = m_tetSupport->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* each_Tetra = (QMeshTetra*)m_tetSupport->GetTetraList().GetNext(Pos);

		// keep the sorce vector
		if (each_Tetra->isVectorSource) continue;
		if (each_Tetra->isBottomTet) each_Tetra->vectorField = { 0.0,1.0,0.0 };
	}


	for (int loop = 0; loop < loopTime; loop++) {

		for (GLKPOSITION Pos = m_tetSupport->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* each_Tetra = (QMeshTetra*)m_tetSupport->GetTetraList().GetNext(Pos);

			// keep the sorce vector
			if (each_Tetra->isVectorSource) continue;
			if (each_Tetra->isBottomTet) continue;

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
}

void SupportGeneration::scalarField_4_supportSpace() {

	this->_compTetMeshVolumeMatrix(m_tetSupport);

	int supportEleNum = m_tetSupport->GetTetraNumber();

	int index = 0;
	for (GLKPOSITION Pos = m_tetSupport->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetSupport->GetNodeList().GetNext(Pos);
		if (Node->model_boundary) Node->supportIndex = -1;
		else { Node->supportIndex = index; index++; }
	}
	int supportNodeNum = index;

	std::cout << " -- support element number = " << supportEleNum << ", "
		" support node number = " << supportNodeNum << std::endl;

	Eigen::SparseMatrix<double> Parameter(3 * supportEleNum, supportNodeNum); //A
	Eigen::VectorXd guideField(supportNodeNum); //x: this vector is for support region
	Eigen::VectorXd b(3 * supportEleNum); //b
	b.setZero();

	std::vector<Eigen::Triplet<double>> ParaTriplet;
	for (GLKPOSITION Pos = m_tetSupport->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)m_tetSupport->GetTetraList().GetNext(Pos);

		double weight = 1.0;
		if (Tetra->isVectorSource) weight = 2.0;
		if (Tetra->isBottomTet) weight = 1.5;

		for (int i = 0; i < 3; i++) b(Tetra->GetIndexNo() * 3 + i) = Tetra->vectorField(i) * weight; // infill B

		for (int j = 0; j < 3; j++) {
			for (int i = 0; i < 4; i++) {

				QMeshNode* Node = Tetra->GetNodeRecordPtr(i + 1);
				int nodeIndex = Node->supportIndex;
				if (nodeIndex >= 0)
					ParaTriplet.push_back(Eigen::Triplet<double>(
						Tetra->GetIndexNo() * 3 + j, nodeIndex, -Tetra->VolumeMatrix(i, j) * weight)); // infill A
				else {
					b(Tetra->GetIndexNo() * 3 + j) -= Node->scalarField_init * (-Tetra->VolumeMatrix(i, j)) * weight; //constrain to B
				}
			}
		}
	}

	Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

	Eigen::SparseMatrix<double> ATA(supportNodeNum, supportNodeNum);
	ATA = Parameter.transpose() * Parameter;
	Eigen::PardisoLU <Eigen::SparseMatrix<double>> Solver;
	Solver.compute(ATA);

	Eigen::VectorXd ATb(supportNodeNum);
	ATb = Parameter.transpose() * b;
	guideField = Solver.solve(ATb);

	for (GLKPOSITION Pos = m_tetSupport->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetSupport->GetNodeList().GetNext(Pos);
		if (Node->supportIndex < 0) continue;
		else { 
			Node->scalarField_init = guideField(Node->supportIndex);
		}
	}

	// clamp the scalar field of the _initial_model_ into [0, 1]
	double minPhi = INFINITY;	double maxPhi = -INFINITY;
	for (GLKPOSITION Pos = m_tetSupport->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetSupport->GetNodeList().GetNext(Pos);

		if (!Node->model_boundary) continue;

		if (minPhi > Node->scalarField_init) minPhi = Node->scalarField_init;
		if (maxPhi < Node->scalarField_init) maxPhi = Node->scalarField_init;
	}
	// compute max and min phi(saclar field) of boundary node of initModel
	double range = maxPhi - minPhi;
	for (GLKPOSITION Pos = m_tetSupport->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetSupport->GetNodeList().GetNext(Pos);

		Node->scalarField = 1.0 - (Node->scalarField_init - minPhi) / range;
	}

	std::cout << "Finish compute scalar field for supportSpace by hard constrain method" << std::endl;
}

void SupportGeneration::_compTetMeshVolumeMatrix(QMeshPatch* tetSupport) {

	//-- This function calculate the volume matrix 
	//   for each tetrahedral elements and installed in formate
	/* [   b1 c1 d1
		   b2 c2 d2
		   b3 c3 d3
		   b4 c4 d4   ] */

	for (GLKPOSITION Pos = tetSupport->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetSupport->GetTetraList().GetNext(Pos);

		Eigen::MatrixXd VolumeMatrix(4, 3);

		Eigen::Vector3d aa, bb, cc, dd, pp;
		Tet->CalCenterPos(pp(0), pp(1), pp(2));

		Tet->GetNodeRecordPtr(1)->GetCoord3D(aa(0), aa(1), aa(2));
		Tet->GetNodeRecordPtr(2)->GetCoord3D(bb(0), bb(1), bb(2));
		Tet->GetNodeRecordPtr(3)->GetCoord3D(cc(0), cc(1), cc(2));
		Tet->GetNodeRecordPtr(4)->GetCoord3D(dd(0), dd(1), dd(2));

		Eigen::Vector3d vap = pp - aa;
		Eigen::Vector3d vbp = pp - bb;

		Eigen::Vector3d vab = bb - aa;
		Eigen::Vector3d vac = cc - aa;
		Eigen::Vector3d vad = dd - aa;

		Eigen::Vector3d vbc = cc - bb;
		Eigen::Vector3d vbd = dd - bb;

		Eigen::Vector3d bd_bc = vbd.cross(vbc);
		Eigen::Vector3d ac_ad = vac.cross(vad);
		Eigen::Vector3d ad_ab = vad.cross(vab);
		Eigen::Vector3d ab_ac = vab.cross(vac);
		double volumeTet = Tet->CalVolume() * 6;

		VolumeMatrix.row(0) = bd_bc / volumeTet;
		VolumeMatrix.row(1) = ac_ad / volumeTet;
		VolumeMatrix.row(2) = ad_ab / volumeTet;
		VolumeMatrix.row(3) = ab_ac / volumeTet;

		Tet->VolumeMatrix = VolumeMatrix;
	}
}

void SupportGeneration::_hollow_tetSupport_generation(QMeshPatch* support_tetPatch, std::string path) {

	path += ".tet";
	// export files - hollowed tet Model file
	std::ofstream str(path);

	// record tet number
	int hollow_supportTet_NUM = 0;
	for (GLKPOSITION Pos = support_tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)support_tetPatch->GetTetraList().GetNext(Pos);

		if (!Tetra->isIn_tetSurfaceMesh) hollow_supportTet_NUM++;
	}

	// record node number
	int hollow_supportNode_Ind = 0;
	for (GLKPOSITION Pos = support_tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)support_tetPatch->GetNodeList().GetNext(Pos);

		if (Node->is_tetSupportNode) {

			Node->hollow_supportTet_Ind = hollow_supportNode_Ind;
			hollow_supportNode_Ind++;
		}
	}
	int hollow_supportNode_NUM = hollow_supportNode_Ind;

	str << hollow_supportNode_NUM << " vertices" << std::endl;
	str << hollow_supportTet_NUM << " tets" << std::endl;


	for (GLKPOSITION posNode = support_tetPatch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)support_tetPatch->GetNodeList().GetNext(posNode);

		if (!node->is_tetSupportNode) continue;

		double pp[3]; node->GetCoord3D(pp[0], pp[1], pp[2]);
		str << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
	}
	for (GLKPOSITION posTet = support_tetPatch->GetTetraList().GetHeadPosition(); posTet != nullptr;) {
		QMeshTetra* tet = (QMeshTetra*)support_tetPatch->GetTetraList().GetNext(posTet);

		if (tet->isIn_tetSurfaceMesh) continue;

		str << "4 " << tet->GetNodeRecordPtr(1)->hollow_supportTet_Ind
			<< " " << tet->GetNodeRecordPtr(2)->hollow_supportTet_Ind
			<< " " << tet->GetNodeRecordPtr(3)->hollow_supportTet_Ind
			<< " " << tet->GetNodeRecordPtr(4)->hollow_supportTet_Ind << std::endl;
	}
	str.close();
	std::cout << "Output hollowed tet model for support generation." << std::endl;
}

void SupportGeneration::update_supportTet_2_hollowed(QMeshPatch* support_tetPatch_hollowed) {

	m_tetSupport = support_tetPatch_hollowed;
	this->_index_initial(m_tetSupport, true);
}

void SupportGeneration::organize_compatibleLayer_Index(PolygenMesh* compatible_isoLayerSet, int compatibleLayer_NUM) {

	std::vector<double> compatibleLayer_isoValueVector; // naturly ordered small->large
	for (GLKPOSITION Pos = compatible_isoLayerSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)compatible_isoLayerSet->GetMeshList().GetNext(Pos);

		if (!layer->is_SupportLayer) continue;
		compatibleLayer_isoValueVector.push_back(layer->isoSurfaceValue);
	}
	//protection code
	if (compatibleLayer_isoValueVector.size() != compatibleLayer_NUM)
		std::cout << "The number of isovalue is not equal to compatibleLayer_NUM, please check." << std::endl;

	//for (int i = 0; i < compatibleLayer_isoValueVector.size(); i++){
	//	std::cout << compatibleLayer_isoValueVector[i] << " ";
	//}	std::cout << "\n";
	//std::sort(compatibleLayer_isoValueVector.begin(), compatibleLayer_isoValueVector.end());

	for (int i = 0; i < compatibleLayer_isoValueVector.size(); i++) {

		for (GLKPOSITION Pos = compatible_isoLayerSet->GetMeshList().GetHeadPosition(); Pos;) {
			QMeshPatch* layer = (QMeshPatch*)compatible_isoLayerSet->GetMeshList().GetNext(Pos);

			if (fabs(layer->isoSurfaceValue - compatibleLayer_isoValueVector[i]) < 1e-8)
				layer->compatible_layer_Index = i;
		}
	}
}

void SupportGeneration::output_compatibleLayer_4_remesh(PolygenMesh* compatible_isoLayerSet) {

	std::string unremeshed_isoLayer_dir = "../DataSet/remesh_operation/input";
	this->_remove_allFile_in_Dir(unremeshed_isoLayer_dir);

	for (GLKPOSITION posMesh = compatible_isoLayerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* each_layer = (QMeshPatch*)compatible_isoLayerSet->GetMeshList().GetNext(posMesh);

		std::string modelORsupport = "M";
		if (each_layer->is_SupportLayer) modelORsupport = "S";

		//e.g. 23_C15_M.obj
		std::string LAYER_dir = unremeshed_isoLayer_dir + "/"
			+ std::to_string(each_layer->GetIndexNo()) + "_C"
			+ std::to_string(each_layer->compatible_layer_Index) + "_"
			+ modelORsupport;
		this->_output_oneSurfaceMesh(each_layer, LAYER_dir);
	}
	std::cout << "Finish output unmeshed layers into : " << unremeshed_isoLayer_dir << std::endl;
}

int SupportGeneration::_remove_allFile_in_Dir(std::string dirPath) {

	struct _finddata_t fb;   //find the storage structure of the same properties file.
	std::string path;
	intptr_t    handle;
	int  resultone;
	int   noFile;            // the tag for the system's hidden files

	noFile = 0;
	handle = 0;

	path = dirPath + "/*";

	handle = _findfirst(path.c_str(), &fb);

	//find the first matching file
	if (handle != -1)
	{
		//find next matching file
		while (0 == _findnext(handle, &fb))
		{
			// "." and ".." are not processed
			noFile = strcmp(fb.name, "..");

			if (0 != noFile)
			{
				path.clear();
				path = dirPath + "/" + fb.name;

				//fb.attrib == 16 means folder
				if (fb.attrib == 16)
				{
					_remove_allFile_in_Dir(path);
				}
				else
				{
					//not folder, delete it. if empty folder, using _rmdir instead.
					remove(path.c_str());
				}
			}
		}
		// close the folder and delete it only if it is closed. For standard c, using closedir instead(findclose -> closedir).
		// when Handle is created, it should be closed at last.
		_findclose(handle);
		return 0;
	}
}

void SupportGeneration::update_inputMesh_4treeSkeleton_Generation(
	QMeshPatch* tetPatch, PolygenMesh* compatible_isoLayerSet, QMeshPatch* platform, PolygenMesh* supportModelSet) {

	m_tetPatch = tetPatch;
	m_layerSet = compatible_isoLayerSet;
	m_platform = platform;
	m_supportModelSet = supportModelSet;

	m_tetPatch->drawOverhangSurface = true;
	//m_platform->drawThisPatch = true;
	m_tetPatch->drawStressField = false;

	//input remeshed compatible layers
	this->_input_remeshed_compatibleLayer(m_layerSet);

	//get max compatible layer index
	int max_compatible_index = -1;
	for (GLKPOSITION posMesh = compatible_isoLayerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* each_patch = (QMeshPatch*)compatible_isoLayerSet->GetMeshList().GetNext(posMesh);

		if (each_patch->compatible_layer_Index > max_compatible_index)
			max_compatible_index = each_patch->compatible_layer_Index;
	}

	m_compatibleLayer_Num = max_compatible_index + 1;

	// the threshold of self-support angle (tau) tuned in parameter update Function
	if (tetPatch->patchName == "tree3") tau = 60.0;
	else if (tetPatch->patchName == "armadillo") tau = 60.0;
	else if (tetPatch->patchName == "fertility") tau = 60.0;
	else if (tetPatch->patchName == "flange" 
		|| tetPatch->patchName == "dome"
		|| tetPatch->patchName == "domeL") 
		tau = 50.0;
	else if (tetPatch->patchName == "topopt_new4") tau = 50.0;
	this->_markSupportFace();
	std::cout << "\n---------------------> mark overhang face with tau = " << tau << std::endl;
	// use small angle for generating tree
	tau = 50.0;//topo 45
	std::cout << "\n---------------------> tree merge with tau = " << tau << std::endl;
}

void SupportGeneration::_input_remeshed_compatibleLayer(PolygenMesh* compatible_isoLayerSet) {

	std::string remeshed_isoLayer_dir = "../DataSet/remesh_operation/output";
	std::vector<std::string> remeshedLayer_FileCell;// File name table
	this->_natSort(remeshed_isoLayer_dir, remeshedLayer_FileCell);

	//read slice files and build mesh_patches
	char file_Dir[1024];
	for (int i = 0; i < remeshedLayer_FileCell.size(); i++)
	{
		sprintf(file_Dir, "%s%s%s", remeshed_isoLayer_dir.c_str(), "/", remeshedLayer_FileCell[i].data());
		//cout << file_Dir << endl;

		QMeshPatch* slice = new QMeshPatch;
		slice->SetIndexNo(compatible_isoLayerSet->GetMeshList().GetCount()); //index begin from 0
		compatible_isoLayerSet->GetMeshList().AddTail(slice);
		slice->patchName = remeshedLayer_FileCell[i].data();

		// is_SupportLayer
		std::string::size_type supportFlag = remeshedLayer_FileCell[i].find("S");
		if (supportFlag == std::string::npos)	slice->is_SupportLayer = false;
		else	slice->is_SupportLayer = true;

		// get compatible layer index
		std::string::size_type p = remeshedLayer_FileCell[i].find('_');
		std::string::size_type pp = remeshedLayer_FileCell[i].find('_', p + 2);
		slice->compatible_layer_Index = stoi(remeshedLayer_FileCell[i].substr(p + 2, pp - p - 1));
		std::cout << "remeshedLayer_FileCell[i]: " << remeshedLayer_FileCell[i] << " layer index: " << slice->GetIndexNo()
			<< " layer compatible index: " << slice->compatible_layer_Index << std::endl;

		slice->inputOBJFile(file_Dir);

		////move slice up with 8 for airbus_topopt case
		//if (m_tetPatch->patchName == "airbus_topopt") {
		//	double upDist = 8.0;
		//	double pp[3] = { 0.0,0.0,0.0 };
		//	for (GLKPOSITION Pos = slice->GetNodeList().GetHeadPosition(); Pos;) {
		//		QMeshNode* Node = (QMeshNode*)slice->GetNodeList().GetNext(Pos);
		//		Node->GetCoord3D_last(pp[0], pp[1], pp[2]);

		//		pp[1] += 8.0;

		//		Node->SetCoord3D(pp[0], pp[1], pp[2]);
		//		Node->SetCoord3D_last(pp[0], pp[1], pp[2]);
		//	}
		//}
	}
}

void SupportGeneration::_natSort(std::string dirctory, std::vector<std::string>& fileNameCell) {

	if (fileNameCell.empty() == false) return;

	DIR* dp;
	struct dirent* ep;
	std::string fullDir = dirctory;
	//cout << fullDir << endl;
	dp = opendir(fullDir.c_str());
	//dp = opendir("../Waypoints");

	if (dp != NULL) {
		while (ep = readdir(dp)) {
			//cout << ep->d_name << endl;
			if ((std::string(ep->d_name) != ".") && (std::string(ep->d_name) != "..")) {
				//cout << ep->d_name << endl;
				fileNameCell.push_back(std::string(ep->d_name));
			}
		}
		(void)closedir(dp);
	}
	else {
		perror("Couldn't open the directory");
	}
	//resort the files with nature order
	sort(fileNameCell.begin(), fileNameCell.end(), doj::alphanum_less<std::string>());

}

void SupportGeneration::build_support_tree() {

	//calculate the direction of support Ray; by laplacian smoothness
	this->_cal_Ray_direction(m_tetPatch);

	//build a new patch for organizing support Tree
	QMeshPatch* supportRay_patch = new QMeshPatch;
	supportRay_patch->SetIndexNo(m_supportModelSet->GetMeshList().GetCount()); //index begin from 0
	m_supportModelSet->GetMeshList().AddTail(supportRay_patch);

	//get compatible layer set table ( i | QmeshPatch* initial | QmeshPatch* support )
	std::vector<std::vector<QMeshPatch*>> compatibleLayer_matrix;
	_get_compatibleLayer_list(compatibleLayer_matrix);

	//for (int m = compatibleLayer_matrix.size() - 1; m >= 0; m--) { // from top to bottom
	//	std::cout << "detect layer index m " << m << "\n";
	//	for (int n = 0; n < compatibleLayer_matrix[m].size(); n++)
	//		std::cout << " n " << n 
	//		<< " layer_index " << compatibleLayer_matrix[m][n]->GetIndexNo()
	//		<< " compatible_index " << compatibleLayer_matrix[m][n]->compatible_layer_Index << std::endl;
	//}

	//reconstruct the Node Table for omp usage (only step 1); as Face write the treeNode, omp cannot be used
	std::vector<QMeshNode*> tet_surfaceNode_set;
	for (GLKPOSITION Pos = m_tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetPatch->GetNodeList().GetNext(Pos);
		// the inner node is not considered
		if (Node->inner) continue;
		if (Node->need_Support == false) continue;

		tet_surfaceNode_set.push_back(Node);
	}
	std::cout << "tree_Node_Num = " << tet_surfaceNode_set.size() << std::endl;

	// 1. find the 1st drop point for Node (needs support) on TET surface
#pragma omp parallel
	{
#pragma omp for 
		for (int i = 0; i < tet_surfaceNode_set.size(); i++) {
			QMeshNode* Node = tet_surfaceNode_set[i];

			//global variable: initial value
			Eigen::Vector3d oringinNode; Node->GetCoord3D(oringinNode[0], oringinNode[1], oringinNode[2]);
			Eigen::Vector3d step_Direction = Node->supportRay_Dir; //from tet surface normal

			//std::cout << "oringinNode " << oringinNode << std::endl;
			//std::cout << "step_Direction " << step_Direction << std::endl;

			bool get_1st_intersectPnt = false;
			// 1. find the 1st intersection between polyline and support layer (OR tetSurface)
			for (int m = compatibleLayer_matrix.size() - 1; m >= 0; m--) { // from top to bottom
				for (int n = 0; n < compatibleLayer_matrix[m].size(); n++) {
					QMeshPatch* detect_layer = compatibleLayer_matrix[m][n];

					Eigen::Vector3d insertP, v1, v2, v3;
					for (GLKPOSITION Pos = detect_layer->GetFaceList().GetHeadPosition(); Pos;) {
						QMeshFace* Face = (QMeshFace*)detect_layer->GetFaceList().GetNext(Pos);

						/*speeding up by distance limit*/
						// when the distance bewteen face and originNode is too far than 5mm(temporary), skip it;
						Face->CalCenterPos();
						Eigen::Vector3d center; Face->GetCenterPos(center[0], center[1], center[2]);
						if ((center - oringinNode).norm() > 5) continue;

						Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
						Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
						Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

						if (_intersetTriangle(oringinNode, step_Direction, v1, v2, v3, insertP)) {	/*intersect Yes*/
							/* calculate t +/-? : insertP = orig + t * dir; */
							//use y dir for plannar case, as the numarical error for x dir is very large
							// || false below is used to debug(remove)
							if (((insertP - oringinNode)[1] / step_Direction[1] > 0.01)) {			/*next layer along the "+" direction of vectorField*/
								get_1st_intersectPnt = true;
								if (detect_layer->is_SupportLayer) {								/*only record the support layer intersection*/

									//create souceNode
									QMeshNode* sourceNode = new QMeshNode;
									sourceNode->is_Processed = true;
									sourceNode->SetCoord3D(oringinNode[0], oringinNode[1], oringinNode[2]);
									sourceNode->SetMeshPatchPtr(supportRay_patch);
									sourceNode->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
									supportRay_patch->GetNodeList().AddTail(sourceNode);

									//create one TreeNode
									QMeshNode* TreeNode = new QMeshNode;
									TreeNode->SetCoord3D(insertP[0], insertP[1], insertP[2]);
									TreeNode->is_Host = true;
									TreeNode->is_Processed = false;
									TreeNode->treeNode_belonging_Face = Face;
									TreeNode->descend_To_TreeNode_Dir = step_Direction;
									TreeNode->treeNode_before_branch_NUM = 1;
									TreeNode->SetMeshPatchPtr(supportRay_patch);
									TreeNode->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
									supportRay_patch->GetNodeList().AddTail(TreeNode);
									Face->support_treeNode_onFace.push_back(TreeNode);

									//create one TreeEdge
									QMeshEdge* TreeEdge = new QMeshEdge;
									TreeEdge->treeEdge_height = m;
									TreeEdge->treeEdge_branch_NUM = 1;
									TreeEdge->SetStartPoint(sourceNode);
									TreeEdge->SetEndPoint(TreeNode);
									TreeEdge->SetMeshPatchPtr(supportRay_patch);
									TreeEdge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0
									supportRay_patch->GetEdgeList().AddTail(TreeEdge);
								}
								break;
							}
						}
					}
					if (get_1st_intersectPnt)break;
				}
				if (get_1st_intersectPnt)break;
			}
		}
	}
	std::cout << "Finish: 1. find the 1st drop point for Node (needs support) on TET surface." << std::endl;

	// 2. tree-like structure generation: dynamic host node
	for (int m = compatibleLayer_matrix.size() - 1; m > 0; m--) { // from top to bottom
		for (int n = 0; n < compatibleLayer_matrix[m].size(); n++) {
			QMeshPatch* detected_support_layer = compatibleLayer_matrix[m][n];
			if (detected_support_layer->is_SupportLayer == false)	continue;
	
			//faces on TOP layer which contain treeNode
			for (GLKPOSITION Pos = detected_support_layer->GetFaceList().GetHeadPosition(); Pos;) {
				QMeshFace* Face = (QMeshFace*)detected_support_layer->GetFaceList().GetNext(Pos);
	
				// skip when no treeNode on Face or all treeNode are processed on Face
				if (Face->support_treeNode_onFace.size() == 0)	continue;
				else {
					bool all_processed = true;
					for (int s = 0; s < Face->support_treeNode_onFace.size(); s++) {
						if (Face->support_treeNode_onFace[s]->is_Processed == false) {
							all_processed = false;
							break;
						}
					}
					if (all_processed)	continue;
				}
	
				/*Main Function of tree generation*/
				// treeNode on the same Face
				// --> find the host treeNode index
				int treeNode_index = -1;// the tree node with the most large of branch_NUM is defined as "hostNode"
				int max_treeNode_NUM = -1;
				for (int i = 0; i < Face->support_treeNode_onFace.size(); i++) {
					if (Face->support_treeNode_onFace[i]->treeNode_before_branch_NUM > max_treeNode_NUM)
						max_treeNode_NUM = Face->support_treeNode_onFace[i]->treeNode_before_branch_NUM;
					treeNode_index = i;
				}
				if (treeNode_index == -1 || max_treeNode_NUM == -1) // for safe
					std::cout << "Error: cannot find the treeNode with max branch NUM!" << std::endl;
	
				// --> find the dropNode for hostNode
				QMeshNode* hostNode = Face->support_treeNode_onFace[treeNode_index];
				/*IMPORTANT: bool: weather incline the projection of hostnode*/
				_compute_descend_Dir_hostNode(hostNode, hostNode_decent_alongNormal);
				QMeshNode* dropNode_hostNode = 
					_build_host_treeNodeEdge(hostNode, compatibleLayer_matrix[m - 1], supportRay_patch, (m - 1));
				hostNode->is_Host = true;
				if (dropNode_hostNode == NULL) std::cout << "The dropNode of hostNode should not be NULL!" << std::endl;
	
				// --> find the dropNode for followNode on the same face;
				for (int i = 0; i < Face->support_treeNode_onFace.size(); i++) {
					if (i == treeNode_index)	continue;
					QMeshNode* followNode = Face->support_treeNode_onFace[i];
					followNode->is_Host = false;
					bool merge = _compute_descend_Dir_followNode(followNode, dropNode_hostNode);
					_build_follow_treeNodeEdge(followNode, dropNode_hostNode, compatibleLayer_matrix[m - 1], merge, supportRay_patch, (m - 1));
				}
	
				///***************** Add 1-ring neighbor into faceSet *****************/
				std::vector<QMeshFace*> faceSet_ring;
				// --> collect 1-ring face
				for (int i_node = 0; i_node < 3; i_node++) {
					for (GLKPOSITION Pos_1ringFace = Face->GetNodeRecordPtr(i_node)->GetFaceList().GetHeadPosition(); Pos_1ringFace;) {
						QMeshFace* Face_1ring = (QMeshFace*)(Face->GetNodeRecordPtr(i_node)->GetFaceList().GetNext(Pos_1ringFace));
	
						if (Face_1ring == Face) continue; // not include self
	
						//1-ring face may repeat, avoid repeating
						bool is_exist = false;
						for (int j = 0; j < faceSet_ring.size(); j++) {
							if (faceSet_ring[j] == Face_1ring) {
								is_exist = true;
								break;
							}
						}
						if (is_exist == false) { faceSet_ring.push_back(Face_1ring); }
					}
				}
				//std::cout << "Face index: " << Face->GetIndexNo(); //right
				//std::cout << " faceSet_ring Num : " << faceSet_ring.size() << std::endl; // right
				/***************** Add 2-ring neighbor into faceSet *****************/
				// --> collect 2-ring face
				int NUM_1ring_treeNode = 0;
				for (int i_face = 0; i_face < faceSet_ring.size(); i_face++) {
					for (int j_node = 0; j_node < faceSet_ring[i_face]->support_treeNode_cell.size(); j_node++) {
						if (faceSet_ring[i_face]->support_treeNode_cell[j_node].isProcessed == false) {
							NUM_1ring_treeNode++;
						}
					}
				}
				//std::cout << "NUM_1ring_treeNode: " << NUM_1ring_treeNode << std::endl;
				int faceSet_1ring_Num = faceSet_ring.size();// record the num of 1-ring faces
				if (NUM_1ring_treeNode == 0) {
	
					for (int index_1ring_face = 0; index_1ring_face < faceSet_1ring_Num; index_1ring_face++) {
						for (int i_node = 0; i_node < 3; i_node++) {
	
							for (GLKPOSITION Pos_2ringFace = faceSet_ring[index_1ring_face]->GetNodeRecordPtr(i_node)->GetFaceList().GetHeadPosition(); Pos_2ringFace;) {
								QMeshFace* Face_2ring = (QMeshFace*)(faceSet_ring[index_1ring_face]->GetNodeRecordPtr(i_node)->GetFaceList().GetNext(Pos_2ringFace));
	
								if (Face_2ring == Face) continue; // not include self
	
								//2-ring face may repeat, avoid repeating
								bool is_exist = false;
								for (int j = 0; j < faceSet_ring.size(); j++) {
									if (faceSet_ring[j] == Face_2ring) {
										is_exist = true;
										break;
									}
								}
								if (is_exist == false) { faceSet_ring.push_back(Face_2ring); }
							}
						}
					}
					//std::cout << " faceSet_2ring Num : " << faceSet_ring.size();
				}
				/***************** Add 3-ring neighbor into faceSet *****************/
				// --> collect 3-ring face
				int NUM_2ring_treeNode = 0;
				for (int i_face = 0; i_face < faceSet_ring.size(); i_face++) {
					for (int j_node = 0; j_node < faceSet_ring[i_face]->support_treeNode_cell.size(); j_node++) {
						if (faceSet_ring[i_face]->support_treeNode_cell[j_node].isProcessed == false) {
							NUM_2ring_treeNode++;
						}
					}
				}
				//std::cout << "NUM_2ring_treeNode: " << NUM_2ring_treeNode << std::endl;
				int faceSet_2ring_Num = faceSet_ring.size();// record the num of 2-ring faces (faceSet_ring.size() will change after add 2-ring Faces)
				if (NUM_2ring_treeNode == 0) {
	
					for (int index_2ring_face = faceSet_1ring_Num; index_2ring_face < faceSet_2ring_Num; index_2ring_face++) {
						for (int i_node = 0; i_node < 3; i_node++) {
	
							for (GLKPOSITION Pos_3ringFace = faceSet_ring[index_2ring_face]->GetNodeRecordPtr(i_node)->GetFaceList().GetHeadPosition(); Pos_3ringFace;) {
								QMeshFace* Face_3ring = (QMeshFace*)(faceSet_ring[index_2ring_face]->GetNodeRecordPtr(i_node)->GetFaceList().GetNext(Pos_3ringFace));
	
								if (Face_3ring == Face) continue; // not include self
	
								//3-ring face may repeat, avoid repeating
								bool is_exist = false;
								for (int j = 0; j < faceSet_ring.size(); j++) {
									if (faceSet_ring[j] == Face_3ring) {
										is_exist = true;
										break;
									}
								}
								if (is_exist == false) { faceSet_ring.push_back(Face_3ring); }
							}
						}
					}
					//std::cout << " faceSet_3ring Num : " << faceSet_ring.size();
				}
				/***************** Add 4-ring neighbor into faceSet *****************/
				// --> collect 4-ring face
				int NUM_3ring_treeNode = 0;
				for (int i_face = 0; i_face < faceSet_ring.size(); i_face++) {
					for (int j_node = 0; j_node < faceSet_ring[i_face]->support_treeNode_cell.size(); j_node++) {
						if (faceSet_ring[i_face]->support_treeNode_cell[j_node].isProcessed == false) {
							NUM_3ring_treeNode++;
						}
					}
				}
				//std::cout << "NUM_3ring_treeNode: " << NUM_3ring_treeNode << std::endl;
				int faceSet_3ring_Num = faceSet_ring.size();// record the num of 2-ring faces (faceSet_ring.size() will change after add 2-ring Faces)
				if (NUM_3ring_treeNode == 0) {
	
					for (int index_3ring_face = faceSet_2ring_Num; index_3ring_face < faceSet_3ring_Num; index_3ring_face++) {
						for (int i_node = 0; i_node < 3; i_node++) {
	
							for (GLKPOSITION Pos_4ringFace = faceSet_ring[index_3ring_face]->GetNodeRecordPtr(i_node)->GetFaceList().GetHeadPosition(); Pos_4ringFace;) {
								QMeshFace* Face_4ring = (QMeshFace*)(faceSet_ring[index_3ring_face]->GetNodeRecordPtr(i_node)->GetFaceList().GetNext(Pos_4ringFace));
	
								if (Face_4ring == Face) continue; // not include self
	
								//4-ring face may repeat, avoid repeating
								bool is_exist = false;
								for (int j = 0; j < faceSet_ring.size(); j++) {
									if (faceSet_ring[j] == Face_4ring) {
										is_exist = true;
										break;
									}
								}
								if (is_exist == false) { faceSet_ring.push_back(Face_4ring); }
							}
						}
					}
					//std::cout << " faceSet_4ring Num : " << faceSet_ring.size();
				}
				//std::cout << std::endl;
	
				/******************************* END ********************************/
	
				// --> compute dropNode for 1/2/3/4-ring treeNode
				for (int i_face = 0; i_face < faceSet_ring.size(); i_face++) {
					for (int j_node = 0; j_node < faceSet_ring[i_face]->support_treeNode_onFace.size(); j_node++) {
	
						QMeshNode* followNode_ring = faceSet_ring[i_face]->support_treeNode_onFace[j_node];
	
						if (followNode_ring->is_Processed == false) {
							followNode_ring->is_Host = false;// 1/2/3-ring neighbor are all followNode
							bool merge = _compute_descend_Dir_followNode(followNode_ring, dropNode_hostNode);
							_build_follow_treeNodeEdge(followNode_ring, dropNode_hostNode, compatibleLayer_matrix[m - 1], merge, supportRay_patch, (m - 1));
						}
					}
				}
			}
		}
		std::cout << ".";
		if ((compatibleLayer_matrix.size() - m) % 100 == 0 || (m == 1)) std::cout << std::endl;
	}

	//get the max branch NUM of treeEdge
	int max_branch_NUM = -1;
	for (GLKPOSITION Pos = supportRay_patch->GetEdgeList().GetHeadPosition(); Pos != NULL;) {
		QMeshEdge* edge = (QMeshEdge*)(supportRay_patch->GetEdgeList().GetNext(Pos));

		if (edge->treeEdge_branch_NUM > max_branch_NUM)	max_branch_NUM = edge->treeEdge_branch_NUM;
	}
	std::cout << "max branch NUM = " << max_branch_NUM << std::endl;
	supportRay_patch->max_branch_NUM = max_branch_NUM;

	//get the max height of treeEdge
	int max_height = -1;
	for (GLKPOSITION Pos = supportRay_patch->GetEdgeList().GetHeadPosition(); Pos != NULL;) {
		QMeshEdge* edge = (QMeshEdge*)(supportRay_patch->GetEdgeList().GetNext(Pos));

		if (edge->treeEdge_height > max_height)	max_height = edge->treeEdge_height;
	}
	std::cout << "max height = " << max_height << std::endl;
	supportRay_patch->max_height = max_height;

	std::cout << "treeEdge NUM = " << supportRay_patch->GetEdgeNumber() << std::endl;
	std::cout << "Finish: 2. find the drop points for tree node (layer by layer) --> tree-like." << std::endl;
}

void SupportGeneration::_get_compatibleLayer_list(std::vector<std::vector<QMeshPatch*>>& compatibleLayer_matrix) {

	//get maximum compatible_layer_Index
	int max_compatibleLayer_index = 0;
	for (GLKPOSITION posMesh = m_layerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* Patch = (QMeshPatch*)m_layerSet->GetMeshList().GetNext(posMesh);

		if (Patch->compatible_layer_Index > max_compatibleLayer_index)
			max_compatibleLayer_index = Patch->compatible_layer_Index;
	}
	int compatibleLayer_NUM = max_compatibleLayer_index + 1;
	compatibleLayer_matrix.resize(compatibleLayer_NUM);

	for (int i = 0; i < compatibleLayer_NUM; i++) {

		QMeshPatch* initial_layer_patch = NULL;
		QMeshPatch* support_layer_patch = NULL;

		for (GLKPOSITION posMesh = m_layerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshPatch* Patch = (QMeshPatch*)m_layerSet->GetMeshList().GetNext(posMesh);

			if (Patch->compatible_layer_Index == i) {
				if (Patch->is_SupportLayer)	support_layer_patch = Patch;
				else initial_layer_patch = Patch;
			}
		}

		if (initial_layer_patch == NULL && support_layer_patch != NULL) {		// only support

			compatibleLayer_matrix[i].push_back(support_layer_patch);
		}
		else if (initial_layer_patch != NULL && support_layer_patch != NULL) {	// all

			compatibleLayer_matrix[i].push_back(initial_layer_patch);
			compatibleLayer_matrix[i].push_back(support_layer_patch);
		}
		else if (initial_layer_patch != NULL && support_layer_patch == NULL) {	// only initial
			compatibleLayer_matrix[i].push_back(initial_layer_patch);
		}
		else {
			std::cout <<"initial and support layer are all NULL, please check!!" << std::endl;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Line: orig + ->dir; triangle: v0, v1, v2; insertP: intersection point
// reference: https://www.cnblogs.com/graphics/archive/2010/08/09/1795348.html
bool SupportGeneration::_intersetTriangle(Eigen::Vector3d& orig, Eigen::Vector3d& dir,
	Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2, Eigen::Vector3d& insertP) {

	double t, u, v;

	// E1,E2,P
	Eigen::Vector3d E1 = v1 - v0;
	Eigen::Vector3d E2 = v2 - v0;
	Eigen::Vector3d P = dir.cross(E2);

	// determinant
	double det = E1.dot(P);

	// keep det > 0, modify T accordingly
	Eigen::Vector3d T;
	if (det > 0)
	{
		T = orig - v0;
	}
	else
	{
		T = v0 - orig;
		det = -det;
	}

	// If determinant is near zero, ray lies in plane of triangle
	if (det < 0.0001) {
		//std::cout << "this node lies in plane!!!" << std::endl;
		return false;
	}

	// Calculate u and make sure u <= 1
	u = T.dot(P);
	if (u < 0.0f || u > det)
		//if (u < -0.5f || u > det*1.5)
		return false;

	// Q
	Eigen::Vector3d Q = T.cross(E1);

	// Calculate v and make sure u + v <= 1
	v = dir.dot(Q);
	if (v < 0.0f || u + v > det)
		//if (v < -0.5f || u + v > det*1.5)
		return false;

	// Calculate t, scale parameters, ray intersects triangle
	t = E2.dot(Q);

	float fInvDet = 1.0f / det;
	t *= fInvDet;
	u *= fInvDet;
	v *= fInvDet;

	insertP = orig + t * dir;
	//std::cout << insertP << endl;
	//std::cout << (1 - u - v)*v0 + u*v1 + v*v2 << endl << endl;

	return true;
}

void SupportGeneration::_compute_descend_Dir_hostNode(QMeshNode* hostNode, bool is_downWard) {

	//incline a self-supporting angle to the z direction
	Eigen::Vector3d descend_dir = { 0.0,-1.0,0.0 };
	Eigen::Vector3d dir_downward = { 0.0,-1.0,0.0 };
	Eigen::Vector3d dir_tilt; 
	hostNode->GetCoord3D(dir_tilt[0], dir_tilt[1], dir_tilt[2]); 
	dir_tilt = -dir_tilt.normalized();

	if (is_downWard) {	descend_dir = dir_downward;	}
	else { descend_dir = dir_tilt;	}

	//incremental decent
	//double alpha = (double)(hostNode->GetMeshPatchPtr()->compatible_layer_Index) / m_compatibleLayer_Num;
	//std::cout << "alpha = " << alpha << std::endl;
	//if (alpha > 1.0) std::cout << "alpha should not larger than 1.0, please check." << std::endl;
	//descend_dir = (1.0 - alpha)* dir_downward + alpha* dir_tilt;
	//descend_dir.normalize();

	Eigen::Vector3d face_normal; 
	hostNode->treeNode_belonging_Face->CalPlaneEquation();
	hostNode->treeNode_belonging_Face->GetNormal(face_normal[0], face_normal[1], face_normal[2]);
	face_normal.normalize();
	
	Eigen::Vector3d rotateAxis = face_normal.cross(descend_dir);

	//angle between _dir and _descend_dir [0,PI]
	double radian_angle = atan2(face_normal.cross(descend_dir).norm(), face_normal.transpose() * descend_dir);
	if (ROTATE_TO_DEGREE(radian_angle) < tau)
		hostNode->descend_From_TreeNode_Dir = descend_dir; // in the range of support angle, directly downward
	else {
		// only rotate a support angle
		Eigen::AngleAxisd V(DEGREE_TO_ROTATE(tau), rotateAxis);//rotateAxis£¬rotate tau(deg)
		hostNode->descend_From_TreeNode_Dir = V * face_normal;
	}
	//hostNode->descend_From_TreeNode_Dir = face_normal;//tianyu test going
}

QMeshNode* SupportGeneration::_build_host_treeNodeEdge(
	QMeshNode* hostNode,
	const std::vector<QMeshPatch*>& largeLayer_vector_m_1,
	QMeshPatch* supportRay_patch,
	int layer_height) {

	bool isIntersect = false;
	Eigen::Vector3d treeNode_coord3D, step_Direction_hostNode;
	QMeshNode* host_dropNode = NULL;

	hostNode->GetCoord3D(treeNode_coord3D[0], treeNode_coord3D[1], treeNode_coord3D[2]);
	step_Direction_hostNode = hostNode->descend_From_TreeNode_Dir;

	// directly drop the node to the bottom when the height is less than threshold
	if (treeNode_coord3D[1] < bottomHeight_threshold) {

		//create one TreeNode on base
		QMeshNode* bottom_TreeNode = new QMeshNode;
		bottom_TreeNode->SetCoord3D(treeNode_coord3D[0], -2.0, treeNode_coord3D[2]);// -2mm: go into the bottom platform
		bottom_TreeNode->is_Host = false;
		bottom_TreeNode->is_Processed = true;   hostNode->is_Processed = true;
		bottom_TreeNode->descend_To_TreeNode_Dir = step_Direction_hostNode;
		bottom_TreeNode->treeNode_before_branch_NUM = hostNode->treeNode_before_branch_NUM;
		bottom_TreeNode->SetMeshPatchPtr(supportRay_patch);
		bottom_TreeNode->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
		supportRay_patch->GetNodeList().AddTail(bottom_TreeNode);
		host_dropNode = bottom_TreeNode;

		//create one TreeEdge
		QMeshEdge* bottom_TreeEdge = new QMeshEdge;
		bottom_TreeEdge->treeEdge_branch_NUM = hostNode->treeNode_before_branch_NUM;
		bottom_TreeEdge->treeEdge_height = layer_height; // same as the layer contain endNode
		bottom_TreeEdge->SetStartPoint(hostNode);
		bottom_TreeEdge->SetEndPoint(bottom_TreeNode);
		bottom_TreeEdge->SetMeshPatchPtr(supportRay_patch);
		bottom_TreeEdge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0
		supportRay_patch->GetEdgeList().AddTail(bottom_TreeEdge);

		return host_dropNode;
	}

	for (int n = 0; n < largeLayer_vector_m_1.size(); n++) {
		QMeshPatch* bellow_layer = largeLayer_vector_m_1[n];

		Eigen::Vector3d insertP, v1, v2, v3;

		for (GLKPOSITION Pos = bellow_layer->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)bellow_layer->GetFaceList().GetNext(Pos);

			/*speeding up by distance limit*/
			// when the distance bewteen face and originNode is too far than 5mm(temporary), skip it;
			Eigen::Vector3d center; Face->GetCenterPos(center[0], center[1], center[2]);
			if ((center - treeNode_coord3D).norm() > 5.0) continue;

			Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
			Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
			Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

			if (_intersetTriangle(treeNode_coord3D, step_Direction_hostNode, v1, v2, v3, insertP)) {

				isIntersect = true;
				//create one TreeNode on lower layer's face
				QMeshNode* TreeNode = new QMeshNode;
				TreeNode->SetCoord3D(insertP[0], insertP[1], insertP[2]);
				TreeNode->is_Host = false;
				TreeNode->is_Processed = false;
				TreeNode->treeNode_belonging_Face = Face;
				TreeNode->descend_To_TreeNode_Dir = step_Direction_hostNode;
				TreeNode->treeNode_before_branch_NUM = hostNode->treeNode_before_branch_NUM;
				TreeNode->SetMeshPatchPtr(supportRay_patch);
				TreeNode->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
				supportRay_patch->GetNodeList().AddTail(TreeNode);
				Face->support_treeNode_onFace.push_back(TreeNode);
				host_dropNode = TreeNode;

				//create one TreeEdge
				QMeshEdge* TreeEdge = new QMeshEdge;
				TreeEdge->treeEdge_height = layer_height;
				TreeEdge->treeEdge_branch_NUM = hostNode->treeNode_before_branch_NUM;
				TreeEdge->SetStartPoint(hostNode);
				TreeEdge->SetEndPoint(TreeNode);
				TreeEdge->SetMeshPatchPtr(supportRay_patch);
				TreeEdge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0
				supportRay_patch->GetEdgeList().AddTail(TreeEdge);

				break;
			}
		}
		if (isIntersect) break;
	}

	if (isIntersect == false) {

		QMeshNode* TreeNode_virtual = new QMeshNode;
		Eigen::Vector3d drop_Node_virtual_host = treeNode_coord3D + 1.0 * step_Direction_hostNode; // 1.0 means the virtual length
		TreeNode_virtual->SetCoord3D(drop_Node_virtual_host[0], drop_Node_virtual_host[1], drop_Node_virtual_host[2]);

		TreeNode_virtual->is_Host = true;
		TreeNode_virtual->is_Processed = false;
		TreeNode_virtual->is_virtual_Host = true;
		TreeNode_virtual->descend_To_TreeNode_Dir = step_Direction_hostNode;
		TreeNode_virtual->treeNode_before_branch_NUM = hostNode->treeNode_before_branch_NUM;
		TreeNode_virtual->SetMeshPatchPtr(supportRay_patch);
		TreeNode_virtual->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
		supportRay_patch->GetNodeList().AddTail(TreeNode_virtual);
		//std::cout << "force the host node have a virtual drop node" << std::endl;
		host_dropNode = TreeNode_virtual;
	}

	// tag --> the hostNode is processed
	hostNode->is_Processed = true;

	return host_dropNode;
}

bool SupportGeneration::_compute_descend_Dir_followNode(QMeshNode* followNode, QMeshNode* dropNode_hostNode) {

	//followNode .
	//            \
	//            _\|
	//              .dropNode_hostNode

	bool is_merged = false;
	//incline a self-supporting angle to the z direction
	Eigen::Vector3d dropNode_hostHost_Coord3D;
	dropNode_hostNode->GetCoord3D(dropNode_hostHost_Coord3D[0], dropNode_hostHost_Coord3D[1], dropNode_hostHost_Coord3D[2]);
	Eigen::Vector3d followNode_coord3D;
	followNode->GetCoord3D(followNode_coord3D[0], followNode_coord3D[1], followNode_coord3D[2]);

	Eigen::Vector3d descend_dir = (dropNode_hostHost_Coord3D - followNode_coord3D).normalized();

	Eigen::Vector3d face_normal;
	followNode->treeNode_belonging_Face->CalPlaneEquation();
	followNode->treeNode_belonging_Face->GetNormal(face_normal[0], face_normal[1], face_normal[2]);
	face_normal.normalize();

	Eigen::Vector3d rotateAxis = face_normal.cross(descend_dir);

	double radian_angle = atan2(face_normal.cross(descend_dir).norm(), face_normal.transpose() * descend_dir);
	if (ROTATE_TO_DEGREE(radian_angle) < tau) {
		followNode->descend_From_TreeNode_Dir = descend_dir; //in the range of support angle
		is_merged = true;
	}
	else {
		// only rotate a support angle
		Eigen::AngleAxisd V(DEGREE_TO_ROTATE(tau), rotateAxis);	//rotateAxis£¬rotate tau(deg)
		followNode->descend_From_TreeNode_Dir = V * face_normal;
		is_merged = false;
	}

	//protect for safe: merged point cannot occur on the virtual hostNode
	if (dropNode_hostNode->is_virtual_Host) is_merged = false;

	return is_merged;
}

void SupportGeneration::_build_follow_treeNodeEdge(
	QMeshNode* followNode,
	QMeshNode* dropNode_hostNode,
	const std::vector<QMeshPatch*>& largeLayer_vector_m_1,
	bool is_merge,
	QMeshPatch* supportRay_patch,
	int layer_height) {

	bool isIntersect = false;
	Eigen::Vector3d treeNode_coord3D, step_Direction_followNode;
	followNode->GetCoord3D(treeNode_coord3D[0], treeNode_coord3D[1], treeNode_coord3D[2]);
	step_Direction_followNode = followNode->descend_From_TreeNode_Dir;

	if (treeNode_coord3D[1] < bottomHeight_threshold) {

		//create one TreeNode on base
		QMeshNode* bottom_TreeNode = new QMeshNode;
		bottom_TreeNode->SetCoord3D(treeNode_coord3D[0], -2.0, treeNode_coord3D[2]);//-2mm for reasonable bottom
		bottom_TreeNode->is_Host = false;
		bottom_TreeNode->is_Processed = true;	followNode->is_Processed = true;
		bottom_TreeNode->descend_To_TreeNode_Dir = step_Direction_followNode;
		bottom_TreeNode->treeNode_before_branch_NUM = followNode->treeNode_before_branch_NUM;
		bottom_TreeNode->SetMeshPatchPtr(supportRay_patch);
		bottom_TreeNode->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
		supportRay_patch->GetNodeList().AddTail(bottom_TreeNode);

		//create one TreeEdge
		QMeshEdge* bottom_TreeEdge = new QMeshEdge;
		bottom_TreeEdge->treeEdge_height = layer_height;
		bottom_TreeEdge->treeEdge_branch_NUM = followNode->treeNode_before_branch_NUM;
		bottom_TreeEdge->SetStartPoint(followNode);
		bottom_TreeEdge->SetEndPoint(bottom_TreeNode);
		bottom_TreeEdge->SetMeshPatchPtr(supportRay_patch);
		bottom_TreeEdge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0
		supportRay_patch->GetEdgeList().AddTail(bottom_TreeEdge);

		return;
	}

	for (int n = 0; n < largeLayer_vector_m_1.size(); n++) {
		QMeshPatch* bellow_layer = largeLayer_vector_m_1[n];

		Eigen::Vector3d insertP, v1, v2, v3;


		for (GLKPOSITION Pos = bellow_layer->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)bellow_layer->GetFaceList().GetNext(Pos);

			/*speeding up by distance limit*/
			// when the distance bewteen face and originNode is too far than 5mm(temporary), skip it;
			Eigen::Vector3d center; Face->GetCenterPos(center[0], center[1], center[2]);
			if ((center - treeNode_coord3D).norm() > 5.0) continue;

			Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
			Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
			Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

			if (_intersetTriangle(treeNode_coord3D, step_Direction_followNode, v1, v2, v3, insertP)) {

				isIntersect = true;
				//create one TreeNode on lower layer's face
				if (is_merge) {
					//merge only create TreeEdge
					QMeshEdge* TreeEdge_merge = new QMeshEdge;
					TreeEdge_merge->treeEdge_height = layer_height;
					TreeEdge_merge->treeEdge_branch_NUM = followNode->treeNode_before_branch_NUM;
					TreeEdge_merge->SetStartPoint(followNode);
					TreeEdge_merge->SetEndPoint(dropNode_hostNode);
					TreeEdge_merge->SetMeshPatchPtr(supportRay_patch);
					TreeEdge_merge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0
					supportRay_patch->GetEdgeList().AddTail(TreeEdge_merge);

					dropNode_hostNode->treeNode_before_branch_NUM += followNode->treeNode_before_branch_NUM;

				}
				else {
					QMeshNode* TreeNode = new QMeshNode;
					TreeNode->SetCoord3D(insertP[0], insertP[1], insertP[2]);
					TreeNode->is_Host = false;
					TreeNode->is_Processed = false;
					TreeNode->treeNode_belonging_Face = Face;
					TreeNode->descend_To_TreeNode_Dir = step_Direction_followNode;
					TreeNode->treeNode_before_branch_NUM = followNode->treeNode_before_branch_NUM;
					TreeNode->SetMeshPatchPtr(supportRay_patch);
					TreeNode->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
					supportRay_patch->GetNodeList().AddTail(TreeNode);
					Face->support_treeNode_onFace.push_back(TreeNode);

					//create one TreeEdge
					QMeshEdge* TreeEdge = new QMeshEdge;
					TreeEdge->treeEdge_height = layer_height;
					TreeEdge->treeEdge_branch_NUM = followNode->treeNode_before_branch_NUM;
					TreeEdge->SetStartPoint(followNode);
					TreeEdge->SetEndPoint(TreeNode);
					TreeEdge->SetMeshPatchPtr(supportRay_patch);
					TreeEdge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0
					supportRay_patch->GetEdgeList().AddTail(TreeEdge);
				}

				break;
			}
		}
		if (isIntersect) break;
	}
	// tag --> the followNode is processed
	followNode->is_Processed = true;
}

void SupportGeneration::compute_Support_tree_Field() {

	std::cout << " --> Implicit Distance Compute Running ..." << std::endl;
	long time = clock();


	for (GLKPOSITION posMesh = m_layerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* each_support_patch = (QMeshPatch*)m_layerSet->GetMeshList().GetNext(posMesh);

		if (!each_support_patch->is_SupportLayer) continue;

		std::vector<QMeshNode*> patch_NodeSet(each_support_patch->GetNodeNumber());

		int tempInt = 0;
		for (GLKPOSITION posNode = each_support_patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode* node = (QMeshNode*)each_support_patch->GetNodeList().GetNext(posNode);
			patch_NodeSet[tempInt] = node;
			tempInt++;
		}


#pragma omp parallel
		{
#pragma omp for

			for (int j = 0; j < patch_NodeSet.size(); j++) {

				QMeshNode* node = patch_NodeSet[j];
				Eigen::Vector3d queryPnt = Eigen::Vector3d::Zero();
				node->GetCoord3D(queryPnt[0], queryPnt[1], queryPnt[2]);

				//constant R
				node->implicitSurface_value = _implicitDistance_Func_tree(queryPnt) - C;
			}
		}
		each_support_patch->drawSupportField = true;
		std::cout << ".";
		if ((each_support_patch->GetIndexNo() + 1) % 100 == 0) std::cout << std::endl;
	}
	std::cout << std::endl;
	std::printf(" --> Solve takes %ld ms.\n", clock() - time);
}

double SupportGeneration::_implicitDistance_Func_tree(Eigen::Vector3d& queryPnt) {

	// TET surface RAY (dynamic host)
	/*  ^ r_i (energy radius)
	*4.5|_______ n
	*	|	    /|
	*	|	   / |
	*   |	  /  |
	*2.5|--m /   |
	*	|____|___|____> sqrt(node section NUM)
	*	0    1   M
	*/

	double F_queryPnt = 0.0;
	QMeshPatch* layer = (QMeshPatch*)m_supportModelSet->GetMeshList().GetHead();

	Eigen::Vector2d m = { 1.0, 3.5 };//3.2
	Eigen::Vector2d n = { std::sqrt((double)layer->max_branch_NUM), 5.5 };//3.2
	double k = (n[1] - m[1]) / (n[0] - m[0]);

	for (GLKPOSITION Pos = layer->GetEdgeList().GetHeadPosition(); Pos != NULL;) {
		QMeshEdge* edge = (QMeshEdge*)(layer->GetEdgeList().GetNext(Pos));

		Eigen::Vector3d O;	edge->GetStartPoint()->GetCoord3D(O[0], O[1], O[2]); // Vs
		Eigen::Vector3d T;	edge->GetEndPoint()->GetCoord3D(T[0], T[1], T[2]); // Ve

		//linear interpolation: sqrt(edge->treeEdge_branch_NUM) V.S. r_i
		double sqrt_branch_NUM = std::sqrt((double)edge->treeEdge_branch_NUM);
		//std::cout << sqrt_branch_NUM << std::endl;
		double r_i = k * (sqrt_branch_NUM - m[0]) + m[1];
		// test: add hight into r_i
		r_i += (layer->max_height - edge->treeEdge_height) * (8.0 / layer->max_height); //3.5
		double R = r_i * detectRadius_ratio;	// refresh detection radius R

		//build a box and _queryPnt is center, only computing the polyline in the box --> speed up
		Eigen::Vector3d delta_Oq = (queryPnt - O);
		Eigen::Vector3d delta_Tq = (queryPnt - T);
		if ((fabs(delta_Oq[0]) > R || fabs(delta_Oq[1]) > R || fabs(delta_Oq[2]) > R) &&
			(fabs(delta_Tq[0]) > R || fabs(delta_Tq[1]) > R || fabs(delta_Tq[2]) > R))
			continue;

		double mu1, mu2;
		bool intersection = _lineIntersectSphere(O, T, queryPnt, R, mu1, mu2);

		if (intersection == false) continue;	// no intersection
		if (mu1 > 1.0 && mu2 > 1.0) continue;	// O -> T ()
		if (mu1 < 0.0 && mu2 < 0.0) continue;	// () O -> T 

		Eigen::Vector3d iPnt_1 = O + mu1 * (T - O); //p1
		Eigen::Vector3d iPnt_2 = O + mu2 * (T - O); //p2

		double s1 = MAX(0, (O - iPnt_1).dot(iPnt_2 - iPnt_1) / (iPnt_2 - iPnt_1).squaredNorm());
		double s2 = MIN(1, (T - iPnt_1).dot(iPnt_2 - iPnt_1) / (iPnt_2 - iPnt_1).squaredNorm());

		if (s1 > 1.0 || s2 < 0.0) {
			std::cout << "------------------" << std::endl;
			std::cout << "ERROR:0.0 < s1 < s2 < 1.0 !!!" << std::endl;
			std::cout << "s1: " << s1 << std::endl;
			std::cout << "s2: " << s2 << std::endl;
			std::cout << "------------------" << std::endl;
		}

		double l = (iPnt_2 - iPnt_1).norm();
		double a = (queryPnt - iPnt_1).dot(iPnt_2 - iPnt_1);

		double F_queryPnt_eachRay = r_i / 15.0 / pow(R, 4) * (
			(3 * pow(l, 4) * pow(s2, 5) - 15 * a * pow(l, 2) * pow(s2, 4) + 20 * pow(a, 2) * pow(s2, 3))
			- (3 * pow(l, 4) * pow(s1, 5) - 15 * a * pow(l, 2) * pow(s1, 4) + 20 * pow(a, 2) * pow(s1, 3)));

		F_queryPnt += F_queryPnt_eachRay;
	}
	return F_queryPnt;
}

// Line: O->T; Sphere: center,R; mu: Explicit argument
// reference: http://www.ambrsoft.com/TrigoCalc/Sphere/SpherLineIntersection_.htm
bool SupportGeneration::_lineIntersectSphere
(Eigen::Vector3d& O, Eigen::Vector3d& T, Eigen::Vector3d& Center, double R, double& mu1, double& mu2) {

	//std::cout << R << std::endl;

	double a = (T - O).squaredNorm();
	double b = -2 * (T - O).dot(Center - O);
	double c = (Center - O).squaredNorm() - R * R;

	double bb4ac = b * b - 4 * a * c;

	if (fabs(a) == 0 || bb4ac <= 0) {
		mu1 = 0;
		mu2 = 0;
		return false;
	}

	mu1 = (-b + sqrt(bb4ac)) / (2 * a);
	mu2 = (-b - sqrt(bb4ac)) / (2 * a);

	// keep mu1 < mu2
	if (mu2 < mu1) {
		double temp_mu = mu1;
		mu1 = mu2;
		mu2 = temp_mu;
		//std::cout << "exchange mu1,mu2" << std::endl;
	}

	return true;
}

void SupportGeneration::build_tight_supportLayers() {

	//clean the single or two negative point
	for (GLKPOSITION posMesh = m_layerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* each_support_patch = (QMeshPatch*)m_layerSet->GetMeshList().GetNext(posMesh);

		if (!each_support_patch->is_SupportLayer) continue;

		//// node loop
		for (GLKPOSITION Pos = each_support_patch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)each_support_patch->GetNodeList().GetNext(Pos);

			if (Node->implicitSurface_value > 0.0) continue;

			// node one ring loop
			int neighbor_positive_nodeNum = 0;
			for (GLKPOSITION Pos_neighbor = Node->GetEdgeList().GetHeadPosition(); Pos_neighbor;) {
				QMeshEdge* oneRing_Edge = (QMeshEdge*)Node->GetEdgeList().GetNext(Pos_neighbor);

				QMeshNode* Node_1ring = oneRing_Edge->GetStartPoint();
				if (Node_1ring == Node) Node_1ring = oneRing_Edge->GetEndPoint();

				if (Node_1ring->implicitSurface_value > 0.0) neighbor_positive_nodeNum++;
			}

			if (neighbor_positive_nodeNum == Node->GetEdgeNumber()
				|| neighbor_positive_nodeNum == (Node->GetEdgeNumber() - 1)) {

				Node->implicitSurface_value = 1.0;
			}
		}
	}

	double cut_isoValue = 0.0; // cutting occurring on the implicit surface
	for (GLKPOSITION posMesh = m_layerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* each_support_patch = (QMeshPatch*)m_layerSet->GetMeshList().GetNext(posMesh);

		if (!each_support_patch->is_SupportLayer) continue;

		/* get the VERTEX NUM of tight support Layer */
		int structuredMesh_NodeNum = 0;
		//// node loop
		for (GLKPOSITION Pos = each_support_patch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)each_support_patch->GetNodeList().GetNext(Pos);

			Node->implicitSurface_cut_index = -1;//reset 

			if (Node->implicitSurface_value > 0) {

				Node->implicitSurface_cut_index = structuredMesh_NodeNum; // start from 0
				structuredMesh_NodeNum++;
			}
		}
		//// edge loop
		for (GLKPOSITION Pos = each_support_patch->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge* Edge = (QMeshEdge*)each_support_patch->GetEdgeList().GetNext(Pos);

			double a = Edge->GetStartPoint()->implicitSurface_value;
			double b = Edge->GetEndPoint()->implicitSurface_value;

			if ((cut_isoValue - a) * (cut_isoValue - b) < 0.0) {
				double alpha = (cut_isoValue - a) / (b - a);
				double p1[3], p2[3], pp[3];
				Edge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
				Edge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

				for (int j = 0; j < 3; j++) {
					//compute the position for cut_isoValue
					pp[j] = (1.0 - alpha) * p1[j] + alpha * p2[j];
				}

				QMeshNode* cut_Node = new QMeshNode;
				cut_Node->cutNode_related_LayerEdge = Edge;
				cut_Node->implicitSurface_value = cut_isoValue;

				cut_Node->SetCoord3D(pp[0], pp[1], pp[2]);
				//cutNode_index should increase based on the "structuredMesh_NodeNum"
				cut_Node->implicitSurface_cut_index = structuredMesh_NodeNum;
				structuredMesh_NodeNum++;

				//install this cutNode to its Edge
				Edge->installed_CutNode = cut_Node;
				Edge->isLocate_CutNode_layerEdge = true;
			}
		}

		/* get the VERTEX Table of tight support Layer */
		if (structuredMesh_NodeNum == 0) continue;// avoid the meanless compute
		Eigen::MatrixXd V = Eigen::MatrixXd::Zero(structuredMesh_NodeNum, 3);
		//// node loop
		for (GLKPOSITION Pos = each_support_patch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)each_support_patch->GetNodeList().GetNext(Pos);

			if (Node->implicitSurface_cut_index >= 0) { // VERTEX needed to be kept

				double xx, yy, zz;
				Node->GetCoord3D(xx, yy, zz);
				V.row(Node->implicitSurface_cut_index) << xx, yy, zz;
			}
		}
		//// edge loop
		for (GLKPOSITION Pos = each_support_patch->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge* Edge = (QMeshEdge*)each_support_patch->GetEdgeList().GetNext(Pos);

			if (Edge->isLocate_CutNode_layerEdge) {

				double xx, yy, zz;
				Edge->installed_CutNode->GetCoord3D(xx, yy, zz);
				V.row(Edge->installed_CutNode->implicitSurface_cut_index) << xx, yy, zz;

			}
		}

		/* get the FACE NUM of tight support Layer */
		int structuredMesh_FaceNum = 0;
		//// face loop
		for (GLKPOSITION Pos = each_support_patch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)each_support_patch->GetFaceList().GetNext(Pos);

			int positive_Num = 0;
			for (int k = 0; k < 3; k++) {
				if (Face->GetNodeRecordPtr(k)->implicitSurface_value > 0) {
					positive_Num++;
				}
			}

			if (positive_Num < 0 || positive_Num > 3)
				std::cout << "ERROR: the NUM of point with positive implicite surface field value is out of range\n";

			if (positive_Num == 3) {
				structuredMesh_FaceNum = structuredMesh_FaceNum + 1;
				Face->faceType = KEEP;
			}
			else if (positive_Num == 2) {
				structuredMesh_FaceNum = structuredMesh_FaceNum + 2;
				Face->faceType = CUT_2;
			}
			else if (positive_Num == 1) {
				structuredMesh_FaceNum = structuredMesh_FaceNum + 1;
				Face->faceType = CUT_1;
			}
			else {
				Face->faceType = DISCARD;
			}
		}// END: get the FACE NUM of tight support Layer

		/* get the FACE Table of tight support Layer */
		Eigen::MatrixXi F = Eigen::MatrixXi::Zero(structuredMesh_FaceNum, 3);
		// face loop
		// collect the 3+ faces (the implicit_cut_values of 3 vertexes of one face are all large than 0)
		int F_row_index = 0;
		for (GLKPOSITION Pos = each_support_patch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)each_support_patch->GetFaceList().GetNext(Pos);

			if (Face->faceType == KEEP) {

				for (int k = 0; k < 3; k++) {
					F(F_row_index, k) = Face->GetNodeRecordPtr(k)->implicitSurface_cut_index;
				}
				F_row_index++;
			}
			else if (Face->faceType == CUT_2) {

				int baseIndex = -1;// record the index of edge without cut_Node --> range: [0-2]
				for (int k = 0; k < 3; k++) {
					if (!Face->GetEdgeRecordPtr(k + 1)->isLocate_CutNode_layerEdge) {
						baseIndex = k;
						break;
					}
				}
				if (baseIndex == -1) std::cout << "ERROR: cannot find the edge without cut_Node" << std::endl;

				// the 1st added face
				F(F_row_index, 0) = Face->GetNodeRecordPtr(baseIndex)->implicitSurface_cut_index;
				F(F_row_index, 1) = Face->GetNodeRecordPtr((baseIndex + 1) % 3)->implicitSurface_cut_index;
				F(F_row_index, 2) = Face->GetEdgeRecordPtr((baseIndex + 1) % 3 + 1)->installed_CutNode->implicitSurface_cut_index;
				F_row_index++;

				// the 2nd added face
				F(F_row_index, 0) = Face->GetNodeRecordPtr(baseIndex)->implicitSurface_cut_index;
				F(F_row_index, 1) = Face->GetEdgeRecordPtr((baseIndex + 1) % 3 + 1)->installed_CutNode->implicitSurface_cut_index;
				F(F_row_index, 2) = Face->GetEdgeRecordPtr((baseIndex + 2) % 3 + 1)->installed_CutNode->implicitSurface_cut_index;
				F_row_index++;
			}
			else if (Face->faceType == CUT_1) {

				int baseIndex = -1;// record the index of edge without cut_Node --> range: [0-2]
				for (int k = 0; k < 3; k++) {
					if (!Face->GetEdgeRecordPtr(k + 1)->isLocate_CutNode_layerEdge) {
						baseIndex = k;
						break;
					}
				}
				if (baseIndex == -1) std::cout << "ERROR: cannot find the edge without cut_Node." << std::endl;

				F(F_row_index, 0) = Face->GetNodeRecordPtr((baseIndex + 2) % 3)->implicitSurface_cut_index;
				F(F_row_index, 1) = Face->GetEdgeRecordPtr((baseIndex + 2) % 3 + 1)->installed_CutNode->implicitSurface_cut_index;
				F(F_row_index, 2) = Face->GetEdgeRecordPtr((baseIndex + 1) % 3 + 1)->installed_CutNode->implicitSurface_cut_index;
				F_row_index++;

			}
			else if (Face->faceType == DISCARD) {
				// do nothing
			}
			else {
				std::cout << "ERROR Face Type !" << std::endl;
			}
		}

		// build new mesh from vertex table and face table
		float* nodeTable;
		nodeTable = (float*)malloc(sizeof(float) * V.rows() * 3);
		for (int j = 0; j < V.rows(); j++) {
			for (int i = 0; i < 3; i++) nodeTable[j * 3 + i] = (float)V(j, i);
		}
		unsigned int* faceTable;
		faceTable = (unsigned int*)malloc(sizeof(unsigned int) * F.rows() * 3);
		for (int j = 0; j < F.rows(); j++) {
			for (int i = 0; i < 3; i++) faceTable[j * 3 + i] = F(j, i);
		}

		QMeshPatch* surface = new QMeshPatch;
		surface->SetIndexNo(m_layerSet->GetMeshList().GetCount()); //index begin from 0
		m_layerSet->GetMeshList().AddTail(surface);
		surface->constructionFromVerFaceTable(V.rows(), nodeTable, F.rows(), faceTable);

		//smoothing layers // this method will cause some problems
		//for (int i = 0; i < 5; i++) _smoothingIsoSurface();

		surface->drawThisPatch = true;
		surface->is_SupportLayer = true;
		surface->is_Slimmed_SupportLayer = true;
		surface->compatible_layer_Index = each_support_patch->compatible_layer_Index;
	}

	//remove the original support layers
	for (GLKPOSITION posMesh = m_layerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		GLKPOSITION temp_posMesh = posMesh;
		QMeshPatch* each_support_patch = (QMeshPatch*)m_layerSet->GetMeshList().GetNext(posMesh);

		if (each_support_patch->is_SupportLayer && !each_support_patch->is_Slimmed_SupportLayer) {

			each_support_patch->ClearAll();		
			m_layerSet->GetMeshList().RemoveAt(temp_posMesh);
			delete each_support_patch;
		}
	}

	for (GLKPOSITION posMesh = m_layerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		GLKPOSITION temp_posMesh = posMesh;
		QMeshPatch* each_support_patch = (QMeshPatch*)m_layerSet->GetMeshList().GetNext(posMesh);

		//cut the support layers under platform (h < 0.0);
		bool keep_thisLayer = this->_planeCutSurfaceMesh_delete(each_support_patch, true, 0.0);

		if (each_support_patch->is_SupportLayer && each_support_patch->is_Slimmed_SupportLayer
			&& (!keep_thisLayer)) {

			each_support_patch->ClearAll();
			m_layerSet->GetMeshList().RemoveAt(temp_posMesh);
			delete each_support_patch;
		}
	}
	std::cout << "the rest layer number is: " << m_layerSet->GetMeshList().GetCount() << std::endl;

	//reorgainize the layer index and compatible layer index
	int tempInd = 0; int min_CompatibleLayer_ind = 10000000;
	for (GLKPOSITION posMesh = m_layerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* each_support_patch = (QMeshPatch*)m_layerSet->GetMeshList().GetNext(posMesh);

		each_support_patch->SetIndexNo(tempInd);
		tempInd++;

		if (each_support_patch->compatible_layer_Index < min_CompatibleLayer_ind)
			min_CompatibleLayer_ind = each_support_patch->compatible_layer_Index;
	}
	for (GLKPOSITION posMesh = m_layerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* each_support_patch = (QMeshPatch*)m_layerSet->GetMeshList().GetNext(posMesh);

		each_support_patch->compatible_layer_Index -= min_CompatibleLayer_ind;
	}

	//output unremeshed slim_support and initial layers
	//move it to mainWindow IO.cpp
	//this->output_slimSupport_inital_4_remesh(m_layerSet);
}

bool SupportGeneration::_planeCutSurfaceMesh_delete(
	QMeshPatch* surfaceMesh, bool Yup, double cutPlaneHeight) {

	int cutFaceCase[3][9] = {
		{ 1,4,5,1,2,4,4,3,5 },
		{ 1,4,5,4,2,5,2,3,5 },
		{ 1,4,5,4,2,5,1,5,3 } };

	// Plane equation: Ax+By+Cz+D = 0	 
	Eigen::Vector3d PlaneDir; double D = cutPlaneHeight;
	if (Yup) PlaneDir << 0.0, -1.0, 0.0;
	else PlaneDir << 0.0, 0.0, -1.0;

	//--------------------------------------------------------------
	/* pre-process to compute node-plane distance*/

	int positiveNodeNum = 0; int negativeNodeNum = 0;

	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		Eigen::Vector3d pp; Node->GetCoord3D(pp(0), pp(1), pp(2));
		Node->nodePlaneDis = pp.dot(PlaneDir) + D;

		if (Node->nodePlaneDis > 0) {
			positiveNodeNum++; 
			Node->SetIndexNo(-1);
		}
		else {
			Node->SetIndexNo(negativeNodeNum); 
			negativeNodeNum++; // node index start from 0
		}
	}

	if (positiveNodeNum == 0) return true; //do nothing
	else if (negativeNodeNum == 0) {
		//surfaceMesh->ClearAll();
		return false; //will delete this layer at the next step
	}
	//--------------------------------------------------------------
	/* detect the intersect edge, build intersect node*/

	int cutNodeIndex = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* thisEdge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);

		double a = thisEdge->GetStartPoint()->nodePlaneDis;
		double b = thisEdge->GetEndPoint()->nodePlaneDis;
		if (a * b > 0) continue;

		double alpha = fabs(a) / fabs(b - a);
		double p1[3], p2[3], pp[3];
		thisEdge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
		thisEdge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

		for (int j = 0; j < 3; j++)
			pp[j] = (1.0 - alpha) * p1[j] + alpha * p2[j];

		QMeshNode* isoNode = new QMeshNode;
		isoNode->SetMeshPatchPtr(surfaceMesh);
		isoNode->SetCoord3D(pp[0], pp[1], pp[2]);

		//std::cout << pp[0] << "," << pp[1] << "," << pp[2] << std::endl;

		surfaceMesh->GetNodeList().AddTail(isoNode);
		isoNode->planeCutNewNode = true;

		isoNode->SetIndexNo(negativeNodeNum + cutNodeIndex); // index start from 0
		cutNodeIndex++;

		thisEdge->intersectNodeIndex = isoNode->GetIndexNo(); // if exist, the node index will large than 0
	}
	//std::cout << "finish generate the cutting node" << std::endl;

	///* Build topology */

	// -- node table
	float* nodeTable;
	int nodeNum = surfaceMesh->GetNodeNumber() - positiveNodeNum;
	nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);

	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		if (Node->GetIndexNo() < 0) continue; //delete those face have node on the left plane
		double pp[3]; Node->GetCoord3D(pp[0], pp[1], pp[2]);
		for (int i = 0; i < 3; i++) nodeTable[Node->GetIndexNo() * 3 + i] = (float)pp[i];

		//std::cout << pp[0] << "," << pp[1] << "," << pp[2] << std::endl;
	}

	// -- face table
	int faceNum = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);

		int face_posNodeNum = 0;
		for (int i = 0; i < 3; i++) {
			if (face->GetNodeRecordPtr(i)->nodePlaneDis > 0) face_posNodeNum++;
		}
		if (face_posNodeNum == 0 || face_posNodeNum == 2) faceNum += 1;
		else if (face_posNodeNum == 1) faceNum += 2;
	}

	unsigned int* faceTable;
	faceTable = (unsigned int*)malloc(sizeof(unsigned int) * faceNum * 3);

	int faceNodeIndex = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);

		int face_posNodeNum = 0;
		for (int i = 0; i < 3; i++) {
			if (face->GetNodeRecordPtr(i)->nodePlaneDis > 0) face_posNodeNum++;
		}

		if (face_posNodeNum == 3) continue;
		else if (face_posNodeNum == 0) {
			for (int i = 0; i < 3; i++)
				faceTable[faceNodeIndex + i] = face->GetNodeRecordPtr(i)->GetIndexNo();
			faceNodeIndex += 3; continue;
		}

		int nodeIndexArray[5] = { 0 }; int noCutEdgeIndex = -1; int cutEdgeIndex = 0;
		for (int i = 0; i < 3; i++)
			nodeIndexArray[i] = face->GetNodeRecordPtr(i)->GetIndexNo();
		for (int i = 0; i < 3; i++) {
			if (face->GetEdgeRecordPtr(i + 1)->intersectNodeIndex < 0) {
				noCutEdgeIndex = i; continue;
			}
			else {
				nodeIndexArray[3 + cutEdgeIndex] = face->GetEdgeRecordPtr(i + 1)->intersectNodeIndex;
				cutEdgeIndex++;
			}
		}

		if (face_posNodeNum == 1) {

			int positiveNodeIndex = -1;
			for (int i = 0; i < 3; i++) {
				if (face->GetNodeRecordPtr(i)->nodePlaneDis > 0) positiveNodeIndex = i;
			}

			for (int i = 0; i < 3; i++) { //case
				if (noCutEdgeIndex == i) {

					int addFaceIndex = 0;
					for (int j = 0; j < 3; j++) { //face

						bool keepThisFace = true;
						for (int k = 0; k < 3; k++) {
							int currentNodeIndex = cutFaceCase[i][3 * j + k] - 1;
							if (currentNodeIndex == positiveNodeIndex) keepThisFace = false;
						}
						if (keepThisFace == false) continue;

						for (int k = 0; k < 3; k++)  //node
							faceTable[faceNodeIndex + 3 * addFaceIndex + k] = nodeIndexArray[cutFaceCase[i][3 * j + k] - 1];
						addFaceIndex++;
					}
				}
			}

			faceNodeIndex += 6;
		}

		else if (face_posNodeNum == 2) {

			int negativeNodeIndex = -1;
			for (int i = 0; i < 3; i++) {
				if (face->GetNodeRecordPtr(i)->nodePlaneDis < 0) negativeNodeIndex = i;
			}

			for (int i = 0; i < 3; i++) { //case
				if (noCutEdgeIndex == i) {
					for (int j = 0; j < 3; j++) { //face

						bool keepThisFace = false;
						for (int k = 0; k < 3; k++) {
							int currentNodeIndex = cutFaceCase[i][3 * j + k] - 1;
							if (currentNodeIndex == negativeNodeIndex) keepThisFace = true;
						}
						if (keepThisFace == false) continue;
						for (int k = 0; k < 3; k++)  //node
							faceTable[faceNodeIndex + k] = nodeIndexArray[cutFaceCase[i][3 * j + k] - 1];
					}
				}
			}

			faceNodeIndex += 3;
		}
	}

	/*for (int i = 0; i < nodeNum * 3; i++) {
		if (i % 3 == 0 && i > 0) std::cout << std::endl;
		std::cout << nodeTable[i] << ",";
	}

	//for (int i = 0; i < faceNum * 3; i++) {
	//	if (i % 3 == 0 && i > 0) std::cout << std::endl;
	//	std::cout << faceTable[i] << ",";
	//}
	//std::cout << std::endl << nodeNum << "," << faceNum << std::endl;
	//std::cout << faceNodeIndex << "," << faceNum << std::endl;
	//std::cout << "finish build node and face table" << std::endl;
	*/

	surfaceMesh->ClearAll();
	surfaceMesh->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);
	free(nodeTable);
	free(faceTable);
	return true;
	//printf("--- finish cut mesh by plane ---------");
}

void SupportGeneration::output_slimSupport_inital_4_remesh(PolygenMesh* sSupport_init_isoLayerSet) {

	std::string unremeshed_isoLayer_dir = "../DataSet/remesh_operation/layers_unremeshed";
	this->_remove_allFile_in_Dir(unremeshed_isoLayer_dir);

	for (GLKPOSITION posMesh = sSupport_init_isoLayerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* each_layer = (QMeshPatch*)sSupport_init_isoLayerSet->GetMeshList().GetNext(posMesh);

		std::string modelORsupport = "M";
		if (each_layer->is_SupportLayer) modelORsupport = "S";

		//e.g. 23_C15_M.obj
		std::string LAYER_dir = unremeshed_isoLayer_dir + "/"
			+ std::to_string(each_layer->GetIndexNo()) + "_C"
			+ std::to_string(each_layer->compatible_layer_Index) + "_"
			+ modelORsupport;
		this->_output_oneSurfaceMesh(each_layer, LAYER_dir);
	}
	std::cout << "Finish output unmeshed layers into : " << unremeshed_isoLayer_dir << std::endl;
}