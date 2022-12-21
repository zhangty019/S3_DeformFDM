#include "toolpathgeneration.h"
#include <iostream>
#include <fstream>
#include "GLKGeometry.h"
#include "heatmethodfield.h"
#include "io.h"
#include "dirent.h"

toolpathGeneration::toolpathGeneration(PolygenMesh* isoLayerSet, PolygenMesh* toolPathSet,
	double deltaWidth, double deltaDistance){

	m_isoLayerSet = isoLayerSet;
	m_toolPathSet = toolPathSet;
	toolpath_Width = deltaWidth;
	toolpath_Distance = deltaDistance;

	this->_input_remeshed_init_slimSupport_layers(m_isoLayerSet);
}

toolpathGeneration::~toolpathGeneration() {}

void toolpathGeneration::_input_remeshed_init_slimSupport_layers(PolygenMesh* m_isoLayerSet) {

	std::string remeshed_isoLayer_dir = "../DataSet/remesh_operation/layers_remeshed";
	std::vector<std::string> remeshedLayer_FileCell;// File name table
	this->_getFileName_Set(remeshed_isoLayer_dir, remeshedLayer_FileCell);

	//read slice files and build mesh_patches
	char file_Dir[1024];
	for (int i = 0; i < remeshedLayer_FileCell.size(); i++)
	{
		sprintf(file_Dir, "%s%s%s", remeshed_isoLayer_dir.c_str(), "/", remeshedLayer_FileCell[i].data());
		//cout << file_Dir << endl;

		QMeshPatch* slice = new QMeshPatch;
		slice->SetIndexNo(m_isoLayerSet->GetMeshList().GetCount()); //index begin from 0
		m_isoLayerSet->GetMeshList().AddTail(slice);
		slice->patchName = remeshedLayer_FileCell[i].data();

		// is_SupportLayer
		std::string::size_type supportFlag = remeshedLayer_FileCell[i].find("S");
		if (supportFlag == std::string::npos)	slice->is_SupportLayer = false;
		else	slice->is_SupportLayer = true;

		// 101_C81_M_1.obj
		// get compatible layer index
		std::string::size_type p = remeshedLayer_FileCell[i].find('_');
		std::string::size_type pp = remeshedLayer_FileCell[i].find('_', p + 2);
		slice->compatible_layer_Index = stoi(remeshedLayer_FileCell[i].substr(p + 2, pp - p - 1));

		std::string::size_type q = remeshedLayer_FileCell[i].find('_', pp + 2);
		std::string::size_type qq = remeshedLayer_FileCell[i].find('.');
		slice->splitIndex = stoi(remeshedLayer_FileCell[i].substr(q + 1, qq - 1));

		//std::cout << p << " " << pp << " " << q << " " << qq << std::endl;

		/*std::cout << "remeshedLayer_FileCell[i]: " << remeshedLayer_FileCell[i] 
			<< " layer index: " << slice->GetIndexNo()
			<< " layer compatible index: " << slice->compatible_layer_Index 
			<< " layer split index: " << slice->splitIndex << std::endl;*/

		slice->inputOBJFile(file_Dir);
	}
}

void toolpathGeneration::_getFileName_Set(std::string dirctory, std::vector<std::string>& fileNameCell) {

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
	std::vector<std::string> copy_fileNameCell = fileNameCell;
	for (int i = 0; i < copy_fileNameCell.size(); i++) {

		//std::cout << copy_fileNameCell[i] << std::endl;

		int layerFile_Ind = -1;
		std::string::size_type q = copy_fileNameCell[i].find('_');
		layerFile_Ind = stoi(copy_fileNameCell[i].substr(0, q));
		
		fileNameCell[layerFile_Ind] = copy_fileNameCell[i];
	}
}

void toolpathGeneration::generate_all_toolPath() {

	if (toolpath_Width <= 0.0 || toolpath_Distance <= 0.0){

		std::cout << "width or distance is zero!" << std::endl;
		return;
	}

	int layerNum = m_isoLayerSet->GetMeshList().GetCount();
	std::vector<QMeshPatch*> sliceVector(layerNum);
	std::vector<QMeshPatch*> toolpathVector(layerNum);

	int tempInd = 0;
	for (GLKPOSITION posMesh = m_isoLayerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* layer = (QMeshPatch*)m_isoLayerSet->GetMeshList().GetNext(posMesh);

		sliceVector[tempInd] = layer;
		tempInd++;
	}

#pragma omp parallel
	{
#pragma omp for  
		for (int i = 0; i < sliceVector.size(); i++) {

			/* ---- Generate boundary heat field ---- */
			QMeshPatch* layer = sliceVector[i];
			heatMethodField* heatField_layer = new heatMethodField(layer);

			//heatField_layer->meshRefinement();
			heatField_layer->compBoundaryHeatKernel();
			delete heatField_layer;
			/* ---- END ---- */

			QMeshPatch* singlePath = this->generate_each_bundaryToolPath(layer);
			this->resampleToolpath(singlePath);
			
			if (singlePath != NULL) {
				if (layer->is_SupportLayer) {
					singlePath->is_SupportLayer = layer->is_SupportLayer;
				}
				singlePath->compatible_layer_Index = layer->compatible_layer_Index;
				singlePath->attached_Layer = layer;
			}

			toolpathVector[layer->GetIndexNo()] = singlePath;
		}
	}

	std::cout << "\nThe number means Toolpath i is NULL, skip it." << std::endl;
	for (int i = 0; i < layerNum; i++) {

		if ((i + 1) % 100 == 0) std::cout << std::endl;
		if (toolpathVector[i] != NULL) std::cout << ".";
		else std::cout << i;

		if (toolpathVector[i] != NULL) {
			QMeshPatch* singlePath = toolpathVector[i];
			singlePath->SetIndexNo(i);
			m_toolPathSet->GetMeshList().AddTail(singlePath);
		}
	}
	std::cout << std::endl;
}

QMeshPatch* toolpathGeneration::generate_each_bundaryToolPath(QMeshPatch* surfaceMesh) {

	int curveNum = autoComputeTPathNum(surfaceMesh, true); int boundaryTp_num = 3;

	if (curveNum > 100) {
		std::cout << "layer = " << surfaceMesh->GetIndexNo() << " high curve num " << curveNum << "." << std::endl;
	}
	if (curveNum <= 0 || curveNum > 100) { return NULL; }
	
	if (curveNum < boundaryTp_num)	curveNum = boundaryTp_num; // small layers needs at lest 3 rings of boundary loop
	std::vector<double> isoValue(curveNum);// store all isoValue
	double deltaIsoValue = 1.0 / curveNum; // deltaIsoValue in [0-1]
	QMeshPatch* singlePath = new QMeshPatch;

	for (int i = 0; i < curveNum; i++) {

		isoValue[i] = (i + 0.5) * deltaIsoValue;
		//build isonode with each isoValue without linkage
		generateBoundaryIsoNode(singlePath, surfaceMesh, isoValue[i]);

		if (singlePath->GetNodeNumber() == 0)
			std::cout << "Warning! the boundary toolpath contains no isonode!" << std::endl;  //means no node in singlePath
	}

	//DEBUG
	//for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode* thisNode = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);
	//	std::cout << thisNode->GetIndexNo() << std::endl;
	//}
	//

	double min_isoValue = isoValue[0];
	double max_isoValue = isoValue[curveNum - 1];
	double startIsoValue;
	//choose printing direction (inner -> outside(true)) or inverse(false)
	bool growDirction = true;
	if (growDirction)	startIsoValue = min_isoValue;
	else startIsoValue = max_isoValue;
	linkEachIsoNode(singlePath, startIsoValue);

	//DEBUG
	/*for (GLKPOSITION posEdge = singlePath->GetEdgeList().GetHeadPosition(); posEdge != nullptr;) {
		QMeshEdge* Edge = (QMeshEdge*)singlePath->GetEdgeList().GetNext(posEdge);
		std::cout << "start Node " << Edge->GetStartPoint()->GetIndexNo() << " end Node " << Edge->GetEndPoint()->GetIndexNo() << std::endl;
	}*/
	//END

	//printf("--> Bundary Toolpath Generation Finish\n");

	return singlePath;
}

//true->boundary; false->zigzag
int toolpathGeneration::autoComputeTPathNum(QMeshPatch* surfaceMesh, bool boundaryORzigzag) {
	
	double distanceTPath = this->toolpath_Width;
	if (!boundaryORzigzag) distanceTPath *= 1.00; //0.95

	int iter = 10;
	std::vector<Eigen::MatrixXd> isoPoint(10);
	Eigen::VectorXd isoPointNum(iter);

	int maxNodeNum = 10000;

	for (int i = 0; i < iter; i++) {
		isoPoint[i] = Eigen::MatrixXd::Zero(maxNodeNum, 3);

		double isoValue = (0.5 + i) * 1.0 / iter;

		int index = 0;
		for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge* Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);

			double a, b;
			if (boundaryORzigzag) {
				a = Edge->GetStartPoint()->boundaryValue;
				b = Edge->GetEndPoint()->boundaryValue;
			}
			else {
				a = Edge->GetStartPoint()->zigzagValue;
				b = Edge->GetEndPoint()->zigzagValue;
			}

			if ((isoValue - a) * (isoValue - b) < 0.0) {
				double alpha = (isoValue - a) / (b - a);
				double p1[3], p2[3], pp[3];
				Edge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
				Edge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

				for (int j = 0; j < 3; j++) {
					//compute the position for this isonode
					if (index > maxNodeNum) { printf("ERROR, node number too high!\n"); break; }
					isoPoint[i](index, j) = (1.0 - alpha) * p1[j] + alpha * p2[j];
				}
				index++;
			}
		}
		isoPointNum(i) = index;
		//std::cout << "isovalue " << isoValue << " layer has " << index << " points" << std::endl;
	}

	//fix bug
	double node_coord3D_sum = 0.0;
	for (int i = 0; i < iter - 1; i++) {

		node_coord3D_sum += isoPoint[i].sum();
	}
	//std::cout << surfaceMesh->GetIndexNo() << " " << node_coord3D_sum << std::endl;
	if (fabs(node_coord3D_sum) < 1e-5) return 0;


	Eigen::VectorXd distance(iter - 1);
	for (int i = 0; i < iter - 1; i++) {
		distance(i) = 1000000.0;

		for (int j = 0; j < isoPointNum(i); j++) {
			for (int k = 0; k < isoPointNum(i + 1); k++) {
				double dis = (isoPoint[i].row(j) - isoPoint[i + 1].row(k)).norm();
				if (dis < distance(i)) distance(i) = dis;
			}
		}
	}
	//std::cout << distance << std::endl; // return the number of cut
	return floor(distance.sum() / distanceTPath); 
}

void toolpathGeneration::generateBoundaryIsoNode(QMeshPatch* singlePath, QMeshPatch* surfaceMesh, double isoValue) {

	//when the node iso-value is equal to surface value, add this eps.
	/*double eps = 1.0e-5;
	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		if (fabs(Node->boundaryValue - isoValue) < eps) {
			if (Node->boundaryValue > isoValue) Node->boundaryValue = isoValue + eps;
			else Node->boundaryValue = isoValue - eps;
		}
	}*/

	//----build the node and install back to the "singlePath"
	for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);

		double a = Edge->GetStartPoint()->boundaryValue;
		double b = Edge->GetEndPoint()->boundaryValue;

		if ((isoValue - a) * (isoValue - b) < 0.0) {
			double alpha = (isoValue - a) / (b - a);
			double p1[3], p2[3], pp[3];
			Edge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
			Edge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

			for (int j = 0; j < 3; j++) {
				//compute the position for this isonode
				pp[j] = (1.0 - alpha) * p1[j] + alpha * p2[j];
			}

			QMeshNode* isoNode = new QMeshNode;
			isoNode->relatedLayerEdge = Edge;
			isoNode->connectTPathProcessed = false;
			isoNode->isoValue = isoValue;

			isoNode->SetMeshPatchPtr(singlePath);
			isoNode->SetCoord3D(pp[0], pp[1], pp[2]);
			isoNode->SetIndexNo(singlePath->GetNodeList().GetCount()); //index should start from 0

			//cal normal of iso point
			double n1[3], n2[3], n3[3];
			Edge->GetLeftFace()->GetNormal(n1[0], n1[1], n1[2]);
			Edge->GetRightFace()->GetNormal(n2[0], n2[1], n2[2]);
			for (int i = 0; i < 3; i++) n3[i] = (n1[i] + n2[i]) / 2;
			isoNode->SetNormal(n3[0], n3[1], n3[2]);

			//install this isoNode of layer to its Edge
			Edge->installedIsoNode_layerEdge.push_back(isoNode);
			Edge->isLocateIsoNode_layerEdge = true;
			singlePath->GetNodeList().AddTail(isoNode);
		}
	}
}

void toolpathGeneration::linkEachIsoNode(QMeshPatch* singlePath, double startIsoValue) {

	/*DEGUG*/
	//for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode* thisNode = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);
	//	std::cout << "thisNode->connectTPathProcessed = " << thisNode->connectTPathProcessed 
	//		<< " thisNode->isoValue = " << thisNode->isoValue << std::endl;
	//}
	/*END*/

	// Method 1: find the First Node(boundFirstNode) which is nearest with x axis
	QMeshNode* X_nearestFirstNode = nullptr;
	double minX = 1e10;
	for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);
		if (thisNode->connectTPathProcessed == false && startIsoValue == thisNode->isoValue) {
			
			double px = 0.0; double py = 0.0; double pz = 0.0;
			thisNode->GetCoord3D(px, py, pz);
			if (px < minX) { 
				minX = px; 
				X_nearestFirstNode = thisNode;
			}
		}
	}
	QMeshNode* boundFirstNode = X_nearestFirstNode;
	boundFirstNode->connectTPathProcessed = true;

	// Method 2: get a First Node(boundFirstNode) to start the toolPath (randomly)
	//QMeshNode* boundFirstNode = nullptr;
	//for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode* thisNode = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);
	//	if (thisNode->connectTPathProcessed == false && startIsoValue == thisNode->isoValue) {
	//		thisNode->connectTPathProcessed = true;
	//		boundFirstNode = thisNode;
	//		break;
	//	}
	//}

	// protect output
	if (boundFirstNode == NULL) std::cout << "Cannot find the start point!, please check." << std::endl;

	QMeshNode* sNode = boundFirstNode;
	QMeshNode* eNode = findNextBoundaryToolPath(sNode, singlePath);

	/* Link all of iso-Node in one Layer*/
	do {
		
		// Start linking one ring with same iso-Value
		QMeshNode* unlinked_eNode = link_OneRing_isoNode(singlePath, sNode, eNode);
		if (unlinked_eNode == nullptr) return;
		
		if (detectAll_isoNode_Processed(singlePath)) return;
		
		QMeshNode* link_sNode = unlinked_eNode;
		QMeshNode* link_eNode = findNextNearestPoint(link_sNode, singlePath);
		if (link_eNode == nullptr) std::cout << "Error: Cannot find link_eNode" << std::endl;
		QMeshEdge* link_Edge = buildNewEdgetoQMeshPatch(singlePath, link_sNode, link_eNode);
		link_Edge->isConnectEdge = true;

		sNode = link_eNode;
		eNode = findNextBoundaryToolPath(sNode, singlePath);
		if (eNode == nullptr) std::cout << "Error: Cannot find next toolpath Node" << std::endl;

	} while (detectAll_isoNode_Processed(singlePath) == false);
}

QMeshNode* toolpathGeneration::findNextBoundaryToolPath(QMeshNode* sNode, QMeshPatch* singlePath) {

	QMeshNode* eNode;
	bool nextNodeDetected = false;
	QMeshEdge* thisEdge = sNode->relatedLayerEdge;

	//detect left face
	for (int i = 0; i < 3; i++) {
		QMeshEdge* NeighborEdge = thisEdge->GetLeftFace()->GetEdgeRecordPtr(i + 1);
		if (NeighborEdge == thisEdge) continue;

		if (NeighborEdge->isLocateIsoNode_layerEdge) {

			for (int j = 0; j < NeighborEdge->installedIsoNode_layerEdge.size(); j++) {

				QMeshNode* bNode = NeighborEdge->installedIsoNode_layerEdge[j];
				if (bNode->connectTPathProcessed == false && bNode->isoValue == sNode->isoValue) {
					nextNodeDetected = true;
					eNode = bNode;
					break;
				}
			}
		}
		if (nextNodeDetected == true) break;
	}

	if (nextNodeDetected) {
		eNode->connectTPathProcessed = true;
		return eNode;
	}

	//detect right face
	for (int i = 0; i < 3; i++) {
		QMeshEdge* NeighborEdge = thisEdge->GetRightFace()->GetEdgeRecordPtr(i + 1);
		if (NeighborEdge == thisEdge) continue;

		if (NeighborEdge->isLocateIsoNode_layerEdge) {

			for (int j = 0; j < NeighborEdge->installedIsoNode_layerEdge.size(); j++) {

				QMeshNode* bNode = NeighborEdge->installedIsoNode_layerEdge[j];
				if (bNode->connectTPathProcessed == false && bNode->isoValue == sNode->isoValue) {
					nextNodeDetected = true;
					eNode = bNode;
					break;
				}
			}
		}
		if (nextNodeDetected == true) break;
	}

	if (nextNodeDetected) { 
		eNode->connectTPathProcessed = true;
		return eNode; 
	}else {
		//std::cout<<"Error, the next node is not found!"<<std::endl;
		return nullptr;
	}
}

QMeshEdge* toolpathGeneration::buildNewEdgetoQMeshPatch(QMeshPatch* patch, QMeshNode* startNode, QMeshNode* endNode) {

	QMeshEdge* isoEdge = new QMeshEdge;

	//std::cout << "start Node " << startNode->GetIndexNo() << " end Node " << endNode->GetIndexNo() << std::endl;

	isoEdge->SetStartPoint(startNode);
	isoEdge->SetEndPoint(endNode);

	isoEdge->SetMeshPatchPtr(patch);
	//isoEdge->SetIndexNo(patch->GetEdgeList().GetCount() + 1);
	isoEdge->SetIndexNo(patch->GetEdgeList().GetCount());

	(startNode->GetEdgeList()).AddTail(isoEdge);
	(endNode->GetEdgeList()).AddTail(isoEdge);
	patch->GetEdgeList().AddTail(isoEdge);

	return isoEdge;
}

QMeshNode* toolpathGeneration::findNextNearestPoint(QMeshNode* sNode, QMeshPatch* singlePath){

	GLKGeometry geo;
	double pp[3]; sNode->GetCoord3D(pp[0], pp[1], pp[2]);
	double p1[3];
	double dist = 1000.0;
	QMeshNode* nextNode = nullptr;

	for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);
		if (Node->connectTPathProcessed == true) continue;
		Node->GetCoord3D(p1[0],p1[1],p1[2]);
		double distancePP = geo.Distance_to_Point(pp, p1);
		if (distancePP < dist) {
			nextNode = Node;
			dist = distancePP;
		}
	}
	if (nextNode == nullptr) { 
		std::cout << "There is no isoNode need to link between different isoValue!" <<std::endl;
		return nullptr;
	}
	else {
		nextNode->connectTPathProcessed = true;
		return nextNode;
	}
}

bool toolpathGeneration::detectAll_isoNode_Processed(QMeshPatch* singlePath) {
	//if all the node being processed return true, else return false.
	for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);
		if (thisNode->connectTPathProcessed == false) return false;
	}
	return true;
}

QMeshNode* toolpathGeneration::link_OneRing_isoNode(QMeshPatch* singlePath, QMeshNode* sNode, QMeshNode* eNode) {

	do {

		if (sNode == nullptr || eNode == nullptr) {
			std::cout << "NULL sNode or eNode" << std::endl;
			return nullptr;
		}

		buildNewEdgetoQMeshPatch(singlePath, sNode, eNode);

		sNode = eNode;
		eNode = findNextBoundaryToolPath(sNode, singlePath);

	} while (eNode != nullptr);

	return sNode;// return the sNode for the link opration between isoNode with different isoValue 
}

void toolpathGeneration::resampleToolpath(QMeshPatch* patch) {

	if (patch == NULL) return;

	double length = this->toolpath_Distance;

	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		Node->resampleChecked = false;
	}

	QMeshEdge* sEdge = (QMeshEdge*)patch->GetEdgeList().GetHead();
	QMeshNode* sNode = sEdge->GetStartPoint(); // the first Node of Toolpath
	if (sEdge->GetStartPoint()->GetEdgeNumber() > 1) sNode = sEdge->GetEndPoint();

	QMeshNode* sPoint = sNode;	QMeshNode* ePoint;	double lsum = 0.0;// temp distance record
	// mark the resampleChecked nodes
	for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);

		ePoint = Edge->GetStartPoint();
		if (ePoint == sPoint) ePoint = Edge->GetEndPoint();

		// give the ancor points (END point)
		if (Edge == patch->GetEdgeList().GetTail()) {

			ePoint->resampleChecked = true;
			sPoint->resampleChecked = true;
			break;
		}
		// give the ancor points (Linkage point)
		if (Edge->isConnectEdge || Edge->isConnectEdge_zigzag) {

			sPoint->resampleChecked = true;	ePoint->resampleChecked = true;

			sPoint = ePoint;
			lsum = 0;
			continue;
		}

		lsum += Edge->CalLength();
		if (lsum > length) {
			
			ePoint->resampleChecked = true;
			sPoint = ePoint;
			lsum = 0;
		}
		else {
			sPoint = ePoint;
		}
	}

	// reorganize the toolpath node order(toolpath_NodeSet)
	std::vector<QMeshNode*> toolpath_NodeSet;
	toolpath_NodeSet.push_back(sNode);
	for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);

		ePoint = Edge->GetStartPoint();
		if (ePoint == sNode) ePoint = Edge->GetEndPoint();

		if (ePoint->resampleChecked)	toolpath_NodeSet.push_back(ePoint);

		sNode = ePoint;
	}

	////DEBUG
	//std::cout << "---------------------------------- " << std::endl;
	//for (int i = 0; i < toolpath_NodeSet.size(); i++) {
	//	std::cout << "Node " << toolpath_NodeSet[i]->GetIndexNo() << std::endl;
	//}
	////END

	//rebuild the edge
	patch->GetEdgeList().RemoveAll();
	for (int i = 0; i < ( toolpath_NodeSet.size() - 1); i++)
		buildNewEdgetoQMeshPatch(patch, toolpath_NodeSet[i], toolpath_NodeSet[i + 1]);

	//printf("--> Resample Toolpath Finish\n");
}

//void toolpathGeneration::output_toolpath(PolygenMesh* toolPath, bool onORoff) {
//
//	if (!onORoff) return;
//
//	std::string TOOLPATH_waypoint_dir = "../DataSet/TOOL_PATH/";
//	this->_remove_allFile_in_Dir(TOOLPATH_waypoint_dir);
//	std::string CURVED_layer_dir = "../DataSet/CURVED_LAYER/";
//	this->_remove_allFile_in_Dir(CURVED_layer_dir);
//
//	std::vector<QMeshPatch*> toolpath_list;
//
//	std::vector<QMeshPatch*> layer_list;
//
//	int min_CompatibleLayer_ind = 1000000; int max_CompatibleLayer_ind = -1000000;
//	for (GLKPOSITION Pos = toolPath->GetMeshList().GetHeadPosition(); Pos;) {
//		QMeshPatch* each_toolpath = (QMeshPatch*)toolPath->GetMeshList().GetNext(Pos);
//
//		if (each_toolpath->compatible_layer_Index > max_CompatibleLayer_ind)
//			max_CompatibleLayer_ind = each_toolpath->compatible_layer_Index;
//
//		if (each_toolpath->compatible_layer_Index < min_CompatibleLayer_ind)
//			min_CompatibleLayer_ind = each_toolpath->compatible_layer_Index;
//	}
//
//	for (int i = min_CompatibleLayer_ind; i <= max_CompatibleLayer_ind; i++) {
//
//		bool find_Support;
//		if (toolpath_list.size() == 0) find_Support = true;
//		else find_Support = toolpath_list.back()->is_SupportLayer;
//
//		// get split num of layer/toolpath i
//		int splitNum = _getSplitNumber_with_CompatibleLayer_ind(i, toolPath, find_Support);
//
//		for (int j = 0; j < splitNum; j++) {
//			QMeshPatch* founded_toolpath = _getToolpath_with_CompatibleLayer_ind(i, toolPath, find_Support);
//		
//			if (founded_toolpath != NULL) {	
//				toolpath_list.push_back(founded_toolpath); 
//				layer_list.push_back(founded_toolpath->attached_Layer);
//				//std::cout << "founded_toolpath: " << founded_toolpath->compatible_layer_Index << std::endl;
//			}
//			find_Support = !find_Support;
//		}
//	}
//
//	//protection code
//	int toolpath_Num = toolPath->GetMeshList().GetCount();
//	std::cout << "toolpath_Num: " << toolpath_Num << std::endl;
//	if (toolpath_list.size() != toolpath_Num) {
//		std::cout << "toolpath_list.size(): " << toolpath_list.size() << std::endl;
//		std::cout << "Error: toolpath number is not same with toolpath list. please check" << std::endl;
//		return;
//	}
//
//	//output waypoints
//	for (int i = 0; i < toolpath_list.size(); i++) {
//
//		QMeshPatch* each_toolpath = toolpath_list[i];
//
//		std::string eachToolpath_dir;
//		if (each_toolpath->is_SupportLayer) {
//			eachToolpath_dir = TOOLPATH_waypoint_dir + std::to_string(i) + "S.txt";
//		}
//		else {
//			eachToolpath_dir = TOOLPATH_waypoint_dir + std::to_string(i) + ".txt";
//		}
//
//		//std::cout << "Output File: " << TOOLPATH_dir << std::endl;
//
//		std::ofstream toolpathFile(eachToolpath_dir);
//
//		double pp[3]; double n[3];
//		QMeshEdge* sEdge = (QMeshEdge*)each_toolpath->GetEdgeList().GetHead(); // the first Edge of Toolpath
//		QMeshNode* sNode = sEdge->GetStartPoint();
//		sNode->GetCoord3D(pp[0], pp[1], pp[2]); sNode->GetNormal(n[0], n[1], n[2]);
//		toolpathFile << pp[0] << " " << pp[1] << " " << pp[2] << " " << n[0] << " " << n[1] << " " << n[2] << std::endl;
//
//		for (GLKPOSITION posEdge = each_toolpath->GetEdgeList().GetHeadPosition(); posEdge != nullptr;) {
//			QMeshEdge* Edge = (QMeshEdge*)each_toolpath->GetEdgeList().GetNext(posEdge);
//
//			//std::cout << "start Node " << Edge->GetStartPoint()->GetIndexNo() << " end Node " << Edge->GetEndPoint()->GetIndexNo() << std::endl;
//
//			QMeshNode* eNode = Edge->GetStartPoint();
//			if (eNode == sNode) eNode = Edge->GetEndPoint();
//
//			eNode->GetCoord3D(pp[0], pp[1], pp[2]); eNode->GetNormal(n[0], n[1], n[2]);
//			toolpathFile << pp[0] << " " << pp[1] << " " << pp[2] << " " << n[0] << " " << n[1] << " " << n[2] << std::endl;
//
//			sNode = eNode;
//		}
//		toolpathFile.close();
//	}
//	std::cout << "Finish output toolpath into : " << ".. / DataSet / TOOL_PATH /" << std::endl;
//
//	//output initial and slimmed support layer
//	for (int i = 0; i < layer_list.size(); i++) {
//
//		QMeshPatch* each_layer = layer_list[i];
//
//		std::string eachLayer_dir;
//		if (each_layer->is_SupportLayer) {
//			eachLayer_dir = CURVED_layer_dir + std::to_string(i) + "S.obj";
//		}
//		else {
//			eachLayer_dir = CURVED_layer_dir + std::to_string(i) + ".obj";
//		}
//
//		//std::cout << "Output File: " << TOOLPATH_dir << std::endl;
//
//		std::ofstream layer_str(eachLayer_dir);
//		int index = 0; double pp[3];
//		for (GLKPOSITION posNode = each_layer->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
//			QMeshNode* node = (QMeshNode*)each_layer->GetNodeList().GetNext(posNode);
//			node->GetCoord3D(pp[0], pp[1], pp[2]);
//			layer_str << "v " << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
//			index++; node->SetIndexNo(index);
//		}
//		for (GLKPOSITION posFace = each_layer->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
//			QMeshFace* face = (QMeshFace*)each_layer->GetFaceList().GetNext(posFace);
//			layer_str << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
//				<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
//				<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << std::endl;
//		}
//		layer_str.close();
//	}
//	std::cout << "Finish output layers into : " << ".. / DataSet / CURVED_LAYER /" << std::endl;
//}
//
//QMeshPatch* toolpathGeneration::_getToolpath_with_CompatibleLayer_ind(
//	int compatibleLayer_ind, PolygenMesh* toolPath, bool find_Support) {
//
//	QMeshPatch* founded_toolpath = NULL;
//
//	for (GLKPOSITION Pos = toolPath->GetMeshList().GetHeadPosition(); Pos;) {
//		QMeshPatch* each_toolpath = (QMeshPatch*)toolPath->GetMeshList().GetNext(Pos);
//
//		if (each_toolpath->compatible_layer_Index == compatibleLayer_ind) {
//			if (each_toolpath->is_SupportLayer == find_Support) {
//				if (!each_toolpath->isInstalled_toolpath) {
//					founded_toolpath = each_toolpath;
//					each_toolpath->isInstalled_toolpath = true;
//					break;
//				}
//			}
//		}
//	}
//
//	return founded_toolpath;
//}
//
//int toolpathGeneration::_getSplitNumber_with_CompatibleLayer_ind(
//	int compatibleLayer_ind, PolygenMesh* toolPath, bool isSupport) {
//
//	int splitNumber = 0;
//
//	for (GLKPOSITION Pos = toolPath->GetMeshList().GetHeadPosition(); Pos;) {
//		QMeshPatch* each_toolpath = (QMeshPatch*)toolPath->GetMeshList().GetNext(Pos);
//
//		if (each_toolpath->compatible_layer_Index == compatibleLayer_ind) {
//			if (each_toolpath->is_SupportLayer == isSupport) {
//
//				splitNumber++;
//			}
//		}
//	}
//
//	return splitNumber;
//}
//
//int toolpathGeneration::_remove_allFile_in_Dir(std::string dirPath) {
//
//	struct _finddata_t fb;   //find the storage structure of the same properties file.
//	std::string path;
//	intptr_t    handle;
//	int  resultone;
//	int   noFile;            // the tag for the system's hidden files
//
//	noFile = 0;
//	handle = 0;
//
//	path = dirPath + "/*";
//
//	handle = _findfirst(path.c_str(), &fb);
//
//	//find the first matching file
//	if (handle != -1)
//	{
//		//find next matching file
//		while (0 == _findnext(handle, &fb))
//		{
//			// "." and ".." are not processed
//			noFile = strcmp(fb.name, "..");
//
//			if (0 != noFile)
//			{
//				path.clear();
//				path = dirPath + "/" + fb.name;
//
//				//fb.attrib == 16 means folder
//				if (fb.attrib == 16)
//				{
//					_remove_allFile_in_Dir(path);
//				}
//				else
//				{
//					//not folder, delete it. if empty folder, using _rmdir instead.
//					remove(path.c_str());
//				}
//			}
//		}
//		// close the folder and delete it only if it is closed. For standard c, using closedir instead(findclose -> closedir).
//		// when Handle is created, it should be closed at last.
//		_findclose(handle);
//		return 0;
//	}
//}

// hybrid toolpath generation
// use heat source for zigzag also
void toolpathGeneration::generate_all_hybrid_toolPath() {

	if (toolpath_Width <= 0.0 || toolpath_Distance <= 0.0) {

		std::cout << "width or distance is zero!" << std::endl;
		return;
	}

	int layerNum = m_isoLayerSet->GetMeshList().GetCount();
	std::vector<QMeshPatch*> sliceVector(layerNum);
	std::vector<QMeshPatch*> toolpathVector(layerNum);

	int tempInd = 0;
	for (GLKPOSITION posMesh = m_isoLayerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* layer = (QMeshPatch*)m_isoLayerSet->GetMeshList().GetNext(posMesh);

		sliceVector[tempInd] = layer;
		tempInd++;
	}

	//tempUse : for armadillo stress toolpath generation
	if (false) this->_importStressField4toolpathGeneration();


#pragma omp parallel
	{
#pragma omp for  
		for (int i = 0; i < sliceVector.size(); i++) {

			/* ---- Generate boundary heat field ---- */
			QMeshPatch* layer = sliceVector[i];
			heatMethodField* heatField_layer = new heatMethodField(layer);

			heatField_layer->meshRefinement(); //it will break original flag
			heatField_layer->compBoundaryHeatKernel();
			if (true) heatField_layer->compZigZagFieldValue();// use vector/scalar to calculate zigzag value
			else heatField_layer->compZigZagHeatKernel();// use heat method
			delete heatField_layer;
			/* ---- END ---- */

			QMeshPatch* singlePath = this->generate_each_hybrid_bundaryToolPath(layer);
			this->resampleToolpath(singlePath);
			this->calToolPath_length(singlePath);

			if (singlePath != NULL) {
				if (layer->is_SupportLayer) {
					singlePath->is_SupportLayer = layer->is_SupportLayer;
				}
				singlePath->compatible_layer_Index = layer->compatible_layer_Index;
				singlePath->attached_Layer = layer;
			}

			toolpathVector[layer->GetIndexNo()] = singlePath;
		}
	}

	std::cout << "\nThe number means Toolpath i is NULL, skip it." << std::endl;
	for (int i = 0; i < layerNum; i++) {

		if ((i + 1) % 100 == 0) std::cout << std::endl;
		if (toolpathVector[i] != NULL) std::cout << ".";
		else std::cout << i;

		if (toolpathVector[i] != NULL) {
			QMeshPatch* singlePath = toolpathVector[i];
			singlePath->SetIndexNo(i);
			m_toolPathSet->GetMeshList().AddTail(singlePath);
		}
	}
	std::cout << std::endl;
}

QMeshPatch* toolpathGeneration::generate_each_hybrid_bundaryToolPath(QMeshPatch* surfaceMesh) {

	int curveNum = autoComputeTPathNum(surfaceMesh, true);	int boundaryTp_num = 2;

	bool only_boundary = false;

	if (curveNum > 500) {
		std::cout << "layer = " << surfaceMesh->GetIndexNo() << " high curve num " << curveNum << "." << std::endl;
	}
	if (curveNum <= 0 || curveNum > 500) { return NULL; }

	if (curveNum < boundaryTp_num) {
		curveNum = boundaryTp_num; // small layers needs at lest boundaryTp_num rings of boundary loop
		only_boundary = true;
	}
	std::vector<double> isoValue_boundary(curveNum);// store all isoValue
	double deltaIsoValue_boundary = 1.0 / curveNum; // deltaIsoValue in [0-1]
	QMeshPatch* singlePath = new QMeshPatch;
	singlePath->SetIndexNo(surfaceMesh->GetIndexNo());

	for (int i = (curveNum - boundaryTp_num); i < curveNum; i++) {

		isoValue_boundary[i] = (i + 0.5) * deltaIsoValue_boundary;
		//build isonode with each isoValue without linkage
		generateBoundaryIsoNode(singlePath, surfaceMesh, isoValue_boundary[i]);
	}
	if (singlePath->GetNodeNumber() == 0)
		std::cout << "Warning! the boundary toolpath contains no isonode!" << std::endl;  //means no node in singlePath

	//DEBUG
	//for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode* thisNode = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);
	//	std::cout << thisNode->GetIndexNo() << std::endl;
	//}

	double min_isoValue = isoValue_boundary[curveNum - boundaryTp_num];
	double max_isoValue = isoValue_boundary[curveNum - 1];
	double startIsoValue;
	//choose printing direction (inner -> outside(true)) or inverse(false)
	bool growDirction = true;
	if (growDirction)	startIsoValue = min_isoValue;
	else startIsoValue = max_isoValue;
	linkEachIsoNode(singlePath, startIsoValue);

	//DEBUG
	/*for (GLKPOSITION posEdge = singlePath->GetEdgeList().GetHeadPosition(); posEdge != nullptr;) {
		QMeshEdge* Edge = (QMeshEdge*)singlePath->GetEdgeList().GetNext(posEdge);
		std::cout << "start Node " << Edge->GetStartPoint()->GetIndexNo() << " end Node " << Edge->GetEndPoint()->GetIndexNo() << std::endl;
	}*/
	//END

	if (only_boundary) return singlePath;
	//return singlePath;
	//printf("--> Bundary Toolpath Generation Finish\n");


	/*Zigzag toolpath generation*/
	// toolpath node
	int contourNode_number = singlePath->GetNodeNumber();
	int zigzagNum = autoComputeTPathNum(surfaceMesh, false);
	if (zigzagNum > 500) {
		std::cout << "layer = " << surfaceMesh->GetIndexNo()
			<< " high zigzag num " << zigzagNum << "." << std::endl;
	}
	if (zigzagNum <= 0 || zigzagNum > 500) { return singlePath; }

	std::vector<double> isoValue_zigzag(zigzagNum);		// store all isoValue
	double deltaIsoValue_zigzag = 1.0 / zigzagNum;		// deltaIsoValue in [0-1]
	double boundaryField_threshold = 1.0 - boundaryTp_num * deltaIsoValue_boundary;

	for (int i = 0; i < zigzagNum; i++) {

		isoValue_zigzag[i] = (i + 0.5) * deltaIsoValue_zigzag;
		//build isonode with each isoValue without linkage
		generateZigzagIsoNode(singlePath, surfaceMesh, isoValue_zigzag[i], boundaryField_threshold);
	}
	if (singlePath->GetNodeNumber() == contourNode_number) {
		std::cout << "Warning! the zigzag toolpath of layer " 
			<< singlePath->GetIndexNo() <<" contains no isonode!" << std::endl;
		return singlePath;
	}


	// link each zigzag node
	// find the link node between boundary and zigzag (last_boundary_node)
	QMeshEdge* last_Edge = (QMeshEdge*)singlePath->GetEdgeList().GetTail();
	QMeshNode* startNode_lastEdge = last_Edge->GetStartPoint();
	QMeshNode* endNode_lastEdge = last_Edge->GetEndPoint();
	QMeshNode* last_boundaryNode = NULL;
	if ((startNode_lastEdge->GetEdgeNumber() == 2) && (endNode_lastEdge->GetEdgeNumber() == 1))	
		last_boundaryNode = endNode_lastEdge;
	else if((startNode_lastEdge->GetEdgeNumber() == 1) && (endNode_lastEdge->GetEdgeNumber() == 2))
		last_boundaryNode = startNode_lastEdge;
	else { std::cout << "error: there should be only one node without two neighbor." << std::endl; }
	
	linkEachIsoNode_zigzag(singlePath, last_boundaryNode);


	return singlePath;
}

void toolpathGeneration::generateZigzagIsoNode(QMeshPatch* singlePath, 
	QMeshPatch* surfaceMesh, double isoValue, double boundaryField_threshold) {

	//----build the node and install back to the "singlePath"
	for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);

		double a = Edge->GetStartPoint()->zigzagValue;
		double b = Edge->GetEndPoint()->zigzagValue;
		double boundaryValue = 0;

		if ((isoValue - a) * (isoValue - b) < 0.0) {
			double alpha = (isoValue - a) / (b - a);
			double p1[3], p2[3], pp[3];
			Edge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
			Edge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

			for (int j = 0; j < 3; j++) {
				//compute the position for this isonode
				pp[j] = (1.0 - alpha) * p1[j] + alpha * p2[j];
				//compute boundary value for this isonode
				boundaryValue = (1.0 - alpha) * Edge->GetStartPoint()->boundaryValue +
					alpha * Edge->GetEndPoint()->boundaryValue;
			}

			//detect if the node inside the boundary region // key step for zigzag shrink
			if (boundaryValue > boundaryField_threshold) continue;

			QMeshNode* isoNode = new QMeshNode;
			isoNode->relatedLayerEdge = Edge;
			isoNode->connectTPathProcessed = false;
			isoNode->isoValue = isoValue;
			isoNode->isZigzagNode = true;

			isoNode->SetMeshPatchPtr(singlePath);
			isoNode->SetCoord3D(pp[0], pp[1], pp[2]);
			//index start from boundaryNode num
			isoNode->SetIndexNo(singlePath->GetNodeList().GetCount()); 

			//cal normal of iso point
			double n1[3], n2[3]; Eigen::Vector3d n3;
			if (Edge->GetLeftFace() != NULL) Edge->GetLeftFace()->GetNormal(n1[0], n1[1], n1[2]);
			if (Edge->GetRightFace() != NULL) Edge->GetRightFace()->GetNormal(n2[0], n2[1], n2[2]);
			for (int i = 0; i < 3; i++) n3[i] = (n1[i] + n2[i]) / 2;
			if (n3.norm() < 0.01) std::cerr << "toolpath - zigzag - NORMAL ERROR!" << std::endl;
			n3 = n3.normalized();
			isoNode->SetNormal(n3[0], n3[1], n3[2]);

			//install this isoNode of layer to its Edge
			Edge->installedIsoNode_layerEdge_zigzag.push_back(isoNode);
			Edge->isLocateIsoNode_layerEdge_zigzag = true;
			singlePath->GetNodeList().AddTail(isoNode);
		}
	}

	//----Detect the iso node located at the two side of single tool-path, used for connection
	//Detect the side node by recording the nuber of isonode in the edges of neighbor face. 
	//If it is 2, middle node. If it is 1, side node.
	//Notice that, the edge contains zig-zag node normally not located at the boundary. If its a boundary edge, we directly
	//set this isonode as boundary of zig-zag tool-path.

	int zigzagPath_sideNode = 0;
	for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);

		if (!thisNode->isZigzagNode) continue;
		QMeshEdge* thisEdge = thisNode->relatedLayerEdge;

		thisNode->isZigzag_sideNode = false;

		if (thisEdge->IsBoundaryEdge()) {
			thisNode->isZigzag_sideNode = true; zigzagPath_sideNode++;
			std::cout << "Warning:: Notice that, the zigzag toolpath node related to a boundary edge! " <<
				"You can try to make the boundary tool-path offset higher." << std::endl;
			continue;
		}

		int lFaceIsoNodeNum = 0; //left face
		for (int i = 0; i < 3; i++) {
			QMeshEdge* NeighborEdge = thisEdge->GetLeftFace()->GetEdgeRecordPtr(i + 1);
			if (NeighborEdge == thisEdge) continue;
			if (NeighborEdge->isLocateIsoNode_layerEdge_zigzag) {
				for (int j = 0; j < NeighborEdge->installedIsoNode_layerEdge_zigzag.size(); j++) {
					if (NeighborEdge->installedIsoNode_layerEdge_zigzag[j]->isoValue
						== thisNode->isoValue) {
						lFaceIsoNodeNum++;
					}
				}
			}
		}
		int rFaceIsoNodeNum = 0; //right face
		for (int i = 0; i < 3; i++) {
			QMeshEdge* NeighborEdge = thisEdge->GetRightFace()->GetEdgeRecordPtr(i + 1);
			if (NeighborEdge == thisEdge) continue;
			if (NeighborEdge->isLocateIsoNode_layerEdge_zigzag) {
				for (int j = 0; j < NeighborEdge->installedIsoNode_layerEdge_zigzag.size(); j++) {
					if (NeighborEdge->installedIsoNode_layerEdge_zigzag[j]->isoValue
						== thisNode->isoValue) {
						rFaceIsoNodeNum++;
					}
				}
			}
		}
		// isolated node, with no left and right boundary node connected, we ignore this region.
		if (lFaceIsoNodeNum == 0 && rFaceIsoNodeNum == 0) {
			thisNode->connectTPathProcessed = true; continue;
		}

		if (lFaceIsoNodeNum + rFaceIsoNodeNum < 2) {
			thisNode->isZigzag_sideNode = true;
			zigzagPath_sideNode++;
		}
	}
	if (zigzagPath_sideNode % 2 != 0) printf("zigzag path boundary node number error! should be even number\n");
}

void toolpathGeneration::linkEachIsoNode_zigzag(QMeshPatch* singlePath, QMeshNode* last_boundaryNode) {

	// get a First Node(zigzag FirstNode) to start the toolPath
	QMeshNode* zigzagFirstNode = nullptr;		double min_dist_L2 = 1.0e10;
	Eigen::Vector3d coord3d; last_boundaryNode->GetCoord3D(coord3d[0], coord3d[1], coord3d[2]);
	for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);
		if (!thisNode->isZigzag_sideNode) continue;
		if (thisNode->connectTPathProcessed == false) {

			Eigen::Vector3d inquire_coord3d; 
			thisNode->GetCoord3D(inquire_coord3d[0], inquire_coord3d[1], inquire_coord3d[2]);
			double dist_L2 = (inquire_coord3d - coord3d).squaredNorm();
			if (min_dist_L2 > dist_L2) { 
				min_dist_L2 = dist_L2;
				zigzagFirstNode = thisNode;
			}
		}
	}

	zigzagFirstNode->connectTPathProcessed = true;
	// protect output
	if (zigzagFirstNode == NULL) std::cout << "Cannot find the start point!, please check." << std::endl;

	QMeshEdge* link_Edge = buildNewEdgetoQMeshPatch(singlePath, last_boundaryNode, zigzagFirstNode);
	link_Edge->isConnectEdge = true;
	QMeshNode* sNode = zigzagFirstNode;
	QMeshNode* eNode = findNextZigzagToolPath(sNode, singlePath);

	/* Link all of iso-Node in one Layer*/
	do {

		// Start linking one ring with same iso-Value
		QMeshNode* unlinked_eNode = link_OneLine_isoNode(singlePath, sNode, eNode);
		if (unlinked_eNode == nullptr) return;

		if (detectAll_isoNode_Processed(singlePath)) return;

		QMeshNode* link_sNode = unlinked_eNode;
		QMeshNode* link_eNode = findNextNearest_sidePoint(link_sNode, singlePath);
		if (link_eNode == nullptr) {
			std::cout << "Error: Cannot find link_eNode: Layer: "<< singlePath->GetIndexNo() << std::endl;
			return;
		}
		QMeshEdge* link_Edge = buildNewEdgetoQMeshPatch(singlePath, link_sNode, link_eNode);

		link_Edge->isConnectEdge = true;

		sNode = link_eNode;
		eNode = findNextZigzagToolPath(sNode, singlePath);
		if (eNode == nullptr) std::cout << "Error: Cannot find next toolpath Node." << std::endl;

	} while (detectAll_isoNode_Processed(singlePath) == false);
}

QMeshNode* toolpathGeneration::findNextZigzagToolPath(QMeshNode* sNode, QMeshPatch* singlePath) {

	QMeshNode* eNode;
	bool nextNodeDetected = false;
	QMeshEdge* thisEdge = sNode->relatedLayerEdge;

	//detect left face
	for (int i = 0; i < 3; i++) {
		QMeshEdge* NeighborEdge = thisEdge->GetLeftFace()->GetEdgeRecordPtr(i + 1);
		if (NeighborEdge == thisEdge) continue;

		if (NeighborEdge->isLocateIsoNode_layerEdge_zigzag) {

			for (int j = 0; j < NeighborEdge->installedIsoNode_layerEdge_zigzag.size(); j++) {

				QMeshNode* bNode = NeighborEdge->installedIsoNode_layerEdge_zigzag[j];
				if (bNode->connectTPathProcessed == false && bNode->isoValue == sNode->isoValue) {
					nextNodeDetected = true;
					eNode = bNode;
					break;
				}
			}
		}
		if (nextNodeDetected == true) break;
	}

	if (nextNodeDetected) {
		eNode->connectTPathProcessed = true;
		return eNode;
	}

	//detect right face
	for (int i = 0; i < 3; i++) {
		QMeshEdge* NeighborEdge = thisEdge->GetRightFace()->GetEdgeRecordPtr(i + 1);
		if (NeighborEdge == thisEdge) continue;

		if (NeighborEdge->isLocateIsoNode_layerEdge_zigzag) {

			for (int j = 0; j < NeighborEdge->installedIsoNode_layerEdge_zigzag.size(); j++) {

				QMeshNode* bNode = NeighborEdge->installedIsoNode_layerEdge_zigzag[j];
				if (bNode->connectTPathProcessed == false && bNode->isoValue == sNode->isoValue) {
					nextNodeDetected = true;
					eNode = bNode;
					break;
				}
			}
		}
		if (nextNodeDetected == true) break;
	}

	if (nextNodeDetected) {
		eNode->connectTPathProcessed = true;
		return eNode;
	}
	else {
		//std::cout<<"Error, the next node is not found!"<<std::endl;
		return nullptr;
	}
}

QMeshNode* toolpathGeneration::link_OneLine_isoNode(QMeshPatch* singlePath, QMeshNode* sNode, QMeshNode* eNode) {

	do {

		if (sNode == nullptr || eNode == nullptr) {
			std::cout << "NULL sNode or eNode" << std::endl;
			return nullptr;
		}

		buildNewEdgetoQMeshPatch(singlePath, sNode, eNode);

		sNode = eNode;
		eNode = findNextZigzagToolPath(sNode, singlePath);

	} while (eNode != nullptr);

	return sNode;// return the sNode for the link opration between isoNode with different isoValue 
}

QMeshNode* toolpathGeneration::findNextNearest_sidePoint(QMeshNode* sNode, QMeshPatch* singlePath) {

	GLKGeometry geo;
	double pp[3]; sNode->GetCoord3D(pp[0], pp[1], pp[2]);
	double p1[3];
	double dist = 1000.0;
	QMeshNode* nextNode = nullptr;

	for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);
		if (Node->connectTPathProcessed) continue;
		if (!Node->isZigzag_sideNode) continue;
		Node->GetCoord3D(p1[0], p1[1], p1[2]);
		double distancePP = geo.Distance_to_Point(pp, p1);
		if (distancePP < dist) {
			nextNode = Node;
			dist = distancePP;
		}
	}
	if (nextNode == nullptr) {
		std::cout << "There is no side isoNode to link!" << std::endl;
		return nullptr;
	}
	else {
		nextNode->connectTPathProcessed = true;
		return nextNode;
	}
}

//Smooth for the layers which is used to generate toolpath
//temp operation
void toolpathGeneration::smoothingIsoSurface() {

	std::cout << "----------------------------------" << std::endl;

	int smoothLoop_time = 1;
	for (int i = 0; i < smoothLoop_time; i++) {

		for (GLKPOSITION posMesh = m_isoLayerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshPatch* layer = (QMeshPatch*)m_isoLayerSet->GetMeshList().GetNext(posMesh);

			for (GLKPOSITION Pos = layer->GetEdgeList().GetHeadPosition(); Pos;) {
				QMeshEdge* thisEdge = (QMeshEdge*)layer->GetEdgeList().GetNext(Pos);
				if (thisEdge->IsBoundaryEdge()) {
					thisEdge->GetStartPoint()->isoSurfaceBoundary = true;
					thisEdge->GetEndPoint()->isoSurfaceBoundary = true;
				}
			}

			//laplacian smoothness
			for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
				QMeshNode* thisNode = (QMeshNode*)layer->GetNodeList().GetNext(Pos);

				if (thisNode->isoSurfaceBoundary) continue;
				else {
					double pp[3] = { 0 }; int neighNum = 0;
					for (GLKPOSITION Pos = thisNode->GetEdgeList().GetHeadPosition(); Pos;) {
						QMeshEdge* neighEdge = (QMeshEdge*)thisNode->GetEdgeList().GetNext(Pos);

						QMeshNode* neighNode = neighEdge->GetStartPoint();
						if (neighNode == thisNode) neighNode = neighEdge->GetEndPoint();

						double p1[3];
						neighNode->GetCoord3D(p1[0], p1[1], p1[2]);

						for (int i = 0; i < 3; i++) pp[i] += p1[i];
						neighNum++;
					}
					for (int i = 0; i < 3; i++) pp[i] /= neighNum;
					thisNode->SetCoord3D(pp[0], pp[1], pp[2]);
				}
			}
		}
		std::cout << "smooth the layers once before toolpath generation" << std::endl;
	}
	std::cout << "----------------------------------" << std::endl;
}

void toolpathGeneration::_importStressField4toolpathGeneration() {

	Eigen::MatrixXd stressFieldSet = this->_read_stressField();
	//give the stressField vector to face center
	for (GLKPOSITION posMesh = m_isoLayerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* each_layer = (QMeshPatch*)m_isoLayerSet->GetMeshList().GetNext(posMesh);

		std::vector<QMeshFace*> layer_FaceSet(each_layer->GetFaceNumber());
		int tempInd = 0;
		for (GLKPOSITION posFace = each_layer->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
			QMeshFace* face = (QMeshFace*)each_layer->GetFaceList().GetNext(posFace);

			layer_FaceSet[tempInd] = face;
			tempInd++;
		}

		//std::cout << "layer_FaceSet.size()" << layer_FaceSet.size() << std::endl;

#pragma omp parallel
		{
#pragma omp for  
			for (int i = 0; i < layer_FaceSet.size(); i++) {
				QMeshFace* face = layer_FaceSet[i];

				this->_findStressVector(face, stressFieldSet);
				this->_getFaceNeighbor(face);
			}
		}

		//printf("step1\n");

		for (int smoothLoop = 0; smoothLoop < 2000; smoothLoop++) {
			for (GLKPOSITION posFace = each_layer->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
				QMeshFace* face = (QMeshFace*)each_layer->GetFaceList().GetNext(posFace);

				//if (face->stessFilament_flag) continue;
				for (int i = 0; i < face->neighFace.size(); i++) {
					face->stessFilament_vector += face->neighFace[i]->stessFilament_vector;
				}
				face->stessFilament_vector.normalize();
			}
		}
		//printf("step2:smooth\n");

		//for (int i = 0; i < layer_FaceSet.size(); i++) {
		//	QMeshFace* face = layer_FaceSet[i];
		//	if (face->stessFilament_flag)
		//		std::cout << "face stressVector" << face->stessFilament_vector.transpose() << std::endl;
		//}
		//is or not critical layer with stress concern
		for (GLKPOSITION posFace = each_layer->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
			QMeshFace* face = (QMeshFace*)each_layer->GetFaceList().GetNext(posFace);

			if (face->stessFilament_flag == true) {
				each_layer->isStressLayer = true;
				//printf("\nface: %d\n", face->GetIndexNo());
				//break;
			}
		}
		//printf("step3\n");

		//printf("layer: %d\n", each_layer->GetIndexNo());
	}

}

Eigen::MatrixXd toolpathGeneration::_read_stressField() {

	char filename[1024];
	//bunny_cut6
	std::sprintf(filename, "%s%s%s", "../DataSet/fabricationTest/test_stressField/", "armadillo4", "_stressField.txt");
	std::cout << "\nStress Field is read from:\n" << filename << std::endl;

	FILE* fp;   char linebuf[256];  int i_temp = 0;
	fp = fopen(filename, "r");
	if (!fp) {
		printf("===============================================\n");
		printf("Can not open the data file 1!\n");
		printf("===============================================\n");
	}
	//get the line number!
	while (1) { // right read CODE !!!
		fgets(linebuf, 255, fp);
		if (feof(fp)) break;
		i_temp++;
	}
	fclose(fp);
	int criticalEle_Num = i_temp;
	Eigen::MatrixXd stressFieldSet = Eigen::MatrixXd::Zero(criticalEle_Num, 6);

	fp = fopen(filename, "r");
	if (!fp) {
		printf("===============================================\n");
		printf("Can not open the data file 2!\n");
		printf("===============================================\n");
	}
	float xx, yy, zz, length, tau_x, tau_y, tau_z, color_x, color_y, color_z;
	i_temp = 0;
	while (1) { // right read CODE !!!
		fgets(linebuf, 255, fp);
		if (feof(fp)) break;
		sscanf(linebuf, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
			&xx, &yy, &zz, &length, &tau_x, &tau_y, &tau_z, &color_x, &color_y, &color_z);
		stressFieldSet.row(i_temp) << xx, yy, zz, tau_x, tau_y, tau_z;
		i_temp++;
	}
	fclose(fp);
	//std::cout << stressFieldSet << std::endl;

	return stressFieldSet;
}

void toolpathGeneration::_findStressVector(QMeshFace* face, Eigen::MatrixXd& stressFieldSet) {

	Eigen::Vector3d centorPos = { 0.0,0.0,0.0 };
	face->CalCenterPos();
	face->GetCenterPos(centorPos[0], centorPos[1], centorPos[2]);

	Eigen::Vector3d nearest_stressVector_pos = { 0.0,0.0,0.0 };
	double minDist = 9999999.9;
	for (int i = 0; i < stressFieldSet.rows(); i++) {

		Eigen::Vector3d temp_stressVector_pos
			= { stressFieldSet(i,0),stressFieldSet(i,1),stressFieldSet(i,2) };
		double tempDist = (centorPos - temp_stressVector_pos).norm();
		if (tempDist > 5.0) continue;
		if (tempDist < minDist) {
			minDist = tempDist;
			face->stessFilament_flag = true;
			if (stressFieldSet(i, 5) > 0.0) {
				face->stessFilament_vector
					= { stressFieldSet(i,3),stressFieldSet(i,4),stressFieldSet(i,5) };
			}
			else {
				face->stessFilament_vector
					= { -stressFieldSet(i,3),-stressFieldSet(i,4),-stressFieldSet(i,5) };
			}
		}
	}
	if (face->stessFilament_vector[0] == 0 && face->stessFilament_vector[1] == 0 && face->stessFilament_vector[2] == 0) {

	}
	else{
	
		//std::cout << "face index " << face->GetIndexNo() << std::endl;
		//std::cout << "the stress vector on face is " << face->stessFilament_vector.transpose() << std::endl;
	}
}

void toolpathGeneration::_getFaceNeighbor(QMeshFace* face) {

	for (int i = 0; i < 3; i++) {
		QMeshFace* temp_leftFace = face->GetEdgeRecordPtr(i+1)->GetLeftFace();
		QMeshFace* temp_rightFace = face->GetEdgeRecordPtr(i+1)->GetRightFace();
		if (temp_leftFace == NULL || temp_rightFace == NULL) continue;
		QMeshFace* NeighbourFace = NULL;
		if (temp_leftFace == face) NeighbourFace = temp_rightFace;
		else NeighbourFace = temp_leftFace;
		if (NeighbourFace == NULL) std::cout << "error: neighbor face is NULL!" << std::endl;
		face->neighFace.push_back(NeighbourFace);
		if (face->neighFace.size() == 0 || face->neighFace.size() > 3)
			std::cout << "neighFace size error!" << std::endl;
	}
}

void toolpathGeneration::calToolPath_length(QMeshPatch* patch) {
	if (patch == NULL) return;

	double total_length = 0.0;
	double threshold_jump = toolpath_Width * 4.5;
	for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);
		Edge->isSpecialShow = false;

		double length = Edge->CalLength();
		if (length < threshold_jump)
			total_length += length;
		else
			Edge->isSpecialShow = true;
	}

	//std::cout << "total_length = " << total_length << std::endl;
}


















