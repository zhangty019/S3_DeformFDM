#include <fstream>

#include "io.h"
#include "IsoLayerGeneration.h"

void IsoLayerGeneration::generateIsoSurface(PolygenMesh* isoSurface, int layerNum) {

	double isoCurveValue = 0.0;
	for (int i = 0; i < layerNum; i++) {

		isoCurveValue = (0.5 + i) * 1 / (double)layerNum;
		QMeshPatch* layer = _generatesingleIsoSurface(isoCurveValue, isoSurface);

		if (layer->GetNodeNumber() == 0) {
			std::cout << "this layer have no node!" << std::endl; continue;
		}
		layer->is_SupportLayer = false;
		layer->SetIndexNo(isoSurface->GetMeshList().GetCount());//index begin from 0
		layer->compatible_layer_Index = layer->GetIndexNo();
		isoSurface->meshList.AddTail(layer);
		std::cout << layer->GetIndexNo() << " Layer, isoValue = " << isoCurveValue << ", nodeNum = " << layer->GetNodeNumber() << std::endl;
	}
}

/*Main function for iso-surface substraction from tetrahedral model*/
QMeshPatch* IsoLayerGeneration::_generatesingleIsoSurface(double isoValue, PolygenMesh* isoSurface){

	QMeshPatch* layer = new QMeshPatch;
	layer->isoSurfaceValue = isoValue;

	//when the node iso-value is equal to surface value, add this eps.
	double eps = 1.0e-5;

	for (GLKPOSITION Pos = m_tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetMesh->GetNodeList().GetNext(Pos);
		if (fabs(Node->scalarField - isoValue) < eps) {
			if (Node->scalarField > isoValue) Node->scalarField = isoValue + eps;
			else Node->scalarField = isoValue - eps;
		}
	}

	// build node list for isoSurface, this comes from the edge list
	for (GLKPOSITION Pos = m_tetMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)m_tetMesh->GetEdgeList().GetNext(Pos);

		Edge->installedIsoNode = nullptr;
		Edge->isLocateIsoNode = false;

		double a = Edge->GetStartPoint()->scalarField;
		double b = Edge->GetEndPoint()->scalarField;

		if ((isoValue - a) * (isoValue - b) < 0.0) {
			double alpha = (isoValue - a) / (b - a);
			double p1[3], p2[3], pp[3];
			Edge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
			Edge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

			for (int j = 0; j < 3; j++)
				pp[j] = (1.0 - alpha) * p1[j] + alpha * p2[j];

			QMeshNode* isoNode = new QMeshNode;
			isoNode->relatedTetEdge = Edge;
			isoNode->SetMeshPatchPtr(layer);
			isoNode->SetCoord3D(pp[0], pp[1], pp[2]);
			isoNode->SetIndexNo(layer->GetNodeList().GetCount() + 1);
			layer->GetNodeList().AddTail(isoNode);

			Edge->installedIsoNode = isoNode;
			Edge->isLocateIsoNode = true;

		}
	}

	// build edge list for isoSurface, this comes from the face list
	for (GLKPOSITION Pos = m_tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)m_tetMesh->GetFaceList().GetNext(Pos);
		Face->installedIsoEdge = nullptr;
		Face->isLocatedIsoEdge = false;

		int positiveNum = 0;
		for (int i = 0; i < 3; i++) {
			if (Face->GetNodeRecordPtr(i)->scalarField > isoValue) positiveNum++;
		}

		if (positiveNum == 0 || positiveNum == 3) continue;
		else if (positiveNum == 1) {
			QMeshEdge* isoEdge = new QMeshEdge;

			//detect which node is positive
			QMeshNode* PostiveNode;
			int index = 0;
			for (index = 0; index < 3; index++) {
				PostiveNode = Face->GetNodeRecordPtr(index);
				if (PostiveNode->scalarField > isoValue) break;
			}

			QMeshEdge* Edge = Face->GetEdgeRecordPtr(index + 1);
			QMeshNode* startNode = Edge->installedIsoNode;
			isoEdge->SetStartPoint(startNode);

			Edge = Face->GetEdgeRecordPtr((index + 2) % 3 + 1);
			QMeshNode* endNode = Edge->installedIsoNode;
			isoEdge->SetEndPoint(endNode);

			isoEdge->SetMeshPatchPtr(layer);
			isoEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);

			(startNode->GetEdgeList()).AddTail(isoEdge);
			(endNode->GetEdgeList()).AddTail(isoEdge);

			layer->GetEdgeList().AddTail(isoEdge);
			Face->installedIsoEdge = isoEdge;
			Face->isLocatedIsoEdge = true;
		}
		else if (positiveNum == 2) {
			QMeshEdge* isoEdge = new QMeshEdge;
			//detect which node is negative
			QMeshNode* NegativeNode;
			int index = 0;
			for (index = 0; index < 3; index++) {
				NegativeNode = Face->GetNodeRecordPtr(index);
				if (NegativeNode->scalarField < isoValue) break;
			}

			QMeshEdge* Edge = Face->GetEdgeRecordPtr((index + 2) % 3 + 1);
			QMeshNode* startNode = Edge->installedIsoNode;
			isoEdge->SetStartPoint(startNode);

			Edge = Face->GetEdgeRecordPtr(index + 1);
			QMeshNode* endNode = Edge->installedIsoNode;
			isoEdge->SetEndPoint(endNode);

			isoEdge->SetMeshPatchPtr(layer);
			isoEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);

			(startNode->GetEdgeList()).AddTail(isoEdge);
			(endNode->GetEdgeList()).AddTail(isoEdge);

			layer->GetEdgeList().AddTail(isoEdge);
			Face->installedIsoEdge = isoEdge;
			Face->isLocatedIsoEdge = true;
		}
	}

	// build face list for isoSurface, this comes from the face list
	for (GLKPOSITION Pos = m_tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)m_tetMesh->GetTetraList().GetNext(Pos);

		int isoEdgeNum = 0;
		for (int i = 0; i < 4; i++) {
			if (Tetra->GetFaceRecordPtr(i + 1)->isLocatedIsoEdge == true) isoEdgeNum++;
		}
		if (isoEdgeNum == 0) continue;
		else if (isoEdgeNum == 2 || isoEdgeNum == 1) std::cout << "Error! isoEdgeNum cannot equal to 1 or 2!" << std::endl << std::endl;
		else if (isoEdgeNum == 3) {
			QMeshFace* isoFace = new QMeshFace;
			
			// build the face list
			std::vector<QMeshFace*> FaceList; FaceList.resize(3);
			int faceIndex = 0;
			for (int i = 0; i < 4; i++) {
				if (Tetra->GetFaceRecordPtr(i + 1)->isLocatedIsoEdge == true) {
					FaceList[faceIndex] = Tetra->GetFaceRecordPtr(i + 1);
					faceIndex++;
				}
			}

			//sorting
			bool firstDir = Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])); //detect which one should be the first node!!!
			if (firstDir == true) {
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[2]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[2]->installedIsoEdge->GetEndPoint()) {
					std::swap(FaceList[1], FaceList[2]);
				}
			}
			else {
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[2]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[2]->installedIsoEdge->GetEndPoint()) {
					std::swap(FaceList[1], FaceList[2]);
				}
			}

			//using the first face and add its edge into the isoFace.
			for (int i = 0; i < 3; i++) {
				isoFace->SetEdgeRecordPtr(i, FaceList[i]->installedIsoEdge);
				isoFace->SetDirectionFlag(i, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])));
				if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
					FaceList[i]->installedIsoEdge->SetLeftFace(isoFace);
				else FaceList[i]->installedIsoEdge->SetRightFace(isoFace);
			}

			//push this isoFace back to layer and compute norm
			isoFace->SetEdgeNum(3);
			isoFace->CalPlaneEquation();
			isoFace->SetMeshPatchPtr(layer);
			isoFace->SetIndexNo(layer->GetFaceList().GetCount() + 1);
			layer->GetFaceList().AddTail(isoFace);
		}

		else if (isoEdgeNum == 4) {
			std::vector<QMeshFace*> isoFace; isoFace.resize(2);
			for (int i = 0; i < 2; i++) {
				isoFace[i] = new QMeshFace;
			}
			// build the face list
			std::vector<QMeshFace*> FaceList; FaceList.resize(4);
			int faceIndex = 0;
			for (int i = 0; i < 4; i++) FaceList[i] = Tetra->GetFaceRecordPtr(i + 1);

			bool firstDir = Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])); //detect which one should be the first node!!!
																						//sorting edge
			if (firstDir == true) {
				for (int i = 0; i < 2; i++) {
					if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[i + 2]->installedIsoEdge->GetStartPoint()
						|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[i + 2]->installedIsoEdge->GetEndPoint()) {
						std::swap(FaceList[1], FaceList[i + 2]);
					}
				}
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
				}
				else std::swap(FaceList[2], FaceList[3]);
			}
			else {
				for (int i = 0; i < 2; i++) {
					if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[i + 2]->installedIsoEdge->GetStartPoint()
						|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[i + 2]->installedIsoEdge->GetEndPoint()) {
						std::swap(FaceList[1], FaceList[i + 2]);
					}
				}
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
				}
				else std::swap(FaceList[2], FaceList[3]);

			}
			////test the sorting
			//cout << Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])) << endl;
			//for (int i = 0; i < 4; i++) {
			//	cout << FaceList[i]->installedIsoEdge->GetStartPoint()->GetIndexNo() << " , " <<
			//		FaceList[i]->installedIsoEdge->GetEndPoint()->GetIndexNo() << endl;	
			//}

			QMeshEdge* midEdge1 = new QMeshEdge;
			midEdge1->isMiddleEdge1 = true;
			if (firstDir == true) {
				midEdge1->SetStartPoint(FaceList[0]->installedIsoEdge->GetStartPoint());
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[1]->installedIsoEdge->GetStartPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[1]->installedIsoEdge->GetEndPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 1" << std::endl;
			}
			else {
				midEdge1->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[1]->installedIsoEdge->GetStartPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[1]->installedIsoEdge->GetEndPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 2" << std::endl;
			}

			/*cout << "The midEdge" <<count << "- "<<midEdge1->GetStartPoint()->GetIndexNo() << " , " <<
			midEdge1->GetEndPoint()->GetIndexNo() << endl;
			cout << endl;*/


			QMeshEdge* midEdge2 = new QMeshEdge;
			midEdge2->isMiddleEdge = true;
			if (firstDir == true) {
				midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());

				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 1" << std::endl;
			}
			else {
				midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetStartPoint());

				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 2" << std::endl;
			}

			/*cout << "The midEdge" <<count << "- "<<midEdge2->GetStartPoint()->GetIndexNo() << " , " <<
			midEdge2->GetEndPoint()->GetIndexNo() << endl;
			cout << endl;*/

			//midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());
			/*
			bool dir = false;
			if (FaceList[2]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
			|| FaceList[2]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
			midEdge2->SetEndPoint(FaceList[2]->installedIsoEdge->GetStartPoint());
			dir = true;
			}
			else if(FaceList[2]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
			|| FaceList[2]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
			midEdge2->SetEndPoint(FaceList[2]->installedIsoEdge->GetEndPoint());
			else cout << "Error!" << endl;*/

			QMeshEdge* midEdge;

			if (midEdge1->CalLength() <= midEdge2->CalLength()) {

				midEdge = midEdge1;
				isoFace[0]->SetEdgeRecordPtr(0, FaceList[0]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])));
				isoFace[0]->SetEdgeRecordPtr(1, FaceList[1]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[1])));
				isoFace[0]->SetEdgeRecordPtr(2, midEdge);
				isoFace[0]->SetDirectionFlag(2, false);

				for (int i = 0; i < 4; i++) {
					if (i == 0 || i == 1) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetRightFace(isoFace[0]);

				isoFace[1]->SetEdgeRecordPtr(0, FaceList[2]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[2])));
				isoFace[1]->SetEdgeRecordPtr(1, FaceList[3]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[3])));
				isoFace[1]->SetEdgeRecordPtr(2, midEdge);
				isoFace[1]->SetDirectionFlag(2, true);
				for (int i = 0; i < 4; i++) {
					if (i == 2 || i == 3) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetLeftFace(isoFace[1]);
			}

			else {
				midEdge = midEdge2;
				//first triangle
				isoFace[0]->SetEdgeRecordPtr(0, FaceList[0]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])));
				isoFace[0]->SetEdgeRecordPtr(1, midEdge);
				isoFace[0]->SetDirectionFlag(1, true);
				isoFace[0]->SetEdgeRecordPtr(2, FaceList[3]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(2, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[3])));
				//set edge right / left face
				for (int i = 0; i < 4; i++) {
					if (i == 0 || i == 3) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetLeftFace(isoFace[0]);

				//first triangle
				isoFace[1]->SetEdgeRecordPtr(0, FaceList[1]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[1])));
				isoFace[1]->SetEdgeRecordPtr(1, FaceList[2]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[2])));
				isoFace[1]->SetEdgeRecordPtr(2, midEdge);
				isoFace[1]->SetDirectionFlag(2, false);
				//set edge right / left face
				for (int i = 0; i < 4; i++) {
					if (i == 1 || i == 2) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[1]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[1]);
					}
				}
				midEdge->SetRightFace(isoFace[1]);
			}

			//push back midEdge
			midEdge->SetMeshPatchPtr(layer);
			midEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			layer->GetEdgeList().AddTail(midEdge);


			/*midEdge1->SetMeshPatchPtr(layer);
			midEdge1->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			layer->GetEdgeList().AddTail(midEdge1);*/

			//midEdge2->SetMeshPatchPtr(layer);
			//midEdge2->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			//layer->GetEdgeList().AddTail(midEdge2);



			//push this isoFace back to layer and compute norm
			for (int i = 0; i < 2; i++) {
				isoFace[i]->SetEdgeNum(3);
				isoFace[i]->CalPlaneEquation();
				isoFace[i]->SetMeshPatchPtr(layer);
				isoFace[i]->SetIndexNo(layer->GetFaceList().GetCount() + 1);
				layer->GetFaceList().AddTail(isoFace[i]);
			}
		}
	}

	//give each node face list
	for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
		for (int i = 0; i < 3; i++) {
			QMeshNode* Node = Face->GetNodeRecordPtr(i);
			Node->GetFaceList().AddTail(Face);
		}
	}

	return layer;
}

void IsoLayerGeneration::smoothingIsoSurface(PolygenMesh* isoSurface) {

	for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* isoLayer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);

		for (GLKPOSITION Pos = isoLayer->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge* thisEdge = (QMeshEdge*)isoLayer->GetEdgeList().GetNext(Pos);
			if (thisEdge->IsBoundaryEdge()) {
				thisEdge->GetStartPoint()->isoSurfaceBoundary = true;
				thisEdge->GetEndPoint()->isoSurfaceBoundary = true;
			}
		}

		//laplacian smoothness
		for (GLKPOSITION Pos = isoLayer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)isoLayer->GetNodeList().GetNext(Pos);

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
}

//void IsoLayerGeneration::outputSurfaceMesh(PolygenMesh* isoSurface, bool isRun) {
//
//	if (!isRun) return;
//
//	std::string unremeshed_isoLayer_dir = "../DataSet/remesh_operation/layers_unremeshed/";
//	this->_remove_allFile_in_Dir(unremeshed_isoLayer_dir);
//
//	for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
//		QMeshPatch* each_layer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);
//
//		std::string LAYER_dir = unremeshed_isoLayer_dir + std::to_string(each_layer->GetIndexNo()) 
//														+ "_C" + std::to_string(each_layer->GetIndexNo())
//														+ "_M";
//		_output_OneSurfaceMesh(each_layer, LAYER_dir);
//	}
//	std::cout << "Finish output layers into : " << ".. / DataSet / remesh_operation / layers_unremeshed /" << std::endl;
//}
//
//void IsoLayerGeneration::output_split_SurfaceMesh(PolygenMesh* isoSurface, bool isRun) {
//
//	if (!isRun) return;
//
//	std::string unremeshed_isoLayer_dir = "../DataSet/remesh_operation/layers_unremeshed/";
//	this->_remove_allFile_in_Dir(unremeshed_isoLayer_dir);
//
//	for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
//		QMeshPatch* each_layer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);
//
//		std::string LAYER_dir = unremeshed_isoLayer_dir + std::to_string(each_layer->GetIndexNo())
//			+ "_C" + std::to_string(each_layer->GetIndexNo())
//			+ "_M";
//
//		std::vector<QMeshPatch*>  each_layer_splited;
//		bool if_split = this->_splitSingleSurfacePatch_detect(each_layer);
//
//		if (if_split) {
//			_splitSingleSurfacePatch_splitmesh(each_layer, each_layer_splited);
//			for (int i = 0; i < each_layer_splited.size(); i++) {
//				this->_output_OneSurfaceMesh(each_layer_splited[i], (LAYER_dir + "_" + std::to_string(i)));
//			}
//
//		}
//		else this->_output_OneSurfaceMesh(each_layer, (LAYER_dir + "_0"));
//
//	}
//	std::cout << "Finish output layers into : " << ".. / DataSet / remesh_operation / layers_unremeshed /" << std::endl;
//}
//
//int IsoLayerGeneration::_remove_allFile_in_Dir(std::string dirPath) {
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
//
//void IsoLayerGeneration::_output_OneSurfaceMesh(QMeshPatch* isoSurface, std::string path) {
//
//	double pp[3];
//	path += ".obj";
//	std::ofstream nodeSelection(path);
//
//	int index = 0;
//	for (GLKPOSITION posNode = isoSurface->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
//		QMeshNode* node = (QMeshNode*)isoSurface->GetNodeList().GetNext(posNode);
//		node->GetCoord3D(pp[0], pp[1], pp[2]);
//		nodeSelection << "v " << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
//		index++; node->SetIndexNo(index);
//	}
//	for (GLKPOSITION posFace = isoSurface->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
//		QMeshFace* face = (QMeshFace*)isoSurface->GetFaceList().GetNext(posFace);
//		nodeSelection << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
//			<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
//			<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << std::endl;
//	}
//	nodeSelection.close();
//}

bool IsoLayerGeneration::planeCutSurfaceMesh_delete(QMeshPatch* surfaceMesh) {

	//printf("--- begin cut the mesh by convex hull plane \n\n");

	int cutFaceCase[3][9] = {
		{ 1,4,5,1,2,4,4,3,5 },
		{ 1,4,5,4,2,5,2,3,5 },
		{ 1,4,5,4,2,5,1,5,3 } };

	// Plane equation: Ax+By+Cz+D = 0
	Eigen::Vector3d PlaneDir; double D = 0.0;
	PlaneDir << 0.0, -1.0, 0.0;

	//--------------------------------------------------------------
	/* pre-process to compute node-plane distance*/

	int positiveNodeNum = 0; int negativeNodeNum = 0;

	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		Eigen::Vector3d pp; Node->GetCoord3D(pp(0), pp(1), pp(2));
		Node->nodePlaneDis = pp.dot(PlaneDir) + D;

		if (Node->nodePlaneDis > 0) {
			positiveNodeNum++; Node->SetIndexNo(-1);
		}
		else {
			Node->SetIndexNo(negativeNodeNum); negativeNodeNum++; // node index start from 0
		}
	}

	if (positiveNodeNum == 0) return true; //do not do anything
	else if (negativeNodeNum == 0) {
		//surfaceMesh->ClearAll();
		return false; //delete this layer
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

	surfaceMesh->ClearAll();
	surfaceMesh->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);

	free(nodeTable);
	free(faceTable);

	return true;
	//printf("--- finish cut mesh by plane ---------");
}

void IsoLayerGeneration::generateIsoSurface_support(PolygenMesh* isoSurface, QMeshPatch* patch_supportTet, int layerNum) {

	m_tetMesh_support = patch_supportTet;

	Eigen::VectorXd scalarField_Support(patch_supportTet->GetNodeNumber());

	for (GLKPOSITION Pos = patch_supportTet->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)patch_supportTet->GetNodeList().GetNext(Pos);
		scalarField_Support(Node->GetIndexNo()) = Node->scalarField;
	}

	double minPhi = scalarField_Support.minCoeff();		double maxPhi = scalarField_Support.maxCoeff();

	double gap = 1.0 / layerNum;
	int minLayerNum = (int)((0.5 * gap - minPhi) / gap);		// layer number on the top
	int maxLayerNum = (int)((maxPhi - (1 - 0.5 * gap)) / gap);	// layer number at the bottom
	int allLayerNum = layerNum + minLayerNum + maxLayerNum;

	std::cout << "[min, max] = " << minPhi << "," << maxPhi << std::endl;
	std::cout << "[minLayerNum, maxLayerNum, allLayerNum] = "
		<< minLayerNum << "," << maxLayerNum << "," << allLayerNum << std::endl;

	//multi-layer
	for (int i = 0; i < allLayerNum; i++) {

		double isoCurveValue = (0.5 - minLayerNum + i) * gap;
		QMeshPatch* layer = _generatesingleIsoSurface_support(isoCurveValue, isoSurface);

		if (layer->GetNodeNumber() == 0) {
			std::cout << "this layer have no node!" << std::endl; continue;
		}

		layer->is_SupportLayer = true;
		layer->SetIndexNo(isoSurface->GetMeshList().GetCount()); //index begin from 0
		isoSurface->meshList.AddTail(layer);

		std::cout << layer->GetIndexNo() << " Layer, isoValue = " << isoCurveValue
			<< ", nodeNum = " << layer->GetNodeNumber() << std::endl;
	}
}

QMeshPatch* IsoLayerGeneration::_generatesingleIsoSurface_support(double isoValue, PolygenMesh* isoSurface) {

	QMeshPatch* layer = new QMeshPatch;
	layer->isoSurfaceValue = isoValue;

	//when the node iso-value is equal to surface value, add this eps.
	double eps = 1.0e-5;

	for (GLKPOSITION Pos = m_tetMesh_support->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetMesh_support->GetNodeList().GetNext(Pos);
		if (fabs(Node->scalarField - isoValue) < eps) {
			if (Node->scalarField > isoValue) Node->scalarField = isoValue + eps;
			else Node->scalarField = isoValue - eps;
		}
	}

	// build node list for isoSurface, this comes from the edge list
	for (GLKPOSITION Pos = m_tetMesh_support->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)m_tetMesh_support->GetEdgeList().GetNext(Pos);

		//new code added by tianyu
		/*if ((!Edge->GetStartPoint()->is_tetSupportNode && !Edge->GetStartPoint()->model_boundary)
			|| (!Edge->GetEndPoint()->is_tetSupportNode && !Edge->GetEndPoint()->model_boundary)) continue;*/

		Edge->installedIsoNode = nullptr;
		Edge->isLocateIsoNode = false;

		double a = Edge->GetStartPoint()->scalarField;
		double b = Edge->GetEndPoint()->scalarField;

		if ((isoValue - a) * (isoValue - b) < 0.0) {
			double alpha = (isoValue - a) / (b - a);
			double p1[3], p2[3], pp[3];
			Edge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
			Edge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

			for (int j = 0; j < 3; j++)
				pp[j] = (1.0 - alpha) * p1[j] + alpha * p2[j];

			QMeshNode* isoNode = new QMeshNode;
			isoNode->relatedTetEdge = Edge;
			isoNode->SetMeshPatchPtr(layer);
			isoNode->SetCoord3D(pp[0], pp[1], pp[2]);
			isoNode->SetIndexNo(layer->GetNodeList().GetCount() + 1);
			layer->GetNodeList().AddTail(isoNode);

			Edge->installedIsoNode = isoNode;
			Edge->isLocateIsoNode = true;

		}
	}

	// build edge list for isoSurface, this comes from the face list
	for (GLKPOSITION Pos = m_tetMesh_support->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)m_tetMesh_support->GetFaceList().GetNext(Pos);
		Face->installedIsoEdge = nullptr;
		Face->isLocatedIsoEdge = false;

		int positiveNum = 0; 
		//int innerNum = 0;
		for (int i = 0; i < 3; i++) {
			if (Face->GetNodeRecordPtr(i)->scalarField > isoValue) positiveNum++;
			//new code added by tianyu
			/*if (!Face->GetNodeRecordPtr(i)->is_tetSupportNode && !Face->GetNodeRecordPtr(i)->model_boundary) 
				innerNum++;*/
		}
		//if (innerNum > 0) continue;//new code added by tianyu
		if (positiveNum == 0 || positiveNum == 3) continue;
		else if (positiveNum == 1) {
			QMeshEdge* isoEdge = new QMeshEdge;

			//detect which node is positive
			QMeshNode* PostiveNode;
			int index = 0;
			for (index = 0; index < 3; index++) {
				PostiveNode = Face->GetNodeRecordPtr(index);
				if (PostiveNode->scalarField > isoValue) break;
			}

			QMeshEdge* Edge = Face->GetEdgeRecordPtr(index + 1);
			QMeshNode* startNode = Edge->installedIsoNode;
			isoEdge->SetStartPoint(startNode);

			Edge = Face->GetEdgeRecordPtr((index + 2) % 3 + 1);
			QMeshNode* endNode = Edge->installedIsoNode;
			isoEdge->SetEndPoint(endNode);

			isoEdge->SetMeshPatchPtr(layer);
			isoEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);

			(startNode->GetEdgeList()).AddTail(isoEdge);
			(endNode->GetEdgeList()).AddTail(isoEdge);

			layer->GetEdgeList().AddTail(isoEdge);
			Face->installedIsoEdge = isoEdge;
			Face->isLocatedIsoEdge = true;
		}
		else if (positiveNum == 2) {
			QMeshEdge* isoEdge = new QMeshEdge;
			//detect which node is negative
			QMeshNode* NegativeNode;
			int index = 0;
			for (index = 0; index < 3; index++) {
				NegativeNode = Face->GetNodeRecordPtr(index);
				if (NegativeNode->scalarField < isoValue) break;
			}

			QMeshEdge* Edge = Face->GetEdgeRecordPtr((index + 2) % 3 + 1);
			QMeshNode* startNode = Edge->installedIsoNode;
			isoEdge->SetStartPoint(startNode);

			Edge = Face->GetEdgeRecordPtr(index + 1);
			QMeshNode* endNode = Edge->installedIsoNode;
			isoEdge->SetEndPoint(endNode);

			isoEdge->SetMeshPatchPtr(layer);
			isoEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);

			(startNode->GetEdgeList()).AddTail(isoEdge);
			(endNode->GetEdgeList()).AddTail(isoEdge);

			layer->GetEdgeList().AddTail(isoEdge);
			Face->installedIsoEdge = isoEdge;
			Face->isLocatedIsoEdge = true;
		}
	}

	// build face list for isoSurface, this comes from the face list
	for (GLKPOSITION Pos = m_tetMesh_support->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)m_tetMesh_support->GetTetraList().GetNext(Pos);

		int isoEdgeNum = 0;
		for (int i = 0; i < 4; i++) {
			if (Tetra->GetFaceRecordPtr(i + 1)->isLocatedIsoEdge == true) isoEdgeNum++;
		}
		if (isoEdgeNum == 0) continue;
		else if (isoEdgeNum == 2 || isoEdgeNum == 1) std::cout << "Error! isoEdgeNum cannot equal to 1 or 2!" << std::endl << std::endl;
		else if (isoEdgeNum == 3) {
			QMeshFace* isoFace = new QMeshFace;

			// build the face list
			std::vector<QMeshFace*> FaceList; FaceList.resize(3);
			int faceIndex = 0;
			for (int i = 0; i < 4; i++) {
				if (Tetra->GetFaceRecordPtr(i + 1)->isLocatedIsoEdge == true) {
					FaceList[faceIndex] = Tetra->GetFaceRecordPtr(i + 1);
					faceIndex++;
				}
			}

			//sorting
			bool firstDir = Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])); //detect which one should be the first node!!!
			if (firstDir == true) {
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[2]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[2]->installedIsoEdge->GetEndPoint()) {
					std::swap(FaceList[1], FaceList[2]);
				}
			}
			else {
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[2]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[2]->installedIsoEdge->GetEndPoint()) {
					std::swap(FaceList[1], FaceList[2]);
				}
			}

			//using the first face and add its edge into the isoFace.
			for (int i = 0; i < 3; i++) {
				isoFace->SetEdgeRecordPtr(i, FaceList[i]->installedIsoEdge);
				isoFace->SetDirectionFlag(i, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])));
				if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
					FaceList[i]->installedIsoEdge->SetLeftFace(isoFace);
				else FaceList[i]->installedIsoEdge->SetRightFace(isoFace);
			}

			//push this isoFace back to layer and compute norm
			isoFace->SetEdgeNum(3);
			isoFace->CalPlaneEquation();
			isoFace->SetMeshPatchPtr(layer);
			isoFace->SetIndexNo(layer->GetFaceList().GetCount() + 1);
			layer->GetFaceList().AddTail(isoFace);
		}

		else if (isoEdgeNum == 4) {
			std::vector<QMeshFace*> isoFace; isoFace.resize(2);
			for (int i = 0; i < 2; i++) {
				isoFace[i] = new QMeshFace;
			}
			// build the face list
			std::vector<QMeshFace*> FaceList; FaceList.resize(4);
			int faceIndex = 0;
			for (int i = 0; i < 4; i++) FaceList[i] = Tetra->GetFaceRecordPtr(i + 1);

			bool firstDir = Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])); //detect which one should be the first node!!!
																						//sorting edge
			if (firstDir == true) {
				for (int i = 0; i < 2; i++) {
					if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[i + 2]->installedIsoEdge->GetStartPoint()
						|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[i + 2]->installedIsoEdge->GetEndPoint()) {
						std::swap(FaceList[1], FaceList[i + 2]);
					}
				}
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
				}
				else std::swap(FaceList[2], FaceList[3]);
			}
			else {
				for (int i = 0; i < 2; i++) {
					if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[i + 2]->installedIsoEdge->GetStartPoint()
						|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[i + 2]->installedIsoEdge->GetEndPoint()) {
						std::swap(FaceList[1], FaceList[i + 2]);
					}
				}
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
				}
				else std::swap(FaceList[2], FaceList[3]);

			}
			////test the sorting
			//cout << Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])) << endl;
			//for (int i = 0; i < 4; i++) {
			//	cout << FaceList[i]->installedIsoEdge->GetStartPoint()->GetIndexNo() << " , " <<
			//		FaceList[i]->installedIsoEdge->GetEndPoint()->GetIndexNo() << endl;	
			//}

			QMeshEdge* midEdge1 = new QMeshEdge;
			midEdge1->isMiddleEdge1 = true;
			if (firstDir == true) {
				midEdge1->SetStartPoint(FaceList[0]->installedIsoEdge->GetStartPoint());
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[1]->installedIsoEdge->GetStartPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[1]->installedIsoEdge->GetEndPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 1" << std::endl;
			}
			else {
				midEdge1->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[1]->installedIsoEdge->GetStartPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[1]->installedIsoEdge->GetEndPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 2" << std::endl;
			}

			/*cout << "The midEdge" <<count << "- "<<midEdge1->GetStartPoint()->GetIndexNo() << " , " <<
			midEdge1->GetEndPoint()->GetIndexNo() << endl;
			cout << endl;*/


			QMeshEdge* midEdge2 = new QMeshEdge;
			midEdge2->isMiddleEdge = true;
			if (firstDir == true) {
				midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());

				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 1" << std::endl;
			}
			else {
				midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetStartPoint());

				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 2" << std::endl;
			}

			/*cout << "The midEdge" <<count << "- "<<midEdge2->GetStartPoint()->GetIndexNo() << " , " <<
			midEdge2->GetEndPoint()->GetIndexNo() << endl;
			cout << endl;*/

			//midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());
			/*
			bool dir = false;
			if (FaceList[2]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
			|| FaceList[2]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
			midEdge2->SetEndPoint(FaceList[2]->installedIsoEdge->GetStartPoint());
			dir = true;
			}
			else if(FaceList[2]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
			|| FaceList[2]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
			midEdge2->SetEndPoint(FaceList[2]->installedIsoEdge->GetEndPoint());
			else cout << "Error!" << endl;*/

			QMeshEdge* midEdge;

			if (midEdge1->CalLength() <= midEdge2->CalLength()) {

				midEdge = midEdge1;
				isoFace[0]->SetEdgeRecordPtr(0, FaceList[0]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])));
				isoFace[0]->SetEdgeRecordPtr(1, FaceList[1]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[1])));
				isoFace[0]->SetEdgeRecordPtr(2, midEdge);
				isoFace[0]->SetDirectionFlag(2, false);

				for (int i = 0; i < 4; i++) {
					if (i == 0 || i == 1) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetRightFace(isoFace[0]);

				isoFace[1]->SetEdgeRecordPtr(0, FaceList[2]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[2])));
				isoFace[1]->SetEdgeRecordPtr(1, FaceList[3]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[3])));
				isoFace[1]->SetEdgeRecordPtr(2, midEdge);
				isoFace[1]->SetDirectionFlag(2, true);
				for (int i = 0; i < 4; i++) {
					if (i == 2 || i == 3) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetLeftFace(isoFace[1]);
			}

			else {
				midEdge = midEdge2;
				//first triangle
				isoFace[0]->SetEdgeRecordPtr(0, FaceList[0]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])));
				isoFace[0]->SetEdgeRecordPtr(1, midEdge);
				isoFace[0]->SetDirectionFlag(1, true);
				isoFace[0]->SetEdgeRecordPtr(2, FaceList[3]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(2, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[3])));
				//set edge right / left face
				for (int i = 0; i < 4; i++) {
					if (i == 0 || i == 3) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetLeftFace(isoFace[0]);

				//first triangle
				isoFace[1]->SetEdgeRecordPtr(0, FaceList[1]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[1])));
				isoFace[1]->SetEdgeRecordPtr(1, FaceList[2]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[2])));
				isoFace[1]->SetEdgeRecordPtr(2, midEdge);
				isoFace[1]->SetDirectionFlag(2, false);
				//set edge right / left face
				for (int i = 0; i < 4; i++) {
					if (i == 1 || i == 2) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[1]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[1]);
					}
				}
				midEdge->SetRightFace(isoFace[1]);
			}

			//push back midEdge
			midEdge->SetMeshPatchPtr(layer);
			midEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			layer->GetEdgeList().AddTail(midEdge);


			/*midEdge1->SetMeshPatchPtr(layer);
			midEdge1->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			layer->GetEdgeList().AddTail(midEdge1);*/

			//midEdge2->SetMeshPatchPtr(layer);
			//midEdge2->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			//layer->GetEdgeList().AddTail(midEdge2);



			//push this isoFace back to layer and compute norm
			for (int i = 0; i < 2; i++) {
				isoFace[i]->SetEdgeNum(3);
				isoFace[i]->CalPlaneEquation();
				isoFace[i]->SetMeshPatchPtr(layer);
				isoFace[i]->SetIndexNo(layer->GetFaceList().GetCount() + 1);
				layer->GetFaceList().AddTail(isoFace[i]);
			}
		}
	}

	//give each node face list
	for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
		for (int i = 0; i < 3; i++) {
			QMeshNode* Node = Face->GetNodeRecordPtr(i);
			Node->GetFaceList().AddTail(Face);
		}
	}

	return layer;
}

/* This function is used to generate isosurface set that guanatee the layer hight is within [min, max] (input parameter) */
// The layer number is not controlled here.
void IsoLayerGeneration::generateIsoSurface_adaptiveDistanceControl(
	PolygenMesh* isoSurface, double minLayerHight, double maxLayerHight) {

	QMeshPatch* layer;  // current layer
	QMeshPatch* bLayer; // bottom layer 
	double isoCurveValue;

	/* first Layer */
	double initLayerISOValue = 0.01; // initial layer iso value
	isoCurveValue = initLayerISOValue;
	layer = _generatesingleIsoSurface(isoCurveValue, isoSurface);
	this->_insertLayerToSurfaceSet_smooth(layer, 0, isoSurface, NULL, false);
	bLayer = layer;

	/* iteration for the rest layer */
	double thisLayerISOValue;
	int iter = 0;
	do
	{
		// first make sure the minimum layer hight
		thisLayerISOValue = this->_detectNextLayerISOValue(bLayer, minLayerHight);

		QMeshPatch* layer = _generatesingleIsoSurface(thisLayerISOValue, isoSurface); // build next layer
		this->_insertLayerToSurfaceSet_smooth(layer, 0, isoSurface, bLayer, false); // always smooth layer before check the layer hight - avoid numerical error

		/* terminal condition - special process is taken for the final layer */
		//if (1.0 - thisLayerISOValue < 0.01){ // this may not always work!
		double theta = 0.05; // parameter to tune! ////////////////////////
		if (this->_checktwoLayerThickness_withinRange(maxLayerHight, minLayerHight - theta, bLayer, layer, true) == false) {
			isoSurface->GetMeshList().Remove(bLayer);
			bLayer->ClearAll(); std::cout << " --- remove the last layer to ensure the protection of the final layer" << std::endl;
			break;
		}

		bLayer = layer; iter++; //update bLayer
	} while (true);

	std::cout << " #################### finish build initial layer set ####################\n" << std::endl;
	//return; //1st break;
	/* adaptive add layer and delete small distance region */
	int iterTime = 0;
	do {
		//if (iterTime > 21) break;
		bool breakIter = false;
		for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshPatch* layer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);

			if (layer->thicknessChecked) {
				std::cout << "\n ----- iter " << iterTime << ", Layer satisfied with the requiement! isoValue = "
					<< layer->isoSurfaceValue << std::endl << std::endl;
				continue;
			}
			else {
				if (layer->GetIndexNo() == isoSurface->GetMeshList().GetCount() - 1) {
					breakIter = true; break;
				}
				bLayer = layer;
				QMeshPatch* tLayer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);
				std::cout << "#### insert layer between [" << bLayer->isoSurfaceValue << "," << tLayer->isoSurfaceValue << "]" << std::endl;
				if (this->_checktwoLayerThickness_withinRange(maxLayerHight, minLayerHight, bLayer, tLayer, false) == true) {
					std::cout << "\n ----- iter " << iterTime << ", Layer satisfied with the requiement! isoValue = " 
						<< layer->isoSurfaceValue << std::endl << std::endl;
					layer->thicknessChecked = true;
				}
				else {
					//add the checking of the thickness control here!!! only the maximum layer hight!!!
					QMeshPatch* layer = _generatesingleIsoSurface((bLayer->isoSurfaceValue + tLayer->isoSurfaceValue) / 2, isoSurface); // build next layer
					this->_insertLayerToSurfaceSet_smooth(layer, 0, isoSurface, bLayer, true); // insert after bLayer

					this->_cutLayer_withOffsetLayer(bLayer, layer, minLayerHight);
					/*this->_cutLayer_insideDetection(bLayer, layer);
					triming_workingSurface(layer);*/

					std::cout << "\n ----- iter " << iterTime << ", add new layer! isoValue = " 
						<< layer->isoSurfaceValue << std::endl << std::endl;

					iterTime++;
				}
				break;
			}
		}
		if (breakIter) break;
	} while (true);

	//return; //2nd break;

	/* triming the working surface to ensure minimum layer hight */
	for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* layer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);
		//update the compatible layer value for output
		layer->compatible_layer_Index = layer->GetIndexNo();
		this->_triming_workingSurface(layer);
	}
}

void IsoLayerGeneration::_insertLayerToSurfaceSet_smooth(
	QMeshPatch* layer, int smmothItertime, PolygenMesh* isoSurface, QMeshPatch* blayer, bool insert) {

	for (int smoothIter = 0; smoothIter < smmothItertime; smoothIter++) this->_smooth_one_IsoSurface(layer);
	if (insert) {
		GLKObNode* pos = isoSurface->meshList.Find(blayer);
		isoSurface->meshList.InsertAfter(pos, layer);
		/* update layer index */
		int index = 0;
		for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshPatch* layer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);
			layer->SetIndexNo(index); index++;
		}
		std::cout << layer->GetIndexNo() << " Layer (INSERT), isoValue = " << layer->isoSurfaceValue 
			<< ", nodeNum = " << layer->GetNodeNumber() << std::endl;
	}
	else {
		isoSurface->meshList.AddTail(layer);
		layer->SetIndexNo(isoSurface->GetMeshList().GetCount()); //index begin from 0
		std::cout << layer->GetIndexNo() << " Layer, isoValue = " << layer->isoSurfaceValue
			<< ", nodeNum = " << layer->GetNodeNumber() << std::endl;
	}
}

void IsoLayerGeneration::_smooth_one_IsoSurface(QMeshPatch* isoLayer) {


	for (GLKPOSITION Pos = isoLayer->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* thisEdge = (QMeshEdge*)isoLayer->GetEdgeList().GetNext(Pos);
		if (thisEdge->IsBoundaryEdge()) {
			thisEdge->GetStartPoint()->isoSurfaceBoundary = true;
			thisEdge->GetEndPoint()->isoSurfaceBoundary = true;
		}
	}

	//laplacian smoothness
	for (GLKPOSITION Pos = isoLayer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)isoLayer->GetNodeList().GetNext(Pos);

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

double IsoLayerGeneration::_detectNextLayerISOValue(QMeshPatch* bLayer, double minLayerHight) {

	double bottomISOValue = bLayer->isoSurfaceValue;
	double higherBound_isoValue = 0.99;
	double lowerBound_isoValue = bottomISOValue + 0.001;

	/* build PQP model */
	Eigen::MatrixXd nodePos; Eigen::VectorXd layerDis;
	PQP_Model* pqpModel = new PQP_Model();

	pqpModel->BeginModel();  int index = 0;
	PQP_REAL p1[3], p2[3], p3[3];

	for (GLKPOSITION Pos = bLayer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)bLayer->GetFaceList().GetNext(Pos);

		Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
		Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
		Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);

		pqpModel->AddTri(p1, p2, p3, index);
		index++;
	}
	pqpModel->EndModel();

	do
	{
		double isoValue = 0.5 * (higherBound_isoValue + lowerBound_isoValue);

		/* build initial face node - without check if the node is outside the Blayer region */
		this->_generateISOSurfaceNode(isoValue, nodePos);
		this->_checkLayerDistance(bLayer, nodePos, layerDis, pqpModel);

		double layerDisMax = layerDis.maxCoeff();
		double layerDisMin = layerDis.minCoeff();

		// control minimum layer thickness 
		if (layerDisMin < minLayerHight) lowerBound_isoValue = isoValue;
		else higherBound_isoValue = isoValue;

		std::cout << " ---- LayerHight range = [" << layerDisMax << "," << layerDisMin << "]. this Iteration isovalue max = "
			<< higherBound_isoValue << ", min = " << lowerBound_isoValue << std::endl;
		
	} while (fabs(higherBound_isoValue - lowerBound_isoValue) > 0.0001);

	delete pqpModel;
	return 0.5 * (higherBound_isoValue + lowerBound_isoValue);
}

void IsoLayerGeneration::_generateISOSurfaceNode(double isoValue, Eigen::MatrixXd& nodePos) {

	Eigen::MatrixXd thisLayerNodePos(m_tetMesh->GetEdgeNumber(), 3);
	int isoNodeIndex = 0;
	for (GLKPOSITION Pos = m_tetMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)m_tetMesh->GetEdgeList().GetNext(Pos);

		double a = Edge->GetStartPoint()->scalarField;
		double b = Edge->GetEndPoint()->scalarField;

		if ((isoValue - a) * (isoValue - b) < 0.0) {
			double alpha = (isoValue - a) / (b - a);
			double p1[3], p2[3];
			Edge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
			Edge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

			for (int j = 0; j < 3; j++) 
				thisLayerNodePos(isoNodeIndex, j) = (1.0 - alpha) * p1[j] + alpha * p2[j];

			isoNodeIndex++;
		}
	}

	nodePos.resize(isoNodeIndex, 3);
	for (int i = 0; i < isoNodeIndex; i++) {
		for (int j = 0; j < 3; j++) nodePos(i, j) = thisLayerNodePos(i, j);
	}
}

void IsoLayerGeneration::_checkLayerDistance(QMeshPatch* bLayer, Eigen::MatrixXd& nodePos,
	Eigen::VectorXd& layerDis, PQP_Model* pqpModel) {

	layerDis = Eigen::VectorXd::Zero(nodePos.rows());
	/* compute distance for each layer and get result */
	for (int index = 0; index < nodePos.rows(); index++) {
		PQP_DistanceResult dres; dres.last_tri = pqpModel->last_tri;
		PQP_REAL p[3]; for (int i = 0; i < 3; i++) p[i] = nodePos(index, i);

		PQP_Distance(&dres, pqpModel, p, 0.0, 0.0);
		float minDist = dres.Distance();
		layerDis(index) = minDist; //save the distance to node
	}
	//std::cout << layerDis << std::endl;
}

bool IsoLayerGeneration::_checktwoLayerThickness_withinRange(double maxLayerHight, double minLayerHight,
	QMeshPatch* bLayer, QMeshPatch* tLayer, bool minControl) {

	int index = 0;
	/* build PQP model */
	PQP_Model* pqpModel = new PQP_Model();
	pqpModel->BeginModel();  PQP_REAL p1[3], p2[3], p3[3];
	for (GLKPOSITION Pos = bLayer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)bLayer->GetFaceList().GetNext(Pos);
		Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]); Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
		Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);
		pqpModel->AddTri(p1, p2, p3, index); index++;
	}
	pqpModel->EndModel();

	/* first check distance */
	Eigen::VectorXd layerDis;
	Eigen::MatrixXd nodePos(tLayer->GetNodeNumber(), 3); index = 0;
	for (GLKPOSITION pos = tLayer->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)tLayer->GetNodeList().GetNext(pos);
		node->GetCoord3D(nodePos(index, 0), nodePos(index, 1), nodePos(index, 2)); node->SetIndexNo(index); index++;
	}

	this->_checkLayerDistance(bLayer, nodePos, layerDis, pqpModel);

	double layerDisMax = layerDis.maxCoeff();
	double layerDisMin = layerDis.minCoeff();

	Eigen::VectorXi deleteNodeIndexSet;
	this->_checkNodeSet_withinBlayerRegion(bLayer, nodePos, layerDisMax, deleteNodeIndexSet);

	/* update distance again */
	this->_checkLayerDistance(bLayer, nodePos, layerDis, pqpModel);
	layerDisMax = layerDis.maxCoeff();
	layerDisMin = layerDis.minCoeff();

	// update for visulization
	for (GLKPOSITION pos = tLayer->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)tLayer->GetNodeList().GetNext(pos);
		if (node->GetIndexNo() == deleteNodeIndexSet(index)) {
			node->isBlayerOffsetCoverAera = false; index++;
		}
	}

	std::cout << " --- checked two layer max distance = " << layerDisMax 
		<< ", min distance = " << layerDisMin << std::endl; // update layerdisMax

	delete pqpModel;

	if (minControl) {
		/* control the minimum hight */
		if (layerDisMin > minLayerHight) return true;
		else {
			//std::cout << layerDisMin << "!! terminal condition detected !!" << endl;
			return false;
		}
	}
	else {
		/* control the maximum hight */
		if (layerDisMax < maxLayerHight) return true;
		else return false;
	}
}

void IsoLayerGeneration::_checkNodeSet_withinBlayerRegion(QMeshPatch* bLayer, 
	Eigen::MatrixXd& nodePosSet, double maxLayerHight, Eigen::VectorXi& deleteNodeIndexSet){
	/* build the face list for bLayer - offset with maxLayerHight, form a prism mesh region */
	Eigen::MatrixXd nodeTable(bLayer->GetNodeNumber() * 2, 3);
	int boundaryEdgeNum = 0;
	for (GLKPOSITION Pos = bLayer->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)bLayer->GetEdgeList().GetNext(Pos);
		if (Edge->GetLeftFace() == NULL || Edge->GetRightFace() == NULL) boundaryEdgeNum++; /* boundary edge */
	}
	Eigen::MatrixXi faceTable(bLayer->GetFaceNumber() * 2 + boundaryEdgeNum * 2, 3);

	/* build table */
	this->_buildOffsetMeshTable(bLayer, nodeTable, faceTable, boundaryEdgeNum, maxLayerHight);

	// check if the node inside the offset space
	std::vector<Eigen::Vector3d> newNodeSetPos;
	deleteNodeIndexSet = Eigen::VectorXi::Zero(nodePosSet.rows()); int deleteNodeNum = 0;
	for (int i = 0; i < nodePosSet.rows(); i++) {
		Eigen::Vector3d checkNodePos = nodePosSet.row(i);
		if (this->_checkNodeInsideOffsetSpace(nodeTable, faceTable, checkNodePos) == true) {
			newNodeSetPos.push_back(checkNodePos);
		}
		else {
			deleteNodeIndexSet(deleteNodeNum) = i;
			deleteNodeNum++;
		}
	}
	std::cout << "before checking, node Num = " << nodePosSet.rows() 
		<< ", after checking left node number = " << newNodeSetPos.size() << std::endl;
	nodePosSet.resize(newNodeSetPos.size(), 3);

	for (int i = 0; i < newNodeSetPos.size(); i++) {
		for (int j = 0; j < 3; j++) nodePosSet(i, j) = newNodeSetPos[i](j);
	}
}

void IsoLayerGeneration::_buildOffsetMeshTable(QMeshPatch* bLayer, Eigen::MatrixXd& nodeTable,
	Eigen::MatrixXi& faceTable, double boundaryEdgeNum, double maxLayerHight) {
	int index = 0;
	for (GLKPOSITION Pos = bLayer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)bLayer->GetNodeList().GetNext(Pos);
		Node->SetIndexNo(index);
		Node->GetCoord3D(nodeTable(index, 0), nodeTable(index, 1), nodeTable(index, 2));

		double normal[3];
		Node->CalNormal(); Node->GetNormal(normal[0], normal[1], normal[2]);
		for (int i = 0; i < 3; i++)
			nodeTable(index + bLayer->GetNodeNumber(), i) = nodeTable(index, i) - (maxLayerHight + 0.1) * normal[i];
		index++;
	}

	int faceIndex = 0;
	/* top and bottom layer face */
	for (GLKPOSITION pos = bLayer->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)bLayer->GetFaceList().GetNext(pos);
		for (int i = 0; i < 3; i++) {
			faceTable(faceIndex, i) = face->GetNodeRecordPtr(i)->GetIndexNo(); // initial layer
			faceTable(faceIndex + 1, i) = face->GetNodeRecordPtr(i)->GetIndexNo() + bLayer->GetNodeNumber(); // offset layer
		}
		faceIndex += 2;
	}
	for (GLKPOSITION Pos = bLayer->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)bLayer->GetEdgeList().GetNext(Pos);
		if (Edge->GetLeftFace() == NULL || Edge->GetRightFace() == NULL) {

			int sIndex = Edge->GetStartPoint()->GetIndexNo();
			int eIndex = Edge->GetEndPoint()->GetIndexNo();
			Eigen::Vector3i face1; face1 << sIndex, sIndex + bLayer->GetNodeNumber(), eIndex;
			Eigen::Vector3i face2; face2 << eIndex, sIndex + bLayer->GetNodeNumber(), eIndex + bLayer->GetNodeNumber();

			faceTable.row(faceIndex) = face1; faceTable.row(faceIndex + 1) = face2; faceIndex += 2;
		}
	}
}

bool IsoLayerGeneration::_checkNodeInsideOffsetSpace(Eigen::MatrixXd& nodeTable, 
	Eigen::MatrixXi& faceTable, Eigen::Vector3d& checkNodePos) {

	// ray tracking dir
	Eigen::Vector3d dir = { 1.0,0.0,0.0 };
	int intersection_Time = 0;

#pragma omp parallel   
	{
#pragma omp for  
		for (int i = 0; i < faceTable.rows(); i++) {
			Eigen::Vector3d v0 = nodeTable.row(faceTable(i, 0));
			Eigen::Vector3d v1 = nodeTable.row(faceTable(i, 1));
			Eigen::Vector3d v2 = nodeTable.row(faceTable(i, 2));

			if (this->IntersectTriangle(checkNodePos, dir, v0, v1, v2))
				intersection_Time++;
		}
	}

	//std::cout << "intersection Num " << intersection_Time << std::endl;
	if (intersection_Time % 2 != 0) {
		//std::cout << "in the mesh" << std::endl;
		return true;
	}
	else return false;
	//std::cout << "be out of mesh" << std::endl;
}

bool IsoLayerGeneration::IntersectTriangle(const Eigen::Vector3d& orig, const Eigen::Vector3d& dir,
	Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2){

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

void IsoLayerGeneration::_cutLayer_withOffsetLayer(QMeshPatch* bLayer, QMeshPatch* layer, double offsetValue){

	/* first update bLayer with offset distance */
	for (GLKPOSITION Pos = bLayer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)bLayer->GetNodeList().GetNext(Pos);
		double nodePos[3], normal[3];
		Node->GetCoord3D(nodePos[0], nodePos[1], nodePos[2]);
		Node->SetCoord3D_last(nodePos[0], nodePos[1], nodePos[2]);
		Node->CalNormal(); Node->GetNormal(normal[0], normal[1], normal[2]);
		for (int i = 0; i < 3; i++) nodePos[i] -= offsetValue * normal[i];
		Node->SetCoord3D(nodePos[0], nodePos[1], nodePos[2]);
	}

	/* compute PQP distance */

	// build PQP model for bottom layer
	PQP_Model* pqpModel = new PQP_Model();
	pqpModel->BeginModel();  int index = 0;
	PQP_REAL p1[3], p2[3], p3[3];

	for (GLKPOSITION Pos = bLayer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)bLayer->GetFaceList().GetNext(Pos);

		Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
		Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
		Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);

		pqpModel->AddTri(p1, p2, p3, index);
		index++;

	}
	pqpModel->EndModel();

	// compute minimal distance for every node at the cutting layer
	double pp[3];
	for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);

		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		PQP_DistanceResult dres; dres.last_tri = pqpModel->last_tri;
		PQP_REAL p[3];
		p[0] = pp[0]; p[1] = pp[1]; p[2] = pp[2];
		PQP_Distance(&dres, pqpModel, p, 0.0, 0.0);

		int minTriId = dres.last_tri->id;	//	closest triangle - note that ID index starts from zero

		QMeshFace* cloestFace;
		int faceIndex = 0;
		for (GLKPOSITION Pos = bLayer->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)bLayer->GetFaceList().GetNext(Pos);
			if (faceIndex == minTriId) {
				cloestFace = Face;
				break;
			}
			faceIndex++;
		}

		Eigen::Vector3d faceNormal; double d;
		cloestFace->CalPlaneEquation();
		cloestFace->GetPlaneEquation(faceNormal[0], faceNormal[1], faceNormal[2], d);

		Eigen::Vector3d nodePos, cloestPos;
		Node->GetCoord3D(nodePos[0], nodePos[1], nodePos[2]);

		if ((nodePos.dot(faceNormal) + d) > 0.001) Node->layerThickDistance = dres.Distance();
		else {
			Node->layerThickDistance = -dres.Distance();
			//std::cout << "below the layer!" << endl;
		}
		//std::cout << "distance field = " << Node->layerThickDistance << endl;
	}

	delete pqpModel;

	for (GLKPOSITION Pos = bLayer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)bLayer->GetNodeList().GetNext(Pos);
		double nodePos[3];
		Node->GetCoord3D_last(nodePos[0], nodePos[1], nodePos[2]);
		Node->SetCoord3D(nodePos[0], nodePos[1], nodePos[2]);
	}
}

void IsoLayerGeneration::_triming_workingSurface(QMeshPatch* isoLayer) {

	double cut_isoValue = 0;

	/* get the VERTEX NUM */
	int structuredMesh_NodeNum = 0;
	//// node loop
	for (GLKPOSITION Pos = isoLayer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)isoLayer->GetNodeList().GetNext(Pos);

		Node->layerThickDistance = -Node->layerThickDistance;
		Node->cutNode_index = -1;//reset 

		if (Node->layerThickDistance > 0) {

			Node->cutNode_index = structuredMesh_NodeNum; // start from 0
			structuredMesh_NodeNum++;
		}
	}
	//// edge loop
	for (GLKPOSITION Pos = isoLayer->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)isoLayer->GetEdgeList().GetNext(Pos);

		double a = Edge->GetStartPoint()->layerThickDistance;
		double b = Edge->GetEndPoint()->layerThickDistance;

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
			cut_Node->layerThickDistance = cut_isoValue;

			cut_Node->SetCoord3D(pp[0], pp[1], pp[2]);
			//cutNode_index should increase based on the "structuredMesh_NodeNum"
			cut_Node->cutNode_index = structuredMesh_NodeNum;
			structuredMesh_NodeNum++;

			//install this cutNode to its Edge
			Edge->installed_CutNode = cut_Node;
			Edge->isLocate_CutNode_layerEdge = true;
		}
	}

	/* get the VERTEX Table of tight support Layer */
	if (structuredMesh_NodeNum == 0) return;// avoid the meanless compute
	Eigen::MatrixXd V = Eigen::MatrixXd::Zero(structuredMesh_NodeNum, 3);
	//// node loop
	for (GLKPOSITION Pos = isoLayer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)isoLayer->GetNodeList().GetNext(Pos);

		if (Node->cutNode_index >= 0) { // VERTEX needed to be kept

			double xx, yy, zz;
			Node->GetCoord3D(xx, yy, zz);
			V.row(Node->cutNode_index) << xx, yy, zz;
		}
	}
	//// edge loop
	for (GLKPOSITION Pos = isoLayer->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)isoLayer->GetEdgeList().GetNext(Pos);

		if (Edge->isLocate_CutNode_layerEdge) {

			double xx, yy, zz;
			Edge->installed_CutNode->GetCoord3D(xx, yy, zz);
			V.row(Edge->installed_CutNode->cutNode_index) << xx, yy, zz;
		}
	}

	/* get the FACE NUM of tight support Layer */
	int structuredMesh_FaceNum = 0;
	//// face loop
	for (GLKPOSITION Pos = isoLayer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)isoLayer->GetFaceList().GetNext(Pos);

		int positive_Num = 0;
		for (int k = 0; k < 3; k++) {
			if (Face->GetNodeRecordPtr(k)->layerThickDistance > 0) {
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
	}
	/*END: get the FACE NUM of working surface*/

	/* get the FACE Table of tight support Layer */
	Eigen::MatrixXi F = Eigen::MatrixXi::Zero(structuredMesh_FaceNum, 3);
	// face loop
	// collect the 3+ faces (the implicit_cut_values of 3 vertexes of one face are all large than 0)
	int F_row_index = 0;
	for (GLKPOSITION Pos = isoLayer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)isoLayer->GetFaceList().GetNext(Pos);

		if (Face->faceType == KEEP) {

			for (int k = 0; k < 3; k++) {
				F(F_row_index, k) = Face->GetNodeRecordPtr(k)->cutNode_index;
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
			F(F_row_index, 0) = Face->GetNodeRecordPtr(baseIndex)->cutNode_index;
			F(F_row_index, 1) = Face->GetNodeRecordPtr((baseIndex + 1) % 3)->cutNode_index;
			F(F_row_index, 2) = Face->GetEdgeRecordPtr((baseIndex + 1) % 3 + 1)->installed_CutNode->cutNode_index;
			F_row_index++;

			// the 2nd added face
			F(F_row_index, 0) = Face->GetNodeRecordPtr(baseIndex)->cutNode_index;
			F(F_row_index, 1) = Face->GetEdgeRecordPtr((baseIndex + 1) % 3 + 1)->installed_CutNode->cutNode_index;
			F(F_row_index, 2) = Face->GetEdgeRecordPtr((baseIndex + 2) % 3 + 1)->installed_CutNode->cutNode_index;
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

			F(F_row_index, 0) = Face->GetNodeRecordPtr((baseIndex + 2) % 3)->cutNode_index;
			F(F_row_index, 1) = Face->GetEdgeRecordPtr((baseIndex + 2) % 3 + 1)->installed_CutNode->cutNode_index;
			F(F_row_index, 2) = Face->GetEdgeRecordPtr((baseIndex + 1) % 3 + 1)->installed_CutNode->cutNode_index;
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

	isoLayer->ClearAll();
	isoLayer->constructionFromVerFaceTable(V.rows(), nodeTable, F.rows(), faceTable);
	free(nodeTable);
	free(faceTable);
}