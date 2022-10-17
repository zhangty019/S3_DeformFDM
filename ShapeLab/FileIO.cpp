#pragma once
#include <fstream>

#include "io.h"
#include "FileIO.h"
#include "../GLKLib/GLKGeometry.h"

//output the boundary surface mash of volume model
void FileIO::changeTet2Surface(PolygenMesh* io_Source, std::string path) {

	QMeshPatch* tetPatch = (QMeshPatch*)io_Source->GetMeshList().GetHead();

	double pp[3];
	std::string tetPath = path + tetPatch->patchName + ".obj";
	std::ofstream str(tetPath);

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
			<< " " << face->GetNodeRecordPtr(2)->Volume2Surface_Ind
			<< " " << face->GetNodeRecordPtr(1)->Volume2Surface_Ind << std::endl;
	}
	str.close();

	//also output the color of boundary face of the scaled tet.
	index = 0;
	std::string FacePath = path + tetPatch->patchName + "_scaleValue_color.txt";
	std::ofstream color_Svalue(FacePath);
	for (GLKPOSITION posFace = tetPatch->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(posFace);
		if (face->inner) continue;
		float rr, gg, bb;
		face->GetColor(rr, gg, bb);
		color_Svalue << index << " " << rr << " " << gg << " " << bb << std::endl;
		index++;
	}
	color_Svalue.close();
}

//output the curved layers with splited ones OR not
void FileIO::outputIsoSurface(PolygenMesh* isoSurface, bool isSplit) {

	std::string unremeshed_isoLayer_dir = "../DataSet/remesh_operation/layers_unremeshed/";
	this->_remove_allFile_in_Dir(unremeshed_isoLayer_dir);

	int layer_index = 0; std::string LAYER_dir;

	for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* each_layer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);

		std::vector<QMeshPatch*>  each_layer_splited;
		bool is_multiRegion = this->_splitSingleSurfacePatch_detect(each_layer);
		if (is_multiRegion && isSplit) {
			this->_splitSingleSurfacePatch_splitmesh(each_layer, each_layer_splited);
			for (int i = 0; i < each_layer_splited.size(); i++) {

				LAYER_dir = unremeshed_isoLayer_dir + std::to_string(layer_index)
					+ "_C" + std::to_string(each_layer->compatible_layer_Index);

				if (each_layer->is_SupportLayer) LAYER_dir += "_S";
				else  LAYER_dir += "_M";
				this->_output_OneSurfaceMesh(each_layer_splited[i], (LAYER_dir + "_" + std::to_string(i)));
				layer_index++;
			}
		}
		else { 
			LAYER_dir = unremeshed_isoLayer_dir + std::to_string(layer_index)
				+ "_C" + std::to_string(each_layer->compatible_layer_Index);

			if (each_layer->is_SupportLayer) LAYER_dir += "_S";
			else  LAYER_dir += "_M";

			this->_output_OneSurfaceMesh(each_layer, (LAYER_dir + "_0"));
			layer_index++;
		}
	}
	std::cout << "Finish output layers into : " << ".. / DataSet / remesh_operation / layers_unremeshed /" << std::endl;
}

bool FileIO::_splitSingleSurfacePatch_detect(QMeshPatch* each_layer) {
	//Function for detecting if single path have more than one closed region - by flooding method//

	/*---------------------------
	Detect if given a closed mesh
	---------------------------*/

	bool closedSurface = true;
	for (GLKPOSITION Pos = each_layer->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* edge = (QMeshEdge*)each_layer->GetEdgeList().GetNext(Pos);
		if (edge->GetLeftFace() == NULL || edge->GetRightFace() == NULL) { closedSurface = false; break; }
	}
	if (closedSurface) return false;

	/*-------------------------------
	give node split index by flooding
	-------------------------------*/
	bool allNodeChecked = true;
	int splitTime = 0;

	for (GLKPOSITION Pos = each_layer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)each_layer->GetNodeList().GetNext(Pos);
		thisNode->splitIndex = -1;
	}

	do {
		splitTime++;

		//-- Find start node
		for (GLKPOSITION Pos = each_layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)each_layer->GetNodeList().GetNext(Pos);
			if (thisNode->splitIndex < 0) { thisNode->splitIndex = splitTime; break; }
		}

		//-- Run flooding searching
		this->_splitSingleSurfacePatch_flooding(each_layer, splitTime);

		//-- Detect if all the node has been checked
		for (GLKPOSITION Pos = each_layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)each_layer->GetNodeList().GetNext(Pos);
			allNodeChecked = true;
			if (thisNode->splitIndex < 0) { allNodeChecked = false; break; }
		}

	} while (!allNodeChecked);

	if (splitTime == 1) { std::cout << "This isoSurface contains single region" << std::endl; return false; }
	else {

		/*----------------------------------------------------------------------------------
		if contains more than one region, save them back to the isoSurfaceSet_splited vector
		----------------------------------------------------------------------------------*/
		each_layer->splitIndex = splitTime;
		std::cout << "This isoSurface contains " << splitTime << " regions." << std::endl;
		return true;
	}
}

void FileIO::_splitSingleSurfacePatch_flooding(QMeshPatch* isoSurface, int index) {
	int StopFlag = 0; 			QMeshNode* NeighboorNode;

	for (GLKPOSITION Pos = isoSurface->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)isoSurface->GetNodeList().GetNext(Pos);

		if (thisNode->splitIndex == index) {
			for (int i = 0; i < thisNode->GetEdgeNumber(); i++) {

				QMeshEdge* thisEdge = thisNode->GetEdgeRecordPtr(i + 1);
				if (thisNode == thisEdge->GetStartPoint()) NeighboorNode = thisEdge->GetEndPoint();
				else NeighboorNode = thisEdge->GetStartPoint();

				if (NeighboorNode->splitIndex == index) continue;
				else {
					NeighboorNode->splitIndex = index;
					StopFlag++;
				}
			}
		}
	}
	//cout << "This iteration adding node number = " << StopFlag << endl;

	if (StopFlag > 0) _splitSingleSurfacePatch_flooding(isoSurface, index);
}

void FileIO::_splitSingleSurfacePatch_splitmesh(
	QMeshPatch* isoSurface, std::vector<QMeshPatch*>& isoSurfaceSet_splited) {
	for (int iter = 1; iter < isoSurface->splitIndex + 1; iter++) {

		QMeshPatch* patch = new QMeshPatch;
		patch->splitIndex = iter; patch->SetIndexNo(isoSurface->GetIndexNo());

		//build nodeNum and nodeTable
		float* nodeTable; int nodeNum = 0; 	double pp[3];

		for (GLKPOSITION Pos = isoSurface->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)isoSurface->GetNodeList().GetNext(Pos);
			if (thisNode->splitIndex == iter) nodeNum++;
		}
		nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);

		int index = 0;
		for (GLKPOSITION Pos = isoSurface->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)isoSurface->GetNodeList().GetNext(Pos);
			if (thisNode->splitIndex == iter) {
				thisNode->GetCoord3D(pp[0], pp[1], pp[2]);
				for (int i = 0; i < 3; i++) nodeTable[index * 3 + i] = (float)pp[i];
				thisNode->splitPatchIndex = index; //VolumetoSurfaceIndex start from 0
				index++;
			}
		}

		//build faceNum and faceTable
		unsigned int* faceTable; int faceNum = 0;
		for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
			for (int i = 0; i < 3; i++) {
				if (face->GetNodeRecordPtr(i)->splitIndex == iter) {
					face->splitIndex = iter; faceNum++; break;
				}
			}
		}
		faceTable = (unsigned int*)malloc(sizeof(unsigned int) * faceNum * 3);
		index = 0;
		for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
			if (face->splitIndex == iter) {
				for (int i = 0; i < 3; i++)
					faceTable[index * 3 + i] = face->GetNodeRecordPtr(i)->splitPatchIndex;
				index++;
			}
		}

		patch->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);
		isoSurfaceSet_splited.push_back(patch);
	}
}

int FileIO::_remove_allFile_in_Dir(std::string dirPath) {

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

void FileIO::_output_OneSurfaceMesh(QMeshPatch* eachLayer, std::string path) {

	double pp[3];
	path += ".obj";
	std::ofstream nodeSelection(path);

	int index = 0;
	for (GLKPOSITION posNode = eachLayer->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)eachLayer->GetNodeList().GetNext(posNode);
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		nodeSelection << "v " << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
		index++; node->SetIndexNo(index);
		
	}
	for (GLKPOSITION posFace = eachLayer->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)eachLayer->GetFaceList().GetNext(posFace);
		nodeSelection << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << std::endl;
	}
	nodeSelection.close();
}

void FileIO::output_TetMesh(QMeshPatch* tetMesh, std::string path) {

	double pp[3];
	path = path + tetMesh->patchName + "_deformed.tet";

	std::ofstream tet_output(path);
	tet_output << tetMesh->GetNodeNumber() << " vertices" << std::endl;
	tet_output << tetMesh->GetTetraNumber() << " tets" << std::endl;

	int index = 0;
	for (GLKPOSITION posNode = tetMesh->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(posNode);
		node->SetIndexNo(index); index++;
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		tet_output << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
	}
	for (GLKPOSITION posTet = tetMesh->GetTetraList().GetHeadPosition(); posTet != nullptr;) {
		QMeshTetra* tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(posTet);
		tet_output << "4 " << tet->GetNodeRecordPtr(1)->GetIndexNo()
			<< " " << tet->GetNodeRecordPtr(2)->GetIndexNo()
			<< " " << tet->GetNodeRecordPtr(3)->GetIndexNo()
			<< " " << tet->GetNodeRecordPtr(4)->GetIndexNo() << std::endl;
	}
	tet_output.close();
}

void FileIO::output_toolpath(PolygenMesh* toolPath) {

	std::string TOOLPATH_waypoint_dir = "../DataSet/TOOL_PATH/";
	this->_remove_allFile_in_Dir(TOOLPATH_waypoint_dir);
	std::string CURVED_layer_dir = "../DataSet/CURVED_LAYER/";
	this->_remove_allFile_in_Dir(CURVED_layer_dir);

	std::vector<QMeshPatch*> toolpath_list;

	std::vector<QMeshPatch*> layer_list;

	int min_CompatibleLayer_ind = 1000000; int max_CompatibleLayer_ind = -1000000;
	for (GLKPOSITION Pos = toolPath->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* each_toolpath = (QMeshPatch*)toolPath->GetMeshList().GetNext(Pos);

		if (each_toolpath->compatible_layer_Index > max_CompatibleLayer_ind)
			max_CompatibleLayer_ind = each_toolpath->compatible_layer_Index;

		if (each_toolpath->compatible_layer_Index < min_CompatibleLayer_ind)
			min_CompatibleLayer_ind = each_toolpath->compatible_layer_Index;
	}

	for (int i = min_CompatibleLayer_ind; i <= max_CompatibleLayer_ind; i++) {

		bool find_Support;
		if (toolpath_list.size() == 0) find_Support = true;
		else find_Support = toolpath_list.back()->is_SupportLayer;

		// get split num of layer/toolpath i
		int splitNum = _getSplitNumber_with_CompatibleLayer_ind(i, toolPath, find_Support);

		for (int j = 0; j < splitNum; j++) {
			QMeshPatch* founded_toolpath = _getToolpath_with_CompatibleLayer_ind(i, toolPath, find_Support);

			if (founded_toolpath != NULL) {
				toolpath_list.push_back(founded_toolpath);
				layer_list.push_back(founded_toolpath->attached_Layer);
				//std::cout << "founded_toolpath: " << founded_toolpath->compatible_layer_Index << std::endl;
			}
		}

		// inverse find type
		find_Support = !find_Support;

		// get split num of layer/toolpath i
		splitNum = _getSplitNumber_with_CompatibleLayer_ind(i, toolPath, find_Support);

		for (int j = 0; j < splitNum; j++) {
			QMeshPatch* founded_toolpath = _getToolpath_with_CompatibleLayer_ind(i, toolPath, find_Support);

			if (founded_toolpath != NULL) {
				toolpath_list.push_back(founded_toolpath);
				layer_list.push_back(founded_toolpath->attached_Layer);
				//std::cout << "founded_toolpath: " << founded_toolpath->compatible_layer_Index << std::endl;
			}
		}
	}

	//protection code
	int toolpath_Num = toolPath->GetMeshList().GetCount();
	std::cout << "toolpath_Num: " << toolpath_Num << std::endl;
	if (toolpath_list.size() != toolpath_Num) {
		std::cout << "toolpath_list.size(): " << toolpath_list.size() << std::endl;
		std::cout << "Error: toolpath number is not same with toolpath list. please check" << std::endl;
		return;
	}

	//output waypoints
	for (int i = 0; i < toolpath_list.size(); i++) {

		QMeshPatch* each_toolpath = toolpath_list[i];

		std::string eachToolpath_dir;
		if (each_toolpath->is_SupportLayer) {
			eachToolpath_dir = TOOLPATH_waypoint_dir + std::to_string(i) + "S.txt";
		}
		else {
			eachToolpath_dir = TOOLPATH_waypoint_dir + std::to_string(i) + ".txt";
		}

		//std::cout << "Output File: " << TOOLPATH_dir << std::endl;

		std::ofstream toolpathFile(eachToolpath_dir);

		double pp[3]; double n[3];
		QMeshEdge* sEdge = (QMeshEdge*)each_toolpath->GetEdgeList().GetHead(); // the first Edge of Toolpath
		QMeshNode* sNode = sEdge->GetStartPoint();
		sNode->GetCoord3D(pp[0], pp[1], pp[2]); sNode->GetNormal(n[0], n[1], n[2]);
		toolpathFile << pp[0] << " " << pp[1] << " " << pp[2] << " " << n[0] << " " << n[1] << " " << n[2] << std::endl;

		for (GLKPOSITION posEdge = each_toolpath->GetEdgeList().GetHeadPosition(); posEdge != nullptr;) {
			QMeshEdge* Edge = (QMeshEdge*)each_toolpath->GetEdgeList().GetNext(posEdge);

			//std::cout << "start Node " << Edge->GetStartPoint()->GetIndexNo() << " end Node " << Edge->GetEndPoint()->GetIndexNo() << std::endl;

			QMeshNode* eNode = Edge->GetStartPoint();
			if (eNode == sNode) eNode = Edge->GetEndPoint();

			eNode->GetCoord3D(pp[0], pp[1], pp[2]); eNode->GetNormal(n[0], n[1], n[2]);
			toolpathFile << pp[0] << " " << pp[1] << " " << pp[2] << " " << n[0] << " " << n[1] << " " << n[2] << std::endl;

			sNode = eNode;
		}
		toolpathFile.close();
	}
	std::cout << "Finish output toolpath into : " << ".. / DataSet / TOOL_PATH /" << std::endl;

	//output initial and slimmed support layer
	for (int i = 0; i < layer_list.size(); i++) {

		QMeshPatch* each_layer = layer_list[i];

		std::string eachLayer_dir;
		if (each_layer->is_SupportLayer) {
			eachLayer_dir = CURVED_layer_dir + std::to_string(i) + "S.obj";
		}
		else {
			eachLayer_dir = CURVED_layer_dir + std::to_string(i) + ".obj";
		}

		//std::cout << "Output File: " << TOOLPATH_dir << std::endl;

		std::ofstream layer_str(eachLayer_dir);
		int index = 0; double pp[3];
		for (GLKPOSITION posNode = each_layer->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode* node = (QMeshNode*)each_layer->GetNodeList().GetNext(posNode);
			node->GetCoord3D(pp[0], pp[1], pp[2]);
			layer_str << "v " << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
			index++; node->SetIndexNo(index);
		}
		for (GLKPOSITION posFace = each_layer->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
			QMeshFace* face = (QMeshFace*)each_layer->GetFaceList().GetNext(posFace);
			layer_str << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
				<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
				<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << std::endl;
		}
		layer_str.close();
	}
	std::cout << "Finish output layers into : " << ".. / DataSet / CURVED_LAYER /" << std::endl;
}

QMeshPatch* FileIO::_getToolpath_with_CompatibleLayer_ind(
	int compatibleLayer_ind, PolygenMesh* toolPath, bool find_Support) {

	QMeshPatch* founded_toolpath = NULL;

	for (GLKPOSITION Pos = toolPath->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* each_toolpath = (QMeshPatch*)toolPath->GetMeshList().GetNext(Pos);

		if (each_toolpath->compatible_layer_Index == compatibleLayer_ind) {
			if (each_toolpath->is_SupportLayer == find_Support) {
				if (!each_toolpath->isInstalled_toolpath) {
					founded_toolpath = each_toolpath;
					each_toolpath->isInstalled_toolpath = true;
					break;
				}
			}
		}
	}

	return founded_toolpath;
}

int FileIO::_getSplitNumber_with_CompatibleLayer_ind(
	int compatibleLayer_ind, PolygenMesh* toolPath, bool isSupport) {

	int splitNumber = 0;

	for (GLKPOSITION Pos = toolPath->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* each_toolpath = (QMeshPatch*)toolPath->GetMeshList().GetNext(Pos);

		//std::cout << "compatible ind = " << each_toolpath->compatible_layer_Index
		//	<< " support? " << each_toolpath->is_SupportLayer << std::endl;

		if (each_toolpath->compatible_layer_Index == compatibleLayer_ind) {
			if (each_toolpath->is_SupportLayer == isSupport) {

				splitNumber++;
			}
		}
	}

	return splitNumber;
}

//can only consider the initial layer toolpath
void FileIO::output_toolpath_compatible(PolygenMesh* toolPath) {

	std::string TOOLPATH_waypoint_dir = "../DataSet/tempUse/compatible_toolpath/";
	this->_remove_allFile_in_Dir(TOOLPATH_waypoint_dir);

	int min_CompatibleLayer_ind = 1000000; int max_CompatibleLayer_ind = -1000000;
	for (GLKPOSITION Pos = toolPath->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* each_toolpath = (QMeshPatch*)toolPath->GetMeshList().GetNext(Pos);

		if (each_toolpath->compatible_layer_Index > max_CompatibleLayer_ind)
			max_CompatibleLayer_ind = each_toolpath->compatible_layer_Index;

		if (each_toolpath->compatible_layer_Index < min_CompatibleLayer_ind)
			min_CompatibleLayer_ind = each_toolpath->compatible_layer_Index;
	}

	for (int i = min_CompatibleLayer_ind; i <= max_CompatibleLayer_ind; i++) {
		std::vector<QMeshPatch*> compatible_patchSet =
			_collect_patches_withSame_compatibleIndex(toolPath, i, false);

		std::string eachToolpath_dir;
		eachToolpath_dir = TOOLPATH_waypoint_dir + std::to_string(i) + ".txt";
		//std::cout << "Output File: " << TOOLPATH_dir << std::endl;
		std::ofstream toolpathFile(eachToolpath_dir);
		
		//output waypoints
		for (int i = 0; i < compatible_patchSet.size(); i++) {

			QMeshPatch* each_small_toolpath = compatible_patchSet[i];

			double pp[3]; double n[3];
			QMeshEdge* sEdge = (QMeshEdge*)each_small_toolpath->GetEdgeList().GetHead(); // the first Edge of Toolpath
			QMeshNode* sNode = sEdge->GetStartPoint();
			sNode->GetCoord3D(pp[0], pp[1], pp[2]); sNode->GetNormal(n[0], n[1], n[2]);
			toolpathFile << pp[0] << " " << pp[1] << " " << pp[2] << " " << n[0] << " " << n[1] << " " << n[2] << std::endl;

			for (GLKPOSITION posEdge = each_small_toolpath->GetEdgeList().GetHeadPosition(); posEdge != nullptr;) {
				QMeshEdge* Edge = (QMeshEdge*)each_small_toolpath->GetEdgeList().GetNext(posEdge);

				//std::cout << "start Node " << Edge->GetStartPoint()->GetIndexNo() << " end Node " << Edge->GetEndPoint()->GetIndexNo() << std::endl;

				QMeshNode* eNode = Edge->GetStartPoint();
				if (eNode == sNode) eNode = Edge->GetEndPoint();

				eNode->GetCoord3D(pp[0], pp[1], pp[2]); eNode->GetNormal(n[0], n[1], n[2]);
				toolpathFile << pp[0] << " " << pp[1] << " " << pp[2] << " " << n[0] << " " << n[1] << " " << n[2] << std::endl;

				sNode = eNode;
			}	
		}
		toolpathFile.close();
	}
	std::cout << "Finish output toolpath into : " << ".. / DataSet / tempUse / compatible_toolpath /" << std::endl;
}

std::vector<QMeshPatch*> FileIO::_collect_patches_withSame_compatibleIndex(
	PolygenMesh* toolPath, int targetInd, bool find_Support) {

	std::vector<QMeshPatch*> compatible_patchSet;

	for (GLKPOSITION Pos = toolPath->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* each_toolpath = (QMeshPatch*)toolPath->GetMeshList().GetNext(Pos);

		if (each_toolpath->compatible_layer_Index == targetInd) {
			if (each_toolpath->is_SupportLayer == find_Support) {
				
				compatible_patchSet.push_back(each_toolpath);
			}
		}
	}
	std::cout << "Find " << compatible_patchSet.size()
		<< " small patches in compatible layer " << targetInd << std::endl;
	return compatible_patchSet;
}

void FileIO::outputComparison_VectorField_vs_Objective(PolygenMesh* tet_Model, int caseType) {

	std::string model_Name = tet_Model->getModelName();
	QMeshPatch* tet_patch = (QMeshPatch*)tet_Model->GetMeshList().GetHead();
	std::string baseDir = "../DataSet/fabricationTest/test_vector_VS_objective/";

	char targetFilename[1024];
	//SUPPORT_LESS
	if (caseType == 1 || caseType == 4 || caseType == 6 || caseType == 7) {

		std::sprintf(targetFilename, "%s%s%s", baseDir.c_str(), model_Name.c_str(), "_supportLess_comparison.txt");
		std::cout << "\nPrinting Dir VS normal angle data is written into:\n" << targetFilename << std::endl;

		FILE* fp = fopen(targetFilename, "w");
		if (!fp)	return;

		for (GLKPOSITION Pos = tet_patch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tet_patch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() != NULL && Face->GetRightTetra() != NULL) continue; //boundary tet

			// boundary tet pointer
			QMeshTetra* thisTetra = NULL;
			if (Face->GetLeftTetra() == NULL) thisTetra = Face->GetRightTetra();
			else thisTetra = Face->GetLeftTetra();

			// skip bottom tet
			if (thisTetra->isBottomTet) continue;

			Eigen::Vector3d normal;
			Face->CalPlaneEquation();
			Face->GetNormal(normal(0), normal(1), normal(2));
			normal = -normal;

			double vVSnAngle = acos(thisTetra->vectorField.normalized().dot(normal.normalized())) * 180.0 / PI;
			Eigen::Vector3d printDir; printDir << 0.0, 1.0, 0.0;
			double pdVSnAngle = acos(printDir.dot(normal.normalized())) * 180.0 / PI;

			std::fprintf(fp, "%f %f\n", pdVSnAngle, vVSnAngle);
		}

		fclose(fp);
		std::cout << "--> SUPPORT_LESS Data (vector field VS face normal) Write Finish!\n" << std::endl;
	}

	//STRENGTH_REINFORCEMENT
	if (caseType == 2 || caseType == 5 || caseType == 6 || caseType == 7) {

		std::sprintf(targetFilename, "%s%s%s", baseDir.c_str(), model_Name.c_str(), "_reinforcement_comparison.txt");
		std::cout << "\nVector VS Stress angle data is written into:\n" << targetFilename << std::endl;

		FILE* fp = fopen(targetFilename, "w");
		if (!fp)	return;

		for (GLKPOSITION Pos = tet_patch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tet_patch->GetTetraList().GetNext(Pos);

			if (!Tetra->isTensileorCompressSelect) continue;

			double vVSsAngle = acos(Tetra->vectorField.normalized().dot(Tetra->tau_max.normalized())) * 180.0 / PI;
			Eigen::Vector3d printDir; printDir << 0.0, 1.0, 0.0;
			double pdVSsAngle = acos(printDir.dot(Tetra->tau_max.normalized())) * 180.0 / PI;

			std::fprintf(fp, "%f %f\n", pdVSsAngle, vVSsAngle);
		}

		fclose(fp);
		std::cout << "--> STRENGTH_REINFORCEMENT Data (vector field VS tau_max) Write Finish!\n" << std::endl;
	}

	//SURFACE_QUALITY
	if (caseType == 3 || caseType == 4 || caseType == 5 || caseType == 7) {

		std::sprintf(targetFilename, "%s%s%s", baseDir.c_str(), model_Name.c_str(), "_surfaceQuality_comparison.txt");
		std::cout << "\nPrinting Dir VS normal angle data is written into:\n" << targetFilename << std::endl;

		FILE* fp = fopen(targetFilename, "w");
		if (!fp)	return;

		for (GLKPOSITION Pos = tet_patch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tet_patch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() != NULL && Face->GetRightTetra() != NULL) continue;
			if (!Face->isQualityProtectFace) continue;
			QMeshTetra* thisTetra = NULL;
			if (Face->GetLeftTetra() == NULL) thisTetra = Face->GetRightTetra();
			else thisTetra = Face->GetLeftTetra();

			Eigen::Vector3d normal;
			Face->CalPlaneEquation();
			Face->GetNormal(normal(0), normal(1), normal(2));
			normal = -normal;

			double vVSnAngle = acos(thisTetra->vectorField.normalized().dot(normal.normalized())) * 180.0 / PI;
			Eigen::Vector3d printDir; printDir << 0.0, 1.0, 0.0;
			double pdVSnAngle = acos(printDir.dot(normal.normalized())) * 180.0 / PI;

			std::fprintf(fp, "%f %f\n", pdVSnAngle, vVSnAngle);
		}

		fclose(fp);
		std::cout << "SURFACE_QUALITY Data (vector field VS face normal) Write Finish!\n" << std::endl;
	}
}

void FileIO::output_Quaternion_VectorField(PolygenMesh* tet_Model) {

	std::string model_Name = tet_Model->getModelName();
	QMeshPatch* tet_patch = (QMeshPatch*)tet_Model->GetMeshList().GetHead();
	std::string baseDir = "../DataSet/fabricationTest/test_QvectorField/";

	char targetFilename[1024];

	std::sprintf(targetFilename, "%s%s%s", baseDir.c_str(), model_Name.c_str(), "_QvectorField.txt");
	std::cout << "\nQuaternion vector field is written into:\n" << targetFilename << std::endl;

	FILE* fp = fopen(targetFilename, "w");
	if (!fp)	return;

	for (GLKPOSITION Pos = tet_patch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* thisTetra = (QMeshTetra*)tet_patch->GetTetraList().GetNext(Pos);

		//bounary tet
		bool outerTet = false;
		for (int i = 0; i < 4; i++) {
			if (!thisTetra->GetFaceRecordPtr(i + 1)->inner) {
				outerTet = true;
				break;
			}
		}
		if (outerTet == false) continue;


		double xx, yy, zz;
		thisTetra->CalCenterPos(xx, yy, zz);
		double tetScalarValue = 0.0;
		for (int i = 0; i < 4; i++) {
			tetScalarValue += thisTetra->GetNodeRecordPtr(i + 1)->scalarField;
		}
		tetScalarValue /= 4.0;
		std::fprintf(fp, "%f %f %f %f %f %f %f\n",
			xx, yy, zz,
			thisTetra->vectorField[0], thisTetra->vectorField[1], thisTetra->vectorField[2], tetScalarValue);
	}

	fclose(fp);
	std::cout << "--> Quaternion Vector Field Write Finish!\n" << std::endl;
}

void FileIO::output_ScalarOrHeight_field(PolygenMesh* tet_Model, bool output_scalarField) {

	std::string model_Name = tet_Model->getModelName();
	QMeshPatch* tet_patch = (QMeshPatch*)tet_Model->GetMeshList().GetHead();
	std::string baseDir = "../DataSet/fabricationTest/test_ScalrOrHeightField/";

	char targetFilename[1024];
	if (output_scalarField) {
		std::sprintf(targetFilename, "%s%s%s", baseDir.c_str(), model_Name.c_str(), "_scalarField.txt");
		std::cout << "\nScalar field on original model is written into:\n" << targetFilename << std::endl;
	}
	else {
		std::sprintf(targetFilename, "%s%s%s", baseDir.c_str(), model_Name.c_str(), "_heightField.txt");
		std::cout << "\Height field on deformed model is written into:\n" << targetFilename << std::endl;
	}

	FILE* fp = fopen(targetFilename, "w");
	if (!fp)	return;

	for (GLKPOSITION Pos = tet_patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)tet_patch->GetNodeList().GetNext(Pos);

		if (thisNode->inner) continue;

		float rr, gg, bb;
		thisNode->GetColor(rr, gg, bb);

		if (output_scalarField) {

			std::fprintf(fp, "%f %f %f %f %f %f\n", 
				thisNode->initial_coord3D[0], thisNode->initial_coord3D[1], thisNode->initial_coord3D[2],
				rr, gg, bb);
		}
		else {
			std::fprintf(fp, "%f %f %f %f %f %f\n",
				thisNode->deformed_coord3D[0], thisNode->deformed_coord3D[1], thisNode->deformed_coord3D[2],
				rr, gg, bb);
		}
	}

	fclose(fp);
	std::cout << "--> Scalar/Height Field Write Finish!\n" << std::endl;
}

void FileIO::output_materialRotation_4CAE(PolygenMesh* tet_Model) {

	std::string model_Name = tet_Model->getModelName();
	QMeshPatch* tet_patch = (QMeshPatch*)tet_Model->GetMeshList().GetHead();
	std::string baseDir = "../DataSet/fabricationTest/test_materialOrientation/";

	char targetFilename[1024];
	std::sprintf(targetFilename, "%s%s%s", baseDir.c_str(), model_Name.c_str(), "_materialOrientation.txt");
	std::cout << "\nmaterial orientation is written into:\n" << targetFilename << std::endl;

	FILE* fp = fopen(targetFilename, "w");
	if (!fp)	return;

	int i = 1;
	Eigen::AngleAxisd v(90.0 * PI / 180, Eigen::Vector3d::UnitZ());
	Eigen::Matrix3d R = v.matrix();
	for (GLKPOSITION Pos = tet_patch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* thisTetra = (QMeshTetra*)tet_patch->GetTetraList().GetNext(Pos);

		Eigen::Vector3d tau_max = R * thisTetra->vectorField;
		Eigen::Vector3d tau_mid = thisTetra->vectorField.cross(tau_max).normalized();

		if (thisTetra->isTensileorCompressSelect) {
			Eigen::Vector3d r_axis = thisTetra->vectorField.cross(thisTetra->tau_max);
			r_axis.normalize();
			Eigen::AngleAxis<double> ToolpathDir
				= Eigen::AngleAxis<double>(PI / 2, r_axis);
			tau_max = ToolpathDir.matrix() * thisTetra->vectorField;
		}

		std::fprintf(fp, "%d %f %f %f %f %f %f\n",
			i, tau_max[0], tau_max[1], tau_max[2],
			//tau_mid[0], tau_mid[1], tau_mid[2]);
			thisTetra->vectorField[0], thisTetra->vectorField[1], thisTetra->vectorField[2]);
		i++;
	}

	fclose(fp);
	std::cout << "--> Material Orientation for CAE Write Finish!\n" << std::endl;
}

void FileIO::output_userWaypoints(PolygenMesh* waypointSet, std::string TOOLPATH_waypoint_dir,
	double xMove,double yMove, double zMove) {

	this->_remove_allFile_in_Dir(TOOLPATH_waypoint_dir);

	//output waypoints
	for (GLKPOSITION Pos = waypointSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* each_toolpath = (QMeshPatch*)waypointSet->GetMeshList().GetNext(Pos);

		int i = each_toolpath->GetIndexNo();
		std::string eachToolpath_dir;
		if (each_toolpath->is_SupportLayer) {
			eachToolpath_dir = TOOLPATH_waypoint_dir + std::to_string(i) + "S.txt";
		}
		else {
			eachToolpath_dir = TOOLPATH_waypoint_dir + std::to_string(i) + ".txt";
		}

		//std::cout << "Output File: " << TOOLPATH_dir << std::endl;

		std::ofstream toolpathFile(eachToolpath_dir);

		for (GLKPOSITION posNode = each_toolpath->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode* Node = (QMeshNode*)each_toolpath->GetNodeList().GetNext(posNode);
			
			toolpathFile << Node->m_printPos[0] - xMove << " "
						 << Node->m_printPos[1] - yMove << " "
						 << Node->m_printPos[2] - zMove << " "
						 << Node->m_printNor[0] << " "
						 << Node->m_printNor[1] << " "
						 << Node->m_printNor[2] << std::endl;	
		}
		toolpathFile.close();
	}
	std::cout << "Finish output toolpath into : " << TOOLPATH_waypoint_dir << std::endl;
}

void FileIO::output_stressField(PolygenMesh* tet_Model) {

	QMeshPatch* tet_patch = (QMeshPatch*)tet_Model->GetMeshList().GetHead();
	std::string baseDir = "../DataSet/fabricationTest/test_stressField/";

	std::string finalDir = baseDir + tet_Model->getModelName() + "_stressField.txt";
	std::ofstream stressField(finalDir);
	std::cout << "\nPrinting Dir VS normal angle data is written into:\n" << finalDir << std::endl;

	for (GLKPOSITION posNode = tet_patch->GetTetraList().GetHeadPosition(); posNode != nullptr;) {
		QMeshTetra* tet = (QMeshTetra*)tet_patch->GetTetraList().GetNext(posNode);
		double pp[3];
		tet->CalCenterPos(pp[0], pp[1], pp[2]);

		//special case for armadillo
		if (tet_patch->patchName == "armadillo4" && (!tet->isTensileorCompressSelect))
			continue;
		if (tet_patch->patchName == "bunny_cut6" && (!tet->isTensileorCompressSelect))
			continue;
		//if (tet->eleStress[0] < 300 || tet->eleStress[0] > 800) continue;

		double length = 0.0;
		if (tet->sigma_max < 0) {
			//length = tet->principleStress / tetModel->minPrincipleStressValue;
			length = log10(50.0 * tet->sigma_max / tet_patch->minPrincipleStressValue);
			if (length < 0.01) length = 0;
		}
		else {
			//length = tet->principleStress / tetModel->maxPrincipleStressValue;
			length = log10(50.0 * tet->sigma_max / tet_patch->maxPrincipleStressValue);
			if (length < 0.01) length = 0;
		}

		stressField << pp[0] << "," << pp[1] << "," << pp[2] << "," << length << "," <<
			tet->tau_max(0) << "," << tet->tau_max(1) << "," << tet->tau_max(2) << "," <<
			tet->principleStressColor[0] << "," << tet->principleStressColor[1] << "," << tet->principleStressColor[2] << std::endl;
	}


	stressField.close();
	std::cout << "--> StressField Data Write Finish!\n" << std::endl;
}

void FileIO::output_robotWaypoints(PolygenMesh* waypointSet, std::string TOOLPATH_waypoint_dir) {

	this->_remove_allFile_in_Dir(TOOLPATH_waypoint_dir);

	int first_inital_Layer_idx = -1;
	int first_support_Layer_idx = -1;

	//output waypoints
	for (GLKPOSITION Pos = waypointSet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* each_toolpath = (QMeshPatch*)waypointSet->GetMeshList().GetNext(Pos);

		int i = each_toolpath->GetIndexNo();

		if (each_toolpath->is_SupportLayer && (first_support_Layer_idx == -1)) {
			first_support_Layer_idx = i;
			std::cout << " --> first_support_Layer_idx = " << i << std::endl;
		}
		if (!each_toolpath->is_SupportLayer && (first_inital_Layer_idx == -1)) {
			first_inital_Layer_idx = i;
			std::cout << " --> first_inital_Layer_idx = " << i << std::endl;
		}

		
		//std::string eachToolpath_dir =
		//	TOOLPATH_waypoint_dir +
		//	std::to_string(i /100 % 10) +
		//	std::to_string(i /10 % 10) +
		//	std::to_string(i /1 % 10);

		std::string eachToolpath_dir = TOOLPATH_waypoint_dir + std::to_string(i);
		if (each_toolpath->is_SupportLayer) {
			eachToolpath_dir += "S.txt";
		}
		else {
			eachToolpath_dir += ".txt";
		}

		std::cout << "Output File: " << eachToolpath_dir << std::endl;

		std::ofstream toolpathFile(eachToolpath_dir);

		for (GLKPOSITION posNode = each_toolpath->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode* Node = (QMeshNode*)each_toolpath->GetNodeList().GetNext(posNode);


			// only used for bunny6 model
			//double alpha = 0.2; Eigen::Vector3d vertical_dp = { 0.0,0.0,1.0 };
			//if ((each_toolpath->GetIndexNo() <= 143) && (each_toolpath->GetIndexNo() >= 134)) {

			//	Eigen::Vector3d newNormal = (1.0 - alpha) * Node->m_printNor_4_robot
			//		+ alpha * vertical_dp;
			//	Node->m_printNor_4_robot = newNormal.normalized();
			//}
			//end

			// only used for dome model
			//double alpha = 0.20; Eigen::Vector3d vertical_dp = { 0.0,0.0,1.0 };
			//if ((each_toolpath->GetIndexNo() >= 99) && (each_toolpath->GetIndexNo() <= 109)) {

			//	Eigen::Vector3d newNormal = (1.0 - alpha) * Node->m_printNor_4_robot
			//		+ alpha * vertical_dp;
			//	Node->m_printNor_4_robot = newNormal.normalized();
			//}
			//end

			// only used for yoga model
			//double alpha = 0.50; Eigen::Vector3d vertical_dp = { 0.0,0.0,1.0 };
			//if ((each_toolpath->GetIndexNo() >= 75) && (each_toolpath->GetIndexNo() <= 120) || 
			//(each_toolpath->GetIndexNo() >= 167) && (each_toolpath->GetIndexNo() <= 190) ||
			//	(each_toolpath->GetIndexNo() >= 190) && (each_toolpath->GetIndexNo() <= 293)
			//	) {

			//	Eigen::Vector3d newNormal = (1.0 - alpha) * Node->m_printNor_4_robot
			//		+ alpha * vertical_dp;
			//	Node->m_printNor_4_robot = newNormal.normalized();
			//}
			//end

			double ratio_extutionVolume = 0.9;
			int supportFlag = 1;
			if (each_toolpath->is_SupportLayer) {
				ratio_extutionVolume = 1.05;
				supportFlag = 0;
			}
			if (i == first_inital_Layer_idx || i == first_support_Layer_idx)
				ratio_extutionVolume *= 2.0;

			int jumpFlag = 0;
			if (Node->Jump_nextSecStart || (Node->GetIndexNo() == 0))
				jumpFlag = 1;

			toolpathFile 
				<< Node->m_printPos[0] << " "
				<< Node->m_printPos[1] << " "
				<< Node->m_printPos[2] << " "
				<< Node->m_printNor_4_robot[0] << " "
				<< Node->m_printNor_4_robot[1] << " "
				<< Node->m_printNor_4_robot[2] << " "
				<< (ratio_extutionVolume * Node->m_XYZBCE(5)) << " "
				<< jumpFlag << " "
				<< supportFlag << std::endl;
		}
		toolpathFile.close();
	}
	std::cout << "Output toolpath into : " << TOOLPATH_waypoint_dir << std::endl;
}

void FileIO::output_discreteTet_beforeBlending(QMeshPatch* tetMesh, std::string path) {

	double pp[3];
	std::string path_tet = path + tetMesh->patchName + "_discrete.tet";
	std::string path_node = path + tetMesh->patchName + "_discrete_boundary.xyz";

	int outerTet_num = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		//bounary tet		
		for (int i = 0; i < 4; i++) {
			if (!Tetra->GetFaceRecordPtr(i + 1)->inner) {
				Tetra->isBoundaryTet = true;
				outerTet_num++;
				break;
			}
		}
	}

	std::ofstream boundaryFace_center_output(path_node);

	std::ofstream tet_output(path_tet);
	tet_output << (outerTet_num * 4) << " vertices" << std::endl;
	tet_output << outerTet_num << " tets" << std::endl;

	// apply local rotate to each tet node
	double scaleRatio = 0.8;
	for (GLKPOSITION posTet = tetMesh->GetTetraList().GetHeadPosition(); posTet != nullptr;) {
		QMeshTetra* tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(posTet);

		if (!tet->isBoundaryTet) continue;

		Eigen::Vector3d center = Eigen::Vector3d::Zero();
		for (int i = 0; i < 4; i++) {		
			center += tet->GetNodeRecordPtr(i + 1)->initial_coord3D;
		}
		center /= 4.0;
		for (int i = 0; i < 4; i++) {

			QMeshNode* each_4nodes = tet->GetNodeRecordPtr(i + 1);
			each_4nodes->each_4nodes_newCoord3D = scaleRatio * tet->R_estimate * (each_4nodes->initial_coord3D - center) + center;
			tet_output 
				<< each_4nodes->each_4nodes_newCoord3D[0] << " "
				<< each_4nodes->each_4nodes_newCoord3D[1] << " "
				<< each_4nodes->each_4nodes_newCoord3D[2] << std::endl;				
		}

		// record the center node position
		for (int i = 0; i < 4; i++) { //each face
			QMeshFace* each_4faces = tet->GetFaceRecordPtr(i + 1);
			if (each_4faces->inner) continue;
			Eigen::Vector3d center_outerFace = Eigen::Vector3d::Zero();
			for (int j = 0; j < 3; j++) {
				center_outerFace += each_4faces->GetNodeRecordPtr(j)->each_4nodes_newCoord3D;
			}
			center_outerFace /= 3.0;

			boundaryFace_center_output
				<< center_outerFace[0] << " " << center_outerFace[1] << " " << center_outerFace[2] << std::endl;
		}
	}
	boundaryFace_center_output.close();

	int ind = 0;
	for (GLKPOSITION posTet = tetMesh->GetTetraList().GetHeadPosition(); posTet != nullptr;) {
		QMeshTetra* tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(posTet);
		if (!tet->isBoundaryTet) continue;
		tet_output << "4 " << (ind * 4 + 0) 
			<< " " << (ind * 4 + 1) 
			<< " " << (ind * 4 + 2) 
			<< " " << (ind * 4 + 3) 
			<< std::endl;
		ind++;
	}
	tet_output.close();
}

void FileIO::output_discreteTet_obj_beforeBlending(QMeshPatch* tetMesh, std::string path) {

	//read the boundary face center position file
	Eigen::MatrixXd boundaryFace_Set = _read_boundaryFace_centerPosFile(tetMesh, path);
	//find the nearest one as boundary face
	this->_mark_boundaryFace(tetMesh,boundaryFace_Set);

	double pp[3];
	std::string path_allFace = path + tetMesh->patchName + ".obj";
	std::ofstream obj_output(path_allFace);

	int index = 0;
	for (GLKPOSITION posNode = tetMesh->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(posNode);
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		obj_output << "v " << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
		index++; node->SetIndexNo(index);
	}
	for (GLKPOSITION posFace = tetMesh->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetMesh->GetFaceList().GetNext(posFace);
		obj_output << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << std::endl;
	}
	obj_output.close();

	//output boundary face
	std::string path_boundaryFace = path + tetMesh->patchName + "_onlyBoundary.obj";
	std::ofstream boundary_obj_output(path_boundaryFace);

	index = 0;
	for (GLKPOSITION posNode = tetMesh->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(posNode);

		if (!node->isDiscreteBoundary) continue;
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		boundary_obj_output << "v " << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
		index++; node->discreteBoundary_ind = index;
	}
	for (GLKPOSITION posFace = tetMesh->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetMesh->GetFaceList().GetNext(posFace);

		if (!face->isDiscreteBoundary) continue;

		boundary_obj_output << "f " << face->GetNodeRecordPtr(0)->discreteBoundary_ind
			<< " " << face->GetNodeRecordPtr(1)->discreteBoundary_ind
			<< " " << face->GetNodeRecordPtr(2)->discreteBoundary_ind << std::endl;
	}
	boundary_obj_output.close();
}

Eigen::MatrixXd FileIO::_read_boundaryFace_centerPosFile(QMeshPatch* tetMesh, std::string path) {

	std::string full_path = path + tetMesh->patchName + "_boundary.xyz";

	char filename[1024];
	std::sprintf(filename, "%s", full_path.c_str());
	std::cout << "boundary face ceter file is read from:\n" << filename << std::endl;

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
	int boundaryFace_Num = i_temp;
	Eigen::MatrixXd boundaryFace_Set = Eigen::MatrixXd::Zero(boundaryFace_Num, 3);

	fp = fopen(filename, "r");
	if (!fp) {
		printf("===============================================\n");
		printf("Can not open the data file 2!\n");
		printf("===============================================\n");
	}
	float xx, yy, zz, heatField;
	i_temp = 0;
	while (1) { // right read CODE !!!
		fgets(linebuf, 255, fp);
		if (feof(fp)) break;
		sscanf(linebuf, "%f %f %f\n", &xx, &yy, &zz);
		boundaryFace_Set.row(i_temp) << xx, yy, zz;
		i_temp++;
	}
	fclose(fp);
	// std::cout << boundaryFace_Set << std::endl;

	return boundaryFace_Set;
}

void FileIO::_mark_boundaryFace(QMeshPatch* tetMesh, const Eigen::MatrixXd& boundaryFace_Set) {

	std::vector<QMeshFace*> faceSet(tetMesh->GetFaceNumber()); int ind = 0;
	for (GLKPOSITION posFace = tetMesh->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetMesh->GetFaceList().GetNext(posFace);
		faceSet[ind] = face;	ind++;
	}
#pragma omp parallel
	{
#pragma omp for
		for (int faceInd = 0; faceInd < faceSet.size(); faceInd++) {

			Eigen::RowVector3d face_center = Eigen::Vector3d::Zero();
			faceSet[faceInd]->CalCenterPos(face_center[0], face_center[1], face_center[2]);

			//get the nearest node distance
			double shortestDist = 1e10;
			for (int i = 0; i < boundaryFace_Set.rows(); i++) {
				double dist = (face_center - boundaryFace_Set.row(i)).squaredNorm();
				if (dist < shortestDist)
					shortestDist = dist;
			}
			if (shortestDist < 1e-5) faceSet[faceInd]->isDiscreteBoundary = true;
			for (int i = 0; i < 3; i++) {
				faceSet[faceInd]->GetNodeRecordPtr(i)->isDiscreteBoundary = true;
			}
		}}
}