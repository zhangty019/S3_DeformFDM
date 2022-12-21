#include <Eigen/Eigen>
#include <Eigen/PardisoSupport>

#include "DeformTet.h"
#include "PrincipleStressField.h"
#include "../GLKLib/GLKGeometry.h"
#include "heatMethod.h"

DeformTet::DeformTet(){}

DeformTet::~DeformTet(){}

void DeformTet::initial(PolygenMesh* polygenMesh_TetModel, 
	int outer_loopTime, int inner_loopTime, 
	double criticalTet_weight_SL, double criticalTet_weight_SR,
	double criticalTet_weight_SQC, double criticalTet_weight_SQV,
	double neighborScale_weight, double regularScale_weight, 
	double globalSmooth_weight, int caseType) {

	tetPatch = (QMeshPatch*)polygenMesh_TetModel->GetMeshList().GetHead();
	tetModel_Name = polygenMesh_TetModel->getModelName();

	this->_index_initial(tetPatch, true);
	this->_build_tetraSet_4SpeedUp(tetPatch);
	this->_moveModelup2ZeroHeight(tetPatch); //should before the initialCoord3d record
	this->_record_initial_coord3D();
	this->_record_neighbor_Tet();
	this->_record_initial_normal3D();
	this->_compTetMeshVolumeMatrix(tetPatch);
	this->_calculate_edgeLength(tetPatch);
	if(caseType == 1 
		//|| caseType == 2 // fix the bottom at SR case if need, please open it
		|| caseType == 3 // fix the bottom at SR case
		|| caseType == 4 
		|| caseType == 6
		|| caseType == 7
		)
		this->_detectBottomTet(m_bottom_height); 

	m_loopTime = outer_loopTime;
	m_innerLoopTime = inner_loopTime;
	m_criticalTet_weight_SL = criticalTet_weight_SL;
	m_criticalTet_weight_SR = criticalTet_weight_SR;
	m_criticalTet_weight_SQ_cover = criticalTet_weight_SQC;
	m_criticalTet_weight_SQ_vertical = criticalTet_weight_SQV;
	m_neighborScale_weight = neighborScale_weight;
	m_regularScale_weight = regularScale_weight;
	m_globalSmooth_weight = globalSmooth_weight;
	m_caseType = caseType;
}

void DeformTet::initial(PolygenMesh* polygenMesh_TetModel) {

	tetPatch = (QMeshPatch*)polygenMesh_TetModel->GetMeshList().GetHead();
	tetModel_Name = polygenMesh_TetModel->getModelName();
	this->_index_initial(tetPatch, true);
}

void DeformTet::update_quaternionSmooth_weight(
	double keepWeight_SL, double keepWeight_SR, 
	double keepWeight_SQC, double keepWeight_SQV) {

	m_keep_SL = keepWeight_SL;
	m_keep_SR = keepWeight_SR;
	m_keep_SQ_cover = keepWeight_SQC;
	m_keep_SQ_vertical = keepWeight_SQV;
}

void DeformTet::_index_initial(QMeshPatch* patch, bool is_TetMesh) {

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

void DeformTet::_build_tetraSet_4SpeedUp(QMeshPatch* patch) {

	tetraPatch_elementSet.resize(patch->GetTetraNumber());
	int index = 0;
	for (GLKPOSITION Pos = patch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)patch->GetTetraList().GetNext(Pos);
		tetraPatch_elementSet[index] = Tetra; index++;
	}

	// protect operation
	if (patch->GetTetraNumber() != index) std::cout << "Error: please check the num of tet mesh(materialSpace)! " << std::endl;

	std::cout << "Finish building tet set for SpeedUp." << std::endl;
}

void DeformTet::_moveModelup2ZeroHeight(QMeshPatch* patch){

	// move to the Zero height (follow the operation in SIGGRAPH2018)
	double xmin, ymin, zmin, xmax, ymax, zmax;
	patch->ComputeBoundingBox(xmin, ymin, zmin, xmax, ymax, zmax);
	std::cout << "the min y coord3D is " << ymin << " move it to Zero Height" << std::endl;
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);
		yy -= ymin;

		node->SetCoord3D(xx, yy, zz);
		node->SetCoord3D_last(xx, yy, zz);
	}
}

void DeformTet::_record_initial_coord3D() {

	 tetPatch->ComputeBoundingBox(inital_range);
	 std::cout <<" X range:" << inital_range[0] << " " << inital_range[1] << std::endl;

	for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

		Eigen::Vector3d initial_pos = Eigen::Vector3d::Zero();
		node->GetCoord3D(initial_pos[0], initial_pos[1], initial_pos[2]);
		node->initial_coord3D = initial_pos;
	}
	std::cout << "Finish recording initial position." << std::endl;
}

void DeformTet::_record_neighbor_Tet() {

	for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[i];
		std::vector< QMeshTetra* > Tetra_neighbSet;
		_detect_each_neighbor_Tet(Tetra_neighbSet, Tetra, false);
		Tetra->neighborCell = Tetra_neighbSet;
	}
	std::cout << "Finish recording neighbor Tet." << std::endl;
}

void DeformTet::_record_initial_normal3D() {

	for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

		if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) {

			double nx_last, ny_last, nz_last = 0.0;
			Face->CalPlaneEquation();
			Face->GetNormal(nx_last, ny_last, nz_last);
			Face->normal_last << nx_last, ny_last, nz_last;

			//std::cout << "------------------->>>>>>>------------------ " << Face->normal_last.transpose() << std::endl;
		}
	}
}

void DeformTet::_compTetMeshVolumeMatrix(QMeshPatch* patch) {

	//-- This function calculate the volume matrix 
	//   for each tetrahedral elements and installed in formate
	/* [   b1 c1 d1
		   b2 c2 d2
		   b3 c3 d3
		   b4 c4 d4   ] */

	for (GLKPOSITION Pos = patch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)patch->GetTetraList().GetNext(Pos);

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

//neighb_type == true; // use node to detect neighbor
//neighb_type == false;// use face to detect neighbor
void DeformTet::_detect_each_neighbor_Tet(std::vector< QMeshTetra* >& TetraSet, QMeshTetra* Tetra, bool neighb_type) {

	//using node to find neighbor tetrahedral 
	if (neighb_type) {
		for (int i = 0; i < 4; i++) {
			QMeshNode* thisNode = Tetra->GetNodeRecordPtr(i + 1);
			for (GLKPOSITION Pos = thisNode->GetTetraList().GetHeadPosition(); Pos;) {
				QMeshTetra* ConnectTetra = (QMeshTetra*)thisNode->GetTetraList().GetNext(Pos);

				if (ConnectTetra == Tetra) continue;
				bool exist_in_set = false;

				for (int j = 0; j < TetraSet.size(); j++) {
					if (ConnectTetra->GetIndexNo() == TetraSet[j]->GetIndexNo()) {
						exist_in_set = true; break;
					}
				}
				if (exist_in_set) continue;

				//the connected tetra has not being processed before
				TetraSet.push_back(ConnectTetra);
			}
		}
	}


	//using face to find neighbor tetrahedral 
	else {
		for (int i = 0; i < 4; i++) {
			QMeshFace* thisFace = Tetra->GetFaceRecordPtr(i + 1);

			if (thisFace->GetLeftTetra() == NULL || thisFace->GetRightTetra() == NULL) continue;

			QMeshTetra* ConnectTetra = thisFace->GetRightTetra();
			if (Tetra == ConnectTetra) ConnectTetra = thisFace->GetLeftTetra();

			if (ConnectTetra == Tetra) continue;
			bool exist_in_set = false;

			for (int j = 0; j < TetraSet.size(); j++) {
				if (ConnectTetra->GetIndexNo() == TetraSet[j]->GetIndexNo()) {
					exist_in_set = true; break;
				}
			}
			if (exist_in_set) continue;

			//the connected tetra has not being processed before
			TetraSet.push_back(ConnectTetra);
		}
	}
}

void DeformTet::_calculate_edgeLength(QMeshPatch* patch) {

	double whole_Length = 0.0;
	for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);

		whole_Length += edge->CalLength();
	}

	std::cout << "average length = " << (whole_Length / (patch->GetEdgeNumber())) << std::endl;
}

// 1->voxelOrder guess;
// 2->heatField file guess;
// 3->heatMethod calculation;
// 4->selected source/sink;
void DeformTet::preProcess_4SupportLess(int initialGuess_case) {
	
	if (initialGuess_case == 1) {
		this->_input_VoxelOrder();
		std::printf("Use initial guess from voxel order.\n");
	}
	if (initialGuess_case == 2)	{
		this->_input_HeatField();
		std::printf("Use initial guess from heat field.\n");
	}
	if (initialGuess_case == 3)	{
		this->_cal_heatMethod();
		std::printf("Use initial guess from heat method.\n");
	}
	if (initialGuess_case == 4)	{
		this->_cal_testScalarField();
		std::printf("Use initial guess from selected source and sink, please first select them use UI tool.\n");
	}
}

// height bellow (1mm) will be considered as bottom
void DeformTet::_detectBottomTet(double threshold) {

	//mark the bottom
	double minHight = 99999.0;
	for (GLKPOSITION pos = tetPatch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(pos);
		double pp[3] = { 0 }; node->GetCoord3D(pp[0], pp[1], pp[2]);
		if (pp[1] < minHight) minHight = pp[1];
	}

	std::printf("The minume height: %f.\n", minHight);
	
	for (GLKPOSITION pos = tetPatch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(pos);
		double pp[3] = { 0 }; node->GetCoord3D(pp[0], pp[1], pp[2]);
		if (pp[1] < minHight + threshold) {
			node->isBottomNode = true;
			//if (pp[2] > 0.0 && pp[0]>15.0) node->isBottomNode = true;
		}
	}
	// mark bottom tet element
	int bottom_Num = 0;
	for (GLKPOSITION pos = tetPatch->GetTetraList().GetHeadPosition(); pos != nullptr;) {
		QMeshTetra* tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(pos);
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
	for (GLKPOSITION pos = tetPatch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(pos);
		node->isBottomNode = false;
	}
}

// read the Voxel order result generated from 
// https://dl.acm.org/doi/10.1145/3197517.3201342
void DeformTet::_input_VoxelOrder() {

	char filename[1024];
	std::sprintf(filename, "%s%s%s", "../DataSet/support_less_initial_guess/", tetModel_Name.c_str(), "_SL_initialGuess.txt");
	std::cout << "\nvoxel order is read from:\n" << filename << std::endl;

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
	int voxel_Num = i_temp;
	Eigen::MatrixXd voxelSet = Eigen::MatrixXd::Zero(voxel_Num, 4);

	fp = fopen(filename, "r");
	if (!fp) {
		printf("===============================================\n");
		printf("Can not open the data file 2!\n");
		printf("===============================================\n");
	}
	float xx, yy, zz; int order;
	i_temp = 0;
	while (1) { // right read CODE !!!
		fgets(linebuf, 255, fp);
		if (feof(fp)) break;
		sscanf(linebuf, "%f %f %f %d\n", &xx, &yy, &zz, &order);
		voxelSet.row(i_temp) << xx, yy, zz, order;
		i_temp++;
	}
	fclose(fp);
	//std::cout << voxelSet << std::endl;

	//build a tetmesh_NodeSet_4SpeedUp
	i_temp = 0;
	std::vector<QMeshNode*> NodeSet_4SpeedUp(tetPatch->GetNodeNumber());
	for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

		NodeSet_4SpeedUp[i_temp] = node;
		i_temp++;
	}

	Eigen::VectorXd guideField = Eigen::VectorXd::Zero(tetPatch->GetNodeNumber());
#pragma omp parallel
	{
#pragma omp for
		for (int j = 0; j < NodeSet_4SpeedUp.size(); j++) {

			double minSquaDist = INFINITY;
			Eigen::Vector3d node_coord3D = Eigen::Vector3d::Zero();
			NodeSet_4SpeedUp[j]->GetCoord3D(node_coord3D[0], node_coord3D[1], node_coord3D[2]);

			for (int k = 0; k < voxelSet.rows(); k++) {

				Eigen::Vector3d voxelCenter_coord3D = Eigen::Vector3d::Zero();
				for (int m = 0; m < 3; m++) {
					voxelCenter_coord3D[m] = voxelSet.row(k)[m];
				}

				double SqualDist = (node_coord3D - voxelCenter_coord3D).squaredNorm();
				if (SqualDist < minSquaDist) {
					minSquaDist = SqualDist;
					guideField[j] = voxelSet.row(k)[3];
				}
			}
			//std::cout << "nodeIndex: " << j << " guideField: " << guideField[j] << std::endl;
		}
	}

	// record the growingField_4voxelOrder(without normalized) for each node
	i_temp = 0;
	for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

		Node->growingField_4voxelOrder = guideField(i_temp);
		i_temp++;
	}
	std::cout << "finish input initialguess growingField [ " << guideField.rows() << " x " << guideField.cols() << " ]" << std::endl;

	// using equation to compute gradient of voxel order
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
		Eigen::VectorXd fieldValue = Eigen::VectorXd::Zero(4);

		for (int i = 0; i < 4; i++) {
			fieldValue(i) = Tetra->GetNodeRecordPtr(i + 1)->growingField_4voxelOrder;
			for (int j = 0; j < 3; j++) {
				gradient(j) += Tetra->VolumeMatrix(i, j) * fieldValue(i);
			}
		}

		//Tetra->scalarFieldGradient = gradient.normalized();
		// modify it to release the constraits of layer height
		Tetra->vectorField_4voxelOrder = gradient; 
	}

	//smooth the vector field
	for (int iter = 0; iter < 20; iter++) {
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int neighborCount = 0;

			QMeshTetra* neighTetra;
			Eigen::Vector3d averageField = Eigen::Vector3d::Zero();

			//find neighbor tetrahedral element
			for (int i = 0; i < 4; i++) {
				QMeshFace* Face = Tetra->GetFaceRecordPtr(i + 1);
				if (Face->GetLeftTetra() == nullptr || Face->GetRightTetra() == nullptr) continue;

				if (Tetra == Face->GetLeftTetra()) neighTetra = Face->GetRightTetra();
				else neighTetra = Face->GetLeftTetra();

				averageField += neighTetra->vectorField_4voxelOrder;

				neighborCount++;
			}

			for (int i = 0; i < 3; i++) averageField(i) /= neighborCount;

			//Tetra->vectorField = averageField.normalized(); // simple show
			// remove the normalize to release the constraits of layer height
			Tetra->vectorField_4voxelOrder = averageField.normalized(); //=====================================================//
		}
	}

	std::printf("\n input voxel order as initial guess.\n");
}

void DeformTet::_input_HeatField() {

	char filename[1024];
	std::sprintf(filename, "%s%s%s", "../DataSet/support_less_initial_guess/", tetModel_Name.c_str(), "_HF_initialGuess.txt");
	std::cout << "\nvoxel order is read from:\n" << filename << std::endl;

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
	int voxel_Num = i_temp;
	Eigen::MatrixXd voxelSet = Eigen::MatrixXd::Zero(voxel_Num, 4);

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
		sscanf(linebuf, "%f %f %f %f\n", &xx, &yy, &zz, &heatField);
		voxelSet.row(i_temp) << xx, yy, zz, heatField;
		i_temp++;
	}
	fclose(fp);
	// std::cout << voxelSet << std::endl;

	// record the growingField_4voxelOrder(without normalized) for each node
	i_temp = 0;
	for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

		Node->growingField_4voxelOrder = 1.0 - voxelSet.row(i_temp)[3];
		i_temp++;
	}

	// using equation to compute gradient of voxel order
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
		Eigen::VectorXd fieldValue = Eigen::VectorXd::Zero(4);

		for (int i = 0; i < 4; i++) {
			fieldValue(i) = Tetra->GetNodeRecordPtr(i + 1)->growingField_4voxelOrder;
			for (int j = 0; j < 3; j++) {
				gradient(j) += Tetra->VolumeMatrix(i, j) * fieldValue(i);
			}
		}

		//Tetra->scalarFieldGradient = gradient.normalized();
		// modify it to release the constraits of layer height
		Tetra->vectorField_4voxelOrder = gradient;
	}

	//smooth the vector field
	for (int iter = 0; iter < 20; iter++) {
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int neighborCount = 0;

			QMeshTetra* neighTetra;
			Eigen::Vector3d averageField = Eigen::Vector3d::Zero();

			//find neighbor tetrahedral element
			for (int i = 0; i < 4; i++) {
				QMeshFace* Face = Tetra->GetFaceRecordPtr(i + 1);
				if (Face->GetLeftTetra() == nullptr || Face->GetRightTetra() == nullptr) continue;

				if (Tetra == Face->GetLeftTetra()) neighTetra = Face->GetRightTetra();
				else neighTetra = Face->GetLeftTetra();

				averageField += neighTetra->vectorField_4voxelOrder;

				neighborCount++;
			}

			for (int i = 0; i < 3; i++) averageField(i) /= neighborCount;

			//Tetra->vectorField = averageField.normalized();
			// remove the normalize to release the constraits of layer height
			//Tetra->vectorField_4voxelOrder = averageField.normalized(); //=====================================================//
		}
	}

	std::printf("\n input heat method result as initial guess.\n");
}

void DeformTet::_cal_heatMethod(){

	heatMethod* Heat = new heatMethod(tetPatch, m_bottom_height);
	Heat->generateHeatField(100);
	delete Heat;

	// record the growingField_4voxelOrder(without normalized) for each node
	for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

		Node->growingField_4voxelOrder = 1.0 - Node->HeatFieldValue;
	}

	// using equation to compute gradient of voxel order
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
		Eigen::VectorXd fieldValue = Eigen::VectorXd::Zero(4);

		for (int i = 0; i < 4; i++) {
			fieldValue(i) = Tetra->GetNodeRecordPtr(i + 1)->growingField_4voxelOrder;
			for (int j = 0; j < 3; j++) {
				gradient(j) += Tetra->VolumeMatrix(i, j) * fieldValue(i);
			}
		}

		//Tetra->scalarFieldGradient = gradient.normalized();
		// modify it to release the constraits of layer height
		Tetra->vectorField_4voxelOrder = gradient;
	}

	//smooth the vector field
	for (int iter = 0; iter < 20; iter++) {
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int neighborCount = 0;

			QMeshTetra* neighTetra;
			Eigen::Vector3d averageField = Eigen::Vector3d::Zero();

			//find neighbor tetrahedral element
			for (int i = 0; i < 4; i++) {
				QMeshFace* Face = Tetra->GetFaceRecordPtr(i + 1);
				if (Face->GetLeftTetra() == nullptr || Face->GetRightTetra() == nullptr) continue;

				if (Tetra == Face->GetLeftTetra()) neighTetra = Face->GetRightTetra();
				else neighTetra = Face->GetLeftTetra();

				averageField += neighTetra->vectorField_4voxelOrder;

				neighborCount++;
			}

			for (int i = 0; i < 3; i++) averageField(i) /= neighborCount;

			Tetra->vectorField = averageField.normalized(); // simple show
			// remove the normalize to release the constraits of layer height
			//Tetra->vectorField_4voxelOrder = averageField.normalized(); //=====================================================//
		}
	}

	std::printf("\n calculate heat value as initial guess.\n");
}

void DeformTet::_cal_testScalarField() {

	for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

		if (thisNode->isFixed) thisNode->growingField_4voxelOrder = 0.0;
		if (thisNode->isHandle) thisNode->growingField_4voxelOrder = 1.0;
	}

	for (int loop = 0; loop < 1000; loop++) {
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

			if (thisNode->isFixed || thisNode->isHandle) continue;

			double avgScalarValue = 0.0;
			for (int i = 0; i < thisNode->GetEdgeNumber(); i++) {

				QMeshEdge* thisEdge = thisNode->GetEdgeRecordPtr(i + 1);
				QMeshNode* NeighboorNode = thisEdge->GetStartPoint();
				if (thisNode == thisEdge->GetStartPoint()) NeighboorNode = thisEdge->GetEndPoint();

				avgScalarValue += NeighboorNode->growingField_4voxelOrder;
			}
			thisNode->growingField_4voxelOrder = avgScalarValue / thisNode->GetEdgeNumber();
		}
	}

	// using equation to compute gradient of voxel order
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
		Eigen::VectorXd fieldValue = Eigen::VectorXd::Zero(4);

		for (int i = 0; i < 4; i++) {
			fieldValue(i) = Tetra->GetNodeRecordPtr(i + 1)->growingField_4voxelOrder;
			for (int j = 0; j < 3; j++) {
				gradient(j) += Tetra->VolumeMatrix(i, j) * fieldValue(i);
			}
		}

		//Tetra->scalarFieldGradient = gradient.normalized();
		// modify it to release the constraits of layer height
		Tetra->vectorField_4voxelOrder = gradient;
	}

	std::printf("\n give selected sorce and sink and do laplacian smooth as initial guess.\n");
}

void DeformTet::runASAP_SupportLess(bool initialGuess) {

	//---------------------------------------------------------------------------------------------------------------
	// Initialize size of nodes, faces and tets
	int node_Num = tetPatch->GetNodeNumber();
	int tet_Num = tetPatch->GetTetraNumber();
	int face_Num = tetPatch->GetFaceNumber();

	int neighborScale_face_num = 0;
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL)	neighborScale_face_num++;
	}

	std::printf("All face num %d , neighborScale_face_num %d.\n", face_Num, neighborScale_face_num);
	std::printf("A matrix sixe %d x %d\n", (5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));

	//---------------------------------------------------------------------------------------------------------------
	// Define variables
	std::vector<Eigen::MatrixXd> frame_Local_new, frame_Local_initial_inverse, frame_Local_initial;
	std::vector<Eigen::VectorXd> vector_X_Pos_Scale, vector_b;
	// Initialize variables
	frame_Local_new.resize(tet_Num);
	frame_Local_initial_inverse.resize(tet_Num);
	frame_Local_initial.resize(tet_Num);
	vector_X_Pos_Scale.resize(3);
	vector_b.resize(3);

	for (int i = 0; i < 3; i++) {
		vector_X_Pos_Scale[i] = Eigen::VectorXd::Zero(node_Num + tet_Num);
		vector_b[i] = Eigen::VectorXd::Zero(5 * tet_Num + neighborScale_face_num);
	}
	std::cout << "Finish initializing ASAP matrices" << std::endl;

	//---------------------------------------------------------------------------------------------------------------
	// Pre-compute the initial frame and inverse of initial frame
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tet->GetIndexNo();
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
		double center[3] = { 0 };
		QMeshNode* nodes[4];

		for (int i = 0; i < 4; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= 4;
		for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		frame_Local_initial[fdx] = P;
		frame_Local_initial_inverse[fdx] = Eigen::MatrixXd::Zero(3, 4);
		frame_Local_initial_inverse[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();
		//std::cout << frame_Local_initial[fdx] << std::endl << std::endl;
	}
	std::printf("Finish computing local frame and its inverse!\n\n");

	this->_detectOverhangFace();
	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	for (int loop = 0; loop < m_loopTime; loop++) {

		//if (loop % 5 == 0)	this->_detectOverhangFace();

		//Calculate the fabrication energy of SupportLess case ()
		this->_calFabricationEnergy_SupportLess();

#pragma omp parallel
		{
#pragma omp for 
			// local rotation and scaling operation (frame_Local_new)
			for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
				QMeshTetra* Tetra = tetraPatch_elementSet[i];

				int fdx = Tetra->GetIndexNo();
				double center[3] = { 0.0 };
				Tetra->CalCenterPos(center[0], center[1], center[2]);

				//This tP is each current frame in each tet.
				Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
				QMeshNode* nodes[4];
				for (int i = 0; i < 4; i++) {
					nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
					nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
				}
				for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

				Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
				Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
				Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

				T = frame_Local_initial_inverse[fdx] * tP;
				T_transpose = T.transpose();

				///// R1 //// Eigen SVD decomposition /////
				Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
				Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
				R = U * V.transpose();
				Tetra->R_estimate = R;
				frame_Local_new[fdx] = R * frame_Local_initial[fdx];

				///// R2.1 //// Fabrication requirement (voxel guess) /////
				if ((loop == 0) && (initialGuess)) {

					Eigen::Vector3d voxelOrder_Dir = Tetra->R_estimate * Tetra->vectorField_4voxelOrder;

					Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();		

					//rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(voxelOrder_Dir, printDir);

					/// <summary>
					/// release the initial rotation constraints
					/// </summary>
					/// <param name="rotation angle 45deg"></param>	
					double threshold_angle = 30.0; //30 is for heat method case
					Eigen::Vector3d voxelOrder_Dir_normalized = voxelOrder_Dir.normalized();
					double rad_angle = acos(printDir.dot(voxelOrder_Dir_normalized));
					if (ROTATE_TO_DEGREE(rad_angle) > threshold_angle) {
						Eigen::Vector3d r_axis = this->printDir.cross(voxelOrder_Dir_normalized);
						r_axis.normalize();
						Eigen::AngleAxis<double> sptFree_rotation = Eigen::AngleAxis<double>(threshold_angle * PI / 180.0, r_axis);
						Eigen::Vector3d targetDir = sptFree_rotation.matrix() * this->printDir;
						rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(voxelOrder_Dir, targetDir);
					}
					/// end

					Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
					frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
				}

				///// R2.2 //// Fabrication requirement (support less) /////
				if (Tetra->isOverhangTet) {

					if ((loop == 0) && initialGuess) continue;

					QMeshFace* eachFace = NULL;		int overhangFace_Num = 0;
					Eigen::Vector3d whole_overhang_face_normal = Eigen::Vector3d::Zero();
					for (int i = 0; i < 4; i++) {
						eachFace = Tetra->GetFaceRecordPtr(i + 1);
						if (eachFace->isOverhangFace) {

							Eigen::Vector3d normal;
							eachFace->CalPlaneEquation();
							eachFace->GetNormal(normal(0), normal(1), normal(2));
							normal = -normal;
							normal.normalize();
							whole_overhang_face_normal += normal;
							overhangFace_Num++;
						}
					}
					if (overhangFace_Num == 0) std::cout << "ERROR: none of overhang face is detected!" << std::endl;
					whole_overhang_face_normal.normalize();

					Eigen::Vector3d r_axis = this->printDir.cross(whole_overhang_face_normal); 
					// CAUSION:!!!!!!!
					// cross product range is [0,180] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					// so
					// angle <1,normal> (<) angle <2,nomal>
					r_axis.normalize();// OMG!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					Eigen::AngleAxis<double> sptFree_rotation
						= Eigen::AngleAxis<double>((this->m_supportFreeAngle + 90.0) * PI / 180.0, r_axis);
					Eigen::Vector3d sptFree_normal = sptFree_rotation.matrix() * this->printDir;

					Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
					rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(whole_overhang_face_normal, sptFree_normal);

					Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
					frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
				}
			}
		}
		std::printf("Finish new local frame calculation.\n");

		bool is_Smooth = true;
		if (is_Smooth) {
			bool concavity_smooth = false;
			if (concavity_smooth) {
				if (loop == 0)
					this->_globalQuaternionSmooth2();
				else 
					this->_globalQuaternionSmooth3();
			}
			else {
				this->_globalQuaternionSmooth2();
				//this->_globalQuaternionSmooth1();
			}

			for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
				QMeshTetra* Tetra = tetraPatch_elementSet[i];
				frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
			}
			std::printf("Finish quaternion smooth calculation.\n");
		}

		// Define variables
		Eigen::SparseMatrix<double> matrix_A_4x;
		Eigen::SparseMatrix<double> matrix_A_transpose_4x;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4x;
		//
		Eigen::SparseMatrix<double> matrix_A_4y;
		Eigen::SparseMatrix<double> matrix_A_transpose_4y;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4y;
		//
		Eigen::SparseMatrix<double> matrix_A_4z;
		Eigen::SparseMatrix<double> matrix_A_transpose_4z;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4z;
		// Initialize variables
		matrix_A_4x.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4x.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4y.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4y.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4z.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4z.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));

		// Fill A matrix for x,y,z
		// Block 11 for x,y,z
		// Important issues
		//give memory to sparse matrix, to accerate the insert speed
		matrix_A_4x.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4y.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4z.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));

		float c1 = -0.25, c2 = 0.75;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int fdx = Tetra->GetIndexNo() * 4;	int vdxArr[4];
			for (int i = 0; i < 4; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

			double weight = 1.0;
			if (Tetra->isOverhangTet )
				weight = m_criticalTet_weight_SL;

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (i == j) {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c2 * weight;
					}
					else {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c1 * weight;
					}
				}
			}
		}

		// Block 12 for x,y,z
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			double weight = 1.0;
			if (Tetra->isOverhangTet)
				weight = m_criticalTet_weight_SL;

			for (int i = 0; i < 4; i++) {
				matrix_A_4x.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](0, i); // x row
				matrix_A_4y.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](1, i); // y row
				matrix_A_4z.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](2, i); // z row
			}
		}
		// Block 21 equals to zero
		// Block 22 for x,y,z
		int constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			constraint_ind++;
		}
		// Block 31 equals to zero
		// Block 32 for x,y,z
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			matrix_A_4x.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4y.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4z.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;

			constraint_ind++;
		}

		std::cout << "Finish fill A matrix." << std::endl;

		long time = clock();

		// Factorize A matrix
		matrix_A_4x.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4x(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4x = matrix_A_4x.transpose();
		matATA_4x = matrix_A_transpose_4x * matrix_A_4x;
		Solver_ASAP_4x.compute(matATA_4x);

		matrix_A_4y.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4y(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4y = matrix_A_4y.transpose();
		matATA_4y = matrix_A_transpose_4y * matrix_A_4y;
		Solver_ASAP_4y.compute(matATA_4y);

		matrix_A_4z.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4z(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4z = matrix_A_4z.transpose();
		matATA_4z = matrix_A_transpose_4z * matrix_A_4z;
		Solver_ASAP_4z.compute(matATA_4z);

		std::printf("Finish factorize materix A\n");

		std::printf("The compress spends %f s.\n", (double(clock() - time)) / CLOCKS_PER_SEC);

		// Fill b vector for x,y,z
		//special case of regular term
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			for (int i = 0; i < 3; i++) {

				vector_b[i](4 * tet_Num + neighborScale_face_num + constraint_ind) = m_regularScale_weight;
			}
			constraint_ind++;
		}

		// Solve x vector
		Eigen::VectorXd ATb_4x = matrix_A_transpose_4x * vector_b[0];
		vector_X_Pos_Scale[0] = Solver_ASAP_4x.solve(ATb_4x);

		Eigen::VectorXd ATb_4y = matrix_A_transpose_4y * vector_b[1];
		vector_X_Pos_Scale[1] = Solver_ASAP_4y.solve(ATb_4y);

		Eigen::VectorXd ATb_4z = matrix_A_transpose_4z * vector_b[2];
		vector_X_Pos_Scale[2] = Solver_ASAP_4z.solve(ATb_4z);

		// Update vertex
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
			int idx = node->GetIndexNo();

			double xx, yy, zz;
			node->SetCoord3D(vector_X_Pos_Scale[0](idx), vector_X_Pos_Scale[1](idx), vector_X_Pos_Scale[2](idx));
			node->GetCoord3D(xx, yy, zz);
			//std::printf("New coordinate xx %f , yy %f , zz %f.\n", xx, yy, zz);
		}

		// record the scale value
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			for (int i = 0; i < 3; i++) {
				Tetra->scaleValue_vector[i] = vector_X_Pos_Scale[i](node_Num + fdx);
				//std::cout << vector_X_Pos_Scale[i](node_Num + fdx) << " ";
			}
			//std::cout << std::endl;
		}

		this->_cal_virtual_Vfield();
		std::printf("End calculation of loop %d.\n\n", loop);
	}

	// detact and draw overhang face
	this->m_supportFreeAngle = 45.0;
	this->_detectOverhangFace();
	tetPatch->drawOverhangSurface = true;
}

//less change for initial guess
void DeformTet::runASAP_SupportLess_test(bool initialGuess) {

	//---------------------------------------------------------------------------------------------------------------
	// Initialize size of nodes, faces and tets
	int node_Num = tetPatch->GetNodeNumber();
	int tet_Num = tetPatch->GetTetraNumber();
	int face_Num = tetPatch->GetFaceNumber();

	int neighborScale_face_num = 0;
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL)	neighborScale_face_num++;
	}

	std::printf("All face num %d , neighborScale_face_num %d.\n", face_Num, neighborScale_face_num);
	std::printf("A matrix sixe %d x %d\n", (5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));

	//---------------------------------------------------------------------------------------------------------------
	// Define variables
	std::vector<Eigen::MatrixXd> frame_Local_new, frame_Local_initial_inverse, frame_Local_initial;
	std::vector<Eigen::VectorXd> vector_X_Pos_Scale, vector_b;
	// Initialize variables
	frame_Local_new.resize(tet_Num);
	frame_Local_initial_inverse.resize(tet_Num);
	frame_Local_initial.resize(tet_Num);
	vector_X_Pos_Scale.resize(3);
	vector_b.resize(3);

	for (int i = 0; i < 3; i++) {
		vector_X_Pos_Scale[i] = Eigen::VectorXd::Zero(node_Num + tet_Num);
		vector_b[i] = Eigen::VectorXd::Zero(5 * tet_Num + neighborScale_face_num);
	}
	std::cout << "Finish initializing ASAP matrices" << std::endl;

	//---------------------------------------------------------------------------------------------------------------
	// Pre-compute the initial frame and inverse of initial frame
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tet->GetIndexNo();
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
		double center[3] = { 0 };
		QMeshNode* nodes[4];

		for (int i = 0; i < 4; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= 4;
		for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		frame_Local_initial[fdx] = P;
		frame_Local_initial_inverse[fdx] = Eigen::MatrixXd::Zero(3, 4);
		frame_Local_initial_inverse[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();
		//std::cout << frame_Local_initial[fdx] << std::endl << std::endl;
	}
	std::printf("Finish computing local frame and its inverse!\n\n");

	//this->_detectOverhangFace();
	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	for (int loop = 0; loop < m_loopTime; loop++) {

		//if (loop % 5 == 0)	this->_detectOverhangFace();
		this->_detectOverhangFace();

		//Calculate the fabrication energy of SupportLess case ()
		this->_calFabricationEnergy_SupportLess();

		for (int innerLoop = 0; innerLoop < 1; innerLoop++) {

#pragma omp parallel
			{
#pragma omp for 
				// local rotation and scaling operation (frame_Local_new)
				for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
					QMeshTetra* Tetra = tetraPatch_elementSet[i];

					int fdx = Tetra->GetIndexNo();
					if (innerLoop == 0) {
						double center[3] = { 0.0 };
						Tetra->CalCenterPos(center[0], center[1], center[2]);

						//This tP is each current frame in each tet.
						Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
						QMeshNode* nodes[4];
						for (int i = 0; i < 4; i++) {
							nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
							nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
						}
						for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

						Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

						T = frame_Local_initial_inverse[fdx] * tP;
						T_transpose = T.transpose();

						///// R1 //// Eigen SVD decomposition /////
						Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
						Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
						R = U * V.transpose();
						Tetra->R_estimate = R;
						frame_Local_new[fdx] = R * frame_Local_initial[fdx];
					}

					///// R2.1 //// Fabrication requirement (voxel guess) /////
					if ((loop == 0) && (initialGuess)) {

						Eigen::Vector3d voxelOrder_Dir = Tetra->R_estimate * Tetra->vectorField_4voxelOrder;

						Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();

						//rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(voxelOrder_Dir, printDir);

						/// <summary>
						/// release the initial rotation constraints
						/// </summary>
						/// <param name="rotation angle 45deg"></param>	
						double threshold_angle = 30.0; //30 is for heat method case
						Eigen::Vector3d voxelOrder_Dir_normalized = voxelOrder_Dir.normalized();
						double rad_angle = acos(printDir.dot(voxelOrder_Dir_normalized));
						if (ROTATE_TO_DEGREE(rad_angle) > threshold_angle) {
							Eigen::Vector3d r_axis = this->printDir.cross(voxelOrder_Dir_normalized);
							r_axis.normalize();
							Eigen::AngleAxis<double> sptFree_rotation = Eigen::AngleAxis<double>(threshold_angle * PI / 180.0, r_axis);
							Eigen::Vector3d targetDir = sptFree_rotation.matrix() * this->printDir;
							rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(voxelOrder_Dir, targetDir);
						}
						/// end

						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					}

					///// R2.2 //// Fabrication requirement (support less) /////
					if (Tetra->isOverhangTet) {

						if ((loop == 0) && initialGuess) continue;

						QMeshFace* eachFace = NULL;		int overhangFace_Num = 0;
						Eigen::Vector3d whole_overhang_face_normal = Eigen::Vector3d::Zero();
						for (int i = 0; i < 4; i++) {
							eachFace = Tetra->GetFaceRecordPtr(i + 1);
							if (eachFace->isOverhangFace) {

								Eigen::Vector3d normal;
								eachFace->CalPlaneEquation();
								eachFace->GetNormal(normal(0), normal(1), normal(2));
								normal = -normal;
								normal.normalize();
								whole_overhang_face_normal += normal;
								overhangFace_Num++;
							}
						}
						if (overhangFace_Num == 0) std::cout << "ERROR: none of overhang face is detected!" << std::endl;
						whole_overhang_face_normal.normalize();

						Eigen::Vector3d r_axis = this->printDir.cross(whole_overhang_face_normal);
						// CAUSION:!!!!!!!
						// cross product range is [0,180] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
						// so
						// angle <1,normal> (<) angle <2,nomal>
						r_axis.normalize();// OMG!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
						Eigen::AngleAxis<double> sptFree_rotation
							= Eigen::AngleAxis<double>((this->m_supportFreeAngle + 90.0) * PI / 180.0, r_axis);
						Eigen::Vector3d sptFree_normal = sptFree_rotation.matrix() * this->printDir;

						Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
						rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(whole_overhang_face_normal, sptFree_normal);

						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					}
				}
			}
			std::printf("Finish new local frame calculation.\n");

			bool is_Smooth = true;
			if (is_Smooth) {
				bool concavity_smooth = false;
				if (concavity_smooth) {
					if (loop == 0)
						this->_globalQuaternionSmooth2();
					else
						this->_globalQuaternionSmooth3();
				}
				else {
					//this->_globalQuaternionSmooth2(); // all
					this->_globalQuaternionSmooth1(); //only critical
				}

				for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
					QMeshTetra* Tetra = tetraPatch_elementSet[i];
					frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
				}
				std::printf("Finish quaternion smooth calculation.\n");
			}
		}

		// Define variables
		Eigen::SparseMatrix<double> matrix_A_4x;
		Eigen::SparseMatrix<double> matrix_A_transpose_4x;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4x;
		//
		Eigen::SparseMatrix<double> matrix_A_4y;
		Eigen::SparseMatrix<double> matrix_A_transpose_4y;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4y;
		//
		Eigen::SparseMatrix<double> matrix_A_4z;
		Eigen::SparseMatrix<double> matrix_A_transpose_4z;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4z;
		// Initialize variables
		matrix_A_4x.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4x.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4y.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4y.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4z.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4z.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));

		// Fill A matrix for x,y,z
		// Block 11 for x,y,z
		// Important issues
		//give memory to sparse matrix, to accerate the insert speed
		matrix_A_4x.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4y.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4z.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));

		float c1 = -0.25, c2 = 0.75;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int fdx = Tetra->GetIndexNo() * 4;	int vdxArr[4];
			for (int i = 0; i < 4; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

			double weight = 1.0;
			if (Tetra->isOverhangTet)
				weight = m_criticalTet_weight_SL;

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (i == j) {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c2 * weight;
					}
					else {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c1 * weight;
					}
				}
			}
		}

		// Block 12 for x,y,z
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			double weight = 1.0;
			if (Tetra->isOverhangTet)
				weight = m_criticalTet_weight_SL;

			for (int i = 0; i < 4; i++) {
				matrix_A_4x.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](0, i); // x row
				matrix_A_4y.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](1, i); // y row
				matrix_A_4z.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](2, i); // z row
			}
		}
		// Block 21 equals to zero
		// Block 22 for x,y,z
		int constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			constraint_ind++;
		}
		// Block 31 equals to zero
		// Block 32 for x,y,z
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			matrix_A_4x.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4y.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4z.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;

			constraint_ind++;
		}

		std::cout << "Finish fill A matrix." << std::endl;

		long time = clock();

		// Factorize A matrix
		matrix_A_4x.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4x(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4x = matrix_A_4x.transpose();
		matATA_4x = matrix_A_transpose_4x * matrix_A_4x;
		Solver_ASAP_4x.compute(matATA_4x);

		matrix_A_4y.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4y(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4y = matrix_A_4y.transpose();
		matATA_4y = matrix_A_transpose_4y * matrix_A_4y;
		Solver_ASAP_4y.compute(matATA_4y);

		matrix_A_4z.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4z(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4z = matrix_A_4z.transpose();
		matATA_4z = matrix_A_transpose_4z * matrix_A_4z;
		Solver_ASAP_4z.compute(matATA_4z);

		std::printf("Finish factorize materix A\n");

		std::printf("The compress spends %f s.\n", (double(clock() - time)) / CLOCKS_PER_SEC);

		// Fill b vector for x,y,z
		//special case of regular term
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			for (int i = 0; i < 3; i++) {

				vector_b[i](4 * tet_Num + neighborScale_face_num + constraint_ind) = m_regularScale_weight;
			}
			constraint_ind++;
		}

		// Solve x vector
		Eigen::VectorXd ATb_4x = matrix_A_transpose_4x * vector_b[0];
		vector_X_Pos_Scale[0] = Solver_ASAP_4x.solve(ATb_4x);

		Eigen::VectorXd ATb_4y = matrix_A_transpose_4y * vector_b[1];
		vector_X_Pos_Scale[1] = Solver_ASAP_4y.solve(ATb_4y);

		Eigen::VectorXd ATb_4z = matrix_A_transpose_4z * vector_b[2];
		vector_X_Pos_Scale[2] = Solver_ASAP_4z.solve(ATb_4z);

		// Update vertex
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
			int idx = node->GetIndexNo();

			double xx, yy, zz;
			node->SetCoord3D(vector_X_Pos_Scale[0](idx), vector_X_Pos_Scale[1](idx), vector_X_Pos_Scale[2](idx));
			node->GetCoord3D(xx, yy, zz);
			//std::printf("New coordinate xx %f , yy %f , zz %f.\n", xx, yy, zz);
		}

		// record the scale value
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			for (int i = 0; i < 3; i++) {
				Tetra->scaleValue_vector[i] = vector_X_Pos_Scale[i](node_Num + fdx);
				//std::cout << vector_X_Pos_Scale[i](node_Num + fdx) << " ";
			}
			//std::cout << std::endl;
		}

		this->_cal_virtual_Vfield();
		std::printf("End calculation of loop %d.\n\n", loop);
	}

	// detact and draw overhang face
	this->m_supportFreeAngle = 45.0;
	this->_detectOverhangFace();
	tetPatch->drawOverhangSurface = true;
}

//heat method only for rotation direction decision
void DeformTet::runASAP_SupportLess_test2(bool initialGuess) {

	//---------------------------------------------------------------------------------------------------------------
	// Initialize size of nodes, faces and tets
	int node_Num = tetPatch->GetNodeNumber();
	int tet_Num = tetPatch->GetTetraNumber();
	int face_Num = tetPatch->GetFaceNumber();

	int neighborScale_face_num = 0;
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL)	neighborScale_face_num++;
	}

	std::printf("All face num %d , neighborScale_face_num %d.\n", face_Num, neighborScale_face_num);
	std::printf("A matrix sixe %d x %d\n", (5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));

	//---------------------------------------------------------------------------------------------------------------
	// Define variables
	std::vector<Eigen::MatrixXd> frame_Local_new, frame_Local_initial_inverse, frame_Local_initial;
	std::vector<Eigen::VectorXd> vector_X_Pos_Scale, vector_b;
	// Initialize variables
	frame_Local_new.resize(tet_Num);
	frame_Local_initial_inverse.resize(tet_Num);
	frame_Local_initial.resize(tet_Num);
	vector_X_Pos_Scale.resize(3);
	vector_b.resize(3);

	for (int i = 0; i < 3; i++) {
		vector_X_Pos_Scale[i] = Eigen::VectorXd::Zero(node_Num + tet_Num);
		vector_b[i] = Eigen::VectorXd::Zero(5 * tet_Num + neighborScale_face_num);
	}
	std::cout << "Finish initializing ASAP matrices" << std::endl;

	//---------------------------------------------------------------------------------------------------------------
	// Pre-compute the initial frame and inverse of initial frame
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tet->GetIndexNo();
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
		double center[3] = { 0 };
		QMeshNode* nodes[4];

		for (int i = 0; i < 4; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= 4;
		for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		frame_Local_initial[fdx] = P;
		frame_Local_initial_inverse[fdx] = Eigen::MatrixXd::Zero(3, 4);
		frame_Local_initial_inverse[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();
		//std::cout << frame_Local_initial[fdx] << std::endl << std::endl;
	}
	std::printf("Finish computing local frame and its inverse!\n\n");

	//this->_detectOverhangFace();
	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	for (int loop = 0; loop < m_loopTime; loop++) {

		//if (loop % 5 == 0)	this->_detectOverhangFace();
		this->_detectOverhangFace();

		//Calculate the fabrication energy of SupportLess case ()
		this->_calFabricationEnergy_SupportLess();

		for (int innerLoop = 0; innerLoop < m_innerLoopTime; innerLoop++) {

#pragma omp parallel
			{
#pragma omp for 
				// local rotation and scaling operation (frame_Local_new)
				for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
					QMeshTetra* Tetra = tetraPatch_elementSet[i];

					int fdx = Tetra->GetIndexNo();
					if (innerLoop == 0) {
						double center[3] = { 0.0 };
						Tetra->CalCenterPos(center[0], center[1], center[2]);

						//This tP is each current frame in each tet.
						Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
						QMeshNode* nodes[4];
						for (int i = 0; i < 4; i++) {
							nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
							nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
						}
						for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

						Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

						T = frame_Local_initial_inverse[fdx] * tP;
						T_transpose = T.transpose();

						///// R1 //// Eigen SVD decomposition /////
						Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
						Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
						R = U * V.transpose();
						Tetra->R_estimate = R;
						frame_Local_new[fdx] = R * frame_Local_initial[fdx];
						Tetra->vectorField_4voxelOrder_now = Tetra->R_estimate * Tetra->vectorField_4voxelOrder;
					}

					///// R2 //// Fabrication requirement (support less) /////
					if (Tetra->isOverhangTet) {

						// get current face normal
						QMeshFace* eachFace = NULL;		int overhangFace_Num = 0;
						Eigen::Vector3d whole_overhang_face_normal = Eigen::Vector3d::Zero();
						for (int i = 0; i < 4; i++) {
							eachFace = Tetra->GetFaceRecordPtr(i + 1);
							if (eachFace->isOverhangFace) {

								Eigen::Vector3d normal;
								//eachFace->CalPlaneEquation();
								//eachFace->GetNormal(normal(0), normal(1), normal(2));
								normal = eachFace->normal_last; // initial face normal
								normal = -normal;
								normal.normalize();
								whole_overhang_face_normal += normal;
								overhangFace_Num++;
							}
						}
						if (overhangFace_Num == 0) std::cout << "ERROR: none of overhang face is detected!" << std::endl;
						whole_overhang_face_normal.normalize();

						// virtual face normal after applying R_estimate
						Eigen::Vector3d face_Normal_Dir = Tetra->R_estimate * whole_overhang_face_normal;
						Tetra->vectorField_4voxelOrder_now = Tetra->R_estimate * Tetra->vectorField_4voxelOrder;
						
						// CAUSION:!!!!!!!
						// cross product range is [0,180] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
						// so
						// angle <1,normal> (<) angle <2,nomal>
						Eigen::Vector3d r_axis = this->printDir.cross(Tetra->vectorField_4voxelOrder_now.normalized());
						r_axis.normalize();// OMG!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
						Eigen::AngleAxis<double> sptFree_rotation_1
							= Eigen::AngleAxis<double>((this->m_supportFreeAngle + 90.0) * PI / 180.0, r_axis);
						Eigen::Vector3d sptFree_normal_1 = sptFree_rotation_1.matrix() * this->printDir;
						Eigen::AngleAxis<double> sptFree_rotation_2
							= Eigen::AngleAxis<double>(-1.0 * (this->m_supportFreeAngle + 90.0) * PI / 180.0, r_axis);
						Eigen::Vector3d sptFree_normal_2 = sptFree_rotation_2.matrix() * this->printDir;

						Eigen::Vector3d sptFree_normal = Eigen::Vector3d::Zero();
						double angel_n_nCandidate1 = acos(sptFree_normal_1.dot(Tetra->vectorField_4voxelOrder_now.normalized()));
						double angel_n_nCandidate2 = acos(sptFree_normal_2.dot(Tetra->vectorField_4voxelOrder_now.normalized()));

						if (angel_n_nCandidate1 < angel_n_nCandidate2) {sptFree_normal = sptFree_normal_1;}
						else {sptFree_normal = sptFree_normal_2;}
						sptFree_normal = sptFree_normal_1;

						Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
						rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(face_Normal_Dir, sptFree_normal);

						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;

						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					}
				}
			}
			std::printf("Finish new local frame calculation.\n");

			bool is_Smooth = true;
			if (is_Smooth) {
				bool concavity_smooth = true;
				if (concavity_smooth) {
					if (loop == 0)
						this->_globalQuaternionSmooth2();
					else
						this->_globalQuaternionSmooth3();
				}
				else {
					//this->_globalQuaternionSmooth2(); // all
					this->_globalQuaternionSmooth1(); //only critical
				}

				for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
					QMeshTetra* Tetra = tetraPatch_elementSet[i];
					frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
				}
				std::printf("Finish quaternion smooth calculation.\n");
			}

			this->_get_energy_innerLoop_supportLess();
		}

		// Define variables
		Eigen::SparseMatrix<double> matrix_A_4x;
		Eigen::SparseMatrix<double> matrix_A_transpose_4x;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4x;
		//
		Eigen::SparseMatrix<double> matrix_A_4y;
		Eigen::SparseMatrix<double> matrix_A_transpose_4y;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4y;
		//
		Eigen::SparseMatrix<double> matrix_A_4z;
		Eigen::SparseMatrix<double> matrix_A_transpose_4z;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4z;
		// Initialize variables
		matrix_A_4x.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4x.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4y.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4y.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4z.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4z.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));

		// Fill A matrix for x,y,z
		// Block 11 for x,y,z
		// Important issues
		//give memory to sparse matrix, to accerate the insert speed
		matrix_A_4x.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4y.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4z.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));

		float c1 = -0.25, c2 = 0.75;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int fdx = Tetra->GetIndexNo() * 4;	int vdxArr[4];
			for (int i = 0; i < 4; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

			double weight = 1.0;
			if (Tetra->isOverhangTet)
				weight = m_criticalTet_weight_SL;

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (i == j) {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c2 * weight;
					}
					else {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c1 * weight;
					}
				}
			}
		}

		// Block 12 for x,y,z
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			double weight = 1.0;
			if (Tetra->isOverhangTet)
				weight = m_criticalTet_weight_SL;

			for (int i = 0; i < 4; i++) {
				matrix_A_4x.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](0, i); // x row
				matrix_A_4y.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](1, i); // y row
				matrix_A_4z.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](2, i); // z row
			}
		}
		// Block 21 equals to zero
		// Block 22 for x,y,z
		int constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			constraint_ind++;
		}
		// Block 31 equals to zero
		// Block 32 for x,y,z
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			matrix_A_4x.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4y.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4z.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;

			constraint_ind++;
		}

		std::cout << "Finish fill A matrix." << std::endl;

		long time = clock();

		// Factorize A matrix
		matrix_A_4x.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4x(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4x = matrix_A_4x.transpose();
		matATA_4x = matrix_A_transpose_4x * matrix_A_4x;
		Solver_ASAP_4x.compute(matATA_4x);

		matrix_A_4y.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4y(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4y = matrix_A_4y.transpose();
		matATA_4y = matrix_A_transpose_4y * matrix_A_4y;
		Solver_ASAP_4y.compute(matATA_4y);

		matrix_A_4z.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4z(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4z = matrix_A_4z.transpose();
		matATA_4z = matrix_A_transpose_4z * matrix_A_4z;
		Solver_ASAP_4z.compute(matATA_4z);

		std::printf("Finish factorize materix A\n");

		std::printf("The compress spends %f s.\n", (double(clock() - time)) / CLOCKS_PER_SEC);

		// Fill b vector for x,y,z
		//special case of regular term
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			for (int i = 0; i < 3; i++) {

				vector_b[i](4 * tet_Num + neighborScale_face_num + constraint_ind) = m_regularScale_weight;
			}
			constraint_ind++;
		}

		// Solve x vector
		Eigen::VectorXd ATb_4x = matrix_A_transpose_4x * vector_b[0];
		vector_X_Pos_Scale[0] = Solver_ASAP_4x.solve(ATb_4x);

		Eigen::VectorXd ATb_4y = matrix_A_transpose_4y * vector_b[1];
		vector_X_Pos_Scale[1] = Solver_ASAP_4y.solve(ATb_4y);

		Eigen::VectorXd ATb_4z = matrix_A_transpose_4z * vector_b[2];
		vector_X_Pos_Scale[2] = Solver_ASAP_4z.solve(ATb_4z);

		// Update vertex
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
			int idx = node->GetIndexNo();

			double xx, yy, zz;
			node->SetCoord3D(vector_X_Pos_Scale[0](idx), vector_X_Pos_Scale[1](idx), vector_X_Pos_Scale[2](idx));
			node->GetCoord3D(xx, yy, zz);
			//std::printf("New coordinate xx %f , yy %f , zz %f.\n", xx, yy, zz);
		}

		// record the scale value
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			for (int i = 0; i < 3; i++) {
				Tetra->scaleValue_vector[i] = vector_X_Pos_Scale[i](node_Num + fdx);
				//std::cout << vector_X_Pos_Scale[i](node_Num + fdx) << " ";
			}
			//std::cout << std::endl;
		}

		this->_cal_virtual_Vfield();
		std::printf("End calculation of loop %d.\n\n", loop);
	}

	// detact and draw overhang face
	this->m_supportFreeAngle = 45.0;
	this->_detectOverhangFace();
	tetPatch->drawOverhangSurface = true;
}

void DeformTet::runASAP_SupportLess_test3() {

	//---------------------------------------------------------------------------------------------------------------
	// Initialize size of nodes, faces and tets
	int node_Num = tetPatch->GetNodeNumber();
	int tet_Num = tetPatch->GetTetraNumber();
	int face_Num = tetPatch->GetFaceNumber();

	int neighborScale_face_num = 0;
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL)	neighborScale_face_num++;
	}

	std::printf("All face num %d , neighborScale_face_num %d.\n", face_Num, neighborScale_face_num);
	std::printf("A matrix sixe %d x %d\n", (5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));

	//---------------------------------------------------------------------------------------------------------------
	// Define variables
	std::vector<Eigen::MatrixXd> frame_Local_new, frame_Local_initial_inverse, frame_Local_initial;
	std::vector<Eigen::VectorXd> vector_X_Pos_Scale, vector_b;
	// Initialize variables
	frame_Local_new.resize(tet_Num);
	frame_Local_initial_inverse.resize(tet_Num);
	frame_Local_initial.resize(tet_Num);
	vector_X_Pos_Scale.resize(3);
	vector_b.resize(3);

	for (int i = 0; i < 3; i++) {
		vector_X_Pos_Scale[i] = Eigen::VectorXd::Zero(node_Num + tet_Num);
		vector_b[i] = Eigen::VectorXd::Zero(5 * tet_Num + neighborScale_face_num);
	}
	std::cout << "Finish initializing ASAP matrices" << std::endl;

	//---------------------------------------------------------------------------------------------------------------
	// Pre-compute the initial frame and inverse of initial frame
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tet->GetIndexNo();
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
		double center[3] = { 0 };
		QMeshNode* nodes[4];

		for (int i = 0; i < 4; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= 4;
		for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		frame_Local_initial[fdx] = P;
		frame_Local_initial_inverse[fdx] = Eigen::MatrixXd::Zero(3, 4);
		frame_Local_initial_inverse[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();
		//std::cout << frame_Local_initial[fdx] << std::endl << std::endl;
	}
	std::printf("Finish computing local frame and its inverse!\n\n");

	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	for (int loop = 0; loop < m_loopTime; loop++) {

		this->_detectOverhangFace();

		//Calculate the fabrication energy of SupportLess case ()
		this->_calFabricationEnergy_SupportLess();

		for (int innerLoop = 0; innerLoop < m_innerLoopTime; innerLoop++) {

#pragma omp parallel
			{
#pragma omp for 
				// local rotation and scaling operation (frame_Local_new)
				for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
					QMeshTetra* Tetra = tetraPatch_elementSet[i];

					int fdx = Tetra->GetIndexNo();
					if (innerLoop == 0) {
						double center[3] = { 0.0 };
						Tetra->CalCenterPos(center[0], center[1], center[2]);

						//This tP is each current frame in each tet.
						Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
						QMeshNode* nodes[4];
						for (int i = 0; i < 4; i++) {
							nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
							nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
						}
						for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

						Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

						T = frame_Local_initial_inverse[fdx] * tP;
						T_transpose = T.transpose();

						///// R1 //// Eigen SVD decomposition /////
						Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
						Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
						R = U * V.transpose();
						Tetra->R_estimate = R;
						frame_Local_new[fdx] = R * frame_Local_initial[fdx];
						Tetra->vectorField_4voxelOrder_now = Tetra->R_estimate * Tetra->vectorField_4voxelOrder;
					}

					///// R2 //// Fabrication requirement (support less) /////
					if (Tetra->isOverhangTet) {

						// get current face normal
						Eigen::Vector3d whole_overhang_face_normal = this->_get_initial_overhang_faceNormal(Tetra);
						
						// virtual face normal after applying R_estimate
						Eigen::Vector3d face_Normal_Dir = Tetra->R_estimate * whole_overhang_face_normal;
						Tetra->vectorField_4voxelOrder_now = Tetra->R_estimate * Tetra->vectorField_4voxelOrder;

						Eigen::Matrix3d rotationMatrix = _cal_rotationMatrix_supportLess(face_Normal_Dir, Tetra);

						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					}
				}
			}
			std::printf("Finish new local frame calculation.\n");

			this->_globalQuaternionSmooth1_supportLess();
				
			for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
				QMeshTetra* Tetra = tetraPatch_elementSet[i];
				frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
			}
			std::printf("Finish quaternion smooth calculation.\n");
			
			this->_get_energy_innerLoop_supportLess();
		}

		// Define variables
		Eigen::SparseMatrix<double> matrix_A_4x;
		Eigen::SparseMatrix<double> matrix_A_transpose_4x;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4x;
		//
		Eigen::SparseMatrix<double> matrix_A_4y;
		Eigen::SparseMatrix<double> matrix_A_transpose_4y;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4y;
		//
		Eigen::SparseMatrix<double> matrix_A_4z;
		Eigen::SparseMatrix<double> matrix_A_transpose_4z;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4z;
		// Initialize variables
		matrix_A_4x.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4x.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4y.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4y.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4z.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4z.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));

		// Fill A matrix for x,y,z
		// Block 11 for x,y,z
		// Important issues
		//give memory to sparse matrix, to accerate the insert speed
		matrix_A_4x.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4y.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4z.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));

		float c1 = -0.25, c2 = 0.75;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int fdx = Tetra->GetIndexNo() * 4;	int vdxArr[4];
			for (int i = 0; i < 4; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

			double weight = 1.0;
			if (Tetra->isOverhangTet)	weight = m_criticalTet_weight_SL;

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (i == j) {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c2 * weight;
					}
					else {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c1 * weight;
					}
				}
			}
		}

		// Block 12 for x,y,z
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			double weight = 1.0;
			if (Tetra->isOverhangTet)	weight = m_criticalTet_weight_SL;

			for (int i = 0; i < 4; i++) {
				matrix_A_4x.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](0, i); // x row
				matrix_A_4y.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](1, i); // y row
				matrix_A_4z.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](2, i); // z row
			}
		}
		// Block 21 equals to zero
		// Block 22 for x,y,z
		int constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			constraint_ind++;
		}
		// Block 31 equals to zero
		// Block 32 for x,y,z
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			matrix_A_4x.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4y.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4z.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;

			constraint_ind++;
		}

		std::cout << "Finish fill A matrix." << std::endl;

		long time = clock();

		// Factorize A matrix
		matrix_A_4x.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4x(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4x = matrix_A_4x.transpose();
		matATA_4x = matrix_A_transpose_4x * matrix_A_4x;
		Solver_ASAP_4x.compute(matATA_4x);

		matrix_A_4y.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4y(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4y = matrix_A_4y.transpose();
		matATA_4y = matrix_A_transpose_4y * matrix_A_4y;
		Solver_ASAP_4y.compute(matATA_4y);

		matrix_A_4z.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4z(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4z = matrix_A_4z.transpose();
		matATA_4z = matrix_A_transpose_4z * matrix_A_4z;
		Solver_ASAP_4z.compute(matATA_4z);

		std::printf("Finish factorize materix A\n");

		std::printf("The compress spends %f s.\n", (double(clock() - time)) / CLOCKS_PER_SEC);

		// Fill b vector for x,y,z
		//special case of regular term
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			for (int i = 0; i < 3; i++) {

				vector_b[i](4 * tet_Num + neighborScale_face_num + constraint_ind) = m_regularScale_weight;
			}
			constraint_ind++;
		}

		// Solve x vector
		Eigen::VectorXd ATb_4x = matrix_A_transpose_4x * vector_b[0];
		vector_X_Pos_Scale[0] = Solver_ASAP_4x.solve(ATb_4x);

		Eigen::VectorXd ATb_4y = matrix_A_transpose_4y * vector_b[1];
		vector_X_Pos_Scale[1] = Solver_ASAP_4y.solve(ATb_4y);

		Eigen::VectorXd ATb_4z = matrix_A_transpose_4z * vector_b[2];
		vector_X_Pos_Scale[2] = Solver_ASAP_4z.solve(ATb_4z);

		// Update vertex
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
			int idx = node->GetIndexNo();

			double xx, yy, zz;
			node->SetCoord3D(vector_X_Pos_Scale[0](idx), vector_X_Pos_Scale[1](idx), vector_X_Pos_Scale[2](idx));
			node->GetCoord3D(xx, yy, zz);
			//std::printf("New coordinate xx %f , yy %f , zz %f.\n", xx, yy, zz);
		}

		// record the scale value
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			for (int i = 0; i < 3; i++) {
				Tetra->scaleValue_vector[i] = vector_X_Pos_Scale[i](node_Num + fdx);
				//std::cout << vector_X_Pos_Scale[i](node_Num + fdx) << " ";
			}
			//std::cout << std::endl;
		}

		this->_cal_virtual_Vfield();
		std::printf("End calculation of loop %d.\n\n", loop);
	}

	// detact and draw overhang face
	this->m_supportFreeAngle = 45.0;
	this->_detectOverhangFace();
	tetPatch->drawOverhangSurface = true;
}

// all overhang face of one tet element are considered
void DeformTet::_detectOverhangFace() {

	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		face->isOverhangFace = false;
		if (face->GetLeftTetra() == NULL || face->GetRightTetra() == NULL) {
			Eigen::Vector3d normal;
			face->CalPlaneEquation();
			face->GetNormal(normal(0), normal(1), normal(2));
			normal.normalize();
			double x = m_supportFreeAngle * 3.1415926 / 180;

			if (-normal.dot(printDir) < -sin(x))
				face->isOverhangFace = true;
		}
	}

	for (GLKPOSITION pos = tetPatch->GetTetraList().GetHeadPosition(); pos != nullptr;) {
		QMeshTetra* tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(pos);
		// clear the last overhangTet flag, if uncomment it, code will add more overhang tet into before set
		tetra->isOverhangTet = false;
		int criticalFaceNum = 0;
		for (int i = 0; i < 4; i++) {
			if (tetra->GetFaceRecordPtr(i + 1)->isOverhangFace) criticalFaceNum++;
		}
		if (criticalFaceNum > 0) tetra->isOverhangTet = true;
		if (criticalFaceNum > 3) {
			std::cout << "Error: critical tet with more than 3 critical face, please check!" << std::endl;}
	}

	// special case of bottom faces at SupportLess case
	for (GLKPOSITION pos = tetPatch->GetTetraList().GetHeadPosition(); pos != nullptr;) {
		QMeshTetra* tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(pos);
		if (tetra->isBottomTet) {
			for (int i = 0; i < 4; i++) {
				tetra->GetFaceRecordPtr(i + 1)->isOverhangFace = false;
			}
			tetra->isOverhangTet = false;
		}
	}

	std::cout << "Detect overhang face once, and find overhang Tet once, dectection angle is " 
		<< this->m_supportFreeAngle << std::endl;
}

// only the element with one boundary face are considered as overhang tet
void DeformTet::_detectOverhangFace2() {

	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		face->isOverhangFace = false;
		if (face->inner == false) {
			Eigen::Vector3d normal;
			face->CalPlaneEquation();
			face->GetNormal(normal(0), normal(1), normal(2));
			normal.normalize();
			double x = m_supportFreeAngle * 3.1415926 / 180;

			if (-normal.dot(printDir) < -sin(x))
				face->isOverhangFace = true;
		}
	}

	for (GLKPOSITION pos = tetPatch->GetTetraList().GetHeadPosition(); pos != nullptr;) {
		QMeshTetra* tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(pos);
		// clear the last overhangTet flag, if uncomment it, code will add more overhang tet into before set
		tetra->isOverhangTet = false;

		// only consider the tet element with one boundary face
		double boundaryFace_num = 0;
		for (int i = 0; i < 4; i++) {
			if (tetra->GetFaceRecordPtr(i + 1)->inner == false) boundaryFace_num++;
		}

		if (boundaryFace_num != 1) continue;

		// mark the overhang tet element
		int criticalFaceNum = 0;
		for (int i = 0; i < 4; i++) {
			if (tetra->GetFaceRecordPtr(i + 1)->isOverhangFace) criticalFaceNum++;
		}
		if (criticalFaceNum == 1)	tetra->isOverhangTet = true;
		else if (criticalFaceNum > 1) {
			std::cout << "Error: overhang faces num should be one( one overhang face) or zero( none overhang face), please check!" << std::endl;
		}
		
	}

	// special case of bottom faces at SupportLess case
	for (GLKPOSITION pos = tetPatch->GetTetraList().GetHeadPosition(); pos != nullptr;) {
		QMeshTetra* tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(pos);
		if (tetra->isBottomTet) {
			for (int i = 0; i < 4; i++) {
				tetra->GetFaceRecordPtr(i + 1)->isOverhangFace = false;
			}
			tetra->isOverhangTet = false;
		}
	}

	// check overhang faces are all boundary face
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		if (face->isOverhangFace && face->inner)
			std::cout << "Error: overhang face could only be boundary face.\n";
	}

	std::cout << "Detect overhang faces and find overhang Tet, dectection angle is "
		<< this->m_supportFreeAngle << std::endl;
}

void DeformTet::_calFabricationEnergy_SupportLess() {

	//Get the fabrication energy of supportLess
	double critical_energy = 0.0;
	double safeAngle = m_supportFreeAngle * PI / 180.0 + PI / 2.0;
	//std::cout << "Safe Angle = " << ROTATE_TO_DEGREE(safeAngle) << std::endl;
	for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[i];

		if (Tetra->isOverhangTet) {
			QMeshFace* eachFace = NULL;		int overhangFace_Num = 0;
			Eigen::Vector3d whole_overhang_face_normal = Eigen::Vector3d::Zero();
			for (int i = 0; i < 4; i++) {
				eachFace = Tetra->GetFaceRecordPtr(i + 1);
				if (eachFace->isOverhangFace) {

					Eigen::Vector3d normal;
					eachFace->CalPlaneEquation();
					eachFace->GetNormal(normal(0), normal(1), normal(2));
					normal = -normal;
					normal.normalize();
					whole_overhang_face_normal += normal;
					overhangFace_Num++;
				}
			}
			if (overhangFace_Num == 0) std::cout << "ERROR: none of overhang face is detected!" << std::endl;
			whole_overhang_face_normal.normalize();
			double currentAngle = acos(whole_overhang_face_normal.dot(printDir));

			if (currentAngle > safeAngle) {
				critical_energy += (currentAngle - safeAngle);
				//std::cout << "Current Angle = " << ROTATE_TO_DEGREE(currentAngle) << std::endl;
			}
		}
	}
	std::cout << "Fabrication energy (SupportLess) = " << critical_energy << std::endl;
}

void DeformTet::_get_energy_innerLoop_supportLess() {

	//Get the fabrication energy of supportLess
	double critical_energy = 0.0;
	double safeAngle = m_supportFreeAngle * PI / 180.0 + PI / 2.0;
	//std::cout << "Safe Angle = " << ROTATE_TO_DEGREE(safeAngle) << std::endl;
	int overhangTet_NUM = 0;
	for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[i];

		if (Tetra->isOverhangTet) {

			overhangTet_NUM++;

			QMeshFace* eachFace = NULL;		int overhangFace_Num = 0;
			Eigen::Vector3d whole_overhang_face_normal = Eigen::Vector3d::Zero();
			for (int i = 0; i < 4; i++) {
				eachFace = Tetra->GetFaceRecordPtr(i + 1);
				if (eachFace->isOverhangFace) {

					Eigen::Vector3d normal;
					normal = eachFace->normal_last; // initial face normal
					normal = -normal;
					normal.normalize();
					whole_overhang_face_normal += normal;
					overhangFace_Num++;
				}
			}
			if (overhangFace_Num == 0) std::cout << "ERROR: none of overhang face is detected!" << std::endl;
			whole_overhang_face_normal.normalize();
			Eigen::Vector3d face_Normal_Dir = Tetra->R_estimate * whole_overhang_face_normal;
			double currentAngle = acos(face_Normal_Dir.dot(printDir));

			if (currentAngle > safeAngle) {
				critical_energy += (currentAngle - safeAngle);
				//std::cout << "Current Angle = " << ROTATE_TO_DEGREE(currentAngle) << std::endl;
			}
		}
	}
	std::cout << "inner loop energy (SupportLess) (AVG) = " << (critical_energy / overhangTet_NUM) << std::endl;
}

void DeformTet::_globalQuaternionSmooth2() {

	double keep_weight = 1.5; // this term will influence smooth quality
	std::cout << "global Quaternion Smooth (keep all of the quaternion," 
		<< " critical keeping weight = "<< keep_weight << ")" << std::endl;

	int tet_Num = tetPatch->GetTetraNumber();

	// Compute the Laplacian Rotation Smooth Matrix A
	// Define matrix for global quaternion smooth
	Eigen::SparseMatrix<double> matLap_A;	matLap_A.resize(2 * tet_Num, tet_Num);
	Eigen::SparseMatrix<double> matLap_AT;	matLap_AT.resize(tet_Num, 2 * tet_Num);
	Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> GlobalQuatSmooth_Solver;
	Eigen::MatrixXd vector_Quat_x = Eigen::MatrixXd::Zero(tet_Num, 4);
	Eigen::MatrixXd vector_matrix_b = Eigen::MatrixXd::Zero(2 * tet_Num, 4);
	//give memory to sparse matrix, to accerate the insert speed
	matLap_A.reserve(Eigen::VectorXi::Constant(2 * tet_Num, 1000));
	//fill L matrix
	// Part 1: (1 -1/k -1/k ... 0)*(q1 q2 q3 ... 0)T = (0 0 0 ... 0)T
	// Part 2: (w1 0 0 ... 0)*(q1 q2 q3 ... 0)T = (w1* t1 w2* t2 ...)T
	for (int m = 0; m < tet_Num; m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];
		matLap_A.insert(m, m) = m_globalSmooth_weight;
		for (int i = 0; i < Tetra->neighborCell.size(); i++) {
			matLap_A.insert(m, Tetra->neighborCell[i]->GetIndexNo()) = -m_globalSmooth_weight / Tetra->neighborCell.size();
		}

		if (Tetra->isBottomTet || Tetra->isOverhangTet)
			matLap_A.insert(m + tet_Num, m) = keep_weight;
			//matLap_A.insert(m + tet_Num, m) = 1.0;
		else
			matLap_A.insert(m + tet_Num, m) = 1.0;

	}// finish Lap matrix build
	matLap_A.makeCompressed();
	matLap_AT = matLap_A.transpose();
	Eigen::SparseMatrix<double> matLap_ATA(tet_Num, tet_Num);
	matLap_ATA = matLap_AT * matLap_A;
	GlobalQuatSmooth_Solver.compute(matLap_ATA);
	std::cout << "Fill Laplacian matrix and factorize it" << std::endl;

	//record the quaternion after local process
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		Eigen::Quaterniond quat(Tetra->R_estimate);
		Tetra->R_quaternion = quat;
	}
	//fill D matrix
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		if (Tetra->isBottomTet || Tetra->isOverhangTet)
			vector_matrix_b.row(m + tet_Num) << Tetra->R_quaternion.w() * keep_weight,Tetra->R_quaternion.vec().transpose()* keep_weight;
		else
			vector_matrix_b.row(m + tet_Num) << Tetra->R_quaternion.w(), Tetra->R_quaternion.vec().transpose();
	}
	std::cout << "Fill D matrix" << std::endl;
	//solve equation
	for (int n = 0; n < 4; n++) {
		Eigen::VectorXd Lap_ATb = matLap_AT * vector_matrix_b.col(n);
		vector_Quat_x.col(n) = GlobalQuatSmooth_Solver.solve(Lap_ATb);
	}
	//update the rotation matrix generated from result quaternion
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		Tetra->R_quaternion.w() = vector_Quat_x(m, 0);
		Eigen::Vector3d Qvector = { vector_Quat_x(m, 1),vector_Quat_x(m, 2),vector_Quat_x(m, 3) };
		Tetra->R_quaternion.vec() = Qvector;
		Tetra->R_estimate = Tetra->R_quaternion.normalized().toRotationMatrix();
	}
}

// Add concavity smooth consideration
void DeformTet::_globalQuaternionSmooth3() {

	std::cout << "global Quaternion Smooth (keep all of the quaternion, change keeping weight" 
		<< " and consider the concavity of rotation) " << std::endl;

	double keep_weight = 4.0;

	//step 1: calculate the concavity of the virtual vector field
	this->_concavityDetection();

	int tet_Num = tetPatch->GetTetraNumber();

	// Compute the Laplacian Rotation Smooth Matrix A
	// Define matrix for global quaternion smooth
	Eigen::SparseMatrix<double> matLap_A;	matLap_A.resize(2 * tet_Num, tet_Num);
	Eigen::SparseMatrix<double> matLap_AT;	matLap_AT.resize(tet_Num, 2 * tet_Num);
	Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> GlobalQuatSmooth_Solver;
	Eigen::MatrixXd vector_Quat_x = Eigen::MatrixXd::Zero(tet_Num, 4);
	Eigen::MatrixXd vector_matrix_b = Eigen::MatrixXd::Zero(2 * tet_Num, 4);
	//give memory to sparse matrix, to accerate the insert speed
	matLap_A.reserve(Eigen::VectorXi::Constant(2 * tet_Num, 1000));
	//fill L matrix
	// Part 1: (sum -angle1 -angle2 ... 0)*(q1 q2 q3 ... 0)T = (0 0 0 ... 0)T
	
	for (int m = 0; m < tet_Num; m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		//double concaveAngle_sum = 0.0; 
		//for (int s = 0; s < 4; s++) {
		//	concaveAngle_sum += Tetra->GetFaceRecordPtr(s + 1)->concaveAngle;
		//}
		//double expand_ratio = 1.0; if (concaveAngle_sum > 0.0)expand_ratio = 100.0;
		//double temp_smoothWeight = m_globalSmooth_weight * expand_ratio;
		//matLap_A.insert(m, m) = temp_smoothWeight;
		//for (int i = 0; i < Tetra->neighborCell.size(); i++) {
		//	matLap_A.insert(m, Tetra->neighborCell[i]->GetIndexNo()) = -temp_smoothWeight / Tetra->neighborCell.size();
		//}

		double weight_sum = 0.0;
		for (int s = 0; s < 4; s++) {
 
			QMeshFace* eachFace_1tet = Tetra->GetFaceRecordPtr(s + 1);
			if (eachFace_1tet->GetLeftTetra() == NULL || eachFace_1tet->GetRightTetra() == NULL) continue;

			QMeshTetra* neighbor_tet = eachFace_1tet->GetLeftTetra();
			if (neighbor_tet == Tetra) neighbor_tet = eachFace_1tet->GetRightTetra();

			// weighting calculation
			// method 1:
			// angle -> 0deg -> weight -> 1; angle -> 180deg -> weight -> 181;
			double weight_each = weight_each = eachFace_1tet->concaveAngle + 1.0;

			//method 2:
			//double weight_each = 101.0 - 100.0 * cos(DEGREE_TO_ROTATE(eachFace_1tet->concaveAngle));

			//method 3:
			//double weight_each = pow(1.2, eachFace_1tet->concaveAngle);

			//method 4:
			//double weight_each = 1.0;
			//if (eachFace_1tet->concaveAngle > 45.0) weight_each = 1.0e7;

			//method 5:
			//double weight_each = 1.0 + 1e2 * pow(eachFace_1tet->concaveAngle, 0.5);

			//method ...
			matLap_A.insert(m, neighbor_tet->GetIndexNo()) = -m_globalSmooth_weight * weight_each;
			weight_sum += weight_each;
		}
		matLap_A.insert(m, m) = m_globalSmooth_weight * weight_sum;  

		// Part 2: (w1 0 0 ... 0)*(q1 q2 q3 ... 0)T = (w1* t1 w2* t2 ...)T
		// for all the quternion
		if (Tetra->isBottomTet || Tetra->isOverhangTet)
			matLap_A.insert(m + tet_Num, m) = keep_weight;
		else
			matLap_A.insert(m + tet_Num, m) = 1.0;

	}// finish Lap matrix build
	matLap_A.makeCompressed();
	matLap_AT = matLap_A.transpose();
	Eigen::SparseMatrix<double> matLap_ATA(tet_Num, tet_Num);
	matLap_ATA = matLap_AT * matLap_A;
	GlobalQuatSmooth_Solver.compute(matLap_ATA);
	std::cout << "Fill Laplacian matrix and factorize it" << std::endl;

	//record the quaternion after local process
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		Eigen::Quaterniond quat(Tetra->R_estimate);
		Tetra->R_quaternion = quat;
	}
	//fill D matrix
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		if (Tetra->isBottomTet || Tetra->isOverhangTet)
			vector_matrix_b.row(m + tet_Num) << Tetra->R_quaternion.w() * keep_weight, Tetra->R_quaternion.vec().transpose()* keep_weight;
		else
			vector_matrix_b.row(m + tet_Num) << Tetra->R_quaternion.w(), Tetra->R_quaternion.vec().transpose();
		//std::cout << vector_matrix_b.row(m + tet_Num) << std::endl;
	}
	std::cout << "Fill D matrix" << std::endl;
	//solve equation
	for (int n = 0; n < 4; n++) {
		Eigen::VectorXd Lap_ATb = matLap_AT * vector_matrix_b.col(n);
		vector_Quat_x.col(n) = GlobalQuatSmooth_Solver.solve(Lap_ATb);
		//std::cout << vector_Quat_x.col(n) << std::endl;
	}

	//update the rotation matrix generated from result quaternion
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		Tetra->R_quaternion.w() = vector_Quat_x(m, 0);
		Eigen::Vector3d Qvector = { vector_Quat_x(m, 1),vector_Quat_x(m, 2),vector_Quat_x(m, 3) };
		Tetra->R_quaternion.vec() = Qvector;
		Tetra->R_estimate = Tetra->R_quaternion.normalized().toRotationMatrix();
	}
}

void DeformTet::_cal_virtual_Vfield() {

	//calculate volume matrix is skiped 
	//as we have get it when we do initialization

	//get height field (scalar field)
	double boundingBox[6]; double heightRange = 0.0;
	tetPatch->ComputeBoundingBox(boundingBox);
	heightRange = boundingBox[3] - boundingBox[2];
	for (GLKPOSITION pos = tetPatch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(pos);

		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);
		node->scalarField_init = yy;
		node->scalarField = (yy - boundingBox[2]) / heightRange;
	}

	//transfer into vectorField
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

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
}

void DeformTet::_concavityDetection() {

	//_build_faceSet_4SpeedUp
	std::vector<QMeshFace*> faceSet(tetPatch->GetFaceNumber());
	int index = 0;
	for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);
		faceSet[index] = Face; index++;
	}
	// protect operation
	if (tetPatch->GetFaceNumber() != index)
		std::cout << "Error: please check the face num of tet mesh!" << std::endl;

	// compute the 2face angle of each pair of tetEle
#pragma omp parallel   
	{
#pragma omp for 

		for (int i = 0; i < faceSet.size(); i++) {
			QMeshFace* eachFace = faceSet[i];

			if (eachFace->GetLeftTetra() == NULL || eachFace->GetRightTetra() == NULL) continue;

			double angle_2face_1 = 0.0;
			bool exist_concave_1 = this->_cal_2FaceAngle(
				eachFace->GetLeftTetra(), eachFace, eachFace->GetRightTetra(), angle_2face_1);
				
			double angle_2face_2 = 0.0;
			bool exist_concave_2 = this->_cal_2FaceAngle(
				eachFace->GetRightTetra(), eachFace, eachFace->GetLeftTetra(), angle_2face_2);
			
			if (!exist_concave_1 && !exist_concave_2) {
				eachFace->concaveAngle = 0.0;
			}
			else if (exist_concave_1 && !exist_concave_2) {
				eachFace->concaveAngle = angle_2face_1;
			}
			else if (!exist_concave_1 && exist_concave_2) {
				eachFace->concaveAngle = angle_2face_2;
			}
			else {
				eachFace->concaveAngle = 0.5 * (angle_2face_1 + angle_2face_2);
			}

			if (eachFace->concaveAngle > 20.0) {
				eachFace->is_concave = true;
				//std::cout << "\n####### The concave angle of face " << eachFace->GetIndexNo()
					//<< " is " << eachFace->concaveAngle << std::endl;
			}
			//std::cout << "\n####### The concave angle of face " << eachFace->GetIndexNo()
					//<< " is " << eachFace->concaveAngle << std::endl;
		}
	}
}

bool DeformTet::_cal_2FaceAngle(QMeshTetra* hostTet, QMeshFace* eachFace,
	QMeshTetra* followTet, double& angle_2face) {

	Eigen::Vector3d Plane_V0 = Eigen::Vector3d::Zero();
	hostTet->CalCenterPos(Plane_V0[0], Plane_V0[1], Plane_V0[2]);
	std::vector<Eigen::Vector3d> intersect_Node_vector;

	for (int j = 0; j < 3; j++) {
		// each edge of face
		QMeshEdge* eachEdge_face = eachFace->GetEdgeRecordPtr(j + 1);
		Eigen::Vector3d Segment_P0 = Eigen::Vector3d::Zero();
		eachEdge_face->GetStartPoint()->GetCoord3D(Segment_P0[0], Segment_P0[1], Segment_P0[2]);
		Eigen::Vector3d Segment_P1 = Eigen::Vector3d::Zero();
		eachEdge_face->GetEndPoint()->GetCoord3D(Segment_P1[0], Segment_P1[1], Segment_P1[2]);
		Eigen::Vector3d intersect_P = Eigen::Vector3d::Zero();

		//segment-plane intersection
		int intersect_case = _intersect3D_SegmentPlane // please make sure vector field towards up
		(Segment_P0, Segment_P1, Plane_V0, hostTet->vectorField, intersect_P);

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
		if (OE0.cross(OE1).dot(hostTet->vectorField) < 0) {
			e = -e;
		}

		//concave case
		if (hostTet->vectorField.cross(followTet->vectorField).dot(e) < 0.0) {
			//std::cout << "concave pair" << std::endl;
			//get concave angle
			double angle_2Face = acos(hostTet->vectorField.dot(followTet->vectorField));
			//std::cout << "the angle_2Face is " << ROTATE_TO_DEGREE(angle_2Face) << std::endl;
			angle_2face = ROTATE_TO_DEGREE(angle_2Face);
			return true;
		}
	}
	angle_2face = -1.0;
	return false;
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
int DeformTet::_intersect3D_SegmentPlane(
	Eigen::Vector3d Segment_P0, Eigen::Vector3d Segment_P1,
	Eigen::Vector3d Plane_V0, Eigen::Vector3d Plane_Nor,
	Eigen::Vector3d& Intersect_P
) {
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

bool DeformTet::preProcess_4StrengthReinforcement() {

	PrincipleStressField* stressFieldComp = new PrincipleStressField(tetPatch);
	bool successful_Read = stressFieldComp->InputFEMResult(tetModel_Name);
	if (successful_Read) {
		stressFieldComp->ComputeElementPrincipleStress();
		stressFieldComp->DetermineCriticalTensileandCompressRegion(m_tensileRegionRatio, m_compressRegionRatio);
		stressFieldComp->DetectSmallCriticalRegionandClear();
	}
	delete stressFieldComp;

	return successful_Read;
}

bool DeformTet::preProcess_4StrengthReinforcement_stressLine() {

	bool new_Read_Method = true;
	bool successful_Read = false;

	PrincipleStressField* stressFieldComp = new PrincipleStressField(tetPatch);
	if (new_Read_Method) {

		
		successful_Read = stressFieldComp->PrincipalStressRead(tetModel_Name);
		if (successful_Read) {
			stressFieldComp->SelectTensileandCompressiveElements_stressLine();
		}
	}

	else {
		successful_Read = stressFieldComp->InputFEMResult_stressLine(tetModel_Name);
		if (successful_Read) {
			stressFieldComp->PrincipalStressAnalysis(false, true);
			stressFieldComp->SelectTensileandCompressiveElements_stressLine();
		}
	}

	delete stressFieldComp;

	return successful_Read;
}


void DeformTet::runASAP_StrengthReinforcement() {

	//---------------------------------------------------------------------------------------------------------------
	// Initialize size of nodes, faces and tets
	int node_Num = tetPatch->GetNodeNumber();
	int tet_Num = tetPatch->GetTetraNumber();
	int face_Num = tetPatch->GetFaceNumber();

	int neighborScale_face_num = 0;
	int critical_tet_num = 0; 
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos != nullptr;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		if (Tet->isTensileorCompressSelect)	critical_tet_num++;
	}
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL)	neighborScale_face_num++;
	}

	std::printf("All tet_Num %d , critical_tet_num %d.\n", tet_Num, critical_tet_num);
	std::printf("All face num %d , neighborScale_face_num %d.\n", face_Num, neighborScale_face_num);
	std::printf("A matrix sixe %d x %d\n", (5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));

	//---------------------------------------------------------------------------------------------------------------
	// Define variables
	std::vector<Eigen::MatrixXd> frame_Local_new, frame_Local_initial_inverse, frame_Local_initial;
	std::vector<Eigen::VectorXd> vector_X_Pos_Scale, vector_b;
	// Initialize variables
	frame_Local_new.resize(tet_Num);
	frame_Local_initial_inverse.resize(tet_Num);
	frame_Local_initial.resize(tet_Num);
	vector_X_Pos_Scale.resize(3);
	vector_b.resize(3);

	for (int i = 0; i < 3; i++) {
		vector_X_Pos_Scale[i] = Eigen::VectorXd::Zero(node_Num + tet_Num);
		vector_b[i] = Eigen::VectorXd::Zero(5 * tet_Num + neighborScale_face_num);
	}
	std::cout << "initial ASAP matrices" << std::endl;

	//---------------------------------------------------------------------------------------------------------------
	// Pre-compute the initial frame and inverse of initla frame
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tet->GetIndexNo();
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
		double center[3] = { 0 };
		QMeshNode* nodes[4];

		for (int i = 0; i < 4; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= 4;
		for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		frame_Local_initial[fdx] = P;
		frame_Local_initial_inverse[fdx] = Eigen::MatrixXd::Zero(3, 4);
		frame_Local_initial_inverse[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();
		//std::cout << frame_Local_initial[fdx] << std::endl << std::endl;
	}
	std::printf("Finish compute local frame and its inverse!\n\n");

	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	for (int loop = 0; loop < m_loopTime; loop++) {

		this->_calFabricationEnergy_StrengthReinforcement(frame_Local_initial_inverse);

#pragma omp parallel
		{
#pragma omp for 
			// local rotation and scaling operation (frame_Local_new)
			for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
				QMeshTetra* Tetra = tetraPatch_elementSet[i];

				int fdx = Tetra->GetIndexNo();
				double center[3] = { 0 };
				Tetra->CalCenterPos(center[0], center[1], center[2]);

				//This tP is each current frame in each tet.
				Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
				QMeshNode* nodes[4];
				for (int i = 0; i < 4; i++) {
					nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
					nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
				}
				for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

				Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
				Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
				Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

				T = frame_Local_initial_inverse[fdx] * tP;
				T_transpose = T.transpose();

				///// R1 //// Eigen SVD decomposition /////
				Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
				Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
				R = U * V.transpose();
				Tetra->R_estimate = R;
				frame_Local_new[fdx] = R * frame_Local_initial[fdx];

				///// R2 //// Fabrication requirement /////
				if (Tetra->isTensileorCompressSelect) {
					Eigen::Vector3d principal_Stress_Dir = Tetra->R_estimate * Tetra->tau_max;
					//Eigen::Vector3d principal_Stress_Dir = Tetra->R_estimate * Tetra->tau_min;

					Eigen::Vector3d projectDir; // assume the printing direction is (0,1,0)
					projectDir << principal_Stress_Dir(0), 0.0, principal_Stress_Dir(2);
					projectDir.normalized();

					Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();

					rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(principal_Stress_Dir, projectDir);

					//if (principal_Stress_Dir(1) > 0.0)
					//	rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(principal_Stress_Dir, printDir);
					//else
					//	rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(principal_Stress_Dir, -printDir);

					Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
					frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
				}
			}
		}
		std::printf("Finish new local frame calculation!\n");

		this->_globalQuaternionSmooth1();
		for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
			QMeshTetra* Tetra = tetraPatch_elementSet[i];
			frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
		}
		std::printf("Finish quaternion smooth calculation.\n");

		// Define variables
		Eigen::SparseMatrix<double> matrix_A_4x;
		Eigen::SparseMatrix<double> matrix_A_transpose_4x;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4x;
		//
		Eigen::SparseMatrix<double> matrix_A_4y;
		Eigen::SparseMatrix<double> matrix_A_transpose_4y;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4y;
		//
		Eigen::SparseMatrix<double> matrix_A_4z;
		Eigen::SparseMatrix<double> matrix_A_transpose_4z;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4z;
		// Initialize variables
		matrix_A_4x.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4x.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4y.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4y.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4z.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4z.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));

		// Fill A matrix for x,y,z
		// Block 11 for x,y,z
		// Important issues
		//give memory to sparse matrix, to accerate the insert speed
		matrix_A_4x.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4y.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4z.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));

		float c1 = -0.25, c2 = 0.75;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int fdx = Tetra->GetIndexNo() * 4;	int vdxArr[4];
			for (int i = 0; i < 4; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

			double weight = 1.0;
			if (Tetra->isTensileorCompressSelect)
				weight = m_criticalTet_weight_SR;

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (i == j) {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c2 * weight;
					}
					else {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c1 * weight;
					}
				}
			}
		}

		// Block 12 for x,y,z
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			double weight = 1.0;
			if (Tetra->isTensileorCompressSelect)
				weight = m_criticalTet_weight_SR;

			for (int i = 0; i < 4; i++) {
				matrix_A_4x.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](0, i); // x row
				matrix_A_4y.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](1, i); // y row
				matrix_A_4z.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](2, i); // z row
			}
		}
		// Block 21 equals to zero
		// Block 22 for x,y,z
		int constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			constraint_ind++;
		}
		// Block 31 equals to zero
		// Block 32 for x,y,z
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			matrix_A_4x.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4y.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4z.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;

			constraint_ind++;
		}

		std::cout << "Finish fill A matrix." << std::endl;

		long time = clock();

		// Factorize A matrix
		matrix_A_4x.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4x(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4x = matrix_A_4x.transpose();
		matATA_4x = matrix_A_transpose_4x * matrix_A_4x;
		Solver_ASAP_4x.compute(matATA_4x);

		matrix_A_4y.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4y(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4y = matrix_A_4y.transpose();
		matATA_4y = matrix_A_transpose_4y * matrix_A_4y;
		Solver_ASAP_4y.compute(matATA_4y);

		matrix_A_4z.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4z(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4z = matrix_A_4z.transpose();
		matATA_4z = matrix_A_transpose_4z * matrix_A_4z;
		Solver_ASAP_4z.compute(matATA_4z);

		std::printf("Finish factorize materix A\n");

		std::printf("The compress spends %f s.\n", (double(clock() - time)) / CLOCKS_PER_SEC);

		// Fill b vector for x,y,z
		//special case of regular term
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			for (int i = 0; i < 3; i++) {

				vector_b[i](4 * tet_Num + neighborScale_face_num + constraint_ind) = m_regularScale_weight;
			}
			constraint_ind++;
		}

		// Solve x vector
		Eigen::VectorXd ATb_4x = matrix_A_transpose_4x * vector_b[0];
		vector_X_Pos_Scale[0] = Solver_ASAP_4x.solve(ATb_4x);

		Eigen::VectorXd ATb_4y = matrix_A_transpose_4y * vector_b[1];
		vector_X_Pos_Scale[1] = Solver_ASAP_4y.solve(ATb_4y);

		Eigen::VectorXd ATb_4z = matrix_A_transpose_4z * vector_b[2];
		vector_X_Pos_Scale[2] = Solver_ASAP_4z.solve(ATb_4z);

		// Update vertex
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
			int idx = node->GetIndexNo();

			double xx, yy, zz;
			node->SetCoord3D(vector_X_Pos_Scale[0](idx), vector_X_Pos_Scale[1](idx), vector_X_Pos_Scale[2](idx));
			node->GetCoord3D(xx, yy, zz);
			//std::printf("New coordinate xx %f , yy %f , zz %f.\n", xx, yy, zz);
		}

		// record the scale value
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			for (int i = 0; i < 3; i++) {
				Tetra->scaleValue_vector[i] = vector_X_Pos_Scale[i](node_Num + fdx);
				//std::cout << vector_X_Pos_Scale[i](node_Num + fdx) << " ";
			}
			//std::cout << std::endl;
		}

		std::printf("End calculation of loop %d.\n\n", loop);
	}

	std::printf("End calculation.\n");
}

void DeformTet::_calFabricationEnergy_StrengthReinforcement(
	const std::vector<Eigen::MatrixXd>& frame_Local_initial_inverse) {

	//Get the fabrication energy of reinforcement
	double critical_energy = 0.0;
	for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[i];

		int fdx = Tetra->GetIndexNo();
		if (Tetra->isTensileorCompressSelect) {
			double center[3] = { 0 };
			Tetra->CalCenterPos(center[0], center[1], center[2]);

			//This tP is current frame.
			Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
			QMeshNode* nodes[4];
			for (int i = 0; i < 4; i++) {
				nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
				nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
			}
			for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

			Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
			Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
			Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

			T = frame_Local_initial_inverse[fdx] * tP;
			T_transpose = T.transpose();

			///// Get current tau_max by //// Eigen SVD decomposition /////
			Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
			Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
			R = U * V.transpose();
			Eigen::Vector3d new_Stress_Dir = R * Tetra->tau_max;
			critical_energy += pow((acos(new_Stress_Dir.dot(printDir)) - PI / 2), 2);
		}
	}
	std::cout << "Fabrication energy (StrengthReinforcement) = " << critical_energy << std::endl;
}

void DeformTet::_get_energy_innerLoop_strengthReinforcement() {

	//Get the fabrication energy of reinforcement
	double critical_energy = 0.0;
	int criticalEle_num = 0;
	for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[i];
	
		if (Tetra->isTensileorCompressSelect) {
			Eigen::Vector3d new_Stress_Dir = Tetra->R_estimate * Tetra->tau_max;
			critical_energy += fabs(acos(new_Stress_Dir.dot(printDir)) - PI / 2);
			criticalEle_num++;
		}
	}

	std::cout << "Fabrication energy (StrengthReinforcement) = "
		<< (critical_energy / criticalEle_num) * 180.0 / PI << std::endl;
}

void DeformTet::_globalQuaternionSmooth1() {

	//TODO: change weight according to the rotation angle

	//std::cout << "global Quaternion Smooth ( only the quaternion of critical tet) " << std::endl;

	int tet_Num = tetPatch->GetTetraNumber();
	int critical_tet_num = 0;

	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos != nullptr;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
		if (Tet->isTensileorCompressSelect || Tet->containProtectSurface
			|| Tet->isOverhangTet || Tet->isBottomTet)	
			critical_tet_num++;
	}

	// Pre-compute the Laplacian Rotation Smooth Matrix A
	// Define matrix for global quaternion smooth
	Eigen::SparseMatrix<double> matLap_A;	matLap_A.resize(tet_Num + critical_tet_num, tet_Num);
	Eigen::SparseMatrix<double> matLap_AT;	matLap_AT.resize(tet_Num, tet_Num + critical_tet_num);
	Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> GlobalQuatSmooth_Solver;
	Eigen::MatrixXd vector_Quat_x = Eigen::MatrixXd::Zero(tet_Num, 4);
	Eigen::MatrixXd vector_matrix_b = Eigen::MatrixXd::Zero(tet_Num + critical_tet_num, 4);
	//give memory to sparse matrix, to accerate the insert speed
	matLap_A.reserve(Eigen::VectorXi::Constant(tet_Num + critical_tet_num, 1000));
	//fill L matrix
	// Part 1: (1 -1/k -1/k ... 0)*(q1 q2 q3 ... 0)T = (0 0 0 ... 0)T
	// Part 2: (w1 0 0 ... 0)*(q1 q2 q3 ... 0)T = (w1* t1 w2* t2 ...)T
	int keep_constraint_lineNum = 0;
	for (int m = 0; m < tet_Num; m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];
		matLap_A.insert(m, m) = m_globalSmooth_weight;
		for (int i = 0; i < Tetra->neighborCell.size(); i++) {
			matLap_A.insert(m, Tetra->neighborCell[i]->GetIndexNo()) = -m_globalSmooth_weight / Tetra->neighborCell.size();
		}

		if (Tetra->isTensileorCompressSelect || Tetra->containProtectSurface
			|| Tetra->isOverhangTet || Tetra->isBottomTet) {
			matLap_A.insert(keep_constraint_lineNum + tet_Num, m) = 1.0;
			keep_constraint_lineNum++;
		}
	}// finish Lap matrix build
	matLap_A.makeCompressed();
	matLap_AT = matLap_A.transpose();
	Eigen::SparseMatrix<double> matLap_ATA(tet_Num, tet_Num);
	matLap_ATA = matLap_AT * matLap_A;
	GlobalQuatSmooth_Solver.compute(matLap_ATA);
	//std::cout << "Fill Laplacian matrix and factorize it" << std::endl;

	//record the quaternion after local process
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		Eigen::Quaterniond quat(Tetra->R_estimate);
		Tetra->R_quaternion = quat;
	}
	//fill D matrix
	keep_constraint_lineNum = 0;
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		if (Tetra->isTensileorCompressSelect || Tetra->containProtectSurface
			|| Tetra->isOverhangTet || Tetra->isBottomTet) {
			vector_matrix_b.row(keep_constraint_lineNum + tet_Num) << Tetra->R_quaternion.w() * 1.0
				, Tetra->R_quaternion.vec().transpose() * 1.0;
			keep_constraint_lineNum++;
		}
	}
	//std::cout << "Fill D matrix" << std::endl;
	//solve equation
	for (int n = 0; n < 4; n++) {
		Eigen::VectorXd Lap_ATb = matLap_AT * vector_matrix_b.col(n);
		vector_Quat_x.col(n) = GlobalQuatSmooth_Solver.solve(Lap_ATb);
	}
	//update the rotation matrix generated from result quaternion
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		//if (Tetra->isOverhangTet == false) continue;
		Tetra->R_quaternion.w() = vector_Quat_x(m, 0);
		Eigen::Vector3d Qvector = { vector_Quat_x(m, 1),vector_Quat_x(m, 2),vector_Quat_x(m, 3) };
		Tetra->R_quaternion.vec() = Qvector;
		Tetra->R_estimate = Tetra->R_quaternion.normalized().toRotationMatrix();
	}
}

void DeformTet::_globalQuaternionSmooth1_supportLess() {

	//std::cout << "----------- " << m_globalSmooth_weight <<  " --------------" << std::endl;

	int tet_Num = tetPatch->GetTetraNumber();
	int critical_tet_num = 0;

	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos != nullptr;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
		if (Tet->isOverhangTet || Tet->isBottomTet)
			critical_tet_num++;
	}

	// Pre-compute the Laplacian Rotation Smooth Matrix A
	// Define matrix for global quaternion smooth
	Eigen::SparseMatrix<double> matLap_A;	matLap_A.resize(tet_Num + critical_tet_num, tet_Num);
	Eigen::SparseMatrix<double> matLap_AT;	matLap_AT.resize(tet_Num, tet_Num + critical_tet_num);
	Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> GlobalQuatSmooth_Solver;
	Eigen::MatrixXd vector_Quat_x = Eigen::MatrixXd::Zero(tet_Num, 4);
	Eigen::MatrixXd vector_matrix_b = Eigen::MatrixXd::Zero(tet_Num + critical_tet_num, 4);
	//give memory to sparse matrix, to accerate the insert speed
	matLap_A.reserve(Eigen::VectorXi::Constant(tet_Num + critical_tet_num, 1000));
	//fill L matrix
	// Part 1: (1 -1/k -1/k ... 0)*(q1 q2 q3 ... 0)T = (0 0 0 ... 0)T
	// Part 2: (w1 0 0 ... 0)*(q1 q2 q3 ... 0)T = (w1* t1 w2* t2 ...)T
	int keep_constraint_lineNum = 0;
	for (int m = 0; m < tet_Num; m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];
		matLap_A.insert(m, m) = m_globalSmooth_weight;
		for (int i = 0; i < Tetra->neighborCell.size(); i++) {
			matLap_A.insert(m, Tetra->neighborCell[i]->GetIndexNo()) 
				= -m_globalSmooth_weight / Tetra->neighborCell.size();
		}

		if (Tetra->isOverhangTet || Tetra->isBottomTet) {
			matLap_A.insert(keep_constraint_lineNum + tet_Num, m) = m_keep_SL;
			keep_constraint_lineNum++;
		}
	}// finish Lap matrix build
	matLap_A.makeCompressed();
	matLap_AT = matLap_A.transpose();
	Eigen::SparseMatrix<double> matLap_ATA(tet_Num, tet_Num);
	matLap_ATA = matLap_AT * matLap_A;
	GlobalQuatSmooth_Solver.compute(matLap_ATA);
	//std::cout << "Fill Laplacian matrix and factorize it" << std::endl;

	//record the quaternion after local process
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		Eigen::Quaterniond quat(Tetra->R_estimate);
		Tetra->R_quaternion = quat;
	}
	//fill D matrix
	keep_constraint_lineNum = 0;
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		if (Tetra->isOverhangTet || Tetra->isBottomTet) {
			vector_matrix_b.row(keep_constraint_lineNum + tet_Num) 
				<< Tetra->R_quaternion.w() * m_keep_SL
				, Tetra->R_quaternion.vec().transpose() * m_keep_SL;
			keep_constraint_lineNum++;
		}
	}
	//std::cout << "Fill D matrix" << std::endl;
	//solve equation
	for (int n = 0; n < 4; n++) {
		Eigen::VectorXd Lap_ATb = matLap_AT * vector_matrix_b.col(n);
		vector_Quat_x.col(n) = GlobalQuatSmooth_Solver.solve(Lap_ATb);
	}
	//update the rotation matrix generated from result quaternion
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		//if (Tetra->isOverhangTet == false) continue;
		Tetra->R_quaternion.w() = vector_Quat_x(m, 0);
		Eigen::Vector3d Qvector = { vector_Quat_x(m, 1),vector_Quat_x(m, 2),vector_Quat_x(m, 3) };
		Tetra->R_quaternion.vec() = Qvector;
		Tetra->R_estimate = Tetra->R_quaternion.normalized().toRotationMatrix();
	}
}

void DeformTet::_globalQuaternionSmooth1_surfaceQuality() {

	double keepWeight = 1.0;
	double keepWeight_4largerLayer = m_keep_SQ_cover;
	double keepWeight_4verticalGrow = m_keep_SQ_vertical;
	//std::cout << "----------- " << m_globalSmooth_weight << " --------------" << std::endl;

	int tet_Num = tetPatch->GetTetraNumber();
	int critical_tet_num = 0;

	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos != nullptr;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
		if (Tet->containProtectSurface || Tet->isBottomTet)
			critical_tet_num++;
	}

	// Pre-compute the Laplacian Rotation Smooth Matrix A
	// Define matrix for global quaternion smooth
	Eigen::SparseMatrix<double> matLap_A;	matLap_A.resize(tet_Num + critical_tet_num, tet_Num);
	Eigen::SparseMatrix<double> matLap_AT;	matLap_AT.resize(tet_Num, tet_Num + critical_tet_num);
	Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> GlobalQuatSmooth_Solver;
	Eigen::MatrixXd vector_Quat_x = Eigen::MatrixXd::Zero(tet_Num, 4);
	Eigen::MatrixXd vector_matrix_b = Eigen::MatrixXd::Zero(tet_Num + critical_tet_num, 4);
	//give memory to sparse matrix, to accerate the insert speed
	matLap_A.reserve(Eigen::VectorXi::Constant(tet_Num + critical_tet_num, 1000));
	//fill L matrix
	// Part 1: (1 -1/k -1/k ... 0)*(q1 q2 q3 ... 0)T = (0 0 0 ... 0)T
	// Part 2: (w1 0 0 ... 0)*(q1 q2 q3 ... 0)T = (w1* t1 w2* t2 ...)T
	int keep_constraint_lineNum = 0;
	for (int m = 0; m < tet_Num; m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];
		matLap_A.insert(m, m) = m_globalSmooth_weight;
		for (int i = 0; i < Tetra->neighborCell.size(); i++) {
			matLap_A.insert(m, Tetra->neighborCell[i]->GetIndexNo()) 
				= -m_globalSmooth_weight / Tetra->neighborCell.size();
		}

		if (Tetra->containProtectSurface || Tetra->isBottomTet) {

			double temp_weight = keepWeight;
			if (Tetra->containProtectSurface && Tetra->select_coverLayer == 2)			
				temp_weight = keepWeight_4largerLayer;
			else if (Tetra->containProtectSurface && Tetra->select_coverLayer == 1)		
				temp_weight = keepWeight_4verticalGrow;
			else																		
				temp_weight = keepWeight;

			matLap_A.insert(keep_constraint_lineNum + tet_Num, m) = temp_weight;
			keep_constraint_lineNum++;
		}
	}// finish Lap matrix build
	matLap_A.makeCompressed();
	matLap_AT = matLap_A.transpose();
	Eigen::SparseMatrix<double> matLap_ATA(tet_Num, tet_Num);
	matLap_ATA = matLap_AT * matLap_A;
	GlobalQuatSmooth_Solver.compute(matLap_ATA);
	//std::cout << "Fill Laplacian matrix and factorize it" << std::endl;

	//record the quaternion after local process
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		Eigen::Quaterniond quat(Tetra->R_estimate);
		Tetra->R_quaternion = quat;
	}
	//fill D matrix
	keep_constraint_lineNum = 0;
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		if (Tetra->containProtectSurface || Tetra->isBottomTet) {

			double temp_weight = keepWeight;
			if (Tetra->containProtectSurface && Tetra->select_coverLayer == 2)			
				temp_weight = keepWeight_4largerLayer;
			else if (Tetra->containProtectSurface && Tetra->select_coverLayer == 1)		
				temp_weight = keepWeight_4verticalGrow;
			else																		
				temp_weight = keepWeight;

			vector_matrix_b.row(keep_constraint_lineNum + tet_Num) 
				<< Tetra->R_quaternion.w() * temp_weight
				, Tetra->R_quaternion.vec().transpose() * temp_weight;
			keep_constraint_lineNum++;
		}
	}
	//std::cout << "Fill D matrix" << std::endl;
	//solve equation
	for (int n = 0; n < 4; n++) {
		Eigen::VectorXd Lap_ATb = matLap_AT * vector_matrix_b.col(n);
		vector_Quat_x.col(n) = GlobalQuatSmooth_Solver.solve(Lap_ATb);
	}
	//update the rotation matrix generated from result quaternion
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		Tetra->R_quaternion.w() = vector_Quat_x(m, 0);
		Eigen::Vector3d Qvector = { vector_Quat_x(m, 1),vector_Quat_x(m, 2),vector_Quat_x(m, 3) };
		Tetra->R_quaternion.vec() = Qvector;
		Tetra->R_estimate = Tetra->R_quaternion.normalized().toRotationMatrix();
	}
}

void DeformTet::_globalQuaternionSmooth1_strengthReinforcement() {

	//std::cout << "----------- " << m_globalSmooth_weight << " --------------" << std::endl;

	int tet_Num = tetPatch->GetTetraNumber();
	int critical_tet_num = 0;

	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos != nullptr;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
		if (Tet->isTensileorCompressSelect || Tet->isBottomTet)
			critical_tet_num++;
	}

	// Pre-compute the Laplacian Rotation Smooth Matrix A
	// Define matrix for global quaternion smooth
	Eigen::SparseMatrix<double> matLap_A;	matLap_A.resize(tet_Num + critical_tet_num, tet_Num);
	Eigen::SparseMatrix<double> matLap_AT;	matLap_AT.resize(tet_Num, tet_Num + critical_tet_num);
	Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> GlobalQuatSmooth_Solver;
	Eigen::MatrixXd vector_Quat_x = Eigen::MatrixXd::Zero(tet_Num, 4);
	Eigen::MatrixXd vector_matrix_b = Eigen::MatrixXd::Zero(tet_Num + critical_tet_num, 4);
	//give memory to sparse matrix, to accerate the insert speed
	matLap_A.reserve(Eigen::VectorXi::Constant(tet_Num + critical_tet_num, 1000));
	//fill L matrix
	// Part 1: (1 -1/k -1/k ... 0)*(q1 q2 q3 ... 0)T = (0 0 0 ... 0)T
	// Part 2: (w1 0 0 ... 0)*(q1 q2 q3 ... 0)T = (w1* t1 w2* t2 ...)T
	int keep_constraint_lineNum = 0;
	for (int m = 0; m < tet_Num; m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];
		matLap_A.insert(m, m) = m_globalSmooth_weight;
		for (int i = 0; i < Tetra->neighborCell.size(); i++) {
			matLap_A.insert(m, Tetra->neighborCell[i]->GetIndexNo()) 
				= -m_globalSmooth_weight / Tetra->neighborCell.size();
		}

		if (Tetra->isBottomTet) {
			matLap_A.insert(keep_constraint_lineNum + tet_Num, m) = 155;
			keep_constraint_lineNum++;
		}
		else if(Tetra->isTensileorCompressSelect) {
			matLap_A.insert(keep_constraint_lineNum + tet_Num, m) = m_keep_SR;
			keep_constraint_lineNum++;
		}

	}// finish Lap matrix build
	matLap_A.makeCompressed();
	matLap_AT = matLap_A.transpose();
	Eigen::SparseMatrix<double> matLap_ATA(tet_Num, tet_Num);
	matLap_ATA = matLap_AT * matLap_A;
	GlobalQuatSmooth_Solver.compute(matLap_ATA);
	//std::cout << "Fill Laplacian matrix and factorize it" << std::endl;

	//record the quaternion after local process
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		Eigen::Quaterniond quat(Tetra->R_estimate);
		Tetra->R_quaternion = quat;
	}
	//fill D matrix
	keep_constraint_lineNum = 0;
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		if (Tetra->isTensileorCompressSelect || Tetra->isBottomTet) {
			vector_matrix_b.row(keep_constraint_lineNum + tet_Num) 
				<< Tetra->R_quaternion.w() * m_keep_SR
				, Tetra->R_quaternion.vec().transpose() * m_keep_SR;
			keep_constraint_lineNum++;
		}
	}
	//std::cout << "Fill D matrix" << std::endl;
	//solve equation
	for (int n = 0; n < 4; n++) {
		Eigen::VectorXd Lap_ATb = matLap_AT * vector_matrix_b.col(n);
		vector_Quat_x.col(n) = GlobalQuatSmooth_Solver.solve(Lap_ATb);
	}
	//update the rotation matrix generated from result quaternion
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		//if (Tetra->isOverhangTet == false) continue;
		Tetra->R_quaternion.w() = vector_Quat_x(m, 0);
		Eigen::Vector3d Qvector = { vector_Quat_x(m, 1),vector_Quat_x(m, 2),vector_Quat_x(m, 3) };
		Tetra->R_quaternion.vec() = Qvector;
		Tetra->R_estimate = Tetra->R_quaternion.normalized().toRotationMatrix();
	}
}

//hybrid consider all of the weighting of critical region
void DeformTet::_globalQuaternionSmooth4() {

	////TODO: change weight according to the surface Preservation Case
	//double keepWeight = 1.0;
	//double keepWeight_4largerLayer = 2.5;
	//double keepWeight_4verticalGrow = 1.0;

	std::cout << "global Quaternion Smooth ( only the quaternion of critical tet) + keep weighting change" << std::endl;

	int tet_Num = tetPatch->GetTetraNumber();
	int critical_tet_num = 0;

	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos != nullptr;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
		if (Tet->isTensileorCompressSelect || Tet->containProtectSurface
			|| Tet->isOverhangTet || Tet->isBottomTet)
			critical_tet_num++;
	}

	// Pre-compute the Laplacian Rotation Smooth Matrix A
	// Define matrix for global quaternion smooth
	Eigen::SparseMatrix<double> matLap_A;	matLap_A.resize(tet_Num + critical_tet_num, tet_Num);
	Eigen::SparseMatrix<double> matLap_AT;	matLap_AT.resize(tet_Num, tet_Num + critical_tet_num);
	Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> GlobalQuatSmooth_Solver;
	Eigen::MatrixXd vector_Quat_x = Eigen::MatrixXd::Zero(tet_Num, 4);
	Eigen::MatrixXd vector_matrix_b = Eigen::MatrixXd::Zero(tet_Num + critical_tet_num, 4);
	//give memory to sparse matrix, to accerate the insert speed
	matLap_A.reserve(Eigen::VectorXi::Constant(tet_Num + critical_tet_num, 1000));
	//fill L matrix
	// Part 1: (1 -1/k -1/k ... 0)*(q1 q2 q3 ... 0)T = (0 0 0 ... 0)T
	// Part 2: (w1 0 0 ... 0)*(q1 q2 q3 ... 0)T = (w1* t1 w2* t2 ...)T
	int keep_constraint_lineNum = 0;
	for (int m = 0; m < tet_Num; m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];
		matLap_A.insert(m, m) = m_globalSmooth_weight;
		for (int i = 0; i < Tetra->neighborCell.size(); i++) {
			matLap_A.insert(m, Tetra->neighborCell[i]->GetIndexNo()) = -m_globalSmooth_weight / Tetra->neighborCell.size();
		}

		if (Tetra->isTensileorCompressSelect || Tetra->containProtectSurface
			|| Tetra->isOverhangTet || Tetra->isBottomTet) {

			double weight = this->_get_globalSmooth_keepWeight(Tetra);
			matLap_A.insert(keep_constraint_lineNum + tet_Num, m) = weight;
			keep_constraint_lineNum++;
		}
	}// finish Lap matrix build
	matLap_A.makeCompressed();
	matLap_AT = matLap_A.transpose();
	Eigen::SparseMatrix<double> matLap_ATA(tet_Num, tet_Num);
	matLap_ATA = matLap_AT * matLap_A;
	GlobalQuatSmooth_Solver.compute(matLap_ATA);
	//std::cout << "Fill Laplacian matrix and factorize it" << std::endl;

	//record the quaternion after local process
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		Eigen::Quaterniond quat(Tetra->R_estimate);
		Tetra->R_quaternion = quat;
	}
	//fill D matrix
	keep_constraint_lineNum = 0;
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		if (Tetra->isTensileorCompressSelect || Tetra->containProtectSurface
			|| Tetra->isOverhangTet || Tetra->isBottomTet) {
																			
			double weight = this->_get_globalSmooth_keepWeight(Tetra);

			vector_matrix_b.row(keep_constraint_lineNum + tet_Num) << Tetra->R_quaternion.w() * weight
				, Tetra->R_quaternion.vec().transpose() * weight;
			keep_constraint_lineNum++;
		}
	}
	//std::cout << "Fill D matrix" << std::endl;
	//solve equation
	for (int n = 0; n < 4; n++) {
		Eigen::VectorXd Lap_ATb = matLap_AT * vector_matrix_b.col(n);
		vector_Quat_x.col(n) = GlobalQuatSmooth_Solver.solve(Lap_ATb);
	}
	//update the rotation matrix generated from result quaternion
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		//if (Tetra->isOverhangTet == false) continue;
		Tetra->R_quaternion.w() = vector_Quat_x(m, 0);
		Eigen::Vector3d Qvector = { vector_Quat_x(m, 1),vector_Quat_x(m, 2),vector_Quat_x(m, 3) };
		Tetra->R_quaternion.vec() = Qvector;
		Tetra->R_estimate = Tetra->R_quaternion.normalized().toRotationMatrix();
	}

	//calculate the L2-norm of face neighbor quaternion
	double max_L2_norm = -1e10;
	for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos != nullptr;) {
		QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

		if (Face->GetRightTetra() == NULL || Face->GetLeftTetra() == NULL) continue;

		Eigen::Vector4d q_r = { Face->GetRightTetra()->R_quaternion.w(),Face->GetRightTetra()->R_quaternion.x(),
			Face->GetRightTetra()->R_quaternion.y(),Face->GetRightTetra()->R_quaternion.z() };
		Eigen::Vector4d q_l = { Face->GetLeftTetra()->R_quaternion.w(),Face->GetLeftTetra()->R_quaternion.x(),
			Face->GetLeftTetra()->R_quaternion.y(),Face->GetLeftTetra()->R_quaternion.z() };

		double L2_norm = (q_r - q_l).norm();
		if (L2_norm > max_L2_norm) max_L2_norm = L2_norm;
	}
	std::cout << "the max L2 norm is: " << max_L2_norm << std::endl;
}

void DeformTet::_globalQuaternionSmooth4_withoutBottom() {

	////TODO: change weight according to the surface Preservation Case
	//double keepWeight = 1.0;
	//double keepWeight_4largerLayer = 2.5;
	//double keepWeight_4verticalGrow = 1.0;

	std::cout << "global Quaternion Smooth ( only the quaternion of critical tet) + keep weighting change" << std::endl;

	int tet_Num = tetPatch->GetTetraNumber();
	int critical_tet_num = 0;

	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos != nullptr;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
		if (Tet->isTensileorCompressSelect || Tet->containProtectSurface
			|| Tet->isOverhangTet 
			//|| Tet->isBottomTet
			)
			critical_tet_num++;
	}

	// Pre-compute the Laplacian Rotation Smooth Matrix A
	// Define matrix for global quaternion smooth
	Eigen::SparseMatrix<double> matLap_A;	matLap_A.resize(tet_Num + critical_tet_num, tet_Num);
	Eigen::SparseMatrix<double> matLap_AT;	matLap_AT.resize(tet_Num, tet_Num + critical_tet_num);
	Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> GlobalQuatSmooth_Solver;
	Eigen::MatrixXd vector_Quat_x = Eigen::MatrixXd::Zero(tet_Num, 4);
	Eigen::MatrixXd vector_matrix_b = Eigen::MatrixXd::Zero(tet_Num + critical_tet_num, 4);
	//give memory to sparse matrix, to accerate the insert speed
	matLap_A.reserve(Eigen::VectorXi::Constant(tet_Num + critical_tet_num, 1000));
	//fill L matrix
	// Part 1: (1 -1/k -1/k ... 0)*(q1 q2 q3 ... 0)T = (0 0 0 ... 0)T
	// Part 2: (w1 0 0 ... 0)*(q1 q2 q3 ... 0)T = (w1* t1 w2* t2 ...)T
	int keep_constraint_lineNum = 0;
	for (int m = 0; m < tet_Num; m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];
		matLap_A.insert(m, m) = m_globalSmooth_weight;
		for (int i = 0; i < Tetra->neighborCell.size(); i++) {
			matLap_A.insert(m, Tetra->neighborCell[i]->GetIndexNo()) = -m_globalSmooth_weight / Tetra->neighborCell.size();
		}

		if (Tetra->isTensileorCompressSelect || Tetra->containProtectSurface
			|| Tetra->isOverhangTet 
			//|| Tetra->isBottomTet
			) {

			double weight = this->_get_globalSmooth_keepWeight(Tetra);
			matLap_A.insert(keep_constraint_lineNum + tet_Num, m) = weight;
			keep_constraint_lineNum++;
		}
	}// finish Lap matrix build
	matLap_A.makeCompressed();
	matLap_AT = matLap_A.transpose();
	Eigen::SparseMatrix<double> matLap_ATA(tet_Num, tet_Num);
	matLap_ATA = matLap_AT * matLap_A;
	GlobalQuatSmooth_Solver.compute(matLap_ATA);
	//std::cout << "Fill Laplacian matrix and factorize it" << std::endl;

	//record the quaternion after local process
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		Eigen::Quaterniond quat(Tetra->R_estimate);
		Tetra->R_quaternion = quat;
	}
	//fill D matrix
	keep_constraint_lineNum = 0;
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		if (Tetra->isTensileorCompressSelect || Tetra->containProtectSurface
			|| Tetra->isOverhangTet 
			//|| Tetra->isBottomTet
			) {

			double weight = this->_get_globalSmooth_keepWeight(Tetra);

			vector_matrix_b.row(keep_constraint_lineNum + tet_Num) << Tetra->R_quaternion.w() * weight
				, Tetra->R_quaternion.vec().transpose()* weight;
			keep_constraint_lineNum++;
		}
	}
	//std::cout << "Fill D matrix" << std::endl;
	//solve equation
	for (int n = 0; n < 4; n++) {
		Eigen::VectorXd Lap_ATb = matLap_AT * vector_matrix_b.col(n);
		vector_Quat_x.col(n) = GlobalQuatSmooth_Solver.solve(Lap_ATb);
	}
	//update the rotation matrix generated from result quaternion
	for (int m = 0; m < tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		//if (Tetra->isOverhangTet == false) continue;
		Tetra->R_quaternion.w() = vector_Quat_x(m, 0);
		Eigen::Vector3d Qvector = { vector_Quat_x(m, 1),vector_Quat_x(m, 2),vector_Quat_x(m, 3) };
		Tetra->R_quaternion.vec() = Qvector;
		Tetra->R_estimate = Tetra->R_quaternion.normalized().toRotationMatrix();
	}
}

void DeformTet::runASAP_StrengthReinforcement_test() {

	//---------------------------------------------------------------------------------------------------------------
	// Initialize size of nodes, faces and tets
	int node_Num = tetPatch->GetNodeNumber();
	int tet_Num = tetPatch->GetTetraNumber();
	int face_Num = tetPatch->GetFaceNumber();

	int neighborScale_face_num = 0;
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL)	neighborScale_face_num++;
	}

	std::printf("All face num %d , neighborScale_face_num %d.\n", face_Num, neighborScale_face_num);
	std::printf("A matrix sixe %d x %d\n", (5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));

	//---------------------------------------------------------------------------------------------------------------
	// Define variables
	std::vector<Eigen::MatrixXd> frame_Local_new, frame_Local_initial_inverse, frame_Local_initial;
	std::vector<Eigen::VectorXd> vector_X_Pos_Scale, vector_b;
	// Initialize variables
	frame_Local_new.resize(tet_Num);
	frame_Local_initial_inverse.resize(tet_Num);
	frame_Local_initial.resize(tet_Num);
	vector_X_Pos_Scale.resize(3);
	vector_b.resize(3);

	for (int i = 0; i < 3; i++) {
		vector_X_Pos_Scale[i] = Eigen::VectorXd::Zero(node_Num + tet_Num);
		vector_b[i] = Eigen::VectorXd::Zero(5 * tet_Num + neighborScale_face_num);
	}
	std::cout << "initial ASAP matrices" << std::endl;

	//---------------------------------------------------------------------------------------------------------------
	// Pre-compute the initial frame and inverse of initla frame
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tet->GetIndexNo();
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
		double center[3] = { 0 };
		QMeshNode* nodes[4];

		for (int i = 0; i < 4; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= 4;
		for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		frame_Local_initial[fdx] = P;
		frame_Local_initial_inverse[fdx] = Eigen::MatrixXd::Zero(3, 4);
		frame_Local_initial_inverse[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();
		//std::cout << frame_Local_initial[fdx] << std::endl << std::endl;
	}
	std::printf("Finish compute local frame and its inverse!\n\n");

	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	for (int loop = 0; loop < m_loopTime; loop++) {

		/*debug
		for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
			QMeshTetra* Tetra = tetraPatch_elementSet[i];
			if (Tetra->isTensileorCompressSelect) {
				//std::cout << "\n------->this is strength ele" << std::endl;
				for (int i = 0; i < 4; i++) {
					Tetra->GetFaceRecordPtr(i + 1)->isSpecialShow = true;
				}
			}
		}
		*/

		this->_calFabricationEnergy_StrengthReinforcement(frame_Local_initial_inverse);

		for (int innerLoop = 0; innerLoop < m_innerLoopTime; innerLoop++) {

#pragma omp parallel
			{
#pragma omp for 
				// local rotation and scaling operation (frame_Local_new)
				for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
					QMeshTetra* Tetra = tetraPatch_elementSet[i];

					int fdx = Tetra->GetIndexNo();
					//add here
					if (innerLoop == 0) {

						double center[3] = { 0 };
						Tetra->CalCenterPos(center[0], center[1], center[2]);

						//This tP is each current frame in each tet.
						Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
						QMeshNode* nodes[4];
						for (int i = 0; i < 4; i++) {
							nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
							nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
						}
						for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

						Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

						T = frame_Local_initial_inverse[fdx] * tP;
						T_transpose = T.transpose();

						///// R1 //// Eigen SVD decomposition /////
						Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
						Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
						R = U * V.transpose();
						Tetra->R_estimate = R;
						frame_Local_new[fdx] = R * frame_Local_initial[fdx];
					}

					///// R2 //// Fabrication requirement /////
					if (Tetra->isTensileorCompressSelect) {
						Eigen::Vector3d principal_Stress_Dir = Tetra->R_estimate * Tetra->tau_max;

						Eigen::Matrix3d rotationMatrix = this->_cal_rotationMatrix_strengthReinforcement(principal_Stress_Dir);

						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					}
				}
			}
			std::printf("Finish new local frame calculation!\n");

			this->_globalQuaternionSmooth1_strengthReinforcement();

			for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
				QMeshTetra* Tetra = tetraPatch_elementSet[i];
				frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
			}
			std::printf("Finish quaternion smooth calculation.\n");

			this->_get_energy_innerLoop_strengthReinforcement();
		}
		// Define variables
		Eigen::SparseMatrix<double> matrix_A_4x;
		Eigen::SparseMatrix<double> matrix_A_transpose_4x;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4x;
		//
		Eigen::SparseMatrix<double> matrix_A_4y;
		Eigen::SparseMatrix<double> matrix_A_transpose_4y;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4y;
		//
		Eigen::SparseMatrix<double> matrix_A_4z;
		Eigen::SparseMatrix<double> matrix_A_transpose_4z;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4z;
		// Initialize variables
		matrix_A_4x.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4x.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4y.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4y.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4z.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4z.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));

		// Fill A matrix for x,y,z
		// Block 11 for x,y,z
		// Important issues
		//give memory to sparse matrix, to accerate the insert speed
		matrix_A_4x.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4y.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4z.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));

		float c1 = -0.25, c2 = 0.75;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int fdx = Tetra->GetIndexNo() * 4;	int vdxArr[4];
			for (int i = 0; i < 4; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

			double weight = 1.0;
			if (Tetra->isTensileorCompressSelect)		weight = m_criticalTet_weight_SR;

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (i == j) {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c2 * weight;
					}
					else {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c1 * weight;
					}
				}
			}
		}

		// Block 12 for x,y,z
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			double weight = 1.0;
			if (Tetra->isTensileorCompressSelect)		weight = m_criticalTet_weight_SR;

			for (int i = 0; i < 4; i++) {
				matrix_A_4x.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](0, i); // x row
				matrix_A_4y.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](1, i); // y row
				matrix_A_4z.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](2, i); // z row
			}
		}
		// Block 21 equals to zero
		// Block 22 for x,y,z
		int constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			constraint_ind++;
		}
		// Block 31 equals to zero
		// Block 32 for x,y,z
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			matrix_A_4x.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4y.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4z.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;

			constraint_ind++;
		}

		std::cout << "Finish fill A matrix." << std::endl;

		long time = clock();

		// Factorize A matrix
		matrix_A_4x.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4x(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4x = matrix_A_4x.transpose();
		matATA_4x = matrix_A_transpose_4x * matrix_A_4x;
		Solver_ASAP_4x.compute(matATA_4x);

		matrix_A_4y.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4y(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4y = matrix_A_4y.transpose();
		matATA_4y = matrix_A_transpose_4y * matrix_A_4y;
		Solver_ASAP_4y.compute(matATA_4y);

		matrix_A_4z.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4z(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4z = matrix_A_4z.transpose();
		matATA_4z = matrix_A_transpose_4z * matrix_A_4z;
		Solver_ASAP_4z.compute(matATA_4z);

		std::printf("Finish factorize materix A\n");

		std::printf("The compress spends %f s.\n", (double(clock() - time)) / CLOCKS_PER_SEC);

		// Fill b vector for x,y,z
		//special case of regular term
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			for (int i = 0; i < 3; i++) {

				vector_b[i](4 * tet_Num + neighborScale_face_num + constraint_ind) = m_regularScale_weight;
			}
			constraint_ind++;
		}

		// Solve x vector
		Eigen::VectorXd ATb_4x = matrix_A_transpose_4x * vector_b[0];
		vector_X_Pos_Scale[0] = Solver_ASAP_4x.solve(ATb_4x);

		Eigen::VectorXd ATb_4y = matrix_A_transpose_4y * vector_b[1];
		vector_X_Pos_Scale[1] = Solver_ASAP_4y.solve(ATb_4y);

		Eigen::VectorXd ATb_4z = matrix_A_transpose_4z * vector_b[2];
		vector_X_Pos_Scale[2] = Solver_ASAP_4z.solve(ATb_4z);

		// Update vertex
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
			int idx = node->GetIndexNo();

			double xx, yy, zz;
			node->SetCoord3D(vector_X_Pos_Scale[0](idx), vector_X_Pos_Scale[1](idx), vector_X_Pos_Scale[2](idx));
			node->GetCoord3D(xx, yy, zz);
			//std::printf("New coordinate xx %f , yy %f , zz %f.\n", xx, yy, zz);
		}

		// record the scale value
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			for (int i = 0; i < 3; i++) {
				Tetra->scaleValue_vector[i] = vector_X_Pos_Scale[i](node_Num + fdx);
				//std::cout << vector_X_Pos_Scale[i](node_Num + fdx) << " ";
			}
			//std::cout << std::endl;
		}

		std::printf("End calculation of loop %d.\n\n", loop);
	}

	std::printf("End calculation.\n");
}

void DeformTet::runASAP_StrengthReinforcement_stressLine() {

	//---------------------------------------------------------------------------------------------------------------
	// Initialize size of nodes, faces and tets
	int node_Num = tetPatch->GetNodeNumber();
	int tet_Num = tetPatch->GetTetraNumber();
	int face_Num = tetPatch->GetFaceNumber();

	int neighborScale_face_num = 0;
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL)	neighborScale_face_num++;
	}

	std::printf("All face num %d , neighborScale_face_num %d.\n", face_Num, neighborScale_face_num);
	std::printf("A matrix sixe %d x %d\n", (5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));

	//---------------------------------------------------------------------------------------------------------------
	// Define variables
	std::vector<Eigen::MatrixXd> frame_Local_new, frame_Local_initial_inverse, frame_Local_initial;
	std::vector<Eigen::VectorXd> vector_X_Pos_Scale, vector_b;
	// Initialize variables
	frame_Local_new.resize(tet_Num);
	frame_Local_initial_inverse.resize(tet_Num);
	frame_Local_initial.resize(tet_Num);
	vector_X_Pos_Scale.resize(3);
	vector_b.resize(3);

	for (int i = 0; i < 3; i++) {
		vector_X_Pos_Scale[i] = Eigen::VectorXd::Zero(node_Num + tet_Num);
		vector_b[i] = Eigen::VectorXd::Zero(5 * tet_Num + neighborScale_face_num);
	}
	std::cout << "initial ASAP matrices" << std::endl;

	//---------------------------------------------------------------------------------------------------------------
	// Pre-compute the initial frame and inverse of initla frame
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tet->GetIndexNo();
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
		double center[3] = { 0 };
		QMeshNode* nodes[4];

		for (int i = 0; i < 4; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= 4;
		for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		frame_Local_initial[fdx] = P;
		frame_Local_initial_inverse[fdx] = Eigen::MatrixXd::Zero(3, 4);
		frame_Local_initial_inverse[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();
		//std::cout << frame_Local_initial[fdx] << std::endl << std::endl;
	}
	std::printf("Finish compute local frame and its inverse!\n\n");

	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	for (int loop = 0; loop < m_loopTime; loop++) {

		/*debug
		for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
			QMeshTetra* Tetra = tetraPatch_elementSet[i];
			if (Tetra->isTensileorCompressSelect) {
				//std::cout << "\n------->this is strength ele" << std::endl;
				for (int i = 0; i < 4; i++) {
					Tetra->GetFaceRecordPtr(i + 1)->isSpecialShow = true;
				}
			}
		}
		*/

		this->_calFabricationEnergy_StrengthReinforcement(frame_Local_initial_inverse);

		for (int innerLoop = 0; innerLoop < m_innerLoopTime; innerLoop++) {

#pragma omp parallel
			{
#pragma omp for 
				// local rotation and scaling operation (frame_Local_new)
				for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
					QMeshTetra* Tetra = tetraPatch_elementSet[i];

					int fdx = Tetra->GetIndexNo();
					//add here
					if (innerLoop == 0) {

						Eigen::Vector3d center = { 0.0,0.0,0.0 };
						Tetra->CalCenterPos(center[0], center[1], center[2]);

						//This tP is each current frame in each tet.
						Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
						QMeshNode* nodes[4];
						for (int i = 0; i < 4; i++) {
							nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
							nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
						}
						for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

						Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

						T = frame_Local_initial_inverse[fdx] * tP;
						T_transpose = T.transpose();

						///// R1 //// Eigen SVD decomposition /////
						Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
						Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
						R = U * V.transpose();
						Tetra->R_estimate = R;
						frame_Local_new[fdx] = R * frame_Local_initial[fdx];
					}

					///// R2 //// Fabrication requirement /////
					if (Tetra->isTensileorCompressSelect) {
						Eigen::Vector3d principal_Stress_Dir = Tetra->R_estimate * Tetra->tau_max;

						//Eigen::Matrix3d rotationMatrix = this->_cal_rotationMatrix_strengthReinforcement_stressLine(principal_Stress_Dir, Tetra); // bar
						Eigen::Matrix3d rotationMatrix = this->_cal_rotationMatrix_strengthReinforcement(principal_Stress_Dir); // topopt
						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					}
				}
			}
			std::printf("Finish new local frame calculation!\n");

			this->_globalQuaternionSmooth1_strengthReinforcement();

			for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
				QMeshTetra* Tetra = tetraPatch_elementSet[i];
				frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
			}
			std::printf("Finish quaternion smooth calculation.\n");

			this->_get_energy_innerLoop_strengthReinforcement();
		}
		// Define variables
		Eigen::SparseMatrix<double> matrix_A_4x;
		Eigen::SparseMatrix<double> matrix_A_transpose_4x;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4x;
		//
		Eigen::SparseMatrix<double> matrix_A_4y;
		Eigen::SparseMatrix<double> matrix_A_transpose_4y;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4y;
		//
		Eigen::SparseMatrix<double> matrix_A_4z;
		Eigen::SparseMatrix<double> matrix_A_transpose_4z;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4z;
		// Initialize variables
		matrix_A_4x.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4x.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4y.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4y.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4z.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4z.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));

		// Fill A matrix for x,y,z
		// Block 11 for x,y,z
		// Important issues
		//give memory to sparse matrix, to accerate the insert speed
		matrix_A_4x.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4y.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4z.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));

		float c1 = -0.25, c2 = 0.75;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int fdx = Tetra->GetIndexNo() * 4;	int vdxArr[4];
			for (int i = 0; i < 4; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

			double weight = 1.0;
			if (Tetra->isTensileorCompressSelect)		weight = m_criticalTet_weight_SR;

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (i == j) {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c2 * weight;
					}
					else {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c1 * weight;
					}
				}
			}
		}

		// Block 12 for x,y,z
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			double weight = 1.0;
			if (Tetra->isTensileorCompressSelect)		weight = m_criticalTet_weight_SR;
			if (Tetra->isBottomTet)						weight = 155;

			for (int i = 0; i < 4; i++) {
				matrix_A_4x.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](0, i); // x row
				matrix_A_4y.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](1, i); // y row
				matrix_A_4z.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](2, i); // z row
			}
		}
		// Block 21 equals to zero
		// Block 22 for x,y,z
		int constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			constraint_ind++;
		}
		// Block 31 equals to zero
		// Block 32 for x,y,z
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			matrix_A_4x.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4y.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4z.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;

			constraint_ind++;
		}

		std::cout << "Finish fill A matrix." << std::endl;

		long time = clock();

		// Factorize A matrix
		matrix_A_4x.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4x(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4x = matrix_A_4x.transpose();
		matATA_4x = matrix_A_transpose_4x * matrix_A_4x;
		Solver_ASAP_4x.compute(matATA_4x);

		matrix_A_4y.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4y(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4y = matrix_A_4y.transpose();
		matATA_4y = matrix_A_transpose_4y * matrix_A_4y;
		Solver_ASAP_4y.compute(matATA_4y);

		matrix_A_4z.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4z(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4z = matrix_A_4z.transpose();
		matATA_4z = matrix_A_transpose_4z * matrix_A_4z;
		Solver_ASAP_4z.compute(matATA_4z);

		std::printf("Finish factorize materix A\n");

		std::printf("The compress spends %f s.\n", (double(clock() - time)) / CLOCKS_PER_SEC);

		// Fill b vector for x,y,z
		//special case of regular term
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			for (int i = 0; i < 3; i++) {

				vector_b[i](4 * tet_Num + neighborScale_face_num + constraint_ind) = m_regularScale_weight;
			}
			constraint_ind++;
		}

		// Solve x vector
		Eigen::VectorXd ATb_4x = matrix_A_transpose_4x * vector_b[0];
		vector_X_Pos_Scale[0] = Solver_ASAP_4x.solve(ATb_4x);

		Eigen::VectorXd ATb_4y = matrix_A_transpose_4y * vector_b[1];
		vector_X_Pos_Scale[1] = Solver_ASAP_4y.solve(ATb_4y);

		Eigen::VectorXd ATb_4z = matrix_A_transpose_4z * vector_b[2];
		vector_X_Pos_Scale[2] = Solver_ASAP_4z.solve(ATb_4z);

		// Update vertex
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
			int idx = node->GetIndexNo();

			double xx, yy, zz;
			node->SetCoord3D(vector_X_Pos_Scale[0](idx), vector_X_Pos_Scale[1](idx), vector_X_Pos_Scale[2](idx));
			node->GetCoord3D(xx, yy, zz);
			//std::printf("New coordinate xx %f , yy %f , zz %f.\n", xx, yy, zz);
		}

		// record the scale value
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			for (int i = 0; i < 3; i++) {
				Tetra->scaleValue_vector[i] = vector_X_Pos_Scale[i](node_Num + fdx);
				//std::cout << vector_X_Pos_Scale[i](node_Num + fdx) << " ";
			}
			//std::cout << std::endl;
		}

		std::printf("End calculation of loop %d.\n\n", loop);
	}

	std::printf("End calculation.\n");
}

// true -> select critical faces // false -> all of the boundary faces
bool DeformTet::preProcess_4SurfaceQuality(bool fucSwitch) {

	if (fucSwitch == false) { 
		this->_give_All_boundary_as_ProtectRegion(tetPatch);
		return true; 
	}
	else {
		bool defineKeptSurface = this->_selectSurfaceQualityProtectRegion(tetPatch);
		return defineKeptSurface;
	}
}

bool DeformTet::_selectSurfaceQualityProtectRegion(QMeshPatch* patch) {

	// handled node -> surface protect node
	int SurfaceProtect_NodeNum = 0;
	for (GLKPOSITION pos = patch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)patch->GetNodeList().GetNext(pos);
		node->isSurfaceProtectNode = false;
		if (node->isHandle && (!node->inner)) {
			node->isSurfaceProtectNode = true;
			SurfaceProtect_NodeNum++;
		}
	}
	if (SurfaceProtect_NodeNum == 0) {
		std::cout << "Warning: The number of selected node for surface_quality_kept is ZERO, please check." << std::endl;
		return false;
	}

	// get protected boundary faces
	for (GLKPOSITION pos = patch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(pos);
		face->isQualityProtectFace = false;
		int handleNodeCount = 0;
		for (int i = 0; i < 3; i++) {
			if (face->GetNodeRecordPtr(i)->isSurfaceProtectNode) {
				handleNodeCount++;
			}
		}
		if (handleNodeCount == 3 && (!face->inner)) face->isQualityProtectFace = true;
	}

	// remark the qualityProtect_node
	for (GLKPOSITION pos = patch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)patch->GetNodeList().GetNext(pos);
		node->isSurfaceProtectNode = false;//clean flag
	}
	for (GLKPOSITION pos = patch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(pos);
		if (face->isQualityProtectFace) {
			for (int i = 0; i < 3; i++) {
				face->GetNodeRecordPtr(i)->isSurfaceProtectNode = true;
			}
		}
	}


	// tetrahedral containing 1 critical face is defined as containProtectFace
	for (GLKPOSITION pos = patch->GetTetraList().GetHeadPosition(); pos != nullptr;) {
		QMeshTetra* tetra = (QMeshTetra*)patch->GetTetraList().GetNext(pos);
		
		tetra->containProtectSurface = false;
		int critical_Face_NUM = 0;
		for (int i = 0; i < 4; i++) {
			if (tetra->GetFaceRecordPtr(i + 1)->isQualityProtectFace) {
				critical_Face_NUM++;
			}
		}
		if (critical_Face_NUM == 1) tetra->containProtectSurface = true;
	}

	// clear handle_flag of nodes
	for (GLKPOSITION pos = patch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)patch->GetNodeList().GetNext(pos);
		node->isHandle = false;
	}
	// clear inner face Handle draw
	for (GLKPOSITION pos = patch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(pos);
		if (face->inner) {
			face->isHandleDraw = false;
		}
	}

	return true;
}

void DeformTet::_give_All_boundary_as_ProtectRegion(QMeshPatch* patch) {

	// tetrahedral containing 1 critical face is defined as containProtectFace
	for (GLKPOSITION pos = patch->GetTetraList().GetHeadPosition(); pos != nullptr;) {
		QMeshTetra* tetra = (QMeshTetra*)patch->GetTetraList().GetNext(pos);


		tetra->containProtectSurface = false;
		// get boundary face
		int boundaryFaceNum_tetEle = 0; 
		QMeshFace* boundaryFace = NULL;

		for (int i = 0; i < 4; i++) {
			if (!tetra->GetFaceRecordPtr(i + 1)->inner) {
				boundaryFace = tetra->GetFaceRecordPtr(i + 1);
				boundaryFaceNum_tetEle++;
			}
		}
		// record tet with only one boundaryFace
		if (boundaryFaceNum_tetEle == 1) {
			tetra->containProtectSurface = true;
			boundaryFace->isQualityProtectFace = true;
		}
	}

	// clear handle_flag of nodes
	for (GLKPOSITION pos = patch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)patch->GetNodeList().GetNext(pos);
		node->isHandle = false;
	}
	// clear Handle draw
	for (GLKPOSITION pos = patch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(pos);
		face->isHandleDraw = false;
	}
}

void DeformTet::runASAP_SurfaceQuality() {

	//---------------------------------------------------------------------------------------------------------------
	// Initialize size of nodes, faces and tets
	int node_Num = tetPatch->GetNodeNumber();
	int tet_Num = tetPatch->GetTetraNumber();
	int face_Num = tetPatch->GetFaceNumber();

	int neighborScale_face_num = 0;
	int critical_tet_num = 0;
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos != nullptr;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		if (Tet->containProtectSurface)	critical_tet_num++;
	}
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL)	neighborScale_face_num++;
	}

	std::printf("All tet_Num %d , critical_tet_num %d.\n", tet_Num, critical_tet_num);
	std::printf("All face num %d , neighborScale_face_num %d.\n", face_Num, neighborScale_face_num);
	std::printf("A matrix sixe %d x %d\n", (5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));

	//---------------------------------------------------------------------------------------------------------------
	// Define variables
	std::vector<Eigen::MatrixXd> frame_Local_new, frame_Local_initial_inverse, frame_Local_initial;
	std::vector<Eigen::VectorXd> vector_X_Pos_Scale, vector_b;
	// Initialize variables
	frame_Local_new.resize(tet_Num);
	frame_Local_initial_inverse.resize(tet_Num);
	frame_Local_initial.resize(tet_Num);
	vector_X_Pos_Scale.resize(3);
	vector_b.resize(3);

	for (int i = 0; i < 3; i++) {
		vector_X_Pos_Scale[i] = Eigen::VectorXd::Zero(node_Num + tet_Num);
		vector_b[i] = Eigen::VectorXd::Zero(5 * tet_Num + neighborScale_face_num);
	}
	std::cout << "initial ASAP matrices" << std::endl;

	//---------------------------------------------------------------------------------------------------------------
	// Pre-compute the initial frame and inverse of initla frame
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tet->GetIndexNo();
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
		double center[3] = { 0 };
		QMeshNode* nodes[4];

		for (int i = 0; i < 4; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= 4;
		for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		frame_Local_initial[fdx] = P;
		frame_Local_initial_inverse[fdx] = Eigen::MatrixXd::Zero(3, 4);
		frame_Local_initial_inverse[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();
		//std::cout << frame_Local_initial[fdx] << std::endl << std::endl;
	}
	std::printf("Finish compute local frame and its inverse!\n\n");

	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	for (int loop = 0; loop < m_loopTime; loop++) {

		this->_calFabricationEnergy_SurfaceQuality();

#pragma omp parallel
		{
#pragma omp for 
			// local rotation and scaling operation (frame_Local_new)
			for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
				QMeshTetra* Tetra = tetraPatch_elementSet[i];

				int fdx = Tetra->GetIndexNo();
				double center[3] = { 0 };
				Tetra->CalCenterPos(center[0], center[1], center[2]);

				//This tP is each current frame in each tet.
				Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
				QMeshNode* nodes[4];
				for (int i = 0; i < 4; i++) {
					nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
					nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
				}
				for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

				Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
				Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
				Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

				T = frame_Local_initial_inverse[fdx] * tP;
				T_transpose = T.transpose();

				///// R1 //// Eigen SVD decomposition /////
				Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
				Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
				R = U * V.transpose();
				Tetra->R_estimate = R;
				frame_Local_new[fdx] = R * frame_Local_initial[fdx];

				///// R2 //// Fabrication requirement /////
				if (Tetra->containProtectSurface) {
					QMeshFace* criticalFace = NULL;
					for (int i = 0; i < 4; i++) {
						QMeshFace* thisFace = Tetra->GetFaceRecordPtr(i + 1);
						if (thisFace->isQualityProtectFace) {
							criticalFace = thisFace;
							break;
						}
					}
					if (criticalFace == NULL) std::cout << "ERROR! No quality protect face" << std::endl;

					Eigen::Vector3d normal;
					criticalFace->CalPlaneEquation();
					criticalFace->GetNormal(normal(0), normal(1), normal(2)); // the normal of last time deform

					Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
					if (normal.dot(printDir) > 0)
						rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(normal, this->printDir);
					else
						rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(-normal,this->printDir);

					//std::cout << "rotation matrix: \n" << rotationMatrix << std::endl;
					Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
					frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
				}
			}
		}
		std::printf("Finish new local frame calculation!\n");

		this->_globalQuaternionSmooth1();
		for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
			QMeshTetra* Tetra = tetraPatch_elementSet[i];
			frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
		}
		std::printf("Finish quaternion smooth calculation.\n");

		// Define variables
		Eigen::SparseMatrix<double> matrix_A_4x;
		Eigen::SparseMatrix<double> matrix_A_transpose_4x;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4x;
		//
		Eigen::SparseMatrix<double> matrix_A_4y;
		Eigen::SparseMatrix<double> matrix_A_transpose_4y;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4y;
		//
		Eigen::SparseMatrix<double> matrix_A_4z;
		Eigen::SparseMatrix<double> matrix_A_transpose_4z;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4z;
		// Initialize variables
		matrix_A_4x.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4x.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4y.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4y.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4z.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4z.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));

		// Fill A matrix for x,y,z
		// Block 11 for x,y,z
		// Important issues
		//give memory to sparse matrix, to accerate the insert speed
		matrix_A_4x.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4y.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4z.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));

		float c1 = -0.25, c2 = 0.75;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int fdx = Tetra->GetIndexNo() * 4;	int vdxArr[4];
			for (int i = 0; i < 4; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

			double weight = 1.0;
			if (Tetra->containProtectSurface)
				weight = m_criticalTet_weight_SQ_cover;

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (i == j) {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c2 * weight;
					}
					else {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c1 * weight;
					}
				}
			}
		}

		// Block 12 for x,y,z
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			double weight = 1.0;
			if (Tetra->containProtectSurface)
				weight = m_criticalTet_weight_SQ_cover;

			for (int i = 0; i < 4; i++) {
				matrix_A_4x.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](0, i); // x row
				matrix_A_4y.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](1, i); // y row
				matrix_A_4z.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](2, i); // z row
			}
		}
		// Block 21 equals to zero
		// Block 22 for x,y,z
		int constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			constraint_ind++;
		}
		// Block 31 equals to zero
		// Block 32 for x,y,z
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			matrix_A_4x.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4y.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4z.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;

			constraint_ind++;
		}

		std::cout << "Finish fill A matrix." << std::endl;

		long time = clock();

		// Factorize A matrix
		matrix_A_4x.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4x(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4x = matrix_A_4x.transpose();
		matATA_4x = matrix_A_transpose_4x * matrix_A_4x;
		Solver_ASAP_4x.compute(matATA_4x);

		matrix_A_4y.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4y(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4y = matrix_A_4y.transpose();
		matATA_4y = matrix_A_transpose_4y * matrix_A_4y;
		Solver_ASAP_4y.compute(matATA_4y);

		matrix_A_4z.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4z(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4z = matrix_A_4z.transpose();
		matATA_4z = matrix_A_transpose_4z * matrix_A_4z;
		Solver_ASAP_4z.compute(matATA_4z);

		std::printf("Finish factorize materix A\n");

		std::printf("The compress spends %f s.\n", (double(clock() - time)) / CLOCKS_PER_SEC);

		// Fill b vector for x,y,z
		//special case of regular term
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			for (int i = 0; i < 3; i++) {

				vector_b[i](4 * tet_Num + neighborScale_face_num + constraint_ind) = m_regularScale_weight;
			}
			constraint_ind++;
		}

		// Solve x vector
		Eigen::VectorXd ATb_4x = matrix_A_transpose_4x * vector_b[0];
		vector_X_Pos_Scale[0] = Solver_ASAP_4x.solve(ATb_4x);

		Eigen::VectorXd ATb_4y = matrix_A_transpose_4y * vector_b[1];
		vector_X_Pos_Scale[1] = Solver_ASAP_4y.solve(ATb_4y);

		Eigen::VectorXd ATb_4z = matrix_A_transpose_4z * vector_b[2];
		vector_X_Pos_Scale[2] = Solver_ASAP_4z.solve(ATb_4z);

		// Update vertex
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
			int idx = node->GetIndexNo();

			double xx, yy, zz;
			node->SetCoord3D(vector_X_Pos_Scale[0](idx), vector_X_Pos_Scale[1](idx), vector_X_Pos_Scale[2](idx));
			node->GetCoord3D(xx, yy, zz);
			//std::printf("New coordinate xx %f , yy %f , zz %f.\n", xx, yy, zz);
		}

		// record the scale value
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			for (int i = 0; i < 3; i++) {
				Tetra->scaleValue_vector[i] = vector_X_Pos_Scale[i](node_Num + fdx);
				//std::cout << vector_X_Pos_Scale[i](node_Num + fdx) << " ";
			}
			//std::cout << std::endl;
		}

		std::printf("End calculation of loop %d.\n\n", loop);
	}

	std::printf("End calculation.\n");
}

void DeformTet::runASAP_SurfaceQuality_test() {

	//---------------------------------------------------------------------------------------------------------------
	// Initialize size of nodes, faces and tets
	int node_Num = tetPatch->GetNodeNumber();
	int tet_Num = tetPatch->GetTetraNumber();
	int face_Num = tetPatch->GetFaceNumber();

	int neighborScale_face_num = 0;
	int critical_tet_num = 0;
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos != nullptr;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		if (Tet->containProtectSurface)	critical_tet_num++;
	}
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL)	neighborScale_face_num++;
	}

	std::printf("All tet_Num %d , critical_tet_num %d.\n", tet_Num, critical_tet_num);
	std::printf("All face num %d , neighborScale_face_num %d.\n", face_Num, neighborScale_face_num);
	std::printf("A matrix sixe %d x %d\n", (5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));

	//---------------------------------------------------------------------------------------------------------------
	// Define variables
	std::vector<Eigen::MatrixXd> frame_Local_new, frame_Local_initial_inverse, frame_Local_initial;
	std::vector<Eigen::VectorXd> vector_X_Pos_Scale, vector_b;
	// Initialize variables
	frame_Local_new.resize(tet_Num);
	frame_Local_initial_inverse.resize(tet_Num);
	frame_Local_initial.resize(tet_Num);
	vector_X_Pos_Scale.resize(3);
	vector_b.resize(3);

	for (int i = 0; i < 3; i++) {
		vector_X_Pos_Scale[i] = Eigen::VectorXd::Zero(node_Num + tet_Num);
		vector_b[i] = Eigen::VectorXd::Zero(5 * tet_Num + neighborScale_face_num);
	}
	std::cout << "initial ASAP matrices" << std::endl;

	//---------------------------------------------------------------------------------------------------------------
	// Pre-compute the initial frame and inverse of initla frame
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tet->GetIndexNo();
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
		double center[3] = { 0 };
		QMeshNode* nodes[4];

		for (int i = 0; i < 4; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= 4;
		for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		frame_Local_initial[fdx] = P;
		frame_Local_initial_inverse[fdx] = Eigen::MatrixXd::Zero(3, 4);
		frame_Local_initial_inverse[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();
		//std::cout << frame_Local_initial[fdx] << std::endl << std::endl;
	}
	std::printf("Finish compute local frame and its inverse!\n\n");

	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	for (int loop = 0; loop < m_loopTime; loop++) {

		this->_calFabricationEnergy_SurfaceQuality();

		for (int innerLoop = 0; innerLoop < m_innerLoopTime; innerLoop++) {

#pragma omp parallel
			{
#pragma omp for 
				// local rotation and scaling operation (frame_Local_new)
				for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
					QMeshTetra* Tetra = tetraPatch_elementSet[i];

					int fdx = Tetra->GetIndexNo();
					if (innerLoop == 0) {
						double center[3] = { 0 };
						Tetra->CalCenterPos(center[0], center[1], center[2]);

						//This tP is each current frame in each tet.
						Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
						QMeshNode* nodes[4];
						for (int i = 0; i < 4; i++) {
							nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
							nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
						}
						for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

						Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

						T = frame_Local_initial_inverse[fdx] * tP;
						T_transpose = T.transpose();

						///// R1 //// Eigen SVD decomposition /////
						Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
						Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
						R = U * V.transpose();
						Tetra->R_estimate = R;
						frame_Local_new[fdx] = R * frame_Local_initial[fdx];
					}
					///// R2 //// Fabrication requirement /////
					if (Tetra->containProtectSurface) {
						QMeshFace* criticalFace = NULL;
						for (int i = 0; i < 4; i++) {
							QMeshFace* thisFace = Tetra->GetFaceRecordPtr(i + 1);
							if (thisFace->isQualityProtectFace) {
								criticalFace = thisFace;
								break;
							}
						}
						if (criticalFace == NULL) std::cout << "ERROR! No quality protect face" << std::endl;

						Eigen::Vector3d normal;
						normal = criticalFace->normal_last; // initial face normal

						// virtual face normal after applying R_estimate
						Eigen::Vector3d face_Normal_Dir = Tetra->R_estimate* normal; 

						Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
						if (face_Normal_Dir.dot(printDir) > 0)
							rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(face_Normal_Dir, this->printDir);
						else
							rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(-face_Normal_Dir, this->printDir);

						//user select case:
						criticalFace->select_coverLayer = 2;
						Tetra->select_coverLayer = 2;
						//std::cout << "rotation matrix: \n" << rotationMatrix << std::endl;
						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					}
				}
			}
			std::printf("Finish new local frame calculation!\n");

			this->_globalQuaternionSmooth1_surfaceQuality();
			for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
				QMeshTetra* Tetra = tetraPatch_elementSet[i];
				frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
			}
			std::printf("Finish quaternion smooth calculation.\n");

			this->_get_energy_innerLoop_surfaceQuality();
		}
		// Define variables
		Eigen::SparseMatrix<double> matrix_A_4x;
		Eigen::SparseMatrix<double> matrix_A_transpose_4x;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4x;
		//
		Eigen::SparseMatrix<double> matrix_A_4y;
		Eigen::SparseMatrix<double> matrix_A_transpose_4y;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4y;
		//
		Eigen::SparseMatrix<double> matrix_A_4z;
		Eigen::SparseMatrix<double> matrix_A_transpose_4z;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4z;
		// Initialize variables
		matrix_A_4x.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4x.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4y.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4y.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4z.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4z.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));

		// Fill A matrix for x,y,z
		// Block 11 for x,y,z
		// Important issues
		//give memory to sparse matrix, to accerate the insert speed
		matrix_A_4x.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4y.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4z.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));

		float c1 = -0.25, c2 = 0.75;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int fdx = Tetra->GetIndexNo() * 4;	int vdxArr[4];
			for (int i = 0; i < 4; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

			double weight = 1.0;
			if (Tetra->containProtectSurface)
				weight = m_criticalTet_weight_SQ_cover;

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (i == j) {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c2 * weight;
					}
					else {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c1 * weight;
					}
				}
			}
		}

		// Block 12 for x,y,z
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			double weight = 1.0;
			if (Tetra->containProtectSurface)
				weight = m_criticalTet_weight_SQ_cover;

			for (int i = 0; i < 4; i++) {
				matrix_A_4x.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](0, i); // x row
				matrix_A_4y.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](1, i); // y row
				matrix_A_4z.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](2, i); // z row
			}
		}
		// Block 21 equals to zero
		// Block 22 for x,y,z
		int constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			constraint_ind++;
		}
		// Block 31 equals to zero
		// Block 32 for x,y,z
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			matrix_A_4x.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4y.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4z.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;

			constraint_ind++;
		}

		std::cout << "Finish fill A matrix." << std::endl;

		long time = clock();

		// Factorize A matrix
		matrix_A_4x.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4x(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4x = matrix_A_4x.transpose();
		matATA_4x = matrix_A_transpose_4x * matrix_A_4x;
		Solver_ASAP_4x.compute(matATA_4x);

		matrix_A_4y.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4y(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4y = matrix_A_4y.transpose();
		matATA_4y = matrix_A_transpose_4y * matrix_A_4y;
		Solver_ASAP_4y.compute(matATA_4y);

		matrix_A_4z.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4z(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4z = matrix_A_4z.transpose();
		matATA_4z = matrix_A_transpose_4z * matrix_A_4z;
		Solver_ASAP_4z.compute(matATA_4z);

		std::printf("Finish factorize materix A\n");

		std::printf("The compress spends %f s.\n", (double(clock() - time)) / CLOCKS_PER_SEC);

		// Fill b vector for x,y,z
		//special case of regular term
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			for (int i = 0; i < 3; i++) {

				vector_b[i](4 * tet_Num + neighborScale_face_num + constraint_ind) = m_regularScale_weight;
			}
			constraint_ind++;
		}

		// Solve x vector
		Eigen::VectorXd ATb_4x = matrix_A_transpose_4x * vector_b[0];
		vector_X_Pos_Scale[0] = Solver_ASAP_4x.solve(ATb_4x);

		Eigen::VectorXd ATb_4y = matrix_A_transpose_4y * vector_b[1];
		vector_X_Pos_Scale[1] = Solver_ASAP_4y.solve(ATb_4y);

		Eigen::VectorXd ATb_4z = matrix_A_transpose_4z * vector_b[2];
		vector_X_Pos_Scale[2] = Solver_ASAP_4z.solve(ATb_4z);

		// Update vertex
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
			int idx = node->GetIndexNo();

			double xx, yy, zz;
			node->SetCoord3D(vector_X_Pos_Scale[0](idx), vector_X_Pos_Scale[1](idx), vector_X_Pos_Scale[2](idx));
			node->GetCoord3D(xx, yy, zz);
			//std::printf("New coordinate xx %f , yy %f , zz %f.\n", xx, yy, zz);
		}

		// record the scale value
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			for (int i = 0; i < 3; i++) {
				Tetra->scaleValue_vector[i] = vector_X_Pos_Scale[i](node_Num + fdx);
				//std::cout << vector_X_Pos_Scale[i](node_Num + fdx) << " ";
			}
			//std::cout << std::endl;
		}

		std::printf("End calculation of loop %d.\n\n", loop);
	}

	std::printf("End calculation.\n");
}

void DeformTet::runASAP_SurfaceQuality_test2() {

	//---------------------------------------------------------------------------------------------------------------
	// Initialize size of nodes, faces and tets
	int node_Num = tetPatch->GetNodeNumber();
	int tet_Num = tetPatch->GetTetraNumber();
	int face_Num = tetPatch->GetFaceNumber();

	int neighborScale_face_num = 0;
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL)	neighborScale_face_num++;
	}

	std::printf("All face num %d , neighborScale_face_num %d.\n", face_Num, neighborScale_face_num);
	std::printf("A matrix sixe %d x %d\n", (5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));

	//---------------------------------------------------------------------------------------------------------------
	// Define variables
	std::vector<Eigen::MatrixXd> frame_Local_new, frame_Local_initial_inverse, frame_Local_initial;
	std::vector<Eigen::VectorXd> vector_X_Pos_Scale, vector_b;
	// Initialize variables
	frame_Local_new.resize(tet_Num);
	frame_Local_initial_inverse.resize(tet_Num);
	frame_Local_initial.resize(tet_Num);
	vector_X_Pos_Scale.resize(3);
	vector_b.resize(3);

	for (int i = 0; i < 3; i++) {
		vector_X_Pos_Scale[i] = Eigen::VectorXd::Zero(node_Num + tet_Num);
		vector_b[i] = Eigen::VectorXd::Zero(5 * tet_Num + neighborScale_face_num);
	}
	std::cout << "initial ASAP matrices" << std::endl;

	//---------------------------------------------------------------------------------------------------------------
	// Pre-compute the initial frame and inverse of initla frame
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tet->GetIndexNo();
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
		double center[3] = { 0 };
		QMeshNode* nodes[4];

		for (int i = 0; i < 4; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= 4;
		for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		frame_Local_initial[fdx] = P;
		frame_Local_initial_inverse[fdx] = Eigen::MatrixXd::Zero(3, 4);
		frame_Local_initial_inverse[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();
		//std::cout << frame_Local_initial[fdx] << std::endl << std::endl;
	}
	std::printf("Finish compute local frame and its inverse!\n\n");

	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	for (int loop = 0; loop < m_loopTime; loop++) {

		this->_calFabricationEnergy_SurfaceQuality();

		for (int innerLoop = 0; innerLoop < m_innerLoopTime; innerLoop++) {

#pragma omp parallel
			{
#pragma omp for 
				// local rotation and scaling operation (frame_Local_new)
				for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
					QMeshTetra* Tetra = tetraPatch_elementSet[i];

					int fdx = Tetra->GetIndexNo();
					if (innerLoop == 0) {
						double center[3] = { 0 };
						Tetra->CalCenterPos(center[0], center[1], center[2]);

						//This tP is each current frame in each tet.
						Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
						QMeshNode* nodes[4];
						for (int i = 0; i < 4; i++) {
							nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
							nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
						}
						for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

						Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

						T = frame_Local_initial_inverse[fdx] * tP;
						T_transpose = T.transpose();

						///// R1 //// Eigen SVD decomposition /////
						Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
						Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
						R = U * V.transpose();
						Tetra->R_estimate = R;
						frame_Local_new[fdx] = R * frame_Local_initial[fdx];
					}
					
					///// R2 //// Fabrication requirement /////
					if (Tetra->containProtectSurface) {

						QMeshFace* criticalFace = this->_get_initial_kept_face(Tetra);
						Eigen::Vector3d normal = criticalFace->normal_last.normalized(); // initial face normal
						normal = Tetra->R_estimate * normal;// virtual face normal after applying R_estimate

						if (fabs(normal(1)) > m_bandWidth_SQ_rotation) { // ny out of band

							Eigen::Matrix3d rotationMatrix = this->_cal_rotationMatrix_surfaceQuality(normal, Tetra, criticalFace);
							Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
							frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
						}
					}
				}
			}
			std::printf("Finish new local frame calculation!\n");

			this->_globalQuaternionSmooth1_surfaceQuality();
			//this->_globalQuaternionSmooth4();
			for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
				QMeshTetra* Tetra = tetraPatch_elementSet[i];
				frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
			}
			std::printf("Finish quaternion smooth calculation.\n");

			this->_get_energy_innerLoop_surfaceQuality();

			if (innerLoop == (m_innerLoopTime - 3)) {
				this->m_keep_SQ_cover = 10;
				std::cout << "Increase the quaternion keep weight in innerloop " << innerLoop << " as " << this->m_keep_SQ_cover << std::endl;
			}

			//clear select_coverLayer flag, which is useful only for smooth4 and Smooth1_surfaceQuality();
			
			//if (innerLoop < m_loopTime) {
			//	for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
			//		QMeshTetra* Tetra = tetraPatch_elementSet[i];
			//		Tetra->select_coverLayer = 0;
			//		for (int j = 0; j < 4; j++) {
			//			Tetra->GetFaceRecordPtr(j + 1)->select_coverLayer = 0;
			//		}
			//	}
			//}
			
		}
		// Define variables
		Eigen::SparseMatrix<double> matrix_A_4x;
		Eigen::SparseMatrix<double> matrix_A_transpose_4x;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4x;
		//
		Eigen::SparseMatrix<double> matrix_A_4y;
		Eigen::SparseMatrix<double> matrix_A_transpose_4y;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4y;
		//
		Eigen::SparseMatrix<double> matrix_A_4z;
		Eigen::SparseMatrix<double> matrix_A_transpose_4z;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4z;
		// Initialize variables
		matrix_A_4x.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4x.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4y.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4y.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4z.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4z.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));

		// Fill A matrix for x,y,z
		// Block 11 for x,y,z
		// Important issues
		//give memory to sparse matrix, to accerate the insert speed
		matrix_A_4x.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4y.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4z.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));

		float c1 = -0.25, c2 = 0.75;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int fdx = Tetra->GetIndexNo() * 4;	int vdxArr[4];
			for (int i = 0; i < 4; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

			double weight = 1.0;
			if (Tetra->containProtectSurface) {
				if (Tetra->select_coverLayer == 2)	weight = m_criticalTet_weight_SQ_cover;
				else if (Tetra->select_coverLayer == 1)	weight = m_criticalTet_weight_SQ_vertical;
				else weight = 0.01;
			}
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (i == j) {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c2 * weight;
					}
					else {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c1 * weight;
					}
				}
			}
		}

		// Block 12 for x,y,z
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			double weight = 1.0;
			if (Tetra->containProtectSurface) {
				if (Tetra->select_coverLayer == 2)	weight = m_criticalTet_weight_SQ_cover;
				else if (Tetra->select_coverLayer == 1)	weight = m_criticalTet_weight_SQ_vertical;
				else weight = 0.01;
			}

			for (int i = 0; i < 4; i++) {
				matrix_A_4x.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](0, i); // x row
				matrix_A_4y.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](1, i); // y row
				matrix_A_4z.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](2, i); // z row
			}
		}
		// Block 21 equals to zero
		// Block 22 for x,y,z
		int constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			constraint_ind++;
		}
		// Block 31 equals to zero
		// Block 32 for x,y,z
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			matrix_A_4x.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4y.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4z.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;

			constraint_ind++;
		}

		std::cout << "Finish fill A matrix." << std::endl;

		long time = clock();

		// Factorize A matrix
		matrix_A_4x.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4x(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4x = matrix_A_4x.transpose();
		matATA_4x = matrix_A_transpose_4x * matrix_A_4x;
		Solver_ASAP_4x.compute(matATA_4x);

		matrix_A_4y.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4y(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4y = matrix_A_4y.transpose();
		matATA_4y = matrix_A_transpose_4y * matrix_A_4y;
		Solver_ASAP_4y.compute(matATA_4y);

		matrix_A_4z.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4z(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4z = matrix_A_4z.transpose();
		matATA_4z = matrix_A_transpose_4z * matrix_A_4z;
		Solver_ASAP_4z.compute(matATA_4z);

		std::printf("Finish factorize materix A\n");

		std::printf("The compress spends %f s.\n", (double(clock() - time)) / CLOCKS_PER_SEC);

		// Fill b vector for x,y,z
		//special case of regular term
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			for (int i = 0; i < 3; i++) {

				vector_b[i](4 * tet_Num + neighborScale_face_num + constraint_ind) = m_regularScale_weight;
			}
			constraint_ind++;
		}

		// Solve x vector
		Eigen::VectorXd ATb_4x = matrix_A_transpose_4x * vector_b[0];
		vector_X_Pos_Scale[0] = Solver_ASAP_4x.solve(ATb_4x);

		Eigen::VectorXd ATb_4y = matrix_A_transpose_4y * vector_b[1];
		vector_X_Pos_Scale[1] = Solver_ASAP_4y.solve(ATb_4y);

		Eigen::VectorXd ATb_4z = matrix_A_transpose_4z * vector_b[2];
		vector_X_Pos_Scale[2] = Solver_ASAP_4z.solve(ATb_4z);

		// Update vertex
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
			int idx = node->GetIndexNo();

			double xx, yy, zz;
			node->SetCoord3D(vector_X_Pos_Scale[0](idx), vector_X_Pos_Scale[1](idx), vector_X_Pos_Scale[2](idx));
			node->GetCoord3D(xx, yy, zz);
			//std::printf("New coordinate xx %f , yy %f , zz %f.\n", xx, yy, zz);
		}

		// record the scale value
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			for (int i = 0; i < 3; i++) {
				Tetra->scaleValue_vector[i] = vector_X_Pos_Scale[i](node_Num + fdx);
				//std::cout << vector_X_Pos_Scale[i](node_Num + fdx) << " ";
			}
			//std::cout << std::endl;
		}

		std::printf("End calculation of loop %d.\n\n", loop);
	}

	std::printf("End calculation.\n");
}

void DeformTet::_calFabricationEnergy_SurfaceQuality() {

	//Get the fabrication energy of surface quality
	double critical_energy = 0.0;
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		if (Tetra->containProtectSurface) {

			QMeshFace* criticalFace = NULL;
			for (int i = 0; i < 4; i++) {
				QMeshFace* thisFace = Tetra->GetFaceRecordPtr(i + 1);
				if (thisFace->isQualityProtectFace) {
					criticalFace = thisFace;
					break;
				}
			}
			if (criticalFace == NULL) std::cout << "ERROR! No quality protect face." << std::endl;

			Eigen::Vector3d normal;
			criticalFace->CalPlaneEquation();
			criticalFace->GetNormal(normal(0), normal(1), normal(2));

			//std::cout << "critcal normal" << normal.transpose() << std::endl;
			if (normal.dot(this->printDir) > 0)	critical_energy += acos(normal.dot(printDir));
			else critical_energy += acos(-normal.dot(this->printDir));
			
		}
	}
	std::cout << "Fabrication energy (SurfaceQuality) = " << critical_energy << std::endl;
}

void DeformTet::_get_energy_innerLoop_surfaceQuality() {
	//Get the fabrication energy of surface quality
	double critical_energy = 0.0;
	int critical_face_num = 0;
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		if (Tetra->containProtectSurface) {

			QMeshFace* criticalFace = NULL;
			for (int i = 0; i < 4; i++) {
				QMeshFace* thisFace = Tetra->GetFaceRecordPtr(i + 1);
				if (thisFace->isQualityProtectFace) {
					criticalFace = thisFace;
					break;
				}
			}
			if (criticalFace == NULL) std::cout << "ERROR! No quality protect face." << std::endl;

			Eigen::Vector3d normal;
			normal = criticalFace->normal_last; // initial face normal
			Eigen::Vector3d face_Normal_Dir = Tetra->R_estimate * normal;
			if (Tetra->select_coverLayer == 2) {
				critical_face_num++;
				double each_critical_energy = 0.0;
				if (normal.dot(this->printDir) > 0) {
					each_critical_energy = acos(face_Normal_Dir.dot(printDir));
					critical_energy += each_critical_energy;
				}
				else {
					each_critical_energy = acos(-face_Normal_Dir.dot(this->printDir));
					critical_energy += each_critical_energy;
				}

				//std::cout << "Critical_energy of Tetra " << Tetra->GetIndexNo() << " is " << each_critical_energy << std::endl;
			}
		}
	}
	std::cout << critical_face_num << "___------___------" << critical_energy << std::endl;
	std::cout << "Fabrication energy (SurfaceQuality) = " << (double)(critical_energy / critical_face_num * 180 / PI) << std::endl;
}

void DeformTet::runASAP_Hybrid_SL_SQ() {

	this->_check_parameters();
	//---------------------------------------------------------------------------------------------------------------
	// Initialize size of nodes, faces and tets
	int node_Num = tetPatch->GetNodeNumber();
	int tet_Num = tetPatch->GetTetraNumber();
	int face_Num = tetPatch->GetFaceNumber();

	int neighborScale_face_num = 0;
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL)	neighborScale_face_num++;
	}

	std::printf("All face num %d , neighborScale_face_num %d.\n", face_Num, neighborScale_face_num);
	std::printf("A matrix sixe %d x %d\n", (5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));

	//---------------------------------------------------------------------------------------------------------------
	// Define variables
	std::vector<Eigen::MatrixXd> frame_Local_new, frame_Local_initial_inverse, frame_Local_initial;
	std::vector<Eigen::VectorXd> vector_X_Pos_Scale, vector_b;
	// Initialize variables
	frame_Local_new.resize(tet_Num);
	frame_Local_initial_inverse.resize(tet_Num);
	frame_Local_initial.resize(tet_Num);
	vector_X_Pos_Scale.resize(3);
	vector_b.resize(3);

	for (int i = 0; i < 3; i++) {
		vector_X_Pos_Scale[i] = Eigen::VectorXd::Zero(node_Num + tet_Num);
		vector_b[i] = Eigen::VectorXd::Zero(5 * tet_Num + neighborScale_face_num);
	}
	std::cout << "Finish initializing ASAP matrices" << std::endl;

	//---------------------------------------------------------------------------------------------------------------
	// Pre-compute the initial frame and inverse of initial frame
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tet->GetIndexNo();
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
		double center[3] = { 0 };
		QMeshNode* nodes[4];

		for (int i = 0; i < 4; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= 4;
		for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		frame_Local_initial[fdx] = P;
		frame_Local_initial_inverse[fdx] = Eigen::MatrixXd::Zero(3, 4);
		frame_Local_initial_inverse[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();
		//std::cout << frame_Local_initial[fdx] << std::endl << std::endl;
	}
	std::printf("Finish computing local frame and its inverse!\n\n");

	//this->_detectOverhangFace();
	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	for (int loop = 0; loop < m_loopTime; loop++) {

		//if (loop % 5 == 0)	this->_detectOverhangFace();
		this->_detectOverhangFace();

		//Calculate the fabrication energy of SupportLess case ()
		this->_calFabricationEnergy_SupportLess();

		//update the last coordinate position
		/*if(loop == 4)
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

			double xx, yy, zz;
			Node->GetCoord3D(xx, yy, zz);
			Node->SetCoord3D_last(xx, yy, zz);

		}*/

		for (int innerLoop = 0; innerLoop < m_innerLoopTime; innerLoop++) {

#pragma omp parallel
			{
#pragma omp for 
				// local rotation and scaling operation (frame_Local_new)
				for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
					QMeshTetra* Tetra = tetraPatch_elementSet[i];

					int fdx = Tetra->GetIndexNo();
					if (innerLoop == 0) {
						double center[3] = { 0.0 };
						Tetra->CalCenterPos(center[0], center[1], center[2]);

						//This tP is each current frame in each tet.
						Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
						QMeshNode* nodes[4];
						for (int i = 0; i < 4; i++) {
							nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
							nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
						}
						for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

						Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

						T = frame_Local_initial_inverse[fdx] * tP;
						T_transpose = T.transpose();

						///// R1 //// Eigen SVD decomposition /////
						Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
						Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
						R = U * V.transpose();
						Tetra->R_estimate = R;
						frame_Local_new[fdx] = R * frame_Local_initial[fdx];
						Tetra->vectorField_4voxelOrder_now = Tetra->R_estimate * Tetra->vectorField_4voxelOrder;
					}

					///// R2 //// Fabrication requirement (support less) /////
					//if (Tetra->isOverhangTet && (loop < 4)) {
					if (Tetra->isOverhangTet) {

						Eigen::Vector3d whole_overhang_face_normal = this->_get_initial_overhang_faceNormal(Tetra);
						/*
						// get current face normal
						//QMeshFace* eachFace = NULL;		int overhangFace_Num = 0;
						//Eigen::Vector3d whole_overhang_face_normal = Eigen::Vector3d::Zero();
						//for (int i = 0; i < 4; i++) {
						//	eachFace = Tetra->GetFaceRecordPtr(i + 1);
						//	if (eachFace->isOverhangFace) {

						//		Eigen::Vector3d normal;
						//		//eachFace->CalPlaneEquation();
						//		//eachFace->GetNormal(normal(0), normal(1), normal(2));
						//		normal = eachFace->normal_last; // initial face normal
						//		normal = -normal;
						//		normal.normalize();
						//		whole_overhang_face_normal += normal;
						//		overhangFace_Num++;
						//	}
						//}
						//if (overhangFace_Num == 0) std::cout << "ERROR: none of overhang face is detected!" << std::endl;
						//whole_overhang_face_normal.normalize();
						*/

						// virtual face normal after applying R_estimate
						Eigen::Vector3d face_Normal_Dir = Tetra->R_estimate * whole_overhang_face_normal;
						Tetra->vectorField_4voxelOrder_now = Tetra->R_estimate * Tetra->vectorField_4voxelOrder;

						Eigen::Matrix3d rotationMatrix = _cal_rotationMatrix_supportLess(face_Normal_Dir, Tetra);
						/*
						Eigen::Vector3d r_axis = face_Normal_Dir.cross(Tetra->vectorField_4voxelOrder_now.normalized());
						r_axis.normalize();// OMG!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
						Eigen::AngleAxis<double> sptFree_rotation_1
							= Eigen::AngleAxis<double>((this->m_supportFreeAngle + 90.0) * PI / 180.0, r_axis);
						Eigen::Vector3d sptFree_normal_1 = sptFree_rotation_1.matrix() * face_Normal_Dir;
						Eigen::AngleAxis<double> sptFree_rotation_2
							= Eigen::AngleAxis<double>(-1.0 * (this->m_supportFreeAngle + 90.0) * PI / 180.0, r_axis);
						Eigen::Vector3d sptFree_normal_2 = sptFree_rotation_2.matrix() * face_Normal_Dir;

						Eigen::Vector3d sptFree_normal = Eigen::Vector3d::Zero();
						double angel_n_nCandidate1 = acos(sptFree_normal_1.dot(Tetra->vectorField_4voxelOrder_now.normalized()));
						double angel_n_nCandidate2 = acos(sptFree_normal_2.dot(Tetra->vectorField_4voxelOrder_now.normalized()));

						if (angel_n_nCandidate1 < angel_n_nCandidate2) { sptFree_normal = sptFree_normal_1; }
						else { sptFree_normal = sptFree_normal_2; }

						Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
						
						rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(sptFree_normal, this->printDir);
						*/

						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					}

					///// R2 //// Fabrication requirement (surface quality) /////
					//if (Tetra->containProtectSurface && (loop >= 4)) {
					if (Tetra->containProtectSurface) {
				
						QMeshFace* criticalFace = this->_get_initial_kept_face(Tetra);
						Eigen::Vector3d normal = criticalFace->normal_last.normalized(); // initial face normal
						/*
						QMeshFace* criticalFace = NULL;
						for (int i = 0; i < 4; i++) {
							QMeshFace* thisFace = Tetra->GetFaceRecordPtr(i + 1);
							if (thisFace->isQualityProtectFace) {
								criticalFace = thisFace;
								break;
							}
						}
						if (criticalFace == NULL) std::cout << "ERROR! No quality protect face" << std::endl;

						Eigen::Vector3d normal = criticalFace->normal_last.normalized(); // initial face normal
						*/
						
						normal = Tetra->R_estimate * normal;// virtual face normal after applying R_estimate

						if (fabs(normal(1)) > m_bandWidth_SQ_rotation) { // ny out of band

							Eigen::Matrix3d rotationMatrix = this->_cal_rotationMatrix_surfaceQuality(normal, Tetra, criticalFace);
							/*
							double bandWidth = m_bandWidth_SQ_rotation;
							// angle between (0 1 0) and face normal
							double angle_dp_nf = 0.0;
							if (normal(1) > 0)	angle_dp_nf = acos(this->printDir.dot(normal));
							else angle_dp_nf = acos(-this->printDir.dot(normal));

							// angle between face normal and band
							Eigen::Vector3d r_axis = Eigen::Vector3d::Zero();
							Eigen::Vector3d sufKeep_normal = Eigen::Vector3d::Zero();
							if (normal(1) > 0) {
								r_axis = this->printDir.cross(normal); // rotation axis // "+"
								r_axis.normalize();
								Eigen::AngleAxis<double> sufKeep_rotation
									= Eigen::AngleAxis<double>(PI / 2 - asin(bandWidth), r_axis);
								sufKeep_normal = sufKeep_rotation.matrix() * this->printDir; // "+"
							}
							else {
								r_axis = -this->printDir.cross(normal); // rotation axis // "-"
								r_axis.normalize();
								Eigen::AngleAxis<double> sufKeep_rotation
									= Eigen::AngleAxis<double>(PI / 2 - asin(bandWidth), r_axis);
								sufKeep_normal = sufKeep_rotation.matrix() * (-this->printDir); // "-"
							}
							double angle_nf_band = acos(sufKeep_normal.dot(normal));

							// decide which side to go
							Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
							if (angle_nf_band < angle_dp_nf) { //band rotation
								rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(normal, sufKeep_normal);
								criticalFace->select_coverLayer = 1;
								Tetra->select_coverLayer = 1;
							}
							else {// large cover
								criticalFace->select_coverLayer = 2;
								Tetra->select_coverLayer = 2;
								if (normal(1) > 0)
									rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(normal, this->printDir);// "+"
								else
									rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(normal, -this->printDir);// "-"
							}
							*/
							Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
							frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
						}
					}
					else {}
				}
			}
			std::printf("Finish new local frame calculation.\n");

			//if((loop < 4))	this->_globalQuaternionSmooth1_supportLess(); //only critical
			//else	this->_globalQuaternionSmooth1_surfaceQuality(); //only critical

			this->_globalQuaternionSmooth4();


			for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
				QMeshTetra* Tetra = tetraPatch_elementSet[i];
				frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
			}
			std::printf("Finish quaternion smooth calculation.\n");
			
			this->_get_energy_innerLoop_supportLess();
		}

		// Define variables
		Eigen::SparseMatrix<double> matrix_A_4x;
		Eigen::SparseMatrix<double> matrix_A_transpose_4x;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4x;
		//
		Eigen::SparseMatrix<double> matrix_A_4y;
		Eigen::SparseMatrix<double> matrix_A_transpose_4y;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4y;
		//
		Eigen::SparseMatrix<double> matrix_A_4z;
		Eigen::SparseMatrix<double> matrix_A_transpose_4z;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4z;
		// Initialize variables
		matrix_A_4x.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4x.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4y.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4y.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4z.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4z.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));

		// Fill A matrix for x,y,z
		// Block 11 for x,y,z
		// Important issues
		//give memory to sparse matrix, to accerate the insert speed
		matrix_A_4x.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4y.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4z.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));

		float c1 = -0.25, c2 = 0.75;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int fdx = Tetra->GetIndexNo() * 4;	int vdxArr[4];
			for (int i = 0; i < 4; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

			//double weight = 1.0;
			//if (Tetra->isOverhangTet)		weight = m_criticalTet_weight;
			double weight = this->_get_critcalTet_Weight(Tetra);

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (i == j) {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c2 * weight;
					}
					else {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c1 * weight;
					}
				}
			}
		}

		// Block 12 for x,y,z
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			//double weight = 1.0;
			//if (Tetra->isOverhangTet) weight = m_criticalTet_weight;
			double weight = this->_get_critcalTet_Weight(Tetra);

			for (int i = 0; i < 4; i++) {
				matrix_A_4x.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](0, i); // x row
				matrix_A_4y.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](1, i); // y row
				matrix_A_4z.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](2, i); // z row
			}
		}
		// Block 21 equals to zero
		// Block 22 for x,y,z
		int constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			constraint_ind++;
		}
		// Block 31 equals to zero
		// Block 32 for x,y,z
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			matrix_A_4x.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4y.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4z.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;

			constraint_ind++;
		}

		std::cout << "Finish fill A matrix." << std::endl;

		long time = clock();

		// Factorize A matrix
		matrix_A_4x.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4x(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4x = matrix_A_4x.transpose();
		matATA_4x = matrix_A_transpose_4x * matrix_A_4x;
		Solver_ASAP_4x.compute(matATA_4x);

		matrix_A_4y.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4y(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4y = matrix_A_4y.transpose();
		matATA_4y = matrix_A_transpose_4y * matrix_A_4y;
		Solver_ASAP_4y.compute(matATA_4y);

		matrix_A_4z.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4z(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4z = matrix_A_4z.transpose();
		matATA_4z = matrix_A_transpose_4z * matrix_A_4z;
		Solver_ASAP_4z.compute(matATA_4z);

		std::printf("Finish factorize materix A\n");

		std::printf("The compress spends %f s.\n", (double(clock() - time)) / CLOCKS_PER_SEC);

		// Fill b vector for x,y,z
		//special case of regular term
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			for (int i = 0; i < 3; i++) {

				vector_b[i](4 * tet_Num + neighborScale_face_num + constraint_ind) = m_regularScale_weight;
			}
			constraint_ind++;
		}

		// Solve x vector
		Eigen::VectorXd ATb_4x = matrix_A_transpose_4x * vector_b[0];
		vector_X_Pos_Scale[0] = Solver_ASAP_4x.solve(ATb_4x);

		Eigen::VectorXd ATb_4y = matrix_A_transpose_4y * vector_b[1];
		vector_X_Pos_Scale[1] = Solver_ASAP_4y.solve(ATb_4y);

		Eigen::VectorXd ATb_4z = matrix_A_transpose_4z * vector_b[2];
		vector_X_Pos_Scale[2] = Solver_ASAP_4z.solve(ATb_4z);

		// Update vertex
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
			int idx = node->GetIndexNo();

			double xx, yy, zz;
			node->SetCoord3D(vector_X_Pos_Scale[0](idx), vector_X_Pos_Scale[1](idx), vector_X_Pos_Scale[2](idx));
			node->GetCoord3D(xx, yy, zz);
			//std::printf("New coordinate xx %f , yy %f , zz %f.\n", xx, yy, zz);
		}

		// record the scale value
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			for (int i = 0; i < 3; i++) {
				Tetra->scaleValue_vector[i] = vector_X_Pos_Scale[i](node_Num + fdx);
				//std::cout << vector_X_Pos_Scale[i](node_Num + fdx) << " ";
			}
			//std::cout << std::endl;
		}

		this->_cal_virtual_Vfield();
		std::printf("End calculation of loop %d.\n\n", loop);
	}

	// detact and draw overhang face
	this->m_supportFreeAngle = 45.0;
	this->_detectOverhangFace();
	tetPatch->drawOverhangSurface = true;
}

void DeformTet::runASAP_Hybrid_SL_SQ_test() {
	
	this->preProcess_4SupportLess(3);
	this->m_loopTime = 3;
	this->m_criticalTet_weight_SL = 15.0;
	this->m_globalSmooth_weight = 40.0;
	this->_check_parameters();
	this->runASAP_SupportLess_test3();

	for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

		double xx, yy, zz;
		Node->GetCoord3D(xx, yy, zz);
		Node->SetCoord3D_last(xx, yy, zz);
	}

	//automatically select
	if (true) {
		this->_record_initial_normal3D();
		this->preProcess_4SurfaceQuality(false);
		this->m_loopTime = 1;
		this->m_globalSmooth_weight = 15.0;
		this->m_criticalTet_weight_SQ_cover = 35.0;
		this->m_criticalTet_weight_SQ_vertical = 15.0;
		this->_check_parameters();
		this->runASAP_SurfaceQuality_test2();
	}
	// user select
	else {
		this->_record_initial_normal3D();
		this->preProcess_4SurfaceQuality(true);
		this->m_loopTime = 4;
		this->m_globalSmooth_weight = 15.0;
		this->m_criticalTet_weight_SQ_cover = 10.0;
		this->m_criticalTet_weight_SQ_vertical = 0.0;
		this->_check_parameters();
		this->runASAP_SurfaceQuality_test();
	}
}

//real hybrid method
void DeformTet::runASAP_Hybrid_SL_SQ_test2() {

	this->_check_parameters();
	//---------------------------------------------------------------------------------------------------------------
	// Initialize size of nodes, faces and tets
	int node_Num = tetPatch->GetNodeNumber();
	int tet_Num = tetPatch->GetTetraNumber();
	int face_Num = tetPatch->GetFaceNumber();

	int neighborScale_face_num = 0;
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL)	neighborScale_face_num++;
	}

	std::printf("All face num %d , neighborScale_face_num %d.\n", face_Num, neighborScale_face_num);
	std::printf("A matrix sixe %d x %d\n", (5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));

	//---------------------------------------------------------------------------------------------------------------
	// Define variables
	std::vector<Eigen::MatrixXd> frame_Local_new, frame_Local_initial_inverse, frame_Local_initial;
	std::vector<Eigen::VectorXd> vector_X_Pos_Scale, vector_b;
	// Initialize variables
	frame_Local_new.resize(tet_Num);
	frame_Local_initial_inverse.resize(tet_Num);
	frame_Local_initial.resize(tet_Num);
	vector_X_Pos_Scale.resize(3);
	vector_b.resize(3);

	for (int i = 0; i < 3; i++) {
		vector_X_Pos_Scale[i] = Eigen::VectorXd::Zero(node_Num + tet_Num);
		vector_b[i] = Eigen::VectorXd::Zero(5 * tet_Num + neighborScale_face_num);
	}
	std::cout << "Finish initializing ASAP matrices" << std::endl;

	//---------------------------------------------------------------------------------------------------------------
	// Pre-compute the initial frame and inverse of initial frame
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tet->GetIndexNo();
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
		double center[3] = { 0 };
		QMeshNode* nodes[4];

		for (int i = 0; i < 4; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= 4;
		for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		frame_Local_initial[fdx] = P;
		frame_Local_initial_inverse[fdx] = Eigen::MatrixXd::Zero(3, 4);
		frame_Local_initial_inverse[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();
		//std::cout << frame_Local_initial[fdx] << std::endl << std::endl;
	}
	std::printf("Finish computing local frame and its inverse!\n\n");

	//this->_detectOverhangFace();
	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	for (int loop = 0; loop < m_loopTime; loop++) {

		// only the element with one boundary face are considered as overhang tet
		this->_detectOverhangFace2();

		//Calculate the fabrication energy of SupportLess case ()
		this->_calFabricationEnergy_SupportLess();

		for (int innerLoop = 0; innerLoop < m_innerLoopTime; innerLoop++) {

#pragma omp parallel
			{
#pragma omp for 
				// local rotation and scaling operation (frame_Local_new)
				for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
					QMeshTetra* Tetra = tetraPatch_elementSet[i];

					int fdx = Tetra->GetIndexNo();
					if (innerLoop == 0) {
						double center[3] = { 0.0 };
						Tetra->CalCenterPos(center[0], center[1], center[2]);

						//This tP is each current frame in each tet.
						Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
						QMeshNode* nodes[4];
						for (int i = 0; i < 4; i++) {
							nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
							nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
						}
						for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

						Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

						T = frame_Local_initial_inverse[fdx] * tP;
						T_transpose = T.transpose();

						///// R1 //// Eigen SVD decomposition /////
						Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
						Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
						R = U * V.transpose();
						Tetra->R_estimate = R;
						frame_Local_new[fdx] = R * frame_Local_initial[fdx];
						Tetra->vectorField_4voxelOrder_now = Tetra->R_estimate * Tetra->vectorField_4voxelOrder;
					}

					///// R2 //// Fabrication requirement: only support-less /////
					if (Tetra->isOverhangTet && !Tetra->containProtectSurface) {

						QMeshFace* overhangFace = this->_get_initial_overhang_face(Tetra);
						// initial face normal
						Eigen::Vector3d overhangFace_normal = -overhangFace->normal_last.normalized();
						// virtual face normal after applying R_estimate
						Eigen::Vector3d overhangFace_normal_new = Tetra->R_estimate * overhangFace_normal;
						Tetra->vectorField_4voxelOrder_now = Tetra->R_estimate * Tetra->vectorField_4voxelOrder;

						Eigen::Matrix3d rotationMatrix = _cal_rotationMatrix_supportLess(overhangFace_normal_new, Tetra);
						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					}

					///// R2 //// Fabrication requirement: : only surface quality /////
					if (Tetra->containProtectSurface && !Tetra->isOverhangTet) {

						QMeshFace* boundary_Face = this->_get_initial_kept_face(Tetra);
						// initial face normal
						Eigen::Vector3d normal = boundary_Face->normal_last.normalized();
						// virtual face normal after applying R_estimate
						normal = Tetra->R_estimate * normal;

						// normla is aready vertical enough
						if (fabs(normal(1)) > m_bandWidth_SQ_rotation) { 

							Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Zero();
							/*if (innerLoop >= (m_innerLoopTime - 3)) {
								rotationMatrix = this->_cal_rotationMatrix_surfaceQuality(normal, Tetra, boundary_Face, true);
							}
							else {
								rotationMatrix = this->_cal_rotationMatrix_surfaceQuality(normal, Tetra, boundary_Face, true);
							}*/
							rotationMatrix = this->_cal_rotationMatrix_surfaceQuality(normal, Tetra, boundary_Face);
							Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
							frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
						}
					}
					
					///// R2 //// Fabrication requirement: support-less and surface-quality /////
					if (Tetra->isOverhangTet && Tetra->containProtectSurface) {

						QMeshFace* overhangFace = this->_get_initial_overhang_face(Tetra);
						QMeshFace* keptFace = this->_get_initial_kept_face(Tetra);
						if (overhangFace != keptFace) {
							std::cout << "overhangFace->normal_last " << overhangFace->normal_last << std::endl;
							std::cout << "keptFace->normal_last " << keptFace->normal_last << std::endl;
						}
						// initial face normal
						Eigen::Vector3d overhangFace_normal = -overhangFace->normal_last.normalized();
						// virtual face normal after applying R_estimate
						Eigen::Vector3d overhangFace_normal_new = Tetra->R_estimate * overhangFace_normal;
						/*std::cout << "overhangFace_normal " << overhangFace_normal.transpose() << std::endl;*/
						Tetra->vectorField_4voxelOrder_now = Tetra->R_estimate * Tetra->vectorField_4voxelOrder;
						/*std::cout << "overhangFace_normal_new " << overhangFace_normal_new.transpose() << std::endl;*/
						Eigen::Matrix3d rotationMatrix = _cal_rotationMatrix_SL_SQ(overhangFace_normal_new, Tetra);
						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					}
				}
			}
			std::printf("Finish new local frame calculation.\n");

			if (innerLoop == (m_innerLoopTime - 3)) {
				this->m_keep_SQ_cover = 10;
				std::cout << "Increase the quaternion keep weight in innerloop " << innerLoop << " as " << this->m_keep_SQ_cover << std::endl;
			}

			this->_globalQuaternionSmooth4();


			for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
				QMeshTetra* Tetra = tetraPatch_elementSet[i];
				frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
			}
			std::printf("Finish quaternion smooth calculation.\n");

			this->_get_energy_innerLoop_supportLess();
			this->_get_energy_innerLoop_surfaceQuality();
		}

		// Define variables
		Eigen::SparseMatrix<double> matrix_A_4x;
		Eigen::SparseMatrix<double> matrix_A_transpose_4x;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4x;
		//
		Eigen::SparseMatrix<double> matrix_A_4y;
		Eigen::SparseMatrix<double> matrix_A_transpose_4y;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4y;
		//
		Eigen::SparseMatrix<double> matrix_A_4z;
		Eigen::SparseMatrix<double> matrix_A_transpose_4z;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4z;
		// Initialize variables
		matrix_A_4x.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4x.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4y.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4y.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4z.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4z.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));

		// Fill A matrix for x,y,z
		// Block 11 for x,y,z
		// Important issues
		//give memory to sparse matrix, to accerate the insert speed
		matrix_A_4x.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4y.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4z.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));

		float c1 = -0.25, c2 = 0.75;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int fdx = Tetra->GetIndexNo() * 4;	int vdxArr[4];
			for (int i = 0; i < 4; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

			//double weight = 1.0;
			//if (Tetra->isOverhangTet)		weight = m_criticalTet_weight;
			double weight = this->_get_critcalTet_Weight(Tetra);

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (i == j) {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c2 * weight;
					}
					else {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c1 * weight;
					}
				}
			}
		}

		// Block 12 for x,y,z
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			//double weight = 1.0;
			//if (Tetra->isOverhangTet) weight = m_criticalTet_weight;
			double weight = this->_get_critcalTet_Weight(Tetra);

			for (int i = 0; i < 4; i++) {
				matrix_A_4x.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](0, i); // x row
				matrix_A_4y.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](1, i); // y row
				matrix_A_4z.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](2, i); // z row
			}
		}
		// Block 21 equals to zero
		// Block 22 for x,y,z
		int constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			constraint_ind++;
		}
		// Block 31 equals to zero
		// Block 32 for x,y,z
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			matrix_A_4x.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4y.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4z.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;

			constraint_ind++;
		}

		std::cout << "Finish fill A matrix." << std::endl;

		long time = clock();

		// Factorize A matrix
		matrix_A_4x.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4x(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4x = matrix_A_4x.transpose();
		matATA_4x = matrix_A_transpose_4x * matrix_A_4x;
		Solver_ASAP_4x.compute(matATA_4x);

		matrix_A_4y.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4y(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4y = matrix_A_4y.transpose();
		matATA_4y = matrix_A_transpose_4y * matrix_A_4y;
		Solver_ASAP_4y.compute(matATA_4y);

		matrix_A_4z.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4z(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4z = matrix_A_4z.transpose();
		matATA_4z = matrix_A_transpose_4z * matrix_A_4z;
		Solver_ASAP_4z.compute(matATA_4z);

		std::printf("Finish factorize materix A\n");

		std::printf("The compress spends %f s.\n", (double(clock() - time)) / CLOCKS_PER_SEC);

		// Fill b vector for x,y,z
		//special case of regular term
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			for (int i = 0; i < 3; i++) {

				vector_b[i](4 * tet_Num + neighborScale_face_num + constraint_ind) = m_regularScale_weight;
			}
			constraint_ind++;
		}

		// Solve x vector
		Eigen::VectorXd ATb_4x = matrix_A_transpose_4x * vector_b[0];
		vector_X_Pos_Scale[0] = Solver_ASAP_4x.solve(ATb_4x);

		Eigen::VectorXd ATb_4y = matrix_A_transpose_4y * vector_b[1];
		vector_X_Pos_Scale[1] = Solver_ASAP_4y.solve(ATb_4y);

		Eigen::VectorXd ATb_4z = matrix_A_transpose_4z * vector_b[2];
		vector_X_Pos_Scale[2] = Solver_ASAP_4z.solve(ATb_4z);

		// Update vertex
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
			int idx = node->GetIndexNo();

			double xx, yy, zz;
			node->SetCoord3D(vector_X_Pos_Scale[0](idx), vector_X_Pos_Scale[1](idx), vector_X_Pos_Scale[2](idx));
			node->GetCoord3D(xx, yy, zz);
			//std::printf("New coordinate xx %f , yy %f , zz %f.\n", xx, yy, zz);
		}

		// record the scale value
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			for (int i = 0; i < 3; i++) {
				Tetra->scaleValue_vector[i] = vector_X_Pos_Scale[i](node_Num + fdx);
				//std::cout << vector_X_Pos_Scale[i](node_Num + fdx) << " ";
			}
			//std::cout << std::endl;
		}

		this->_cal_virtual_Vfield();
		std::printf("End calculation of loop %d.\n\n", loop);
	}

	// detact and draw overhang face
	this->m_supportFreeAngle = 45.0;
	this->_detectOverhangFace();
	tetPatch->drawOverhangSurface = true;
}

void DeformTet::runASAP_Hybrid_SR_SQ() {

	this->_check_parameters();
	//---------------------------------------------------------------------------------------------------------------
	// Initialize size of nodes, faces and tets
	int node_Num = tetPatch->GetNodeNumber();
	int tet_Num = tetPatch->GetTetraNumber();
	int face_Num = tetPatch->GetFaceNumber();

	int neighborScale_face_num = 0;
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL)	neighborScale_face_num++;
	}

	std::printf("All face num %d , neighborScale_face_num %d.\n", face_Num, neighborScale_face_num);
	std::printf("A matrix sixe %d x %d\n", (5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));

	//---------------------------------------------------------------------------------------------------------------
	// Define variables
	std::vector<Eigen::MatrixXd> frame_Local_new, frame_Local_initial_inverse, frame_Local_initial;
	std::vector<Eigen::VectorXd> vector_X_Pos_Scale, vector_b;
	// Initialize variables
	frame_Local_new.resize(tet_Num);
	frame_Local_initial_inverse.resize(tet_Num);
	frame_Local_initial.resize(tet_Num);
	vector_X_Pos_Scale.resize(3);
	vector_b.resize(3);

	for (int i = 0; i < 3; i++) {
		vector_X_Pos_Scale[i] = Eigen::VectorXd::Zero(node_Num + tet_Num);
		vector_b[i] = Eigen::VectorXd::Zero(5 * tet_Num + neighborScale_face_num);
	}
	std::cout << "initial ASAP matrices" << std::endl;

	//---------------------------------------------------------------------------------------------------------------
	// Pre-compute the initial frame and inverse of initla frame
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tet->GetIndexNo();
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
		double center[3] = { 0 };
		QMeshNode* nodes[4];

		for (int i = 0; i < 4; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= 4;
		for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		frame_Local_initial[fdx] = P;
		frame_Local_initial_inverse[fdx] = Eigen::MatrixXd::Zero(3, 4);
		frame_Local_initial_inverse[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();
		//std::cout << frame_Local_initial[fdx] << std::endl << std::endl;
	}
	std::printf("Finish compute local frame and its inverse!\n\n");

	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	for (int loop = 0; loop < m_loopTime; loop++) {

		this->_calFabricationEnergy_StrengthReinforcement(frame_Local_initial_inverse);

		for (int innerLoop = 0; innerLoop < m_innerLoopTime; innerLoop++) {

#pragma omp parallel
			{
#pragma omp for 
				// local rotation and scaling operation (frame_Local_new)
				for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
					QMeshTetra* Tetra = tetraPatch_elementSet[i];

					int fdx = Tetra->GetIndexNo();
					//add here
					if (innerLoop == 0) {

						double center[3] = { 0 };
						Tetra->CalCenterPos(center[0], center[1], center[2]);

						//This tP is each current frame in each tet.
						Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
						QMeshNode* nodes[4];
						for (int i = 0; i < 4; i++) {
							nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
							nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
						}
						for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

						Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

						T = frame_Local_initial_inverse[fdx] * tP;
						T_transpose = T.transpose();

						///// R1 //// Eigen SVD decomposition /////
						Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
						Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
						R = U * V.transpose();
						Tetra->R_estimate = R;
						frame_Local_new[fdx] = R * frame_Local_initial[fdx];
					}

					///// R2 //// Fabrication requirement /////
					if (Tetra->isTensileorCompressSelect && !Tetra->containProtectSurface) {
						Eigen::Vector3d principal_Stress_Dir = Tetra->R_estimate * Tetra->tau_max;

						Eigen::Matrix3d rotationMatrix = this->_cal_rotationMatrix_strengthReinforcement(principal_Stress_Dir);

						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					}

					///// R2 //// Fabrication requirement (surface quality) /////
					if (Tetra->containProtectSurface && !Tetra->isTensileorCompressSelect) {

						QMeshFace* keptFace = this->_get_initial_kept_face(Tetra);
						Eigen::Vector3d normal = keptFace->normal_last.normalized(); // initial face normal

						normal = Tetra->R_estimate * normal;// virtual face normal after applying R_estimate

						if (fabs(normal(1)) > m_bandWidth_SQ_rotation) { // ny out of band

							Eigen::Matrix3d rotationMatrix = this->_cal_rotationMatrix_surfaceQuality(normal, Tetra, keptFace);

							Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
							frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
						}
					}
					if (Tetra->containProtectSurface && Tetra->isTensileorCompressSelect){
						// virtual tau_max after applying R_estimate
						Eigen::Vector3d principal_Stress_Dir = Tetra->R_estimate * Tetra->tau_max;
						// initial face normal
						QMeshFace* keptFace = this->_get_initial_kept_face(Tetra);
						Eigen::Vector3d normal = keptFace->normal_last.normalized(); 
						// virtual face normal after applying R_estimate
						normal = Tetra->R_estimate * normal;

						Eigen::Matrix3d rotationMatrix = this->_cal_rotationMatrix_SR_SQ(principal_Stress_Dir, normal, Tetra);

						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					}
				}
			}
			std::printf("Finish new local frame calculation!\n");

			if (innerLoop == (m_innerLoopTime - 3)) {
				this->m_keep_SQ_cover = 10;
				std::cout << "Increase the quaternion keep weight in innerloop " << innerLoop << " as " << this->m_keep_SQ_cover << std::endl;
			}
			this->_globalQuaternionSmooth4();

			for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
				QMeshTetra* Tetra = tetraPatch_elementSet[i];
				frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
			}
			std::printf("Finish quaternion smooth calculation.\n");

			this->_get_energy_innerLoop_strengthReinforcement();
			this->_get_energy_innerLoop_surfaceQuality();
		}
		// Define variables
		Eigen::SparseMatrix<double> matrix_A_4x;
		Eigen::SparseMatrix<double> matrix_A_transpose_4x;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4x;
		//
		Eigen::SparseMatrix<double> matrix_A_4y;
		Eigen::SparseMatrix<double> matrix_A_transpose_4y;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4y;
		//
		Eigen::SparseMatrix<double> matrix_A_4z;
		Eigen::SparseMatrix<double> matrix_A_transpose_4z;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4z;
		// Initialize variables
		matrix_A_4x.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4x.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4y.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4y.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4z.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4z.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));

		// Fill A matrix for x,y,z
		// Block 11 for x,y,z
		// Important issues
		//give memory to sparse matrix, to accerate the insert speed
		matrix_A_4x.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4y.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4z.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));

		float c1 = -0.25, c2 = 0.75;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int fdx = Tetra->GetIndexNo() * 4;	int vdxArr[4];
			for (int i = 0; i < 4; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

			//double weight = 1.0;
			//if (Tetra->containProtectSurface) {
			//	if (Tetra->select_coverLayer == 2)	weight = m_criticalTet_weight_SQ_cover;
			//	else if (Tetra->select_coverLayer == 1)	weight = m_criticalTet_weight_SQ_vertical;
			//	else weight = 1;
			//}

			double weight = this->_get_critcalTet_Weight(Tetra);

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (i == j) {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c2 * weight;
					}
					else {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c1 * weight;
					}
				}
			}
		}

		// Block 12 for x,y,z
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			//double weight = 1.0;
			//if (Tetra->containProtectSurface) {
			//	if (Tetra->select_coverLayer == 2)	weight = m_criticalTet_weight_SQ_cover;
			//	else if (Tetra->select_coverLayer == 1)	weight = m_criticalTet_weight_SQ_vertical;
			//	else weight = 1;
			//}

			double weight = this->_get_critcalTet_Weight(Tetra);

			for (int i = 0; i < 4; i++) {
				matrix_A_4x.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](0, i); // x row
				matrix_A_4y.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](1, i); // y row
				matrix_A_4z.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](2, i); // z row
			}
		}
		// Block 21 equals to zero
		// Block 22 for x,y,z
		int constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			constraint_ind++;
		}
		// Block 31 equals to zero
		// Block 32 for x,y,z
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			matrix_A_4x.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4y.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4z.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;

			constraint_ind++;
		}

		std::cout << "Finish fill A matrix." << std::endl;

		long time = clock();

		// Factorize A matrix
		matrix_A_4x.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4x(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4x = matrix_A_4x.transpose();
		matATA_4x = matrix_A_transpose_4x * matrix_A_4x;
		Solver_ASAP_4x.compute(matATA_4x);

		matrix_A_4y.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4y(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4y = matrix_A_4y.transpose();
		matATA_4y = matrix_A_transpose_4y * matrix_A_4y;
		Solver_ASAP_4y.compute(matATA_4y);

		matrix_A_4z.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4z(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4z = matrix_A_4z.transpose();
		matATA_4z = matrix_A_transpose_4z * matrix_A_4z;
		Solver_ASAP_4z.compute(matATA_4z);

		std::printf("Finish factorize materix A\n");

		std::printf("The compress spends %f s.\n", (double(clock() - time)) / CLOCKS_PER_SEC);

		// Fill b vector for x,y,z
		//special case of regular term
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			for (int i = 0; i < 3; i++) {

				vector_b[i](4 * tet_Num + neighborScale_face_num + constraint_ind) = m_regularScale_weight;
			}
			constraint_ind++;
		}

		// Solve x vector
		Eigen::VectorXd ATb_4x = matrix_A_transpose_4x * vector_b[0];
		vector_X_Pos_Scale[0] = Solver_ASAP_4x.solve(ATb_4x);

		Eigen::VectorXd ATb_4y = matrix_A_transpose_4y * vector_b[1];
		vector_X_Pos_Scale[1] = Solver_ASAP_4y.solve(ATb_4y);

		Eigen::VectorXd ATb_4z = matrix_A_transpose_4z * vector_b[2];
		vector_X_Pos_Scale[2] = Solver_ASAP_4z.solve(ATb_4z);

		// Update vertex
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
			int idx = node->GetIndexNo();

			double xx, yy, zz;
			node->SetCoord3D(vector_X_Pos_Scale[0](idx), vector_X_Pos_Scale[1](idx), vector_X_Pos_Scale[2](idx));
			node->GetCoord3D(xx, yy, zz);
			//std::printf("New coordinate xx %f , yy %f , zz %f.\n", xx, yy, zz);
		}

		// record the scale value
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			for (int i = 0; i < 3; i++) {
				Tetra->scaleValue_vector[i] = vector_X_Pos_Scale[i](node_Num + fdx);
				//std::cout << vector_X_Pos_Scale[i](node_Num + fdx) << " ";
			}
			//std::cout << std::endl;
		}

		std::printf("End calculation of loop %d.\n\n", loop);
	}

	std::printf("End calculation.\n");
}

void DeformTet::runASAP_Hybrid_SR_SQ_test() {

	bool preProcessFinish = this->preProcess_4StrengthReinforcement();
	if (preProcessFinish == false) { std::cout << "Pre-process did not finish, please check the FEM file read. " << std::endl; return; }
	this->m_loopTime = 2;
	this->m_globalSmooth_weight = 40.0;
	this->_check_parameters();
	this->runASAP_StrengthReinforcement_test();

	for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

		double xx, yy, zz;
		Node->GetCoord3D(xx, yy, zz);
		Node->SetCoord3D_last(xx, yy, zz);
	}

	//automatically select
	if (true) {
		this->_record_initial_normal3D();
		this->preProcess_4SurfaceQuality(false);
		this->m_criticalTet_weight_SR = 15.0;
		this->m_regularScale_weight = 1.0;
		this->m_loopTime = 3;
		this->m_globalSmooth_weight = 15.0;
		this->_check_parameters();
		this->runASAP_SurfaceQuality_test2();
	}
	else {// topology layers = 97
		this->_record_initial_normal3D();
		this->preProcess_4SurfaceQuality(true);
		this->m_criticalTet_weight_SR = 20.0;
		this->m_regularScale_weight = 0.2;
		this->m_loopTime = 3;
		this->m_globalSmooth_weight = 10.0;
		this->_check_parameters();
		this->runASAP_SurfaceQuality_test();// user select
	}

	//mark green flatted regions
	/*
			//for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
			//	QMeshTetra* Tetra = tetraPatch_elementSet[i];
			//	if (Tetra->containProtectSurface) {
			//		QMeshFace* criticalFace = NULL;
			//		for (int i = 0; i < 4; i++) {
			//			QMeshFace* thisFace = Tetra->GetFaceRecordPtr(i + 1);
			//			if (thisFace->isQualityProtectFace) {
			//				criticalFace = thisFace;
			//				break;
			//			}
			//		}
			//		if (criticalFace == NULL) std::cout << "ERROR! No quality protect face" << std::endl;
			//
			//		for (int k = 0; k < 3; k++) {
			//			criticalFace->GetNodeRecordPtr(k)->isSurfaceProtectNode = false;
			//		}
			//		if (criticalFace->select_coverLayer == 2) {
			//			for (int k = 0; k < 3; k++) {
			//				criticalFace->GetNodeRecordPtr(k)->isSurfaceProtectNode = true;
			//			}
			//		}
			//	}
			//}
			*/

	/*for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
				QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

				if (Face->GetLeftTetra() != NULL && Face->GetRightTetra() != NULL) continue;

				if (Face->isQualityProtectFace && Face->select_coverLayer == 2) {
					for (int k = 0; k < 3; k++) {
						Face->GetNodeRecordPtr(k)->isSurfaceProtectNode_temp = true;
					}
				}
			}
			for (int shrinkLoop = 0; shrinkLoop < 2; shrinkLoop++) {
				for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
					QMeshNode* thisNode = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

					if (thisNode->inner) continue;

					for (int i = 0; i < thisNode->GetEdgeNumber(); i++) {

						QMeshEdge* thisEdge = thisNode->GetEdgeRecordPtr(i + 1);
						QMeshNode* NeighboorNode = thisEdge->GetStartPoint();
						if (thisNode == thisEdge->GetStartPoint()) NeighboorNode = thisEdge->GetEndPoint();
						if (NeighboorNode->inner) continue;

						if (!NeighboorNode->isSurfaceProtectNode_temp) {
							thisNode->isSurfaceProtectNode = false;
							break;
						}
						else thisNode->isSurfaceProtectNode = true;
					}
				}
			}*/

	/*
	for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);
		if (Face->GetLeftTetra() != NULL && Face->GetRightTetra() != NULL) continue;

		if (Face->isQualityProtectFace && Face->select_coverLayer == 2) {
			double nx, ny, nz;
			Face->CalPlaneEquation();
			Face->GetNormal(nx, ny, nz);
			if (abs(ny) < 0.95) Face->select_coverLayer = 0;
		}
	}
	for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

		if (Face->GetLeftTetra() != NULL && Face->GetRightTetra() != NULL) continue;
		for (int k = 0; k < 3; k++) { Face->GetNodeRecordPtr(k)->isSurfaceProtectNode = true; }
	}
	for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);
		if (Face->GetLeftTetra() != NULL && Face->GetRightTetra() != NULL) continue;

		if (Face->isQualityProtectFace && Face->select_coverLayer != 2) {
			for (int k = 0; k < 3; k++) {
				Face->GetNodeRecordPtr(k)->isSurfaceProtectNode = false;
			}
		}
	}
	*/
	//end
}

//real hybrid method
void DeformTet::runASAP_Hybrid_SL_SR() {

	this->_check_parameters();
	//---------------------------------------------------------------------------------------------------------------
	// Initialize size of nodes, faces and tets
	int node_Num = tetPatch->GetNodeNumber();
	int tet_Num = tetPatch->GetTetraNumber();
	int face_Num = tetPatch->GetFaceNumber();

	int neighborScale_face_num = 0;
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL)	neighborScale_face_num++;
	}

	std::printf("All face num %d , neighborScale_face_num %d.\n", face_Num, neighborScale_face_num);
	std::printf("A matrix sixe %d x %d\n", (5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));

	//---------------------------------------------------------------------------------------------------------------
	// Define variables
	std::vector<Eigen::MatrixXd> frame_Local_new, frame_Local_initial_inverse, frame_Local_initial;
	std::vector<Eigen::VectorXd> vector_X_Pos_Scale, vector_b;
	// Initialize variables
	frame_Local_new.resize(tet_Num);
	frame_Local_initial_inverse.resize(tet_Num);
	frame_Local_initial.resize(tet_Num);
	vector_X_Pos_Scale.resize(3);
	vector_b.resize(3);

	for (int i = 0; i < 3; i++) {
		vector_X_Pos_Scale[i] = Eigen::VectorXd::Zero(node_Num + tet_Num);
		vector_b[i] = Eigen::VectorXd::Zero(5 * tet_Num + neighborScale_face_num);
	}
	std::cout << "Finish initializing ASAP matrices" << std::endl;

	//---------------------------------------------------------------------------------------------------------------
	// Pre-compute the initial frame and inverse of initial frame
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tet->GetIndexNo();
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
		double center[3] = { 0 };
		QMeshNode* nodes[4];

		for (int i = 0; i < 4; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= 4;
		for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		frame_Local_initial[fdx] = P;
		frame_Local_initial_inverse[fdx] = Eigen::MatrixXd::Zero(3, 4);
		frame_Local_initial_inverse[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();
		//std::cout << frame_Local_initial[fdx] << std::endl << std::endl;
	}
	std::printf("Finish computing local frame and its inverse!\n\n");

	//this->_detectOverhangFace();
	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	for (int loop = 0; loop < m_loopTime; loop++) {

		// only the element with one boundary face are considered as overhang tet
		this->_detectOverhangFace2();

		// less overhang element has tau_max, we modify it
		this->_giveBoundary_OverhangTet_stressInfo();

		////debug
		//for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		//	QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
		//
		//	//std::cout << "I am here ++++++++++++++++++++++++++++++++++++++ " << std::endl;
		//
		//	if (Tetra->isOverhangTet && Tetra->isTensileorCompressSelect) {
		//		std::cout << "\n>this is hybrid ele" << std::endl;
		//	}
		//
		//	if (Tetra->isOverhangTet && !Tetra->isTensileorCompressSelect) {
		//		//std::cout << "\nthis is overhang ele------->" << std::endl;
		//	}
		//
		//	if (!Tetra->isOverhangTet && Tetra->isTensileorCompressSelect) {
		//		//std::cout << "\n------->this is strength ele" << std::endl;
		//		for (int i = 0; i < 4; i++) {
		//			Tetra->GetFaceRecordPtr(i + 1)->isSpecialShow = true;
		//		}
		//	}
		//}
		////

		//Calculate the fabrication energy of SupportLess case ()
		this->_calFabricationEnergy_SupportLess();

		for (int innerLoop = 0; innerLoop < m_innerLoopTime; innerLoop++) {

#pragma omp parallel
			{
#pragma omp for 
				// local rotation and scaling operation (frame_Local_new)
				for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
					QMeshTetra* Tetra = tetraPatch_elementSet[i];

					int fdx = Tetra->GetIndexNo();
					if (innerLoop == 0) {
						double center[3] = { 0.0 };
						Tetra->CalCenterPos(center[0], center[1], center[2]);

						//This tP is each current frame in each tet.
						Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
						QMeshNode* nodes[4];
						for (int i = 0; i < 4; i++) {
							nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
							nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
						}
						for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

						Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

						T = frame_Local_initial_inverse[fdx] * tP;
						T_transpose = T.transpose();

						///// R1 //// Eigen SVD decomposition /////
						Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
						Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
						R = U * V.transpose();
						Tetra->R_estimate = R;
						frame_Local_new[fdx] = R * frame_Local_initial[fdx];
						Tetra->vectorField_4voxelOrder_now = Tetra->R_estimate * Tetra->vectorField_4voxelOrder;
					}

					///// R2 //// Fabrication requirement: only support-less /////
					if (Tetra->isOverhangTet && !Tetra->isTensileorCompressSelect) {
					//if (Tetra->isOverhangTet) {

						QMeshFace* overhangFace = this->_get_initial_overhang_face(Tetra);
						// initial face normal
						Eigen::Vector3d overhangFace_normal = -overhangFace->normal_last.normalized();
						// virtual face normal after applying R_estimate
						Eigen::Vector3d overhangFace_normal_new = Tetra->R_estimate * overhangFace_normal;
						Tetra->vectorField_4voxelOrder_now = Tetra->R_estimate * Tetra->vectorField_4voxelOrder;

						Eigen::Matrix3d rotationMatrix = _cal_rotationMatrix_supportLess(overhangFace_normal_new, Tetra);
						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					}

					///// R2 //// Fabrication requirement: : only surface quality /////
					if (Tetra->isTensileorCompressSelect && !Tetra->isOverhangTet ) {
						Eigen::Vector3d principal_Stress_Dir = Tetra->R_estimate * Tetra->tau_max;

						Eigen::Matrix3d rotationMatrix = this->_cal_rotationMatrix_strengthReinforcement(principal_Stress_Dir);

						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];				
					}

					/////// R2 //// Fabrication requirement: support-less and strengthReinforcement /////
					if (Tetra->isOverhangTet && Tetra->isTensileorCompressSelect) {

						QMeshFace* overhangFace = this->_get_initial_overhang_face(Tetra);
						// initial face normal
						Eigen::Vector3d overhangFace_normal = -overhangFace->normal_last.normalized();
						// virtual face normal after applying R_estimate
						Eigen::Vector3d overhangFace_normal_new = Tetra->R_estimate * overhangFace_normal;
						/*std::cout << "overhangFace_normal " << overhangFace_normal.transpose() << std::endl;*/

						Eigen::Vector3d principal_Stress_Dir = Tetra->R_estimate * Tetra->tau_max;
						Eigen::Matrix3d rotationMatrix = _cal_rotationMatrix_SL_SR_Iter(
							overhangFace_normal_new, principal_Stress_Dir, Tetra);
						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					}
				}
			}
			std::printf("Finish new local frame calculation.\n");

			/*if (innerLoop == (m_innerLoopTime - 3)) {
				this->m_keep_SQ_cover = 10;
				std::cout << "Increase the quaternion keep weight in innerloop " << innerLoop << " as " << this->m_keep_SQ_cover << std::endl;
			}*/

			//this->_globalQuaternionSmooth4_withoutBottom();
			this->_globalQuaternionSmooth4();

			for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
				QMeshTetra* Tetra = tetraPatch_elementSet[i];
				frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
			}
			std::printf("Finish quaternion smooth calculation.\n");

			this->_get_energy_innerLoop_supportLess();
			this->_get_energy_innerLoop_strengthReinforcement();
		}

		// Define variables
		Eigen::SparseMatrix<double> matrix_A_4x;
		Eigen::SparseMatrix<double> matrix_A_transpose_4x;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4x;
		//
		Eigen::SparseMatrix<double> matrix_A_4y;
		Eigen::SparseMatrix<double> matrix_A_transpose_4y;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4y;
		//
		Eigen::SparseMatrix<double> matrix_A_4z;
		Eigen::SparseMatrix<double> matrix_A_transpose_4z;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4z;
		// Initialize variables
		matrix_A_4x.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4x.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4y.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4y.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4z.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4z.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));

		// Fill A matrix for x,y,z
		// Block 11 for x,y,z
		// Important issues
		//give memory to sparse matrix, to accerate the insert speed
		matrix_A_4x.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4y.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4z.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));

		float c1 = -0.25, c2 = 0.75;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int fdx = Tetra->GetIndexNo() * 4;	int vdxArr[4];
			for (int i = 0; i < 4; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

			//double weight = 1.0;
			//if (Tetra->isOverhangTet)		weight = m_criticalTet_weight;
			double weight = this->_get_critcalTet_Weight(Tetra);

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (i == j) {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c2 * weight;
					}
					else {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c1 * weight;
					}
				}
			}
		}

		// Block 12 for x,y,z
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			//double weight = 1.0;
			//if (Tetra->isOverhangTet) weight = m_criticalTet_weight;
			double weight = this->_get_critcalTet_Weight(Tetra);

			for (int i = 0; i < 4; i++) {
				matrix_A_4x.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](0, i); // x row
				matrix_A_4y.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](1, i); // y row
				matrix_A_4z.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](2, i); // z row
			}
		}
		// Block 21 equals to zero
		// Block 22 for x,y,z
		int constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			constraint_ind++;
		}
		// Block 31 equals to zero
		// Block 32 for x,y,z
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			matrix_A_4x.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4y.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4z.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;

			constraint_ind++;
		}

		std::cout << "Finish fill A matrix." << std::endl;

		long time = clock();

		// Factorize A matrix
		matrix_A_4x.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4x(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4x = matrix_A_4x.transpose();
		matATA_4x = matrix_A_transpose_4x * matrix_A_4x;
		Solver_ASAP_4x.compute(matATA_4x);

		matrix_A_4y.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4y(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4y = matrix_A_4y.transpose();
		matATA_4y = matrix_A_transpose_4y * matrix_A_4y;
		Solver_ASAP_4y.compute(matATA_4y);

		matrix_A_4z.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4z(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4z = matrix_A_4z.transpose();
		matATA_4z = matrix_A_transpose_4z * matrix_A_4z;
		Solver_ASAP_4z.compute(matATA_4z);

		std::printf("Finish factorize materix A\n");

		std::printf("The compress spends %f s.\n", (double(clock() - time)) / CLOCKS_PER_SEC);

		// Fill b vector for x,y,z
		//special case of regular term
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			for (int i = 0; i < 3; i++) {

				vector_b[i](4 * tet_Num + neighborScale_face_num + constraint_ind) = m_regularScale_weight;
			}
			constraint_ind++;
		}

		// Solve x vector
		Eigen::VectorXd ATb_4x = matrix_A_transpose_4x * vector_b[0];
		vector_X_Pos_Scale[0] = Solver_ASAP_4x.solve(ATb_4x);

		Eigen::VectorXd ATb_4y = matrix_A_transpose_4y * vector_b[1];
		vector_X_Pos_Scale[1] = Solver_ASAP_4y.solve(ATb_4y);

		Eigen::VectorXd ATb_4z = matrix_A_transpose_4z * vector_b[2];
		vector_X_Pos_Scale[2] = Solver_ASAP_4z.solve(ATb_4z);

		// Update vertex
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
			int idx = node->GetIndexNo();

			double xx, yy, zz;
			node->SetCoord3D(vector_X_Pos_Scale[0](idx), vector_X_Pos_Scale[1](idx), vector_X_Pos_Scale[2](idx));
			node->GetCoord3D(xx, yy, zz);
			//std::printf("New coordinate xx %f , yy %f , zz %f.\n", xx, yy, zz);
		}

		// record the scale value
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			for (int i = 0; i < 3; i++) {
				Tetra->scaleValue_vector[i] = vector_X_Pos_Scale[i](node_Num + fdx);
				//std::cout << vector_X_Pos_Scale[i](node_Num + fdx) << " ";
			}
			//std::cout << std::endl;
		}

		this->_cal_virtual_Vfield();
		std::printf("End calculation of loop %d.\n\n", loop);
	}

	// detact and draw overhang face
	//tetPatch->drawStressField = false;
	this->m_supportFreeAngle = 45.0;
	this->_detectOverhangFace();
	tetPatch->drawOverhangSurface = true;
}

//real hybrid method
void DeformTet::runASAP_Hybrid_SL_SR_SQ() {

	this->_check_parameters();
	//---------------------------------------------------------------------------------------------------------------
	// Initialize size of nodes, faces and tets
	int node_Num = tetPatch->GetNodeNumber();
	int tet_Num = tetPatch->GetTetraNumber();
	int face_Num = tetPatch->GetFaceNumber();

	int neighborScale_face_num = 0;
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

		if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL)	neighborScale_face_num++;
	}

	std::printf("All face num %d , neighborScale_face_num %d.\n", face_Num, neighborScale_face_num);
	std::printf("A matrix sixe %d x %d\n", (5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));

	//---------------------------------------------------------------------------------------------------------------
	// Define variables
	std::vector<Eigen::MatrixXd> frame_Local_new, frame_Local_initial_inverse, frame_Local_initial;
	std::vector<Eigen::VectorXd> vector_X_Pos_Scale, vector_b;
	// Initialize variables
	frame_Local_new.resize(tet_Num);
	frame_Local_initial_inverse.resize(tet_Num);
	frame_Local_initial.resize(tet_Num);
	vector_X_Pos_Scale.resize(3);
	vector_b.resize(3);

	for (int i = 0; i < 3; i++) {
		vector_X_Pos_Scale[i] = Eigen::VectorXd::Zero(node_Num + tet_Num);
		vector_b[i] = Eigen::VectorXd::Zero(5 * tet_Num + neighborScale_face_num);
	}
	std::cout << "Finish initializing ASAP matrices" << std::endl;

	//---------------------------------------------------------------------------------------------------------------
	// Pre-compute the initial frame and inverse of initial frame
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tet->GetIndexNo();
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
		double center[3] = { 0 };
		QMeshNode* nodes[4];

		for (int i = 0; i < 4; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= 4;
		for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		frame_Local_initial[fdx] = P;
		frame_Local_initial_inverse[fdx] = Eigen::MatrixXd::Zero(3, 4);
		frame_Local_initial_inverse[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();
		//std::cout << frame_Local_initial[fdx] << std::endl << std::endl;
	}
	std::printf("Finish computing local frame and its inverse!\n\n");

	//this->_detectOverhangFace();
	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	for (int loop = 0; loop < m_loopTime; loop++) {

		// only the element with one boundary face are considered as overhang tet
		this->_detectOverhangFace2();

		// less overhang element has tau_max, we modify it
		// this->_giveBoundary_OverhangTet_stressInfo();

		//Calculate the fabrication energy of SupportLess case ()
		this->_calFabricationEnergy_SupportLess();

		for (int innerLoop = 0; innerLoop < m_innerLoopTime; innerLoop++) {

#pragma omp parallel
			{
#pragma omp for 
				// local rotation and scaling operation (frame_Local_new)
				for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
					QMeshTetra* Tetra = tetraPatch_elementSet[i];

					int fdx = Tetra->GetIndexNo();
					if (innerLoop == 0) {
						double center[3] = { 0.0 };
						Tetra->CalCenterPos(center[0], center[1], center[2]);

						//This tP is each current frame in each tet.
						Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
						QMeshNode* nodes[4];
						for (int i = 0; i < 4; i++) {
							nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
							nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
						}
						for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

						Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
						Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

						T = frame_Local_initial_inverse[fdx] * tP;
						T_transpose = T.transpose();

						///// R1 //// Eigen SVD decomposition /////
						Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
						Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
						R = U * V.transpose();
						Tetra->R_estimate = R;
						frame_Local_new[fdx] = R * frame_Local_initial[fdx];
						Tetra->vectorField_4voxelOrder_now = Tetra->R_estimate * Tetra->vectorField_4voxelOrder;
					}

					///// R2 //// Fabrication requirement: only support-less /////
					if (Tetra->isOverhangTet) {
						QMeshFace* overhangFace = this->_get_initial_overhang_face(Tetra);
						// initial face normal
						Eigen::Vector3d overhangFace_normal = -overhangFace->normal_last.normalized();
						// virtual face normal after applying R_estimate
						Eigen::Vector3d overhangFace_normal_new = Tetra->R_estimate * overhangFace_normal;
						Tetra->vectorField_4voxelOrder_now = Tetra->R_estimate * Tetra->vectorField_4voxelOrder;

						Eigen::Matrix3d rotationMatrix = _cal_rotationMatrix_supportLess(overhangFace_normal_new, Tetra);
						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					}

					///// R2 //// Fabrication requirement: : only strength reinforcement /////
					else if (Tetra->isTensileorCompressSelect) {
						Eigen::Vector3d principal_Stress_Dir = Tetra->R_estimate * Tetra->tau_max;

						Eigen::Matrix3d rotationMatrix = this->_cal_rotationMatrix_strengthReinforcement(principal_Stress_Dir);

						Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
						frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					}

					/////// R2 //// Fabrication requirement: only surface quality /////
					else if (Tetra->containProtectSurface) {

						QMeshFace* keptFace = this->_get_initial_kept_face(Tetra);
						Eigen::Vector3d normal = keptFace->normal_last.normalized(); // initial face normal

						normal = Tetra->R_estimate * normal;// virtual face normal after applying R_estimate

						if (fabs(normal(1)) > m_bandWidth_SQ_rotation) { // ny out of band

							Eigen::Matrix3d rotationMatrix = this->_cal_rotationMatrix_surfaceQuality(normal, Tetra, keptFace);

							Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
							frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
						}
					}
					else {

					}
				}
			}
			std::printf("Finish new local frame calculation.\n");

			if (innerLoop == (m_innerLoopTime - 3)) {
				this->m_keep_SQ_cover = 10;
				std::cout << "Increase the quaternion keep weight in innerloop " << innerLoop << " as " << this->m_keep_SQ_cover << std::endl;
			}

			//this->_globalQuaternionSmooth4_withoutBottom();
			this->_globalQuaternionSmooth4();

			for (int i = 0; i < tetPatch->GetTetraNumber(); i++) {
				QMeshTetra* Tetra = tetraPatch_elementSet[i];
				frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
			}
			std::printf("Finish quaternion smooth calculation.\n");

			this->_get_energy_innerLoop_supportLess();
			this->_get_energy_innerLoop_strengthReinforcement();
			this->_get_energy_innerLoop_surfaceQuality();
		}

		// Define variables
		Eigen::SparseMatrix<double> matrix_A_4x;
		Eigen::SparseMatrix<double> matrix_A_transpose_4x;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4x;
		//
		Eigen::SparseMatrix<double> matrix_A_4y;
		Eigen::SparseMatrix<double> matrix_A_transpose_4y;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4y;
		//
		Eigen::SparseMatrix<double> matrix_A_4z;
		Eigen::SparseMatrix<double> matrix_A_transpose_4z;
		Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4z;
		// Initialize variables
		matrix_A_4x.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4x.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4y.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4y.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
		matrix_A_4z.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
		matrix_A_transpose_4z.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));

		// Fill A matrix for x,y,z
		// Block 11 for x,y,z
		// Important issues
		//give memory to sparse matrix, to accerate the insert speed
		matrix_A_4x.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4y.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
		matrix_A_4z.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));

		float c1 = -0.25, c2 = 0.75;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			int fdx = Tetra->GetIndexNo() * 4;	int vdxArr[4];
			for (int i = 0; i < 4; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

			//double weight = 1.0;
			//if (Tetra->isOverhangTet)		weight = m_criticalTet_weight;
			double weight = this->_get_critcalTet_Weight(Tetra);

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (i == j) {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c2 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c2 * weight;
					}
					else {
						matrix_A_4x.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4y.insert(fdx + j, vdxArr[i]) = c1 * weight;
						matrix_A_4z.insert(fdx + j, vdxArr[i]) = c1 * weight;
					}
				}
			}
		}

		// Block 12 for x,y,z
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			//double weight = 1.0;
			//if (Tetra->isOverhangTet) weight = m_criticalTet_weight;
			double weight = this->_get_critcalTet_Weight(Tetra);

			for (int i = 0; i < 4; i++) {
				matrix_A_4x.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](0, i); // x row
				matrix_A_4y.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](1, i); // y row
				matrix_A_4z.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](2, i); // z row
			}
		}
		// Block 21 equals to zero
		// Block 22 for x,y,z
		int constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)tetPatch->GetFaceList().GetNext(Pos);

			if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_neighborScale_weight;
			matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_neighborScale_weight;

			constraint_ind++;
		}
		// Block 31 equals to zero
		// Block 32 for x,y,z
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			matrix_A_4x.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4y.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;
			matrix_A_4z.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_regularScale_weight;

			constraint_ind++;
		}

		std::cout << "Finish fill A matrix." << std::endl;

		long time = clock();

		// Factorize A matrix
		matrix_A_4x.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4x(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4x = matrix_A_4x.transpose();
		matATA_4x = matrix_A_transpose_4x * matrix_A_4x;
		Solver_ASAP_4x.compute(matATA_4x);

		matrix_A_4y.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4y(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4y = matrix_A_4y.transpose();
		matATA_4y = matrix_A_transpose_4y * matrix_A_4y;
		Solver_ASAP_4y.compute(matATA_4y);

		matrix_A_4z.makeCompressed();
		Eigen::SparseMatrix<double> matATA_4z(node_Num + tet_Num, node_Num + tet_Num);
		matrix_A_transpose_4z = matrix_A_4z.transpose();
		matATA_4z = matrix_A_transpose_4z * matrix_A_4z;
		Solver_ASAP_4z.compute(matATA_4z);

		std::printf("Finish factorize materix A\n");

		std::printf("The compress spends %f s.\n", (double(clock() - time)) / CLOCKS_PER_SEC);

		// Fill b vector for x,y,z
		//special case of regular term
		constraint_ind = 0;
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

			for (int i = 0; i < 3; i++) {

				vector_b[i](4 * tet_Num + neighborScale_face_num + constraint_ind) = m_regularScale_weight;
			}
			constraint_ind++;
		}

		// Solve x vector
		Eigen::VectorXd ATb_4x = matrix_A_transpose_4x * vector_b[0];
		vector_X_Pos_Scale[0] = Solver_ASAP_4x.solve(ATb_4x);

		Eigen::VectorXd ATb_4y = matrix_A_transpose_4y * vector_b[1];
		vector_X_Pos_Scale[1] = Solver_ASAP_4y.solve(ATb_4y);

		Eigen::VectorXd ATb_4z = matrix_A_transpose_4z * vector_b[2];
		vector_X_Pos_Scale[2] = Solver_ASAP_4z.solve(ATb_4z);

		// Update vertex
		for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);
			int idx = node->GetIndexNo();

			double xx, yy, zz;
			node->SetCoord3D(vector_X_Pos_Scale[0](idx), vector_X_Pos_Scale[1](idx), vector_X_Pos_Scale[2](idx));
			node->GetCoord3D(xx, yy, zz);
			//std::printf("New coordinate xx %f , yy %f , zz %f.\n", xx, yy, zz);
		}

		// record the scale value
		for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);
			int fdx = Tetra->GetIndexNo();

			for (int i = 0; i < 3; i++) {
				Tetra->scaleValue_vector[i] = vector_X_Pos_Scale[i](node_Num + fdx);
				//std::cout << vector_X_Pos_Scale[i](node_Num + fdx) << " ";
			}
			//std::cout << std::endl;
		}

		this->_cal_virtual_Vfield();
		std::printf("End calculation of loop %d.\n\n", loop);
	}

	// detact and draw overhang face
	//tetPatch->drawStressField = false;
	this->m_supportFreeAngle = 45.0;
	this->_detectOverhangFace();
	tetPatch->drawOverhangSurface = true;
}

//only for better draw, it will be replaced by the position of original model 
void DeformTet::postProcess() {

	this->_moveModel2Center();
}

void DeformTet::_moveModel2Center() {

	double centerXx, centerYy, centerZz;
	double minYy = 9999.99;
	for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);
		if (minYy > yy) minYy = yy;
		centerXx += xx; centerYy += yy; centerZz += zz;
	}

	centerXx /= tetPatch->GetNodeNumber();
	centerYy /= tetPatch->GetNodeNumber();
	centerZz /= tetPatch->GetNodeNumber();

	for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);

		xx -= centerXx;
		yy -= minYy;
		zz -= centerZz;

		node->SetCoord3D(xx, yy, zz);
		//node->SetCoord3D_last(xx, yy, zz);
	}
}

// Ref:https://blog.csdn.net/xuejinglingai/article/details/113267713?utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EsearchFromBaidu%7Edefault-1.pc_relevant_baidujshouduan&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EsearchFromBaidu%7Edefault-1.pc_relevant_baidujshouduan#t6
void DeformTet::find_bestPrintingOrientation(bool is_Compute) {

	// Tips: please increase the bottom_height to 10.0 to make the effect of bottom face less
	double m_bottom_height_temp = m_bottom_height_temp + 4.0;
	Eigen::Matrix3d rotationMatrix;
	Eigen::Vector3d bestOrientation_arrow = printDir;

	if (is_Compute) {
		// get uniformed sample normal
		int nums_points = 100;
		double radius = 1.0;
		Eigen::MatrixXd loc = Eigen::MatrixXd::Zero(nums_points + 1, 3);
		for (int ii = 0; ii < nums_points; ii++) {
			double phi = acos(-1.0 + (2.0 * (ii + 1) - 1.0) / nums_points);
			double theta = sqrt(nums_points * PI) * phi;
			loc(ii, 0) = radius * cos(theta) * sin(phi);
			loc(ii, 1) = radius * sin(theta) * sin(phi);
			loc(ii, 2) = radius * cos(phi);
		}
		loc.row(nums_points) << 0.0, 1.0, 0.0;

		// get overhang Area
		Eigen::VectorXd eachOverhangArea(nums_points + 1);
		for (int ii = 0; ii < nums_points + 1; ii++) {

			//move coordinate of node
			rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(printDir, loc.row(ii));

			for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
				QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

				Eigen::Vector3d temp_Position;
				node->GetCoord3D_last(temp_Position[0], temp_Position[1], temp_Position[2]);

				temp_Position = rotationMatrix * temp_Position;
				node->SetCoord3D(temp_Position[0], temp_Position[1], temp_Position[2]);
			}

			//mark overhang face
			for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
				QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);
				face->isOverhangFace = false;

				if (face->GetLeftTetra() == NULL || face->GetRightTetra() == NULL) {
					Eigen::Vector3d normal; double d;
					face->CalPlaneEquation();
					face->GetPlaneEquation(normal(0), normal(1), normal(2), d);
					double x = m_supportFreeAngle * PI / 180;

					if (-normal.dot(printDir) < -sin(x))
						face->isOverhangFace = true;
				}
			}
			// find the minimal height // bottom should not be consindered as overhang face
			double minHight = 99999.0;
			for (GLKPOSITION pos = tetPatch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
				QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(pos);
				double pp[3] = { 0 }; node->GetCoord3D(pp[0], pp[1], pp[2]);
				if (pp[1] < minHight) minHight = pp[1];
			}
			for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
				QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);
				double pp[3] = { 0 };
				for (int i = 0; i < 3; i++) {
					face->GetNodeRecordPtr(i)->GetCoord3D(pp[0], pp[1], pp[2]);
					if (pp[1] < minHight + m_bottom_height_temp) {
						face->isOverhangFace = false;
						break;
					}
				}
			}
			// record the area of all of overhang faces
			double areaAlloverHangFace = 0.0;
			for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
				QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);

				if (face->isOverhangFace)	areaAlloverHangFace += face->CalArea();
			}
			eachOverhangArea[ii] = areaAlloverHangFace;

			std::cout << ".";
		}
		std::cout << std::endl;
		//std::cout << eachOverhangArea << std::endl << loc << std::endl;

		int ind;
		eachOverhangArea.minCoeff(&ind);
		bestOrientation_arrow = loc.row(ind);
		std::cout << "The minimal overhang area is " << eachOverhangArea[ind]
			<< ", the orientation is " << bestOrientation_arrow.transpose() << std::endl;
	}
	else {
		if (tetModel_Name == "fertility") bestOrientation_arrow = { 1.0, -1.0 , 0.0 };
		if (tetModel_Name == "topopt_new3") bestOrientation_arrow = { 1.0, -1.2 , 0.0 };
		//if (tetModel_Name == "topopt_new4") bestOrientation_arrow = { 1.0, -1.14 , 0.0 };
		//if (tetModel_Name == "topopt_new4") bestOrientation_arrow = { -0.2, -0.2 , 1.0 };
		if (tetModel_Name == "hook") bestOrientation_arrow = { 1.0, 0.0 , 0.0 };

		bestOrientation_arrow.normalize();
	}

	//move coordinate of node into best one
	rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(printDir, bestOrientation_arrow);

	for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

		Eigen::Vector3d temp_Position;
		node->GetCoord3D_last(temp_Position[0], temp_Position[1], temp_Position[2]);

		temp_Position = rotationMatrix * temp_Position;
		node->SetCoord3D(temp_Position[0], temp_Position[1], temp_Position[2]);
		node->SetCoord3D_last(temp_Position[0], temp_Position[1], temp_Position[2]);
	}

	//rotate the tau_max
	for (GLKPOSITION Pos = tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(Pos);

		if(Tetra->isTensileorCompressSelect)	Tetra->tau_max = rotationMatrix * Tetra->tau_max;
	}

	//clean the overhangFace flag
	for (GLKPOSITION pos = tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(pos);
		face->isOverhangFace = false;
	}

	// update initial information
	this->_moveModelup2ZeroHeight(tetPatch); //should before the initialCoord3d record
	this->_record_initial_coord3D();
	this->_record_initial_normal3D();
	this->_compTetMeshVolumeMatrix(tetPatch);
	if (m_caseType == 1	|| m_caseType == 4) 
		this->_detectBottomTet(m_bottom_height); 
}

void DeformTet::_check_parameters() {

	std::cout << "------------------------------------------------------"
		<< "\n---->m_loopTime:  " << m_loopTime
		<< "\n---->m_innerLoopTime:  " << m_innerLoopTime
		<< "\n---------------------------------"
		<< "\n---->m_criticalTet_weight_SL:  " << m_criticalTet_weight_SL
		<< "\n---->m_criticalTet_weight_SR:  " << m_criticalTet_weight_SR
		<< "\n---->m_criticalTet_weight_SQ_cover:  " << m_criticalTet_weight_SQ_cover
		<< "\n---->m_criticalTet_weight_SQ_vertical:  " << m_criticalTet_weight_SQ_vertical
		<< "\n---------------------------------"
		<< "\n---->m_neighborScale_weight:  " << m_neighborScale_weight
		<< "\n---->m_regularScale_weight:  " << m_regularScale_weight
		<< "\n---->m_globalSmooth_weight:  " << m_globalSmooth_weight
		<< "\n---------------------------------"
		<< "\n---->m_keep_SL:  " << m_keep_SL
		<< "\n---->m_keep_SR:  " << m_keep_SR
		<< "\n---->m_keep_SQ_cover:  " << m_keep_SQ_cover
		<< "\n---->m_keep_SQ_vertical:  " << m_keep_SQ_vertical
		<< "\n------------------------------------------------------"
		<< std::endl;
}

Eigen::Vector3d DeformTet::_get_initial_overhang_faceNormal(QMeshTetra* Tetra) {

	// get current face normal
	QMeshFace* eachFace = NULL;		int overhangFace_Num = 0;
	Eigen::Vector3d whole_overhang_face_normal = Eigen::Vector3d::Zero();
	for (int i = 0; i < 4; i++) {
		eachFace = Tetra->GetFaceRecordPtr(i + 1);
		if (eachFace->isOverhangFace) {

			Eigen::Vector3d normal;
			normal = eachFace->normal_last; // initial face normal
			normal = -normal;
			normal.normalize();
			whole_overhang_face_normal += normal;
			overhangFace_Num++;
			break; // only consider one overhang face for one tet element
		}
	}
	if (overhangFace_Num == 0) std::cout << "ERROR: none of overhang face is detected!" << std::endl;
	whole_overhang_face_normal.normalize();

	return whole_overhang_face_normal;
}

QMeshFace* DeformTet::_get_initial_overhang_face(QMeshTetra* Tetra) {

	QMeshFace* overhangFace = NULL;		int overhangFace_Num = 0;
	for (int i = 0; i < 4; i++) {
		QMeshFace* eachFace = Tetra->GetFaceRecordPtr(i + 1);
		if (eachFace->isOverhangFace) {
			if (eachFace->inner)
				std::cout << "Error: overhang face should not be inner face." << std::endl;

			overhangFace = eachFace;
			overhangFace_Num++;
		}
	}
	if (overhangFace_Num != 1) 
		std::cout << "ERROR: the num of overhang face should be one, please check!" << std::endl;

	return overhangFace;
}

Eigen::Matrix3d DeformTet::_cal_rotationMatrix_supportLess(Eigen::Vector3d face_Normal_Dir, QMeshTetra* Tetra) {

	Eigen::Vector3d r_axis = face_Normal_Dir.cross(Tetra->vectorField_4voxelOrder_now.normalized());
	r_axis.normalize();
	Eigen::AngleAxis<double> sptFree_rotation_1
		= Eigen::AngleAxis<double>((this->m_supportFreeAngle + 90.0) * PI / 180.0, r_axis);
	Eigen::Vector3d sptFree_normal_1 = sptFree_rotation_1.matrix() * face_Normal_Dir;
	Eigen::AngleAxis<double> sptFree_rotation_2
		= Eigen::AngleAxis<double>(-1.0 * (this->m_supportFreeAngle + 90.0) * PI / 180.0, r_axis);
	Eigen::Vector3d sptFree_normal_2 = sptFree_rotation_2.matrix() * face_Normal_Dir;

	Eigen::Vector3d sptFree_normal = Eigen::Vector3d::Zero();
	double angel_n_nCandidate1 = acos(sptFree_normal_1.dot(Tetra->vectorField_4voxelOrder_now.normalized()));
	double angel_n_nCandidate2 = acos(sptFree_normal_2.dot(Tetra->vectorField_4voxelOrder_now.normalized()));

	if (angel_n_nCandidate1 < angel_n_nCandidate2) { sptFree_normal = sptFree_normal_1; }
	else { sptFree_normal = sptFree_normal_2; }

	Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
	rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(sptFree_normal, this->printDir);

	return rotationMatrix;
}

Eigen::Matrix3d DeformTet::_cal_rotationMatrix_strengthReinforcement(Eigen::Vector3d principal_Stress_Dir) {

	Eigen::Vector3d projectDir; // assume the printing direction is (0,1,0)
	projectDir << principal_Stress_Dir(0), 0.0, principal_Stress_Dir(2);
	projectDir.normalized();

	Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();

	rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(principal_Stress_Dir, projectDir);

	return rotationMatrix;
}

Eigen::Matrix3d DeformTet::_cal_rotationMatrix_strengthReinforcement_stressLine(
	Eigen::Vector3d principal_Stress_Dir, QMeshTetra* Tetra) {

	Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
	Eigen::Vector3d coord3D_eleCenter = Eigen::Vector3d::Zero();
	Tetra->CalCenterPos(coord3D_eleCenter[0], coord3D_eleCenter[1], coord3D_eleCenter[2]);
		
	if (coord3D_eleCenter[0] > 60 && coord3D_eleCenter[0] < 140 )
	{

		Eigen::Vector3d projectDir; // assume the printing direction is (0,1,0)
		projectDir << principal_Stress_Dir(0), 0.0, principal_Stress_Dir(2);
		projectDir.normalized();

		rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(principal_Stress_Dir, projectDir);
	}
	
	if (coord3D_eleCenter[0] > 0)
	{
		Eigen::Vector3d source = { 0.0, 1.0, 0.0 };
		double t = ((inital_range[1] - coord3D_eleCenter[0]) / (inital_range[1] - inital_range[0])) * PI * 0.7;
		Eigen::Vector3d target = { 0.0, sin(t), cos(t) };
		//std::cout << "target vector :" << target.transpose() << std::endl;
		rotationMatrix = rotationMatrix * Eigen::Quaterniond().setFromTwoVectors(source, target);
	}

	return rotationMatrix;
}

// one tet only has one kept face
QMeshFace* DeformTet::_get_initial_kept_face(QMeshTetra* Tetra){

	QMeshFace* criticalFace = NULL;
	for (int i = 0; i < 4; i++) {
		QMeshFace* thisFace = Tetra->GetFaceRecordPtr(i + 1);
		if (thisFace->isQualityProtectFace) {
			criticalFace = thisFace;
			break;
		}
	}
	if (criticalFace == NULL) std::cout << "ERROR! No quality protect face" << std::endl;

	return criticalFace;
}

Eigen::Matrix3d DeformTet::_cal_rotationMatrix_surfaceQuality(Eigen::Vector3d normal, QMeshTetra* Tetra, QMeshFace* criticalFace) {

	double bandWidth = m_bandWidth_SQ_rotation;
	// angle between (0 1 0) and face normal
	double angle_dp_nf = 0.0;
	if (normal(1) > 0)	angle_dp_nf = acos(this->printDir.dot(normal));
	else angle_dp_nf = acos(-this->printDir.dot(normal));

	// angle between face normal and band
	Eigen::Vector3d r_axis = Eigen::Vector3d::Zero();
	Eigen::Vector3d sufKeep_normal = Eigen::Vector3d::Zero();
	if (normal(1) > 0) {
		r_axis = this->printDir.cross(normal); // rotation axis // "+"
		r_axis.normalize();
		Eigen::AngleAxis<double> sufKeep_rotation
			= Eigen::AngleAxis<double>(PI / 2 - asin(bandWidth), r_axis);
		sufKeep_normal = sufKeep_rotation.matrix() * this->printDir; // "+"
	}
	else {
		r_axis = -this->printDir.cross(normal); // rotation axis // "-"
		r_axis.normalize();
		Eigen::AngleAxis<double> sufKeep_rotation
			= Eigen::AngleAxis<double>(PI / 2 - asin(bandWidth), r_axis);
		sufKeep_normal = sufKeep_rotation.matrix() * (-this->printDir); // "-"
	}
	double angle_nf_band = acos(sufKeep_normal.dot(normal));

	//reset the flag
	criticalFace->select_coverLayer = 0;
	Tetra->select_coverLayer = 0;

	// decide which side to go
	Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
	if (angle_nf_band < angle_dp_nf) { //band rotation
		rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(normal, sufKeep_normal);
		criticalFace->select_coverLayer = 1;
		Tetra->select_coverLayer = 1;
	}
	else {// large cover
		criticalFace->select_coverLayer = 2;
		Tetra->select_coverLayer = 2;
		if (normal(1) > 0)
			rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(normal, this->printDir);// "+"
		else
			rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(normal, -this->printDir);// "-"
	}
	return rotationMatrix;
}

Eigen::Matrix3d DeformTet::_cal_rotationMatrix_surfaceQuality(Eigen::Vector3d normal, QMeshTetra* Tetra, QMeshFace* criticalFace, bool update_coverFlag) {

	double bandWidth = m_bandWidth_SQ_rotation;
	// angle between (0 1 0) and face normal
	double angle_dp_nf = 0.0;
	if (normal(1) > 0)	angle_dp_nf = acos(this->printDir.dot(normal));
	else angle_dp_nf = acos(-this->printDir.dot(normal));

	// angle between face normal and band
	Eigen::Vector3d r_axis = Eigen::Vector3d::Zero();
	Eigen::Vector3d sufKeep_normal = Eigen::Vector3d::Zero();
	if (normal(1) > 0) {
		r_axis = this->printDir.cross(normal); // rotation axis // "+"
		r_axis.normalize();
		Eigen::AngleAxis<double> sufKeep_rotation
			= Eigen::AngleAxis<double>(PI / 2 - asin(bandWidth), r_axis);
		sufKeep_normal = sufKeep_rotation.matrix() * this->printDir; // "+"
	}
	else {
		r_axis = -this->printDir.cross(normal); // rotation axis // "-"
		r_axis.normalize();
		Eigen::AngleAxis<double> sufKeep_rotation
			= Eigen::AngleAxis<double>(PI / 2 - asin(bandWidth), r_axis);
		sufKeep_normal = sufKeep_rotation.matrix() * (-this->printDir); // "-"
	}
	double angle_nf_band = acos(sufKeep_normal.dot(normal));

	
	// decide which side to go
	Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
	if (update_coverFlag) {

		//reset the flag
		criticalFace->select_coverLayer = 0;
		Tetra->select_coverLayer = 0;

		if (angle_nf_band < angle_dp_nf) { //band rotation
			rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(normal, sufKeep_normal);
			criticalFace->select_coverLayer = 1;
			Tetra->select_coverLayer = 1;
		}
		else {// large cover
			criticalFace->select_coverLayer = 2;
			Tetra->select_coverLayer = 2;
			if (normal(1) > 0)
				rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(normal, this->printDir);// "+"
			else
				rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(normal, -this->printDir);// "-"
		}
	}
	else {
		if (Tetra->select_coverLayer == 2) {
			if (normal(1) > 0)
				rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(normal, this->printDir);// "+"
			else
				rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(normal, -this->printDir);// "-"
		}
		else if (Tetra->select_coverLayer == 1) {
			rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(normal, sufKeep_normal);
		}
	}

	return rotationMatrix;
}

double DeformTet::_get_globalSmooth_keepWeight(QMeshTetra* Tetra) {

	//TODO: consider the bottom element;

	double weight = 1.0;
		
	if (Tetra->containProtectSurface && Tetra->select_coverLayer == 2)			weight = m_keep_SQ_cover;
	else if (Tetra->containProtectSurface && Tetra->select_coverLayer == 1)		weight = m_keep_SQ_vertical;
	else																		weight = 1.0;

	if (Tetra->isOverhangTet || Tetra->isBottomTet)								weight = m_keep_SL;
	if (Tetra->isTensileorCompressSelect)										weight = m_keep_SR;

	return weight;
}

double DeformTet::_get_critcalTet_Weight(QMeshTetra* Tetra) {

	double weight = 1.0;

	if (Tetra->containProtectSurface && Tetra->select_coverLayer == 2)			weight = m_criticalTet_weight_SQ_cover;
	else if (Tetra->containProtectSurface && Tetra->select_coverLayer == 1)		weight = m_criticalTet_weight_SQ_vertical;
	else																		weight = 1.0;

	if (Tetra->isOverhangTet)													weight = m_criticalTet_weight_SL;
	if (Tetra->isTensileorCompressSelect)										weight = m_criticalTet_weight_SR;

	return weight;
}

Eigen::Matrix3d DeformTet::_cal_rotationMatrix_SL_SQ(Eigen::Vector3d overhangFace_normal_new, QMeshTetra* Tetra) {

	// plane: ( vertical to n_f with D(+/-0.1) )
	//  keepFace_normal_new = overhangFace_normal_new
	Eigen::Vector3d n_plane = overhangFace_normal_new;
	//n_plane = { -0.5, -sqrt(3)/2 ,0 };
	n_plane.normalize();
	//double D1 = m_bandWidth_SQ_rotation; double D2 = -D1;
	// circle: (d_g & n_f)
	Eigen::Vector3d rAxis = overhangFace_normal_new.cross(Tetra->vectorField_4voxelOrder_now.normalized());
	//rAxis = { 0,0,1 };
	rAxis.normalize();
	Eigen::Vector3d i = { 1.0,0.0,0.0 };	Eigen::Vector3d k = { 0.0,0.0,1.0 };
	Eigen::Vector3d a = rAxis.cross(i);
	if ((a[0] == 0) && (a[1] == 0) && (a[2] == 0)) {

		a = rAxis.cross(k);
	}
	a.normalize();
	Eigen::Vector3d b = rAxis.cross(a);
	b.normalize();

	double m = n_plane.dot(a); 
	if (m == 0.0) {
		std::cout << "error: the m should not be zero, please check!\n " <<
			"C:/Users/zhang/Dropbox/Siggraph_2022/Randering/Matlab_code/intersection_plane_circle.m " << std::endl;
		/*std::cout << a.transpose() << "   \n" << n_plane.transpose() << std::endl;*/
		//std::cout << "overhangFace_normal_new " << overhangFace_normal_new.transpose() << std::endl;
		//std::cout << "Tetra->R_estimate " << Tetra->R_estimate.transpose() << std::endl;
	}
	double n = n_plane.dot(b); 

	double theta = atan(-m / n);
	Eigen::Vector3d intersect_Pnt_1 = a * cos(theta) + b * sin(theta);
	//intersect_Pnt_1 << intersect_Pnt_1(2), intersect_Pnt_1(1), intersect_Pnt_1(0);
	theta += PI;
	Eigen::Vector3d intersect_Pnt_2 = a * cos(theta) + b * sin(theta);
	//intersect_Pnt_2 << intersect_Pnt_2(2), intersect_Pnt_2(1), intersect_Pnt_2(0);

	//Tetra->vectorField_4voxelOrder_now << sqrt(3) / 2, 0.5, 0;
	//Tetra->vectorField = Tetra->vectorField_4voxelOrder_now;
	//Tetra->isCollisionTetra = true;

	//5 candidates
	Eigen::MatrixXd candidate_Vec(5, 3);
	candidate_Vec.setZero();
	//one
	Eigen::AngleAxis<double> sl_sq_R_1 = Eigen::AngleAxis<double>(asin(this->m_bandWidth_SQ_rotation), rAxis);  //"+" angle /intersect_Pnt_1
	candidate_Vec.row(0) << (sl_sq_R_1.matrix() * intersect_Pnt_1).transpose();
	//std::cout << "----------- sl_sq_R_1.matrix(): ------------ \n" << sl_sq_R_1.matrix() << std::endl;
	//std::cout << "----------- sl_sq_R_1.matrix() * intersect_Pnt_1: ------------ \n" << sl_sq_R_1.matrix() * intersect_Pnt_1 << std::endl;
	//two
	Eigen::AngleAxis<double> sl_sq_R_2 = Eigen::AngleAxis<double>(-asin(this->m_bandWidth_SQ_rotation), rAxis); //"-" angle /intersect_Pnt_1
	candidate_Vec.row(1) << (sl_sq_R_2.matrix() * intersect_Pnt_1).transpose();
	//std::cout << "----------- sl_sq_R_2.matrix(): ------------ \n" << sl_sq_R_2.matrix() << std::endl;
	//std::cout << "----------- sl_sq_R_2.matrix() * intersect_Pnt_1: ------------ \n" << sl_sq_R_2.matrix() * intersect_Pnt_1 << std::endl;
	
	//three
	Eigen::AngleAxis<double> sl_sq_R_3 = Eigen::AngleAxis<double>(asin(this->m_bandWidth_SQ_rotation), rAxis);  //"+" angle /intersect_Pnt_2
	candidate_Vec.row(2) << (sl_sq_R_3.matrix() * intersect_Pnt_2).transpose();
	//four
	Eigen::AngleAxis<double> sl_sq_R_4 = Eigen::AngleAxis<double>(-asin(this->m_bandWidth_SQ_rotation), rAxis); //"-" angle /intersect_Pnt_2
	candidate_Vec.row(3) << (sl_sq_R_4.matrix() * intersect_Pnt_2).transpose();
	//five
	candidate_Vec.row(4) << overhangFace_normal_new.transpose();

	//std::cout << "----------- candidate_Vec: ------------ \n" << candidate_Vec << std::endl;

	Eigen::Vector3d selected_normal = Eigen::Vector3d::Zero();
	double min_angel_n_nCandidate = PI;
	for (int i = 0; i < 5; i++) {
		
		double angel_n_nCandidate = acos(candidate_Vec.row(i).dot(Tetra->vectorField_4voxelOrder_now.normalized()));
		if (angel_n_nCandidate < min_angel_n_nCandidate) { 
			selected_normal = candidate_Vec.row(i); 
			min_angel_n_nCandidate = angel_n_nCandidate;
		}
	}

	//std::cout << "----------- selected_normal: ------------ \n" << selected_normal.transpose() << std::endl;

	Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
	rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(selected_normal, this->printDir);

	return rotationMatrix;
}

Eigen::Matrix3d DeformTet::_cal_rotationMatrix_SR_SQ(
	Eigen::Vector3d principal_Stress_Dir, Eigen::Vector3d keptFace_normal, QMeshTetra* Tetra) {

	Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();

	// plane: ( vertical to n_f with D(+/-0.1) )
	Eigen::Vector3d n_plane = keptFace_normal;
	n_plane.normalize();
	// circle: (tau_max')
	Eigen::Vector3d rAxis = principal_Stress_Dir;
	rAxis.normalize();

	// special case: plane and circle are nearly parallel
	double angle_plane_circle = acos(n_plane.dot(rAxis));
	if (angle_plane_circle < asin(0.1)) {
		rotationMatrix = Eigen::Matrix3d::Identity();
		return rotationMatrix;
	}

	Eigen::Vector3d i = { 1.0,0.0,0.0 };	Eigen::Vector3d k = { 0.0,0.0,1.0 };
	Eigen::Vector3d a = rAxis.cross(i);
	if ((a[0] == 0) && (a[1] == 0) && (a[2] == 0)) {

		a = rAxis.cross(k);
	}
	a.normalize();
	Eigen::Vector3d b = rAxis.cross(a);
	b.normalize();

	double m = n_plane.dot(a);
	if (m == 0.0) {
		std::cout << "error: the m should not be zero, please check!\n " <<
			"C:/Users/zhang/Dropbox/Siggraph_2022/Randering/Matlab_code/intersection_plane_circle.m " << std::endl;
	}
	double n = n_plane.dot(b);

	double theta = atan(-m / n);
	Eigen::Vector3d intersect_Pnt_1 = a * cos(theta) + b * sin(theta);
	theta += PI;
	Eigen::Vector3d intersect_Pnt_2 = a * cos(theta) + b * sin(theta);

	//6 candidates
	Eigen::MatrixXd candidate_Vec(6, 3);
	candidate_Vec.setZero();
	//one
	Eigen::AngleAxis<double> sl_sq_R_1 = Eigen::AngleAxis<double>(asin(this->m_bandWidth_SQ_rotation), rAxis);  //"+" angle /intersect_Pnt_1
	candidate_Vec.row(0) << (sl_sq_R_1.matrix() * intersect_Pnt_1).transpose();
	//std::cout << "----------- sl_sq_R_1.matrix(): ------------ \n" << sl_sq_R_1.matrix() << std::endl;
	//std::cout << "----------- sl_sq_R_1.matrix() * intersect_Pnt_1: ------------ \n" << sl_sq_R_1.matrix() * intersect_Pnt_1 << std::endl;
	//two
	Eigen::AngleAxis<double> sl_sq_R_2 = Eigen::AngleAxis<double>(-asin(this->m_bandWidth_SQ_rotation), rAxis); //"-" angle /intersect_Pnt_1
	candidate_Vec.row(1) << (sl_sq_R_2.matrix() * intersect_Pnt_1).transpose();
	//std::cout << "----------- sl_sq_R_2.matrix(): ------------ \n" << sl_sq_R_2.matrix() << std::endl;
	//std::cout << "----------- sl_sq_R_2.matrix() * intersect_Pnt_1: ------------ \n" << sl_sq_R_2.matrix() * intersect_Pnt_1 << std::endl;

	//three
	Eigen::AngleAxis<double> sl_sq_R_3 = Eigen::AngleAxis<double>(asin(this->m_bandWidth_SQ_rotation), rAxis);  //"+" angle /intersect_Pnt_2
	candidate_Vec.row(2) << (sl_sq_R_3.matrix() * intersect_Pnt_2).transpose();
	//four
	Eigen::AngleAxis<double> sl_sq_R_4 = Eigen::AngleAxis<double>(-asin(this->m_bandWidth_SQ_rotation), rAxis); //"-" angle /intersect_Pnt_2
	candidate_Vec.row(3) << (sl_sq_R_4.matrix() * intersect_Pnt_2).transpose();
	//five
	candidate_Vec.row(4) << n_plane.transpose();
	//six
	candidate_Vec.row(5) << -n_plane.transpose();

	//std::cout << "----------- candidate_Vec: ------------ \n" << candidate_Vec << std::endl;

	Eigen::Vector3d selected_normal = Eigen::Vector3d::Zero();
	double min_angel_n_nCandidate = PI;
	for (int i = 0; i < 6; i++) {

		double angel_n_nCandidate = acos(candidate_Vec.row(i).dot(this->printDir));
		if (angel_n_nCandidate < min_angel_n_nCandidate) {
			selected_normal = candidate_Vec.row(i);
			min_angel_n_nCandidate = angel_n_nCandidate;
		}
	}

	//std::cout << "----------- selected_normal: ------------ \n" << selected_normal.transpose() << std::endl;

	
	rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(selected_normal, this->printDir);

	return rotationMatrix;
}

Eigen::Matrix3d DeformTet::_cal_rotationMatrix_SL_SR(
	Eigen::Vector3d overhangFace_normal, Eigen::Vector3d principal_Stress_Dir, QMeshTetra* Tetra) {

	overhangFace_normal.normalize();
	principal_Stress_Dir.normalize();

	Eigen::Vector3d selected_normal = Eigen::Vector3d::Zero();
	Eigen::MatrixXd candidate_Vec(2, 3);
	candidate_Vec.setZero();
	double min_angel_n_nCandidate = PI;
	/*
	//double instersection node
	if (fabs(overhangFace_normal.dot(principal_Stress_Dir)) < 
		cos((m_supportFreeAngle * PI / 180.0))
		){

		Eigen::Vector3d vec = Eigen::Vector3d::Zero();
		Eigen::Vector3d node = Eigen::Vector3d::Zero();
		bool isIntersect_2plane = _intersect3D_plane_plane(
			overhangFace_normal, sin((m_supportFreeAngle * PI / 180.0)),
			principal_Stress_Dir, 0.0, node, vec);
		if (isIntersect_2plane) {
			Eigen::Vector3d center = Eigen::Vector3d::Zero();
			double mu1, mu2 = 0.0;
			bool isIntersect_line_sphere = _intersect3D_line_sphere(node, node + vec, center, 1.0, mu1, mu2);
			if (isIntersect_line_sphere) {
				candidate_Vec.row(0) = (node + mu1 * vec).transpose(); //p1
				candidate_Vec.row(1) = (node + mu2 * vec).transpose(); //p2
				std::cout << "0. find two intersection node, please check.\n";
			}
			else {
				std::cout << "1. there is no intersection node, please check.\n";
			}
		}
		else 
			std::cout << "2. there is no intersection line, please check.\n";

	}
	//infinite node on the ring (band)
	else {

		Eigen::Vector3d rAxis = principal_Stress_Dir.cross(this->printDir);
		rAxis.normalize();
		Eigen::AngleAxis<double> SL_SR_rotation	= Eigen::AngleAxis<double>(PI / 2, rAxis);
		candidate_Vec.row(0) = (SL_SR_rotation.matrix() * principal_Stress_Dir).transpose(); // "+"
		SL_SR_rotation = Eigen::AngleAxis<double>(-PI / 2, rAxis);
		candidate_Vec.row(1) = (SL_SR_rotation.matrix() * principal_Stress_Dir).transpose(); // "-"
	}
	*/

	//double instersection node
	Eigen::Vector3d vec = Eigen::Vector3d::Zero();
	Eigen::Vector3d node = Eigen::Vector3d::Zero();
	bool isIntersect_2plane = _intersect3D_plane_plane(
		overhangFace_normal, sin((m_supportFreeAngle * PI / 180.0)),
		principal_Stress_Dir, 0.0, node, vec);
	if (isIntersect_2plane) {
		Eigen::Vector3d center = Eigen::Vector3d::Zero();
		double mu1, mu2 = 0.0;
		bool isIntersect_line_sphere = _intersect3D_line_sphere(node, node + vec, center, 1.0, mu1, mu2);
		if (isIntersect_line_sphere) {
			candidate_Vec.row(0) = (node + mu1 * vec).transpose(); //p1
			candidate_Vec.row(1) = (node + mu2 * vec).transpose(); //p2
			std::cout << "0. find two intersection node.\n";
		}
		else {
			Eigen::Vector3d rAxis = principal_Stress_Dir.cross(this->printDir);
			rAxis.normalize();
			Eigen::AngleAxis<double> SL_SR_rotation = Eigen::AngleAxis<double>(PI / 2, rAxis);
			candidate_Vec.row(0) = (SL_SR_rotation.matrix() * principal_Stress_Dir).transpose(); // "+"
			SL_SR_rotation = Eigen::AngleAxis<double>(-PI / 2, rAxis);
			candidate_Vec.row(1) = (SL_SR_rotation.matrix() * principal_Stress_Dir).transpose(); // "-"
			std::cout << "1. there is no intersection node.\n";
		}
	}
	else
		std::cout << "2. there is no intersection line, please check.\n";


	for (int i = 0; i < 2; i++) {

		double angel_n_nCandidate = acos(candidate_Vec.row(i).dot(this->printDir));
		if (angel_n_nCandidate < min_angel_n_nCandidate) {
			selected_normal = candidate_Vec.row(i);
			min_angel_n_nCandidate = angel_n_nCandidate;
		}
	}

	Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
	rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(selected_normal, this->printDir);
	//std::cout << "\n/--------------_cal_rotationMatrix_SL_SR once. ---------------/" << std::endl;
	return rotationMatrix;
}

Eigen::Matrix3d DeformTet::_cal_rotationMatrix_SL_SR_Iter(
	Eigen::Vector3d overhangFace_normal, Eigen::Vector3d principal_Stress_Dir,
	QMeshTetra* Tetra, int innerLoop) {

	overhangFace_normal.normalize();
	principal_Stress_Dir.normalize();

	Eigen::Vector3d current_pos = Tetra->R_estimate.col(1);
	current_pos.normalize();

	Eigen::Vector3d selected_normal = this->printDir;

	//std::cout << "the innerLoop number is " << innerLoop << std::endl;

	if (innerLoop % 2 == 0) {
		selected_normal = _projection_vector2Plane(current_pos, principal_Stress_Dir);
	}
	else {
		//cone face equation is (overhangFace_normal).dot(X Y Z) + sin((m_supportFreeAngle * PI / 180.0)
		//need further projected onto the support-free cone
		if (overhangFace_normal.dot(current_pos) < sin((m_supportFreeAngle * PI / 180.0))) {
			Eigen::Vector3d proj_vec_support = _projection_vector2Plane(current_pos, overhangFace_normal);
			proj_vec_support.normalize();
			Eigen::Vector3d r_axis = overhangFace_normal.cross(overhangFace_normal);
			r_axis.normalize();
			selected_normal = _rotate_vector_deg(proj_vec_support, r_axis, 45.0);
		}
	}

	Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
	rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(selected_normal, this->printDir);
	//std::cout << "\n/--------------_cal_rotationMatrix_SL_SR_Iter once. ---------------/" << std::endl;
	return rotationMatrix;
}

Eigen::Matrix3d DeformTet::_cal_rotationMatrix_SL_SR_Iter(
	Eigen::Vector3d overhangFace_normal, Eigen::Vector3d principal_Stress_Dir, QMeshTetra* Tetra) {

	Eigen::Vector3d selected_normal = Eigen::Vector3d::Zero();

	overhangFace_normal.normalize();
	principal_Stress_Dir.normalize();

	Eigen::Vector3d current_pos = Tetra->R_estimate.col(1);
	current_pos.normalize();
	Eigen::Vector3d proj_vec_stress = _projection_vector2Plane(current_pos, principal_Stress_Dir);

	//cone face equation is (overhangFace_normal).dot(X Y Z) + sin((m_supportFreeAngle * PI / 180.0)
	//need further projected onto the support-free cone
	if (overhangFace_normal.dot(current_pos) < sin((m_supportFreeAngle * PI / 180.0))) {
		proj_vec_stress.normalize();
		Eigen::Vector3d proj_vec_support = _projection_vector2Plane(proj_vec_stress, overhangFace_normal);
		proj_vec_support.normalize();
		Eigen::Vector3d r_axis = overhangFace_normal.cross(proj_vec_stress);
		r_axis.normalize();
		selected_normal = _rotate_vector_deg(proj_vec_support, r_axis, 45.0);
	}
	//already stay in the support-free cone
	else {
		selected_normal = proj_vec_stress;
	}

	Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
	rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(selected_normal, this->printDir);
	//std::cout << "\n/--------------_cal_rotationMatrix_SL_SR_Iter once. ---------------/" << std::endl;
	return rotationMatrix;
}

// plane_2_plane intersection
// plane 1: n1 = (a1, b1, c1) d1 --> a1x + b1y + c1z + d1 = 0
// plane 2: n2 = (a2, b2, c2) d2-- > a2x + b2y + c2z + d2 = 0
// reference: https://blog.csdn.net/qinqinxiansheng/article/details/104615531?spm=1001.2101.3001.6650.6&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ERate-6.pc_relevant_default&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ERate-6.pc_relevant_default&utm_relevant_index=9
bool DeformTet::_intersect3D_plane_plane(
	Eigen::Vector3d n1, double d1, Eigen::Vector3d n2, double d2, 
	Eigen::Vector3d& node, Eigen::Vector3d& vec) {
	// get vector
	vec = n1.cross(n2);
	if (vec.norm() < 1e-5) return false;
	vec.normalize();
	// get node
	node.setZero();
	Eigen::Vector3d temp_node = Eigen::Vector3d::Zero();
	double x, y, z = 0.0;
	double a1 = n1(0); double b1 = n1(1); double c1 = n1(2);
	double a2 = n2(0); double b2 = n2(1); double c2 = n2(2);
	if (vec(0) != 0) { // given x = 0 for two plane equation
		x = 0;
		z = -(d1 / b1 - d2 / b2) / (c1 / b1 - c2 / b2);
		y = -c1 / b1 * z - d1 / b1;
	}
	temp_node << x, y, z;
	node << x, y, z;

	if (vec(1) != 0) { // given y = 0 for two plane equation
		y = 0;
		z = (a1 * d2 - a2 * d1) / (a1 * c2 - a2 * c1);
		x = -c1 / a1 * z - d1 / a1;
	}
	temp_node << x, y, z;
	if (node.norm() > temp_node.norm())		node << x, y, z;

	if (vec(2) != 0) { // given z = 0 for two plane equation
		z = 0;
		y = (a2 * d1 - a1 * d2) / (a1 * b2 - a2 * b1);
		x = -b1 / a1 * y - d1 / a1;
	}
	if (node.norm() > temp_node.norm())		node << x, y, z;

	return true;
}

// sphere_2_line intersection
// Line: O->T; Sphere: center,R; mu: Explicit argument
// reference: http://www.ambrsoft.com/TrigoCalc/Sphere/SpherLineIntersection_.htm
bool DeformTet::_intersect3D_line_sphere
(Eigen::Vector3d O, Eigen::Vector3d T, Eigen::Vector3d Center, double R, double& mu1, double& mu2) {

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

void DeformTet::_giveBoundary_OverhangTet_stressInfo() {

	for (GLKPOSITION pos = tetPatch->GetTetraList().GetHeadPosition(); pos != nullptr;) {
		QMeshTetra* tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(pos);

		if (tetra->isOverhangTet){

			int neighbor_StressEle_num = 0;
			Eigen::Vector3d neighbor_StressEle_tau_max = Eigen::Vector3d::Zero();
			for (int i = 0; i < tetra->neighborCell.size(); i++) {
				if (tetra->neighborCell[i]->isTensileorCompressSelect){
					neighbor_StressEle_num++;
					neighbor_StressEle_tau_max += tetra->neighborCell[i]->tau_max;
				}
			}

			if (neighbor_StressEle_num > 0) {
				tetra->isTensileorCompressSelect = true;
				tetra->tau_max = neighbor_StressEle_tau_max.normalized();
				std::cout << "the neighbor_StressEle_num is " << neighbor_StressEle_num <<
					" " << "the tetra index is " << tetra->GetIndexNo() << std::endl;
			}
		}
	}
}

// project vec_u to Plane (normalized vector)
Eigen::Vector3d DeformTet::_projection_vector2Plane(
	Eigen::Vector3d vec_u, Eigen::Vector3d planeNormal_n) {

	// planeNormal_n is a normalized one
	Eigen::Vector3d proj_vec = vec_u - vec_u.dot(planeNormal_n) * planeNormal_n;
	proj_vec.normalize();
	return proj_vec;
}

// angle is deg angle
Eigen::Vector3d DeformTet::_rotate_vector_deg(Eigen::Vector3d pnt,
	Eigen::Vector3d rotate_axis, double angle) {

	double rotation_angle = angle * PI / 180.0;
	rotate_axis.normalize();
	Eigen::AngleAxis<double> R = Eigen::AngleAxis<double>(rotation_angle, rotate_axis);
	Eigen::Vector3d new_pnt = R.matrix()* pnt;
	return new_pnt;
}

//
void DeformTet::delete_selected_ele_stress_line(){

	for (GLKPOSITION pos = tetPatch->GetTetraList().GetHeadPosition(); pos != nullptr;) {
		QMeshTetra* tetra = (QMeshTetra*)tetPatch->GetTetraList().GetNext(pos);

		int selectedNUM = 0;
		for (int i = 0; i < 4; i++) {
			if (tetra->GetNodeRecordPtr(i + 1)->isHandle)
				selectedNUM++;
		}

		if (selectedNUM == 4) {
			tetra->isTensileorCompressSelect = false;
			//std::cout << "delete 1 selected_ele stress_line. " << std::endl;
		}

	}

	for (GLKPOSITION Pos = tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(Pos);

		node->isHandle = false;
	}
}