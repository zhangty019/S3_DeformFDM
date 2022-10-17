#include "PrincipleStressField.h"
#include <fstream>

#define TENSILE 0
#define COMPRESS 1

PrincipleStressField::PrincipleStressField(QMeshPatch *mesh) { tetMesh = mesh; }

PrincipleStressField::~PrincipleStressField() { }

bool PrincipleStressField::InputFEMResult(std::string filename)
{
	const char * c = filename.c_str();

	char *cstr = new char[filename.length() + 1];
	strcpy(cstr, filename.c_str());

	const char * split = ".";
	char* p = strtok(cstr, split);

	char input_filename[256];
	strcpy(input_filename, "..\\DataSet\\fem_result\\");
	strcat(input_filename, cstr);
	char filetype[64];
	strcpy(filetype, ".txt");
	strcat(input_filename, filetype);

	std::ifstream stressInfor(input_filename);
	if (!stressInfor) {
		std::cerr << "Sorry! We were unable to open the file, please check whether the FEM file exists.\n";
		return false;
	}

	Eigen::VectorXi eleIndex = Eigen::VectorXi::Zero(tetMesh->GetTetraNumber());
	Eigen::MatrixXd stressMatrix = Eigen::MatrixXd::Zero(tetMesh->GetTetraNumber(), 7);

	//string line;
	int lineIndex = 0;
	std::string sss;
	while (getline(stressInfor, sss))
	{
		const char * c = sss.c_str();
		sscanf(c, "%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
			&eleIndex(lineIndex), &stressMatrix(lineIndex, 0), &stressMatrix(lineIndex, 1), &stressMatrix(lineIndex, 2)
			, &stressMatrix(lineIndex, 3), &stressMatrix(lineIndex, 4), &stressMatrix(lineIndex, 5), &stressMatrix(lineIndex, 6));
		lineIndex++;
	}

	stressInfor.close();

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *element = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		element->eleStress = Eigen::VectorXd::Zero(7);
		for (int i = 0; i < 7; i++) element->eleStress(i) = stressMatrix(element->GetIndexNo(), i);
		//cout << element->GetIndexNo() << " = " << element->eleStress << endl;
	}

	tetMesh->drawStressField = true;

	double max = 0, min = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *element = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		double thisStress = element->eleStress[0];
		if (thisStress > max) max = thisStress;
		if (thisStress < min) min = thisStress;
	}
	tetMesh->minStressValue = min;
	tetMesh->maxStressValue = max;

	printf(" Principle Stress Field -- Finish input FEM result!\n\n");
	return true;
}

void PrincipleStressField::ComputeElementPrincipleStress() {

	//---- Compute the principle stress from the Tensor by SVD
	Eigen::Matrix3d stressTensor;

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		stressTensor <<
			Tet->eleStress(1), Tet->eleStress(4), Tet->eleStress(5),
			Tet->eleStress(4), Tet->eleStress(2), Tet->eleStress(6),
			Tet->eleStress(5), Tet->eleStress(6), Tet->eleStress(3);

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(stressTensor, Eigen::ComputeEigenvectors);
		Eigen::Matrix3d eigenvectorsMatrix = eigenSolver.eigenvectors();
		Eigen::Vector3d eigenvalue = eigenSolver.eigenvalues();

		std::ptrdiff_t first_index, last_index;
		//eigenvalue.maxCoeff(&index); //Max principle stress

		Eigen::Vector3d eigenvalueABS; for (int i = 0; i < 3; i++) eigenvalueABS(i) = abs(eigenvalue(i));

		//(eigenvalue.cwiseAbs()).maxCoeff(&index); //Max principle stress (ABS)

		eigenvalueABS.maxCoeff(&first_index); //Max principle stress (ABS)
		eigenvalueABS.minCoeff(&last_index); //Third principle stress (ABS)

		Tet->tau_max = eigenvectorsMatrix.col(first_index); //vector
		Tet->tau_min = eigenvectorsMatrix.col(last_index);

		Tet->sigma_max = eigenvalue(first_index); // double
		Tet->sigma_min = eigenvalue(last_index);

		for (int i = 0; i < 3; i++) {
			if (i != first_index && i != last_index) {
				Tet->tau_mid = eigenvectorsMatrix.col(i);
				Tet->sigma_mid = eigenvalue(i);
			}
		}
	}

	//---- Compute min and max principle stress value
	double max = 0, min = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* element = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		double thisStress = element->sigma_max;
		if (thisStress > max) max = thisStress;// +max
		if (thisStress < min) min = thisStress;// -max
	}
	tetMesh->minPrincipleStressValue = min;
	tetMesh->maxPrincipleStressValue = max;
}

void PrincipleStressField::DetermineCriticalTensileandCompressRegion(double rangeT, double rangeC) {
	
	tensileEleNum = 0;
	compressEleNum = 0;

	//--------------------------------------------------------------------
	//Step 1: initialize the index and flag, detect tensile and compress element number

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Tet->stressIndex = -1;
		if (Tet->sigma_max >= 0.0) this->tensileEleNum++;
		else this->compressEleNum++;
	}

	Eigen::VectorXd tensileValue(tensileEleNum);
	Eigen::VectorXd compressValue(compressEleNum);

	int cEleNum = 0, tEleNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra *Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tet->sigma_max >= 0.0) {
			tensileValue(tEleNum) = Tet->sigma_max; tEleNum++;
		}
		else {
			compressValue(cEleNum) = fabs(Tet->sigma_max); cEleNum++;
		}
	}

	//--------------------------------------------------------------------
	//Step 2: sort the element by tensile or compress stress value

	Eigen::VectorXi tind, cind;
	tind = Eigen::VectorXi::LinSpaced(tEleNum, 0, tEleNum - 1); //[0 1 2 3 ... N-1]
	cind = Eigen::VectorXi::LinSpaced(cEleNum, 0, cEleNum - 1); //[0 1 2 3 ... N-1]

	auto ruleT = [tensileValue](int i, int j)->bool {
		return tensileValue(i)>tensileValue(j); //sorting rules
	};

	std::sort(tind.data(), tind.data() + tind.size(), ruleT);

auto ruleC = [compressValue](int i, int j)->bool {
	return compressValue(i) > compressValue(j); //sorting rules
};

std::sort(cind.data(), cind.data() + cind.size(), ruleC);

tEleNum = 0; cEleNum = 0;
for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
	QMeshTetra* Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
	if (Tet->sigma_max >= 0.0) {
		for (int i = 0; i < tensileEleNum; i++) {
			if (tEleNum == tind(i)) {
				Tet->stressIndex = i; break;
			}
		}
		tEleNum++;
	}
	else {
		for (int i = 0; i < compressEleNum; i++) {
			if (cEleNum == cind(i)) {
				Tet->stressIndex = i; break;
			}
		}
		cEleNum++;
	}
}

//--------------------------------------------------------------------
//Step 3: select compute region by ratio
this->_selectTensileandCompressiveRegion(rangeT, rangeC);
}

void PrincipleStressField::_selectTensileandCompressiveRegion(double rangeT, double rangeC) {

	int initialGuessRegionNum = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Tet->isTensileorCompressSelect = false;

		int Tnumber = floor(tensileEleNum * rangeT);
		int Cnumber = floor(compressEleNum * rangeC);

		if (Tet->sigma_max >= 0.0) {
			if (Tet->stressIndex < Tnumber)
				Tet->isTensileorCompressSelect = true;
		}
		else {
			if (Tet->stressIndex < Cnumber)
				Tet->isTensileorCompressSelect = true;
		}
	}
}

void PrincipleStressField::DetectSmallCriticalRegionandClear() {

	// ---- Delete all the tetra element (both tensile and compress region) that not have enough neighbor ---- //
	int smallRegionSize = 50;

	int stressEleNum = 0, removeEleNum_compress = 0;
	int tensileEleNum = 0, removeEleNum_tensile = 0;

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Tetra->lessNeighborFlag = false;

		if (Tetra->isTensileorCompressSelect && Tetra->sigma_max < 0) {
			std::vector< QMeshTetra* > TetraSet; stressEleNum++;
			this->_detectNeighbor_criticalTet(TetraSet, Tetra, COMPRESS);
			if (TetraSet.size() < smallRegionSize) Tetra->lessNeighborFlag = true;
		}

		if (Tetra->isTensileorCompressSelect && Tetra->sigma_max >= 0) {
			std::vector< QMeshTetra* > TetraSet; tensileEleNum++;
			this->_detectNeighbor_criticalTet(TetraSet, Tetra, TENSILE);
			if (TetraSet.size() < smallRegionSize) Tetra->lessNeighborFlag = true;
		}
	}

	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->lessNeighborFlag && Tetra->sigma_max < 0) { Tetra->isTensileorCompressSelect = false; removeEleNum_compress++; }
		if (Tetra->lessNeighborFlag && Tetra->sigma_max > 0) { Tetra->isTensileorCompressSelect = false; removeEleNum_tensile++; }
	}

	// special case // delete the stress lower than ?mm
	//if (tetMesh->patchName == "armadillo4" && false) {// for only selected stress field output
	if (tetMesh->patchName == "armadillo4") {
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

			//bounary tet
			bool outerTet = false;
			for (int i = 0; i < 4; i++) {
				if (!Tetra->GetFaceRecordPtr(i + 1)->inner) {
					outerTet = true;
					break;
				}
			}

			double xx, yy, zz;
			Tetra->CalCenterPos(xx, yy, zz);
			if (Tetra->sigma_max < 0){
				if ((yy > 30 && yy < 100) 
					|| (yy < 15)
					|| (yy < 30 && yy > 15 && (outerTet == false))
					) {

					Tetra->isTensileorCompressSelect = false; removeEleNum_compress++;
				}
			}
			if (Tetra->sigma_max > 0) {
				if ((yy > 30 && yy < 100) 
					|| (yy < 15) 
					|| (yy < 30 && yy >15 && outerTet == false)
					) {

					Tetra->isTensileorCompressSelect = false; removeEleNum_tensile++;
				}
			}
		}
	}

	if (tetMesh->patchName == "airbus_topopt" || tetMesh->patchName == "airbus_topopt1") {
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

			double xx, yy, zz;
			Tetra->CalCenterPos(xx, yy, zz);
			if (Tetra->sigma_max < 0) {
				if (yy < 20.0) {
					Tetra->isTensileorCompressSelect = false; removeEleNum_compress++;
				}
			}
			if (Tetra->sigma_max > 0) {
				if (yy < 20.0){
					Tetra->isTensileorCompressSelect = false; removeEleNum_tensile++;
				}
			}
		}
	}

	std::cout << "For COMPRESS region, move " << removeEleNum_compress << " elements out of " << stressEleNum << std::endl;
	std::cout << "For TENSILE region, move " << removeEleNum_tensile << " elements out of " << tensileEleNum << std::endl;
}

void PrincipleStressField::_detectNeighbor_criticalTet(
	std::vector< QMeshTetra* >& TetraSet, QMeshTetra* Tetra, int ele_type) {

	//using node to find neighbor tetrahedral 
	for (int i = 0; i < 4; i++) {
		QMeshNode* thisNode = Tetra->GetNodeRecordPtr(i + 1);
		for (GLKPOSITION Pos = thisNode->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* ConnectTetra = (QMeshTetra*)thisNode->GetTetraList().GetNext(Pos);

			if (ConnectTetra == Tetra) continue;
			if (!ConnectTetra->isTensileorCompressSelect) continue;

			if (ConnectTetra->sigma_max >= 0 && ele_type == COMPRESS) continue;
			if (ConnectTetra->sigma_max < 0 && ele_type == TENSILE) continue;

			//decide whether the connected tetra has been recorded before
			bool exist_in_set = false;
			for (int j = 0; j < TetraSet.size(); j++) {
				if (ConnectTetra->GetIndexNo() == TetraSet[j]->GetIndexNo()) {
					exist_in_set = true; break;
				}
			}
			if (exist_in_set) continue;	
			TetraSet.push_back(ConnectTetra);
		}
	}
}
