#pragma once
#include "..\QMeshLib\PolygenMesh.h"

class PrincipleStressField
{
public:
	PrincipleStressField(QMeshPatch *mesh);
	~PrincipleStressField();

	QMeshPatch* tetMesh;

	bool InputFEMResult(std::string filename);
	bool InputFEMResult_stressLine(std::string filename);
	bool PrincipalStressRead(std::string filename);
	void ComputeElementPrincipleStress();
	void PrincipalStressAnalysis(bool ABSResult, bool isTensileCase);
	void DetermineCriticalTensileandCompressRegion(double rangeT, double rangeC);
	void SelectTensileandCompressiveElements_stressLine();
	void DetectSmallCriticalRegionandClear();

private:

	int compressEleNum, tensileEleNum;

	void _selectTensileandCompressiveRegion(double rangeT, double rangeC);
	void _detectNeighbor_criticalTet(std::vector< QMeshTetra* >& TetraSet, QMeshTetra* Tetra, int ele_type);
};
