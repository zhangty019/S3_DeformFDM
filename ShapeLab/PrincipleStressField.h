#pragma once
#include "..\QMeshLib\PolygenMesh.h"

class PrincipleStressField
{
public:
	PrincipleStressField(QMeshPatch *mesh);
	~PrincipleStressField();

	QMeshPatch* tetMesh;

	bool InputFEMResult(std::string filename);
	void ComputeElementPrincipleStress();
	void DetermineCriticalTensileandCompressRegion(double rangeT, double rangeC);
	void DetectSmallCriticalRegionandClear();

private:

	int compressEleNum, tensileEleNum;

	void _selectTensileandCompressiveRegion(double rangeT, double rangeC);
	void _detectNeighbor_criticalTet(std::vector< QMeshTetra* >& TetraSet, QMeshTetra* Tetra, int ele_type);
};
