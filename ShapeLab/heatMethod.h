#pragma once

#include "../QMeshLib/PolygenMesh.h"
#include<Eigen/Eigen>
#include<Eigen/PardisoSupport>
#include<vector>

/*
USAGE:
	heatMethod Heat = new heatMethod(TetMeshPatch);
		---where TetMeshPatch is of type QMeshPatch* and contains the Tet Mesh
	Heat.generateHeatField(BoundaryCOndition);
		---where BoundaryCondition is of type int and in range [0,100];
			0 means and Dirichlet Sol. and 100 means a Neumann Sol.
		--- Note that a Dirichelet Sol. also requires the defination of Sink which
			can be added by modifiying the function 'heatMethod::_defineBouindaryNodes()'


	The following to be added to QMeshNode definition:
		bool isSource= false;
		bool isBoundary=false;
		double HeatFieldValue= -1.0;
		int FieldIndexNumber = -999;
*/


class heatMethod {
private:
	QMeshPatch* materialSpace;

	int constNodeNumber = 0;
	std::vector<bool> constIndex;
	Eigen::SparseMatrix<double> WeightMatrix;
	Eigen::SparseMatrix<double> LaplacianMatrix;
	Eigen::SparseMatrix<double> VolumeMatrix;
	Eigen::SparseMatrix<double> DensityMatrix;
	Eigen::MatrixXd VectorFieldMatrix;

	int Num_tetInConvexHull = 0;
	int Num_nodesInConvexHull = 0;
	int Num_nodesOnConvexHullBoundary = 0;

	double m_bottomHeight = 1.0;

private:
	void _initialiseMesh();
	void _createWeightMatrix();
	double _getEdgeWeight(QMeshEdge* edge, QMeshTetra* tet);
	void _solveHeatField(int BoundaryCondition);
	double _getAverageEdgeLength();
	void _initialiseSystemValues2(Eigen::VectorXd& u, double SourceVal, double otherVal);
	void _inverseVolumeMatrix();
	void _generateVectorField();
	void _generateScalarField();
	void _generateDivergence(Eigen::VectorXd& divVector);
	Eigen::Vector3d _getOppositeNormalWithMag(QMeshNode* node, QMeshTetra* tet);
	int _defineBouindaryNodes();

public:
	void generateHeatField(int BoundCon);
	heatMethod(QMeshPatch* MatSp, double bottomHeight);
};
