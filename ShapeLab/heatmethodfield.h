/*Heat method field value computing*/
#ifndef HEATMETHODFIELD_H
#define HEATMETHODFIELD_H

#include "QMeshPatch.h"
#include "PolygenMesh.h"
#include "QMeshNode.h"
#include "QMeshEdge.h"
#include "QMeshFace.h"
#include "../GLKLib/GLKObList.h"
#include <../ThirdPartyDependence/eigen3/Eigen/Sparse> 

class heatMethodField
{
public:
    heatMethodField(QMeshPatch* inputMesh);
    ~heatMethodField();

    void compBoundaryHeatKernel();
    void compZigZagHeatKernel();
    void compZigZagFieldValue();
    //void inputStressFieldValue();

    void runHeatMethod();
    void planeCutSelection(double direction[]);
    void meshRefinement();

private:
    QMeshPatch* surfaceMesh;
    int genus;

    Eigen::SparseMatrix<double> L;
    Eigen::SparseMatrix<double> A;
    double t;
    Eigen::SparseMatrix<double> F;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> heatSolver;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> poissonSolver;

    void _initBoundaryHeatKernel();
    void _initZigZagHeatKernel();

    //int detectGenus();

    void initialMeshIndex();
    void heatMethodPreProcess();
    void heatMethodCompwithConstrain( Eigen::VectorXd &u);

    void buildLaplacian(Eigen::SparseMatrix<double> &L,bool constrain) const;
    void buildAreaMatrix(Eigen::SparseMatrix<double> &A) const;
    void computeVectorField(Eigen::MatrixXd &gradients, const Eigen::VectorXd &u) const;
    void computeIntegratedDivergence(Eigen::VectorXd &integratedDivs,
        const Eigen::MatrixXd &gradients) const;
    double subtractMinimumDistance(Eigen::VectorXd &phi) const;
    double computeEdgeAngle(QMeshNode* Node, QMeshEdge* connectEdge) const;
    double meanEdgeLength() const;

    bool scalarFieldCompute_zigzag();
};

#endif // HEATMETHODFIELD_H
