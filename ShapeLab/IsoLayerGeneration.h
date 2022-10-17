#pragma once
class PolygenMesh;
class QMeshPatch;
#include "../QMeshLib/PolygenMesh.h"
#include "../ThirdPartyDependence/PQPLib/PQP.h"

class IsoLayerGeneration {
public:
    IsoLayerGeneration(QMeshPatch* Patch) { m_tetMesh = Patch; }
    ~IsoLayerGeneration() {};

    void generateIsoSurface(PolygenMesh* isoSurface, int layerNum);
    void smoothingIsoSurface(PolygenMesh* isoSurface);
    bool planeCutSurfaceMesh_delete(QMeshPatch* surfaceMesh);
    void outputSurfaceMesh(PolygenMesh* isoSurface, bool isRun);
    void output_split_SurfaceMesh(PolygenMesh* isoSurface, bool isRun);

    void generateIsoSurface_support(
        PolygenMesh* isoSurface, QMeshPatch* patch_supportTet, int layerNum);
    void generateIsoSurface_adaptiveDistanceControl(
        PolygenMesh* isoSurface, double minLayerHight, double maxLayerHight);

private:
    QMeshPatch* _generatesingleIsoSurface(double isoValue, PolygenMesh* isoSurface);
    QMeshPatch* _generatesingleIsoSurface_support
    (double isoValue, PolygenMesh* isoSurface);
    int _remove_allFile_in_Dir(std::string dirPath);
    void _output_OneSurfaceMesh(QMeshPatch* isoSurface, std::string path);
    bool _splitSingleSurfacePatch_detect(QMeshPatch* each_layer);


    void _insertLayerToSurfaceSet_smooth(QMeshPatch* layer, int smmothItertime,
        PolygenMesh* isoSurface, QMeshPatch* blayer, bool insert);
    void _smooth_one_IsoSurface(QMeshPatch* isoLayer);
    double _detectNextLayerISOValue(QMeshPatch* bLayer, double minLayerHight);
    void _generateISOSurfaceNode(double isoValue, Eigen::MatrixXd& nodePos);
    void _checkLayerDistance(QMeshPatch* bLayer, Eigen::MatrixXd& nodePos,
        Eigen::VectorXd& layerDis, PQP_Model* pqpModel);
    bool _checktwoLayerThickness_withinRange(double maxLayerHight, double minLayerHight,
        QMeshPatch* bLayer, QMeshPatch* tLayer, bool minControl);
    void _checkNodeSet_withinBlayerRegion(QMeshPatch* bLayer, Eigen::MatrixXd& nodePosSet,
        double maxLayerHight, Eigen::VectorXi& deleteNodeIndexSet);
    void _buildOffsetMeshTable(QMeshPatch* bLayer, Eigen::MatrixXd& nodeTable,
        Eigen::MatrixXi& faceTable, double boundaryEdgeNum, double maxLayerHight);
    bool _checkNodeInsideOffsetSpace(Eigen::MatrixXd& nodeTable,
        Eigen::MatrixXi& faceTable, Eigen::Vector3d& checkNodePos);
    bool IntersectTriangle(const Eigen::Vector3d& orig, const Eigen::Vector3d& dir,
        Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2);
    void _cutLayer_withOffsetLayer(QMeshPatch* bLayer, 
        QMeshPatch* layer, double offsetValue);
    void _triming_workingSurface(QMeshPatch* isoLayer);

    QMeshPatch* m_tetMesh = NULL;
    QMeshPatch* m_tetMesh_support = NULL;
};