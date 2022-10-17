#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSignalMapper>
#include <QStandardItemModel>
#include "../GLKLib/GLKLib.h"
#include "../QMeshLib/PolygenMesh.h"
#include <omp.h>
#include <QTimer>
#include <QLabel>

#include "Fabrication.h"
#include "SupportGeneration.h"
#include "GcodeGeneration.h"

#define PI		3.141592654
#define DEGREE_TO_ROTATE(x)		0.0174532922222*x
#define ROTATE_TO_DEGREE(x)		57.295780490443*x

typedef enum S_type {
    //for SIGGRAPH paper
    NONE,
    SUPPORT_LESS,
    STRENGTH_REINFORCEMENT,
    SURFACE_QUALITY,
    HYBRID_SL_SQ,
    HYBRID_SR_SQ,
    HYBRID_SL_SR,
    HYBRID_SL_SR_SQ
};

using namespace std;

class DeformTet;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
	// Qtimer - defined function
    void doTimerGcodeMoving();

private:
    Ui::MainWindow *ui;
    GLKLib *pGLK;

    /* add for Gcode generation */
    QTimer Gcode_timer; //Gcode Simulation timer
    int gcodetimerItertime;
    int simuLayerInd;
    Eigen::MatrixXf Gcode_Table;
    //unsigned int operationTime = 0;
    /* ------------------------ */
	GLKObList polygenMeshList;

private:
    void createActions();
    void createTreeView();
	void showTetraDeformationRatio();
	void MoveHandleRegion();
	void QTgetscreenshoot();

    PolygenMesh *getSelectedPolygenMesh();

    QSignalMapper *signalMapper;
    QStandardItemModel *treeModel;

    //DeformTet* Deformation;
    //FileIO* IO_operator;
    Fabrication* fabriOperator;
    SupportGeneration* supportGene;
    GcodeGeneration* GcodeGene;

// functions for S^3 DeformFDM
private:
    PolygenMesh* _buildPolygenMesh(mesh_type type, std::string name);
    PolygenMesh* _detectPolygenMesh(mesh_type type);
    QMeshPatch* MainWindow::_detectPolygenMesh(mesh_type type, std::string patch_name); // detect certain patch by patchName
    void _updateFrameworkParameter();
    /*void _setParameter(int loopTime, S_type caseType, double criticalTet_weight,
        double neighborScale_weight, double regularScale_weight, double globalSmooth_weight,
        double supportFreeAngle, double tensileRegionRatio, double compressRegionRatio);*/
    void _setParameter(int outer_loopTime, int inner_loopTime, S_type caseType,
        double criticalTet_weight_SL, double criticalTet_weight_SR,
        double criticalTet_weight_SQC, double criticalTet_weight_SQV,
        double neighborScale_weight, double regularScale_weight, double globalSmooth_weight,
        double supportFreeAngle, double tensileRegionRatio, double compressRegionRatio);
    
    void _input_CNC_part();
    void _scalarField_2_vectorField(bool is_normalize);
    void _scalarField_2_vectorField_unNormalized(QMeshPatch* patch);
    void _vectorField_smooth(int smoothLoop, bool is_normalize);
    void _vectorField_2_scalarField(bool Up_Vector_Direction);
    void _vectorField_2_scalarField_withHardConstrain(QMeshPatch* tetMesh);
    void _compVectorField_smooth_withHardConstrain(QMeshPatch* tetMesh);

    int _mark_SurfaceKeep_region(QMeshPatch* patch);
    void _surfaceKeepRegion_flooding(QMeshPatch* patch, int index);
    void _fixScalarField_surfaceKeepRegion(QMeshPatch* patch, int splitTime);
    void _compScalarField_normalizedValue(QMeshPatch* tetMesh);
    void _detect_Neighbor_Tetrahedral(std::vector< QMeshTetra* >& TetraSet,
        QMeshTetra* Tetra, bool neighb_type);

    void _setParameter(double rot_phi, double rot_theta, double modelHeight_y);
    void _compTetMeshVolumeMatrix(QMeshPatch* patch);

    void _updateFabractionParameter();
    void _setParameter(bool Yup2Zup, 
        double Xmove, double Ymove, double Zmove);

    Eigen::Vector3d _calPartGuesture(bool table_OR_head, Eigen::Vector3d printPos,
        double X, double Y, double Z, double B, double C);

    S_type S3_case;
    bool newConfig_CNC = false;

protected:
    void dragEnterEvent(QDragEnterEvent *event);
    void dropEvent(QDropEvent *event);

private slots:
    void open();
    void save();
	void saveSelection();
	bool readSelection();

    void signalNavigation(int flag);
    void shiftToOrigin();
    void updateTree();
	void mouseMoveEvent(QMouseEvent *event);
    void on_pushButton_clearAll_clicked();
    void on_treeView_clicked(const QModelIndex &index);

	/*This is S^3 Deformmation*/
    void runBest_Orientation_calculation();
    void runDeformation_supportLess_ASAP();
    //void runDeformation_supportLess_ASAP_test();
    void runDeformation_strengthReinforcement_ASAP();
    void runDeformation_surfaceQuality_ASAP();
    void runDeformation_SL_SQ_ASAP();
    void runDeformation_SR_SQ_ASAP();
    void runDeformation_SL_SR_ASAP();
    void runDeformation_SL_SR_SQ_ASAP();

    void inverseDeformation();
    void curvedLayer_Generation();
    void adaptiveHeight_curvedLayer_Generation();

    /*This is forIO operation*/
    void run_tet2surface();
    void output_IsoLayer_set();
    void output_Toolpath_set();
    void output_vectorVSobjective();
    void output_userWaypoints();
    void output_stress_field();
    void output_Qvector_field();
    void output_ScalarOrHeight_field();
    void output_deformedTet();
    void output_preProcess_waypoints_4_Robot();
    void output_discreteTet();
    void output_discreteTet_obj();
    void output_materialRot_4CAE();

    /*This is for Display*/
    void change_isoLayer_Display();
    void changeWaypointDisplay();
    void all_isoLayer_Display();
    void show_ScaleValue();
    void split_Show_TET_Mesh();
    void change_maxLayerNum_normalORcompatible();
    void special_Draw_Command();

    /*This is for Fabrication*/
    void model_ROT_MOVE();
    void concave_Detect_vector();
    void concave_Smooth_vector();
    void concave_Detect_Smooth_vector();
    void collisionChecking_layer();
    void reMark_collisionVector();
    void collisionAware_Flattening();

    void initial_Guess_SupportEnvelope_Generation();
    void initial_Guess_SupportEnvelope_Generation_plus();

    void compatibleLayer_Generation();
    void mark_OverhangFace();
    void reMark_OverhangFace();
    void build_Support_Tree_Skeleton();
    void generate_slim_supportLayer();
    void toolPath_Generation();
    void toolPath_hybrid_Generation();

    /*This is for G-code Generation*/
    void readGcodeSourceData();
    void runDHWcalculation();
    void runSingularityOpt();
    void runCollisionCheck();
    void runCollisionElimination();
    void runWriteGcode();
    void runGcodeSimulation();

    /*This is for Distance Calculation between scanning and model.*/
    void cal_Dist_scanAndmodel();
};

#endif // MAINWINDOW_H
