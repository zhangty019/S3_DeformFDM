/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QTreeView>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionOpen;
    QAction *actionFront;
    QAction *actionBack;
    QAction *actionTop;
    QAction *actionBottom;
    QAction *actionLeft;
    QAction *actionRight;
    QAction *actionIsometric;
    QAction *actionZoom_In;
    QAction *actionZoom_Out;
    QAction *actionZoom_All;
    QAction *actionZoom_Window;
    QAction *actionShade;
    QAction *actionMesh;
    QAction *actionNode;
    QAction *actionSave;
    QAction *actionSelectNode;
    QAction *actionSelectFace;
    QAction *actionShifttoOrigin;
    QAction *actionProfile;
    QAction *actionFaceNormal;
    QAction *actionNodeNormal;
    QAction *actionSelectEdge;
    QAction *actionGenerate;
    QAction *actionTest_1;
    QAction *actionSelectFix;
    QAction *actionSelectHandle;
    QAction *actionSaveSelection;
    QAction *actionReadSelection;
    QAction *actionSelectChamber;
    QAction *actionExport_to_Abaqus_model;
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    QToolBar *navigationToolBar;
    QStatusBar *statusBar;
    QToolBar *selectionToolBar;
    QDockWidget *dockWidget;
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout;
    QLabel *label_S3_DeformFDM;
    QFrame *line;
    QTabWidget *tabWidget;
    QWidget *tab;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_12;
    QLabel *label;
    QCheckBox *boxDeselect;
    QFrame *line_2;
    QHBoxLayout *horizontalLayout_31;
    QLabel *label_46;
    QLabel *label_47;
    QSpinBox *spinBox_shapeUpIterNum;
    QLabel *label_48;
    QSpinBox *spinBox_innerIterNum;
    QHBoxLayout *horizontalLayout_32;
    QLabel *label_49;
    QLabel *label_50;
    QDoubleSpinBox *doubleSpinBox_criticalTETWeight_SL;
    QLabel *label_51;
    QDoubleSpinBox *doubleSpinBox_criticalTETWeight_SR;
    QHBoxLayout *horizontalLayout_33;
    QLabel *label_52;
    QLabel *label_2;
    QDoubleSpinBox *doubleSpinBox_criticalTETWeight_SQC;
    QLabel *label_53;
    QDoubleSpinBox *doubleSpinBox_criticalTETWeight_SQV;
    QHBoxLayout *horizontalLayout_34;
    QLabel *label_55;
    QDoubleSpinBox *doubleSpinBox_neighborScaleWeight;
    QLabel *label_56;
    QDoubleSpinBox *doubleSpinBox_regularScaleWeight;
    QLabel *label_57;
    QDoubleSpinBox *doubleSpinBox_globalQuatSmoothWeight;
    QFrame *line_14;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_8;
    QDoubleSpinBox *doubleSpinBox_supportFreeAngle;
    QHBoxLayout *horizontalLayout_28;
    QPushButton *pushButton_calBestOrientation;
    QCheckBox *checkBox_computeBestOrientation;
    QPushButton *pushButton_s3DeformFDM_supportLess_ASAP;
    QFrame *line_4;
    QHBoxLayout *horizontalLayout_30;
    QLabel *label_10;
    QLabel *label_44;
    QDoubleSpinBox *doubleSpinBox_tensileRegionRatio;
    QLabel *label_11;
    QDoubleSpinBox *doubleSpinBox_compressRegionRatio;
    QPushButton *pushButton_s3DeformFDM_strengthReinforcement_ASAP;
    QPushButton *pushButton_s3DeformFDM_surfaceQuality_ASAP;
    QFrame *line_6;
    QLabel *label_4;
    QHBoxLayout *horizontalLayout_36;
    QLabel *label_6;
    QDoubleSpinBox *doubleSpinBox_keepWeight_SL;
    QLabel *label_58;
    QDoubleSpinBox *doubleSpinBox_keepWeight_SR;
    QHBoxLayout *horizontalLayout_35;
    QLabel *label_5;
    QDoubleSpinBox *doubleSpinBox_keepWeight_SQC;
    QLabel *label_54;
    QDoubleSpinBox *doubleSpinBox_keepWeight_SQV;
    QHBoxLayout *horizontalLayout_37;
    QPushButton *pushButton_s3DeformFDM_hybrid_SL_SQ_ASAP;
    QPushButton *pushButton_s3DeformFDM_hybrid_SR_SQ_ASAP;
    QPushButton *pushButton_s3DeformFDM_hybrid_SL_SR_ASAP;
    QPushButton *pushButton_s3DeformFDM_hybrid_SL_SR_SQ_ASAP;
    QPushButton *pushButton_s3DeformFDM_inverseDeform;
    QSpacerItem *verticalSpacer_2;
    QWidget *tab_4;
    QVBoxLayout *verticalLayout_5;
    QLabel *label_7;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_14;
    QSlider *horizontalSlider_slice_Multi_dir;
    QComboBox *comboBox_planeDir;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *pushButton_drawScaleValue;
    QComboBox *comboBox_scaleValueDirection;
    QLabel *label_9;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_12;
    QSpinBox *spinBox_ShowLayerIndex;
    QSpacerItem *horizontalSpacer_6;
    QCheckBox *checkBox_EachLayerSwitch;
    QHBoxLayout *horizontalLayout_8;
    QPushButton *pushButton_ShowAllLayers;
    QRadioButton *radioButton_compatibleLayer;
    QFrame *line_3;
    QLabel *label_3;
    QHBoxLayout *horizontalLayout_23;
    QLabel *label_36;
    QLabel *label_39;
    QDoubleSpinBox *doubleSpinBox_rot_phi;
    QLabel *label_40;
    QDoubleSpinBox *doubleSpinBox_rot_theta;
    QHBoxLayout *horizontalLayout_24;
    QLabel *label_41;
    QSpacerItem *horizontalSpacer_5;
    QLabel *label_43;
    QDoubleSpinBox *doubleSpinBox_modelHeight_y;
    QPushButton *pushButton_model_move_rotation;
    QFrame *line_13;
    QLabel *label_45;
    QPushButton *pushButton_tet2surface;
    QPushButton *pushButton_output_vectorVSobjective;
    QPushButton *pushButton_output_userWaypoints;
    QPushButton *pushButton_output_streeField;
    QPushButton *pushButton_output2Robot;
    QPushButton *pushButton_output_QvectorField;
    QHBoxLayout *horizontalLayout_38;
    QPushButton *pushButton_output_ScalarOrHeight_Field;
    QCheckBox *checkBox_scalarOrHeight_field;
    QPushButton *pushButton_output_deformedTet;
    QPushButton *pushButton_output_discretedTet;
    QPushButton *pushButton_output_discretedTet_obj;
    QPushButton *pushButton_output_materialRot_4CAE;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_13;
    QSpinBox *spinBox_isoLayerNumber;
    QPushButton *pushButton_outputIsoLayerSet;
    QPushButton *pushButton_isoLayerGeneration;
    QPushButton *pushButton_adaptiveHeightSlicing;
    QFrame *line_12;
    QLabel *label_38;
    QHBoxLayout *horizontalLayout_27;
    QLabel *label_37;
    QPushButton *pushButton_collisionChecking_vector;
    QPushButton *pushButton_concaveSmooth;
    QPushButton *pushButton_concaveRelaxLoop;
    QSpinBox *spinBox_concaveDetecSmooth_loopTime;
    QPushButton *pushButton_collisionChecking;
    QHBoxLayout *horizontalLayout_25;
    QPushButton *pushButton_remarkCollisionTetra;
    QPushButton *pushButton_collisionAware_flattening;
    QFrame *line_11;
    QFrame *line_5;
    QLabel *label_15;
    QLabel *label_18;
    QFormLayout *formLayout_4;
    QPushButton *pushButton_Comp_initialGuess_envelopSupport;
    QLabel *label_19;
    QPushButton *pushButton_MeshCombination;
    QLabel *label_20;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *pushButton_compatibleLayer_generation;
    QLabel *label_16;
    QHBoxLayout *horizontalLayout_26;
    QPushButton *pushButton_markOverhangFace;
    QPushButton *pushButton_remarkOverhangFace;
    QPushButton *pushButton_treeSkeletonGeneration;
    QPushButton *pushButton_slimmedSupportGeneration;
    QFrame *line_7;
    QLabel *label_17;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_22;
    QDoubleSpinBox *doubleSpinBox_toolPathWidth;
    QLabel *label_21;
    QDoubleSpinBox *doubleSpinBox_toolPathDistance;
    QFrame *line_15;
    QPushButton *pushButton_toolPathGeneration;
    QHBoxLayout *horizontalLayout_9;
    QPushButton *pushButton_outputToolpath;
    QCheckBox *checkBox_outputCompatiblewaypoint;
    QWidget *tab_3;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_23;
    QSpacerItem *horizontalSpacer;
    QCheckBox *checkBox_Yup2Zup;
    QHBoxLayout *horizontalLayout_29;
    QLabel *label_42;
    QDoubleSpinBox *doubleSpinBox_toolLength;
    QHBoxLayout *horizontalLayout_13;
    QLabel *label_24;
    QDoubleSpinBox *doubleSpinBox_Xmove;
    QLabel *label_25;
    QDoubleSpinBox *doubleSpinBox_Ymove;
    QLabel *label_26;
    QDoubleSpinBox *doubleSpinBox_Zmove;
    QHBoxLayout *horizontalLayout_17;
    QPushButton *pushButton_readGcodeSourceData;
    QLabel *label_30;
    QLineEdit *lineEdit_SorceDataDir;
    QHBoxLayout *horizontalLayout_14;
    QLabel *label_27;
    QLabel *label_28;
    QSpinBox *spinBox_StartLayerIndex;
    QLabel *label_29;
    QSpinBox *spinBox_ShowLayerIndex_2;
    QCheckBox *checkBox_EachLayerSwitch_2;
    QHBoxLayout *horizontalLayout_15;
    QPushButton *pushButton_ShowAllLayersORToolpathes;
    QSpacerItem *horizontalSpacer_2;
    QLabel *label_currentFile;
    QFrame *line_8;
    QHBoxLayout *horizontalLayout_16;
    QLabel *label_31;
    QSpacerItem *horizontalSpacer_3;
    QLabel *label_32;
    QSpinBox *spinBox_GcodeGeneFromIndex;
    QLabel *label_33;
    QSpinBox *spinBox_GcodeGeneToIndex;
    QFrame *line_9;
    QHBoxLayout *horizontalLayout_18;
    QLabel *label_34;
    QCheckBox *checkBox_varyDistance;
    QCheckBox *checkBox_varyHeight;
    QCheckBox *checkBox_varyWidth;
    QCheckBox *checkBox_TestDHW_Switch;
    QPushButton *pushButton_calDWH;
    QHBoxLayout *horizontalLayout_19;
    QLabel *label_35;
    QDoubleSpinBox *doubleSpinBox_lambda;
    QPushButton *pushButton_calSingularOpt;
    QHBoxLayout *horizontalLayout_20;
    QCheckBox *checkBox_showSingularityNode;
    QCheckBox *checkBox_showSolveSelection;
    QPushButton *pushButton_calCollision;
    QPushButton *pushButton_calCollisionElimination;
    QPushButton *pushButton_Gcode_writting;
    QHBoxLayout *horizontalLayout_21;
    QPushButton *pushButton_GcodeSimulation;
    QSpacerItem *horizontalSpacer_4;
    QCheckBox *checkBox_showCNC;
    QHBoxLayout *horizontalLayout_22;
    QProgressBar *progressBar_GcodeSimulation;
    QCheckBox *checkBox_stopSimulation;
    QFrame *line_10;
    QPushButton *pushButton_calDist_scanAndmodel;
    QSpacerItem *verticalSpacer;
    QTreeView *treeView;
    QPushButton *pushButton_clearAll;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuView;
    QMenu *menuSelect;
    QToolBar *toolBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1300, 986);
        MainWindow->setMinimumSize(QSize(0, 0));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        MainWindow->setFont(font);
        MainWindow->setMouseTracking(true);
        MainWindow->setFocusPolicy(Qt::StrongFocus);
        MainWindow->setAcceptDrops(true);
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/resource/Open Folder.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionOpen->setIcon(icon);
        actionFront = new QAction(MainWindow);
        actionFront->setObjectName(QString::fromUtf8("actionFront"));
        actionFront->setCheckable(false);
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/resource/Front View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFront->setIcon(icon1);
        actionBack = new QAction(MainWindow);
        actionBack->setObjectName(QString::fromUtf8("actionBack"));
        actionBack->setCheckable(false);
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/resource/Back View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionBack->setIcon(icon2);
        actionTop = new QAction(MainWindow);
        actionTop->setObjectName(QString::fromUtf8("actionTop"));
        actionTop->setCheckable(false);
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/resource/Top View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionTop->setIcon(icon3);
        actionBottom = new QAction(MainWindow);
        actionBottom->setObjectName(QString::fromUtf8("actionBottom"));
        actionBottom->setCheckable(false);
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/resource/Bottom View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionBottom->setIcon(icon4);
        actionLeft = new QAction(MainWindow);
        actionLeft->setObjectName(QString::fromUtf8("actionLeft"));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/resource/Left View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionLeft->setIcon(icon5);
        actionRight = new QAction(MainWindow);
        actionRight->setObjectName(QString::fromUtf8("actionRight"));
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/resource/Right View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionRight->setIcon(icon6);
        actionIsometric = new QAction(MainWindow);
        actionIsometric->setObjectName(QString::fromUtf8("actionIsometric"));
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/resource/Isometric View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionIsometric->setIcon(icon7);
        actionZoom_In = new QAction(MainWindow);
        actionZoom_In->setObjectName(QString::fromUtf8("actionZoom_In"));
        QIcon icon8;
        icon8.addFile(QString::fromUtf8(":/resource/Zoom In.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_In->setIcon(icon8);
        actionZoom_Out = new QAction(MainWindow);
        actionZoom_Out->setObjectName(QString::fromUtf8("actionZoom_Out"));
        QIcon icon9;
        icon9.addFile(QString::fromUtf8(":/resource/Zoom Out.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_Out->setIcon(icon9);
        actionZoom_All = new QAction(MainWindow);
        actionZoom_All->setObjectName(QString::fromUtf8("actionZoom_All"));
        QIcon icon10;
        icon10.addFile(QString::fromUtf8(":/resource/Zoom All.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_All->setIcon(icon10);
        actionZoom_Window = new QAction(MainWindow);
        actionZoom_Window->setObjectName(QString::fromUtf8("actionZoom_Window"));
        QIcon icon11;
        icon11.addFile(QString::fromUtf8(":/resource/Zoom Window.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_Window->setIcon(icon11);
        actionShade = new QAction(MainWindow);
        actionShade->setObjectName(QString::fromUtf8("actionShade"));
        actionShade->setCheckable(true);
        QIcon icon12;
        icon12.addFile(QString::fromUtf8(":/resource/Shade.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionShade->setIcon(icon12);
        actionMesh = new QAction(MainWindow);
        actionMesh->setObjectName(QString::fromUtf8("actionMesh"));
        actionMesh->setCheckable(true);
        QIcon icon13;
        icon13.addFile(QString::fromUtf8(":/resource/Mesh.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionMesh->setIcon(icon13);
        actionNode = new QAction(MainWindow);
        actionNode->setObjectName(QString::fromUtf8("actionNode"));
        actionNode->setCheckable(true);
        QIcon icon14;
        icon14.addFile(QString::fromUtf8(":/resource/Node.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNode->setIcon(icon14);
        actionSave = new QAction(MainWindow);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        QIcon icon15;
        icon15.addFile(QString::fromUtf8(":/resource/Save as.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSave->setIcon(icon15);
        actionSelectNode = new QAction(MainWindow);
        actionSelectNode->setObjectName(QString::fromUtf8("actionSelectNode"));
        QIcon icon16;
        icon16.addFile(QString::fromUtf8(":/resource/selectNode.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectNode->setIcon(icon16);
        actionSelectFace = new QAction(MainWindow);
        actionSelectFace->setObjectName(QString::fromUtf8("actionSelectFace"));
        QIcon icon17;
        icon17.addFile(QString::fromUtf8(":/resource/selectFace.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectFace->setIcon(icon17);
        actionShifttoOrigin = new QAction(MainWindow);
        actionShifttoOrigin->setObjectName(QString::fromUtf8("actionShifttoOrigin"));
        actionProfile = new QAction(MainWindow);
        actionProfile->setObjectName(QString::fromUtf8("actionProfile"));
        actionProfile->setCheckable(true);
        QIcon icon18;
        icon18.addFile(QString::fromUtf8(":/resource/Profile.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionProfile->setIcon(icon18);
        actionFaceNormal = new QAction(MainWindow);
        actionFaceNormal->setObjectName(QString::fromUtf8("actionFaceNormal"));
        actionFaceNormal->setCheckable(true);
        QIcon icon19;
        icon19.addFile(QString::fromUtf8(":/resource/FaceNormal.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFaceNormal->setIcon(icon19);
        actionNodeNormal = new QAction(MainWindow);
        actionNodeNormal->setObjectName(QString::fromUtf8("actionNodeNormal"));
        actionNodeNormal->setCheckable(true);
        QIcon icon20;
        icon20.addFile(QString::fromUtf8(":/resource/NodeNormal.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNodeNormal->setIcon(icon20);
        actionSelectEdge = new QAction(MainWindow);
        actionSelectEdge->setObjectName(QString::fromUtf8("actionSelectEdge"));
        QIcon icon21;
        icon21.addFile(QString::fromUtf8(":/resource/selectEdge.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectEdge->setIcon(icon21);
        actionGenerate = new QAction(MainWindow);
        actionGenerate->setObjectName(QString::fromUtf8("actionGenerate"));
        actionTest_1 = new QAction(MainWindow);
        actionTest_1->setObjectName(QString::fromUtf8("actionTest_1"));
        actionSelectFix = new QAction(MainWindow);
        actionSelectFix->setObjectName(QString::fromUtf8("actionSelectFix"));
        QIcon icon22;
        icon22.addFile(QString::fromUtf8(":/resource/selectFix.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectFix->setIcon(icon22);
        actionSelectHandle = new QAction(MainWindow);
        actionSelectHandle->setObjectName(QString::fromUtf8("actionSelectHandle"));
        QIcon icon23;
        icon23.addFile(QString::fromUtf8(":/resource/selectHandle.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectHandle->setIcon(icon23);
        actionSaveSelection = new QAction(MainWindow);
        actionSaveSelection->setObjectName(QString::fromUtf8("actionSaveSelection"));
        QIcon icon24;
        icon24.addFile(QString::fromUtf8(":/resource/SaveSelection.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSaveSelection->setIcon(icon24);
        actionReadSelection = new QAction(MainWindow);
        actionReadSelection->setObjectName(QString::fromUtf8("actionReadSelection"));
        QIcon icon25;
        icon25.addFile(QString::fromUtf8(":/resource/InputSelection.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionReadSelection->setIcon(icon25);
        actionSelectChamber = new QAction(MainWindow);
        actionSelectChamber->setObjectName(QString::fromUtf8("actionSelectChamber"));
        actionExport_to_Abaqus_model = new QAction(MainWindow);
        actionExport_to_Abaqus_model->setObjectName(QString::fromUtf8("actionExport_to_Abaqus_model"));
        actionExport_to_Abaqus_model->setCheckable(false);
        QIcon icon26;
        icon26.addFile(QString::fromUtf8(":/resource/abaqus logo.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionExport_to_Abaqus_model->setIcon(icon26);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        centralWidget->setMouseTracking(true);
        centralWidget->setAcceptDrops(true);
        horizontalLayout = new QHBoxLayout(centralWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        MainWindow->setCentralWidget(centralWidget);
        navigationToolBar = new QToolBar(MainWindow);
        navigationToolBar->setObjectName(QString::fromUtf8("navigationToolBar"));
        navigationToolBar->setMovable(false);
        navigationToolBar->setIconSize(QSize(25, 25));
        navigationToolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, navigationToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);
        selectionToolBar = new QToolBar(MainWindow);
        selectionToolBar->setObjectName(QString::fromUtf8("selectionToolBar"));
        selectionToolBar->setMovable(false);
        selectionToolBar->setIconSize(QSize(25, 25));
        selectionToolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, selectionToolBar);
        dockWidget = new QDockWidget(MainWindow);
        dockWidget->setObjectName(QString::fromUtf8("dockWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(dockWidget->sizePolicy().hasHeightForWidth());
        dockWidget->setSizePolicy(sizePolicy);
        dockWidget->setMinimumSize(QSize(350, 900));
        dockWidget->setMaximumSize(QSize(350, 900));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        dockWidgetContents->setLayoutDirection(Qt::LeftToRight);
        verticalLayout = new QVBoxLayout(dockWidgetContents);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label_S3_DeformFDM = new QLabel(dockWidgetContents);
        label_S3_DeformFDM->setObjectName(QString::fromUtf8("label_S3_DeformFDM"));
        QFont font1;
        font1.setPointSize(10);
        label_S3_DeformFDM->setFont(font1);

        verticalLayout->addWidget(label_S3_DeformFDM);

        line = new QFrame(dockWidgetContents);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line);

        tabWidget = new QTabWidget(dockWidgetContents);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalLayout_3 = new QVBoxLayout(tab);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        label = new QLabel(tab);
        label->setObjectName(QString::fromUtf8("label"));
        label->setFont(font);
        label->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 255);"));

        horizontalLayout_12->addWidget(label);

        boxDeselect = new QCheckBox(tab);
        boxDeselect->setObjectName(QString::fromUtf8("boxDeselect"));

        horizontalLayout_12->addWidget(boxDeselect);


        verticalLayout_3->addLayout(horizontalLayout_12);

        line_2 = new QFrame(tab);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout_3->addWidget(line_2);

        horizontalLayout_31 = new QHBoxLayout();
        horizontalLayout_31->setSpacing(6);
        horizontalLayout_31->setObjectName(QString::fromUtf8("horizontalLayout_31"));
        label_46 = new QLabel(tab);
        label_46->setObjectName(QString::fromUtf8("label_46"));

        horizontalLayout_31->addWidget(label_46);

        label_47 = new QLabel(tab);
        label_47->setObjectName(QString::fromUtf8("label_47"));
        QFont font2;
        font2.setBold(false);
        font2.setWeight(50);
        label_47->setFont(font2);

        horizontalLayout_31->addWidget(label_47);

        spinBox_shapeUpIterNum = new QSpinBox(tab);
        spinBox_shapeUpIterNum->setObjectName(QString::fromUtf8("spinBox_shapeUpIterNum"));
        spinBox_shapeUpIterNum->setFont(font2);
        spinBox_shapeUpIterNum->setValue(0);

        horizontalLayout_31->addWidget(spinBox_shapeUpIterNum);

        label_48 = new QLabel(tab);
        label_48->setObjectName(QString::fromUtf8("label_48"));
        label_48->setFont(font2);

        horizontalLayout_31->addWidget(label_48);

        spinBox_innerIterNum = new QSpinBox(tab);
        spinBox_innerIterNum->setObjectName(QString::fromUtf8("spinBox_innerIterNum"));
        spinBox_innerIterNum->setFont(font2);

        horizontalLayout_31->addWidget(spinBox_innerIterNum);


        verticalLayout_3->addLayout(horizontalLayout_31);

        horizontalLayout_32 = new QHBoxLayout();
        horizontalLayout_32->setSpacing(6);
        horizontalLayout_32->setObjectName(QString::fromUtf8("horizontalLayout_32"));
        label_49 = new QLabel(tab);
        label_49->setObjectName(QString::fromUtf8("label_49"));
        label_49->setFont(font2);

        horizontalLayout_32->addWidget(label_49);

        label_50 = new QLabel(tab);
        label_50->setObjectName(QString::fromUtf8("label_50"));

        horizontalLayout_32->addWidget(label_50);

        doubleSpinBox_criticalTETWeight_SL = new QDoubleSpinBox(tab);
        doubleSpinBox_criticalTETWeight_SL->setObjectName(QString::fromUtf8("doubleSpinBox_criticalTETWeight_SL"));
        doubleSpinBox_criticalTETWeight_SL->setFont(font2);
        doubleSpinBox_criticalTETWeight_SL->setSingleStep(1.000000000000000);
        doubleSpinBox_criticalTETWeight_SL->setValue(0.000000000000000);

        horizontalLayout_32->addWidget(doubleSpinBox_criticalTETWeight_SL);

        label_51 = new QLabel(tab);
        label_51->setObjectName(QString::fromUtf8("label_51"));

        horizontalLayout_32->addWidget(label_51);

        doubleSpinBox_criticalTETWeight_SR = new QDoubleSpinBox(tab);
        doubleSpinBox_criticalTETWeight_SR->setObjectName(QString::fromUtf8("doubleSpinBox_criticalTETWeight_SR"));
        doubleSpinBox_criticalTETWeight_SR->setFont(font2);

        horizontalLayout_32->addWidget(doubleSpinBox_criticalTETWeight_SR);


        verticalLayout_3->addLayout(horizontalLayout_32);

        horizontalLayout_33 = new QHBoxLayout();
        horizontalLayout_33->setSpacing(6);
        horizontalLayout_33->setObjectName(QString::fromUtf8("horizontalLayout_33"));
        label_52 = new QLabel(tab);
        label_52->setObjectName(QString::fromUtf8("label_52"));
        label_52->setFont(font2);

        horizontalLayout_33->addWidget(label_52);

        label_2 = new QLabel(tab);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_33->addWidget(label_2);

        doubleSpinBox_criticalTETWeight_SQC = new QDoubleSpinBox(tab);
        doubleSpinBox_criticalTETWeight_SQC->setObjectName(QString::fromUtf8("doubleSpinBox_criticalTETWeight_SQC"));
        doubleSpinBox_criticalTETWeight_SQC->setFont(font2);

        horizontalLayout_33->addWidget(doubleSpinBox_criticalTETWeight_SQC);

        label_53 = new QLabel(tab);
        label_53->setObjectName(QString::fromUtf8("label_53"));

        horizontalLayout_33->addWidget(label_53);

        doubleSpinBox_criticalTETWeight_SQV = new QDoubleSpinBox(tab);
        doubleSpinBox_criticalTETWeight_SQV->setObjectName(QString::fromUtf8("doubleSpinBox_criticalTETWeight_SQV"));
        doubleSpinBox_criticalTETWeight_SQV->setFont(font2);

        horizontalLayout_33->addWidget(doubleSpinBox_criticalTETWeight_SQV);


        verticalLayout_3->addLayout(horizontalLayout_33);

        horizontalLayout_34 = new QHBoxLayout();
        horizontalLayout_34->setSpacing(6);
        horizontalLayout_34->setObjectName(QString::fromUtf8("horizontalLayout_34"));
        label_55 = new QLabel(tab);
        label_55->setObjectName(QString::fromUtf8("label_55"));

        horizontalLayout_34->addWidget(label_55);

        doubleSpinBox_neighborScaleWeight = new QDoubleSpinBox(tab);
        doubleSpinBox_neighborScaleWeight->setObjectName(QString::fromUtf8("doubleSpinBox_neighborScaleWeight"));
        doubleSpinBox_neighborScaleWeight->setFont(font2);
        doubleSpinBox_neighborScaleWeight->setValue(0.000000000000000);

        horizontalLayout_34->addWidget(doubleSpinBox_neighborScaleWeight);

        label_56 = new QLabel(tab);
        label_56->setObjectName(QString::fromUtf8("label_56"));

        horizontalLayout_34->addWidget(label_56);

        doubleSpinBox_regularScaleWeight = new QDoubleSpinBox(tab);
        doubleSpinBox_regularScaleWeight->setObjectName(QString::fromUtf8("doubleSpinBox_regularScaleWeight"));
        doubleSpinBox_regularScaleWeight->setFont(font2);
        doubleSpinBox_regularScaleWeight->setValue(0.000000000000000);

        horizontalLayout_34->addWidget(doubleSpinBox_regularScaleWeight);

        label_57 = new QLabel(tab);
        label_57->setObjectName(QString::fromUtf8("label_57"));

        horizontalLayout_34->addWidget(label_57);

        doubleSpinBox_globalQuatSmoothWeight = new QDoubleSpinBox(tab);
        doubleSpinBox_globalQuatSmoothWeight->setObjectName(QString::fromUtf8("doubleSpinBox_globalQuatSmoothWeight"));
        doubleSpinBox_globalQuatSmoothWeight->setFont(font2);
        doubleSpinBox_globalQuatSmoothWeight->setMaximum(999.899999999999977);
        doubleSpinBox_globalQuatSmoothWeight->setValue(0.000000000000000);

        horizontalLayout_34->addWidget(doubleSpinBox_globalQuatSmoothWeight);


        verticalLayout_3->addLayout(horizontalLayout_34);

        line_14 = new QFrame(tab);
        line_14->setObjectName(QString::fromUtf8("line_14"));
        line_14->setFrameShape(QFrame::HLine);
        line_14->setFrameShadow(QFrame::Sunken);

        verticalLayout_3->addWidget(line_14);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_8 = new QLabel(tab);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setFont(font2);

        horizontalLayout_2->addWidget(label_8);

        doubleSpinBox_supportFreeAngle = new QDoubleSpinBox(tab);
        doubleSpinBox_supportFreeAngle->setObjectName(QString::fromUtf8("doubleSpinBox_supportFreeAngle"));
        doubleSpinBox_supportFreeAngle->setValue(0.000000000000000);

        horizontalLayout_2->addWidget(doubleSpinBox_supportFreeAngle);


        verticalLayout_3->addLayout(horizontalLayout_2);

        horizontalLayout_28 = new QHBoxLayout();
        horizontalLayout_28->setSpacing(6);
        horizontalLayout_28->setObjectName(QString::fromUtf8("horizontalLayout_28"));
        pushButton_calBestOrientation = new QPushButton(tab);
        pushButton_calBestOrientation->setObjectName(QString::fromUtf8("pushButton_calBestOrientation"));
        pushButton_calBestOrientation->setFont(font2);

        horizontalLayout_28->addWidget(pushButton_calBestOrientation);

        checkBox_computeBestOrientation = new QCheckBox(tab);
        checkBox_computeBestOrientation->setObjectName(QString::fromUtf8("checkBox_computeBestOrientation"));
        checkBox_computeBestOrientation->setFont(font2);

        horizontalLayout_28->addWidget(checkBox_computeBestOrientation);


        verticalLayout_3->addLayout(horizontalLayout_28);

        pushButton_s3DeformFDM_supportLess_ASAP = new QPushButton(tab);
        pushButton_s3DeformFDM_supportLess_ASAP->setObjectName(QString::fromUtf8("pushButton_s3DeformFDM_supportLess_ASAP"));
        pushButton_s3DeformFDM_supportLess_ASAP->setFont(font);

        verticalLayout_3->addWidget(pushButton_s3DeformFDM_supportLess_ASAP);

        line_4 = new QFrame(tab);
        line_4->setObjectName(QString::fromUtf8("line_4"));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);

        verticalLayout_3->addWidget(line_4);

        horizontalLayout_30 = new QHBoxLayout();
        horizontalLayout_30->setSpacing(6);
        horizontalLayout_30->setObjectName(QString::fromUtf8("horizontalLayout_30"));
        label_10 = new QLabel(tab);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setFont(font2);

        horizontalLayout_30->addWidget(label_10);

        label_44 = new QLabel(tab);
        label_44->setObjectName(QString::fromUtf8("label_44"));
        label_44->setFont(font2);

        horizontalLayout_30->addWidget(label_44);

        doubleSpinBox_tensileRegionRatio = new QDoubleSpinBox(tab);
        doubleSpinBox_tensileRegionRatio->setObjectName(QString::fromUtf8("doubleSpinBox_tensileRegionRatio"));
        doubleSpinBox_tensileRegionRatio->setFont(font2);
        doubleSpinBox_tensileRegionRatio->setSingleStep(0.100000000000000);
        doubleSpinBox_tensileRegionRatio->setValue(0.000000000000000);

        horizontalLayout_30->addWidget(doubleSpinBox_tensileRegionRatio);

        label_11 = new QLabel(tab);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setFont(font2);

        horizontalLayout_30->addWidget(label_11);

        doubleSpinBox_compressRegionRatio = new QDoubleSpinBox(tab);
        doubleSpinBox_compressRegionRatio->setObjectName(QString::fromUtf8("doubleSpinBox_compressRegionRatio"));
        doubleSpinBox_compressRegionRatio->setFont(font2);
        doubleSpinBox_compressRegionRatio->setSingleStep(0.100000000000000);
        doubleSpinBox_compressRegionRatio->setValue(0.000000000000000);

        horizontalLayout_30->addWidget(doubleSpinBox_compressRegionRatio);


        verticalLayout_3->addLayout(horizontalLayout_30);

        pushButton_s3DeformFDM_strengthReinforcement_ASAP = new QPushButton(tab);
        pushButton_s3DeformFDM_strengthReinforcement_ASAP->setObjectName(QString::fromUtf8("pushButton_s3DeformFDM_strengthReinforcement_ASAP"));

        verticalLayout_3->addWidget(pushButton_s3DeformFDM_strengthReinforcement_ASAP);

        pushButton_s3DeformFDM_surfaceQuality_ASAP = new QPushButton(tab);
        pushButton_s3DeformFDM_surfaceQuality_ASAP->setObjectName(QString::fromUtf8("pushButton_s3DeformFDM_surfaceQuality_ASAP"));

        verticalLayout_3->addWidget(pushButton_s3DeformFDM_surfaceQuality_ASAP);

        line_6 = new QFrame(tab);
        line_6->setObjectName(QString::fromUtf8("line_6"));
        line_6->setFrameShape(QFrame::HLine);
        line_6->setFrameShadow(QFrame::Sunken);

        verticalLayout_3->addWidget(line_6);

        label_4 = new QLabel(tab);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        verticalLayout_3->addWidget(label_4);

        horizontalLayout_36 = new QHBoxLayout();
        horizontalLayout_36->setSpacing(6);
        horizontalLayout_36->setObjectName(QString::fromUtf8("horizontalLayout_36"));
        label_6 = new QLabel(tab);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setFont(font2);

        horizontalLayout_36->addWidget(label_6);

        doubleSpinBox_keepWeight_SL = new QDoubleSpinBox(tab);
        doubleSpinBox_keepWeight_SL->setObjectName(QString::fromUtf8("doubleSpinBox_keepWeight_SL"));
        doubleSpinBox_keepWeight_SL->setFont(font2);
        doubleSpinBox_keepWeight_SL->setMinimum(0.010000000000000);
        doubleSpinBox_keepWeight_SL->setMaximum(999.990000000000009);
        doubleSpinBox_keepWeight_SL->setValue(1.000000000000000);

        horizontalLayout_36->addWidget(doubleSpinBox_keepWeight_SL);

        label_58 = new QLabel(tab);
        label_58->setObjectName(QString::fromUtf8("label_58"));
        label_58->setFont(font2);

        horizontalLayout_36->addWidget(label_58);

        doubleSpinBox_keepWeight_SR = new QDoubleSpinBox(tab);
        doubleSpinBox_keepWeight_SR->setObjectName(QString::fromUtf8("doubleSpinBox_keepWeight_SR"));
        doubleSpinBox_keepWeight_SR->setFont(font2);
        doubleSpinBox_keepWeight_SR->setMinimum(0.010000000000000);
        doubleSpinBox_keepWeight_SR->setMaximum(999.990000000000009);
        doubleSpinBox_keepWeight_SR->setValue(1.000000000000000);

        horizontalLayout_36->addWidget(doubleSpinBox_keepWeight_SR);


        verticalLayout_3->addLayout(horizontalLayout_36);

        horizontalLayout_35 = new QHBoxLayout();
        horizontalLayout_35->setSpacing(6);
        horizontalLayout_35->setObjectName(QString::fromUtf8("horizontalLayout_35"));
        label_5 = new QLabel(tab);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setFont(font2);

        horizontalLayout_35->addWidget(label_5);

        doubleSpinBox_keepWeight_SQC = new QDoubleSpinBox(tab);
        doubleSpinBox_keepWeight_SQC->setObjectName(QString::fromUtf8("doubleSpinBox_keepWeight_SQC"));
        doubleSpinBox_keepWeight_SQC->setFont(font2);
        doubleSpinBox_keepWeight_SQC->setMinimum(0.010000000000000);
        doubleSpinBox_keepWeight_SQC->setMaximum(999.990000000000009);
        doubleSpinBox_keepWeight_SQC->setValue(1.000000000000000);

        horizontalLayout_35->addWidget(doubleSpinBox_keepWeight_SQC);

        label_54 = new QLabel(tab);
        label_54->setObjectName(QString::fromUtf8("label_54"));
        label_54->setFont(font2);

        horizontalLayout_35->addWidget(label_54);

        doubleSpinBox_keepWeight_SQV = new QDoubleSpinBox(tab);
        doubleSpinBox_keepWeight_SQV->setObjectName(QString::fromUtf8("doubleSpinBox_keepWeight_SQV"));
        doubleSpinBox_keepWeight_SQV->setFont(font2);
        doubleSpinBox_keepWeight_SQV->setMinimum(0.010000000000000);
        doubleSpinBox_keepWeight_SQV->setMaximum(999.990000000000009);
        doubleSpinBox_keepWeight_SQV->setValue(1.000000000000000);

        horizontalLayout_35->addWidget(doubleSpinBox_keepWeight_SQV);


        verticalLayout_3->addLayout(horizontalLayout_35);

        horizontalLayout_37 = new QHBoxLayout();
        horizontalLayout_37->setSpacing(6);
        horizontalLayout_37->setObjectName(QString::fromUtf8("horizontalLayout_37"));
        pushButton_s3DeformFDM_hybrid_SL_SQ_ASAP = new QPushButton(tab);
        pushButton_s3DeformFDM_hybrid_SL_SQ_ASAP->setObjectName(QString::fromUtf8("pushButton_s3DeformFDM_hybrid_SL_SQ_ASAP"));
        pushButton_s3DeformFDM_hybrid_SL_SQ_ASAP->setEnabled(true);

        horizontalLayout_37->addWidget(pushButton_s3DeformFDM_hybrid_SL_SQ_ASAP);

        pushButton_s3DeformFDM_hybrid_SR_SQ_ASAP = new QPushButton(tab);
        pushButton_s3DeformFDM_hybrid_SR_SQ_ASAP->setObjectName(QString::fromUtf8("pushButton_s3DeformFDM_hybrid_SR_SQ_ASAP"));
        pushButton_s3DeformFDM_hybrid_SR_SQ_ASAP->setEnabled(true);

        horizontalLayout_37->addWidget(pushButton_s3DeformFDM_hybrid_SR_SQ_ASAP);

        pushButton_s3DeformFDM_hybrid_SL_SR_ASAP = new QPushButton(tab);
        pushButton_s3DeformFDM_hybrid_SL_SR_ASAP->setObjectName(QString::fromUtf8("pushButton_s3DeformFDM_hybrid_SL_SR_ASAP"));

        horizontalLayout_37->addWidget(pushButton_s3DeformFDM_hybrid_SL_SR_ASAP);


        verticalLayout_3->addLayout(horizontalLayout_37);

        pushButton_s3DeformFDM_hybrid_SL_SR_SQ_ASAP = new QPushButton(tab);
        pushButton_s3DeformFDM_hybrid_SL_SR_SQ_ASAP->setObjectName(QString::fromUtf8("pushButton_s3DeformFDM_hybrid_SL_SR_SQ_ASAP"));

        verticalLayout_3->addWidget(pushButton_s3DeformFDM_hybrid_SL_SR_SQ_ASAP);

        pushButton_s3DeformFDM_inverseDeform = new QPushButton(tab);
        pushButton_s3DeformFDM_inverseDeform->setObjectName(QString::fromUtf8("pushButton_s3DeformFDM_inverseDeform"));
        pushButton_s3DeformFDM_inverseDeform->setEnabled(false);

        verticalLayout_3->addWidget(pushButton_s3DeformFDM_inverseDeform);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_2);

        tabWidget->addTab(tab, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        verticalLayout_5 = new QVBoxLayout(tab_4);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        label_7 = new QLabel(tab_4);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        verticalLayout_5->addWidget(label_7);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_14 = new QLabel(tab_4);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setFont(font2);

        horizontalLayout_6->addWidget(label_14);

        horizontalSlider_slice_Multi_dir = new QSlider(tab_4);
        horizontalSlider_slice_Multi_dir->setObjectName(QString::fromUtf8("horizontalSlider_slice_Multi_dir"));
        horizontalSlider_slice_Multi_dir->setFont(font2);
        horizontalSlider_slice_Multi_dir->setSliderPosition(50);
        horizontalSlider_slice_Multi_dir->setOrientation(Qt::Horizontal);

        horizontalLayout_6->addWidget(horizontalSlider_slice_Multi_dir);

        comboBox_planeDir = new QComboBox(tab_4);
        comboBox_planeDir->addItem(QString());
        comboBox_planeDir->addItem(QString());
        comboBox_planeDir->addItem(QString());
        comboBox_planeDir->setObjectName(QString::fromUtf8("comboBox_planeDir"));
        comboBox_planeDir->setFont(font2);

        horizontalLayout_6->addWidget(comboBox_planeDir);


        verticalLayout_5->addLayout(horizontalLayout_6);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        pushButton_drawScaleValue = new QPushButton(tab_4);
        pushButton_drawScaleValue->setObjectName(QString::fromUtf8("pushButton_drawScaleValue"));
        pushButton_drawScaleValue->setFont(font2);

        horizontalLayout_5->addWidget(pushButton_drawScaleValue);

        comboBox_scaleValueDirection = new QComboBox(tab_4);
        comboBox_scaleValueDirection->addItem(QString());
        comboBox_scaleValueDirection->addItem(QString());
        comboBox_scaleValueDirection->addItem(QString());
        comboBox_scaleValueDirection->addItem(QString());
        comboBox_scaleValueDirection->addItem(QString());
        comboBox_scaleValueDirection->setObjectName(QString::fromUtf8("comboBox_scaleValueDirection"));
        comboBox_scaleValueDirection->setFont(font2);

        horizontalLayout_5->addWidget(comboBox_scaleValueDirection);


        verticalLayout_5->addLayout(horizontalLayout_5);

        label_9 = new QLabel(tab_4);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        verticalLayout_5->addWidget(label_9);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_12 = new QLabel(tab_4);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setFont(font2);

        horizontalLayout_4->addWidget(label_12);

        spinBox_ShowLayerIndex = new QSpinBox(tab_4);
        spinBox_ShowLayerIndex->setObjectName(QString::fromUtf8("spinBox_ShowLayerIndex"));
        spinBox_ShowLayerIndex->setFont(font2);
        spinBox_ShowLayerIndex->setMaximum(999);

        horizontalLayout_4->addWidget(spinBox_ShowLayerIndex);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_6);

        checkBox_EachLayerSwitch = new QCheckBox(tab_4);
        checkBox_EachLayerSwitch->setObjectName(QString::fromUtf8("checkBox_EachLayerSwitch"));
        checkBox_EachLayerSwitch->setFont(font2);

        horizontalLayout_4->addWidget(checkBox_EachLayerSwitch);


        verticalLayout_5->addLayout(horizontalLayout_4);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        pushButton_ShowAllLayers = new QPushButton(tab_4);
        pushButton_ShowAllLayers->setObjectName(QString::fromUtf8("pushButton_ShowAllLayers"));
        pushButton_ShowAllLayers->setFont(font2);

        horizontalLayout_8->addWidget(pushButton_ShowAllLayers);

        radioButton_compatibleLayer = new QRadioButton(tab_4);
        radioButton_compatibleLayer->setObjectName(QString::fromUtf8("radioButton_compatibleLayer"));
        radioButton_compatibleLayer->setEnabled(false);
        radioButton_compatibleLayer->setFont(font2);

        horizontalLayout_8->addWidget(radioButton_compatibleLayer);


        verticalLayout_5->addLayout(horizontalLayout_8);

        line_3 = new QFrame(tab_4);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);

        verticalLayout_5->addWidget(line_3);

        label_3 = new QLabel(tab_4);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout_5->addWidget(label_3);

        horizontalLayout_23 = new QHBoxLayout();
        horizontalLayout_23->setSpacing(6);
        horizontalLayout_23->setObjectName(QString::fromUtf8("horizontalLayout_23"));
        label_36 = new QLabel(tab_4);
        label_36->setObjectName(QString::fromUtf8("label_36"));
        label_36->setFont(font2);

        horizontalLayout_23->addWidget(label_36);

        label_39 = new QLabel(tab_4);
        label_39->setObjectName(QString::fromUtf8("label_39"));
        label_39->setFont(font2);

        horizontalLayout_23->addWidget(label_39);

        doubleSpinBox_rot_phi = new QDoubleSpinBox(tab_4);
        doubleSpinBox_rot_phi->setObjectName(QString::fromUtf8("doubleSpinBox_rot_phi"));
        doubleSpinBox_rot_phi->setFont(font2);
        doubleSpinBox_rot_phi->setMinimum(-180.000000000000000);
        doubleSpinBox_rot_phi->setMaximum(180.000000000000000);

        horizontalLayout_23->addWidget(doubleSpinBox_rot_phi);

        label_40 = new QLabel(tab_4);
        label_40->setObjectName(QString::fromUtf8("label_40"));
        label_40->setFont(font2);

        horizontalLayout_23->addWidget(label_40);

        doubleSpinBox_rot_theta = new QDoubleSpinBox(tab_4);
        doubleSpinBox_rot_theta->setObjectName(QString::fromUtf8("doubleSpinBox_rot_theta"));
        doubleSpinBox_rot_theta->setFont(font2);
        doubleSpinBox_rot_theta->setMaximum(360.000000000000000);

        horizontalLayout_23->addWidget(doubleSpinBox_rot_theta);


        verticalLayout_5->addLayout(horizontalLayout_23);

        horizontalLayout_24 = new QHBoxLayout();
        horizontalLayout_24->setSpacing(6);
        horizontalLayout_24->setObjectName(QString::fromUtf8("horizontalLayout_24"));
        label_41 = new QLabel(tab_4);
        label_41->setObjectName(QString::fromUtf8("label_41"));
        label_41->setFont(font2);

        horizontalLayout_24->addWidget(label_41);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_24->addItem(horizontalSpacer_5);

        label_43 = new QLabel(tab_4);
        label_43->setObjectName(QString::fromUtf8("label_43"));
        label_43->setFont(font2);

        horizontalLayout_24->addWidget(label_43);

        doubleSpinBox_modelHeight_y = new QDoubleSpinBox(tab_4);
        doubleSpinBox_modelHeight_y->setObjectName(QString::fromUtf8("doubleSpinBox_modelHeight_y"));
        doubleSpinBox_modelHeight_y->setFont(font2);

        horizontalLayout_24->addWidget(doubleSpinBox_modelHeight_y);


        verticalLayout_5->addLayout(horizontalLayout_24);

        pushButton_model_move_rotation = new QPushButton(tab_4);
        pushButton_model_move_rotation->setObjectName(QString::fromUtf8("pushButton_model_move_rotation"));

        verticalLayout_5->addWidget(pushButton_model_move_rotation);

        line_13 = new QFrame(tab_4);
        line_13->setObjectName(QString::fromUtf8("line_13"));
        line_13->setFrameShape(QFrame::HLine);
        line_13->setFrameShadow(QFrame::Sunken);

        verticalLayout_5->addWidget(line_13);

        label_45 = new QLabel(tab_4);
        label_45->setObjectName(QString::fromUtf8("label_45"));

        verticalLayout_5->addWidget(label_45);

        pushButton_tet2surface = new QPushButton(tab_4);
        pushButton_tet2surface->setObjectName(QString::fromUtf8("pushButton_tet2surface"));

        verticalLayout_5->addWidget(pushButton_tet2surface);

        pushButton_output_vectorVSobjective = new QPushButton(tab_4);
        pushButton_output_vectorVSobjective->setObjectName(QString::fromUtf8("pushButton_output_vectorVSobjective"));

        verticalLayout_5->addWidget(pushButton_output_vectorVSobjective);

        pushButton_output_userWaypoints = new QPushButton(tab_4);
        pushButton_output_userWaypoints->setObjectName(QString::fromUtf8("pushButton_output_userWaypoints"));
        pushButton_output_userWaypoints->setEnabled(false);

        verticalLayout_5->addWidget(pushButton_output_userWaypoints);

        pushButton_output_streeField = new QPushButton(tab_4);
        pushButton_output_streeField->setObjectName(QString::fromUtf8("pushButton_output_streeField"));

        verticalLayout_5->addWidget(pushButton_output_streeField);

        pushButton_output2Robot = new QPushButton(tab_4);
        pushButton_output2Robot->setObjectName(QString::fromUtf8("pushButton_output2Robot"));
        pushButton_output2Robot->setEnabled(false);

        verticalLayout_5->addWidget(pushButton_output2Robot);

        pushButton_output_QvectorField = new QPushButton(tab_4);
        pushButton_output_QvectorField->setObjectName(QString::fromUtf8("pushButton_output_QvectorField"));
        pushButton_output_QvectorField->setEnabled(false);

        verticalLayout_5->addWidget(pushButton_output_QvectorField);

        horizontalLayout_38 = new QHBoxLayout();
        horizontalLayout_38->setSpacing(6);
        horizontalLayout_38->setObjectName(QString::fromUtf8("horizontalLayout_38"));
        pushButton_output_ScalarOrHeight_Field = new QPushButton(tab_4);
        pushButton_output_ScalarOrHeight_Field->setObjectName(QString::fromUtf8("pushButton_output_ScalarOrHeight_Field"));
        pushButton_output_ScalarOrHeight_Field->setEnabled(false);

        horizontalLayout_38->addWidget(pushButton_output_ScalarOrHeight_Field);

        checkBox_scalarOrHeight_field = new QCheckBox(tab_4);
        checkBox_scalarOrHeight_field->setObjectName(QString::fromUtf8("checkBox_scalarOrHeight_field"));
        checkBox_scalarOrHeight_field->setFont(font2);

        horizontalLayout_38->addWidget(checkBox_scalarOrHeight_field);


        verticalLayout_5->addLayout(horizontalLayout_38);

        pushButton_output_deformedTet = new QPushButton(tab_4);
        pushButton_output_deformedTet->setObjectName(QString::fromUtf8("pushButton_output_deformedTet"));

        verticalLayout_5->addWidget(pushButton_output_deformedTet);

        pushButton_output_discretedTet = new QPushButton(tab_4);
        pushButton_output_discretedTet->setObjectName(QString::fromUtf8("pushButton_output_discretedTet"));

        verticalLayout_5->addWidget(pushButton_output_discretedTet);

        pushButton_output_discretedTet_obj = new QPushButton(tab_4);
        pushButton_output_discretedTet_obj->setObjectName(QString::fromUtf8("pushButton_output_discretedTet_obj"));
        pushButton_output_discretedTet_obj->setEnabled(true);

        verticalLayout_5->addWidget(pushButton_output_discretedTet_obj);

        pushButton_output_materialRot_4CAE = new QPushButton(tab_4);
        pushButton_output_materialRot_4CAE->setObjectName(QString::fromUtf8("pushButton_output_materialRot_4CAE"));

        verticalLayout_5->addWidget(pushButton_output_materialRot_4CAE);

        tabWidget->addTab(tab_4, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        verticalLayout_2 = new QVBoxLayout(tab_2);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_13 = new QLabel(tab_2);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setFont(font2);

        horizontalLayout_3->addWidget(label_13);

        spinBox_isoLayerNumber = new QSpinBox(tab_2);
        spinBox_isoLayerNumber->setObjectName(QString::fromUtf8("spinBox_isoLayerNumber"));
        spinBox_isoLayerNumber->setMinimumSize(QSize(0, 0));
        spinBox_isoLayerNumber->setFont(font2);
        spinBox_isoLayerNumber->setMaximum(999);
        spinBox_isoLayerNumber->setValue(100);

        horizontalLayout_3->addWidget(spinBox_isoLayerNumber);

        pushButton_outputIsoLayerSet = new QPushButton(tab_2);
        pushButton_outputIsoLayerSet->setObjectName(QString::fromUtf8("pushButton_outputIsoLayerSet"));

        horizontalLayout_3->addWidget(pushButton_outputIsoLayerSet);


        verticalLayout_2->addLayout(horizontalLayout_3);

        pushButton_isoLayerGeneration = new QPushButton(tab_2);
        pushButton_isoLayerGeneration->setObjectName(QString::fromUtf8("pushButton_isoLayerGeneration"));
        pushButton_isoLayerGeneration->setEnabled(false);

        verticalLayout_2->addWidget(pushButton_isoLayerGeneration);

        pushButton_adaptiveHeightSlicing = new QPushButton(tab_2);
        pushButton_adaptiveHeightSlicing->setObjectName(QString::fromUtf8("pushButton_adaptiveHeightSlicing"));
        pushButton_adaptiveHeightSlicing->setEnabled(true);

        verticalLayout_2->addWidget(pushButton_adaptiveHeightSlicing);

        line_12 = new QFrame(tab_2);
        line_12->setObjectName(QString::fromUtf8("line_12"));
        line_12->setFrameShape(QFrame::HLine);
        line_12->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_12);

        label_38 = new QLabel(tab_2);
        label_38->setObjectName(QString::fromUtf8("label_38"));

        verticalLayout_2->addWidget(label_38);

        horizontalLayout_27 = new QHBoxLayout();
        horizontalLayout_27->setSpacing(6);
        horizontalLayout_27->setObjectName(QString::fromUtf8("horizontalLayout_27"));
        label_37 = new QLabel(tab_2);
        label_37->setObjectName(QString::fromUtf8("label_37"));
        label_37->setFont(font2);

        horizontalLayout_27->addWidget(label_37);

        pushButton_collisionChecking_vector = new QPushButton(tab_2);
        pushButton_collisionChecking_vector->setObjectName(QString::fromUtf8("pushButton_collisionChecking_vector"));
        QFont font3;
        font3.setPointSize(8);
        font3.setBold(false);
        font3.setWeight(50);
        pushButton_collisionChecking_vector->setFont(font3);

        horizontalLayout_27->addWidget(pushButton_collisionChecking_vector);

        pushButton_concaveSmooth = new QPushButton(tab_2);
        pushButton_concaveSmooth->setObjectName(QString::fromUtf8("pushButton_concaveSmooth"));
        pushButton_concaveSmooth->setFont(font3);

        horizontalLayout_27->addWidget(pushButton_concaveSmooth);

        pushButton_concaveRelaxLoop = new QPushButton(tab_2);
        pushButton_concaveRelaxLoop->setObjectName(QString::fromUtf8("pushButton_concaveRelaxLoop"));
        pushButton_concaveRelaxLoop->setFont(font3);

        horizontalLayout_27->addWidget(pushButton_concaveRelaxLoop);

        spinBox_concaveDetecSmooth_loopTime = new QSpinBox(tab_2);
        spinBox_concaveDetecSmooth_loopTime->setObjectName(QString::fromUtf8("spinBox_concaveDetecSmooth_loopTime"));
        spinBox_concaveDetecSmooth_loopTime->setFont(font2);
        spinBox_concaveDetecSmooth_loopTime->setMaximum(999);
        spinBox_concaveDetecSmooth_loopTime->setValue(25);

        horizontalLayout_27->addWidget(spinBox_concaveDetecSmooth_loopTime);


        verticalLayout_2->addLayout(horizontalLayout_27);

        pushButton_collisionChecking = new QPushButton(tab_2);
        pushButton_collisionChecking->setObjectName(QString::fromUtf8("pushButton_collisionChecking"));

        verticalLayout_2->addWidget(pushButton_collisionChecking);

        horizontalLayout_25 = new QHBoxLayout();
        horizontalLayout_25->setSpacing(6);
        horizontalLayout_25->setObjectName(QString::fromUtf8("horizontalLayout_25"));
        pushButton_remarkCollisionTetra = new QPushButton(tab_2);
        pushButton_remarkCollisionTetra->setObjectName(QString::fromUtf8("pushButton_remarkCollisionTetra"));
        pushButton_remarkCollisionTetra->setEnabled(false);
        pushButton_remarkCollisionTetra->setFont(font2);

        horizontalLayout_25->addWidget(pushButton_remarkCollisionTetra);

        pushButton_collisionAware_flattening = new QPushButton(tab_2);
        pushButton_collisionAware_flattening->setObjectName(QString::fromUtf8("pushButton_collisionAware_flattening"));
        pushButton_collisionAware_flattening->setEnabled(false);

        horizontalLayout_25->addWidget(pushButton_collisionAware_flattening);


        verticalLayout_2->addLayout(horizontalLayout_25);

        line_11 = new QFrame(tab_2);
        line_11->setObjectName(QString::fromUtf8("line_11"));
        line_11->setFrameShape(QFrame::HLine);
        line_11->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_11);

        line_5 = new QFrame(tab_2);
        line_5->setObjectName(QString::fromUtf8("line_5"));
        line_5->setFrameShape(QFrame::HLine);
        line_5->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_5);

        label_15 = new QLabel(tab_2);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        verticalLayout_2->addWidget(label_15);

        label_18 = new QLabel(tab_2);
        label_18->setObjectName(QString::fromUtf8("label_18"));
        label_18->setFont(font2);

        verticalLayout_2->addWidget(label_18);

        formLayout_4 = new QFormLayout();
        formLayout_4->setSpacing(6);
        formLayout_4->setObjectName(QString::fromUtf8("formLayout_4"));
        pushButton_Comp_initialGuess_envelopSupport = new QPushButton(tab_2);
        pushButton_Comp_initialGuess_envelopSupport->setObjectName(QString::fromUtf8("pushButton_Comp_initialGuess_envelopSupport"));
        pushButton_Comp_initialGuess_envelopSupport->setFont(font2);

        formLayout_4->setWidget(0, QFormLayout::LabelRole, pushButton_Comp_initialGuess_envelopSupport);

        label_19 = new QLabel(tab_2);
        label_19->setObjectName(QString::fromUtf8("label_19"));
        label_19->setFont(font2);

        formLayout_4->setWidget(0, QFormLayout::FieldRole, label_19);

        pushButton_MeshCombination = new QPushButton(tab_2);
        pushButton_MeshCombination->setObjectName(QString::fromUtf8("pushButton_MeshCombination"));
        pushButton_MeshCombination->setFont(font2);

        formLayout_4->setWidget(1, QFormLayout::LabelRole, pushButton_MeshCombination);

        label_20 = new QLabel(tab_2);
        label_20->setObjectName(QString::fromUtf8("label_20"));
        label_20->setFont(font2);

        formLayout_4->setWidget(1, QFormLayout::FieldRole, label_20);


        verticalLayout_2->addLayout(formLayout_4);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        pushButton_compatibleLayer_generation = new QPushButton(tab_2);
        pushButton_compatibleLayer_generation->setObjectName(QString::fromUtf8("pushButton_compatibleLayer_generation"));
        pushButton_compatibleLayer_generation->setFont(font2);

        horizontalLayout_7->addWidget(pushButton_compatibleLayer_generation);

        label_16 = new QLabel(tab_2);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setFont(font2);

        horizontalLayout_7->addWidget(label_16);


        verticalLayout_2->addLayout(horizontalLayout_7);

        horizontalLayout_26 = new QHBoxLayout();
        horizontalLayout_26->setSpacing(6);
        horizontalLayout_26->setObjectName(QString::fromUtf8("horizontalLayout_26"));
        pushButton_markOverhangFace = new QPushButton(tab_2);
        pushButton_markOverhangFace->setObjectName(QString::fromUtf8("pushButton_markOverhangFace"));
        pushButton_markOverhangFace->setFont(font2);

        horizontalLayout_26->addWidget(pushButton_markOverhangFace);

        pushButton_remarkOverhangFace = new QPushButton(tab_2);
        pushButton_remarkOverhangFace->setObjectName(QString::fromUtf8("pushButton_remarkOverhangFace"));
        pushButton_remarkOverhangFace->setEnabled(false);
        pushButton_remarkOverhangFace->setFont(font2);

        horizontalLayout_26->addWidget(pushButton_remarkOverhangFace);


        verticalLayout_2->addLayout(horizontalLayout_26);

        pushButton_treeSkeletonGeneration = new QPushButton(tab_2);
        pushButton_treeSkeletonGeneration->setObjectName(QString::fromUtf8("pushButton_treeSkeletonGeneration"));
        pushButton_treeSkeletonGeneration->setEnabled(false);

        verticalLayout_2->addWidget(pushButton_treeSkeletonGeneration);

        pushButton_slimmedSupportGeneration = new QPushButton(tab_2);
        pushButton_slimmedSupportGeneration->setObjectName(QString::fromUtf8("pushButton_slimmedSupportGeneration"));
        pushButton_slimmedSupportGeneration->setEnabled(false);

        verticalLayout_2->addWidget(pushButton_slimmedSupportGeneration);

        line_7 = new QFrame(tab_2);
        line_7->setObjectName(QString::fromUtf8("line_7"));
        line_7->setFrameShape(QFrame::HLine);
        line_7->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_7);

        label_17 = new QLabel(tab_2);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        verticalLayout_2->addWidget(label_17);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        label_22 = new QLabel(tab_2);
        label_22->setObjectName(QString::fromUtf8("label_22"));
        label_22->setFont(font2);

        horizontalLayout_10->addWidget(label_22);

        doubleSpinBox_toolPathWidth = new QDoubleSpinBox(tab_2);
        doubleSpinBox_toolPathWidth->setObjectName(QString::fromUtf8("doubleSpinBox_toolPathWidth"));
        doubleSpinBox_toolPathWidth->setFont(font2);
        doubleSpinBox_toolPathWidth->setSingleStep(0.100000000000000);
        doubleSpinBox_toolPathWidth->setValue(0.700000000000000);

        horizontalLayout_10->addWidget(doubleSpinBox_toolPathWidth);

        label_21 = new QLabel(tab_2);
        label_21->setObjectName(QString::fromUtf8("label_21"));
        label_21->setFont(font2);

        horizontalLayout_10->addWidget(label_21);

        doubleSpinBox_toolPathDistance = new QDoubleSpinBox(tab_2);
        doubleSpinBox_toolPathDistance->setObjectName(QString::fromUtf8("doubleSpinBox_toolPathDistance"));
        doubleSpinBox_toolPathDistance->setFont(font2);
        doubleSpinBox_toolPathDistance->setValue(1.000000000000000);

        horizontalLayout_10->addWidget(doubleSpinBox_toolPathDistance);


        verticalLayout_2->addLayout(horizontalLayout_10);

        line_15 = new QFrame(tab_2);
        line_15->setObjectName(QString::fromUtf8("line_15"));
        line_15->setFrameShape(QFrame::HLine);
        line_15->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_15);

        pushButton_toolPathGeneration = new QPushButton(tab_2);
        pushButton_toolPathGeneration->setObjectName(QString::fromUtf8("pushButton_toolPathGeneration"));

        verticalLayout_2->addWidget(pushButton_toolPathGeneration);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        pushButton_outputToolpath = new QPushButton(tab_2);
        pushButton_outputToolpath->setObjectName(QString::fromUtf8("pushButton_outputToolpath"));

        horizontalLayout_9->addWidget(pushButton_outputToolpath);

        checkBox_outputCompatiblewaypoint = new QCheckBox(tab_2);
        checkBox_outputCompatiblewaypoint->setObjectName(QString::fromUtf8("checkBox_outputCompatiblewaypoint"));
        QFont font4;
        font4.setPointSize(9);
        font4.setBold(true);
        font4.setWeight(75);
        checkBox_outputCompatiblewaypoint->setFont(font4);

        horizontalLayout_9->addWidget(checkBox_outputCompatiblewaypoint);


        verticalLayout_2->addLayout(horizontalLayout_9);

        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        verticalLayout_4 = new QVBoxLayout(tab_3);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        label_23 = new QLabel(tab_3);
        label_23->setObjectName(QString::fromUtf8("label_23"));

        horizontalLayout_11->addWidget(label_23);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_11->addItem(horizontalSpacer);

        checkBox_Yup2Zup = new QCheckBox(tab_3);
        checkBox_Yup2Zup->setObjectName(QString::fromUtf8("checkBox_Yup2Zup"));
        checkBox_Yup2Zup->setFont(font2);

        horizontalLayout_11->addWidget(checkBox_Yup2Zup);


        verticalLayout_4->addLayout(horizontalLayout_11);

        horizontalLayout_29 = new QHBoxLayout();
        horizontalLayout_29->setSpacing(6);
        horizontalLayout_29->setObjectName(QString::fromUtf8("horizontalLayout_29"));
        label_42 = new QLabel(tab_3);
        label_42->setObjectName(QString::fromUtf8("label_42"));

        horizontalLayout_29->addWidget(label_42);

        doubleSpinBox_toolLength = new QDoubleSpinBox(tab_3);
        doubleSpinBox_toolLength->setObjectName(QString::fromUtf8("doubleSpinBox_toolLength"));
        doubleSpinBox_toolLength->setMinimum(-1.000000000000000);
        doubleSpinBox_toolLength->setMaximum(300.000000000000000);
        doubleSpinBox_toolLength->setValue(77.849999999999994);

        horizontalLayout_29->addWidget(doubleSpinBox_toolLength);


        verticalLayout_4->addLayout(horizontalLayout_29);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setSpacing(6);
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        label_24 = new QLabel(tab_3);
        label_24->setObjectName(QString::fromUtf8("label_24"));

        horizontalLayout_13->addWidget(label_24);

        doubleSpinBox_Xmove = new QDoubleSpinBox(tab_3);
        doubleSpinBox_Xmove->setObjectName(QString::fromUtf8("doubleSpinBox_Xmove"));
        doubleSpinBox_Xmove->setFont(font2);
        doubleSpinBox_Xmove->setMinimum(-200.000000000000000);
        doubleSpinBox_Xmove->setMaximum(200.000000000000000);

        horizontalLayout_13->addWidget(doubleSpinBox_Xmove);

        label_25 = new QLabel(tab_3);
        label_25->setObjectName(QString::fromUtf8("label_25"));

        horizontalLayout_13->addWidget(label_25);

        doubleSpinBox_Ymove = new QDoubleSpinBox(tab_3);
        doubleSpinBox_Ymove->setObjectName(QString::fromUtf8("doubleSpinBox_Ymove"));
        doubleSpinBox_Ymove->setFont(font2);
        doubleSpinBox_Ymove->setMinimum(-200.000000000000000);
        doubleSpinBox_Ymove->setValue(0.000000000000000);

        horizontalLayout_13->addWidget(doubleSpinBox_Ymove);

        label_26 = new QLabel(tab_3);
        label_26->setObjectName(QString::fromUtf8("label_26"));

        horizontalLayout_13->addWidget(label_26);

        doubleSpinBox_Zmove = new QDoubleSpinBox(tab_3);
        doubleSpinBox_Zmove->setObjectName(QString::fromUtf8("doubleSpinBox_Zmove"));
        doubleSpinBox_Zmove->setFont(font2);
        doubleSpinBox_Zmove->setMinimum(-200.000000000000000);
        doubleSpinBox_Zmove->setMaximum(200.000000000000000);

        horizontalLayout_13->addWidget(doubleSpinBox_Zmove);


        verticalLayout_4->addLayout(horizontalLayout_13);

        horizontalLayout_17 = new QHBoxLayout();
        horizontalLayout_17->setSpacing(6);
        horizontalLayout_17->setObjectName(QString::fromUtf8("horizontalLayout_17"));
        pushButton_readGcodeSourceData = new QPushButton(tab_3);
        pushButton_readGcodeSourceData->setObjectName(QString::fromUtf8("pushButton_readGcodeSourceData"));
        pushButton_readGcodeSourceData->setFont(font);

        horizontalLayout_17->addWidget(pushButton_readGcodeSourceData);

        label_30 = new QLabel(tab_3);
        label_30->setObjectName(QString::fromUtf8("label_30"));
        label_30->setFont(font2);

        horizontalLayout_17->addWidget(label_30);

        lineEdit_SorceDataDir = new QLineEdit(tab_3);
        lineEdit_SorceDataDir->setObjectName(QString::fromUtf8("lineEdit_SorceDataDir"));
        lineEdit_SorceDataDir->setFont(font);

        horizontalLayout_17->addWidget(lineEdit_SorceDataDir);


        verticalLayout_4->addLayout(horizontalLayout_17);

        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setSpacing(6);
        horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
        label_27 = new QLabel(tab_3);
        label_27->setObjectName(QString::fromUtf8("label_27"));

        horizontalLayout_14->addWidget(label_27);

        label_28 = new QLabel(tab_3);
        label_28->setObjectName(QString::fromUtf8("label_28"));
        label_28->setFont(font2);

        horizontalLayout_14->addWidget(label_28);

        spinBox_StartLayerIndex = new QSpinBox(tab_3);
        spinBox_StartLayerIndex->setObjectName(QString::fromUtf8("spinBox_StartLayerIndex"));
        spinBox_StartLayerIndex->setFont(font2);

        horizontalLayout_14->addWidget(spinBox_StartLayerIndex);

        label_29 = new QLabel(tab_3);
        label_29->setObjectName(QString::fromUtf8("label_29"));
        label_29->setFont(font2);

        horizontalLayout_14->addWidget(label_29);

        spinBox_ShowLayerIndex_2 = new QSpinBox(tab_3);
        spinBox_ShowLayerIndex_2->setObjectName(QString::fromUtf8("spinBox_ShowLayerIndex_2"));
        spinBox_ShowLayerIndex_2->setFont(font2);

        horizontalLayout_14->addWidget(spinBox_ShowLayerIndex_2);

        checkBox_EachLayerSwitch_2 = new QCheckBox(tab_3);
        checkBox_EachLayerSwitch_2->setObjectName(QString::fromUtf8("checkBox_EachLayerSwitch_2"));
        checkBox_EachLayerSwitch_2->setFont(font2);

        horizontalLayout_14->addWidget(checkBox_EachLayerSwitch_2);


        verticalLayout_4->addLayout(horizontalLayout_14);

        horizontalLayout_15 = new QHBoxLayout();
        horizontalLayout_15->setSpacing(6);
        horizontalLayout_15->setObjectName(QString::fromUtf8("horizontalLayout_15"));
        pushButton_ShowAllLayersORToolpathes = new QPushButton(tab_3);
        pushButton_ShowAllLayersORToolpathes->setObjectName(QString::fromUtf8("pushButton_ShowAllLayersORToolpathes"));
        pushButton_ShowAllLayersORToolpathes->setFont(font2);

        horizontalLayout_15->addWidget(pushButton_ShowAllLayersORToolpathes);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_15->addItem(horizontalSpacer_2);

        label_currentFile = new QLabel(tab_3);
        label_currentFile->setObjectName(QString::fromUtf8("label_currentFile"));
        label_currentFile->setFont(font2);

        horizontalLayout_15->addWidget(label_currentFile);


        verticalLayout_4->addLayout(horizontalLayout_15);

        line_8 = new QFrame(tab_3);
        line_8->setObjectName(QString::fromUtf8("line_8"));
        line_8->setFrameShape(QFrame::HLine);
        line_8->setFrameShadow(QFrame::Sunken);

        verticalLayout_4->addWidget(line_8);

        horizontalLayout_16 = new QHBoxLayout();
        horizontalLayout_16->setSpacing(6);
        horizontalLayout_16->setObjectName(QString::fromUtf8("horizontalLayout_16"));
        label_31 = new QLabel(tab_3);
        label_31->setObjectName(QString::fromUtf8("label_31"));
        label_31->setFont(font);

        horizontalLayout_16->addWidget(label_31);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_16->addItem(horizontalSpacer_3);

        label_32 = new QLabel(tab_3);
        label_32->setObjectName(QString::fromUtf8("label_32"));
        label_32->setFont(font2);

        horizontalLayout_16->addWidget(label_32);

        spinBox_GcodeGeneFromIndex = new QSpinBox(tab_3);
        spinBox_GcodeGeneFromIndex->setObjectName(QString::fromUtf8("spinBox_GcodeGeneFromIndex"));
        spinBox_GcodeGeneFromIndex->setFont(font2);

        horizontalLayout_16->addWidget(spinBox_GcodeGeneFromIndex);

        label_33 = new QLabel(tab_3);
        label_33->setObjectName(QString::fromUtf8("label_33"));
        label_33->setFont(font2);

        horizontalLayout_16->addWidget(label_33);

        spinBox_GcodeGeneToIndex = new QSpinBox(tab_3);
        spinBox_GcodeGeneToIndex->setObjectName(QString::fromUtf8("spinBox_GcodeGeneToIndex"));
        spinBox_GcodeGeneToIndex->setFont(font2);

        horizontalLayout_16->addWidget(spinBox_GcodeGeneToIndex);


        verticalLayout_4->addLayout(horizontalLayout_16);

        line_9 = new QFrame(tab_3);
        line_9->setObjectName(QString::fromUtf8("line_9"));
        line_9->setFrameShape(QFrame::HLine);
        line_9->setFrameShadow(QFrame::Sunken);

        verticalLayout_4->addWidget(line_9);

        horizontalLayout_18 = new QHBoxLayout();
        horizontalLayout_18->setSpacing(6);
        horizontalLayout_18->setObjectName(QString::fromUtf8("horizontalLayout_18"));
        label_34 = new QLabel(tab_3);
        label_34->setObjectName(QString::fromUtf8("label_34"));

        horizontalLayout_18->addWidget(label_34);

        checkBox_varyDistance = new QCheckBox(tab_3);
        checkBox_varyDistance->setObjectName(QString::fromUtf8("checkBox_varyDistance"));
        checkBox_varyDistance->setFont(font2);
        checkBox_varyDistance->setChecked(true);

        horizontalLayout_18->addWidget(checkBox_varyDistance);

        checkBox_varyHeight = new QCheckBox(tab_3);
        checkBox_varyHeight->setObjectName(QString::fromUtf8("checkBox_varyHeight"));
        checkBox_varyHeight->setFont(font2);
        checkBox_varyHeight->setChecked(true);

        horizontalLayout_18->addWidget(checkBox_varyHeight);

        checkBox_varyWidth = new QCheckBox(tab_3);
        checkBox_varyWidth->setObjectName(QString::fromUtf8("checkBox_varyWidth"));
        checkBox_varyWidth->setFont(font2);
        checkBox_varyWidth->setChecked(false);

        horizontalLayout_18->addWidget(checkBox_varyWidth);

        checkBox_TestDHW_Switch = new QCheckBox(tab_3);
        checkBox_TestDHW_Switch->setObjectName(QString::fromUtf8("checkBox_TestDHW_Switch"));
        checkBox_TestDHW_Switch->setFont(font2);
        checkBox_TestDHW_Switch->setChecked(true);

        horizontalLayout_18->addWidget(checkBox_TestDHW_Switch);


        verticalLayout_4->addLayout(horizontalLayout_18);

        pushButton_calDWH = new QPushButton(tab_3);
        pushButton_calDWH->setObjectName(QString::fromUtf8("pushButton_calDWH"));

        verticalLayout_4->addWidget(pushButton_calDWH);

        horizontalLayout_19 = new QHBoxLayout();
        horizontalLayout_19->setSpacing(6);
        horizontalLayout_19->setObjectName(QString::fromUtf8("horizontalLayout_19"));
        label_35 = new QLabel(tab_3);
        label_35->setObjectName(QString::fromUtf8("label_35"));

        horizontalLayout_19->addWidget(label_35);

        doubleSpinBox_lambda = new QDoubleSpinBox(tab_3);
        doubleSpinBox_lambda->setObjectName(QString::fromUtf8("doubleSpinBox_lambda"));
        doubleSpinBox_lambda->setValue(6.000000000000000);

        horizontalLayout_19->addWidget(doubleSpinBox_lambda);


        verticalLayout_4->addLayout(horizontalLayout_19);

        pushButton_calSingularOpt = new QPushButton(tab_3);
        pushButton_calSingularOpt->setObjectName(QString::fromUtf8("pushButton_calSingularOpt"));

        verticalLayout_4->addWidget(pushButton_calSingularOpt);

        horizontalLayout_20 = new QHBoxLayout();
        horizontalLayout_20->setSpacing(6);
        horizontalLayout_20->setObjectName(QString::fromUtf8("horizontalLayout_20"));
        checkBox_showSingularityNode = new QCheckBox(tab_3);
        checkBox_showSingularityNode->setObjectName(QString::fromUtf8("checkBox_showSingularityNode"));
        checkBox_showSingularityNode->setFont(font2);

        horizontalLayout_20->addWidget(checkBox_showSingularityNode);

        checkBox_showSolveSelection = new QCheckBox(tab_3);
        checkBox_showSolveSelection->setObjectName(QString::fromUtf8("checkBox_showSolveSelection"));
        checkBox_showSolveSelection->setFont(font2);

        horizontalLayout_20->addWidget(checkBox_showSolveSelection);


        verticalLayout_4->addLayout(horizontalLayout_20);

        pushButton_calCollision = new QPushButton(tab_3);
        pushButton_calCollision->setObjectName(QString::fromUtf8("pushButton_calCollision"));

        verticalLayout_4->addWidget(pushButton_calCollision);

        pushButton_calCollisionElimination = new QPushButton(tab_3);
        pushButton_calCollisionElimination->setObjectName(QString::fromUtf8("pushButton_calCollisionElimination"));
        pushButton_calCollisionElimination->setEnabled(true);

        verticalLayout_4->addWidget(pushButton_calCollisionElimination);

        pushButton_Gcode_writting = new QPushButton(tab_3);
        pushButton_Gcode_writting->setObjectName(QString::fromUtf8("pushButton_Gcode_writting"));

        verticalLayout_4->addWidget(pushButton_Gcode_writting);

        horizontalLayout_21 = new QHBoxLayout();
        horizontalLayout_21->setSpacing(6);
        horizontalLayout_21->setObjectName(QString::fromUtf8("horizontalLayout_21"));
        pushButton_GcodeSimulation = new QPushButton(tab_3);
        pushButton_GcodeSimulation->setObjectName(QString::fromUtf8("pushButton_GcodeSimulation"));
        pushButton_GcodeSimulation->setMinimumSize(QSize(0, 0));

        horizontalLayout_21->addWidget(pushButton_GcodeSimulation);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_21->addItem(horizontalSpacer_4);

        checkBox_showCNC = new QCheckBox(tab_3);
        checkBox_showCNC->setObjectName(QString::fromUtf8("checkBox_showCNC"));
        checkBox_showCNC->setFont(font2);

        horizontalLayout_21->addWidget(checkBox_showCNC);


        verticalLayout_4->addLayout(horizontalLayout_21);

        horizontalLayout_22 = new QHBoxLayout();
        horizontalLayout_22->setSpacing(6);
        horizontalLayout_22->setObjectName(QString::fromUtf8("horizontalLayout_22"));
        progressBar_GcodeSimulation = new QProgressBar(tab_3);
        progressBar_GcodeSimulation->setObjectName(QString::fromUtf8("progressBar_GcodeSimulation"));
        progressBar_GcodeSimulation->setMinimumSize(QSize(90, 0));
        progressBar_GcodeSimulation->setValue(0);

        horizontalLayout_22->addWidget(progressBar_GcodeSimulation);

        checkBox_stopSimulation = new QCheckBox(tab_3);
        checkBox_stopSimulation->setObjectName(QString::fromUtf8("checkBox_stopSimulation"));
        checkBox_stopSimulation->setFont(font2);

        horizontalLayout_22->addWidget(checkBox_stopSimulation);


        verticalLayout_4->addLayout(horizontalLayout_22);

        line_10 = new QFrame(tab_3);
        line_10->setObjectName(QString::fromUtf8("line_10"));
        line_10->setFrameShape(QFrame::HLine);
        line_10->setFrameShadow(QFrame::Sunken);

        verticalLayout_4->addWidget(line_10);

        pushButton_calDist_scanAndmodel = new QPushButton(tab_3);
        pushButton_calDist_scanAndmodel->setObjectName(QString::fromUtf8("pushButton_calDist_scanAndmodel"));

        verticalLayout_4->addWidget(pushButton_calDist_scanAndmodel);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer);

        tabWidget->addTab(tab_3, QString());

        verticalLayout->addWidget(tabWidget);

        treeView = new QTreeView(dockWidgetContents);
        treeView->setObjectName(QString::fromUtf8("treeView"));
        treeView->setEnabled(true);
        treeView->setProperty("showDropIndicator", QVariant(true));
        treeView->setIndentation(5);
        treeView->header()->setVisible(false);

        verticalLayout->addWidget(treeView);

        pushButton_clearAll = new QPushButton(dockWidgetContents);
        pushButton_clearAll->setObjectName(QString::fromUtf8("pushButton_clearAll"));

        verticalLayout->addWidget(pushButton_clearAll);

        dockWidget->setWidget(dockWidgetContents);
        MainWindow->addDockWidget(static_cast<Qt::DockWidgetArea>(2), dockWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1300, 26));
        menuBar->setLayoutDirection(Qt::LeftToRight);
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuView = new QMenu(menuBar);
        menuView->setObjectName(QString::fromUtf8("menuView"));
        menuSelect = new QMenu(menuBar);
        menuSelect->setObjectName(QString::fromUtf8("menuSelect"));
        MainWindow->setMenuBar(menuBar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        toolBar->setMovable(false);
        toolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);

        navigationToolBar->addAction(actionFront);
        navigationToolBar->addAction(actionBack);
        navigationToolBar->addAction(actionTop);
        navigationToolBar->addAction(actionBottom);
        navigationToolBar->addAction(actionLeft);
        navigationToolBar->addAction(actionRight);
        navigationToolBar->addAction(actionIsometric);
        navigationToolBar->addSeparator();
        navigationToolBar->addAction(actionZoom_In);
        navigationToolBar->addAction(actionZoom_Out);
        navigationToolBar->addAction(actionZoom_All);
        navigationToolBar->addAction(actionZoom_Window);
        navigationToolBar->addSeparator();
        navigationToolBar->addAction(actionShade);
        navigationToolBar->addAction(actionMesh);
        navigationToolBar->addAction(actionNode);
        navigationToolBar->addAction(actionProfile);
        navigationToolBar->addAction(actionFaceNormal);
        navigationToolBar->addAction(actionNodeNormal);
        selectionToolBar->addAction(actionSaveSelection);
        selectionToolBar->addAction(actionReadSelection);
        selectionToolBar->addSeparator();
        selectionToolBar->addAction(actionSelectNode);
        selectionToolBar->addAction(actionSelectEdge);
        selectionToolBar->addAction(actionSelectFace);
        selectionToolBar->addAction(actionSelectFix);
        selectionToolBar->addAction(actionSelectHandle);
        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuView->menuAction());
        menuBar->addAction(menuSelect->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionSave);
        menuFile->addAction(actionSaveSelection);
        menuFile->addAction(actionReadSelection);
        menuView->addAction(actionFront);
        menuView->addAction(actionBack);
        menuView->addAction(actionTop);
        menuView->addAction(actionBottom);
        menuView->addAction(actionLeft);
        menuView->addAction(actionRight);
        menuView->addAction(actionIsometric);
        menuView->addSeparator();
        menuView->addAction(actionZoom_In);
        menuView->addAction(actionZoom_Out);
        menuView->addAction(actionZoom_All);
        menuView->addAction(actionZoom_Window);
        menuView->addSeparator();
        menuView->addAction(actionShade);
        menuView->addAction(actionMesh);
        menuView->addAction(actionNode);
        menuView->addAction(actionProfile);
        menuView->addSeparator();
        menuView->addAction(actionShifttoOrigin);
        menuSelect->addAction(actionSelectNode);
        menuSelect->addAction(actionSelectEdge);
        menuSelect->addAction(actionSelectFace);
        menuSelect->addSeparator();
        menuSelect->addAction(actionSelectFix);
        menuSelect->addAction(actionSelectHandle);
        menuSelect->addSeparator();
        toolBar->addAction(actionOpen);
        toolBar->addAction(actionSave);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        actionOpen->setText(QApplication::translate("MainWindow", "Open", nullptr));
        actionFront->setText(QApplication::translate("MainWindow", "Front", nullptr));
        actionBack->setText(QApplication::translate("MainWindow", "Back", nullptr));
        actionTop->setText(QApplication::translate("MainWindow", "Top", nullptr));
        actionBottom->setText(QApplication::translate("MainWindow", "Bottom", nullptr));
        actionLeft->setText(QApplication::translate("MainWindow", "Left", nullptr));
        actionRight->setText(QApplication::translate("MainWindow", "Right", nullptr));
        actionIsometric->setText(QApplication::translate("MainWindow", "Isometric", nullptr));
        actionZoom_In->setText(QApplication::translate("MainWindow", "Zoom In", nullptr));
        actionZoom_Out->setText(QApplication::translate("MainWindow", "Zoom Out", nullptr));
        actionZoom_All->setText(QApplication::translate("MainWindow", "Zoom All", nullptr));
        actionZoom_Window->setText(QApplication::translate("MainWindow", "Zoom Window", nullptr));
        actionShade->setText(QApplication::translate("MainWindow", "Shade", nullptr));
        actionMesh->setText(QApplication::translate("MainWindow", "Mesh", nullptr));
        actionNode->setText(QApplication::translate("MainWindow", "Node", nullptr));
        actionSave->setText(QApplication::translate("MainWindow", "Save", nullptr));
        actionSelectNode->setText(QApplication::translate("MainWindow", "Node", nullptr));
        actionSelectFace->setText(QApplication::translate("MainWindow", "Face", nullptr));
        actionShifttoOrigin->setText(QApplication::translate("MainWindow", "Shift to Origin", nullptr));
        actionProfile->setText(QApplication::translate("MainWindow", "Profile", nullptr));
        actionFaceNormal->setText(QApplication::translate("MainWindow", "FaceNormal", nullptr));
        actionNodeNormal->setText(QApplication::translate("MainWindow", "NodeNormal", nullptr));
        actionSelectEdge->setText(QApplication::translate("MainWindow", "Edge", nullptr));
        actionGenerate->setText(QApplication::translate("MainWindow", "Generate", nullptr));
        actionTest_1->setText(QApplication::translate("MainWindow", "Test_1", nullptr));
        actionSelectFix->setText(QApplication::translate("MainWindow", "Fix", nullptr));
        actionSelectHandle->setText(QApplication::translate("MainWindow", "Handle & Rigid", nullptr));
        actionSaveSelection->setText(QApplication::translate("MainWindow", "Save selection", nullptr));
        actionReadSelection->setText(QApplication::translate("MainWindow", "Read selection", nullptr));
        actionSelectChamber->setText(QApplication::translate("MainWindow", "Select Chamber (SORO)", nullptr));
        actionExport_to_Abaqus_model->setText(QApplication::translate("MainWindow", "Export to Abaqus model", nullptr));
        navigationToolBar->setWindowTitle(QApplication::translate("MainWindow", "navigationToolBar", nullptr));
        selectionToolBar->setWindowTitle(QApplication::translate("MainWindow", "selectionToolBar", nullptr));
        label_S3_DeformFDM->setText(QApplication::translate("MainWindow", "S^3 DeformFDM", nullptr));
        label->setText(QApplication::translate("MainWindow", "Parameter:", nullptr));
        boxDeselect->setText(QApplication::translate("MainWindow", "Deselect", nullptr));
        label_46->setText(QApplication::translate("MainWindow", "LoopTime:", nullptr));
        label_47->setText(QApplication::translate("MainWindow", "Outer", nullptr));
        label_48->setText(QApplication::translate("MainWindow", "Inner", nullptr));
        label_49->setText(QApplication::translate("MainWindow", "Weight:", nullptr));
        label_50->setText(QApplication::translate("MainWindow", "W_sl", nullptr));
        label_51->setText(QApplication::translate("MainWindow", "W_sr", nullptr));
        label_52->setText(QApplication::translate("MainWindow", "Weight:", nullptr));
        label_2->setText(QApplication::translate("MainWindow", "W_sqc", nullptr));
        label_53->setText(QApplication::translate("MainWindow", "W_sqv", nullptr));
        label_55->setText(QApplication::translate("MainWindow", "Wn", nullptr));
        label_56->setText(QApplication::translate("MainWindow", "Wr", nullptr));
        label_57->setText(QApplication::translate("MainWindow", "Ws", nullptr));
        label_8->setText(QApplication::translate("MainWindow", "Overhang angle", nullptr));
        pushButton_calBestOrientation->setText(QApplication::translate("MainWindow", "Get Best Posture", nullptr));
        checkBox_computeBestOrientation->setText(QApplication::translate("MainWindow", "compute", nullptr));
        pushButton_s3DeformFDM_supportLess_ASAP->setText(QApplication::translate("MainWindow", "1.1 Support-less ASAP", nullptr));
        label_10->setText(QApplication::translate("MainWindow", "Ratio:", nullptr));
        label_44->setText(QApplication::translate("MainWindow", "tensile", nullptr));
        label_11->setText(QApplication::translate("MainWindow", "compress", nullptr));
        pushButton_s3DeformFDM_strengthReinforcement_ASAP->setText(QApplication::translate("MainWindow", "1.2 Strength-reinforcement ASAP", nullptr));
        pushButton_s3DeformFDM_surfaceQuality_ASAP->setText(QApplication::translate("MainWindow", "1.3 Surface-quality ASAP", nullptr));
        label_4->setText(QApplication::translate("MainWindow", "Hybrid cases", nullptr));
        label_6->setText(QApplication::translate("MainWindow", "Wk_sl", nullptr));
        label_58->setText(QApplication::translate("MainWindow", "Wk_sr", nullptr));
        label_5->setText(QApplication::translate("MainWindow", "Wk_sqc", nullptr));
        label_54->setText(QApplication::translate("MainWindow", "Wk_sqv", nullptr));
        pushButton_s3DeformFDM_hybrid_SL_SQ_ASAP->setText(QApplication::translate("MainWindow", "1.4 SL_SQ", nullptr));
        pushButton_s3DeformFDM_hybrid_SR_SQ_ASAP->setText(QApplication::translate("MainWindow", "1.5 SR_SQ", nullptr));
        pushButton_s3DeformFDM_hybrid_SL_SR_ASAP->setText(QApplication::translate("MainWindow", "6. SL_SR", nullptr));
        pushButton_s3DeformFDM_hybrid_SL_SR_SQ_ASAP->setText(QApplication::translate("MainWindow", "1.7 SL_SR_SQ", nullptr));
        pushButton_s3DeformFDM_inverseDeform->setText(QApplication::translate("MainWindow", "2. Inverse Deformation", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "Deformation", nullptr));
        label_7->setText(QApplication::translate("MainWindow", "TET", nullptr));
        label_14->setText(QApplication::translate("MainWindow", "Cross-Section", nullptr));
        comboBox_planeDir->setItemText(0, QApplication::translate("MainWindow", "X plane", nullptr));
        comboBox_planeDir->setItemText(1, QApplication::translate("MainWindow", "Y plane", nullptr));
        comboBox_planeDir->setItemText(2, QApplication::translate("MainWindow", "Z plane", nullptr));

        comboBox_planeDir->setCurrentText(QApplication::translate("MainWindow", "X plane", nullptr));
        pushButton_drawScaleValue->setText(QApplication::translate("MainWindow", "Show Tetra Scale", nullptr));
        comboBox_scaleValueDirection->setItemText(0, QApplication::translate("MainWindow", "None", nullptr));
        comboBox_scaleValueDirection->setItemText(1, QApplication::translate("MainWindow", "X direction", nullptr));
        comboBox_scaleValueDirection->setItemText(2, QApplication::translate("MainWindow", "Y direction", nullptr));
        comboBox_scaleValueDirection->setItemText(3, QApplication::translate("MainWindow", "Z direction", nullptr));
        comboBox_scaleValueDirection->setItemText(4, QApplication::translate("MainWindow", "Average", nullptr));

        label_9->setText(QApplication::translate("MainWindow", "LAYER", nullptr));
        label_12->setText(QApplication::translate("MainWindow", "Display", nullptr));
        checkBox_EachLayerSwitch->setText(QApplication::translate("MainWindow", "Single", nullptr));
        pushButton_ShowAllLayers->setText(QApplication::translate("MainWindow", "All", nullptr));
        radioButton_compatibleLayer->setText(QApplication::translate("MainWindow", "Compatible Layer", nullptr));
        label_3->setText(QApplication::translate("MainWindow", "Move & Rotate model", nullptr));
        label_36->setText(QApplication::translate("MainWindow", "Rot:", nullptr));
        label_39->setText(QApplication::translate("MainWindow", "phi", nullptr));
        label_40->setText(QApplication::translate("MainWindow", "theta", nullptr));
        label_41->setText(QApplication::translate("MainWindow", "Move: Height", nullptr));
        label_43->setText(QApplication::translate("MainWindow", "Y", nullptr));
        pushButton_model_move_rotation->setText(QApplication::translate("MainWindow", "Rotate / Move Model", nullptr));
        label_45->setText(QApplication::translate("MainWindow", "IO operation", nullptr));
        pushButton_tet2surface->setText(QApplication::translate("MainWindow", "Output boundary mesh of Tet", nullptr));
        pushButton_output_vectorVSobjective->setText(QApplication::translate("MainWindow", "Output vectorField VS objective", nullptr));
        pushButton_output_userWaypoints->setText(QApplication::translate("MainWindow", "Output userWaypoints", nullptr));
        pushButton_output_streeField->setText(QApplication::translate("MainWindow", "Output stress field", nullptr));
        pushButton_output2Robot->setText(QApplication::translate("MainWindow", "Output Layer/Waypoint for ROBOT", nullptr));
        pushButton_output_QvectorField->setText(QApplication::translate("MainWindow", "Output Quaternion vectorField", nullptr));
        pushButton_output_ScalarOrHeight_Field->setText(QApplication::translate("MainWindow", "Output Scalar/Height Field", nullptr));
        checkBox_scalarOrHeight_field->setText(QApplication::translate("MainWindow", "S/H", nullptr));
        pushButton_output_deformedTet->setText(QApplication::translate("MainWindow", "Output deformed tetMesh", nullptr));
        pushButton_output_discretedTet->setText(QApplication::translate("MainWindow", "Output discretedTet", nullptr));
        pushButton_output_discretedTet_obj->setText(QApplication::translate("MainWindow", "Output discreteTet obj", nullptr));
        pushButton_output_materialRot_4CAE->setText(QApplication::translate("MainWindow", "Output material rotation for CAE", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QApplication::translate("MainWindow", "Display/IO", nullptr));
        label_13->setText(QApplication::translate("MainWindow", "Slicing Layer Num", nullptr));
        pushButton_outputIsoLayerSet->setText(QApplication::translate("MainWindow", "Output", nullptr));
        pushButton_isoLayerGeneration->setText(QApplication::translate("MainWindow", "3.1.0 Layer Generation (scalar)", nullptr));
        pushButton_adaptiveHeightSlicing->setText(QApplication::translate("MainWindow", "3.1.1 Adaptive Height Slicing  ", nullptr));
        label_38->setText(QApplication::translate("MainWindow", "Collision Elimination", nullptr));
        label_37->setText(QApplication::translate("MainWindow", "Concave", nullptr));
        pushButton_collisionChecking_vector->setText(QApplication::translate("MainWindow", "detect", nullptr));
        pushButton_concaveSmooth->setText(QApplication::translate("MainWindow", " smooth", nullptr));
        pushButton_concaveRelaxLoop->setText(QApplication::translate("MainWindow", "Loop", nullptr));
        pushButton_collisionChecking->setText(QApplication::translate("MainWindow", "3.2 Collision Checking (layer)", nullptr));
        pushButton_remarkCollisionTetra->setText(QApplication::translate("MainWindow", "Re-Mark Collision", nullptr));
        pushButton_collisionAware_flattening->setText(QApplication::translate("MainWindow", "3.3 Flattening", nullptr));
        label_15->setText(QApplication::translate("MainWindow", "Support Generation", nullptr));
        label_18->setText(QApplication::translate("MainWindow", "4.0 Get Initial Guess Envelop", nullptr));
        pushButton_Comp_initialGuess_envelopSupport->setText(QApplication::translate("MainWindow", "Envelop Generation", nullptr));
        label_19->setText(QApplication::translate("MainWindow", "-> Remesh", nullptr));
        pushButton_MeshCombination->setText(QApplication::translate("MainWindow", " Mesh Combination ", nullptr));
        label_20->setText(QApplication::translate("MainWindow", "-> Voxelization", nullptr));
        pushButton_compatibleLayer_generation->setText(QApplication::translate("MainWindow", "4.1 Get Compatible Layers", nullptr));
        label_16->setText(QApplication::translate("MainWindow", "-> Remesh", nullptr));
        pushButton_markOverhangFace->setText(QApplication::translate("MainWindow", "Mark Overhang Face", nullptr));
        pushButton_remarkOverhangFace->setText(QApplication::translate("MainWindow", "Re-Mark", nullptr));
        pushButton_treeSkeletonGeneration->setText(QApplication::translate("MainWindow", "4.2 Get Tree Skeleton", nullptr));
        pushButton_slimmedSupportGeneration->setText(QApplication::translate("MainWindow", "4.3 Get Slimmed Support", nullptr));
        label_17->setText(QApplication::translate("MainWindow", "Tool-Path Generation", nullptr));
        label_22->setText(QApplication::translate("MainWindow", "Width", nullptr));
        label_21->setText(QApplication::translate("MainWindow", "Distance", nullptr));
        pushButton_toolPathGeneration->setText(QApplication::translate("MainWindow", "5.1 Get Tool-Path", nullptr));
        pushButton_outputToolpath->setText(QApplication::translate("MainWindow", "Output", nullptr));
        checkBox_outputCompatiblewaypoint->setText(QApplication::translate("MainWindow", "Compatible", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "Support", nullptr));
        label_23->setText(QApplication::translate("MainWindow", "Coordinate System", nullptr));
        checkBox_Yup2Zup->setText(QApplication::translate("MainWindow", "Yup->Zup", nullptr));
        label_42->setText(QApplication::translate("MainWindow", "Tool length (also CAD)", nullptr));
        label_24->setText(QApplication::translate("MainWindow", "X", nullptr));
        label_25->setText(QApplication::translate("MainWindow", "Y", nullptr));
        label_26->setText(QApplication::translate("MainWindow", "Z", nullptr));
        pushButton_readGcodeSourceData->setText(QApplication::translate("MainWindow", "Read Data", nullptr));
        label_30->setText(QApplication::translate("MainWindow", "from", nullptr));
        lineEdit_SorceDataDir->setText(QApplication::translate("MainWindow", "curvedBar_hybrid", nullptr));
        label_27->setText(QApplication::translate("MainWindow", "Show:", nullptr));
        label_28->setText(QApplication::translate("MainWindow", "from", nullptr));
        label_29->setText(QApplication::translate("MainWindow", "to", nullptr));
        checkBox_EachLayerSwitch_2->setText(QApplication::translate("MainWindow", "each", nullptr));
        pushButton_ShowAllLayersORToolpathes->setText(QApplication::translate("MainWindow", "ALL Layer/Toolpath", nullptr));
        label_currentFile->setText(QApplication::translate("MainWindow", "Current Layer", nullptr));
        label_31->setText(QApplication::translate("MainWindow", "Opt Compute:", nullptr));
        label_32->setText(QApplication::translate("MainWindow", "From", nullptr));
        label_33->setText(QApplication::translate("MainWindow", "To", nullptr));
        label_34->setText(QApplication::translate("MainWindow", "Filament:", nullptr));
        checkBox_varyDistance->setText(QApplication::translate("MainWindow", "D", nullptr));
        checkBox_varyHeight->setText(QApplication::translate("MainWindow", "H", nullptr));
        checkBox_varyWidth->setText(QApplication::translate("MainWindow", "W", nullptr));
        checkBox_TestDHW_Switch->setText(QApplication::translate("MainWindow", "output", nullptr));
        pushButton_calDWH->setText(QApplication::translate("MainWindow", "6.1 Filament Volume Calculation", nullptr));
        label_35->setText(QApplication::translate("MainWindow", "Singularity threshold:", nullptr));
        pushButton_calSingularOpt->setText(QApplication::translate("MainWindow", "6.2 Singularity Optimization", nullptr));
        checkBox_showSingularityNode->setText(QApplication::translate("MainWindow", "Singular Node", nullptr));
        checkBox_showSolveSelection->setText(QApplication::translate("MainWindow", "Solve Selection", nullptr));
        pushButton_calCollision->setText(QApplication::translate("MainWindow", "6.3 Collision Checking", nullptr));
        pushButton_calCollisionElimination->setText(QApplication::translate("MainWindow", "6.4 Collision Elimination", nullptr));
        pushButton_Gcode_writting->setText(QApplication::translate("MainWindow", "6.5 G Code Writing", nullptr));
        pushButton_GcodeSimulation->setText(QApplication::translate("MainWindow", "G Gode Simulation", nullptr));
        checkBox_showCNC->setText(QApplication::translate("MainWindow", "show CNC", nullptr));
        checkBox_stopSimulation->setText(QApplication::translate("MainWindow", "stop", nullptr));
        pushButton_calDist_scanAndmodel->setText(QApplication::translate("MainWindow", "Test Function", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("MainWindow", "Manufacture ", nullptr));
        pushButton_clearAll->setText(QApplication::translate("MainWindow", "Clear All", nullptr));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", nullptr));
        menuView->setTitle(QApplication::translate("MainWindow", "View", nullptr));
        menuSelect->setTitle(QApplication::translate("MainWindow", "Select", nullptr));
        toolBar->setWindowTitle(QApplication::translate("MainWindow", "toolBar", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
