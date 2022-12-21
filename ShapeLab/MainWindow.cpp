#include <QFileDialog>
#include <QtDebug>
#include <QDesktopWidget>
#include <QCoreApplication>
#include <QMimeData>
#include <QTreeView>
#include <QThread>
#include <QTimer>
#include <QDateTime>
#include <QMessageBox>
#include <QScreen>
#include <QStyleFactory>
#include <fstream>
#include <Eigen/Eigen>
#include <Eigen/PardisoSupport>
#include <dirent.h>
#include <iostream>

#include "stdafx.h"
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "../GLKLib/GLKCameraTool.h"
#include "../GLKLib/InteractiveTool.h"
#include "../GLKLib/GLKMatrixLib.h"
#include "../GLKLib/GLKGeometry.h"
#include "../QMeshLib/QMeshPatch.h"
#include "../QMeshLib/QMeshTetra.h"
#include "../QMeshLib/QMeshFace.h"
#include "../QMeshLib/QMeshEdge.h"
#include "../QMeshLib/QMeshNode.h"
#include "DeformTet.h"
#include "IsoLayerGeneration.h"
#include "toolpathGeneration.h"
#include "FileIO.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	QApplication::setStyle(QStyleFactory::create("Fusion"));

    signalMapper = new QSignalMapper(this);
    addToolBar(ui->toolBar);
    addToolBar(ui->navigationToolBar);
    addToolBar(ui->selectionToolBar);

    createTreeView();
    createActions();

    pGLK = new GLKLib();
    ui->horizontalLayout->addWidget(pGLK);
    ui->horizontalLayout->setMargin(0);
    pGLK->setFocus();

    pGLK->clear_tools();
    pGLK->set_tool(new GLKCameraTool(pGLK,ORBITPAN));
	
	//connect timer with timer function
	connect(&Gcode_timer, SIGNAL(timeout()), this, SLOT(doTimerGcodeMoving()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::createActions()
{
    // file IO
    connect(ui->actionOpen, SIGNAL(triggered(bool)), this, SLOT(open()));
    connect(ui->actionSave, SIGNAL(triggered(bool)), this, SLOT(save()));
	connect(ui->actionSaveSelection, SIGNAL(triggered(bool)), this, SLOT(saveSelection()));
	connect(ui->actionReadSelection, SIGNAL(triggered(bool)), this, SLOT(readSelection()));

    // navigation
    connect(ui->actionFront, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionBack, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionTop, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionBottom, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionLeft, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionRight, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionIsometric, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_In, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_Out, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_All, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_Window, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    signalMapper->setMapping (ui->actionFront, 0);
    signalMapper->setMapping (ui->actionBack, 1);
    signalMapper->setMapping (ui->actionTop, 2);
    signalMapper->setMapping (ui->actionBottom, 3);
    signalMapper->setMapping (ui->actionLeft, 4);
    signalMapper->setMapping (ui->actionRight, 5);
    signalMapper->setMapping (ui->actionIsometric, 6);
    signalMapper->setMapping (ui->actionZoom_In, 7);
    signalMapper->setMapping (ui->actionZoom_Out, 8);
    signalMapper->setMapping (ui->actionZoom_All, 9);
    signalMapper->setMapping (ui->actionZoom_Window, 10);

    // view
    connect(ui->actionShade, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionMesh, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionNode, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionProfile, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionFaceNormal, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionNodeNormal, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    signalMapper->setMapping (ui->actionShade, 20);
    signalMapper->setMapping (ui->actionMesh, 21);
    signalMapper->setMapping (ui->actionNode, 22);
    signalMapper->setMapping (ui->actionProfile, 23);
    signalMapper->setMapping (ui->actionFaceNormal, 24);
    signalMapper->setMapping (ui->actionNodeNormal, 25);
    ui->actionShade->setChecked(true);

    connect(ui->actionShifttoOrigin, SIGNAL(triggered(bool)), this, SLOT(shiftToOrigin()));

    // select
    connect(ui->actionSelectNode, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionSelectEdge, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionSelectFace, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionSelectFix, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionSelectHandle, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));

	signalMapper->setMapping (ui->actionSelectNode, 30);
    signalMapper->setMapping (ui->actionSelectEdge, 31);
    signalMapper->setMapping (ui->actionSelectFace, 32);
	signalMapper->setMapping(ui->actionSelectFix, 33);
	signalMapper->setMapping(ui->actionSelectHandle, 34);


    connect (signalMapper, SIGNAL(mapped(int)), this, SLOT(signalNavigation(int)));

    //For S^3 DeformFDM
   
    //Deformation
    //connect(ui->pushButton_calBestOrientation, SIGNAL(released()), this, SLOT(runBest_Orientation_calculation()));
    connect(ui->pushButton_s3DeformFDM_supportLess_ASAP, SIGNAL(released()), this, SLOT(runDeformation_supportLess_ASAP()));
    //connect(ui->pushButton_test, SIGNAL(released()), this, SLOT(runDeformation_supportLess_ASAP_test()));
    connect(ui->pushButton_s3DeformFDM_strengthReinforcement_ASAP, SIGNAL(released()), this, SLOT(runDeformation_strengthReinforcement_ASAP()));
    connect(ui->pushButton_s3DeformFDM_surfaceQuality_ASAP, SIGNAL(released()), this, SLOT(runDeformation_surfaceQuality_ASAP()));
    connect(ui->pushButton_s3DeformFDM_hybrid_SL_SQ_ASAP, SIGNAL(released()), this, SLOT(runDeformation_SL_SQ_ASAP()));
    connect(ui->pushButton_s3DeformFDM_hybrid_SR_SQ_ASAP, SIGNAL(released()), this, SLOT(runDeformation_SR_SQ_ASAP()));
    connect(ui->pushButton_s3DeformFDM_hybrid_SL_SR_ASAP, SIGNAL(released()), this, SLOT(runDeformation_SL_SR_ASAP()));
    connect(ui->pushButton_s3DeformFDM_hybrid_SL_SR_SQ_ASAP, SIGNAL(released()), this, SLOT(runDeformation_SL_SR_SQ_ASAP()));

    //connect(ui->pushButton_readStressField, SIGNAL(released()), this, SLOT(readStressField_strengthReinforcement_ASAP_stressLine()));
    //connect(ui->pushButton_deleteFEAselection, SIGNAL(released()), this, SLOT(deleteFEAselection_strengthReinforcement_ASAP_stressLine()));
    //connect(ui->pushButton_s3DeformFDM_strengthReinforcement_ASAP_stressLine, SIGNAL(released()), this, SLOT(runDeformation_strengthReinforcement_ASAP_stressLine()));
    
    connect(ui->pushButton_s3DeformFDM_inverseDeform, SIGNAL(released()), this, SLOT(inverseDeformation()));
    connect(ui->pushButton_isoLayerGeneration, SIGNAL(released()), this, SLOT(curvedLayer_Generation()));
    connect(ui->pushButton_adaptiveHeightSlicing, SIGNAL(released()), this, SLOT(adaptiveHeight_curvedLayer_Generation()));

    //File IO
    //connect(ui->pushButton_tet2surface, SIGNAL(released()), this, SLOT(run_tet2surface()));
    connect(ui->pushButton_outputIsoLayerSet, SIGNAL(released()), this, SLOT(output_IsoLayer_set()));
    connect(ui->pushButton_outputToolpath, SIGNAL(released()), this, SLOT(output_Toolpath_set()));
    //connect(ui->pushButton_output_vectorVSobjective, SIGNAL(released()), this, SLOT(output_vectorVSobjective()));
    //connect(ui->pushButton_output_userWaypoints, SIGNAL(released()), this, SLOT(output_userWaypoints()));
    //connect(ui->pushButton_output_streeField, SIGNAL(released()), this, SLOT(output_stress_field()));
    //connect(ui->pushButton_output_QvectorField, SIGNAL(released()), this, SLOT(output_Qvector_field()));
    //connect(ui->pushButton_output_ScalarOrHeight_Field, SIGNAL(released()), this, SLOT(output_ScalarOrHeight_field()));
    //connect(ui->pushButton_output_deformedTet, SIGNAL(released()), this, SLOT(output_deformedTet()));
    //connect(ui->pushButton_output_discretedTet, SIGNAL(released()), this, SLOT(output_discreteTet()));
    //connect(ui->pushButton_output_discretedTet_obj, SIGNAL(released()), this, SLOT(output_discreteTet_obj()));
    //connect(ui->pushButton_output_materialRot_4CAE, SIGNAL(released()), this, SLOT(output_materialRot_4CAE()));

    connect(ui->pushButton_calDist_scanAndmodel, SIGNAL(released()), this, SLOT(cal_Dist_scanAndmodel()));
    
	//Display
	connect(ui->spinBox_ShowLayerIndex, SIGNAL(valueChanged(int)), this, SLOT(change_isoLayer_Display()));
    connect(ui->spinBox_ShowLayerIndex_2, SIGNAL(valueChanged(int)), this, SLOT(changeWaypointDisplay()));
    connect(ui->pushButton_ShowAllLayers, SIGNAL(released()), this, SLOT(all_isoLayer_Display()));  
    connect(ui->pushButton_ShowAllLayersORToolpathes, SIGNAL(released()), this, SLOT(all_isoLayer_Display()));
    //connect(ui->horizontalSlider_slice_Multi_dir, SIGNAL(valueChanged(int)), this, SLOT(split_Show_TET_Mesh()));/* split show the tet mesh */
    //connect(ui->pushButton_drawScaleValue, SIGNAL(released()), this, SLOT(show_ScaleValue()));/* show the scale value of tet mesh */
    connect(ui->radioButton_compatibleLayer, SIGNAL(released()), this, SLOT(change_maxLayerNum_normalORcompatible()));
    connect(ui->checkBox_showSingularityNode, SIGNAL(released()), this, SLOT(special_Draw_Command()));
    connect(ui->checkBox_showSolveSelection, SIGNAL(released()), this, SLOT(special_Draw_Command()));

    //Fabrication
    //connect(ui->pushButton_model_move_rotation, SIGNAL(released()), this, SLOT(model_ROT_MOVE()));
    //connect(ui->pushButton_collisionChecking_vector, SIGNAL(released()), this, SLOT(concave_Detect_vector()));
    //connect(ui->pushButton_concaveSmooth, SIGNAL(released()), this, SLOT(concave_Smooth_vector()));
    //connect(ui->pushButton_concaveRelaxLoop, SIGNAL(released()), this, SLOT(concave_Detect_Smooth_vector()));
    //connect(ui->pushButton_collisionChecking, SIGNAL(released()), this, SLOT(collisionChecking_layer()));
    //connect(ui->pushButton_remarkCollisionTetra, SIGNAL(released()), this, SLOT(reMark_collisionVector()));
    //connect(ui->pushButton_collisionAware_flattening, SIGNAL(released()), this, SLOT(collisionAware_Flattening()));
    
    //Support Generation
    //connect(ui->pushButton_Comp_initialGuess_envelopSupport, SIGNAL(released()), this, SLOT(initial_Guess_SupportEnvelope_Generation()));
    //connect(ui->pushButton_MeshCombination, SIGNAL(released()), this, SLOT(initial_Guess_SupportEnvelope_Generation_plus()));
    //connect(ui->pushButton_compatibleLayer_generation, SIGNAL(released()), this, SLOT(compatibleLayer_Generation()));
    //connect(ui->pushButton_markOverhangFace, SIGNAL(released()), this, SLOT(mark_OverhangFace()));
    //connect(ui->pushButton_remarkOverhangFace, SIGNAL(released()), this, SLOT(reMark_OverhangFace()));
    //connect(ui->pushButton_treeSkeletonGeneration, SIGNAL(released()), this, SLOT(build_Support_Tree_Skeleton()));
    //connect(ui->pushButton_slimmedSupportGeneration, SIGNAL(released()), this, SLOT(generate_slim_supportLayer()));

    //ToolPath Generation
    //connect(ui->pushButton_toolPathGeneration, SIGNAL(released()), this, SLOT(toolPath_Generation()));
    connect(ui->pushButton_toolPathGeneration, SIGNAL(released()), this, SLOT(toolPath_hybrid_Generation()));

    //Gcode Generation
    connect(ui->pushButton_readGcodeSourceData, SIGNAL(released()), this, SLOT(readGcodeSourceData()));
    connect(ui->pushButton_calDWH, SIGNAL(released()), this, SLOT(runDHWcalculation()));
    //connect(ui->pushButton_output2Robot, SIGNAL(released()), this, SLOT(output_preProcess_waypoints_4_Robot()));
    connect(ui->pushButton_calSingularOpt, SIGNAL(released()), this, SLOT(runSingularityOpt()));
    connect(ui->pushButton_calCollision, SIGNAL(released()), this, SLOT(runCollisionCheck()));
    connect(ui->pushButton_calCollisionElimination, SIGNAL(released()), this, SLOT(runCollisionElimination()));
    connect(ui->pushButton_Gcode_writting, SIGNAL(released()), this, SLOT(runWriteGcode()));
    connect(ui->pushButton_GcodeSimulation, SIGNAL(released()), this, SLOT(runGcodeSimulation()));
}

void MainWindow::open()
{
    QString filenameStr = QFileDialog::getOpenFileName(this, tr("Open File,"), "../DataSet/TET_MODEL/", tr(""));
    QFileInfo fileInfo(filenameStr);
    QString fileSuffix = fileInfo.suffix();
    QByteArray filenameArray = filenameStr.toLatin1();
    char *filename = filenameArray.data();

    // set polygen name
    std::string strFilename(filename);
    std::size_t foundStart = strFilename.find_last_of("/");
    std::size_t foundEnd = strFilename.find_last_of(".");
    std::string modelName;
    modelName = strFilename.substr(0,foundEnd);
    modelName = modelName.substr(foundStart+1);
    
    if (QString::compare(fileSuffix,"obj") == 0){
        PolygenMesh *polygenMesh = new PolygenMesh(UNDEFINED);
        polygenMesh->ImportOBJFile(filename,modelName);
        polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
        pGLK->AddDisplayObj(polygenMesh,true);
        polygenMeshList.AddTail(polygenMesh);
    }

	else if (QString::compare(fileSuffix, "tet") == 0) {
		PolygenMesh *polygenMesh = new PolygenMesh(TET_MODEL);
		std::cout << filename << std::endl;
		std::cout << modelName << std::endl;
		polygenMesh->ImportTETFile(filename, modelName);
		polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
		pGLK->AddDisplayObj(polygenMesh, true);
		polygenMeshList.AddTail(polygenMesh);
        this->_updateFrameworkParameter();
	}

    updateTree();

    shiftToOrigin();
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::save()
{
    return;
	PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh)
		polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	if (!polygenMesh)
		return;
	QString filenameStr = QFileDialog::getSaveFileName(this, tr("OBJ File Export,"), "..", tr("OBJ(*.obj)"));
	QFileInfo fileInfo(filenameStr);
	QString fileSuffix = fileInfo.suffix();

	if (QString::compare(fileSuffix, "obj") == 0) {
		QFile exportFile(filenameStr);
		if (exportFile.open(QFile::WriteOnly | QFile::Truncate)) {
			QTextStream out(&exportFile);
			for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
				QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);
				for (GLKPOSITION posNode = patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
					QMeshNode *node = (QMeshNode*)patch->GetNodeList().GetNext(posNode);
					double xx, yy, zz;
					node->GetCoord3D(xx, yy, zz);
					float r, g, b;
					node->GetColor(r, g, b);
					out << "v " << xx << " " << yy << " " << zz << " " << node->value1 << endl;
				}
				for (GLKPOSITION posFace = patch->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
					QMeshFace *face = (QMeshFace*)patch->GetFaceList().GetNext(posFace);
					out << "f " << face->GetNodeRecordPtr(0)->GetIndexNo() << " " << face->GetNodeRecordPtr(1)->GetIndexNo() << " " << face->GetNodeRecordPtr(2)->GetIndexNo() << endl;
				}
			}
		}
		exportFile.close();
	}
}

void MainWindow::saveSelection()
{
	//printf("%s exported\n", Model->ModelName);

    PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
    if (polygenMesh == NULL || polygenMesh->meshType != TET_MODEL) { 
        std:: printf(" -- system contains no tet model, return! \n"); 
        return;
    }
	QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	std::string filename = polygenMesh->getModelName();
	const char * c = filename.c_str();
	char *cstr = new char[filename.length() + 1];
	strcpy(cstr, filename.c_str());

	const char * split = ".";
	char* p = strtok(cstr, split);

	char output_filename[256];
	strcpy(output_filename, "../DataSet/selection_file/");
	strcat(output_filename, cstr);
	char filetype[64];
	strcpy(filetype, ".txt");
	strcat(output_filename, filetype);

	ofstream nodeSelection(output_filename);
	if (!nodeSelection)
		cerr << "Sorry! We were unable to build the file NodeSelect!\n";
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *CheckNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		nodeSelection << CheckNode->GetIndexNo() << ":";
		//for the selection of fixing part
		if (CheckNode->isFixed == true) nodeSelection << "1:";
		else nodeSelection << "0:";
		//for the selection of hard part
		if (CheckNode->isHandle == true) nodeSelection << "1:" << endl;
		else nodeSelection << "0:" << endl;
	}

	nodeSelection.close();
	printf("Finish output selection \n");
}

bool MainWindow::readSelection()
{
    PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
    if (polygenMesh == NULL || polygenMesh->meshType != TET_MODEL) {
        std::printf(" -- system contains no tet model, return! \n");
        return false;
    }
	QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	std::string filename = polygenMesh->getModelName();
	const char * c = filename.c_str();

	char *cstr = new char[filename.length() + 1];
	strcpy(cstr, filename.c_str());

	const char * split = ".";
	char* p = strtok(cstr, split);

	char input_filename[256];
    strcpy(input_filename, "../DataSet/selection_file/");
	strcat(input_filename, cstr);
	char filetype[64];
	strcpy(filetype, ".txt");
	strcat(input_filename, filetype);

	ifstream nodeSelect(input_filename);
    if (!nodeSelect) {
        cerr << "Sorry! We were unable to open the selection file!\n";
        return false;
    }
	vector<int> NodeIndex(patch->GetNodeNumber()), checkNodeFixed(patch->GetNodeNumber()), checkNodeHandle(patch->GetNodeNumber());
	//string line;
	int LineIndex1 = 0;
	string sss;
	while (getline(nodeSelect, sss)){
		const char * c = sss.c_str();
		sscanf(c, "%d:%d:%d", &NodeIndex[LineIndex1], &checkNodeFixed[LineIndex1], &checkNodeHandle[LineIndex1]);
		LineIndex1++;
	}

	nodeSelect.close();
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *CheckNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		if (checkNodeFixed[CheckNode->GetIndexNo() - 1] == 1) CheckNode->isFixed = true;
		if (checkNodeHandle[CheckNode->GetIndexNo() - 1] == 1) CheckNode->isHandle = true;
	}

	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshFace* Face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
		if (Face->GetNodeRecordPtr(0)->isHandle == true &&
			Face->GetNodeRecordPtr(1)->isHandle == true &&
			Face->GetNodeRecordPtr(2)->isHandle == true)
			Face->isHandleDraw = true;
		else Face->isHandleDraw = false;

		if (Face->GetNodeRecordPtr(0)->isFixed == true &&
			Face->GetNodeRecordPtr(1)->isFixed == true &&
			Face->GetNodeRecordPtr(2)->isFixed == true)
			Face->isFixedDraw = true;
		else Face->isFixedDraw = false;
	}
	printf("Finish input selection \n");
	pGLK->refresh(true);

    return true;

}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
	//QMouseEvent *e = (QMouseEvent*)event;
	//QPoint pos = e->pos();
	//cout << "Mouse position updated" << endl;
	//double wx, wy, wz;
	//pGLK->screen_to_wcl(100.0, 100.0, wx, wy, wz);
	//ui->CorrdinateMouse->setText(QString("X = %1").arg(wx));

	//QString text;
	//text = QString("%1 X %2").arg(event->pos().x()).arg(event->pos().y());
	///** Update the info text */
	//ui->statusBar->showMessage(text);
}

void MainWindow::signalNavigation(int flag)
{
    if (flag <= 10)
        pGLK->setNavigation(flag);
    if (flag >=20 && flag <=25){
        pGLK->setViewModel(flag-20);
        switch (flag) {
        case 20:
            ui->actionShade->setChecked(pGLK->getViewModel(0));
            break;
        case 21:
            ui->actionMesh->setChecked(pGLK->getViewModel(1));
            break;
        case 22:
            ui->actionNode->setChecked(pGLK->getViewModel(2));
            break;
        case 23:
            ui->actionProfile->setChecked(pGLK->getViewModel(3));
            break;
        case 24:
            ui->actionFaceNormal->setChecked(pGLK->getViewModel(4));
            break;
        case 25:
            ui->actionNodeNormal->setChecked(pGLK->getViewModel(5));
            break;
        }
    }
    if (flag==30 || flag==31 || flag==32 || flag == 33 || flag == 34){
        InteractiveTool *tool;
        switch (flag) {
        case 30:
            tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), NODE, ui->boxDeselect->isChecked());
            break;
        case 31:
            tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), EDGE, ui->boxDeselect->isChecked());
            break;
        case 32:
            tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), FACE, ui->boxDeselect->isChecked());
            break;
		case 33:
			tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), FIX, ui->boxDeselect->isChecked());
			break;
		case 34:
			tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), NHANDLE, ui->boxDeselect->isChecked());
			break;
        }
        pGLK->set_tool(tool);
    }
}

void MainWindow::shiftToOrigin()
{
    
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasUrls())
        event->acceptProposedAction();
}

void MainWindow::dropEvent(QDropEvent *event)
{
    QString filenameStr;
    foreach (const QUrl &url, event->mimeData()->urls())
        filenameStr = url.toLocalFile();
    QByteArray filenameArray = filenameStr.toLatin1();
    char *filename = filenameArray.data();

    PolygenMesh *polygenMesh = new PolygenMesh(UNDEFINED);

    // set polygen name
    std::string strFilename(filename);
    std::size_t foundStart = strFilename.find_last_of("/");
    std::size_t foundEnd = strFilename.find_last_of(".");
    std::string modelName;
    modelName = strFilename.substr(0,foundEnd);
    modelName = modelName.substr(foundStart+1);
    int i = 0;
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr;){
        PolygenMesh *polygen = (PolygenMesh*)polygenMeshList.GetNext(pos);
        std::string name = (polygen->getModelName()).substr(0,(polygen->getModelName()).find(' '));
        if (name == modelName)
            i++;
    }
    if (i > 0)
        modelName += " "+std::to_string(i);

	QFileInfo fileInfo(filenameStr);
	QString fileSuffix = fileInfo.suffix();
	if (QString::compare(fileSuffix, "obj") == 0) {
		polygenMesh->ImportOBJFile(filename, modelName);
        polygenMesh->meshType = CNC_PRT;
	}
	else if (QString::compare(fileSuffix, "tet") == 0) {
		polygenMesh->ImportTETFile(filename, modelName);
        polygenMesh->meshType = TET_MODEL;
	}
	polygenMesh->m_bVertexNormalShading = false;	
    polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
    pGLK->AddDisplayObj(polygenMesh,true);
    polygenMeshList.AddTail(polygenMesh);
    updateTree();
}

void MainWindow::createTreeView()
{
    treeModel = new QStandardItemModel();
    ui->treeView->setModel(treeModel);
    ui->treeView->setHeaderHidden(true);
    ui->treeView->setContextMenuPolicy(Qt::CustomContextMenu);
    ui->treeView->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->treeView->expandAll();
}

void MainWindow::updateTree()
{
    treeModel->clear();
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr;){
        PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        std::string modelName_stdString = polygenMesh->getModelName();
        QString modelName = QString::fromStdString(modelName_stdString);
        QStandardItem *modelListItem = new QStandardItem(modelName);
        modelListItem->setCheckable(true);
        modelListItem->setCheckState(Qt::Checked);
        treeModel->appendRow(modelListItem);
    }
	pGLK->refresh(true);
}

PolygenMesh *MainWindow::getSelectedPolygenMesh()
{
    if (!treeModel->hasChildren())
        return nullptr;
    QModelIndex index = ui->treeView->currentIndex();
    QString selectedModelName = index.data(Qt::DisplayRole).toString();
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr;){
        PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        QString modelName = QString::fromStdString(polygenMesh->getModelName());
        if (QString::compare(selectedModelName,modelName) == 0)
            return polygenMesh;
    }
    return nullptr;
}

void MainWindow::on_pushButton_clearAll_clicked(){

    Gcode_timer.stop();

    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        pGLK->DelDisplayObj(polygenMesh);
    }
    polygenMeshList.RemoveAll();
    pGLK->ClearDisplayObjList();
    
    pGLK->refresh();
    updateTree();
}

void MainWindow::on_treeView_clicked(const QModelIndex &index)
{
    ui->treeView->currentIndex();
    QStandardItem *modelListItem = treeModel->itemFromIndex(index);
    ui->treeView->setCurrentIndex(index);
    PolygenMesh *polygenMesh = getSelectedPolygenMesh();
    if (modelListItem->checkState() == Qt::Checked)
        polygenMesh->bShow = true;
    else
        polygenMesh->bShow = false;
    pGLK->refresh(true);
}

PolygenMesh* MainWindow::_buildPolygenMesh(mesh_type type, std::string name) {

    PolygenMesh* newMesh = new PolygenMesh(type);
    newMesh->setModelName(name);
    newMesh->BuildGLList(newMesh->m_bVertexNormalShading);
    pGLK->AddDisplayObj(newMesh, true);
    polygenMeshList.AddTail(newMesh);
    updateTree();
    return newMesh;

}

PolygenMesh* MainWindow::_detectPolygenMesh(mesh_type type) {

    PolygenMesh* detectedMesh = NULL;
    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* thispolygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        if (thispolygenMesh->meshType == type) {
            detectedMesh = thispolygenMesh; break;
        }
    }
    return detectedMesh;
}

QMeshPatch* MainWindow::_detectPolygenMesh(mesh_type type, std::string patch_name) {

    PolygenMesh* detectedMesh = NULL;
    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* thispolygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);

        if (thispolygenMesh->meshType == type) {     
            detectedMesh = thispolygenMesh; 
            break;    
        }
    }

    if (detectedMesh == NULL) return NULL;

    QMeshPatch* detectedPatch = NULL;
    for (GLKPOSITION posMesh = detectedMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
        QMeshPatch* thisPatch = (QMeshPatch*)detectedMesh->GetMeshList().GetNext(posMesh);

        if (thisPatch->patchName == patch_name) {
            detectedPatch = thisPatch;
            break;
        }
    }
    return detectedPatch;
}

void MainWindow::change_isoLayer_Display() {

    bool single = ui->checkBox_EachLayerSwitch->isChecked();
    int currentLayerIndex = ui->spinBox_ShowLayerIndex->value();

    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        if (polygenMesh->meshType != CURVED_LAYER
            && polygenMesh->meshType != TOOL_PATH) continue;

        for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
            QMeshPatch* Patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);

            Patch->drawThisPatch = false;

            if (ui->radioButton_compatibleLayer->isChecked()) {
                if (single) {
                    if (Patch->compatible_layer_Index == currentLayerIndex)
                        Patch->drawThisPatch = true;
                }
                else {
                    if (Patch->compatible_layer_Index <= currentLayerIndex)
                        Patch->drawThisPatch = true;
                }           
            }
            else {

                if (single) {
                    if (Patch->GetIndexNo() == currentLayerIndex)
                        Patch->drawThisPatch = true;
                }
                else {
                    if (Patch->GetIndexNo() <= currentLayerIndex)
                        Patch->drawThisPatch = true;
                }
            }
        }
    }

    pGLK->refresh(true);
}

void MainWindow::all_isoLayer_Display() {
    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);

        if (polygenMesh->meshType != CURVED_LAYER
            && polygenMesh->meshType != TOOL_PATH) continue;

        for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
            QMeshPatch* Patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);
            Patch->drawThisPatch = true;
        }
    }
    pGLK->refresh(true);
}

void MainWindow::special_Draw_Command() {

    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);

        if (polygenMesh->meshType != TOOL_PATH) continue;

        for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
            QMeshPatch* Patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);

            Patch->drawSingularity = false;
            Patch->drawSolveSelect = false;

            if (ui->checkBox_showSingularityNode->isChecked()) {
                Patch->drawSingularity = true;
            }

            if (ui->checkBox_showSolveSelection->isChecked()) {
                Patch->drawSolveSelect = true;
            }
        }
    }
    pGLK->refresh(true);
}

void MainWindow::_updateFrameworkParameter() {

    std::string modelName = this->_detectPolygenMesh(TET_MODEL)->getModelName();

    if (modelName == "tree3")               this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 5.0, 1.0, 40.0, 32.0, 0.0, 0.0);
    else if (modelName == "fertility")      this->_setParameter(3, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 5.0, 3.0, 40.0, 35.0, 0.0, 0.0);
    else if (modelName == "ring")           this->_setParameter(2, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 5.0, 1.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "armadillo")      this->_setParameter(2, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 5.0, 40.0, 43.0, 0.0, 0.0);
    else if (modelName == "bunny")          this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 1.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "bunny_cut")      this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 1.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "tube")           this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 1.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "curved_Bar")     this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 1.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "bunnyhead2")     this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 5.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "connector4")     this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 1.0, 40.0, 32.0, 0.0, 0.0);
    else if (modelName == "letterS")        this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 5.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "topopt_new5")    this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 5.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "dragon")         this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 5.0, 40.0, 40.0, 0.0, 0.0);
    else if (modelName == "teapot1")        this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 1.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "AnkleBaseV2")    this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 1.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "wing_mirror_step1")    this->_setParameter(1, 1, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 1.0, 40.0, 45.0, 0.0, 0.0);
    else if (modelName == "bunny_cut_simplified")    this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 1.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "bunny_cut_refined")     this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 1.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "calibration")     this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 1.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "planar_calibration")     this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 1.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "yuhu_model")  this->_setParameter(1, 7, SUPPORT_LESS, 15.0, 0.0, 0.0, 0.0, 3.0, 1.0, 40.0, 30.0, 0.0, 0.0);

    else if (modelName == "bridgeSmall")    this->_setParameter(1, 7, STRENGTH_REINFORCEMENT, 0.0, 8.0, 0.0, 0.0, 3.0, 1.0, 50.0, 0.0, 0.3, 0.0);
    else if (modelName == "bridgeSmall_new")this->_setParameter(1, 7, STRENGTH_REINFORCEMENT, 0.0, 8.0, 0.0, 0.0, 3.0, 1.0, 50.0, 0.0, 0.3, 0.0);
    else if (modelName == "airbus_topopt")  this->_setParameter(1, 7, STRENGTH_REINFORCEMENT, 0.0, 8.0, 0.0, 0.0, 3.0, 1.0, 50.0, 0.0, 0.15, 0.0);
    else if (modelName == "GEbracket")      this->_setParameter(1, 7, STRENGTH_REINFORCEMENT, 0.0, 8.0, 0.0, 0.0, 3.0, 1.0, 50.0, 0.0, 0.1, 0.1);
    else if (modelName == "Shelf_Bracket")  this->_setParameter(1, 7, STRENGTH_REINFORCEMENT, 0.0, 8.0, 0.0, 0.0, 3.0, 1.0, 50.0, 0.0, 0.1, 0.1);
    else if (modelName == "topopt_new")     this->_setParameter(1, 7, STRENGTH_REINFORCEMENT, 0.0, 8.0, 0.0, 0.0, 3.0, 1.0, 50.0, 0.0, 0.1, 0.1);
    else if (modelName == "ring3")          this->_setParameter(1, 7, STRENGTH_REINFORCEMENT, 0.0, 8.0, 0.0, 0.0, 3.0, 1.0, 50.0, 0.0, 0.1, 0.0);
    else if (modelName == "YogaNew")        this->_setParameter(1, 7, STRENGTH_REINFORCEMENT, 0.0, 8.0, 0.0, 0.0, 3.0, 5.0, 50.0, 0.0, 0.05, 0.0);
    else if (modelName == "bunny_cut1")     this->_setParameter(1, 7, STRENGTH_REINFORCEMENT, 0.0, 8.0, 0.0, 0.0, 3.0, 1.0, 50.0, 0.0, 0.02, 0.02);
    else if (modelName == "bunnyhead3")     this->_setParameter(1, 7, STRENGTH_REINFORCEMENT, 0.0, 8.0, 0.0, 0.0, 3.0, 1.0, 50.0, 0.0, 0.1, 0.1);
    else if (modelName == "armadillo5")     this->_setParameter(1, 7, STRENGTH_REINFORCEMENT, 0.0, 8.0, 0.0, 0.0, 3.0, 1.0, 50.0, 0.0, 0.2, 0.0);
    else if (modelName == "connecter3DSimple")this->_setParameter(1, 1, STRENGTH_REINFORCEMENT, 0.0, 12.0, 0.0, 0.0, 4.0, 6.0, 150.0, 0.0, 0.0, 0.0);
    else if (modelName == "topopt")         this->_setParameter(1, 7, STRENGTH_REINFORCEMENT, 0.0, 8.0, 0.0, 0.0, 3.0, 1.0, 80.0, 0.0, 0.0, 0.0);
    else if (modelName == "GEbracket")      this->_setParameter(1, 7, STRENGTH_REINFORCEMENT, 0.0, 10.0, 0.0, 0.0, 3.0, 1.0, 50.0, 0.0, 0.0, 0.0);

    else if (modelName == "AnkleBaseV1")    this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 24.0, 15.0, 3.0, 0.1, 6.0, 0.0, 0.0, 0.0);
    else if (modelName == "CSquare")        this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 15.0, 15.0, 3.0, 0.7, 12.5, 0.0, 0.0, 0.0);
    else if (modelName == "dome")           this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 15.0, 15.0, 3.0, 1.0, 15.0, 0.0, 0.0, 0.0);
    else if (modelName == "ring2")          this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 15.0, 15.0, 3.0, 1.0, 18.0, 0.0, 0.0, 0.0);
    else if (modelName == "topopt_new2")    this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 15.0, 15.0, 3.0, 3.0, 15.0, 0.0, 0.0, 0.0);
    else if (modelName == "GEbracket2")     this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 15.0, 15.0, 3.0, 0.5, 16.0, 0.0, 0.0, 0.0);
    else if (modelName == "flange")         this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 15.0, 15.0, 3.0, 1.0, 15.0, 0.0, 0.0, 0.0);
    else if (modelName == "ball_test")      this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 15.0, 15.0, 3.0, 1.0, 15.0, 0.0, 0.0, 0.0);
    else if (modelName == "bunny2")         this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 15.0, 15.0, 3.0, 1.0, 15.0, 0.0, 0.0, 0.0);
    else if (modelName == "bunny_cut2")     this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 30.0, 15.0, 3.0, 1.0, 15.0, 0.0, 0.0, 0.0);//3/1
    else if (modelName == "bunny_cut_SL")   this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 15.0, 15.0, 3.0, 1.0, 15.0, 0.0, 0.0, 0.0);
    else if (modelName == "topopt_new_SR")  this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 15.0, 15.0, 3.0, 1.0, 15.0, 0.0, 0.0, 0.0);
    else if (modelName == "armadillo3")     this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 35.0, 15.0, 3.0, 1.0, 15.0, 0.0, 0.0, 0.0);
    else if (modelName == "teapot2")        this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 35.0, 15.0, 3.0, 1.0, 15.0, 0.0, 0.0, 0.0);
    else if (modelName == "wing_mirror_step3")this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 35.0, 15.0, 3.0, 1.0, 15.0, 0.0, 0.0, 0.0);
    else if (modelName == "wing_mirror_step2")this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 35.0, 15.0, 3.0, 1.0, 15.0, 0.0, 0.0, 0.0);
    else if (modelName == "wing_mirror_step4")this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 35.0, 15.0, 3.0, 1.0, 15.0, 0.0, 0.0, 0.0);
    else if (modelName == "domeL")          this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 15.0, 15.0, 3.0, 1.0, 15.0, 0.0, 0.0, 0.0);
    //else if (modelName == "wing_mirror_step1")this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 35.0, 15.0, 3.0, 1.0, 15.0, 0.0, 0.0, 0.0);
    else if (modelName == "curvedprinting") this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 15.0, 15.0, 3.0, 0.2, 15.0, 0.0, 0.0, 0.0);
    else if (modelName == "wedge") this->_setParameter(1, 7, SURFACE_QUALITY, 0.0, 0.0, 15.0, 15.0, 3.0, 0.2, 15.0, 0.0, 0.0, 0.0);

    else if (modelName == "bunny_cut3")     this->_setParameter(1, 7, HYBRID_SL_SQ, 15.0, 0.0, 35.0, 15.0, 5.0, 3.0, 40.0, 30.0, 0.0, 0.0); 
    //else if (modelName == "bunny_cut_simplified")     
    //    this->_setParameter(1, 7, HYBRID_SL_SQ, 15.0, 0.0, 35.0, 15.0, 5.0, 3.0, 40.0, 30.0, 0.0, 0.0);
    //else if (modelName == "bunny_cut_refined")     
    //    this->_setParameter(1, 7, HYBRID_SL_SQ, 15.0, 0.0, 35.0, 15.0, 5.0, 3.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "ring4")          this->_setParameter(1, 20, HYBRID_SL_SQ, 15.0, 0.0, 30.0, 8.0, 3.0, 1.0, 50.0, 30.0, 0.0, 0.0); 
    else if (modelName == "armadillo2")     this->_setParameter(1, 7, HYBRID_SL_SQ, 15.0, 0.0, 35.0, 15.0, 3.0, 1.0, 40.0, 40.0, 0.0, 0.0);
    else if (modelName == "teapot")         this->_setParameter(1, 7, HYBRID_SL_SQ, 15.0, 0.0, 30.0, 15.0, 3.0, 1.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "curved_Bar1")    this->_setParameter(1, 7, HYBRID_SL_SQ, 15.0, 0.0, 30.0, 15.0, 3.0, 1.0, 40.0, 30.0, 0.0, 0.0);
    else if (modelName == "turbine_blade")this->_setParameter(1, 7, HYBRID_SL_SQ, 15.0, 0.0, 30.0, 15.0, 3.0, 1.0, 40.0, 1.0, 0.0, 0.0);

    else if (modelName == "topopt_new3")    this->_setParameter(2, 7, HYBRID_SR_SQ, 0.0, 5.0, 40.0, 20.0, 3.0, 0.6, 10.0, 0.0, 0.1, 0.1);
    else if (modelName == "GEbracket3")     this->_setParameter(1, 7, HYBRID_SR_SQ, 0.0, 8.0, 15.0, 15.0, 3.0, 1.0, 50.0, 0.0, 0.1, 0.1);
    else if (modelName == "bunny_cut5")     this->_setParameter(1, 7, HYBRID_SR_SQ, 0.0, 8.0, 35.0, 15.0, 3.0, 1.0, 50.0, 0.0, 0.03, 0.0);
    else if (modelName == "airbus_topopt1")  this->_setParameter(1, 7, HYBRID_SR_SQ, 0.0, 8.0, 10.0, 5.0, 3.0, 1.0, 50.0, 0.0, 0.15, 0.0);

    else if (modelName == "topopt_new4")    this->_setParameter(1, 7, HYBRID_SL_SR, 15.0, 8.0, 0.0, 0.0, 3.0, 1.0, 50.0, 30.0, 0.12, 0.12);
    else if (modelName == "bunny_cut4")     this->_setParameter(2, 7, HYBRID_SL_SR, 15.0, 8.0, 0.0, 0.0, 5.0, 3.0, 40.0, 30.0, 0.04, 0.0);
    else if (modelName == "bunnyhead")      this->_setParameter(2, 7, HYBRID_SL_SR, 15.0, 8.0, 0.0, 0.0, 5.0, 3.0, 40.0, 30.0, 0.1, 0.0);
    else if (modelName == "hook")           this->_setParameter(1, 7, HYBRID_SL_SR, 15.0, 8.0, 0.0, 0.0, 5.0, 3.0, 40.0, 30.0, 0.08, 0.08);
    else if (modelName == "CSquare2")       this->_setParameter(1, 7, HYBRID_SL_SR, 15.0, 8.0, 0.0, 0.0, 3.0, 1.0, 40.0, 30.0, 0.2, 0.0);

    else if (modelName == "CSquare3")       this->_setParameter(1, 7, HYBRID_SL_SR_SQ, 35.0, 8.0, 15.0, 15.0, 3.0, 0.5, 40.0, 30.0, 0.2, 0.0);
    else if (modelName == "bunny_cut6")     this->_setParameter(1, 7, HYBRID_SL_SR_SQ, 35.0, 8.0, 30.0, 7.0, 3.0, 1.5, 40.0, 30.0, 0.04, 0.0);
    //local SL weight = 2
    else if (modelName == "armadillo4")     this->_setParameter(1, 7, HYBRID_SL_SR_SQ, 35.0, 8.0, 30.0, 15.0, 3.0, 1.0, 40.0, 40.0, 0.2, 0.0);

    else {
        std::cout << "This model is not found, using initial parameters!" << std::endl;
        this->_setParameter(1, 7, NONE, 0.0, 0.0, 0.0, 0.0, 3.0, 1.0, 4.0, 90.0, 0.0, 0.0); return;
    }

    //read pre-defined layer number
    int layerNum = 0;
    if (modelName == "AnkleBaseV1")     layerNum = 43;
    else if (modelName == "dome")       layerNum = 60;
    else if (modelName == "domeL")       layerNum = 101;
    else if (modelName == "tree3")      layerNum = 180;
    else if (modelName == "armadillo")  layerNum = 200;
    else if (modelName == "armadillo2") layerNum = 250;
    else if (modelName == "armadillo4") layerNum = 250;
    else if (modelName == "fertility")  layerNum = 150;
    else if (modelName == "airbus_topopt")  layerNum = 100;
    else if (modelName == "bunny")      layerNum = 150;
    else if (modelName == "bunny_cut" || modelName == "bunny_cut1" || modelName == "bunny_cut2")   layerNum = 180;
    else if (modelName == "bunny_cut6") layerNum = 181;
    else if (modelName == "curved_Bar") layerNum = 180;
    else if (modelName == "tube")       layerNum = 180;
    else if (modelName == "bunny_cut3") layerNum = 186;
    else if (modelName == "topopt_new3")layerNum = 99;
    else if (modelName == "teapot")     layerNum = 150;
    else if (modelName == "wing_mirror_step1")     layerNum = 70;
    else if (modelName == "wing_mirror_step3")     layerNum = 12;
    else if (modelName == "wing_mirror_step2")     layerNum = 22;
    else if (modelName == "wing_mirror_step4")     layerNum = 10;
    else if (modelName == "bridgeSmall_new")       layerNum = 180;
    else layerNum = 100;

    ui->spinBox_isoLayerNumber->setValue(layerNum);

    std::cout << "Finish inputing parameters for the given model." << std::endl;
}

//void MainWindow::_setParameter(int loopTime, S_type caseType, double criticalTet_weight,
//    double neighborScale_weight, double regularScale_weight, double globalSmooth_weight,
//    double supportFreeAngle, double tensileRegionRatio, double compressRegionRatio) {
//
//    ui->spinBox_shapeUpIterNum->setValue(loopTime);
//    ui->doubleSpinBox_criticalTETWeight->setValue(criticalTet_weight);
//    ui->doubleSpinBox_neighborScaleWeight->setValue(neighborScale_weight);
//    ui->doubleSpinBox_regularScaleWeight->setValue(regularScale_weight);
//    ui->doubleSpinBox_globalQuatSmoothWeight->setValue(globalSmooth_weight);
//    ui->doubleSpinBox_supportFreeAngle->setValue(supportFreeAngle);
//    ui->doubleSpinBox_tensileRegionRatio->setValue(tensileRegionRatio);
//    ui->doubleSpinBox_compressRegionRatio->setValue(compressRegionRatio);
//
//    ui->pushButton_s3DeformFDM_supportLess_ASAP->setEnabled(false);
//    ui->pushButton_s3DeformFDM_strengthReinforcement_ASAP->setEnabled(false);
//    ui->pushButton_s3DeformFDM_surfaceQuality_ASAP->setEnabled(false);
//    ui->pushButton_s3DeformFDM_hybrid_SL_SQ_ASAP->setEnabled(false);
//    ui->pushButton_s3DeformFDM_hybrid_SR_SQ_ASAP->setEnabled(false);
//
//    S3_case = caseType;
//    this->_input_CNC_part();
//
//    if (caseType == SUPPORT_LESS) {
//        ui->pushButton_s3DeformFDM_supportLess_ASAP->setEnabled(true);
//        ui->label_S3_DeformFDM->setText(QString::fromStdString(("S^3 DeformFDM: SUPPORT_LESS")));
//    }
//    if (caseType == STRENGTH_REINFORCEMENT) {
//        ui->pushButton_s3DeformFDM_strengthReinforcement_ASAP->setEnabled(true);
//        ui->label_S3_DeformFDM->setText(QString::fromStdString(("S^3 DeformFDM: STRENGTH_REINFORCEMENT")));
//    }
//    if (caseType == SURFACE_QUALITY) {
//        
//        if (this->readSelection()) {
//            ui->pushButton_s3DeformFDM_surfaceQuality_ASAP->setEnabled(true);
//            ui->label_S3_DeformFDM->setText(QString::fromStdString(("S^3 DeformFDM: SURFACE_QUALITY")));
//        }
//        else {
//            std::cout << "Cannot find the selection file or TET file, please check and then restart the program.\n";
//        }
//    }
//    if (caseType == HYBRID_SL_SQ) {
//
//        if (this->readSelection()) {
//            ui->pushButton_s3DeformFDM_hybrid_SL_SQ_ASAP->setEnabled(true);
//            ui->label_S3_DeformFDM->setText(QString::fromStdString(("S^3 DeformFDM: HYBRID_SL_SQ")));
//        }
//        else {
//            std::cout << "Cannot find the selection file or TET file, please check and then restart the program.\n";
//        }
//
//    }
//
//    if (caseType == HYBRID_SR_SQ) {
//        if (this->readSelection()) {
//            ui->pushButton_s3DeformFDM_hybrid_SR_SQ_ASAP->setEnabled(true);
//            ui->label_S3_DeformFDM->setText(QString::fromStdString(("S^3 DeformFDM: HYBRID_SR_SQ")));
//        }
//        else {
//            std::cout << "Cannot find the selection file or TET file, please check and then restart the program.\n";
//        }
//    }
//}

void MainWindow::_setParameter(int outer_loopTime, int inner_loopTime, S_type caseType, 
    double criticalTet_weight_SL, double criticalTet_weight_SR, 
    double criticalTet_weight_SQC, double criticalTet_weight_SQV,
    double neighborScale_weight, double regularScale_weight, double globalSmooth_weight,
    double supportFreeAngle, double tensileRegionRatio, double compressRegionRatio) {

    ui->spinBox_shapeUpIterNum->setValue(outer_loopTime);
    ui->spinBox_innerIterNum->setValue(inner_loopTime);

    ui->doubleSpinBox_criticalTETWeight_SL->setValue(criticalTet_weight_SL);
    ui->doubleSpinBox_criticalTETWeight_SR->setValue(criticalTet_weight_SR);
    ui->doubleSpinBox_criticalTETWeight_SQC->setValue(criticalTet_weight_SQC);
    ui->doubleSpinBox_criticalTETWeight_SQV->setValue(criticalTet_weight_SQV);

    ui->doubleSpinBox_neighborScaleWeight->setValue(neighborScale_weight);
    ui->doubleSpinBox_regularScaleWeight->setValue(regularScale_weight);
    ui->doubleSpinBox_globalQuatSmoothWeight->setValue(globalSmooth_weight);

    ui->doubleSpinBox_supportFreeAngle->setValue(supportFreeAngle);
    ui->doubleSpinBox_tensileRegionRatio->setValue(tensileRegionRatio);
    ui->doubleSpinBox_compressRegionRatio->setValue(compressRegionRatio);

    ui->pushButton_s3DeformFDM_supportLess_ASAP->setEnabled(false);
    ui->pushButton_s3DeformFDM_strengthReinforcement_ASAP->setEnabled(false);
    ui->pushButton_s3DeformFDM_surfaceQuality_ASAP->setEnabled(false);
    ui->pushButton_s3DeformFDM_hybrid_SL_SQ_ASAP->setEnabled(false);
    ui->pushButton_s3DeformFDM_hybrid_SR_SQ_ASAP->setEnabled(false);
    ui->pushButton_s3DeformFDM_hybrid_SL_SR_ASAP->setEnabled(false);
    ui->pushButton_s3DeformFDM_hybrid_SL_SR_SQ_ASAP->setEnabled(false);

    S3_case = caseType;
    this->_input_CNC_part();

    if (caseType == SUPPORT_LESS) {
        ui->pushButton_s3DeformFDM_supportLess_ASAP->setEnabled(true);
        ui->label_S3_DeformFDM->setText(QString::fromStdString(("S^3 DeformFDM: SUPPORT_LESS")));
    }
    if (caseType == STRENGTH_REINFORCEMENT) {
        ui->pushButton_s3DeformFDM_strengthReinforcement_ASAP->setEnabled(true);
        ui->label_S3_DeformFDM->setText(QString::fromStdString(("S^3 DeformFDM: STRENGTH_REINFORCEMENT")));
    }
    if (caseType == SURFACE_QUALITY) {

        if (this->readSelection()) {
            ui->pushButton_s3DeformFDM_surfaceQuality_ASAP->setEnabled(true);
            ui->label_S3_DeformFDM->setText(QString::fromStdString(("S^3 DeformFDM: SURFACE_QUALITY")));
        }
        else {
            std::cout << "Cannot find the selection file or TET file, please check and then restart the program.\n";
        }
    }
    if (caseType == HYBRID_SL_SQ) {

        if (this->readSelection()) {
            ui->pushButton_s3DeformFDM_hybrid_SL_SQ_ASAP->setEnabled(true);
            ui->label_S3_DeformFDM->setText(QString::fromStdString(("S^3 DeformFDM: HYBRID_SL_SQ")));
        }
        else {
            std::cout << "Cannot find the selection file or TET file, please check and then restart the program.\n";
        }

    }

    if (caseType == HYBRID_SR_SQ) {
        if (this->readSelection()) {
            ui->pushButton_s3DeformFDM_hybrid_SR_SQ_ASAP->setEnabled(true);
            ui->label_S3_DeformFDM->setText(QString::fromStdString(("S^3 DeformFDM: HYBRID_SR_SQ")));
        }
        else {
            std::cout << "Cannot find the selection file or TET file, please check and then restart the program.\n";
        }
    }

    if (caseType == HYBRID_SL_SR) {  
        ui->pushButton_s3DeformFDM_hybrid_SL_SR_ASAP->setEnabled(true);
        ui->label_S3_DeformFDM->setText(QString::fromStdString(("S^3 DeformFDM: HYBRID_SL_SR")));
    }

    if (caseType == HYBRID_SL_SR_SQ) {
        if (this->readSelection()) {
            ui->pushButton_s3DeformFDM_hybrid_SL_SR_SQ_ASAP->setEnabled(true);
            ui->label_S3_DeformFDM->setText(QString::fromStdString(("S^3 DeformFDM: HYBRID_SL_SR_SQ")));
        }
        else {
            std::cout << "Cannot find the selection file or TET file, please check and then restart the program.\n";
        }
    }
}

void MainWindow::runBest_Orientation_calculation() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }
    
    DeformTet* ShapeUpOperator = new DeformTet();
    ShapeUpOperator->initial(tet_Model);
    ShapeUpOperator->m_supportFreeAngle = ui->doubleSpinBox_supportFreeAngle->value();
    //ShapeUpOperator->find_bestPrintingOrientation(ui->checkBox_computeBestOrientation->isChecked());
    delete ShapeUpOperator;

    std::cout << "Finish Best Printing Orientation calculation. " << std::endl;
    pGLK->refresh(true);
}

void MainWindow::run_tet2surface() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }
    FileIO* IO_operator = new FileIO();
    std::string surface_dir = "../DataSet/tempUse/";
    IO_operator->changeTet2Surface(tet_Model, surface_dir);
    delete IO_operator;

    std::cout << "Finish outputing surface mesh of the tetmesh. " << std::endl;  
}

void MainWindow::output_deformedTet() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }
    QMeshPatch* tetPatch = (QMeshPatch*)tet_Model->GetMeshList().GetHead();
    FileIO* IO_operator = new FileIO();
    std::string tet_dir = "../DataSet/tempUse/";
    IO_operator->output_TetMesh(tetPatch, tet_dir);
    delete IO_operator;

    std::cout << "Finish outputing deformed tet mesh. " << std::endl;
}

void MainWindow::output_discreteTet() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }
    QMeshPatch* tetPatch = (QMeshPatch*)tet_Model->GetMeshList().GetHead();
    FileIO* IO_operator = new FileIO();
    std::string tet_dir = "../DataSet/tempUse/";
    IO_operator->output_discreteTet_beforeBlending(tetPatch, tet_dir);
    delete IO_operator;
    std::cout << "Finish outputing discrete tet mesh. Please Clear all, and open a '_discrete.tet' file." << std::endl;
}

void MainWindow::output_discreteTet_obj() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please first input '_discrete.tet' file." << std::endl; return;
    }
    QMeshPatch* tetPatch = (QMeshPatch*)tet_Model->GetMeshList().GetHead();
    FileIO* IO_operator = new FileIO();
    std::string tet_dir = "../DataSet/tempUse/";
    IO_operator->output_discreteTet_obj_beforeBlending(tetPatch, tet_dir);
    delete IO_operator;

    pGLK->refresh(true);
    std::cout << "Finish outputing obj file of discrete tet model. " << std::endl;
}


void MainWindow::runDeformation_supportLess_ASAP() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }

    DeformTet* ShapeUpOperator = new DeformTet();
    //auto ShapeUpOperator = std::make_shared<DeformTet>;

    ShapeUpOperator->initial(tet_Model,
        ui->spinBox_shapeUpIterNum->value(),
        ui->spinBox_innerIterNum->value(),
        ui->doubleSpinBox_criticalTETWeight_SL->value(),
        ui->doubleSpinBox_criticalTETWeight_SR->value(),
        ui->doubleSpinBox_criticalTETWeight_SQC->value(),
        ui->doubleSpinBox_criticalTETWeight_SQV->value(),
        ui->doubleSpinBox_neighborScaleWeight->value(),
        ui->doubleSpinBox_regularScaleWeight->value(),
        ui->doubleSpinBox_globalQuatSmoothWeight->value(),
        SUPPORT_LESS);
    ShapeUpOperator->m_supportFreeAngle = ui->doubleSpinBox_supportFreeAngle->value();

    ShapeUpOperator->preProcess_4SupportLess(3); //different initial guess case
    //ShapeUpOperator->runASAP_SupportLess(true); // whether use initial guess
    //ShapeUpOperator->runASAP_SupportLess_test(true); // whether use initial guess
    //ShapeUpOperator->runASAP_SupportLess_test2(false); // concavity consider
    ShapeUpOperator->runASAP_SupportLess_test3(); // good enough

    ShapeUpOperator->postProcess();
    delete ShapeUpOperator;

    std::cout << "Finish SUPPORT_LESS ASAP deformation. " << std::endl;
    ui->pushButton_s3DeformFDM_inverseDeform->setEnabled(true);
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::runDeformation_strengthReinforcement_ASAP() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }

    DeformTet* ShapeUpOperator = new DeformTet();
    ShapeUpOperator->initial(tet_Model,
        ui->spinBox_shapeUpIterNum->value(),
        ui->spinBox_innerIterNum->value(),
        ui->doubleSpinBox_criticalTETWeight_SL->value(),
        ui->doubleSpinBox_criticalTETWeight_SR->value(),
        ui->doubleSpinBox_criticalTETWeight_SQC->value(),
        ui->doubleSpinBox_criticalTETWeight_SQV->value(),
        ui->doubleSpinBox_neighborScaleWeight->value(),
        ui->doubleSpinBox_regularScaleWeight->value(),
        ui->doubleSpinBox_globalQuatSmoothWeight->value(),
        STRENGTH_REINFORCEMENT);
    ShapeUpOperator->m_tensileRegionRatio = ui->doubleSpinBox_tensileRegionRatio->value();
    ShapeUpOperator->m_compressRegionRatio = ui->doubleSpinBox_compressRegionRatio->value();

    bool preProcessFinish = ShapeUpOperator->preProcess_4StrengthReinforcement();
    if (preProcessFinish) {
        //ShapeUpOperator->runASAP_StrengthReinforcement();
        ShapeUpOperator->runASAP_StrengthReinforcement_test(); //good enough
        ShapeUpOperator->postProcess();
        std::cout << "Finish strengthReinforcement ASAP deformation. " << std::endl;
        ui->pushButton_s3DeformFDM_inverseDeform->setEnabled(true);
    }
    else {
        std::cout << "Pre-process did not finish, please check the FEM file read. " << std::endl;
    }
    delete ShapeUpOperator;
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::readStressField_strengthReinforcement_ASAP_stressLine() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }

    DeformTet* ShapeUpOperator = new DeformTet();
    ShapeUpOperator->initial(tet_Model,
        ui->spinBox_shapeUpIterNum->value(),
        ui->spinBox_innerIterNum->value(),
        ui->doubleSpinBox_criticalTETWeight_SL->value(),
        ui->doubleSpinBox_criticalTETWeight_SR->value(),
        ui->doubleSpinBox_criticalTETWeight_SQC->value(),
        ui->doubleSpinBox_criticalTETWeight_SQV->value(),
        ui->doubleSpinBox_neighborScaleWeight->value(),
        ui->doubleSpinBox_regularScaleWeight->value(),
        ui->doubleSpinBox_globalQuatSmoothWeight->value(),
        STRENGTH_REINFORCEMENT);
    ShapeUpOperator->m_tensileRegionRatio = ui->doubleSpinBox_tensileRegionRatio->value();
    ShapeUpOperator->m_compressRegionRatio = ui->doubleSpinBox_compressRegionRatio->value();

    bool preProcessFinish = ShapeUpOperator->preProcess_4StrengthReinforcement_stressLine();
    if (preProcessFinish) {
        std::cout << "Pre-process finish. " << std::endl;
        ShapeUpOperator_stressLine = ShapeUpOperator;
    }
    else {
        std::cout << "Pre-process did not finish, please check the FEM file read. " << std::endl;
    }

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}


void MainWindow::deleteFEAselection_strengthReinforcement_ASAP_stressLine() {

    if (ShapeUpOperator_stressLine != NULL) {
        ShapeUpOperator_stressLine->delete_selected_ele_stress_line();
        std::cout << "delete_selected_ele_stress_line finish. " << std::endl;
    }
    else {
        std::cout << "ShapeUpOperator_stressLine == NULL, please check the FEM file read. " << std::endl;
    }

    pGLK->refresh(true);
    //pGLK->Zoom_All_in_View();
}

void MainWindow::runDeformation_strengthReinforcement_ASAP_stressLine() {

    if (ShapeUpOperator_stressLine != NULL) {

        ShapeUpOperator_stressLine->runASAP_StrengthReinforcement_stressLine();
        ShapeUpOperator_stressLine->postProcess();
        std::cout << "Finish strengthReinforcement ASAP deformation. " << std::endl;
        ui->pushButton_s3DeformFDM_inverseDeform->setEnabled(true);
    }
    else {
        std::cout << "ShapeUpOperator_stressLine == NULL, please check the FEM file read. " << std::endl;
    }

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();

}

void MainWindow::output_stress_field() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }
    FileIO* IO_operator = new FileIO();
    IO_operator->output_stressField(tet_Model);
    delete IO_operator;

    std::cout << "Finish outputing stress field. " << std::endl;
}

void MainWindow::runDeformation_surfaceQuality_ASAP() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }
    DeformTet* ShapeUpOperator = new DeformTet();
    ShapeUpOperator->initial(tet_Model,
        ui->spinBox_shapeUpIterNum->value(),
        ui->spinBox_innerIterNum->value(),
        ui->doubleSpinBox_criticalTETWeight_SL->value(),
        ui->doubleSpinBox_criticalTETWeight_SR->value(),
        ui->doubleSpinBox_criticalTETWeight_SQC->value(),
        ui->doubleSpinBox_criticalTETWeight_SQV->value(),
        ui->doubleSpinBox_neighborScaleWeight->value(),
        ui->doubleSpinBox_regularScaleWeight->value(),
        ui->doubleSpinBox_globalQuatSmoothWeight->value(),
        SURFACE_QUALITY);

    bool preProcessFinish = ShapeUpOperator->preProcess_4SurfaceQuality(true);
    if (preProcessFinish) {
        //ShapeUpOperator->runASAP_SurfaceQuality(); // get original ankleBaseV1 (user select) ^ true
        ShapeUpOperator->runASAP_SurfaceQuality_test(); // add innerLoop (user select) ^ true
        //ShapeUpOperator->runASAP_SurfaceQuality_test2(); //automatically select the kept faces ^ false
        ShapeUpOperator->postProcess();
        std::cout << "Finish SURFACE_QUALITY ASAP deformation. " << std::endl;
        ui->pushButton_s3DeformFDM_inverseDeform->setEnabled(true);
    }
    else {std::cout << "Pre-process did not finish, please check the kept region. " << std::endl;
    }
    delete ShapeUpOperator;
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::runDeformation_SL_SQ_ASAP() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }

    DeformTet* ShapeUpOperator = new DeformTet();
    //auto ShapeUpOperator = std::make_shared<DeformTet>;

    ShapeUpOperator->initial(tet_Model,
        ui->spinBox_shapeUpIterNum->value(),
        ui->spinBox_innerIterNum->value(),
        ui->doubleSpinBox_criticalTETWeight_SL->value(),
        ui->doubleSpinBox_criticalTETWeight_SR->value(),
        ui->doubleSpinBox_criticalTETWeight_SQC->value(),
        ui->doubleSpinBox_criticalTETWeight_SQV->value(),
        ui->doubleSpinBox_neighborScaleWeight->value(),
        ui->doubleSpinBox_regularScaleWeight->value(),
        ui->doubleSpinBox_globalQuatSmoothWeight->value(),
        HYBRID_SL_SQ);

    ShapeUpOperator->m_supportFreeAngle = ui->doubleSpinBox_supportFreeAngle->value();

    ShapeUpOperator->update_quaternionSmooth_weight(
        ui->doubleSpinBox_keepWeight_SL->value(),
        ui->doubleSpinBox_keepWeight_SR->value(),
        ui->doubleSpinBox_keepWeight_SQC->value(),
        ui->doubleSpinBox_keepWeight_SQV->value());

    ShapeUpOperator->preProcess_4SupportLess(3); //different initial guess case   
    ShapeUpOperator->preProcess_4SurfaceQuality(true); // true -> select critical faces // false -> all of the boundary faces
    //ShapeUpOperator->runASAP_Hybrid_SL_SQ();  // for bunny_cut3  
    ShapeUpOperator->runASAP_Hybrid_SL_SQ_test2();// real hybrid
    //ShapeUpOperator->runASAP_Hybrid_SL_SQ_test(); // 1+1 fake hybrid
    ShapeUpOperator->postProcess();
    delete ShapeUpOperator;

    std::cout << "Finish SupportLess_SurfaceQuality ASAP deformation. " << std::endl;
    ui->pushButton_s3DeformFDM_inverseDeform->setEnabled(true);
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::runDeformation_SR_SQ_ASAP() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }

    DeformTet* ShapeUpOperator = new DeformTet();
    //auto ShapeUpOperator = std::make_shared<DeformTet>;

    ShapeUpOperator->initial(tet_Model,
        ui->spinBox_shapeUpIterNum->value(),
        ui->spinBox_innerIterNum->value(),
        ui->doubleSpinBox_criticalTETWeight_SL->value(),
        ui->doubleSpinBox_criticalTETWeight_SR->value(),
        ui->doubleSpinBox_criticalTETWeight_SQC->value(),
        ui->doubleSpinBox_criticalTETWeight_SQV->value(),
        ui->doubleSpinBox_neighborScaleWeight->value(),
        ui->doubleSpinBox_regularScaleWeight->value(),
        ui->doubleSpinBox_globalQuatSmoothWeight->value(),
        HYBRID_SR_SQ);

    ShapeUpOperator->m_tensileRegionRatio = ui->doubleSpinBox_tensileRegionRatio->value();
    ShapeUpOperator->m_compressRegionRatio = ui->doubleSpinBox_compressRegionRatio->value();

    ShapeUpOperator->update_quaternionSmooth_weight(
        ui->doubleSpinBox_keepWeight_SL->value(),
        ui->doubleSpinBox_keepWeight_SR->value(),
        ui->doubleSpinBox_keepWeight_SQC->value(),
        ui->doubleSpinBox_keepWeight_SQV->value());

    bool preProcessFinish = ShapeUpOperator->preProcess_4StrengthReinforcement();
    if (preProcessFinish == false) { std::cout << "Pre-process did not finish, please check the FEM file read. " << std::endl; return; }
    ShapeUpOperator->find_bestPrintingOrientation(false);
    ShapeUpOperator->preProcess_4SurfaceQuality(true);// true -> select critical faces // false -> all of the boundary faces
    ShapeUpOperator->runASAP_Hybrid_SR_SQ(); // real hybrid

    //ShapeUpOperator->runASAP_Hybrid_SR_SQ_test();// 1+1 fake hybrid
    ShapeUpOperator->postProcess();
    delete ShapeUpOperator;

    std::cout << "Finish StrengthReinforcement_SurfaceQuality_ASAP ASAP deformation. " << std::endl;
    ui->pushButton_s3DeformFDM_inverseDeform->setEnabled(true);
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::runDeformation_SL_SR_ASAP() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }

    DeformTet* ShapeUpOperator = new DeformTet();
    //auto ShapeUpOperator = std::make_shared<DeformTet>;

    ShapeUpOperator->initial(tet_Model,
        ui->spinBox_shapeUpIterNum->value(),
        ui->spinBox_innerIterNum->value(),
        ui->doubleSpinBox_criticalTETWeight_SL->value(),
        ui->doubleSpinBox_criticalTETWeight_SR->value(),
        ui->doubleSpinBox_criticalTETWeight_SQC->value(),
        ui->doubleSpinBox_criticalTETWeight_SQV->value(),
        ui->doubleSpinBox_neighborScaleWeight->value(),
        ui->doubleSpinBox_regularScaleWeight->value(),
        ui->doubleSpinBox_globalQuatSmoothWeight->value(),
        HYBRID_SL_SR);

    ShapeUpOperator->m_supportFreeAngle = ui->doubleSpinBox_supportFreeAngle->value();

    ShapeUpOperator->m_tensileRegionRatio = ui->doubleSpinBox_tensileRegionRatio->value();
    ShapeUpOperator->m_compressRegionRatio = ui->doubleSpinBox_compressRegionRatio->value();

    ShapeUpOperator->update_quaternionSmooth_weight(
        ui->doubleSpinBox_keepWeight_SL->value(),
        ui->doubleSpinBox_keepWeight_SR->value(),
        ui->doubleSpinBox_keepWeight_SQC->value(),
        ui->doubleSpinBox_keepWeight_SQV->value());

    bool preProcessFinish = ShapeUpOperator->preProcess_4StrengthReinforcement();
    if (preProcessFinish == false) { std::cout << "Pre-process did not finish, please check the FEM file read. " << std::endl; return; }
    ShapeUpOperator->find_bestPrintingOrientation(false);
    ShapeUpOperator->preProcess_4SupportLess(3); //different initial guess case
    ShapeUpOperator->runASAP_Hybrid_SL_SR();
    ShapeUpOperator->postProcess();
    delete ShapeUpOperator;

    /*
    //debug
    //solved: isTensileorCompressSelect is added nice
    QMeshPatch* tet_patch = (QMeshPatch*)tet_Model->GetMeshList().GetHead();
    for (GLKPOSITION Pos = tet_patch->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)tet_patch->GetTetraList().GetNext(Pos);

        if (Tetra->isTensileorCompressSelect) {
            for (int i = 0; i < 4; i++) {
                Tetra->GetFaceRecordPtr(i + 1)->isSpecialShow = true;
            }
            //std::cout << "\nisTensileorCompressSelect" << std::endl;
            for (int i = 0; i < 4; i++) {

                QMeshFace* eachFace = Tetra->GetFaceRecordPtr(i + 1);

                if (eachFace->GetLeftTetra() == NULL || eachFace->GetRightTetra() == NULL) {
                    std::cout << "\n --------------------> this strength tet has boundary face" << std::endl;
                    //eachFace->isSpecialShow = true;
                    Eigen::Vector3d faceNormal;
                    Eigen::Vector3d printingDir = { 0,1,0 };
                    eachFace->CalPlaneEquation();
                    eachFace->GetNormal(faceNormal[0], faceNormal[1], faceNormal[2]);
                    double x = 30.0 * 3.1415926 / 180;
                    if(faceNormal.dot(printingDir) < sin(x) && (Tetra->isOverhangTet))
                        std::cout << "\n this strength tet has overhang face" << std::endl;
                }
            }
        }
    }
    */

    std::cout << "Finish SupportLess_StrengthReinforcement ASAP deformation. " << std::endl;
    ui->pushButton_s3DeformFDM_inverseDeform->setEnabled(true);
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::runDeformation_SL_SR_SQ_ASAP() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }

    DeformTet* ShapeUpOperator = new DeformTet();
    //auto ShapeUpOperator = std::make_shared<DeformTet>;

    ShapeUpOperator->initial(tet_Model,
        ui->spinBox_shapeUpIterNum->value(),
        ui->spinBox_innerIterNum->value(),
        ui->doubleSpinBox_criticalTETWeight_SL->value(),
        ui->doubleSpinBox_criticalTETWeight_SR->value(),
        ui->doubleSpinBox_criticalTETWeight_SQC->value(),
        ui->doubleSpinBox_criticalTETWeight_SQV->value(),
        ui->doubleSpinBox_neighborScaleWeight->value(),
        ui->doubleSpinBox_regularScaleWeight->value(),
        ui->doubleSpinBox_globalQuatSmoothWeight->value(),
        HYBRID_SL_SR_SQ);

    ShapeUpOperator->m_supportFreeAngle = ui->doubleSpinBox_supportFreeAngle->value();

    ShapeUpOperator->m_tensileRegionRatio = ui->doubleSpinBox_tensileRegionRatio->value();
    ShapeUpOperator->m_compressRegionRatio = ui->doubleSpinBox_compressRegionRatio->value();

    ShapeUpOperator->update_quaternionSmooth_weight(
        ui->doubleSpinBox_keepWeight_SL->value(),
        ui->doubleSpinBox_keepWeight_SR->value(),
        ui->doubleSpinBox_keepWeight_SQC->value(),
        ui->doubleSpinBox_keepWeight_SQV->value());

    bool preProcessFinish = ShapeUpOperator->preProcess_4StrengthReinforcement();
    if (preProcessFinish == false) { std::cout << "Pre-process did not finish, please check the FEM file read. " << std::endl; return; }
    //ShapeUpOperator->find_bestPrintingOrientation(false);
    ShapeUpOperator->preProcess_4SupportLess(3); //different initial guess case
    ShapeUpOperator->preProcess_4SurfaceQuality(true);// true -> select critical faces // false -> all of the boundary faces
    ShapeUpOperator->runASAP_Hybrid_SL_SR_SQ();
    ShapeUpOperator->postProcess();
    delete ShapeUpOperator;

    std::cout << "Finish SupportLess_StrengthReinforcement_SurfaceQuality ASAP deformation. " << std::endl;
    ui->pushButton_s3DeformFDM_inverseDeform->setEnabled(true);
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::output_vectorVSobjective(){

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }
    FileIO* IO_operator = new FileIO();
    IO_operator->outputComparison_VectorField_vs_Objective(tet_Model, S3_case);
    delete IO_operator;

    std::cout << "Finish outputing Vector Field VS objectives. " << std::endl;
}

void MainWindow::inverseDeformation() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }
    QMeshPatch* tetMesh = (QMeshPatch*)tet_Model->GetMeshList().GetHead();

    if (S3_case == SUPPORT_LESS 
        || S3_case == STRENGTH_REINFORCEMENT 
        || (S3_case == SURFACE_QUALITY 
            && (tet_Model->getModelName() != "AnkleBaseV1") 
            && (S3_case == SURFACE_QUALITY && tet_Model->getModelName() != "curvedprinting")
            && (S3_case == SURFACE_QUALITY && tet_Model->getModelName() != "wedge")
           )
        //|| (S3_case == SURFACE_QUALITY)
        || S3_case == HYBRID_SL_SQ
        || S3_case == HYBRID_SR_SQ
        || S3_case == HYBRID_SL_SR
        || S3_case == HYBRID_SL_SR_SQ
        ) {

        // Planar printing button
        bool planar_printing_Button = false;
        // resume the original position
        if (planar_printing_Button == true) {
            for (GLKPOSITION pos = tetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;) {
                QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(pos);

                double pp[3];
                for (int i = 0; i < 3; i++) { pp[i] = node->initial_coord3D[i]; }
                node->SetCoord3D(pp[0], pp[1], pp[2]);
            }
        }

        // Get height field (scalar field)
        double boundingBox[6]; double heightRange = 0.0;
        tetMesh->ComputeBoundingBox(boundingBox);
        heightRange = boundingBox[3] - boundingBox[2];
        for (GLKPOSITION pos = tetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;) {
            QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(pos);

            double xx, yy, zz;
            node->GetCoord3D(xx, yy, zz);
            node->deformed_coord3D << xx, yy, zz;
            node->scalarField_init = yy;
            node->scalarField = (yy - boundingBox[2]) / heightRange;
        }

        // resume the original position
        if (planar_printing_Button == false) {
            for (GLKPOSITION pos = tetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;) {
                QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(pos);

                double pp[3];
                for (int i = 0; i < 3; i++) { pp[i] = node->initial_coord3D[i]; }
                node->SetCoord3D(pp[0], pp[1], pp[2]);
            }
        }

        // get a smoothed scalar field (for further support generation)
        this->_scalarField_2_vectorField(true);
        //if (tet_Model->getModelName() != "AnkleBaseV1") {
        if (S3_case == SUPPORT_LESS
            || S3_case == STRENGTH_REINFORCEMENT 
            || S3_case == HYBRID_SL_SR
            || (S3_case == SURFACE_QUALITY && tet_Model->getModelName() != "AnkleBaseV1")
            //|| S3_case == HYBRID_SL_SQ
            || (S3_case == HYBRID_SL_SR_SQ && planar_printing_Button)
            ) {
            this->_vectorField_smooth(10, true);
            this->_vectorField_2_scalarField(false);
        }

        std::cout << "Use height field" << std::endl;
        
    }
    else if ((S3_case == SURFACE_QUALITY) && (tet_Model->getModelName() == "AnkleBaseV1")
        || (S3_case == SURFACE_QUALITY) && (tet_Model->getModelName() == "curvedprinting")
        || (S3_case == SURFACE_QUALITY) && (tet_Model->getModelName() == "wedge")
        ){

        // Get height field (scalar field)
        double boundingBox[6]; double heightRange = 0.0;
        tetMesh->ComputeBoundingBox(boundingBox);
        heightRange = boundingBox[3] - boundingBox[2];
        for (GLKPOSITION pos = tetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;) {
            QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(pos);

            double xx, yy, zz;
            node->GetCoord3D(xx, yy, zz);
            node->deformed_coord3D << xx, yy, zz;
            node->scalarField_init = yy;
            node->scalarField = (yy - boundingBox[2]) / heightRange;
        }

        // resume the original position
        for (GLKPOSITION pos = tetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;) {
            QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(pos);

            double pp[3];
            for (int i = 0; i < 3; i++) { pp[i] = node->initial_coord3D[i]; }
            node->SetCoord3D(pp[0], pp[1], pp[2]);
        }
        this->_compTetMeshVolumeMatrix(tetMesh);

        this->_scalarField_2_vectorField_unNormalized(tetMesh);
        this->_vectorField_2_scalarField(false);//false->local vector // get real scalar field
        // collect the node set of each critical region
        int splitTime = this->_mark_SurfaceKeep_region(tetMesh);
        // calculate the AVERAGE(MAX) scalar value of each node set and foce it to each node
        this->_fixScalarField_surfaceKeepRegion(tetMesh, splitTime);
        // compute scalar field with hard Constrain
        this->_vectorField_2_scalarField_withHardConstrain(tetMesh);
        this->_compScalarField_normalizedValue(tetMesh);
        // update unnormalized vector field
        this->_scalarField_2_vectorField_unNormalized(tetMesh);
        // re-compute scalar field with hard Constrain
        this->_vectorField_2_scalarField_withHardConstrain(tetMesh);
        this->_compScalarField_normalizedValue(tetMesh);

        
        std::cout << "Use post-process field" << std::endl;
    }
    else {
        std::cout << "this is unknown S3_case" << std::endl;
    }

    // supplementary code
    if (tet_Model->getModelName() != "AnkleBaseV1") {
        ui->pushButton_isoLayerGeneration->setEnabled(true);
    }
    else {
        ui->pushButton_adaptiveHeightSlicing->setEnabled(true);
        ui->pushButton_isoLayerGeneration->setEnabled(true);
        //ui->checkBox_outputLayer->setChecked(true);
    }
    //ui->pushButton_isoLayerGeneration->setEnabled(true);
    //ui->pushButton_output_QvectorField->setEnabled(true);
    //ui->pushButton_output_ScalarOrHeight_Field->setEnabled(true);
    tetMesh->drawStressField = false;
    std::cout << "\nFinish inverse deformation. " << std::endl;
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::output_Qvector_field() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }
    FileIO* IO_operator = new FileIO();
    IO_operator->output_Quaternion_VectorField(tet_Model);
    delete IO_operator;

    std::cout << "Finish outputing Quaternion Vector field. " << std::endl;
}

void MainWindow::output_ScalarOrHeight_field() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }
    FileIO* IO_operator = new FileIO();
    //IO_operator->output_ScalarOrHeight_field(tet_Model, ui->checkBox_scalarOrHeight_field->isChecked());
    delete IO_operator;

    std::cout << "Finish outputing Scalar/Height field. " << std::endl;
}

void MainWindow::output_materialRot_4CAE() {

    PolygenMesh* tet_Model = this->_detectPolygenMesh(TET_MODEL);
    if (tet_Model == NULL) {
        std::cerr << "There is no Tet mesh, please check." << std::endl; return;
    }
    FileIO* IO_operator = new FileIO();
    IO_operator->output_materialRotation_4CAE(tet_Model);
    delete IO_operator;

    std::cout << "Finish outputing material orientation for CAE. " << std::endl;

}

int MainWindow::_mark_SurfaceKeep_region(QMeshPatch* patch) {

    bool allNodeChecked = true;
    int splitTime = 0;

    // initial the index of surface protect node with -1;
    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* thisNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
        if (thisNode->inner) continue;

        if (thisNode->isSurfaceProtectNode)
            thisNode->surfaceKeep_region_idx = -1;
        else
            thisNode->surfaceKeep_region_idx = 0;
    }

    do {
        splitTime++;

        //-- Find start node
        for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* thisNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
            if (thisNode->inner) continue;

            if (thisNode->surfaceKeep_region_idx < 0) { thisNode->surfaceKeep_region_idx = splitTime; break; }
        }

        //-- Run flooding searching
        this->_surfaceKeepRegion_flooding(patch, splitTime);

        //-- Detect if all the node has been checked
        for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* thisNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
            if (thisNode->inner) continue;

            allNodeChecked = true;
            if (thisNode->surfaceKeep_region_idx < 0) { allNodeChecked = false; break; }
        }
    } while (!allNodeChecked);

    std::cout << "split " << splitTime << " Times." << std::endl;
    return splitTime;
}

void MainWindow::_surfaceKeepRegion_flooding(QMeshPatch* patch, int index) {

    int StopFlag = 0;

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* thisNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        if (thisNode->inner) continue;

        if (thisNode->surfaceKeep_region_idx == index) {
            for (int i = 0; i < thisNode->GetEdgeNumber(); i++) {

                QMeshEdge* thisEdge = thisNode->GetEdgeRecordPtr(i + 1);
                QMeshNode* NeighboorNode = thisEdge->GetStartPoint();
                if (thisNode == thisEdge->GetStartPoint()) NeighboorNode = thisEdge->GetEndPoint();

                if (NeighboorNode->inner || (!NeighboorNode->isSurfaceProtectNode)) continue;

                if (NeighboorNode->surfaceKeep_region_idx == index) continue;
                else {
                    NeighboorNode->surfaceKeep_region_idx = index;
                    StopFlag++;
                }
            }
        }
    }
    //cout << "This iteration adding node number = " << StopFlag << endl;

    if (StopFlag > 0) _surfaceKeepRegion_flooding(patch, index);
}

void MainWindow::_scalarField_2_vectorField_unNormalized(QMeshPatch* patch) {

    double vector_Scale_ratio = 1.0;
    // get vectorField (unNormalized)
    for (GLKPOSITION Pos = patch->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)patch->GetTetraList().GetNext(Pos);

        Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
        Eigen::VectorXd fieldValue = Eigen::VectorXd::Zero(4);

        for (int i = 0; i < 4; i++) {
            fieldValue(i) = Tetra->GetNodeRecordPtr(i + 1)->scalarField;
            for (int j = 0; j < 3; j++) {
                gradient(j) += Tetra->VolumeMatrix(i, j) * fieldValue(i);
            }
        }
        Tetra->vectorField = gradient;
        vector_Scale_ratio = 1.0 / Tetra->vectorField.norm(); // actually use the final vector length

        /*added for test face direction*/
        //if (Tetra->containProtectSurface) {
        //	QMeshFace* protectFace;
        //	for (int j = 0; j < 4; j++) {
        //		protectFace = Tetra->GetFaceRecordPtr(j + 1);
        //		if (protectFace->isQualityProtectFace) break;
        //	}
        //	protectFace->CalPlaneEquation();
        //	Eigen::Vector3d face_Normal;
        //	protectFace->GetNormal(face_Normal[0], face_Normal[1], face_Normal[2]);
        //	Tetra->vectorField = face_Normal;
        //	if(face_Normal[1]<0)	Tetra->vectorField = -face_Normal;
        //}
    }
    for (GLKPOSITION Pos = patch->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)patch->GetTetraList().GetNext(Pos);
        Tetra->vectorField *= vector_Scale_ratio;
    }
}

void MainWindow::_fixScalarField_surfaceKeepRegion(QMeshPatch* patch, int splitTime) {

    // use the init scalar value to calculate
    for (int i = 1; i <= splitTime; i++) {
        int nodeNum_eachSplit = 0;
        double average_HeightField_init = 0.0;
        double average_HeightField = 0.0;

        double max_HeightField = -INFINITE;
        for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* thisNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
            if (thisNode->inner) continue;
            if (thisNode->surfaceKeep_region_idx == i) {
                nodeNum_eachSplit++;
                average_HeightField_init += thisNode->scalarField_init;
                average_HeightField += thisNode->scalarField;
            }
            if (max_HeightField < thisNode->scalarField_init) max_HeightField = thisNode->scalarField_init;
        }

        average_HeightField_init /= nodeNum_eachSplit;
        average_HeightField /= nodeNum_eachSplit;
        std::cout << "region: " << i << ", nodeNum " << nodeNum_eachSplit
            << ", heightField init value " << average_HeightField_init
            << ", scalarField value " << average_HeightField << std::endl;

        for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* thisNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
            if (thisNode->inner) continue;
            if (thisNode->surfaceKeep_region_idx == i) {
                thisNode->scalarField_init = average_HeightField_init;
                thisNode->scalarField = average_HeightField;
            }
        }
    }
}

void MainWindow::_vectorField_2_scalarField_withHardConstrain(QMeshPatch* tetMesh) {

    int index = 0;
    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        if (Node->isSurfaceProtectNode) Node->surfaceKeepIndex = -1;
        else { Node->surfaceKeepIndex = index; index++; }
    }
    int nodeNum = index;

    index = 0;
    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

        if (Tetra->GetNodeRecordPtr(1)->surfaceKeepIndex < 0 && Tetra->GetNodeRecordPtr(2)->surfaceKeepIndex < 0 &&
            Tetra->GetNodeRecordPtr(3)->surfaceKeepIndex < 0 && Tetra->GetNodeRecordPtr(4)->surfaceKeepIndex < 0) {
            Tetra->surfaceKeepIndex = -1;
        }
        else { Tetra->surfaceKeepIndex = index; index++; }
    }

    int eleNum = index;

    std::cout << " -- none surfaceKeep element number = " << eleNum << ", "
        " none surfaceKeep node number = " << nodeNum << std::endl;

    Eigen::SparseMatrix<double> Parameter(3 * eleNum, nodeNum); //A

    Eigen::VectorXd scalarField_unNormalized(nodeNum); //x

    Eigen::VectorXd b(3 * eleNum); //b
    b.setZero();

    std::vector<Eigen::Triplet<double>> ParaTriplet;

    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

        if (Tetra->surfaceKeepIndex < 0) continue;

        double weight = 1.0;

        //bottom region should be preserved, yoga 3D model is not sutible for this!
        //if (Tetra->isBottomTet) weight = 5.0;

        for (int i = 0; i < 3; i++) b(Tetra->surfaceKeepIndex * 3 + i) = Tetra->vectorField(i) * weight; // infill B

        for (int j = 0; j < 3; j++) {
            for (int i = 0; i < 4; i++) {

                QMeshNode* Node = Tetra->GetNodeRecordPtr(i + 1);
                int nodeIndex = Node->surfaceKeepIndex;
                if (nodeIndex >= 0)
                    ParaTriplet.push_back(Eigen::Triplet<double>(
                        Tetra->surfaceKeepIndex * 3 + j, nodeIndex, -Tetra->VolumeMatrix(i, j) * weight)); // infill A
                else {
                    b(Tetra->surfaceKeepIndex * 3 + j)
                        -= Node->scalarField_init * (-Tetra->VolumeMatrix(i, j)) * weight; //constrain to B
                }
            }
        }
    }
    Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

    Eigen::SparseMatrix<double> ATA(nodeNum, nodeNum);
    ATA = Parameter.transpose() * Parameter;
    Eigen::SparseLU <Eigen::SparseMatrix<double>> Solver;

    Solver.compute(ATA);

    Eigen::VectorXd ATb(nodeNum);
    ATb = Parameter.transpose() * b;
    scalarField_unNormalized = Solver.solve(ATb); // new scalar field

    index = 0;
    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        if (Node->surfaceKeepIndex < 0) continue;
        else Node->scalarField_init = scalarField_unNormalized(Node->surfaceKeepIndex);
    }
    std::cout << " Finish compute scalar field by hard constrain method" << std::endl;
}

void MainWindow::_compScalarField_normalizedValue(QMeshPatch* tetMesh) {

    Eigen::VectorXd scalarField_unNormalized(tetMesh->GetNodeNumber());

    int index = 0;
    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);

        scalarField_unNormalized(index) = Node->scalarField_init;
        index++;
    }

    // get max and min phis
    double minPhi = scalarField_unNormalized.minCoeff();
    double maxPhi = scalarField_unNormalized.maxCoeff();
    double range = maxPhi - minPhi;
    std::cout << "min,max,range" << minPhi << " , " << maxPhi << " , " << range << std::endl;

    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        Node->scalarField = 1.0 - (Node->scalarField_init - minPhi) / range;
    }
}

void MainWindow::_compVectorField_smooth_withHardConstrain(QMeshPatch* tetMesh) {

    int index = 0;
    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

        if (Tetra->containProtectSurface) 	Tetra->surfaceKeepIndex = -1;
        else { Tetra->surfaceKeepIndex = index; index++; }

    }
    int eleNum = index;

    std::cout << " -- none surfaceKeep element number = " << eleNum << std::endl;

    //initialize matrix for global vector field smooth
    Eigen::SparseMatrix<double> Parameter(eleNum, eleNum); //A

    std::vector<Eigen::VectorXd>  vectorField_unNormalized(3); //x
    for (int i = 0; i < 3; i++) { vectorField_unNormalized[i] = Eigen::VectorXd::Zero(eleNum); }

    std::vector<Eigen::VectorXd> b(3); //b (infill B with zero)
    for (int i = 0; i < 3; i++) { b[i] = Eigen::VectorXd::Zero(eleNum); }

    std::vector<Eigen::Triplet<double>> ParaTriplet;

    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

        if (Tetra->surfaceKeepIndex < 0) continue;
        int idx = Tetra->surfaceKeepIndex;
        std::vector< QMeshTetra* > Tetra_neighbSet;
        _detect_Neighbor_Tetrahedral(Tetra_neighbSet, Tetra, false);


        ParaTriplet.push_back(Eigen::Triplet<double>(idx, idx, 1.0));
        for (int i = 0; i < Tetra_neighbSet.size(); i++) {

            if (Tetra_neighbSet[i]->surfaceKeepIndex >= 0) {
                // infill A
                ParaTriplet.push_back(Eigen::Triplet<double>(
                    idx, Tetra_neighbSet[i]->surfaceKeepIndex, -1.0 / Tetra_neighbSet.size()));
            }
            else {
                //constrain to B
                for (int j = 0; j < 3; j++) {
                    b[j](idx) = 1.0 / Tetra_neighbSet.size() * Tetra_neighbSet[i]->vectorField(j);
                }
            }
        }

    }
    Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

    Eigen::SparseMatrix<double> ATA(eleNum, eleNum);
    ATA = Parameter.transpose() * Parameter;
    Eigen::SparseLU <Eigen::SparseMatrix<double>> Solver;
    Solver.compute(ATA);

    std::vector<Eigen::VectorXd> ATb(3);
    for (int i = 0; i < 3; i++) {
        ATb[i] = Parameter.transpose() * b[i];
        vectorField_unNormalized[i] = Solver.solve(ATb[i]); // new vector field
    }

    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
        if (Tetra->surfaceKeepIndex < 0) continue;
        else
            for (int i = 0; i < 3; i++) {
                Tetra->vectorField[i] = vectorField_unNormalized[i](Tetra->surfaceKeepIndex);
            }
    }
    std::cout << " Finish compute vector field by hard constrain method" << std::endl;
}

//neighb_type == true; // use node to detect neighbor
//neighb_type == false;// use face to detect neighbor
void MainWindow::_detect_Neighbor_Tetrahedral(std::vector< QMeshTetra* >& TetraSet, 
    QMeshTetra* Tetra, bool neighb_type) {

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

void MainWindow::curvedLayer_Generation() {

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    PolygenMesh* isoLayerSet = this->_detectPolygenMesh(CURVED_LAYER);
    if (isoLayerSet == NULL) {
        isoLayerSet = this->_buildPolygenMesh(CURVED_LAYER, "isoLayers");
        std::cerr << "There is no layer set, we build a new one." << std::endl;
    }
    else {
        isoLayerSet->ClearAll();
    }

    IsoLayerGeneration* slicer = new IsoLayerGeneration(model);
    slicer->generateIsoSurface(isoLayerSet, ui->spinBox_isoLayerNumber->value());
    ui->spinBox_ShowLayerIndex->setMinimum((int)0);
    delete slicer;
    //for (int i = 0; i < 1; i++) slicer->smoothingIsoSurface(isoLayerSet);
    //slicer->outputSurfaceMesh(isoLayerSet, ui->checkBox_outputLayer->isChecked());
    

    ui->radioButton_compatibleLayer->setEnabled(false);
    ui->spinBox_ShowLayerIndex->setMaximum(isoLayerSet->GetMeshList().GetCount() - 1);
    std::cout << "Finish curved layer generation." << std::endl;
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::output_IsoLayer_set() {

    PolygenMesh* layerSet = this->_detectPolygenMesh(CURVED_LAYER);
    if (layerSet == NULL) {
        std::cerr << "There is no layer set, please check." << std::endl; return;
    }
    FileIO* IO_operator = new FileIO();
    IO_operator->outputIsoSurface(layerSet, true);
    delete IO_operator;

    std::cout << "Finish outputing surface mesh of the tetmesh. " << std::endl;
}

void MainWindow::adaptiveHeight_curvedLayer_Generation() {

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    PolygenMesh* isoLayerSet = this->_buildPolygenMesh(CURVED_LAYER, "isoLayers");
    IsoLayerGeneration* slicer = new IsoLayerGeneration(model);
    //slicer->generateIsoSurface(isoLayerSet, ui->spinBox_isoLayerNumber->value());
    slicer->generateIsoSurface_adaptiveDistanceControl(isoLayerSet, 0.4, 1.0);
    ui->spinBox_ShowLayerIndex->setMinimum((int)0);

    //for (int i = 0; i < 1; i++) slicer->smoothingIsoSurface(isoLayerSet);
    //slicer->outputSurfaceMesh(isoLayerSet, ui->checkBox_outputLayer->isChecked());
    delete slicer;

    ui->spinBox_ShowLayerIndex->setMaximum(isoLayerSet->GetMeshList().GetCount() - 1);
    std::cout << "Finish curved layer generation." << std::endl;
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::show_ScaleValue() {

    //0-NONE;1-X;2-Y;3-Z;4-Average
    int scaleValue_Direction = 0;// ui->comboBox_scaleValueDirection->currentIndex();

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh detected!" << std::endl; return; }
    QMeshPatch* patch = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    if (scaleValue_Direction == 0) {
        std::cout << "NO scale value show!" << std::endl;

        for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos;) {
            QMeshFace* face = (QMeshFace*)patch->GetTetraList().GetNext(Pos);

            face->show_scale_value = false;
        }
        pGLK->refresh(true);
        return;
    }

    for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* face = (QMeshFace*)patch->GetTetraList().GetNext(Pos);

        face->show_scale_value = true;

        QMeshTetra* leftTetra = face->GetLeftTetra();
        QMeshTetra* rightTetra = face->GetRightTetra();
        Eigen::Vector3d scaleVector = Eigen::Vector3d::Zero();
        if (leftTetra != NULL && rightTetra != NULL)
            scaleVector = (rightTetra->scaleValue_vector + leftTetra->scaleValue_vector) / 2.0;
        else if (leftTetra != NULL && rightTetra == NULL)
            scaleVector = leftTetra->scaleValue_vector;
        else if (leftTetra == NULL && rightTetra != NULL)
            scaleVector = rightTetra->scaleValue_vector;
        else
            std::printf("Error: face topology error!");

        if (scaleValue_Direction == 1) {
            face->scale_value_onFace = scaleVector[0];
        }
        else if (scaleValue_Direction == 2) {
            face->scale_value_onFace = scaleVector[1];
        }
        else if (scaleValue_Direction == 3) {
            face->scale_value_onFace = scaleVector[2];
        }
        else if (scaleValue_Direction == 4) {
            face->scale_value_onFace = scaleVector.sum() / scaleVector.size();
        }
        else { std::cout << "Error: no other case for scale show!" << std::endl; }

        //std::cout << face->scale_value_onFace << std::endl;
    }
    pGLK->refresh(true);
}

void MainWindow::split_Show_TET_Mesh() {

    //0-X;1-Y;2-Z
    int plane_Move = 0;// ui->comboBox_planeDir->currentIndex();

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh detected!" << std::endl; return; }
    QMeshPatch* patch = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    //refesh inner flag
    for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);

        if (face->GetLeftTetra() && face->GetRightTetra()) { face->inner = true; }
        else face->inner = false;

        face->show_innerFace_split = false;
    }

    //get boundary of mesh
    double xmin, ymin, zmin, xmax, ymax, zmax;
    patch->ComputeBoundingBox(xmin, ymin, zmin, xmax, ymax, zmax);
    xmin *= 1.1;	ymin *= 1.1;	zmin *= 1.1;
    xmax *= 1.1;	ymax *= 1.1;	zmax *= 1.1;

    //get plane position
    double t = 0;// ui->horizontalSlider_slice_Multi_dir->value() / 100.0;
    Eigen::Vector3d plane_Position =
    { (1 - t) * xmin + t * xmax,(1 - t) * ymin + t * ymax,(1 - t) * zmin + t * zmax };

    for (GLKPOSITION Pos = patch->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)patch->GetTetraList().GetNext(Pos);
        Tetra->show_innerTet_split = false;

        int temp_count = 0;
        for (int k = 0; k < 4; k++) {
            QMeshNode* eachNode = Tetra->GetNodeRecordPtr(k + 1);
            Eigen::Vector3d node_coord3D;
            eachNode->GetCoord3D(node_coord3D[0], node_coord3D[1], node_coord3D[2]);
            if (node_coord3D[plane_Move] < plane_Position[plane_Move])
                temp_count++;
        }
        // mark the Split inner face
        if (temp_count == 3 || temp_count == 2 || temp_count == 1) {
            Tetra->show_innerTet_split = true;
            for (int i = 0; i < 4; i++) {
                Tetra->GetFaceRecordPtr(i + 1)->show_innerFace_split = true;
            }
        }
        // hide the part outside the range
        if (temp_count == 0) {
            for (int i = 0; i < 4; i++) {
                Tetra->GetFaceRecordPtr(i + 1)->inner = true;
            }
        }
        // remove overlap innerFace Case 1
        for (int i = 0; i < 4; i++) {
            QMeshFace* eachFace = Tetra->GetFaceRecordPtr(i + 1);

            int temp_count2 = 0;
            for (int j = 0; j < 3; j++) {
                QMeshNode* eachNode = eachFace->GetNodeRecordPtr(j);
                Eigen::Vector3d node_coord3D;
                eachNode->GetCoord3D(node_coord3D[0], node_coord3D[1], node_coord3D[2]);
                if (node_coord3D[plane_Move] < plane_Position[plane_Move])
                    temp_count2++;
            }
            if (temp_count2 == 3) eachFace->show_innerFace_split = false;
        }
    }

    // remove overlap innerFace Case 2
    for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);

        if (face->GetLeftTetra() == NULL || face->GetRightTetra() == NULL) continue;

        if (face->GetLeftTetra()->show_innerTet_split
            && face->GetRightTetra()->show_innerTet_split)
            face->show_innerFace_split = false;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////

    // give flag of innerEdge for split 
    for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
        QMeshEdge* edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);

        edge->show_innerEdge_split = false;
    }
    for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);

        if (face->show_innerFace_split) {
            for (int i = 0; i < 3; i++) {
                face->GetEdgeRecordPtr(i + 1)->show_innerEdge_split = true;
            }
        }
    }

    // give flag of inner for Edge
    for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
        QMeshEdge* edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);

        edge->inner = true;
    }
    for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);

        if (face->GetLeftTetra() == NULL || face->GetRightTetra() == NULL) {
            for (int i = 0; i < 3; i++) {
                face->GetEdgeRecordPtr(i + 1)->inner = false;
            }
        }
    }
    for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
        QMeshEdge* edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);

        Eigen::Vector3d sNode_xyz = Eigen::Vector3d::Zero();
        Eigen::Vector3d eNode_xyz = Eigen::Vector3d::Zero();
        edge->GetStartPoint()->GetCoord3D(sNode_xyz[0], sNode_xyz[1], sNode_xyz[2]);
        edge->GetEndPoint()->GetCoord3D(eNode_xyz[0], eNode_xyz[1], eNode_xyz[2]);

        if ((sNode_xyz[plane_Move] > plane_Position[plane_Move]) && 
            (eNode_xyz[plane_Move] > plane_Position[plane_Move]) &&
            (edge->inner == false)
            ) {
            edge->inner = true;
        }
    }

    pGLK->refresh(true);
}

void MainWindow::model_ROT_MOVE() {

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh is detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    std::string modelName = tetModel->getModelName();
    if (modelName == "bridgeSmall") this->_setParameter(180.0, 0.0, 20.0);
    else if (modelName == "bridgeSmall_new") this->_setParameter(180.0, 0.0, 20.0);
    else if (modelName == "Shelf_Bracket") this->_setParameter(15.0, 0.0, 0.0);
    else if (modelName == "topopt_new") this->_setParameter(145.0, 0.0, 5.0);
    else if (modelName == "topopt_new2") this->_setParameter(145.0, 0.0, 5.0);
    else if (modelName == "GEbracket") this->_setParameter(-10.0, 90.0, 15.0);
    else if (modelName == "fertility") this->_setParameter(-100.0, 0.0, 0.0);
    else if (modelName == "airbus_topopt") this->_setParameter(-20.0, 90.0, 20.0);
    //else if (modelName == "airbus_topopt") this->_setParameter(130.0, 90.0, 20.0);
    else this->_setParameter(0.0, 0.0, 0.0);

    fabriOperator = new Fabrication();
    fabriOperator->initial(model);
    //fabriOperator->mode_RotMov_4fabrication(ui->doubleSpinBox_rot_phi->value(), 
    //    ui->doubleSpinBox_rot_theta->value(), ui->doubleSpinBox_modelHeight_y->value());
    delete fabriOperator;

    this->_scalarField_2_vectorField(true);//update vector field

    updateTree();
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::_setParameter(double rot_phi, double rot_theta, double modelHeight_y) {

    //ui->doubleSpinBox_rot_phi->setValue(rot_phi);
    //ui->doubleSpinBox_rot_theta->setValue(rot_theta);
    //ui->doubleSpinBox_modelHeight_y->setValue(modelHeight_y);
}

void MainWindow::concave_Detect_vector() {

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh is detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    fabriOperator = new Fabrication();
    fabriOperator->initial(model);
    fabriOperator->concaveVector_detection();

    pGLK->refresh(true);
}

void MainWindow::concave_Smooth_vector() {

    fabriOperator->concaveVector_smooth();
    delete fabriOperator;
    updateTree();
    pGLK->refresh(true);
}

void MainWindow::concave_Detect_Smooth_vector() {


    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh is detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    fabriOperator = new Fabrication();
    fabriOperator->initial(model);

    //for (int i = 0; i < ui->spinBox_concaveDetecSmooth_loopTime->value(); i++) {
    //    fabriOperator->concaveVector_detection();
    //    fabriOperator->concaveVector_smooth();
    //}
    delete fabriOperator;

    this->_vectorField_2_scalarField(false);
    pGLK->refresh(true);
}

void MainWindow::collisionChecking_layer() {

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh is detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    PolygenMesh* isoLayerSet = this->_detectPolygenMesh(CURVED_LAYER);
    if (isoLayerSet == nullptr) { std::cerr << "No initial curved layers is detected!" << std::endl; return; }

    QMeshPatch* nozzle = this->_detectPolygenMesh(CNC_PRT, "nozzle");
    if (nozzle == nullptr) { std::cerr << "No nozzle is detected!" << std::endl; return;}

    fabriOperator = new Fabrication();
    fabriOperator->initial(model, isoLayerSet, nozzle);
    bool existCollision = fabriOperator->collisionChecking();

    if (existCollision == false) {

        // remove the layers if there is no collision
        isoLayerSet->ClearAll();
        polygenMeshList.Remove(isoLayerSet);

        std::cout << "There is no collision after checking each layer.\n" << std::endl;
        //ui->pushButton_collisionAware_flattening->setEnabled(false);
        delete fabriOperator;

    }
    else {
        std::cout << "There is collision, please do the flattening operation.\n" << std::endl;
        //ui->pushButton_collisionAware_flattening->setEnabled(true);
        //ui->pushButton_remarkCollisionTetra->setEnabled(true);
    }
    updateTree();
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::reMark_collisionVector() {

    PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
    if (polygenMesh == NULL || polygenMesh->meshType != TET_MODEL) {
        std::printf(" -- system contains no tet model, return! \n");
        return;
    }
    QMeshPatch* patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

    for (GLKPOSITION pos = patch->GetTetraList().GetHeadPosition(); pos != nullptr;) {
        QMeshTetra* tetra = (QMeshTetra*)patch->GetTetraList().GetNext(pos);

        for (int i = 0; i < 4; i++) {
            if (tetra->GetNodeRecordPtr(i + 1)->isHandle) {
                tetra->isCollisionTetra = true;
                break;
            }
        }
    }

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        Node->isHandle = false;
    }
    for (GLKPOSITION pos = patch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
        QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(pos);

        face->isHandleDraw = false;
    }
    updateTree();
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
    printf("Finish modifying the selected vector of tetrahedron.\n");
}

void MainWindow::collisionAware_Flattening() {

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    PolygenMesh* isoLayerSet = this->_detectPolygenMesh(CURVED_LAYER);
    if (isoLayerSet == nullptr) { std::cerr << "No initial curved layers detected!" << std::endl; return; }

    fabriOperator->collisionAware_flattening();
    delete fabriOperator;

    isoLayerSet->ClearAll();
    polygenMeshList.Remove(isoLayerSet);

    this->_vectorField_2_scalarField(false);
    std::cout << "Finish collision-aware flattening generation, please slice again." << std::endl;
    updateTree();
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::_input_CNC_part() {

    PolygenMesh* cncModelSet = this->_buildPolygenMesh(CNC_PRT, "CNC_PRT");

    std::vector<std::string> CNCfileSet = { "nozzle","platform" };

    //read CNC files and build mesh_patches
    char filename[1024];

    for (int i = 0; i < CNCfileSet.size(); i++) {
        sprintf(filename, "%s%s%s", "../DataSet/CNC_MODEL/", CNCfileSet[i].c_str(), ".obj");
        cout << "input " << CNCfileSet[i].data() << " from: " << filename << endl;

        QMeshPatch* cncPatch = new QMeshPatch;
        cncPatch->SetIndexNo(cncModelSet->GetMeshList().GetCount()); //index begin from 0
        cncModelSet->GetMeshList().AddTail(cncPatch);
        cncPatch->inputOBJFile(filename);
        cncPatch->patchName = CNCfileSet[i].data();
        cncPatch->drawThisPatch = false;
    }
}

void MainWindow::_scalarField_2_vectorField(bool is_normalize) {

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    this->_compTetMeshVolumeMatrix(model);
    //generate the vector field in the tetMesh
    for (GLKPOSITION Pos = model->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)model->GetTetraList().GetNext(Pos);

        Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
        Eigen::VectorXd fieldValue = Eigen::VectorXd::Zero(4);

        for (int i = 0; i < 4; i++) {
            fieldValue(i) = Tetra->GetNodeRecordPtr(i + 1)->scalarField;
            for (int j = 0; j < 3; j++) {
                gradient(j) += Tetra->VolumeMatrix(i, j) * fieldValue(i);
            }
        }
        if(is_normalize) Tetra->vectorField = gradient.normalized();
        else { Tetra->vectorField = gradient; }
    }
}

void MainWindow::_compTetMeshVolumeMatrix(QMeshPatch* patch) {

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

void MainWindow::_vectorField_smooth(int smoothLoop, bool is_normalize) {

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    for (int loop = 0; loop < smoothLoop; loop++) {

        for (GLKPOSITION Pos = model->GetTetraList().GetHeadPosition(); Pos;) {
            QMeshTetra* each_Tetra = (QMeshTetra*)model->GetTetraList().GetNext(Pos);

            double vectorField_Length = each_Tetra->vectorField.norm();

            for (int i = 0; i < 4; i++) {
                QMeshFace* thisFace = each_Tetra->GetFaceRecordPtr(i + 1);

                if (!thisFace->inner) continue;

                QMeshTetra* ConnectTetra = thisFace->GetRightTetra();
                if (each_Tetra == ConnectTetra) ConnectTetra = thisFace->GetLeftTetra();

                each_Tetra->vectorField += ConnectTetra->vectorField;
            }
            each_Tetra->vectorField.normalize();
            if (!is_normalize)  each_Tetra->vectorField *= vectorField_Length;
        }
    }
}

void MainWindow::_vectorField_2_scalarField(bool Up_Vector_Direction) {

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    Eigen::SparseMatrix<double> Parameter(3 * model->GetTetraNumber(), model->GetNodeNumber()); //A
    //std::cout << 3 * model->GetTetraNumber() << "," << model->GetNodeNumber() << std::endl;

    Eigen::VectorXd guideField(model->GetNodeNumber()); //x

    Eigen::VectorXd b(3 * model->GetTetraNumber()); //b
    b.setZero();

    std::vector<Eigen::Triplet<double>> ParaTriplet;

    for (GLKPOSITION Pos = model->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)model->GetTetraList().GetNext(Pos);

        double weight = 1.0;
        if (Up_Vector_Direction)    Tetra->vectorField << 0.0, 1.0, 0.0;

        for (int j = 0; j < 3; j++) {
            for (int i = 0; i < 4; i++) {
                QMeshNode* Node = Tetra->GetNodeRecordPtr(i + 1);
                ParaTriplet.push_back(Eigen::Triplet<double>(
                    Tetra->GetIndexNo() * 3 + j, Node->GetIndexNo(), -Tetra->VolumeMatrix(i, j) * weight)); // infill A
            }
        }
        for (int i = 0; i < 3; i++)	b(3 * Tetra->GetIndexNo() + i) = Tetra->vectorField(i) * weight;
    }

    Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

    Eigen::SparseMatrix<double> ATA(model->GetNodeNumber(), model->GetNodeNumber());
    ATA = Parameter.transpose() * Parameter;
    Eigen::PardisoLU <Eigen::SparseMatrix<double>> Solver;

    Solver.analyzePattern(ATA);
    Solver.factorize(ATA);
    Solver.compute(ATA);

    Eigen::VectorXd ATb(model->GetNodeNumber());
    ATb = Parameter.transpose() * b;
    guideField = Solver.solve(ATb);

    Eigen::VectorXd guideFieldNormalize(model->GetNodeNumber());
    // compute max and min phis
    double minPhi = INFINITY;
    double maxPhi = -INFINITY;

    for (int i = 0; i < model->GetNodeNumber(); i++) {
        if (minPhi > guideField(i)) minPhi = guideField(i);
        if (maxPhi < guideField(i)) maxPhi = guideField(i);
    }
    double range = maxPhi - minPhi;

    for (int i = 0; i < model->GetNodeNumber(); i++)
        guideFieldNormalize(i) = 1 - (guideField(i) - minPhi) / range;

    for (GLKPOSITION Pos = model->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)model->GetNodeList().GetNext(Pos);
        Node->scalarField = guideFieldNormalize(Node->GetIndexNo());
        Node->scalarField_init = guideField(Node->GetIndexNo());
    }
}

//compute initial Guess SupportEnvelope
void MainWindow::initial_Guess_SupportEnvelope_Generation() {

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh is detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    QMeshPatch* platform = this->_detectPolygenMesh(CNC_PRT, "platform");
    if (platform == nullptr) { std::cerr << "No platform is detected!" << std::endl; return; }

    PolygenMesh* supportModelSet = _buildPolygenMesh(SUPPORT_RAY, "Support_Ray");

    if (tetModel == NULL || platform == NULL || supportModelSet == NULL)
        std::cout << "There is no needed PolygenMesh, please check" << std::endl;

    supportGene = new SupportGeneration();
    supportGene->initial(model, platform, supportModelSet);
    supportGene->initial_Guess_SupportEnvelope();           //get support node
    supportGene->collect_Support_Polyline_fromTETsurface(); //collect support node and edge
    supportGene->compute_initialGuess_EnvelopeHull();       //build convex hull of initial guess
    supportGene->outputMaterial_4_envelopHullGeneration();  //output the tet and conveHull

    std::cout << "\nFinish initial guess support envelope generation STEP 1." << std::endl;
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

//compute initial Guess SupportEnvelope_plus
void MainWindow::initial_Guess_SupportEnvelope_Generation_plus() {

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh is detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    PolygenMesh* supportModelSet = _detectPolygenMesh(SUPPORT_RAY);
    if (supportModelSet == NULL)
        std::cout << "There is no supportModel PolygenMesh, please check!" << std::endl;

    //input 2 source mesh: tet_surface , remeshed convexHull
    char filename[1024];
    sprintf(filename, "%s%s%s", "../DataSet/support_convex/", model->patchName.c_str(),"_ConvexHull.obj");
    QMeshPatch* remeshedCH = new QMeshPatch;
    remeshedCH->SetIndexNo(supportModelSet->GetMeshList().GetCount()); //index begin from 0
    supportModelSet->GetMeshList().AddTail(remeshedCH);
    remeshedCH->inputOBJFile(filename, false);
    //
    sprintf(filename, "%s%s%s", "../DataSet/support_convex/", model->patchName.c_str(), "_tetSurface.obj");
    QMeshPatch* tet_surface = new QMeshPatch;
    tet_surface->SetIndexNo(supportModelSet->GetMeshList().GetCount()); //index begin from 0
    supportModelSet->GetMeshList().AddTail(tet_surface);
    tet_surface->inputOBJFile(filename, false);

    //output combined model for tetgen
    supportGene->outputMaterial_4_envelopHullGeneration_plus(); 
    delete supportGene;

    std::cout << "\nFinish initial guess support envelope generation STEP 2." << std::endl;
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::compatibleLayer_Generation() {

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh is detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    PolygenMesh* supportModelSet = _detectPolygenMesh(SUPPORT_RAY);
    if (supportModelSet == NULL) {
        supportModelSet = _buildPolygenMesh(SUPPORT_RAY, "Support_Ray");
    }
    else {
        supportModelSet->ClearAll();
        //polygenMeshList.Remove(supportModelSet);
        std::cout << "There is already existing a supportModel PolygenMesh, it has been reconstructed!" << std::endl;
    }
    //input tetSurface mesh
    char filename[1024];
    sprintf(filename, "%s%s%s", "../DataSet/support_convex/", model->patchName.c_str(), "_tetSurface.obj");
    QMeshPatch* model_surface = new QMeshPatch;
    model_surface->SetIndexNo(supportModelSet->GetMeshList().GetCount()); //index begin from 0
    supportModelSet->GetMeshList().AddTail(model_surface);
    model_surface->inputOBJFile(filename, false);

    //input support Tet 
    sprintf(filename, "%s%s%s", "../DataSet/TET_MODEL/", model->patchName.c_str(), "_supportTet.tet");
    PolygenMesh* support_tetModel = this->_buildPolygenMesh(SUPPORT_TET_MODEL, model->patchName + "_Support");
    QMeshPatch* patch_supportTet = new QMeshPatch;
    patch_supportTet->SetIndexNo(support_tetModel->GetMeshList().GetCount()); //index begin from 0
    support_tetModel->GetMeshList().AddTail(patch_supportTet);
    patch_supportTet->inputTETFile(filename, false);

    supportGene = new SupportGeneration();
    supportGene->initial(model, model_surface, patch_supportTet);
    
    //avoid detecting boundary repeatly
    sprintf(filename, "%s%s%s", "../DataSet/TET_MODEL/", model->patchName.c_str(), "_supportTet_hollowed.tet");
    std::ifstream f(filename);
    if (f.good()) {
        std::cout << model->patchName << "_supportTet_hollowed.tet is existed." << std::endl;
    }
    else {
        supportGene->boundary_Detection();//and generate a hollowed support space
    }
    
    support_tetModel->ClearAll();
    //sprintf(filename, "%s%s%s", "../DataSet/TET_MODEL/", model->patchName.c_str(), "_supportTet_hollowed.tet");
    QMeshPatch* patch_supportTet_hollowed = new QMeshPatch;
    patch_supportTet_hollowed->SetIndexNo(support_tetModel->GetMeshList().GetCount()); //index begin from 0
    support_tetModel->GetMeshList().AddTail(patch_supportTet_hollowed);
    patch_supportTet_hollowed->inputTETFile(filename, false);
    if (support_tetModel->GetMeshList().GetCount() != 1) 
        std::cout << "the mesh number should be 1, please check." << std::endl;

    //scalar & vector transfer
    supportGene->update_supportTet_2_hollowed(patch_supportTet_hollowed);
    supportGene->transfer_ScalarField_2_supportSpace();
    supportGene->transfer_VectorField_2_supportSpace();
    supportGene->scalarField_4_supportSpace();

    //updateTree();
    //pGLK->refresh(true);
    //return;

    //generate compatible layers
    PolygenMesh* compatible_isoLayerSet = this->_buildPolygenMesh(CURVED_LAYER, "compatible_isoLayers");
    IsoLayerGeneration* slicer = new IsoLayerGeneration(model);
    slicer->generateIsoSurface(compatible_isoLayerSet, ui->spinBox_isoLayerNumber->value());
    slicer->generateIsoSurface_support(
        compatible_isoLayerSet, patch_supportTet_hollowed, ui->spinBox_isoLayerNumber->value());
    delete slicer;
    ui->spinBox_ShowLayerIndex->setMinimum((int)0);
    ui->radioButton_compatibleLayer->setEnabled(true);
    ui->spinBox_ShowLayerIndex->setMaximum(compatible_isoLayerSet->GetMeshList().GetCount() - 1);
    //ui->pushButton_remarkOverhangFace->setEnabled(true);

    //organize the compatible layers
    supportGene->organize_compatibleLayer_Index(compatible_isoLayerSet,
        compatible_isoLayerSet->GetMeshList().GetCount() - ui->spinBox_isoLayerNumber->value());
    supportGene->output_compatibleLayer_4_remesh(compatible_isoLayerSet);
    delete supportGene;

    std::cout << "\nFinish compatible layers generation." << std::endl;
    updateTree();
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

//change the maxLayer number
void MainWindow::change_maxLayerNum_normalORcompatible() {

    PolygenMesh* compatible_isoLayerSet = this->_detectPolygenMesh(CURVED_LAYER);
    if (compatible_isoLayerSet == nullptr) { std::cerr << "No compatible layers is detected!" << std::endl; return; }

    int max_compatible_index = -1;
    for (GLKPOSITION posMesh = compatible_isoLayerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
        QMeshPatch* each_patch = (QMeshPatch*)compatible_isoLayerSet->GetMeshList().GetNext(posMesh);

        if (each_patch->compatible_layer_Index > max_compatible_index)
            max_compatible_index = each_patch->compatible_layer_Index;
    }

    if (ui->radioButton_compatibleLayer->isChecked()) {
        ui->spinBox_ShowLayerIndex->setMaximum(max_compatible_index);
    }
    else {
        ui->spinBox_ShowLayerIndex->setMaximum(compatible_isoLayerSet->GetMeshList().GetCount() - 1);
    }
}

void MainWindow::mark_OverhangFace() {

    supportGene = new SupportGeneration();

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == NULL) { std::cerr << "No tet mesh is detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    PolygenMesh* compatible_isoLayerSet = this->_detectPolygenMesh(CURVED_LAYER);
    if (compatible_isoLayerSet == NULL) {
        compatible_isoLayerSet = this->_buildPolygenMesh(CURVED_LAYER, "compatible_isoLayers");
    }
    else {
        compatible_isoLayerSet->ClearAll();
        std::cout << "There is already existing a compatible_isoLayerSet PolygenMesh, it will be reconstructed!" << std::endl;
    }

    QMeshPatch* platform = this->_detectPolygenMesh(CNC_PRT, "platform");
    if (platform == NULL) { std::cerr << "No platform is detected!" << std::endl; return; }

    PolygenMesh* supportModelSet = _detectPolygenMesh(SUPPORT_RAY);
    if (supportModelSet == NULL) {
        supportModelSet = _buildPolygenMesh(SUPPORT_RAY, "Support_Ray");
    }
    else {
        supportModelSet->ClearAll();
        std::cout << "There is already existing a supportModel PolygenMesh, it has been reconstructed!" << std::endl;
    }

    supportGene->update_inputMesh_4treeSkeleton_Generation(model, compatible_isoLayerSet, platform, supportModelSet);

    ui->spinBox_ShowLayerIndex->setMinimum((int)0);
    ui->radioButton_compatibleLayer->setEnabled(true);
    //ui->pushButton_treeSkeletonGeneration->setEnabled(true);
    //ui->pushButton_remarkOverhangFace->setEnabled(true);
    ui->radioButton_compatibleLayer->setChecked(false);
    ui->spinBox_ShowLayerIndex->setMaximum(compatible_isoLayerSet->GetMeshList().GetCount() - 1);

    std::cout << "\nFinish Overhang face mark." << std::endl;
    updateTree();
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::reMark_OverhangFace() {

    PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
    if (polygenMesh == NULL || polygenMesh->meshType != TET_MODEL) {
        std::printf(" -- system contains no tet model, return! \n");
        return;
    }
    QMeshPatch* patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
        if(Node->inner || Node->isHandle) Node->need_Support = false;
    }

    for (GLKPOSITION pos = patch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
        QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(pos);

        if (face->inner) continue;

        for (int i = 0; i < 3; i++) {
            if (face->GetNodeRecordPtr(i)->need_Support == false) {
                face->needSupport = false;
                break;
            }
        }
    }

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        Node->isHandle = false;
    }
    for (GLKPOSITION pos = patch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
        QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(pos);

        face->isHandleDraw = false;
    }
    updateTree();
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
    printf("Finish modifying the selected vector of tetrahedron.\n");
}

void MainWindow::build_Support_Tree_Skeleton() {

    supportGene->build_support_tree();
    //ui->pushButton_slimmedSupportGeneration->setEnabled(true);

    std::cout << "\nFinish tree skeleton generation." << std::endl;
    updateTree();
    pGLK->refresh(true);
    //pGLK->Zoom_All_in_View();
}

void MainWindow::generate_slim_supportLayer() {

    supportGene->compute_Support_tree_Field();
    std::cout << "Finish the implicit field calculation." << std::endl;
    supportGene->build_tight_supportLayers();
    std::cout << "Finish the tight support layer generation." << std::endl;
    delete supportGene;

    this->output_IsoLayer_set();

    PolygenMesh* compatible_isoLayerSet = this->_detectPolygenMesh(CURVED_LAYER);
    ui->spinBox_ShowLayerIndex->setMaximum(compatible_isoLayerSet->GetMeshList().GetCount() - 1);
    std::cout << "\nFinish slim support layer generation." << std::endl;
    pGLK->refresh(true);
    //pGLK->Zoom_All_in_View();
}

void MainWindow::toolPath_Generation() {

    PolygenMesh* compatible_isoLayerSet = this->_detectPolygenMesh(CURVED_LAYER);
    if (compatible_isoLayerSet == NULL) {
        compatible_isoLayerSet = _buildPolygenMesh(CURVED_LAYER, "compatible_isoLayers");
    }
    else {
        compatible_isoLayerSet->ClearAll();
        //polygenMeshList.Remove(supportModelSet);
        std::cout << "There is already existing a compatible_isoLayers PolygenMesh, it has been reconstructed!" << std::endl;
    }

    PolygenMesh* toolpathSet = this->_buildPolygenMesh(TOOL_PATH, "toolPath");
    toolpathGeneration* ToolPathComp_layer = new toolpathGeneration(compatible_isoLayerSet, toolpathSet,
        ui->doubleSpinBox_toolPathWidth->value(), ui->doubleSpinBox_toolPathDistance->value());

    ToolPathComp_layer->generate_all_toolPath();
    //ToolPathComp_layer->output_toolpath(toolpathSet, ui->checkBox_outputToolpath->isChecked());
    delete ToolPathComp_layer;

    ui->radioButton_compatibleLayer->setEnabled(true);
    ui->spinBox_ShowLayerIndex->setMaximum(compatible_isoLayerSet->GetMeshList().GetCount() - 1);
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
    std::cout << "Finish generating toolpath in the materialSpace.\n" << std::endl;
}

void MainWindow::toolPath_hybrid_Generation() {

    PolygenMesh* compatible_isoLayerSet = this->_detectPolygenMesh(CURVED_LAYER);
    if (compatible_isoLayerSet == NULL) {
        compatible_isoLayerSet = _buildPolygenMesh(CURVED_LAYER, "compatible_isoLayers");
    }
    else {
        compatible_isoLayerSet->ClearAll();
        //polygenMeshList.Remove(supportModelSet);
        std::cout << "There is already existing a compatible_isoLayers PolygenMesh, it has been reconstructed!" << std::endl;
    }

    PolygenMesh* toolpathSet = this->_buildPolygenMesh(TOOL_PATH, "toolPath");
    toolpathGeneration* ToolPathComp_layer = new toolpathGeneration(compatible_isoLayerSet, toolpathSet,
        ui->doubleSpinBox_toolPathWidth->value(), ui->doubleSpinBox_toolPathDistance->value());

    //temp smooth
    ToolPathComp_layer->smoothingIsoSurface();

    ToolPathComp_layer->generate_all_hybrid_toolPath();
    //ToolPathComp_layer->output_toolpath(toolpathSet, ui->checkBox_outputToolpath->isChecked());
    delete ToolPathComp_layer;

    ui->radioButton_compatibleLayer->setEnabled(true);
    ui->spinBox_ShowLayerIndex->setMaximum(compatible_isoLayerSet->GetMeshList().GetCount() - 1);
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
    std::cout << "Finish generating toolpath in the materialSpace.\n" << std::endl;
}

void MainWindow::output_Toolpath_set() {

    PolygenMesh* toolpathSet = this->_detectPolygenMesh(TOOL_PATH);
    if (toolpathSet == NULL) {
        std::cerr << "There is no toolpath set, please check." << std::endl; return;
    }
    FileIO* IO_operator = new FileIO();
    if (ui->checkBox_outputCompatiblewaypoint->isChecked()) {
        IO_operator->output_toolpath_compatible(toolpathSet);
        std::cout << "The output toolpath is compatible, but the layer is splited to generate resonable toolpath Num." << std::endl;
    }
    else {
    IO_operator->output_toolpath(toolpathSet);
    }
    delete IO_operator;

    std::cout << "Finish outputing surface mesh of the tetmesh. " << std::endl;
}

void MainWindow::readGcodeSourceData() {

    this->on_pushButton_clearAll_clicked();

    PolygenMesh* isoLayerSet = this->_detectPolygenMesh(CURVED_LAYER);
    if (isoLayerSet == NULL) {
        isoLayerSet = this->_buildPolygenMesh(CURVED_LAYER, "IsoLayer");}
    else {
        isoLayerSet->ClearAll();
        std::cout << "There is already existing a isoLayerSet PolygenMesh, it will be reconstructed!" << std::endl;
    }

    PolygenMesh* toolpathSet = this->_detectPolygenMesh(TOOL_PATH);
    if (toolpathSet == NULL) {
        toolpathSet = this->_buildPolygenMesh(TOOL_PATH, "Toolpath");}
    else {
        toolpathSet->ClearAll();
        std::cout << "There is already existing a toolpathSet PolygenMesh, it will be reconstructed!" << std::endl;
    }

    PolygenMesh* cncSet = this->_detectPolygenMesh(CNC_PRT);
    if (cncSet == NULL) {
        cncSet = this->_buildPolygenMesh(CNC_PRT,"CNC_PRT");
    }
    else {
        cncSet->ClearAll();
        std::cout << "There is already existing a CNC part PolygenMesh, it will be reconstructed!" << std::endl;
    }

    this->_updateFabractionParameter();

    if (ui->doubleSpinBox_toolLength->value() < 0.0) { std::cout << "the length of tool is error!!!" << std::endl;  return; }

    GcodeGene = new GcodeGeneration();
    std::string modelName = ui->lineEdit_SorceDataDir->text().toStdString();
    std::string FileDir = "../DataSet/fabricationSource/" + modelName;
    GcodeGene->initial(isoLayerSet, toolpathSet, cncSet, ui->checkBox_Yup2Zup->isChecked(), FileDir,
        ui->doubleSpinBox_Xmove->value(), ui->doubleSpinBox_Ymove->value(), ui->doubleSpinBox_Zmove->value(),
        ui->doubleSpinBox_toolLength->value(), modelName);
    int layerNum = GcodeGene->read_layer_toolpath_cnc_files();

    ui->spinBox_ShowLayerIndex_2->setMaximum(layerNum - 1);
    ui->spinBox_StartLayerIndex->setMaximum(layerNum - 1);
    ui->spinBox_GcodeGeneFromIndex->setMaximum(layerNum - 1);
    ui->spinBox_GcodeGeneToIndex->setMaximum(layerNum - 1);
    ui->spinBox_GcodeGeneToIndex->setValue(layerNum - 1);
    //ui->pushButton_output_userWaypoints->setEnabled(true);

    updateTree();
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
    std::cout << "Finish inputing GcodeSource data.\n" << std::endl;
}

void MainWindow::output_userWaypoints() {

    PolygenMesh* toolpathSet = this->_detectPolygenMesh(TOOL_PATH);
    if (toolpathSet == NULL) {
        std::cout << "there is no waypoints info, please check." << std::endl;
        return;
    }

    FileIO* IO_operator = new FileIO();
    std::string surface_dir = "../DataSet/tempUse/waypoint_user/";
    IO_operator->output_userWaypoints(toolpathSet, surface_dir,
                ui->doubleSpinBox_Xmove->value(),
                ui->doubleSpinBox_Ymove->value(),
                ui->doubleSpinBox_Zmove->value());
    delete IO_operator;

    std::cout << "Finish outputing user waypoints. " << std::endl;
}

void MainWindow::_updateFabractionParameter() {

    std::string modelName = ui->lineEdit_SorceDataDir->text().toStdString();
    this->_setParameter(true, 0.0, 0.0, 0.3);
    if (modelName == "tree3") this->_setParameter(true, 0.0, 0.0, 92);
    if (modelName == "curvedBar") this->_setParameter(true, 0.0, 0.0, 0.6);
    if (modelName == "dome") this->_setParameter(true, 0.0, 0.0, 0.0);
    if (modelName == "domeL") this->_setParameter(true, 0.0, 0.0, 0.0);
    if (modelName == "ankleBaseV1") this->_setParameter(true, 0.0, 0.0, 0.6);
    if (modelName == "bunny_cut6") this->_setParameter(true, 0.0, 0.0, 1.0);
    if (modelName == "bunny_5contour") this->_setParameter(true, 0.0, 0.0, 0.4);
    if (modelName == "bunny_Allcontour") this->_setParameter(true, 0.0, 0.0, 0.4);
    if (modelName == "bunny_YM") this->_setParameter(true, 0.0, 0.0, 0.4);
    if (modelName == "bunny_cut6_plane") this->_setParameter(true, 0.0, 0.0, 1.0);

    if (modelName == "armadillo_5contour") this->_setParameter(true, 0.0, 0.0, 1.0);
    if (modelName == "armadillo_YM") this->_setParameter(true, 0.0, 0.0, 1.0);

    if (modelName == "simple_curve") this->_setParameter(false, 0.0, 0.0, 1.0);
    if (modelName == "model_1") this->_setParameter(false, 0.0, 0.0, 1.0);
    if (modelName == "model_2") this->_setParameter(false, 0.0, 0.0, 1.0);

    if (modelName == "wing_mirror_step1") this->_setParameter(true, 0, 0, 0.6);
    if (modelName == "wing_mirror_step3") this->_setParameter(true, 34.0, -62.0, 93.3035);
    if (modelName == "wing_mirror_step2") this->_setParameter(true, 56.0, 40.5, 118.2035);
    if (modelName == "wing_mirror_step4") this->_setParameter(true, 0, 0, 118.2035);

    if (modelName == "yoga_icra") this->_setParameter(false, 0.0, 0.0, 0.0);

    if (modelName == "turbine_blade_surface") this->_setParameter(true, 0.0, 0.0, 108.0);
}

void MainWindow::_setParameter(bool Yup2Zup, double Xmove, double Ymove, double Zmove) {

    ui->checkBox_Yup2Zup->setChecked(Yup2Zup);
    ui->doubleSpinBox_Xmove->setValue(Xmove);
    ui->doubleSpinBox_Ymove->setValue(Ymove);
    ui->doubleSpinBox_Zmove->setValue(Zmove);
}

void MainWindow::changeWaypointDisplay() {

    bool single = ui->checkBox_EachLayerSwitch_2->isChecked();
    int startLayerIndex = ui->spinBox_StartLayerIndex->value();
    ui->spinBox_ShowLayerIndex_2->setMinimum(startLayerIndex);
    int currentLayerIndex = ui->spinBox_ShowLayerIndex_2->value();

    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        if (polygenMesh->meshType != CURVED_LAYER
            && polygenMesh->meshType != TOOL_PATH)
            continue;

        int index = 0;

        for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
            QMeshPatch* Patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);

            Patch->drawThisPatch = false;

            if (index >= startLayerIndex) {
                if (single == true) {
                    if (index == currentLayerIndex)
                        Patch->drawThisPatch = true;
                }
                else {
                    if (index <= currentLayerIndex)
                        Patch->drawThisPatch = true;
                }

                if (index == currentLayerIndex) {
                    if (polygenMesh->meshType == TOOL_PATH)
                        ui->label_currentFile->setText(QString::fromStdString(("File: " + Patch->patchName)));
                }
            }
            index++;
        }
    }
    pGLK->refresh(true);
}

void MainWindow::runDHWcalculation() {

    ui->checkBox_varyDistance->setCheckState(Qt::Checked); // it is necessary for all calculation, so defalt

    GcodeGene->updateParameter(ui->spinBox_GcodeGeneFromIndex->value(), ui->spinBox_GcodeGeneToIndex->value(),
        ui->doubleSpinBox_lambda->value(), ui->checkBox_varyDistance->isChecked(), ui->checkBox_varyHeight->isChecked(),
        ui->checkBox_varyWidth->isChecked(), ui->checkBox_TestDHW_Switch->isChecked());


    GcodeGene->calDHW();
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
    std::cout << "Finish calculating Distance/Height/Width.\n" << std::endl;
}

void MainWindow::runSingularityOpt() {

    if (newConfig_CNC == false) {
        GcodeGene->singularityOpt();
    }
    else {
        GcodeGene->singularityOpt_newConfig();
    }
    GcodeGene->testXYZBCE(true);
    //ui->pushButton_output2Robot->setEnabled(true);
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::output_preProcess_waypoints_4_Robot() {

    PolygenMesh* toolpathSet = this->_detectPolygenMesh(TOOL_PATH);
    if (toolpathSet == NULL) {
        std::cout << "there is no waypoints info, please check." << std::endl;
        return;
    }

    FileIO* IO_operator = new FileIO();
    std::string robot_Wp_dir = "../DataSet/tempUse/waypoint_robot/";
    IO_operator->output_robotWaypoints(toolpathSet, robot_Wp_dir);
    delete IO_operator;

    std::cout << "Finish outputing robot waypoints (Yup->Zup, initial smooth, E value, Jump flag). " << std::endl;
}

void MainWindow::runCollisionCheck() {

    GcodeGene->detectCollision_2();
    pGLK->refresh(true);
}

void MainWindow::runCollisionElimination() {
    if (newConfig_CNC == false) {
        GcodeGene->graph_Search_Shortest_Path();
    }
    else {
        GcodeGene->graph_Search_Shortest_Path_newConfig();
    }
    GcodeGene->testXYZBCE(true); // does not output the inserted nodes
    pGLK->refresh(true);
}

void MainWindow::runWriteGcode() {

    GcodeGene->feedrateOpt();

    string targetFileName = (ui->lineEdit_SorceDataDir->text()).toStdString() + "_Gcode.txt";
    GcodeGene->writeGcode(targetFileName);
    pGLK->refresh(true);
}

void MainWindow::runGcodeSimulation() {

    PolygenMesh* Slices = _detectPolygenMesh(CURVED_LAYER);
    if (Slices != NULL) {
        Slices->bShow = false;
    }
    else {
        std::cout << "There is no needed mesh, return." << std::endl;
        return;
    }

    string FileName = (ui->lineEdit_SorceDataDir->text()).toStdString() + "_Gcode.txt";
    simuLayerInd = ui->spinBox_GcodeGeneFromIndex->value() - 1;//  will meet "G1 F1500" at the first several lines, so -1
    ui->checkBox_EachLayerSwitch->setCheckState(Qt::Checked);
    if (newConfig_CNC == false) GcodeGene->readGcodeFile(Gcode_Table, FileName);
    else GcodeGene->readGcodeFile_newConfig(Gcode_Table, FileName);
    ui->progressBar_GcodeSimulation->setRange(0, Gcode_Table.rows() - 1);

    gcodetimerItertime = 0;
    Gcode_timer.start(20);

    std::cout << "------------------------------------------- G code Simulation Running ..." << std::endl;
}

void MainWindow::doTimerGcodeMoving() {

    int simuLayerInd_1st = ui->spinBox_GcodeGeneFromIndex->value();
    //int simuLayerInd_1st = 0;
    bool singleLayer = ui->checkBox_EachLayerSwitch->isChecked();
    ui->spinBox_ShowLayerIndex_2->setValue(simuLayerInd);
    double machine_X = Gcode_Table(gcodetimerItertime, 0);
    double machine_Y = Gcode_Table(gcodetimerItertime, 1);
    double machine_Z = Gcode_Table(gcodetimerItertime, 2);
    double machine_B = Gcode_Table(gcodetimerItertime, 3);
    double machine_C = Gcode_Table(gcodetimerItertime, 4);
    double newLayerFlag = Gcode_Table(gcodetimerItertime, 5);

    PolygenMesh* waypoints = _detectPolygenMesh(TOOL_PATH);
    PolygenMesh* cnc = _detectPolygenMesh(CNC_PRT);
    if (cnc == NULL || waypoints == NULL) {
        cerr << "There is no CNC model OR waypoints" << endl; return;
    }

    if (newLayerFlag != 0.0) {
        simuLayerInd++;
        std::cout << "Simulating the printing of Layer " << simuLayerInd << std::endl;
    }

    // Model waypoint show
    for (GLKPOSITION waypointsPos = waypoints->GetMeshList().GetHeadPosition(); waypointsPos;) {
        QMeshPatch* WayPointsPatch = (QMeshPatch*)waypoints->GetMeshList().GetNext(waypointsPos);
        // show one layer
        if (singleLayer == true) {
            if (WayPointsPatch->GetIndexNo() != simuLayerInd) {
                WayPointsPatch->drawThisPatch = false;	continue;
            }
            else {
                WayPointsPatch->drawThisPatch = true;
            }
        }
        // show before layers
        else {
            if (WayPointsPatch->GetIndexNo() <= simuLayerInd && WayPointsPatch->GetIndexNo() >= simuLayerInd_1st) {
                WayPointsPatch->drawThisPatch = true;
            }
            else {
                WayPointsPatch->drawThisPatch = false; 	continue;
            }
        }
    }

    //waypoint move
    for (GLKPOSITION waypointsPos = waypoints->GetMeshList().GetHeadPosition(); waypointsPos;) {
        QMeshPatch* WayPointsPatch = (QMeshPatch*)waypoints->GetMeshList().GetNext(waypointsPos);

        if (WayPointsPatch->GetIndexNo() > simuLayerInd) continue;

        for (GLKPOSITION Pos = WayPointsPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* node = (QMeshNode*)WayPointsPatch->GetNodeList().GetNext(Pos);

            Eigen::Vector3d waypoints_Base_Position = node->m_printPos;
            //node->GetCoord3D_last(waypoints_Base_Position[0], waypoints_Base_Position[1], waypoints_Base_Position[2]);
            Eigen::Vector3d waypoints_New_Position = _calPartGuesture(true, waypoints_Base_Position, 0.0, 0.0, 0.0, 0.0, machine_C);
            node->SetCoord3D(waypoints_New_Position[0], waypoints_New_Position[1], waypoints_New_Position[2]);

            Eigen::Matrix3d Rot_C; double rad_C = DEGREE_TO_ROTATE(machine_C);
            Rot_C << cos(rad_C), -sin(rad_C), 0, sin(rad_C), cos(rad_C), 0, 0, 0, 1;
            Eigen::Vector3d waypoints_New_Normal = Rot_C * node->m_printNor;
            node->SetNormal(waypoints_New_Normal[0], waypoints_New_Normal[1], waypoints_New_Normal[2]);
        }
    }

    // CNC move
    for (GLKPOSITION cncPos = cnc->GetMeshList().GetHeadPosition(); cncPos;) {
        QMeshPatch* cncPatch = (QMeshPatch*)cnc->GetMeshList().GetNext(cncPos);

        if (!ui->checkBox_showCNC->isChecked()) {
            if (cncPatch->patchName == "c_axis" || cncPatch->patchName == "nozzle")
                cncPatch->drawThisPatch = true;
            else
                cncPatch->drawThisPatch = false;
        }
        else
            cncPatch->drawThisPatch = true;

        for (GLKPOSITION Pos = cncPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* node = (QMeshNode*)cncPatch->GetNodeList().GetNext(Pos);

            Eigen::Vector3d cnc_Base_Position; node->GetCoord3D_last(cnc_Base_Position[0], cnc_Base_Position[1], cnc_Base_Position[2]);
            Eigen::Vector3d cnc_New_Position;

            if (cncPatch->patchName == "b_axis" || cncPatch->patchName == "nozzle") {

                cnc_New_Position = _calPartGuesture(false, cnc_Base_Position, machine_X, machine_Y, machine_Z, machine_B, 0.0);
            }

            if (cncPatch->patchName == "c_axis"|| 
                cncPatch->patchName == "fixture_5AM"|| cncPatch->patchName == "baseModel_5AM" ||
                cncPatch->patchName == "fixture_self" || cncPatch->patchName == "baseModel_self" ||
                cncPatch->patchName == "fixture_after_machining_5AX" 
                || cncPatch->patchName == "baseModel_after_machining_5AX") {

                cnc_New_Position = _calPartGuesture(true, cnc_Base_Position, 0.0, 0.0, 0.0, 0.0, machine_C);
            }

            if (cncPatch->patchName == "z_axis") {

                cnc_New_Position = _calPartGuesture(false, cnc_Base_Position, machine_X, machine_Y, machine_Z, 0.0, 0.0);
            }

            if (cncPatch->patchName == "y_axis") {

                cnc_New_Position = _calPartGuesture(false, cnc_Base_Position, machine_X, machine_Y, 0.0, 0.0, 0.0);
            }

            if (cncPatch->patchName == "x_axis") {

                cnc_New_Position = _calPartGuesture(false, cnc_Base_Position, machine_X, 0.0, 0.0, 0.0, 0.0);
            }

            if (cncPatch->patchName == "frame") {
                cnc_New_Position = _calPartGuesture(true, cnc_Base_Position, 0.0, 0.0, 0.0, 0.0, 0.0);
            }

            node->SetCoord3D(cnc_New_Position[0], cnc_New_Position[1], cnc_New_Position[2]);
        }
    }

    gcodetimerItertime++;
    ui->progressBar_GcodeSimulation->setValue(gcodetimerItertime);
    if (gcodetimerItertime >= Gcode_Table.rows()) {
        Gcode_timer.stop();
        std::cout << "------------------------------------------- G code Simulation Finish!" << endl;
    }
    pGLK->refresh(true);
    changeWaypointDisplay();
    bool stopSimulation_switch = ui->checkBox_stopSimulation->isChecked();
    if (stopSimulation_switch == true) {
        Gcode_timer.stop();
        ui->checkBox_stopSimulation->setCheckState(Qt::Unchecked);
        std::cout << "------------------------------------------- Quit G code Simulation !" << endl;
    }
}

Eigen::Vector3d MainWindow::_calPartGuesture(
    bool table_OR_head, Eigen::Vector3d printPos, double X, double Y, double Z, double B, double C) {

    Eigen::Vector3d temp = Eigen::Vector3d::Zero();
    Eigen::Vector4d temp_4d = Eigen::Vector4d::Zero();

    Eigen::Vector4d part_Postion = Eigen::Vector4d::Zero();
    part_Postion << printPos[0], printPos[1], printPos[2], 1.0;
    double rad_B = DEGREE_TO_ROTATE(B);
    double rad_C = DEGREE_TO_ROTATE(C);

    double h = ui->doubleSpinBox_toolLength->value();

    if (table_OR_head) {

        Eigen::Matrix4d Rot_C;
        Rot_C << cos(rad_C), -sin(rad_C), 0, 0,
            sin(rad_C), cos(rad_C), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        temp_4d = Rot_C * part_Postion;

    }
    else {

        Eigen::Matrix4d Offset;
        Offset << 1, 0, 0, X,
            0, 1, 0, Y,
            0, 0, 1, Z + h,
            0, 0, 0, 1;

        Eigen::Matrix4d Rot_B;
        Rot_B << 1, 0, 0, 0,
            0, cos(rad_B), -sin(rad_B), 0,
            0, sin(rad_B), cos(rad_B), 0,
            0, 0, 0, 1;

        Eigen::Matrix4d Offset_back;
        Offset_back << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, -h,
            0, 0, 0, 1;

        temp_4d = Offset * Rot_B * Offset_back * part_Postion;
    }
    temp << temp_4d[0], temp_4d[1], temp_4d[2];

    return temp;
}

void MainWindow::cal_Dist_scanAndmodel() {

    this->on_pushButton_clearAll_clicked();
    char filename[1024];
    std::string packName = "../DataSet/fabricationTest/test_scanning/aligned";
    PolygenMesh* scanningMeshSet = this->_detectPolygenMesh(SURFACE_MESH);
    if (scanningMeshSet == NULL) {
        scanningMeshSet = this->_buildPolygenMesh(SURFACE_MESH, "Surface_Mesh");
    }
    else {
        scanningMeshSet->ClearAll();
        std::cout << "There is already existing a scanningMeshSet PolygenMesh, please check!" << std::endl;
    }

    //std::vector<std::string> fileCell = { "armadillo_original.obj","armardillo_planar(transformed).obj" };
    //std::vector<std::string> fileCell = { "armadillo_original.obj","armadillo_curved(transformed).obj" }; 
    std::vector<std::string> fileCell = { "bunny_original.obj","bunny_curved(transformed).obj" };
    //std::vector<std::string> fileCell = { "bunny_original.obj","bunny_planar(transformed).obj" };
    //std::vector<std::string> fileCell = { "teapot_original.obj","teapot_planar(transformed).obj" };
    //std::vector<std::string> fileCell = { "teapot_original.obj","teapot_curved(transformed).obj" };
    //std::vector<std::string> fileCell = { "ankle_original.obj","ankle_plannar(transformed).obj" };
    //std::vector<std::string> fileCell = { "ankle_original.obj","ankle_curved(transformed).obj" };

    for (int i = 0; i < fileCell.size(); i++) {

        sprintf(filename, "%s%s%s", packName.c_str(), "/", fileCell[i].data());

        QMeshPatch* layers = new QMeshPatch;
        layers->SetIndexNo(scanningMeshSet->GetMeshList().GetCount()); //index begin from 0
        scanningMeshSet->GetMeshList().AddTail(layers);
        layers->patchName = fileCell[i].data();
        layers->inputOBJFile(filename);
    }

    std::cout << "finish reading files, start PQP distance calculation." << std::endl;

    //////// calculate the minimum distance between scanning node to model surface ////////
    QMeshPatch* modelPatch = (QMeshPatch*)scanningMeshSet->GetMeshList().GetHead();
    QMeshPatch* scanningPatch = (QMeshPatch*)scanningMeshSet->GetMeshList().GetTail();
    // build PQP model for bottom layers
    PQP_Model* pqpModel = new PQP_Model();
    pqpModel->BeginModel();  int index = 0;
    PQP_REAL p1[3], p2[3], p3[3];

    for (GLKPOSITION Pos = modelPatch->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* Face = (QMeshFace*)modelPatch->GetFaceList().GetNext(Pos);

        Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
        Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
        Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);

        pqpModel->AddTri(p1, p2, p3, index);
        index++;
    }
    pqpModel->EndModel();

    for (GLKPOSITION Pos = scanningPatch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)scanningPatch->GetNodeList().GetNext(Pos);

        double minHeight = 99999.99;
        PQP_DistanceResult dres; dres.last_tri = pqpModel->last_tri;
        PQP_REAL p[3];
        Node->GetCoord3D(p[0], p[1], p[2]);
        PQP_Distance(&dres, pqpModel, p, 0.0, 0.0);        
        Node->m_icpDist = dres.Distance(); // ICP distance
    }
    //tempuse
    for (GLKPOSITION Pos = scanningPatch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)scanningPatch->GetNodeList().GetNext(Pos);

        double xx, yy, zz;
        Node->GetCoord3D(xx, yy, zz);
        //Node->SetCoord3D(xx, yy, zz-5.0);
        //if ((yy < 18.0 && yy >0.0) || (yy < 63.0 && yy >39.0) || (yy < 96.0 && yy >85.0))
        //if (yy >59.0)
        //    Node->m_icpDist *= 2.0;
    }

    //get avg/max error
    double maxError = -9999999.9;
    double avgError = 0.0;
    int nodeNum = 0;
    for (GLKPOSITION Pos = scanningPatch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)scanningPatch->GetNodeList().GetNext(Pos);

        if (Node->m_icpDist > 2.0) continue;

        if (maxError < Node->m_icpDist)  maxError = Node->m_icpDist;
        avgError += Node->m_icpDist;
        nodeNum++;
    }
    avgError /= nodeNum;
    std::cout << "the maxError = " << maxError << " the avgError =  " << avgError << std::endl;

    //end

    modelPatch->drawThisPatch = false;

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}