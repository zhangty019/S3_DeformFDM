#include <string>
#include <iostream>

#include "GcodeGeneration.h"
#include "GLKGeometry.h"
#include "dirent.h"
#include "../ThirdPartyDependence/PQPLib/PQP.h"
#include "io.h"
#include "Graph.h"

void GcodeGeneration::initial(
	PolygenMesh* Slices, PolygenMesh* Waypoints, PolygenMesh* CncPart,
	bool Yup2Zup, std::string Dir, 
    double Xmove, double Ymove, double Zmove,
    double toolLength,
    std::string modelName) {

	m_Slices = Slices;
	m_Waypoints = Waypoints;
	m_CncPart = CncPart;

	m_Yup2Zup = Yup2Zup;
	m_Dir = Dir;
	m_Xmove = Xmove;
	m_Ymove = Ymove;
	m_Zmove = Zmove;
    h = toolLength;
    m_modelName = modelName;
}

int GcodeGeneration::read_layer_toolpath_cnc_files() {

	std::string PosNorFileDir = m_Dir + "/waypoint";
	std::string LayerFileDir = m_Dir + "/layer";

	this->_getFileName_Set(PosNorFileDir, wayPointFileCell);
    this->_getFileName_Set(LayerFileDir, sliceSetFileCell);

    if (wayPointFileCell.size() != sliceSetFileCell.size()) { 
        std::cout << "The file number of slics and toolpath is not the same, please check." << std::endl;
        return 0; 
    }

    this->_readWayPointData(PosNorFileDir);
    this->_readSliceData(LayerFileDir);
    this->_readCncData(1);

    return wayPointFileCell.size();
}

void GcodeGeneration::_getFileName_Set(std::string dirctory, std::vector<std::string>& fileNameCell) {

    if (fileNameCell.empty() == false) return;

    DIR* dp;
    struct dirent* ep;
    std::string fullDir = dirctory;
    //cout << fullDir << endl;
    dp = opendir(fullDir.c_str());
    //dp = opendir("../Waypoints");

    if (dp != NULL) {
        while (ep = readdir(dp)) {
            //cout << ep->d_name << endl;
            if ((std::string(ep->d_name) != ".") && (std::string(ep->d_name) != "..")) {
                //cout << ep->d_name << endl;
                fileNameCell.push_back(std::string(ep->d_name));
            }
        }
        (void)closedir(dp);
    }
    else {
        perror("Couldn't open the directory");
    }
    //resort the files with nature order
    std::vector<std::string> copy_fileNameCell = fileNameCell;
    for (int i = 0; i < copy_fileNameCell.size(); i++) {

        //std::cout << copy_fileNameCell[i] << std::endl;

        int layerFile_Ind = -1;
        std::string::size_type supportFlag = copy_fileNameCell[i].find("S");
        if (supportFlag == std::string::npos) {

            std::string::size_type p = copy_fileNameCell[i].find('.');
            layerFile_Ind = stoi(copy_fileNameCell[i].substr(0, p));

        }
        else {
            std::string::size_type q = copy_fileNameCell[i].find('.');
            layerFile_Ind = stoi(copy_fileNameCell[i].substr(0, q - 1));
        }
        fileNameCell[layerFile_Ind] = copy_fileNameCell[i];
    }
}

void GcodeGeneration::_modifyCoord(QMeshPatch* patchFile,bool Yup2Zup) {

    for (GLKPOSITION Pos = patchFile->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patchFile->GetNodeList().GetNext(Pos);

        double xx, yy, zz, nx, ny, nz;
        Node->GetCoord3D(xx, yy, zz);
        Node->GetNormal(nx, ny, nz);

        if (Yup2Zup) {

            double tempPosYZ = yy;
            double tempNorYZ = ny;

            yy = -zz;
            zz = tempPosYZ;

            ny = -nz;
            nz = tempNorYZ;
        }

        Node->SetCoord3D_last(xx, yy, zz);// base data of MoveModel
        Node->SetNormal_last(nx, ny, nz);
        Node->SetCoord3D(xx, yy, zz);
        Node->SetNormal(nx, ny, nz);
    }
}

void GcodeGeneration::_readWayPointData(std::string packName) {

    //read waypoint files and build mesh_patches
    char filename[1024];
    for (int i = 0; i < wayPointFileCell.size(); i++) {

        sprintf(filename, "%s%s%s", packName.c_str(), "/", wayPointFileCell[i].data());

        QMeshPatch* waypoint = new QMeshPatch;
        waypoint->SetIndexNo(m_Waypoints->GetMeshList().GetCount()); //index begin from 0
        m_Waypoints->GetMeshList().AddTail(waypoint);
        waypoint->patchName = wayPointFileCell[i].data();

        // isSupportLayer
        std::string::size_type supportFlag = wayPointFileCell[i].find("S");
        if (supportFlag == std::string::npos)	waypoint->is_SupportLayer = false;
        else	waypoint->is_SupportLayer = true;

        waypoint->inputPosNorFile(filename, m_Yup2Zup);
        //if (m_modelName == "yoga_icra") this->cleanInputNormal(waypoint);
        this->_modifyCoord(waypoint, m_Yup2Zup); // give last normal and position info
    }

    // MoveModel(Xmove, Ymove, Zmove);
    for (GLKPOSITION patchPos = m_Waypoints->GetMeshList().GetHeadPosition(); patchPos;) {
        QMeshPatch* waypoint_patch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(patchPos);
        for (GLKPOSITION Pos = waypoint_patch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)waypoint_patch->GetNodeList().GetNext(Pos);

            double xx, yy, zz, nx, ny, nz;
            Node->GetCoord3D_last(xx, yy, zz);
            Node->GetNormal_last(nx, ny, nz);

            xx += m_Xmove;	yy += m_Ymove;	zz += m_Zmove;

            Node->m_printPos << xx, yy, zz;
            Node->m_printNor << -nx, -ny, -nz;
            //if (m_modelName == "yoga_icra") Node->m_printNor << nx, ny, nz; // for yoga
            Node->m_printNor = Node->m_printNor.normalized();

            Node->SetCoord3D(xx, yy, zz);
            Node->SetCoord3D_last(xx, yy, zz);

            Node->SetNormal(Node->m_printNor[0], Node->m_printNor[1], Node->m_printNor[2]);
            Node->SetNormal_last(Node->m_printNor[0], Node->m_printNor[1], Node->m_printNor[2]);

        }
    }
    std::cout << "------------------------------------------- WayPoints Load Finish!" << std::endl;
}

void GcodeGeneration::_readSliceData(std::string packName) {

    //read slice files and build mesh_patches
    char filename[1024];
    for (int i = 0; i < sliceSetFileCell.size(); i++) {

        sprintf(filename, "%s%s%s", packName.c_str(), "/", sliceSetFileCell[i].data());

        QMeshPatch* layers = new QMeshPatch;
        layers->SetIndexNo(m_Slices->GetMeshList().GetCount()); //index begin from 0
        m_Slices->GetMeshList().AddTail(layers);
        layers->patchName = sliceSetFileCell[i].data();

        // isSupportLayer
        std::string::size_type supportFlag = sliceSetFileCell[i].find("S");
        if (supportFlag == std::string::npos)	layers->is_SupportLayer = false;
        else	layers->is_SupportLayer = true;

        layers->inputOBJFile(filename);

        this->_modifyCoord(layers, m_Yup2Zup);
    }

    // MoveModel(Xmove, Ymove, Zmove);
    for (GLKPOSITION patchPos = m_Slices->GetMeshList().GetHeadPosition(); patchPos;) {
        QMeshPatch* slice_patch = (QMeshPatch*)m_Slices->GetMeshList().GetNext(patchPos);
        for (GLKPOSITION Pos = slice_patch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)slice_patch->GetNodeList().GetNext(Pos);

            double xx, yy, zz, nx, ny, nz;
            Node->GetCoord3D_last(xx, yy, zz);

            xx += m_Xmove;	yy += m_Ymove;	zz += m_Zmove;

            Node->SetCoord3D(xx, yy, zz);
        }
    }
    std::cout << "------------------------------------------- Slices Load Finish!" << std::endl;
}

void GcodeGeneration::_readCncData(int input_Step) {

    //avoid repeatly read files
    if (m_CncPart->GetMeshList().GetCount() > 10) { 
        std::cout << "there are already cnc file. skip read cnc data." << std::endl;
        return;
    }

    std::vector<std::string> CNCfileSet;
    if (input_Step == 1) {
        if (m_modelName == "wing_mirror_step2" || m_modelName == "wing_mirror_step4")
            CNCfileSet = { "c_axis", "nozzle_4C", "baseModel_5AM", "fixture_5AM"};
        else if (m_modelName == "wing_mirror_step3")
            CNCfileSet = { "c_axis", "nozzle_4C", "baseModel_self", "fixture_5AM_self" };
        else
            CNCfileSet = { "c_axis", "nozzle_4C" };
    }
    else if(input_Step == 2)
        CNCfileSet = { "frame", "x_axis", "y_axis", "z_axis", "b_axis", "nozzle" };
    else 
        CNCfileSet = { "frame", "x_axis", "y_axis", "z_axis_newConfig", "c_axis_newConfig",
        "b_axis_newConfig", "nozzle_newConfig" };

    //read CNC files and build mesh_patches
    char filename[1024];

    for (int i = 0; i < CNCfileSet.size(); i++) {
        sprintf(filename, "%s%s%s", "../DataSet/CNC_MODEL/cnc_assembly_", CNCfileSet[i].c_str(), ".obj");
        std::cout << "input " << CNCfileSet[i].data() << " from: " << filename << std::endl;

        QMeshPatch* cncPatch = new QMeshPatch;
        cncPatch->SetIndexNo(m_CncPart->GetMeshList().GetCount()); //index begin from 0
        m_CncPart->GetMeshList().AddTail(cncPatch);
        cncPatch->inputOBJFile(filename);
        cncPatch->patchName = CNCfileSet[i].data();
        if(input_Step == 1)
            cncPatch->drawThisPatch = true;
        //this->_modifyCoord(cncPatch, m_Yup2Zup);
    }
}

void GcodeGeneration::updateParameter(int FromIndex, int ToIndex, double lambdaValue,
    bool varyDistance_switch, bool varyHeight_switch, bool varyWidth_switch, bool outputDHW) {

    m_FromIndex = FromIndex;
    m_ToIndex = ToIndex;
    m_lambdaValue = lambdaValue;
    m_varyDistance_switch = varyDistance_switch;
    m_varyHeight_switch = varyHeight_switch;
    m_varyWidth_switch = varyWidth_switch;
    m_outputDHW = outputDHW;
}

void GcodeGeneration::calDHW() {

    if (m_varyDistance_switch)
        this->_cal_Dist();

    this->_initialSmooth(10);

    if(m_varyHeight_switch)
        this->_cal_Height();

    if(m_varyWidth_switch)
        this->_cal_Width();

    if(m_outputDHW)
        this->_output_DHW();
}

void GcodeGeneration::_cal_Dist() {

    std::cout << "------------------------------------------- Waypoint Distance Calculation Running ..." << std::endl;
    long time = clock();

    double initial_largeJ_Length = 4.0;
    double support_largerJ_Length = 6.0;
    std::cout << "--> initial_largeJ_Length: " << initial_largeJ_Length
        << "\n--> support_largerJ_Length: " << support_largerJ_Length << std::endl;
    double largeJ_Length = 0.0;// install the above Length Value

#pragma omp parallel
    {
#pragma omp for  
        for (int omptime = 0; omptime < Core; omptime++) {

            for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
                QMeshPatch* layer = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

                if (layer->GetIndexNo() < m_FromIndex || layer->GetIndexNo() > m_ToIndex) continue;

                if (layer->GetIndexNo() % Core != omptime) continue;

                if (layer->is_SupportLayer) { largeJ_Length = support_largerJ_Length; }
                else { largeJ_Length = initial_largeJ_Length; }

                for (GLKPOSITION nodePos = layer->GetNodeList().GetHeadPosition(); nodePos;) {
                    QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(nodePos);

                    double D = 0.0;
                    int lines = layer->GetNodeNumber();
                    if (Node->GetIndexNo() == (lines - 1)) { D = 0.0; }
                    else {

                        GLKPOSITION nextPos = layer->GetNodeList().Find(Node)->next;
                        QMeshNode* nextNode = (QMeshNode*)layer->GetNodeList().GetAt(nextPos);

                        D = (Node->m_printPos - nextNode->m_printPos).norm();

                        if (D > largeJ_Length) {
                            D = 0.0;								// inject the D to the Node/startPnt of Edge
                            Node->Jump_preSecEnd = true;			// end of prev section
                            nextNode->Jump_nextSecStart = true;		// start of next section
                        }
                    }
                    Node->m_DHW(0) = D;
                }
            }
        }
    }
    printf("TIMER -- Distance Calculation takes %ld ms.\n", clock() - time);
    std::cout << "------------------------------------------- Waypoint Distance Calculation Finish!\n" << std::endl;
}

void GcodeGeneration::_initialSmooth(int loopTime) {

    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        if (WayPointPatch->GetIndexNo() < m_FromIndex || WayPointPatch->GetIndexNo() > m_ToIndex) continue;

        int patch_NodeNum = WayPointPatch->GetNodeNumber();
        std::vector<bool> fix_Flag(patch_NodeNum);
        std::vector<Eigen::Vector3d> NodeNormal_temp(patch_NodeNum); // [Nx Ny Nz fix_flag]

        for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

            /*fixed at first / end / jump_start / jump_end points*/
            if (Node->GetIndexNo() == 0 || Node->GetIndexNo() == patch_NodeNum - 1
                || Node->Jump_preSecEnd || Node->Jump_nextSecStart) {
                fix_Flag[Node->GetIndexNo()] = true;
            }
            else { fix_Flag[Node->GetIndexNo()] = false; }

            NodeNormal_temp[Node->GetIndexNo()] = Node->m_printNor;

        }

        //smooth normal by (n-1) + X*(n) + (n+1)
        for (int loop = 0; loop < loopTime; loop++) {
            for (int i = 0; i < fix_Flag.size(); i++) {

                if (fix_Flag[i] == false) {
                    NodeNormal_temp[i] = (NodeNormal_temp[i - 1] + 0.5 * NodeNormal_temp[i] + NodeNormal_temp[i + 1]).normalized();
                }

            }
        }

        for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

            Node->m_printNor = NodeNormal_temp[Node->GetIndexNo()];
            Node->SetNormal(Node->m_printNor(0), Node->m_printNor(1), Node->m_printNor(2));
            Node->m_printNor_4_robot = Node->m_printNor;
        }
    }
    std::cout << "------------------------------------------- Initial Smooth Finish!\n" << std::endl;
}

void GcodeGeneration::_cal_Height() {

    std::cout << "------------------------------------------- Waypoint Height Calculation Running ..." << std::endl;
    long time = clock();

    // get the patch polygenMesh_PrintPlatform
    QMeshPatch* patch_PrintPlatform = NULL;
    for (GLKPOSITION posMesh = m_CncPart->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
        QMeshPatch* thisPatch = (QMeshPatch*)m_CncPart->GetMeshList().GetNext(posMesh);

        if (m_modelName == "wing_mirror_step3"){
            if (thisPatch->patchName == "baseModel_self") {
                patch_PrintPlatform = thisPatch;
                break;
            }
        }
        else if (m_modelName == "wing_mirror_step2" || m_modelName == "wing_mirror_step4") {
            if (thisPatch->patchName == "baseModel_5AM") {
                patch_PrintPlatform = thisPatch;
                break;
            }
        }
        else {
            if (thisPatch->patchName == "c_axis") {
                patch_PrintPlatform = thisPatch;
                break;
            }
        }

    }

    if (patch_PrintPlatform == NULL) {
        std::cout << "patch_PrintPlatform is NULL, please check." << std::endl;
        return;
    }

    //// Temp move the plateform to model bottom for height calculaltion
    //for (GLKPOSITION Pos = patch_PrintPlatform->GetNodeList().GetHeadPosition(); Pos;) {
    //    QMeshNode* platform_Node = (QMeshNode*)patch_PrintPlatform->GetNodeList().GetNext(Pos);

    //    double xx, yy, zz;
    //    platform_Node->GetCoord3D(xx, yy, zz);
    //    platform_Node->SetCoord3D(xx + m_Xmove, yy + m_Ymove, zz + m_Zmove);

    //}

#pragma omp parallel
    {
#pragma omp for  
        for (int omptime = 0; omptime < Core; omptime++) {

            // topLayer --> layer on the highest place [travel head to tail]
            for (GLKPOSITION Pos = m_Slices->GetMeshList().GetHeadPosition(); Pos;) {
                QMeshPatch* topLayer = (QMeshPatch*)m_Slices->GetMeshList().GetNext(Pos); // order: get data -> pnt move

                if (topLayer->GetIndexNo() < m_FromIndex || topLayer->GetIndexNo() > m_ToIndex) continue;

                if (topLayer->GetIndexNo() % Core != omptime) continue;

                std::vector<QMeshPatch*> bottomLayers;

                bottomLayers.push_back(patch_PrintPlatform);
                // construct a bottomLayers[i] to store the point of bottom layers for every toplayer
                for (GLKPOSITION beforePos = m_Slices->GetMeshList().Find(topLayer)->prev; beforePos;) {
                    QMeshPatch* beforePatch = (QMeshPatch*)m_Slices->GetMeshList().GetPrev(beforePos);

                    bottomLayers.push_back(beforePatch);
                    if (bottomLayers.size() > layerNum) break;
                }


                //--build PQP model
                std::vector<PQP_Model*> bLayerPQP;
                bLayerPQP.resize(bottomLayers.size());
                for (int i = 0; i < bottomLayers.size(); i++) {
                    if (bottomLayers[i]->GetNodeNumber() < 3) continue;
                    // build PQP model for bottom layers
                    PQP_Model* pqpModel = new PQP_Model();
                    pqpModel->BeginModel();  int index = 0;
                    PQP_REAL p1[3], p2[3], p3[3];

                    for (GLKPOSITION Pos = bottomLayers[i]->GetFaceList().GetHeadPosition(); Pos;) {
                        QMeshFace* Face = (QMeshFace*)bottomLayers[i]->GetFaceList().GetNext(Pos);

                        Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
                        Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
                        Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);

                        pqpModel->AddTri(p1, p2, p3, index);
                        index++;

                    }
                    pqpModel->EndModel();
                    bLayerPQP[i] = pqpModel;
                }//--build PQP model END

                int layerIndex = topLayer->GetIndexNo();

                GLKPOSITION WayPointPatch_Pos = m_Waypoints->GetMeshList().FindIndex(layerIndex);
                QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetAt(WayPointPatch_Pos);

                for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
                    QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

                    double minHeight = 99999.99;
                    for (int i = 0; i < bottomLayers.size(); i++) {
                        if (bottomLayers[i]->GetNodeNumber() < 3) continue;

                        PQP_DistanceResult dres; dres.last_tri = bLayerPQP[i]->last_tri;
                        PQP_REAL p[3];
                        p[0] = Node->m_printPos(0); p[1] = Node->m_printPos(1); p[2] = Node->m_printPos(2);
                        PQP_Distance(&dres, bLayerPQP[i], p, 0.0, 0.0);

                        double Height = dres.Distance(); // Height of this layer
                        //int minTriIndex = dres.last_tri->id;	// closest triangle
                        if (minHeight > Height) minHeight = Height;
                    }
                    //cout << minHeight << endl;
                    Node->m_DHW(1) = minHeight;//if (WayPointPatch->GetIndexNo() == 0) Node->m_DHW(1) -= m_Zmove;

                    //retouch layer height
                    double x = Node->m_DHW(1);
                    double ratio = 1.0;
                    
                    if (x < 0.4)
                        ratio = 1.0;
                    else if (x < 0.8)
                        ratio = (1.4 - 1.0) / (0.8 - 0.4) * x + 0.6;
                    else
                        ratio = 1.4;
                    

                    //if (x < 0.4)
                    //    ratio = 0.8;
                    //else if (x < 0.8)
                    //    ratio = 0.8 * exp(2.8 * (x - 0.4));
                    //else
                    //    ratio = 2.5;


                    Node->m_DHW(1) = ratio * x;
                }

                //	free memory
                for (int i = 0; i < bottomLayers.size(); i++) { delete bLayerPQP[i]; }
            }
        }
    }

    // return the plateform to zero bottom
    //for (GLKPOSITION Pos = patch_PrintPlatform->GetNodeList().GetHeadPosition(); Pos;) {
    //    QMeshNode* platform_Node = (QMeshNode*)patch_PrintPlatform->GetNodeList().GetNext(Pos);

    //    double xx, yy, zz;
    //    platform_Node->GetCoord3D(xx, yy, zz);
    //    platform_Node->SetCoord3D(xx - m_Xmove, yy - m_Ymove, zz - m_Zmove);

    //}

    std::printf("TIMER -- Height Calculation takes %ld ms.\n", clock() - time);
    std::cout << "------------------------------------------- Waypoint Height Calculation Finish!\n" << std::endl;
}

void GcodeGeneration::_cal_Width() {

    std::cout << "------------------------------------------- Waypoint Width Calculation Running ..." << std::endl;
    long time = clock();

#pragma omp parallel
    {
#pragma omp for  
        for (int omptime = 0; omptime < Core; omptime++) {

            for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
                QMeshPatch* layer = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

                if (layer->GetIndexNo() < m_FromIndex || layer->GetIndexNo() > m_ToIndex) continue;

                if (layer->GetIndexNo() % Core != omptime) continue;

                for (GLKPOSITION nodePos = layer->GetNodeList().GetHeadPosition(); nodePos;) {
                    QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(nodePos);

                    double W = 0.0; double minW = 99999.9;

                    if (Node->GetIndexNo() == 0) {
                        for (GLKPOSITION otherNodePos = layer->GetNodeList().GetHeadPosition(); otherNodePos;) {
                            QMeshNode* otherNode = (QMeshNode*)layer->GetNodeList().GetNext(otherNodePos);

                            if (otherNode->GetIndexNo() != Node->GetIndexNo() && otherNode->GetIndexNo() != (Node->GetIndexNo() + 1)) {

                                W = (Node->m_printPos - otherNode->m_printPos).norm();
                                if (W < minW) minW = W;
                            }

                        }
                    }
                    else if (Node->GetIndexNo() == (layer->GetNodeNumber() - 1)) {
                        for (GLKPOSITION otherNodePos = layer->GetNodeList().GetHeadPosition(); otherNodePos;) {
                            QMeshNode* otherNode = (QMeshNode*)layer->GetNodeList().GetNext(otherNodePos);

                            if (otherNode->GetIndexNo() != Node->GetIndexNo() && otherNode->GetIndexNo() != (Node->GetIndexNo() - 1)) {

                                W = (Node->m_printPos - otherNode->m_printPos).norm();
                                if (W < minW) minW = W;
                            }

                        }
                    }
                    else {
                        for (GLKPOSITION otherNodePos = layer->GetNodeList().GetHeadPosition(); otherNodePos;) {
                            QMeshNode* otherNode = (QMeshNode*)layer->GetNodeList().GetNext(otherNodePos);

                            if (otherNode->GetIndexNo() != Node->GetIndexNo() && otherNode->GetIndexNo() != (Node->GetIndexNo() - 1)
                                && otherNode->GetIndexNo() != (Node->GetIndexNo() + 1)) {

                                W = (Node->m_printPos - otherNode->m_printPos).norm();
                                if (W < minW) minW = W;
                            }

                        }
                    }

                    Node->m_DHW(2) = minW;
                }
            }
        }
    }
    std::printf("TIMER -- Width Calculation takes %ld ms.\n", clock() - time);
    std::cout << "------------------------------------------- Waypoint Width Calculation Finish!\n" << std::endl;
}

void GcodeGeneration::_output_DHW() {

    if (m_outputDHW == false) return;
    std::string testFile_Dir = "../DataSet/fabricationTest/test_DHW/";
    this->_remove_allFile_in_Dir(testFile_Dir);

    std::cout << "------------------------------------------- Waypoint DHW Data Writing ..." << std::endl;

    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        if (WayPointPatch->GetIndexNo() < m_FromIndex || WayPointPatch->GetIndexNo() > m_ToIndex) continue;

        char targetFilename[1024];

        if (WayPointPatch->is_SupportLayer == true) {
            std::sprintf(targetFilename, "%s%d%s", testFile_Dir.c_str(), WayPointPatch->GetIndexNo(), "S.txt");
        }
        else {
            std::sprintf(targetFilename, "%s%d%s", testFile_Dir.c_str(), WayPointPatch->GetIndexNo(), ".txt");
        }

        // cout << targetFilename << endl;

        FILE* fp = fopen(targetFilename, "w");
        if (!fp)	return;

        // danger check: The point is JumpEnd and also JumpStart.
        for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

            if (Node->Jump_preSecEnd == true && Node->Jump_nextSecStart == true) {
                std::cout << "The point is JumpEnd and also JumpStart, please check" << std::endl; // important issues!
                std::cout << "The Layer Index is " << WayPointPatch->GetIndexNo() << std::endl;
                std::cout << "The Layer name is " << WayPointPatch->patchName << std::endl;
            }

            fprintf(fp, "%f %f %f %d\n", Node->m_DHW(0), Node->m_DHW(1), Node->m_DHW(2), Node->Jump_preSecEnd);
        }

        fclose(fp);

        //cout << "-------------------- Layer " << WayPointPatch->GetIndexNo() << " LayerHeight Data Write" << endl;
    }
    std::cout << "\n------------------------------------------- Waypoint DHW Data Write Finish!\n" << std::endl;

}

int GcodeGeneration::_remove_allFile_in_Dir(std::string dirPath) {

    struct _finddata_t fb;   //find the storage structure of the same properties file.
    std::string path;
    intptr_t    handle;
    int  resultone;
    int   noFile;            // the tag for the system's hidden files

    noFile = 0;
    handle = 0;

    path = dirPath + "/*";

    handle = _findfirst(path.c_str(), &fb);

    //find the first matching file
    if (handle != -1)
    {
        //find next matching file
        while (0 == _findnext(handle, &fb))
        {
            // "." and ".." are not processed
            noFile = strcmp(fb.name, "..");

            if (0 != noFile)
            {
                path.clear();
                path = dirPath + "/" + fb.name;

                //fb.attrib == 16 means folder
                if (fb.attrib == 16)
                {
                    _remove_allFile_in_Dir(path);
                }
                else
                {
                    //not folder, delete it. if empty folder, using _rmdir instead.
                    remove(path.c_str());
                }
            }
        }
        // close the folder and delete it only if it is closed. For standard c, using closedir instead(findclose -> closedir).
        // when Handle is created, it should be closed at last.
        _findclose(handle);
        return 0;
    }
}

void GcodeGeneration::singularityOpt() {

    std::cout << "------------------------------------------- XYZBCE Calculation running ... " << std::endl;
    long time = clock();

#pragma omp parallel
    {
#pragma omp for  
        for (int omptime = 0; omptime < Core; omptime++) {

            for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
                QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

                if (WayPointPatch->GetIndexNo() < m_FromIndex || WayPointPatch->GetIndexNo() > m_ToIndex) continue;
                if (WayPointPatch->GetIndexNo() % Core != omptime) continue;

                std::vector<QMeshPatch*> layerJumpPatchSet = _getJumpSection_patchSet(WayPointPatch);

                Eigen::RowVector2d prevBC = { 0.0,0.0 };
                // give the message of BC for the first Node (only one)
                for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
                    QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

                    // solve 1
                    prevBC(0) = ROTATE_TO_DEGREE(-_safe_acos(Node->m_printNor(2)));
                    prevBC(1) = ROTATE_TO_DEGREE(atan2(Node->m_printNor(0), Node->m_printNor(1)));

                    // solve 2
                    double C2temp = prevBC(1) + 180.0;
                    if (C2temp > 180.0)	C2temp -= 360.0; // control the range of C2 into the (-180,180]

                    //// prevBC always BC1
                    //if (fabs(C2temp) < fabs(prevBC(1))) {
                    //    prevBC(0) = -prevBC(0);
                    //    prevBC(1) = C2temp;
                    //}

                    break;
                }
                //cout << "prevBC: " << prevBC << endl;

                for (int Index = 0; Index < layerJumpPatchSet.size(); Index++) {
                    //1.0 find the singularity waypoints
                    _markSingularNode(layerJumpPatchSet[Index]);
                    //1.1 filter single singular waypoint (XXOXX -> XXXXX)
                    _filterSingleSingularNode(layerJumpPatchSet[Index]);

                    Eigen::MatrixXd sectionTable, B1C1table, B2C2table;
                    //2.0 get the range of singularity Sections
                    _getSingularSec(layerJumpPatchSet[Index], sectionTable);
                    //2.1 project normal to the singular region boundary and check
                    _projectAnchorPoint(layerJumpPatchSet[Index]);

                    //3. calculate the 2 solves baced on kinematics of CNC
                    _getBCtable2(layerJumpPatchSet[Index], B1C1table, B2C2table);
                    //4. Main singularity optimization algorithm
                    _motionPlanning3(layerJumpPatchSet[Index], sectionTable, B1C1table, B2C2table, prevBC);
                    //5. reset steps: CNC XYZ calculation and E(=DHW) calculation
                    _getXYZ(layerJumpPatchSet[Index]);
                    _calDHW2E(layerJumpPatchSet[Index], true);
                }

                //aim to eliminate the -pi to pi sharp change
                _optimizationC(WayPointPatch);

                // from delta_E of each point to E in Gcode
                double E = 0.0;
                for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
                    QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);
                    E = E + Node->m_XYZBCE(5);
                    Node->m_XYZBCE(5) = E;
                }

                // get insert waypoints
                for (int Index = 0; Index < layerJumpPatchSet.size(); Index++) {

                    bool is_print = true;
                    if (layerJumpPatchSet[Index]->GetNodeNumber() < toolpath_filter_threshold) {
                        is_print = false; continue;
                    }
                    // get insert num for all of points
                    for (GLKPOSITION Pos = layerJumpPatchSet[Index]->GetNodeList().GetHeadPosition(); Pos;) {
                        QMeshNode* Node = (QMeshNode*)layerJumpPatchSet[Index]->GetNodeList().GetNext(Pos);

                        // ignore the last point(without next)
                        if (Node->Jump_SecIndex == (layerJumpPatchSet[Index]->GetNodeNumber() - 1)) continue;

                        GLKPOSITION nextPos = layerJumpPatchSet[Index]->GetNodeList().Find(Node)->next;
                        QMeshNode* nextNode = (QMeshNode*)layerJumpPatchSet[Index]->GetNodeList().GetAt(nextPos);

                        // the diff C of two neighbor node is too large
                        double delataC_deg = fabs(Node->m_XYZBCE(4) - nextNode->m_XYZBCE(4));

                        if (delataC_deg > maxDeltaC) {

                            std::cout << "\n\n\n I am here \n\n\n" << std::endl;

                            Node->insertNodesNum = int(delataC_deg / maxDeltaC) + 1;

                            std::cout << "Node->insertNodesNum" << Node->insertNodesNum << std::endl;

                            Node->insertNodesInfo.resize(Node->insertNodesNum);
                            for (int i = 0; i < Node->insertNodesInfo.size(); i++) {

                                int alpha = i + 1;

                                Eigen::Vector3d node_printPos = Node->m_printPos;
                                //Node->GetCoord3D_last(node_printPos[0], node_printPos[1], node_printPos[2]);
                                Eigen::Vector3d next_node_printPos = nextNode->m_printPos;
                                //nextNode->GetCoord3D_last(next_node_printPos[0], next_node_printPos[1], next_node_printPos[2]);

                                Eigen::Vector3d iNode_coord3D = Eigen::Vector3d::Zero();
                                iNode_coord3D = (1.0 - alpha / double(Node->insertNodesNum + 1)) * node_printPos +
                                    (alpha / double(Node->insertNodesNum + 1)) * next_node_printPos;

                                double iNode_B = (1.0 - alpha / double(Node->insertNodesNum + 1)) * Node->m_XYZBCE(3) +
                                    (alpha / double(Node->insertNodesNum + 1)) * nextNode->m_XYZBCE(3);

                                double iNode_C = (1.0 - alpha / double(Node->insertNodesNum + 1)) * Node->m_XYZBCE(4) +
                                    (alpha / double(Node->insertNodesNum + 1)) * nextNode->m_XYZBCE(4);

                                double iNode_E = (1.0 - alpha / double(Node->insertNodesNum + 1)) * Node->m_XYZBCE(5) +
                                    (alpha / double(Node->insertNodesNum + 1)) * nextNode->m_XYZBCE(5);

                                double iNode_B_rad = DEGREE_TO_ROTATE(iNode_B);
                                double iNode_C_rad = DEGREE_TO_ROTATE(iNode_C);

                                double iNode_X = iNode_coord3D[0] * cos(iNode_C_rad) - iNode_coord3D[1] * sin(iNode_C_rad);
                                double iNode_Y = iNode_coord3D[0] * sin(iNode_C_rad) + iNode_coord3D[1] * cos(iNode_C_rad) - h * sin(iNode_B_rad);
                                double iNode_Z = iNode_coord3D[2] - h * (1- cos(iNode_B_rad));

                                Node->insertNodesInfo[i] = Eigen::MatrixXd::Zero(1, 6);

                                Node->insertNodesInfo[i] << iNode_X, iNode_Y, iNode_Z, iNode_B, iNode_C, iNode_E;
                            }

                        }
                    }
                }
            }
        }
    }

    std::cout << "-------------------------------------------" << std::endl;
    std::printf("TIMER -- XYZBCE Calculation takes %ld ms.\n", clock() - time);
    std::cout << "------------------------------------------- XYZBCE Calculation Finish!\n " << std::endl;

    this->_verifyPosNor();
}

void GcodeGeneration::singularityOpt_newConfig() {

    std::cout << "------------------------------------------- XYZBCE Calculation running ... " << std::endl;
    long time = clock();

#pragma omp parallel
    {
#pragma omp for  
        for (int omptime = 0; omptime < Core; omptime++) {

            for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
                QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

                if (WayPointPatch->GetIndexNo() < m_FromIndex || WayPointPatch->GetIndexNo() > m_ToIndex) continue;
                if (WayPointPatch->GetIndexNo() % Core != omptime) continue;

                std::vector<QMeshPatch*> layerJumpPatchSet = _getJumpSection_patchSet(WayPointPatch);

                Eigen::RowVector2d prevBC = { 0.0,0.0 };
                // give the message of BC for the first Node (only one)
                for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
                    QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

                    // solve 1
                    prevBC(0) = ROTATE_TO_DEGREE(-_safe_acos(Node->m_printNor(2)));
                    prevBC(1) = ROTATE_TO_DEGREE(-atan2(Node->m_printNor(0), Node->m_printNor(1)));

                    // solve 2
                    double C2temp = prevBC(1) + 180.0;
                    if (C2temp > 180.0)	C2temp -= 360.0; // control the range of C2 into the (-180,180]

                    // prevBC always min
                    if (fabs(C2temp) < fabs(prevBC(1))) {
                        prevBC(0) = -prevBC(0);
                        prevBC(1) = C2temp;
                    }

                    break;
                }
                //cout << "prevBC: " << prevBC << endl;

                for (int Index = 0; Index < layerJumpPatchSet.size(); Index++) {
                    //1.0 find the singularity waypoints
                    _markSingularNode(layerJumpPatchSet[Index]);
                    //1.1 filter single singular waypoint (XXOXX -> XXXXX)
                    _filterSingleSingularNode(layerJumpPatchSet[Index]);

                    Eigen::MatrixXd sectionTable, B1C1table, B2C2table;
                    //2.0 get the range of singularity Sections
                    _getSingularSec(layerJumpPatchSet[Index], sectionTable);
                    //2.1 project normal to the singular region boundary and check
                    _projectAnchorPoint(layerJumpPatchSet[Index]);

                    //3. calculate the 2 solves baced on kinematics of CNC
                    _getBCtable2_newConfig(layerJumpPatchSet[Index], B1C1table, B2C2table);
                    //4. Main singularity optimization algorithm
                    _motionPlanning3(layerJumpPatchSet[Index], sectionTable, B1C1table, B2C2table, prevBC);
                    //5. reset steps: CNC XYZ calculation and E(=DHW) calculation
                    _getXYZ_newConfig(layerJumpPatchSet[Index]);
                    _calDHW2E(layerJumpPatchSet[Index], true);
                }

                //aim to eliminate the -pi to pi sharp change
                _optimizationC(WayPointPatch);
                _limit_C_range(WayPointPatch);

                // from delta_E of each point to E in Gcode
                double E = 0.0;
                for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
                    QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);
                    E = E + Node->m_XYZBCE(5);
                    Node->m_XYZBCE(5) = E;
                }

                // get insert waypoints (I don't use it now)
                if(false){
                    for (int Index = 0; Index < layerJumpPatchSet.size(); Index++) {

                        bool is_print = true;
                        if (layerJumpPatchSet[Index]->GetNodeNumber() < toolpath_filter_threshold) {
                            is_print = false; continue;
                        }
                        // get insert num for all of points
                        for (GLKPOSITION Pos = layerJumpPatchSet[Index]->GetNodeList().GetHeadPosition(); Pos;) {
                            QMeshNode* Node = (QMeshNode*)layerJumpPatchSet[Index]->GetNodeList().GetNext(Pos);

                            // ignore the last point(without next)
                            if (Node->Jump_SecIndex == (layerJumpPatchSet[Index]->GetNodeNumber() - 1)) continue;

                            GLKPOSITION nextPos = layerJumpPatchSet[Index]->GetNodeList().Find(Node)->next;
                            QMeshNode* nextNode = (QMeshNode*)layerJumpPatchSet[Index]->GetNodeList().GetAt(nextPos);

                            // the diff C of two neighbor node is too large
                            double delataC_deg = fabs(Node->m_XYZBCE(4) - nextNode->m_XYZBCE(4));

                            if (delataC_deg > maxDeltaC) {

                                std::cout << "\n\n\n I am here \n\n\n" << std::endl;

                                Node->insertNodesNum = int(delataC_deg / maxDeltaC) + 1;

                                std::cout << "Node->insertNodesNum" << Node->insertNodesNum << std::endl;

                                Node->insertNodesInfo.resize(Node->insertNodesNum);
                                for (int i = 0; i < Node->insertNodesInfo.size(); i++) {

                                    int alpha = i + 1;

                                    Eigen::Vector3d node_printPos = Node->m_printPos;
                                    //Node->GetCoord3D_last(node_printPos[0], node_printPos[1], node_printPos[2]);
                                    Eigen::Vector3d next_node_printPos = nextNode->m_printPos;
                                    //nextNode->GetCoord3D_last(next_node_printPos[0], next_node_printPos[1], next_node_printPos[2]);

                                    Eigen::Vector3d iNode_coord3D = Eigen::Vector3d::Zero();
                                    iNode_coord3D = (1.0 - alpha / double(Node->insertNodesNum + 1)) * node_printPos +
                                        (alpha / double(Node->insertNodesNum + 1)) * next_node_printPos;

                                    double iNode_B = (1.0 - alpha / double(Node->insertNodesNum + 1)) * Node->m_XYZBCE(3) +
                                        (alpha / double(Node->insertNodesNum + 1)) * nextNode->m_XYZBCE(3);

                                    double iNode_C = (1.0 - alpha / double(Node->insertNodesNum + 1)) * Node->m_XYZBCE(4) +
                                        (alpha / double(Node->insertNodesNum + 1)) * nextNode->m_XYZBCE(4);

                                    double iNode_E = (1.0 - alpha / double(Node->insertNodesNum + 1)) * Node->m_XYZBCE(5) +
                                        (alpha / double(Node->insertNodesNum + 1)) * nextNode->m_XYZBCE(5);

                                    double iNode_B_rad = DEGREE_TO_ROTATE(iNode_B);
                                    double iNode_C_rad = DEGREE_TO_ROTATE(iNode_C);

                                    double iNode_X = iNode_coord3D[0] - r * cos(iNode_C_rad) + h * sin(iNode_C_rad) * sin(iNode_B_rad) + r;
                                    double iNode_Y = iNode_coord3D[1] - r * sin(iNode_C_rad) - h * cos(iNode_C_rad) * sin(iNode_B_rad);
                                    double iNode_Z = iNode_coord3D[2] + h * cos(iNode_B_rad) - h;

                                    Node->insertNodesInfo[i] = Eigen::MatrixXd::Zero(1, 6);

                                    Node->insertNodesInfo[i] << iNode_X, iNode_Y, iNode_Z, iNode_B, iNode_C, iNode_E;
                                }

                            }
                        }
                    }
                }
            }
        }
    }

    std::cout << "-------------------------------------------" << std::endl;
    std::printf("TIMER -- XYZBCE Calculation takes %ld ms.\n", clock() - time);
    std::cout << "------------------------------------------- XYZBCE Calculation Finish!\n " << std::endl;

    this->_verifyPosNor_newConfig();
}

std::vector<QMeshPatch*> GcodeGeneration::_getJumpSection_patchSet(QMeshPatch* patch) {

    // Get the Jump section Num
    int JumpPatchNum = 1;
    for (GLKPOSITION Pos_Node = patch->GetNodeList().GetHeadPosition(); Pos_Node;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos_Node);

        if (Node->Jump_nextSecStart == true) JumpPatchNum++;
    }

    // molloc the space for each jumpPatch
    std::vector<QMeshPatch*> layerJumpPatchSet(JumpPatchNum);
    for (int i = 0; i < JumpPatchNum; i++) {
        layerJumpPatchSet[i] = new QMeshPatch();
        layerJumpPatchSet[i]->rootPatch_jumpPatch = patch;
    }

    // Add node into each JumpPatch
    int Jump_PatchIndex = 0;
    int JumpPatch_NodeIndex = 0;
    for (GLKPOSITION Pos_Node = patch->GetNodeList().GetHeadPosition(); Pos_Node;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos_Node);

        if (Node->Jump_nextSecStart == true) {
            Jump_PatchIndex++;
            JumpPatch_NodeIndex = 0;
        }

        layerJumpPatchSet[Jump_PatchIndex]->GetNodeList().AddTail(Node);
        Node->Jump_SecIndex = JumpPatch_NodeIndex;
        JumpPatch_NodeIndex++;
    }
    //std::cout << "-----------------------------------" << std::endl;
    //std::cout << "--> Split ToolPath into JumpSection" << std::endl;

    return layerJumpPatchSet;
}

double GcodeGeneration::_safe_acos(double value) {
    if (value <= -1.0) {
        return PI;
    }
    else if (value >= 1.0) {
        return 0;
    }
    else {
        return acos(value);
    }
}

void GcodeGeneration::_markSingularNode(QMeshPatch* patch) {

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        Eigen::Vector2d Cspece_Coord;
        // Cal C space Coordinate : Cspece = Nx/Nz, Ny/Nz;
        Cspece_Coord << Node->m_printNor(0) / Node->m_printNor(2),
            Node->m_printNor(1) / Node->m_printNor(2);

        double R = Cspece_Coord.norm();
        double radLambda = DEGREE_TO_ROTATE(m_lambdaValue);

        if (R < tan(radLambda)) Node->isSingularNode = true;
    }
}

void GcodeGeneration::_filterSingleSingularNode(QMeshPatch* patch) {

    //protect
    if (patch->GetNodeNumber() < 4) return;

    std::vector<QMeshNode*> nodeSet(patch->GetNodeNumber());
    std::vector<bool> kept_Singular_Flag(patch->GetNodeNumber());

    int tempIndex = 0;
    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        nodeSet[tempIndex] = Node;
        kept_Singular_Flag[tempIndex] = Node->isSingularNode;
        tempIndex++;
    }


    // remove OXX ... XOX ... XXO
    for (int i = 0; i < kept_Singular_Flag.size(); i++) {

        if (i == 0) {
            if (kept_Singular_Flag[i] == false && kept_Singular_Flag[i + 1] == true && kept_Singular_Flag[i + 2] == true) {
                nodeSet[i]->isSingularNode = true;
            }
        }
        else if (i == (kept_Singular_Flag.size() - 1)) {
            if (kept_Singular_Flag[i - 2] == true && kept_Singular_Flag[i - 1] == true && kept_Singular_Flag[i] == false) {
                nodeSet[i]->isSingularNode = true;
            }
        }
        else {
            if (kept_Singular_Flag[i - 1] == true && kept_Singular_Flag[i] == false && kept_Singular_Flag[i + 1] == true) {
                nodeSet[i]->isSingularNode = true;
            }
        }
    }

    // remove XOOX
    if (patch->GetNodeNumber() < 5) return;
    for (int i = 0; i < kept_Singular_Flag.size(); i++) {
        kept_Singular_Flag[i] = nodeSet[i]->isSingularNode;
    }
    for (int i = 0; i < kept_Singular_Flag.size() - 3; i++) {

        if (kept_Singular_Flag[i] == true
            && kept_Singular_Flag[i + 1] == false
            && kept_Singular_Flag[i + 2] == false
            && kept_Singular_Flag[i + 3] == true) {
            nodeSet[i + 1]->isSingularNode = true;
            nodeSet[i + 2]->isSingularNode = true;
        }
    }
    // remove XOOOX
    if (patch->GetNodeNumber() < 6) return;
    for (int i = 0; i < kept_Singular_Flag.size(); i++) {
        kept_Singular_Flag[i] = nodeSet[i]->isSingularNode;
    }
    for (int i = 0; i < kept_Singular_Flag.size() - 4; i++) {

        if (kept_Singular_Flag[i] == true
            && kept_Singular_Flag[i + 1] == false
            && kept_Singular_Flag[i + 2] == false
            && kept_Singular_Flag[i + 3] == false
            && kept_Singular_Flag[i + 4] == true) {
            nodeSet[i + 1]->isSingularNode = true;
            nodeSet[i + 2]->isSingularNode = true;
            nodeSet[i + 3]->isSingularNode = true;
        }
    }
}

void GcodeGeneration::_getSingularSec(QMeshPatch* patch, Eigen::MatrixXd& sectionTable) {

    int lines = patch->GetNodeNumber();
    std::vector<int> srtPntIndTable, endPntIndTable;

    for (int i = 0; i < lines - 1; i++) {

        GLKPOSITION Node_Pos = patch->GetNodeList().FindIndex(i);
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetAt(Node_Pos);

        GLKPOSITION nextNode_Pos = patch->GetNodeList().FindIndex(i)->next;
        QMeshNode* nextNode = (QMeshNode*)patch->GetNodeList().GetAt(nextNode_Pos);


        if ((Node->isSingularNode == false && nextNode->isSingularNode == true)
            || (Node->isSingularNode == true && Node->Jump_SecIndex == 0)) {
            srtPntIndTable.push_back(Node->Jump_SecIndex);
            Node->isSingularSecStartNode = true;
        }
        if ((Node->isSingularNode == true && nextNode->isSingularNode == false)
            || (nextNode->isSingularNode == true && nextNode->Jump_SecIndex == lines - 1)) {
            endPntIndTable.push_back(nextNode->Jump_SecIndex);
            nextNode->isSingularSecEndNode = true;
        }
    }

    if (srtPntIndTable.size() == endPntIndTable.size()) sectionTable.resize(srtPntIndTable.size(), 2);
    else std::cout << "ERROR : srtPntIndTable.size() != endPntIndTable.size()" << std::endl;

    for (int i = 0; i < srtPntIndTable.size(); i++) {
        sectionTable(i, 0) = srtPntIndTable[i];
        sectionTable(i, 1) = endPntIndTable[i];
    }
    //std::cout << "sectionTable:\n"<<sectionTable << std::endl;
}

void GcodeGeneration::_projectAnchorPoint(QMeshPatch* patch) {

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        if (Node->isSingularSecStartNode == true && Node->isSingularSecEndNode == true) {
            std::cout << "Error: the normal of anchor point cannot move to the boundary of singular region" << std::endl;
            std::cout << "Error: as one normal cannot move to double directions" << std::endl;
        }

        if (Node->GetIndexNo() == 0 || Node->GetIndexNo() == (patch->GetNodeNumber() - 1)) continue;


        if (Node->isSingularSecStartNode == true || Node->isSingularSecEndNode == true) {

            Eigen::Vector3d m_printNor_before = Node->m_printNor;

            double anchor_Nz = cos(DEGREE_TO_ROTATE(m_lambdaValue));
            double temp_k = Node->m_printNor(1) / Node->m_printNor(0);
            double anchor_Nx = sqrt((1 - anchor_Nz * anchor_Nz) / (1 + temp_k * temp_k));
            if (Node->m_printNor(0) < 0.0) anchor_Nx = -anchor_Nx;
            double anchor_Ny = anchor_Nx * temp_k;

            Node->SetNormal(anchor_Nx, anchor_Ny, anchor_Nz);
            Node->SetNormal_last(anchor_Nx, anchor_Ny, anchor_Nz);
            Node->m_printNor << anchor_Nx, anchor_Ny, anchor_Nz;

            //cal the angle of before and after of anchor normal
            if (false) {
                double change = ROTATE_TO_DEGREE(
                    _safe_acos(
                        m_printNor_before.dot(Node->m_printNor)
                        / m_printNor_before.norm() / Node->m_printNor.norm()));
                std::cout << " the angle of before and after of anchor normal is " << change << std::endl;
            }
        }
    }
}

void GcodeGeneration::_getBCtable2(QMeshPatch* patch, Eigen::MatrixXd& B1C1table, Eigen::MatrixXd& B2C2table) {

    int lines = patch->GetNodeNumber();

    B1C1table = Eigen::MatrixXd::Zero(lines, 2);	B2C2table = Eigen::MatrixXd::Zero(lines, 2);

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        int i = Node->Jump_SecIndex;

        // solve 1
        // B1 deg // -acos(Nz)
        B1C1table(i, 0) = ROTATE_TO_DEGREE(-_safe_acos(Node->m_printNor(2)));
        // C1 deg //atan2(Nx, Ny)
        B1C1table(i, 1) = ROTATE_TO_DEGREE(atan2(Node->m_printNor(0), Node->m_printNor(1)));

        // solve 2
        // B2 deg // acos(Nz)
        B2C2table(i, 0) = -B1C1table(i, 0);
        // C2 deg //atan2(Ny, Nx) +/- 180
        double C2temp = B1C1table(i, 1) + 180.0;
        if (C2temp > 180.0)C2temp -= 360.0; // control the range of C2 into the (-180,180]
        B2C2table(i, 1) = C2temp;

        // only use solve 2
        // modified by tianyu 10/04/2022
        //B1C1table(i, 0) = B2C2table(i, 0);
        //B1C1table(i, 1) = B2C2table(i, 1);
    }
}

void GcodeGeneration::_getBCtable2_newConfig(QMeshPatch* patch, Eigen::MatrixXd& B1C1table, Eigen::MatrixXd& B2C2table) {

    int lines = patch->GetNodeNumber();

    B1C1table = Eigen::MatrixXd::Zero(lines, 2);	B2C2table = Eigen::MatrixXd::Zero(lines, 2);

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        int i = Node->Jump_SecIndex;

        // solve 1
        // B1 deg // -acos(Nz)
        B1C1table(i, 0) = ROTATE_TO_DEGREE(-_safe_acos(Node->m_printNor(2)));
        // C1 deg //atan2(Nx, Ny)
        B1C1table(i, 1) = ROTATE_TO_DEGREE(-atan2(Node->m_printNor(0), Node->m_printNor(1)));

        // solve 2
        // B2 deg // acos(Nz)
        B2C2table(i, 0) = -B1C1table(i, 0);
        // C2 deg //atan2(Ny, Nx) +/- 180
        double C2temp = B1C1table(i, 1) + 180.0;
        if (C2temp > 180.0)C2temp -= 360.0; // control the range of C2 into the (-180,180]
        B2C2table(i, 1) = C2temp;
    }
}

void GcodeGeneration::_motionPlanning3(
    QMeshPatch* patch, const Eigen::MatrixXd& sectionTable, const Eigen::MatrixXd& B1C1table,
    const Eigen::MatrixXd& B2C2table, Eigen::RowVector2d& prevBC) {

    int lines = patch->GetNodeNumber();
    Eigen::MatrixXd BC_Matrix(lines, 2); BC_Matrix = Eigen::MatrixXd::Zero(lines, 2);
    std::vector<int> solveFlag(lines);// 1 -> solve 1 // 2 -> solve 2
    // std::vector<int> insertNum(lines);// insertNum for large BC change at the beginning point
    int sectionNumber = 0;
    int sectionAmount = sectionTable.rows();

    int i = 0;

    while (i < lines) {
        //all points of current path are OUT of the sigularity region
        if (sectionAmount == 0) {

            if (_chooseB1C1(B1C1table.row(i), B2C2table.row(i), prevBC)) {
                prevBC << B1C1table.row(i);
                solveFlag[i] = 1;
            }
            else {
                prevBC << B2C2table.row(i);
                solveFlag[i] = 2;
            }

            BC_Matrix.row(i) = prevBC;
            i = i + 1;
        }
        else {
            Eigen::RowVector2d tempBC;
            //all points of current path are IN the sigularity region
            if (i == sectionTable(sectionNumber, 0) && i == 0 && sectionTable(sectionNumber, 1) == (lines - 1)) {
                for (int secLineIndex = i; secLineIndex < lines; secLineIndex++) {

                    tempBC = { 0.0, prevBC(1) };
                    prevBC = tempBC;
                    solveFlag[secLineIndex] = 1;
                    BC_Matrix.row(secLineIndex) = prevBC;
                }
                i = lines;
            }
            // start from the singularity region (end in singularity region or not)
            else if (i == sectionTable(sectionNumber, 0) && i == 0 && sectionTable(sectionNumber, 1) != (lines - 1)) {

                int secEndIndex = sectionTable(sectionNumber, 1);

                for (int secLineIndex = i; secLineIndex < secEndIndex; secLineIndex++) {

                    if (_chooseB1C1(B1C1table.row(secEndIndex), B2C2table.row(secEndIndex), prevBC)
                        || (secLineIndex == 0) // this can make sure start from solution 1
                        ) {
                        tempBC << B1C1table.row(secEndIndex);
                        solveFlag[secLineIndex] = 1;
                    }
                    else {
                        tempBC << B2C2table.row(secEndIndex);
                        solveFlag[secLineIndex] = 2;
                    }

                    prevBC = tempBC;
                    BC_Matrix.row(secLineIndex) = prevBC;

                }

                i = secEndIndex;
                if (sectionNumber != (sectionAmount - 1))	sectionNumber++;
            }
            // end in the singularity region / finish path
            else if (i == sectionTable(sectionNumber, 0) && i != 0 && sectionTable(sectionNumber, 1) == (lines - 1)) {

                int secStartIndex = sectionTable(sectionNumber, 0);

                for (int secLineIndex = i; secLineIndex < lines; secLineIndex++) {

                    if (_chooseB1C1(B1C1table.row(secStartIndex), B2C2table.row(secStartIndex), prevBC)) {
                        tempBC << B1C1table.row(secStartIndex);
                        solveFlag[secLineIndex] = 1;
                    }
                    else {
                        tempBC << B2C2table.row(secStartIndex);
                        solveFlag[secLineIndex] = 2;
                    }

                    prevBC = tempBC;
                    BC_Matrix.row(secLineIndex) = prevBC;
                }

                i = lines;
            }
            // path passes through the sigularity region
            else if (i == sectionTable(sectionNumber, 0) && i != 0 && sectionTable(sectionNumber, 1) != (lines - 1)) {

                // give the message to anchor point (start point)
                int secStartIndex = sectionTable(sectionNumber, 0);
                if (_chooseB1C1(B1C1table.row(secStartIndex), B2C2table.row(secStartIndex), prevBC)) {
                    prevBC << B1C1table.row(secStartIndex);
                    solveFlag[secStartIndex] = 1;
                }
                else {
                    prevBC << B2C2table.row(secStartIndex);
                    solveFlag[secStartIndex] = 2;
                }

                // record the deg_BC of secStart point
                Eigen::RowVector2d startPntBC = prevBC;

                // decide the solve of End point
                int secEndIndex = sectionTable(sectionNumber, 1);
                int pointNum = secEndIndex - secStartIndex;

                double rad_end_B1 = DEGREE_TO_ROTATE(B1C1table(secEndIndex, 0));	double rad_end_C1 = DEGREE_TO_ROTATE(B1C1table(secEndIndex, 1));
                double rad_end_B2 = DEGREE_TO_ROTATE(B2C2table(secEndIndex, 0));	double rad_end_C2 = DEGREE_TO_ROTATE(B2C2table(secEndIndex, 1));
                double rad_start_B = DEGREE_TO_ROTATE(startPntBC(0));				double rad_start_C = DEGREE_TO_ROTATE(startPntBC(1));

                Eigen::Vector2d v_start_C = { cos(rad_start_C),sin(rad_start_C) };
                Eigen::Vector2d v_end_C1 = { cos(rad_end_C1),sin(rad_end_C1) };
                Eigen::Vector2d v_end_C2 = { cos(rad_end_C2),sin(rad_end_C2) };
                //compute the actural angle of 2 vectors
                double rad_end_C1_start_C = _safe_acos(v_end_C1.dot(v_start_C));		double rad_end_B1_start_B = rad_end_B1 - rad_start_B;
                double rad_end_C2_start_C = _safe_acos(v_end_C2.dot(v_start_C));		double rad_end_B2_start_B = rad_end_B2 - rad_start_B;
                //get rad_C/B_start_end
                double rad_C_start_end = 0.0;
                double rad_B_start_end = 0.0;
                Eigen::Vector2d v_end_C = { 0.0,0.0 };

                int solveFlag_passThrough = 0; // 1 -> solve 1 // 2 -> solve 2

                if ((rad_end_C1_start_C) <= (rad_end_C2_start_C)) {
                    rad_C_start_end = rad_end_C1_start_C;
                    rad_B_start_end = rad_end_B1_start_B;
                    v_end_C = v_end_C1;
                    solveFlag_passThrough = 1;
                }
                else {
                    rad_C_start_end = rad_end_C2_start_C;
                    rad_B_start_end = rad_end_B2_start_B;
                    v_end_C = v_end_C2;
                    solveFlag_passThrough = 2;
                }

                //decide the rotation direction of C axis
                double sign = _toLeft({ 0.0,0.0 }, v_start_C, v_end_C);

                //get tha delta Angel of deg_B/C
                double C_delta_Angle = ROTATE_TO_DEGREE(rad_C_start_end) / pointNum;
                double B_delta_Angle = ROTATE_TO_DEGREE(rad_B_start_end) / pointNum;

                unsigned int times = 0;
                for (int secLineIndex = secStartIndex; secLineIndex < secEndIndex; secLineIndex++) {

                    tempBC(0) = startPntBC(0) + times * B_delta_Angle;
                    tempBC(1) = startPntBC(1) + sign * times * C_delta_Angle;

                    prevBC = tempBC;

                    if (prevBC(1) > 180.0) prevBC(1) -= 360.0;
                    if (prevBC(1) < -180.0) prevBC(1) += 360.0;

                    solveFlag[secLineIndex] = solveFlag_passThrough;
                    BC_Matrix.row(secLineIndex) = prevBC;

                    times++;
                }

                i = secEndIndex;

                if (sectionNumber != (sectionAmount - 1))	sectionNumber = sectionNumber + 1;

            }
            // other points out of the singularity region
            else {

                if (_chooseB1C1(B1C1table.row(i), B2C2table.row(i), prevBC)) {

                    prevBC << B1C1table.row(i);
                    solveFlag[i] = 1;
                }
                else {
                    prevBC << B2C2table.row(i);
                    solveFlag[i] = 2;
                }

                BC_Matrix.row(i) = prevBC;
                i = i + 1;
            }
        }
    }

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        int nodeIndex = Node->Jump_SecIndex;
        Node->m_XYZBCE(3) = BC_Matrix(nodeIndex, 0); //deg
        Node->m_XYZBCE(4) = BC_Matrix(nodeIndex, 1); //deg

        Node->solveSeclct = solveFlag[nodeIndex];
        //cout << Node->solveSeclct << endl;
        //Node->insertNodesNum = insertNum[nodeIndex];

    }
}

bool GcodeGeneration::_chooseB1C1(const Eigen::RowVector2d& B1C1, const Eigen::RowVector2d& B2C2, Eigen::RowVector2d& prevBC) {

    double rad_B1 = DEGREE_TO_ROTATE(B1C1(0));	double rad_C1 = DEGREE_TO_ROTATE(B1C1(1));
    double rad_B2 = DEGREE_TO_ROTATE(B2C2(0));	double rad_C2 = DEGREE_TO_ROTATE(B2C2(1));
    double rad_Bp = DEGREE_TO_ROTATE(prevBC(0)); double rad_Cp = DEGREE_TO_ROTATE(prevBC(1));

    Eigen::Vector2d v_Cp = { cos(rad_Cp),sin(rad_Cp) };
    Eigen::Vector2d v_C1 = { cos(rad_C1),sin(rad_C1) };
    Eigen::Vector2d v_C2 = { cos(rad_C2),sin(rad_C2) };
    //compute the actural angle

    double rad_v_C1_v_Cp = _safe_acos(v_C1.dot(v_Cp));		double rad_B1_rad_Bp = fabs(rad_B1 - rad_Bp);
    double rad_v_C2_v_Cp = _safe_acos(v_C2.dot(v_Cp));		double rad_B2_rad_Bp = fabs(rad_B2 - rad_Bp);

    bool isB1C1 = true;

    if ((rad_v_C1_v_Cp + rad_B1_rad_Bp) > (rad_v_C2_v_Cp + rad_B2_rad_Bp)) {
        isB1C1 = false;

        //std::cout << "----------------------------\n use 2 solve" << std::endl;
        //std::cout << "B1C1 = " << B1C1 << std::endl;
        //std::cout << "B2C2 = " << B2C2 << std::endl;
        //std::cout << "prevBC = " << prevBC << std::endl;
        //std::cout << "rad_v_C1_v_Cp = " << rad_v_C1_v_Cp << std::endl;
        //std::cout << "rad_B1_rad_Bp = " << rad_B1_rad_Bp << std::endl;
        //std::cout << "rad_v_C2_v_Cp = " << rad_v_C2_v_Cp << std::endl;
        //std::cout << "rad_B2_rad_Bp = " << rad_B2_rad_Bp << std::endl;
    }
    return isB1C1;
}

double GcodeGeneration::_toLeft(
    const Eigen::RowVector2d& origin_p, const Eigen::RowVector2d& startPnt_q, const Eigen::RowVector2d& endPnt_s) {

    double Area2 = origin_p(0) * startPnt_q(1) - origin_p(1) * startPnt_q(0)
        + startPnt_q(0) * endPnt_s(1) - startPnt_q(1) * endPnt_s(0)
        + endPnt_s(0) * origin_p(1) - endPnt_s(1) * origin_p(0);

    double isLeft = -1.0;
    if (Area2 > 0.0) isLeft = 1.0;

    return isLeft;
}

void GcodeGeneration::_getXYZ(QMeshPatch* patch) {

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        double px = Node->m_printPos(0);
        double py = Node->m_printPos(1);
        double pz = Node->m_printPos(2);

        double rad_B = DEGREE_TO_ROTATE(Node->m_XYZBCE(3));// rad
        double rad_C = DEGREE_TO_ROTATE(Node->m_XYZBCE(4));// rad

        if (is_planar_printing) {
            rad_B = 0.0; rad_C = 0.0;
            Node->m_XYZBCE(3) = 0.0; Node->m_XYZBCE(4) = 0.0;
        }

        //cal XYZ
        Node->m_XYZBCE(0) = px * cos(rad_C) - py * sin(rad_C);
        Node->m_XYZBCE(1) = px * sin(rad_C) + py * cos(rad_C) - h * sin(rad_B);
        Node->m_XYZBCE(2) = pz - h * (1 - cos(rad_B));

        //if (Node->m_XYZBCE(2) < 0.0) { Node->negativeZ = true; }
    }
}

void GcodeGeneration::_getXYZ_newConfig(QMeshPatch* patch) {

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        double px = Node->m_printPos(0);
        double py = Node->m_printPos(1);
        double pz = Node->m_printPos(2);

        double rad_B = DEGREE_TO_ROTATE(Node->m_XYZBCE(3));// rad
        double rad_C = DEGREE_TO_ROTATE(Node->m_XYZBCE(4));// rad

        //cal XYZ
        Node->m_XYZBCE(0) = px - r * cos(rad_C) + h * sin(rad_C) * sin(rad_B) + r;
        Node->m_XYZBCE(1) = py - r * sin(rad_C) - h * cos(rad_C) * sin(rad_B);
        Node->m_XYZBCE(2) = pz + h * cos(rad_B) - h;

        //if (Node->m_XYZBCE(2) < 0.0) { Node->negativeZ = true; }
    }
}

void GcodeGeneration::_calDHW2E(QMeshPatch* patch, bool hysteresis_switch) {

    // E = E + ratio * height * length * width;
    // Dicided by CNC W.R.T (E:Volume:E = 0.45)

    double ratio = 0.58;
    double D, H, W;

    // optimize the Hysteresis of extruder
    std::vector<double> Hysteresis_Ks = { 2.0,2.0,1.7,1.5,1.25 };
    std::vector<double> Hysteresis_Ke = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
    int lines = patch->GetNodeNumber();
    std::vector<double> deltaE(lines);

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        if (m_varyDistance_switch == true) D = Node->m_DHW(0);
        else D = 0.65;
        if (m_varyHeight_switch == true) H = Node->m_DHW(1);
        else H = 0.8;
        if (m_varyWidth_switch == true) W = Node->m_DHW(2);
        else W = 0.5;

        deltaE[Node->Jump_SecIndex] = ratio * H * D * W;
    }

    if (lines >= (Hysteresis_Ks.size() + Hysteresis_Ke.size()) && hysteresis_switch) {

        for (int i = 0; i < lines; i++) {

            if (i >= 0 && i < Hysteresis_Ks.size()) {
                deltaE[i] = deltaE[i] * Hysteresis_Ks[i];
            }

            if (i >= (lines - Hysteresis_Ke.size()) && i < lines) {
                deltaE[i] = deltaE[i] * Hysteresis_Ke[i - lines + Hysteresis_Ke.size()];
            }
        }
    }

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        Node->m_XYZBCE(5) = deltaE[Node->Jump_SecIndex];
    }
}

void GcodeGeneration::_optimizationC(QMeshPatch* patch) {

    for (int loop = 0; loop < 20; loop++) {

        double threshhold = 180.0;

        for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

            double C = Node->m_XYZBCE(4); // deg

            if (Node->GetIndexNo() == 0) continue;
            GLKPOSITION prevPos = patch->GetNodeList().Find(Node)->prev;
            QMeshNode* prevNode = (QMeshNode*)patch->GetNodeList().GetAt(prevPos);
            double preC = prevNode->m_XYZBCE(4);

            if (C - preC < -threshhold) {
                C = C + 360;
            }
            else if (C - preC > threshhold) {
                C = C - 360;
            }
            else {}

            Node->m_XYZBCE(4) = C;
        }
    }
}

void GcodeGeneration::_limit_C_range(QMeshPatch* patch) {

    //limit the C angle into --> [-pi,pi]
    double delta_C = 0.0;
    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        if ((Node->m_XYZBCE(4) + delta_C) > 180.0) {
            Node->is_PiFlip_JumpEnd = true;
            delta_C -= 360.0;
        }

        if ((Node->m_XYZBCE(4) + delta_C) < -180.0) {
            Node->is_PiFlip_JumpEnd = 1.0;
            delta_C += 360.0;
        }
        Node->m_XYZBCE(4) += delta_C;
    }
}

void GcodeGeneration::_verifyPosNor() {

    std::cout << "------------------------------------------- PosNor verification running ... " << std::endl;
    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        if (WayPointPatch->GetIndexNo() < m_FromIndex || WayPointPatch->GetIndexNo() > m_ToIndex) continue;

        for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

            double X = Node->m_XYZBCE(0);	double Y = Node->m_XYZBCE(1);	double Z = Node->m_XYZBCE(2);
            double rad_B = DEGREE_TO_ROTATE(Node->m_XYZBCE(3));
            double rad_C = DEGREE_TO_ROTATE(Node->m_XYZBCE(4));

            double finalNx = -sin(rad_B) * sin(rad_C);
            double finalNy = -sin(rad_B) * cos(rad_C);
            double finalNz = cos(rad_B);
            double finalPx = X * cos(rad_C) + sin(rad_C) * (h * sin(rad_B) + Y);
            double finalPy = -X * sin(rad_C) + cos(rad_C) * (h * sin(rad_B) + Y);
            double finalPz = Z + h - h * cos(rad_B);

            bool equalPx = (fabs(finalPx - Node->m_printPos(0)) < 0.001) ? true : false;
            bool equalPy = (fabs(finalPy - Node->m_printPos(1)) < 0.001) ? true : false;
            bool equalPz = (fabs(finalPz - Node->m_printPos(2)) < 0.001) ? true : false;

            if ((equalPx == false) || (equalPy == false) || (equalPz == false)) {
                std::cout << "++++++++++++++++++++++++++++++++" << std::endl;
                std::cout << "Layer " << WayPointPatch->GetIndexNo() << " Point Index " << Node->GetIndexNo() << std::endl;
                std::cout << "final Position" << std::endl;
                std::cout << finalPx << "\n" << finalPy << "\n" << finalPz << std::endl;
                std::cout << "print Position\n" << Node->m_printPos << std::endl;
            }

            Eigen::Vector3d finalNormal = { finalNx, finalNy, finalNz };
            double angle = _getAngle3D(finalNormal, Node->m_printNor, true);
            //if (angle >= 0.0001) {
            if (angle > (m_lambdaValue * 2 + 1.0)) {
                //if (Node->isSingularNode) cout << "this is a singular node";
                std::cout << "--------------------------------" << std::endl;
                std::cout << "Layer " << WayPointPatch->GetIndexNo() << " Point Index " << Node->GetIndexNo() << std::endl;
                std::cout << "The angle is " << angle << std::endl;
                std::cout << "final Normal\n" << finalNormal.transpose() << std::endl;
                std::cout << "print Normal\n" << Node->m_printNor.transpose() << std::endl;
            }

            //update the normal after singular optimization
            Node->SetNormal(finalNormal(0), finalNormal(1), finalNormal(2));// for show the final normal on the GUI
            Node->SetNormal_last(finalNormal(0), finalNormal(1), finalNormal(2));
            Node->m_printNor = finalNormal;
        }

    }

    std::cout << "------------------------------------------- PosNor verification Finish!\n" << std::endl;

}

void GcodeGeneration::_verifyPosNor_newConfig() {

    std::cout << "------------------------------------------- PosNor verification running ... " << std::endl;
    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        if (WayPointPatch->GetIndexNo() < m_FromIndex || WayPointPatch->GetIndexNo() > m_ToIndex) continue;

        for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

            double X = Node->m_XYZBCE(0);	double Y = Node->m_XYZBCE(1);	double Z = Node->m_XYZBCE(2);
            double rad_B = DEGREE_TO_ROTATE(Node->m_XYZBCE(3));
            double rad_C = DEGREE_TO_ROTATE(Node->m_XYZBCE(4));

            double finalNx = sin(rad_B) * sin(rad_C);
            double finalNy = -sin(rad_B) * cos(rad_C);
            double finalNz = cos(rad_B);
            double finalPx = X + r * cos(rad_C) - h * sin(rad_C) * sin(rad_B) - r;
            double finalPy = Y + r * sin(rad_C) + h * cos(rad_C) * sin(rad_B);
            double finalPz = Z - h * cos(rad_B) + h;

            bool equalPx = (fabs(finalPx - Node->m_printPos(0)) < 0.001) ? true : false;
            bool equalPy = (fabs(finalPy - Node->m_printPos(1)) < 0.001) ? true : false;
            bool equalPz = (fabs(finalPz - Node->m_printPos(2)) < 0.001) ? true : false;

            if ((equalPx == false) || (equalPy == false) || (equalPz == false)) {
                std::cout << "++++++++++++++++++++++++++++++++" << std::endl;
                std::cout << "Layer " << WayPointPatch->GetIndexNo() << " Point Index " << Node->GetIndexNo() << std::endl;
                std::cout << "final Position" << std::endl;
                std::cout << finalPx << "\n" << finalPy << "\n" << finalPz << std::endl;
                std::cout << "print Position\n" << Node->m_printPos << std::endl;
            }

            Eigen::Vector3d finalNormal = { finalNx, finalNy, finalNz };
            double angle = _getAngle3D(finalNormal, Node->m_printNor, true);
            //if (angle >= 0.0001) {
            if (angle > (m_lambdaValue * 2 + 1.0)) {
                //if (Node->isSingularNode) cout << "this is a singular node";
                std::cout << "--------------------------------" << std::endl;
                std::cout << "Layer " << WayPointPatch->GetIndexNo() << " Point Index " << Node->GetIndexNo() << std::endl;
                std::cout << "The angle is " << angle << std::endl;
                std::cout << "final Normal\n" << finalNormal.transpose() << std::endl;
                std::cout << "print Normal\n" << Node->m_printNor.transpose() << std::endl;
            }

            //update the normal after singular optimization
            Node->SetNormal(finalNormal(0), finalNormal(1), finalNormal(2));// for show the final normal on the GUI
            Node->SetNormal_last(finalNormal(0), finalNormal(1), finalNormal(2));
            Node->m_printNor = finalNormal;
        }

    }

    std::cout << "------------------------------------------- PosNor verification Finish!\n" << std::endl;

}

double GcodeGeneration::_getAngle3D(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const bool in_degree) {
    //compute the actural angle
    double rad = v1.normalized().dot(v2.normalized());//dot product
    if (rad < -1.0)
        rad = -1.0;
    else if (rad > 1.0)
        rad = 1.0;
    return (in_degree ? _safe_acos(rad) * 180.0 / PI : _safe_acos(rad));
}

void GcodeGeneration::testXYZBCE(bool testXYZBC_E_switch) {

    if (testXYZBC_E_switch == false)	return;
    std::cout << "------------------------------------------- XYZBCD Data Writing ..." << std::endl;

    std::string testData_Dir = "../DataSet/fabricationTest/test_XYZBCE/";
    this->_remove_allFile_in_Dir(testData_Dir);

    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        if (WayPointPatch->GetIndexNo() < m_FromIndex || WayPointPatch->GetIndexNo() > m_ToIndex) continue;

        char targetFilename[1024];

        if (WayPointPatch->is_SupportLayer) {
            std::sprintf(targetFilename, "%s%d%s", testData_Dir.c_str(), WayPointPatch->GetIndexNo(), "S.txt");
        }
        else {
            std::sprintf(targetFilename, "%s%d%s", testData_Dir.c_str(), WayPointPatch->GetIndexNo(), ".txt");
        }

        // cout << targetFilename << endl;

        FILE* fp = fopen(targetFilename, "w");
        if (!fp)	return;

        for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

            Eigen::MatrixXd XYZBCE = Node->m_XYZBCE;

            fprintf(fp, "%f %f %f %f %f %f %d %d %d\n",
                XYZBCE(0), XYZBCE(1), XYZBCE(2), XYZBCE(3), XYZBCE(4), XYZBCE(5),
                Node->Jump_preSecEnd, Node->Jump_nextSecStart, 0);

            if (Node->insertNodesInfo.size() == 0) continue;

            for (int i = 0; i < Node->insertNodesInfo.size(); i++) {

                //cout << "Node->insertNodesInfo.size() = " << Node->insertNodesInfo.size() << endl;

                Eigen::MatrixXd eachInsertNode_XYZBCE = Node->insertNodesInfo[i];

                //cout << "eachInsertNode_XYZBCE = " << eachInsertNode_XYZBCE << endl;

                fprintf(fp, "%f %f %f %f %f %f %d %d %d\n",
                    eachInsertNode_XYZBCE(0), eachInsertNode_XYZBCE(1), eachInsertNode_XYZBCE(2),
                    eachInsertNode_XYZBCE(3), eachInsertNode_XYZBCE(4), eachInsertNode_XYZBCE(5),
                    0, 0, 1);

            }
        }

        std::fclose(fp);

        //cout << "-------------------- Layer " << WayPointPatch->GetIndexNo() << " XYZBCD Data Write" << endl;	
    }
    std::cout << "------------------------------------------- XYZBCD Data Write Finish!\n" << std::endl;
}

void GcodeGeneration::detectCollision_2() {

    std::cout << "------------------------------------------- Collision Detection Running ..." << std::endl;
    long time = clock();

    // get the patch polygenMesh_PrintPlatform
    QMeshPatch* plateform_Patch = NULL;
    for (GLKPOSITION posMesh = m_CncPart->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
        QMeshPatch* thisPatch = (QMeshPatch*)m_CncPart->GetMeshList().GetNext(posMesh);

        if (m_modelName == "wing_mirror_step3") {
            if (thisPatch->patchName == "baseModel_self") {
                plateform_Patch = thisPatch;
                break;
            }
        }
        else if (m_modelName == "wing_mirror_step2" || m_modelName == "wing_mirror_step4") {
            if (thisPatch->patchName == "baseModel_5AM") {
                plateform_Patch = thisPatch;
                break;
            }
        }
        else {
            if (thisPatch->patchName == "c_axis") {
                plateform_Patch = thisPatch;
                break;
            }
        }
    }

    //std::cout << "model name = " << m_modelName << std::endl;
    if(plateform_Patch == NULL) std::cout << "no platform patch, please check" << std::endl;
    // get the patch polygenMesh_PrintHead
    QMeshPatch* eHeadPatch = NULL;
    for (GLKPOSITION posMesh = m_CncPart->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
        QMeshPatch* thisPatch = (QMeshPatch*)m_CncPart->GetMeshList().GetNext(posMesh);

        if (thisPatch->patchName == "nozzle_4C") {
            eHeadPatch = thisPatch;
            break;
        }
    }

    if (plateform_Patch == NULL || eHeadPatch == NULL) {
        std::cout << "Platform or PrintHead is NULL, please check." << std::endl;
        return;
    }
    plateform_Patch->drawThisPatch = true; eHeadPatch->drawThisPatch = true;

    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        bool collisionPatch = false;
        if (WayPointPatch->GetIndexNo() < m_FromIndex || WayPointPatch->GetIndexNo() > m_ToIndex) continue;

        if (m_modelName == "yoga_icra" && WayPointPatch->GetIndexNo() < 200) continue;
        if (m_modelName == "yoga_icra" && WayPointPatch->GetIndexNo() > 225) continue;

        for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

            Node->isCollision = false;

            Eigen::Vector3d nodePos, norm;
            Node->GetCoord3D(nodePos(0), nodePos(1), nodePos(2));
            Node->GetNormal(norm(0), norm(1), norm(2));
            _locate_EHead_printPos(eHeadPatch, nodePos, norm);

            // Test all of the previous-layers' node +-+-+ all of the previous-node before the deteted node at the same layer
            std::vector<QMeshNode*> check_below_WpSet;
            int layerLoop = 0;
            // collect the Waypoint patch before the printed node now
            for (GLKPOSITION prevPos = m_Waypoints->GetMeshList().Find(WayPointPatch); prevPos;) {
                QMeshPatch* prevWpPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetPrev(prevPos);

                if (layerLoop >= layerDepth) break;

                for (GLKPOSITION prevWpNodePos = prevWpPatch->GetNodeList().GetHeadPosition(); prevWpNodePos;) {
                    QMeshNode* prevWpNode = (QMeshNode*)prevWpPatch->GetNodeList().GetNext(prevWpNodePos);

                    if (WayPointPatch->GetIndexNo() == prevWpPatch->GetIndexNo()) {
                        if (prevWpNode->GetIndexNo() >= Node->GetIndexNo()) continue;
                    }
                    check_below_WpSet.push_back(prevWpNode);
                }

                layerLoop++;
            }

            //get the Bounding Box of the nozzle
            double lowerB[3], upperB[3];
            eHeadPatch->ComputeBoundingBox(lowerB[0], lowerB[1], lowerB[2], upperB[0], upperB[1], upperB[2]);
            double node_threshold = 0.5;
            //end

            Eigen::VectorXi check_below_WpSet_flag = Eigen::VectorXi::Zero(check_below_WpSet.size());
            // speed up the code about the _checkSingleNodeCollision();
#pragma omp parallel
            {
#pragma omp for  
                for (int i = 0; i < check_below_WpSet.size(); i++) {

                    //speed up with AABB box (not tree)
                    Eigen::Vector3d pp = check_below_WpSet[i]->m_printPos;
                    bool overLap = true;
                    double pp_lowerB[3], pp_upperB[3];
                    for (int dimId = 0; dimId < 3; dimId++) {
                        pp_lowerB[dimId] = pp[dimId] - node_threshold;
                        pp_upperB[dimId] = pp[dimId] + node_threshold;
                    }
                    for (int dimId = 0; dimId < 3; dimId++) {
                        if (pp_lowerB[dimId] > upperB[dimId]
                            || pp_upperB[dimId] < lowerB[dimId]) {
                            overLap = false;
                            break;
                        }
                    }
                    if (overLap == false) continue;
                    //end


                    bool isInHull = _isPnt_in_mesh2(eHeadPatch, check_below_WpSet[i]->m_printPos);
                    if (isInHull) { check_below_WpSet_flag[i] = 1; }
                    else { check_below_WpSet_flag[i] = 0; }
                }
            }

            if (check_below_WpSet_flag.sum() > 0) {

                collisionPatch = true;
                Node->isCollision = true;

                for (int i = 0; i < check_below_WpSet_flag.size(); i++) {
                    if (check_below_WpSet_flag[i] == 1) {
                        check_below_WpSet[i]->iscollided = true;
                        //std::cout << "Layer:[" << Node->GetMeshPatchPtr()->GetIndexNo() 
                        //    << "]\tPnt Index:[" << Node->GetIndexNo() << "]\t-COLLISION." << std::endl;
                        //std::cout << "Collided point: Layer:[" << check_below_WpSet[i]->GetMeshPatchPtr()->GetIndexNo()
                        //    << "]\tPnt Index:[" << check_below_WpSet[i]->GetIndexNo() << "]" << std::endl;
                    }
                }
            }

            int table_Collision_Detect_method = 2;
            if (m_modelName == "wing_mirror_step3" || m_modelName == "wing_mirror_step2" || m_modelName == "wing_mirror_step4")
                table_Collision_Detect_method = 1;
            //check platform nodes (This method needs C axis model to be densen mesh, which is not friendly to simulation update)
            if (table_Collision_Detect_method == 1) {
                for (GLKPOSITION plateform_NodePos = plateform_Patch->GetNodeList().GetHeadPosition(); plateform_NodePos;) {
                    QMeshNode* plateform_Node = (QMeshNode*)plateform_Patch->GetNodeList().GetNext(plateform_NodePos);

                    bool isInHull = _isPnt_in_mesh2(eHeadPatch, plateform_Node->m_printPos);

                    if (isInHull) {
                        collisionPatch = true;
                        Node->isCollision = true;
                        plateform_Node->iscollided = true;
                        std::cout << "Layer:[" << WayPointPatch->GetIndexNo() << "]\tPnt Index:[" << Node->GetIndexNo()
                            << "]\t-COLLISION (platform)." << std::endl;
                        break;
                    }
                }
            }
            //check platform nodes (This method only needs check Z value less than 0)
            else {
                for (GLKPOSITION eHead_NodePos = eHeadPatch->GetNodeList().GetHeadPosition(); eHead_NodePos;) {
                    QMeshNode* eHead_Node = (QMeshNode*)eHeadPatch->GetNodeList().GetNext(eHead_NodePos);

                    double eHead_xx, eHead_yy, eHead_zz = 0.0;
                    eHead_Node->GetCoord3D(eHead_xx, eHead_yy, eHead_zz);
                    // this is decided by Zup and 0-height(z_move) table.
                    if (eHead_zz < m_Zmove) {
                        collisionPatch = true;
                        Node->isCollision = true;
                        std::cout << "Layer:[" << WayPointPatch->GetIndexNo() << "]\tPnt Index:[" << Node->GetIndexNo()
                            << "]\t-COLLISION (platform)." << std::endl;
                        break;
                    }
                }
            }
            /*if (collisionPatch) break;*/ // check whitch node causes collision
        }

        if (collisionPatch) {
            std::cout << " --> Collision patch Index " << WayPointPatch->GetIndexNo() << std::endl;
        }
        else {
            std::cout << " --> Collision-free patch Index " << WayPointPatch->GetIndexNo() << std::endl;
        }
    }
    std::printf("\nTIMER -- Collision Detection takes %ld ms.\n", clock() - time);
    std::cout << "------------------------------------------- Collision Detection Finish!\n" << std::endl;
}

void GcodeGeneration::_locate_EHead_printPos(QMeshPatch* eHeadPatch, Eigen::Vector3d nodePos, Eigen::Vector3d norm) {
    // rotate the nozzle mesh to print orientation and move to the tip position
    Eigen::Vector3d eHeadInitNorm = { 0, 0, 1.0 };
    Eigen::Matrix3d rotationMatrix;

    rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(eHeadInitNorm, norm);

    //std::cout << rotationMatrix << std::endl;

    for (GLKPOSITION Pos = eHeadPatch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* eHeadNode = (QMeshNode*)eHeadPatch->GetNodeList().GetNext(Pos);

        Eigen::Vector3d eHeadNodePos;

        eHeadNode->GetCoord3D_last(eHeadNodePos(0), eHeadNodePos(1), eHeadNodePos(2));
        //nozzleNodePos = rotationMatrix * nozzleNodePos;
        eHeadNodePos = rotationMatrix * eHeadNodePos + nodePos;

        eHeadNode->SetCoord3D(eHeadNodePos(0), eHeadNodePos(1), eHeadNodePos(2));
        //std::cout << nozzleNodePos(0) << "," << nozzleNodePos(1) << "," << nozzleNodePos(2) << std::endl;
    }
}

bool GcodeGeneration::_isPnt_in_mesh(QMeshPatch* surfaceMesh, Eigen::Vector3d node_coord3D) {

    Eigen::Vector3d dir = { 1.0,0.0,0.0 };
    Eigen::Vector3d orig = node_coord3D;
    int intersection_Time = 0;


    for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* each_face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);

        double xx, yy, zz;
        each_face->GetNodeRecordPtr(0)->GetCoord3D(xx, yy, zz);
        Eigen::Vector3d v0 = { xx,yy,zz };

        each_face->GetNodeRecordPtr(1)->GetCoord3D(xx, yy, zz);
        Eigen::Vector3d v1 = { xx,yy,zz };

        each_face->GetNodeRecordPtr(2)->GetCoord3D(xx, yy, zz);
        Eigen::Vector3d v2 = { xx,yy,zz };

        if (this->IntersectTriangle(orig, dir, v0, v1, v2))
            intersection_Time++;
    }
    //std::cout << "intersection Num " << intersection_Time << std::endl;
    if (intersection_Time % 2 != 0) {
        //std::cout << "in the mesh" << std::endl;
        return true;
    }
    else return false;
    //std::cout << "be out of mesh" << std::endl;
}

// Determine whether a ray intersect with a triangle
// Parameters
// orig: origin of the ray
// dir: direction of the ray
// v0, v1, v2: vertices of triangle
// t(out): weight of the intersection for the ray
// u(out), v(out): barycentric coordinate of intersection

bool GcodeGeneration::IntersectTriangle(const Eigen::Vector3d& orig, const Eigen::Vector3d& dir,
    Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2)
{
    // E1
    Eigen::Vector3d E1 = v1 - v0;

    // E2
    Eigen::Vector3d E2 = v2 - v0;

    // P
    Eigen::Vector3d P = dir.cross(E2);

    // determinant
    float det = E1.dot(P);

    // keep det > 0, modify T accordingly
    Eigen::Vector3d T;
    if (det > 0)
    {
        T = orig - v0;
    }
    else
    {
        T = v0 - orig;
        det = -det;
    }

    // If determinant is near zero, ray lies in plane of triangle
    if (det < 0.000001f)
        return false;

    // Calculate u and make sure u <= 1
    double t, u, v;
    u = T.dot(P);
    if (u < 0.0f || u > det)
        return false;

    // Q
    Eigen::Vector3d Q = T.cross(E1);

    // Calculate v and make sure u + v <= 1
    v = dir.dot(Q);
    if (v < 0.0f || u + v > det)
        return false;

    // Calculate t, scale parameters, ray intersects triangle
    t = E2.dot(Q);
    if (t < 0) return false;

    float fInvDet = 1.0f / det;
    t *= fInvDet;
    u *= fInvDet;
    v *= fInvDet;

    return true;
}

bool GcodeGeneration::_isPnt_in_mesh2(QMeshPatch* surfaceMesh, Eigen::Vector3d node_coord3D) {

    double A, B, C, D;
    for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* each_face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);

        each_face->CalPlaneEquation();
        each_face->GetPlaneEquation(A, B, C, D);
        Eigen::Vector3d normVec = { A,B,C };
        normVec.normalize();

        if ((node_coord3D.dot(normVec) + D) >= 0.0)
            return false;
    }
    return true;
}


/*******************************************/
/* Main Function for Graph Search */
/*******************************************/
void GcodeGeneration::graph_Search_Shortest_Path() {

    std::cout << "------------------------------------------- Graph Search running ... " << std::endl;
    long time = clock();

    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        if (WayPointPatch->GetIndexNo() < m_FromIndex || WayPointPatch->GetIndexNo() > m_ToIndex) continue;

        std::vector<QMeshPatch*> layerJumpPatchSet = _getJumpSection_patchSet(WayPointPatch);

        //each Jump Section
        for (int Index = 0; Index < layerJumpPatchSet.size(); Index++) {
            //1. generate a vector to store each graph_Node in the form of collision_Node(a temp struct)
            std::vector<collision_Node> graph_Node;
            _get_GraphNode_List(layerJumpPatchSet[Index], graph_Node);
            _build_Graph(layerJumpPatchSet[Index], graph_Node);
            _getXYZ(layerJumpPatchSet[Index]);
        }
        //aim to eliminate the -pi to pi sharp change
        _optimizationC(WayPointPatch);
    }

    std::cout << "-------------------------------------------" << std::endl;
    std::printf("TIMER -- Graph Search takes %ld ms.\n", clock() - time);
    std::cout << "------------------------------------------- Graph Search Finish!\n " << std::endl;
}

void GcodeGeneration::graph_Search_Shortest_Path_newConfig() {

    std::cout << "------------------------------------------- Graph Search running ... " << std::endl;
    long time = clock();

    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        if (WayPointPatch->GetIndexNo() < m_FromIndex || WayPointPatch->GetIndexNo() > m_ToIndex) continue;

        std::vector<QMeshPatch*> layerJumpPatchSet = _getJumpSection_patchSet(WayPointPatch);

        //each Jump Section
        for (int Index = 0; Index < layerJumpPatchSet.size(); Index++) {
            //1. generate a vector to store each graph_Node in the form of collision_Node(a temp struct)
            std::vector<collision_Node> graph_Node;
            _get_GraphNode_List_newConfig(layerJumpPatchSet[Index], graph_Node);
            _build_Graph_newConfig(layerJumpPatchSet[Index], graph_Node);
            _getXYZ_newConfig(layerJumpPatchSet[Index]);
        }
        //aim to eliminate the -pi to pi sharp change
        _optimizationC(WayPointPatch);
        _limit_C_range(WayPointPatch);
    }

    std::cout << "-------------------------------------------" << std::endl;
    std::printf("TIMER -- Graph Search takes %ld ms.\n", clock() - time);
    std::cout << "------------------------------------------- Graph Search Finish!\n " << std::endl;
}

void GcodeGeneration::_get_GraphNode_List(QMeshPatch* patch, std::vector<collision_Node>& graph_Node) {

    // get the patch polygenMesh_PrintHead
    QMeshPatch* eHeadPatch = NULL;
    for (GLKPOSITION posMesh = m_CncPart->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
        QMeshPatch* thisPatch = (QMeshPatch*)m_CncPart->GetMeshList().GetNext(posMesh);

        if (thisPatch->patchName == "nozzle_4C") {
            eHeadPatch = thisPatch;
            break;
        }
    }
    if (eHeadPatch == NULL) { std::cout << "PrintHead is NULL, please check." << std::endl; return; }


    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        if (Node->isCollision) {

            double candidate_normal_NUM = 0;
            for (int ZrotateAngle = 0; ZrotateAngle < 360; ZrotateAngle = ZrotateAngle + delta_Z) {
                for (int XtiltAngle = 90; XtiltAngle > 0; XtiltAngle = XtiltAngle - delta_X) {

                    double rad_ZrotateAngle = DEGREE_TO_ROTATE(ZrotateAngle);
                    double rad_XtiltAngle = DEGREE_TO_ROTATE(XtiltAngle);

                    Eigen::Vector3d candidateNor = _calCandidateNormal(Node->m_printNor, rad_ZrotateAngle, rad_XtiltAngle);
                    //cout << "candidateNor:\n " << candidateNor << endl;

                    bool iscollision_candidate_cNode = false;
                    _locate_EHead_printPos(eHeadPatch, Node->m_printPos, candidateNor);
                    // Test all of the previous-layers' node +-+-+ all of the previous-node before the deteted node at the same layer
                    std::vector<QMeshPatch*> check_below_WpSet;
                    int layerLoop = 0;
                    // collect the Waypoint patch before the printed node now
                    for (GLKPOSITION prevPos = m_Waypoints->GetMeshList().Find(patch->rootPatch_jumpPatch); prevPos;) {
                        QMeshPatch* prevWpPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetPrev(prevPos);

                        if (layerLoop >= layerDepth) break;

                        check_below_WpSet.push_back(prevWpPatch);

                        layerLoop++;
                    }
                    // speed up the code about the _checkSingleNodeCollision();
#pragma omp parallel
                    {
#pragma omp for  
                        for (int i = 0; i < check_below_WpSet.size(); i++) {

                            for (GLKPOSITION prevWpNodePos = check_below_WpSet[i]->GetNodeList().GetHeadPosition(); prevWpNodePos;) {
                                QMeshNode* prevWpNode = (QMeshNode*)check_below_WpSet[i]->GetNodeList().GetNext(prevWpNodePos);

                                if (patch->rootPatch_jumpPatch->GetIndexNo() == check_below_WpSet[i]->GetIndexNo()) {
                                    if (prevWpNode->GetIndexNo() >= Node->GetIndexNo()) continue;
                                }

                                bool isInHull = _isPnt_in_mesh(eHeadPatch, prevWpNode->m_printPos);

                                if (isInHull) {
                                    iscollision_candidate_cNode = true;
                                    break;
                                }
                            }
                            if (iscollision_candidate_cNode == true) break;
                        }
                    }

                    //add platform collision detection
                    for (GLKPOSITION eHead_NodePos = eHeadPatch->GetNodeList().GetHeadPosition(); eHead_NodePos;) {
                        QMeshNode* eHead_Node = (QMeshNode*)eHeadPatch->GetNodeList().GetNext(eHead_NodePos);

                        double eHead_xx, eHead_yy, eHead_zz = 0.0;
                        eHead_Node->GetCoord3D(eHead_xx, eHead_yy, eHead_zz);
                        // this is decided by Zup and 0-height table.
                        if (eHead_zz < 0.0) {
                            iscollision_candidate_cNode = true;
                            break;
                        }
                    }


                    if (iscollision_candidate_cNode == false) {
                        candidate_normal_NUM++;
                        _install_BC(candidateNor, graph_Node, Node);

                    }
                }
            }
            // no collision-free candidate, use the orginal one (temp)
            if (candidate_normal_NUM == 0) {
                _install_BC(Node->m_printNor, graph_Node, Node);
            }
        }
        else {
            _install_BC(Node->m_printNor, graph_Node, Node);
        }
    }

    //protect: no candidate normal is danger
    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        bool exist_Candidate = false;
        for (int i = 0; i < graph_Node.size(); i++) {

            if (graph_Node[i].waypoint_N_i == Node) {
                exist_Candidate = true;
                break;
            }
        }
        if (!exist_Candidate) std::cout << "The Node " << Node->GetIndexNo()
            << " in patch " << patch->rootPatch_jumpPatch->GetIndexNo()
            << " has no collision-free normal!" << std::endl;
    }
}

void GcodeGeneration::_get_GraphNode_List_newConfig(QMeshPatch* patch, std::vector<collision_Node>& graph_Node) {

    // get the patch polygenMesh_PrintHead
    QMeshPatch* eHeadPatch = NULL;
    for (GLKPOSITION posMesh = m_CncPart->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
        QMeshPatch* thisPatch = (QMeshPatch*)m_CncPart->GetMeshList().GetNext(posMesh);

        if (thisPatch->patchName == "nozzle_4C") {
            eHeadPatch = thisPatch;
            break;
        }
    }
    if (eHeadPatch == NULL) { std::cout << "PrintHead is NULL, please check." << std::endl; return; }


    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        if (Node->isCollision) {

            double candidate_normal_NUM = 0;
            for (int ZrotateAngle = 0; ZrotateAngle < 360; ZrotateAngle = ZrotateAngle + delta_Z) {
                for (int XtiltAngle = 90; XtiltAngle > 0; XtiltAngle = XtiltAngle - delta_X) {

                    double rad_ZrotateAngle = DEGREE_TO_ROTATE(ZrotateAngle);
                    double rad_XtiltAngle = DEGREE_TO_ROTATE(XtiltAngle);

                    Eigen::Vector3d candidateNor = _calCandidateNormal(Node->m_printNor, rad_ZrotateAngle, rad_XtiltAngle);
                    //cout << "candidateNor:\n " << candidateNor << endl;

                    bool iscollision_candidate_cNode = false;
                    _locate_EHead_printPos(eHeadPatch, Node->m_printPos, candidateNor);
                    // Test all of the previous-layers' node +-+-+ all of the previous-node before the deteted node at the same layer
                    std::vector<QMeshPatch*> check_below_WpSet;
                    int layerLoop = 0;
                    // collect the Waypoint patch before the printed node now
                    for (GLKPOSITION prevPos = m_Waypoints->GetMeshList().Find(patch->rootPatch_jumpPatch); prevPos;) {
                        QMeshPatch* prevWpPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetPrev(prevPos);

                        if (layerLoop >= layerDepth) break;

                        check_below_WpSet.push_back(prevWpPatch);

                        layerLoop++;
                    }
                    // speed up the code about the _checkSingleNodeCollision();
#pragma omp parallel
                    {
#pragma omp for  
                        for (int i = 0; i < check_below_WpSet.size(); i++) {

                            for (GLKPOSITION prevWpNodePos = check_below_WpSet[i]->GetNodeList().GetHeadPosition(); prevWpNodePos;) {
                                QMeshNode* prevWpNode = (QMeshNode*)check_below_WpSet[i]->GetNodeList().GetNext(prevWpNodePos);

                                if (patch->rootPatch_jumpPatch->GetIndexNo() == check_below_WpSet[i]->GetIndexNo()) {
                                    if (prevWpNode->GetIndexNo() >= Node->GetIndexNo()) continue;
                                }

                                bool isInHull = _isPnt_in_mesh(eHeadPatch, prevWpNode->m_printPos);

                                if (isInHull) {
                                    iscollision_candidate_cNode = true;
                                    break;
                                }
                            }
                            if (iscollision_candidate_cNode == true) break;
                        }
                    }

                    //add platform collision detection
                    for (GLKPOSITION eHead_NodePos = eHeadPatch->GetNodeList().GetHeadPosition(); eHead_NodePos;) {
                        QMeshNode* eHead_Node = (QMeshNode*)eHeadPatch->GetNodeList().GetNext(eHead_NodePos);

                        double eHead_xx, eHead_yy, eHead_zz = 0.0;
                        eHead_Node->GetCoord3D(eHead_xx, eHead_yy, eHead_zz);
                        // this is decided by Zup and 0-height table.
                        if (eHead_zz < 0.0) {
                            iscollision_candidate_cNode = true;
                            break;
                        }
                    }


                    if (iscollision_candidate_cNode == false) {
                        candidate_normal_NUM++;
                        _install_BC_newConfig(candidateNor, graph_Node, Node);

                    }
                }
            }
            // no collision-free candidate, use the orginal one (temp)
            if (candidate_normal_NUM == 0) {
                _install_BC_newConfig(Node->m_printNor, graph_Node, Node);
            }
        }
        else {
            _install_BC_newConfig(Node->m_printNor, graph_Node, Node);
        }
    }

    //protect: no candidate normal is danger
    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        bool exist_Candidate = false;
        for (int i = 0; i < graph_Node.size(); i++) {

            if (graph_Node[i].waypoint_N_i == Node) {
                exist_Candidate = true;
                break;
            }
        }
        if (!exist_Candidate) std::cout << "The Node " << Node->GetIndexNo()
            << " in patch " << patch->rootPatch_jumpPatch->GetIndexNo()
            << " has no collision-free normal!" << std::endl;
    }
}

Eigen::Vector3d GcodeGeneration::_calCandidateNormal(Eigen::Vector3d normal, double rad_ZrotateAngle, double rad_XtiltAngle) {

    Eigen::Matrix3d Xrot_Matrix, Zrot_Matrix, Xback_Matrix, Zback_Matrix;
    Eigen::Vector2d normalXY_temp;
    Eigen::Vector3d candidateNormal, candidateNor_temp;

    normalXY_temp << normal(0), normal(1);// (nx, ny)
    double alpha = atan2(normal(0), normal(1));// normal Z rotate
    double beta = atan2(normalXY_temp.norm(), normal(2));// normal X rotate
    Xback_Matrix << 1, 0, 0, 0, cos(-beta), -sin(-beta), 0, sin(-beta), cos(-beta);
    Zback_Matrix << cos(-alpha), -sin(-alpha), 0, sin(-alpha), cos(-alpha), 0, 0, 0, 1;

    Xrot_Matrix << 1, 0, 0, 0, cos(rad_XtiltAngle), -sin(rad_XtiltAngle), 0, sin(rad_XtiltAngle), cos(rad_XtiltAngle);
    Zrot_Matrix << cos(rad_ZrotateAngle), -sin(rad_ZrotateAngle), 0, sin(rad_ZrotateAngle), cos(rad_ZrotateAngle), 0, 0, 0, 1;
    candidateNor_temp = (Zrot_Matrix * Xrot_Matrix).col(2);// extract last vector of Z direction

    candidateNormal = Zback_Matrix * Xback_Matrix * candidateNor_temp;
    return candidateNormal;
}

void GcodeGeneration::_install_BC(Eigen::Vector3d temp_Normal, 
    std::vector<collision_Node>& graph_Node, QMeshNode* sourceNode) {

    collision_Node cNode_1;
    cNode_1.B_value = ROTATE_TO_DEGREE(-_safe_acos(temp_Normal(2)));
    cNode_1.C_value = ROTATE_TO_DEGREE(atan2(temp_Normal(0), temp_Normal(1)));
    cNode_1.waypoint_N_i = sourceNode;

    // solve 2
    collision_Node cNode_2;
    cNode_2.B_value = -cNode_1.B_value;
    double C2temp = cNode_1.C_value + 180.0;
    if (C2temp > 180.0)	C2temp -= 360.0; // control the range of C2 into the (-180,180]
    cNode_2.C_value = C2temp;
    cNode_2.waypoint_N_i = sourceNode;

    //keep the first is less than second BC solve
    if (fabs(cNode_1.C_value) < fabs(cNode_2.C_value)) {
        graph_Node.push_back(cNode_1);
        graph_Node.push_back(cNode_2);
    }
    else {
        graph_Node.push_back(cNode_2);
        graph_Node.push_back(cNode_1);
    }

    //keep the first is Solution 1   
    //graph_Node.push_back(cNode_1);
    //graph_Node.push_back(cNode_2);

    //keep use Solution 1
    //graph_Node.push_back(cNode_2);
    //graph_Node.push_back(cNode_2);
}

void GcodeGeneration::_install_BC_newConfig(Eigen::Vector3d temp_Normal,
    std::vector<collision_Node>& graph_Node, QMeshNode* sourceNode) {

    collision_Node cNode_1;
    cNode_1.B_value = ROTATE_TO_DEGREE(-_safe_acos(temp_Normal(2)));
    cNode_1.C_value = ROTATE_TO_DEGREE(-atan2(temp_Normal(0), temp_Normal(1)));
    cNode_1.waypoint_N_i = sourceNode;

    // solve 2
    collision_Node cNode_2;
    cNode_2.B_value = -cNode_1.B_value;
    double C2temp = cNode_1.C_value + 180.0;
    if (C2temp > 180.0)	C2temp -= 360.0; // control the range of C2 into the (-180,180]
    cNode_2.C_value = C2temp;
    cNode_2.waypoint_N_i = sourceNode;

    //keep the first is less than second BC solve
    if (fabs(cNode_1.C_value) < fabs(cNode_2.C_value)) {
        graph_Node.push_back(cNode_1);
        graph_Node.push_back(cNode_2);
    }
    else {
        graph_Node.push_back(cNode_2);
        graph_Node.push_back(cNode_1);
    }

    //keep the first is Solution 1   
    //graph_Node.push_back(cNode_1);
    //graph_Node.push_back(cNode_2);

    //keep use Solution 1
    //graph_Node.push_back(cNode_2);
    //graph_Node.push_back(cNode_2);
}

void GcodeGeneration::_build_Graph(QMeshPatch* patch, std::vector<collision_Node>& graph_Node) {

    int V = graph_Node.size();
    std::vector<int> startNode_NoSet;
    std::vector<int> endNode_NoSet;

    //get index of end Nodes of graph
    for (int i = 0; i < graph_Node.size(); i++) {

        if (graph_Node[i].waypoint_N_i->Jump_SecIndex == 0)
            startNode_NoSet.push_back(i);

        if (graph_Node[i].waypoint_N_i->Jump_SecIndex == (patch->GetNodeNumber() - 1))
            endNode_NoSet.push_back(i);
    }

    Graph g(V, endNode_NoSet);

    for (int i = 0; i < graph_Node.size(); i++) {
        for (int j = (i + 1); j < graph_Node.size(); j++) {

            if ((graph_Node[i].waypoint_N_i->Jump_SecIndex + 1)
                == graph_Node[j].waypoint_N_i->Jump_SecIndex) {

                double weight = _weight_calculation(
                    graph_Node[i].B_value, graph_Node[i].C_value,
                    graph_Node[j].B_value, graph_Node[j].C_value);

                //if (i == graph_Node.size() / 2) weight *= 5;

                g.addEdge(i, weight, j);
            }
        }
    }

    g.shortestPath(0);
    for (int i = 0; i < g.path.size(); i++) {
        graph_Node[g.path[i]].waypoint_N_i->m_XYZBCE(3) = graph_Node[g.path[i]].B_value;
        graph_Node[g.path[i]].waypoint_N_i->m_XYZBCE(4) = graph_Node[g.path[i]].C_value;
        //std::cout << g.path[i] << " ";
    }
    //std::cout << std::endl;

    //BC -> normal -> show
    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        double B = DEGREE_TO_ROTATE(Node->m_XYZBCE(3));
        double C = DEGREE_TO_ROTATE(Node->m_XYZBCE(4));

        double finalNx = -sin(B) * sin(C);
        double finalNy = -sin(B) * cos(C);
        double finalNz = cos(B);

        Node->m_printNor << finalNx, finalNy, finalNz;
        Node->SetNormal(finalNx, finalNy, finalNz);
    }
}

void GcodeGeneration::_build_Graph_newConfig(QMeshPatch* patch, std::vector<collision_Node>& graph_Node) {

    int V = graph_Node.size();
    std::vector<int> startNode_NoSet;
    std::vector<int> endNode_NoSet;

    //get index of end Nodes of graph
    for (int i = 0; i < graph_Node.size(); i++) {

        if (graph_Node[i].waypoint_N_i->Jump_SecIndex == 0)
            startNode_NoSet.push_back(i);

        if (graph_Node[i].waypoint_N_i->Jump_SecIndex == (patch->GetNodeNumber() - 1))
            endNode_NoSet.push_back(i);
    }

    Graph g(V, endNode_NoSet);

    for (int i = 0; i < graph_Node.size(); i++) {
        for (int j = (i + 1); j < graph_Node.size(); j++) {

            if ((graph_Node[i].waypoint_N_i->Jump_SecIndex + 1)
                == graph_Node[j].waypoint_N_i->Jump_SecIndex) {

                double weight = _weight_calculation(
                    graph_Node[i].B_value, graph_Node[i].C_value,
                    graph_Node[j].B_value, graph_Node[j].C_value);

                //if (i == graph_Node.size() / 2) weight *= 5;

                g.addEdge(i, weight, j);
            }
        }
    }

    g.shortestPath(0);
    for (int i = 0; i < g.path.size(); i++) {
        graph_Node[g.path[i]].waypoint_N_i->m_XYZBCE(3) = graph_Node[g.path[i]].B_value;
        graph_Node[g.path[i]].waypoint_N_i->m_XYZBCE(4) = graph_Node[g.path[i]].C_value;
        //std::cout << g.path[i] << " ";
    }
    //std::cout << std::endl;

    //BC -> normal -> show
    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        double B = DEGREE_TO_ROTATE(Node->m_XYZBCE(3));
        double C = DEGREE_TO_ROTATE(Node->m_XYZBCE(4));

        double finalNx = sin(B) * sin(C);
        double finalNy = -sin(B) * cos(C);
        double finalNz = cos(B);

        Node->m_printNor << finalNx, finalNy, finalNz;
        Node->SetNormal(finalNx, finalNy, finalNz);
    }
}

double GcodeGeneration::_weight_calculation(double B_i, double C_i, double B_ii, double C_ii) {

    double rad_B_i = DEGREE_TO_ROTATE(B_i);
    double rad_C_i = DEGREE_TO_ROTATE(C_i);
    double rad_B_ii = DEGREE_TO_ROTATE(B_ii);
    double rad_C_ii = DEGREE_TO_ROTATE(C_ii);

    Eigen::Vector2d v_C_i = { cos(rad_C_i),sin(rad_C_i) };
    Eigen::Vector2d v_C_ii = { cos(rad_C_ii),sin(rad_C_ii) };

    //compute the actural angle of 2 vectors
    double rad_delta_B = fabs(rad_B_i - rad_B_ii);
    double rad_delta_C = _safe_acos(v_C_i.dot(v_C_ii));

    return (rad_delta_B + rad_delta_C);							    //L1-Norm
    //return std::sqrt(pow(rad_delta_B, 2) + pow(rad_delta_C, 2));	//L2-Norm
}

//Function for feedrate optimization
void GcodeGeneration::feedrateOpt() {

    double base_Feedrate = 600.0;
    double max_Feedrate = 2500.0;
    double min_Feedrate = 500.0;

    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        if (WayPointPatch->GetIndexNo() < m_FromIndex || WayPointPatch->GetIndexNo() > m_ToIndex) continue;

        double last_feedRate = 0.0;
        for (GLKPOSITION nodePos = WayPointPatch->GetNodeList().GetHeadPosition(); nodePos;) {
            QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(nodePos);

            double feedRate = 0.0;
            int lines = WayPointPatch->GetNodeNumber();
            if (Node->GetIndexNo() == (lines - 1)) { feedRate = last_feedRate; }
            else {

                GLKPOSITION nextPos = WayPointPatch->GetNodeList().Find(Node)->next;
                QMeshNode* nextNode = (QMeshNode*)WayPointPatch->GetNodeList().GetAt(nextPos);

                double l = (Node->m_printPos - nextNode->m_printPos).norm(); //(l: Euclidean distance)    
                double d = (Node->m_XYZBCE - nextNode->m_XYZBCE).norm(); //(d: Joint space distance)

                if (Node->Jump_preSecEnd) {
                    feedRate = last_feedRate;
                }
                else {
                    //feedRate = base_Feedrate * d / l;
                    feedRate = base_Feedrate * sqrt(d);
                }

                if (Node->Jump_nextSecStart && Node->Jump_preSecEnd)
                {
                    std::cout << "Error: the node cannot be start and end at the same time!" << std::endl;
                    return;
                }

                if (feedRate >= max_Feedrate) {
                    feedRate = max_Feedrate;
                    //std::cout << "more than max: node ind: "<< Node->GetIndexNo() << " l: " << l << " d: " << d << std::endl;
                }
                if (feedRate <= min_Feedrate) {
                    feedRate = min_Feedrate;
                    //std::cout << "less than min: node ind: " << Node->GetIndexNo() << " l: " << l << " d: " << d << std::endl;
                }
                    
            }

            //if (is_planar_printing) feedRate = 500;
            Node->m_F = feedRate;
            last_feedRate = feedRate;
        }
    }
}

/*******************************************/
/* Main Function for Gcode Write_continousPrint */
/*******************************************/

void GcodeGeneration::writeGcode(std::string GcodeDir) {
    std::cout << "------------------------------------------- " << GcodeDir << " Gcode Writing ..." << std::endl;

    // First varify the tip height is larger than 0.0; IMPORTANT
    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        if (WayPointPatch->GetIndexNo() < m_FromIndex || WayPointPatch->GetIndexNo() > m_ToIndex) continue;

        Eigen::Vector4d tipPos_4d_initial = Eigen::Vector4d::Zero();    tipPos_4d_initial << 0.0, 0.0, 0.0, 1.0;
        Eigen::Vector4d tipPos_4d = Eigen::Vector4d::Zero();

        for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);
            double X = Node->m_XYZBCE(0); double Y = Node->m_XYZBCE(1); double Z = Node->m_XYZBCE(2);
            double B = Node->m_XYZBCE(3); double C = Node->m_XYZBCE(4); double E = Node->m_XYZBCE(5);

            double rad_B = DEGREE_TO_ROTATE(B);
            double rad_C = DEGREE_TO_ROTATE(C);

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

            tipPos_4d = Offset * Rot_B * Offset_back * tipPos_4d_initial;

            if (tipPos_4d[2] < 0.0) {

                std::cout << "error: Tip will hit the bottom, DANGER!!!" << std::endl;
                return;
            }
        }
    }

    // Define the basic parameter
    double Z_home = 250;						// The hight of Home point; / mm
    double Z_high = 5;// h;  						// The hight of G1 point(for safety); / mm
    if (is_planar_printing) Z_high = 2.0;
    double Z_compensateUpDistance = 2;			// The hight of waiting distance of extruder; / mm

    int F_G1_move = 3000;                       // Speed of move
    int F_G1_1stlayer = 650;				    // Speed of G1(special 1st layer)
    int F_G1_original = 750;					// Speed of G1 original material (normal 2ed~layers)
    int F_G1_support = F_G1_original;			// Speed of G1 support material (normal 2ed~layers)
    int F_PumpBack = 6000;						// Speed of F_PumpBack
    int F_PumpCompensate = 900;				    // Speed of PumpCompensate

    double E_PumpBack = -12.0; 					// The extruder pump back Xmm
    double E_PumpCompensate = 10.0;				// The extruder pump compensate Xmm
    double E_PumpCompensateL1 = 10;				// The extruder pump compensate for 1st layer Xmm
    double E_PumpCompensateNewE = 10;			// The extruder pump compensate for new type layer Xmm

    char targetFilename[1024];
    std::sprintf(targetFilename, "%s%s", "../DataSet/G_CODE/", GcodeDir.c_str());
    FILE* fp = fopen(targetFilename, "w");
    if (!fp) {
        perror("Couldn't open the directory");
        return;
    }

    // Record the max Z for security consideration (the first printed layer)
    GLKPOSITION layer1st_Pos = m_Waypoints->GetMeshList().FindIndex(m_FromIndex);
    QMeshPatch* layer1st_WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetAt(layer1st_Pos);

    double Z_max = -99999.9;
    for (GLKPOSITION Pos = layer1st_WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)layer1st_WayPointPatch->GetNodeList().GetNext(Pos);

        if (Node->m_XYZBCE(2) > Z_max) { Z_max = Node->m_XYZBCE(2); }
    }
    // Record the layer type of the first printed layer
    bool IsSupportLayer_last = layer1st_WayPointPatch->is_SupportLayer;

    // Give the start message of G_code
    std::fprintf(fp, "G21\n");
    std::fprintf(fp, "G40\n");
    std::fprintf(fp, "G49\n");
    std::fprintf(fp, "G80\n");
    std::fprintf(fp, "G90\n");
    std::fprintf(fp, "M5\n");
    std::fprintf(fp, "T1 M6\n");
    std::fprintf(fp, "G54\n");
    std::fprintf(fp, "(Position 1)\n");
    std::fprintf(fp, "G94\n");
    std::fprintf(fp, "G1 X0.000 Y0.000 Z%.2f B0.000 C0.000 F%d\n", Z_home, F_G1_move);

    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        if (WayPointPatch->GetIndexNo() < m_FromIndex || WayPointPatch->GetIndexNo() > m_ToIndex) continue;

        bool showOnece = true; // show the Z < 0 once

        for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);
            double X = Node->m_XYZBCE(0); double Y = Node->m_XYZBCE(1); double Z = Node->m_XYZBCE(2);
            double B = Node->m_XYZBCE(3); double C = Node->m_XYZBCE(4); double E = Node->m_XYZBCE(5);
            int F = (int)Node->m_F;

            if (WayPointPatch->GetIndexNo() == 0) E *= 1.0; // increase extrusion of 1st layer
            //if (WayPointPatch->GetIndexNo() == m_FromIndex) E *= 1.04;// increase the extrusion of the first printed layer
            if (WayPointPatch->is_SupportLayer) E *= 0.6; // decrease extrusion of support layer (temp)

            // check the huge change of C angle
            if (Node->GetIndexNo() != 0)
            {
                GLKPOSITION prevPos = WayPointPatch->GetNodeList().Find(Node)->prev;
                QMeshNode* prevNode = (QMeshNode*)WayPointPatch->GetNodeList().GetAt(prevPos);

                double C_prev = prevNode->m_XYZBCE(4);

                if (fabs(C - C_prev) > 300
                    && prevNode->Jump_preSecEnd == false && Node->Jump_nextSecStart == false) {

                    std::cerr << "fabs(C - C_prev) ERROR! " << fabs(C - C_prev) << std::endl;
                    std::cout << "WayPointPatch->GetIndexNo() " << WayPointPatch->GetIndexNo() << std::endl;
                }

            }
            // check the negative Z motion
            if (Z < -(h - 10) && showOnece) {
                std::cout << "Layer: " << WayPointPatch->GetIndexNo() << " Z < -h hit the bottom " << std::endl;
                showOnece = false;
            }

            // Record the max Z for security consideration (rest layers)
            if (Z > Z_max) { Z_max = Z; }

            //---------------------------------------------------------------------------------------------------------------------------------
            // meed to be verified
            B = -B; C = -C;
            //---------------------------------------------------------------------------------------------------------------------------------

            // Add some auxiliary G code
            if (Node->GetIndexNo() == 0) {// for the 1st point
                // for the 1st(LayerInd_From) printed layer
                if (WayPointPatch->GetIndexNo() == m_FromIndex) {
                    // move to start of printing location
                    std::fprintf(fp, "G1 X%.2f Y%.2f B%.2f C%.2f F%d\n", X, Y, B, C, F_G1_move);
                    // slowly lower for printing
                    std::fprintf(fp, "G1 Z%.2f F%d\n", (Z_max + Z_compensateUpDistance), F_G1_move);
                    // zero extruded length(set E axis to 0)
                    std::fprintf(fp, "G92 A0\n");
                    std::fprintf(fp, "G1 A%.2f F%d\n", E_PumpCompensateL1, F_PumpCompensate);
                    std::fprintf(fp, "G92 A0\n");
                    std::fprintf(fp, "G1 F%d\n", F_G1_1stlayer);
                }
                //new layer and same extruder
                else if (WayPointPatch->is_SupportLayer == IsSupportLayer_last) {

                    // return to the safe point Z_max + Z_high
                    std::fprintf(fp, "G1 Z%.2f F%d\n", (Z_max + Z_high), F_G1_move);

                    std::fprintf(fp, "G92 A0\n");
                    std::fprintf(fp, "G1 A%.2f F%d\n", E_PumpBack, F_PumpBack);
                    std::fprintf(fp, "G92 A0\n");
                    //// return to the safe point Z_max + Z_high (move this to front of retraction of extrusion)
                    //std::fprintf(fp, "G1 Z%.2f F%d\n", (Z_max + Z_high), F_G1_move);
                    // move to start of printing location
                    std::fprintf(fp, "G1 X%.2f Y%.2f B%.2f C%.2f F%d\n", X, Y, B, C, F_G1_move);
                    // slowly lower for printing
                    std::fprintf(fp, "G1 Z%.2f F%d\n", (Z + Z_compensateUpDistance), F_G1_move);
                    // compensate extrusion
                    std::fprintf(fp, "G1 A%.2f F%d\n", E_PumpCompensate, F_PumpCompensate);
                    std::fprintf(fp, "G92 A0\n");
                    std::fprintf(fp, "G1 F%d\n", F_G1_original);

                }
                // case: exchange extrude material
                else {
                    std::fprintf(fp, "G92 A0\n");
                    std::fprintf(fp, "G1 A%.2f F%d\n", E_PumpBack, F_PumpBack);
                    // return to the home point Z_home
                    std::fprintf(fp, "G1 Z%.2f F%d\n", Z_home, F_G1_move);
                    std::fprintf(fp, "G92 A0\n");
                    // move to start of printing location
                    std::fprintf(fp, "G1 X%.2f Y%.2f B%.2f C%.2f F%d\n", X, Y, B, C, F_G1_move);
                    std::fprintf(fp, "G1 Z%.2f F%d\n", (Z + Z_compensateUpDistance), F_G1_move);
                    // slowly lower for printing
                    std::fprintf(fp, "G1 A%.2f F%d\n", E_PumpCompensateNewE, F_PumpCompensate);
                    std::fprintf(fp, "G92 A0\n");
                    std::fprintf(fp, "G1 F%d\n", F_G1_support);
                }
                std::fprintf(fp, "G1 X%.2f Y%.2f Z%.2f B%.2f C%.2f A%.2f F%d\n", X, Y, Z, B, C, E, F);

                //add by tianyu 10/04/2022
                //output the insert information

                if (Node->insertNodesInfo.size() != 0) {
                    std::cout << "\n\n I am here! \n" << std::endl;
                    std::cout << "Node->insertNodesInfo.size()" << Node->insertNodesInfo.size() << std::endl;
                    for (int i = 0; i < Node->insertNodesInfo.size(); i++) {
                        std::fprintf(fp, "G1 X%.2f Y%.2f Z%.2f B%.2f C%.2f A%.2f F%d\n",
                            Node->insertNodesInfo[i](0), Node->insertNodesInfo[i](1), Node->insertNodesInfo[i](2),
                            -Node->insertNodesInfo[i](3), -Node->insertNodesInfo[i](4), Node->insertNodesInfo[i](5)), F;
                    }
                }
            }
            else {
                // Consider the waypoints with too large Length //OR large Singularity areas
                if (Node->Jump_nextSecStart
                    //|| Node->isSingularSecEndNode 	
                    //|| Node->isCollisionStart 	|| Node->isCollisionEnd 
                    //|| Node->isNegZStart || Node->isNegZEnd
                    || Node->is_PiFlip_JumpEnd
                    ) {

                    if (WayPointPatch->is_SupportLayer == true) {

                        std::fprintf(fp, "G1 A%.2f F%d\n", (E + E_PumpBack * 0.8), F_PumpBack);
                        std::fprintf(fp, "G1 Z%.2f F%d\n", (Z_max + Z_high), F_G1_move);
                        std::fprintf(fp, "G1 X%.2f Y%.2f B%.2f C%.2f F%d\n", X, Y, B, C, F_G1_move);
                        std::fprintf(fp, "G1 Z%.2f F%d\n", (Z + Z_compensateUpDistance), F_G1_move);
                        std::fprintf(fp, "G1 A%.2f F%d\n", (E - 0.1), F_PumpCompensate);
                        std::fprintf(fp, "G1 Z%.2f A%.2f F%d\n", Z, E, F_G1_support);
                    }
                    else {

                        std::fprintf(fp, "G1 A%.2f F%d\n", (E + E_PumpBack), F_PumpBack);
                        std::fprintf(fp, "G1 Z%.2f F%d\n", (Z_max + Z_high), F_G1_move);
                        std::fprintf(fp, "G1 X%.2f Y%.2f B%.2f C%.2f F%d\n", X, Y, B, C, F_G1_move);
                        std::fprintf(fp, "G1 Z%.2f F%d\n", (Z + Z_compensateUpDistance), F_G1_move);
                        std::fprintf(fp, "G1 A%.2f F%d\n", (E - 0.1), F_PumpCompensate);
                        std::fprintf(fp, "G1 Z%.2f A%.2f F%d\n", Z, E, F_G1_original);
                    }
                }
                std::fprintf(fp, "G1 X%.2f Y%.2f Z%.2f B%.2f C%.2f A%.2f F%d\n", X, Y, Z, B, C, E, F);

                //add by tianyu 10/04/2022
                //output the insert information

                if (Node->insertNodesInfo.size() != 0) {
                    std::cout << "\n\n I am here! \n" << std::endl;
                    std::cout << "Node->insertNodesInfo.size()" << Node->insertNodesInfo.size() << std::endl;
                    for (int i = 0; i < Node->insertNodesInfo.size(); i++) {
                        std::fprintf(fp, "G1 X%.2f Y%.2f Z%.2f B%.2f C%.2f A%.2f F%d\n",
                            Node->insertNodesInfo[i](0), Node->insertNodesInfo[i](1), Node->insertNodesInfo[i](2),
                            -Node->insertNodesInfo[i](3), -Node->insertNodesInfo[i](4), Node->insertNodesInfo[i](5), F);
                    }
                }
            }
        }
        IsSupportLayer_last = WayPointPatch->is_SupportLayer;
    }

    std::fprintf(fp, "G92 A0\n");
    std::fprintf(fp, "G1 A%.2f F%d\n", E_PumpBack, F_G1_move); // PumpBack
    std::fprintf(fp, "G1 Z%.2f F%d\n", Z_home, F_G1_move); // return to the home point Z_home
    std::fprintf(fp, "G1 X0.0 Y0.0 B0.0 C0.0 F%d\n", F_G1_move);
    std::fprintf(fp, "M30\n");// Stop all of the motion

    std::fclose(fp);

    std::cout << "------------------------------------------- " << GcodeDir << " Gcode Write Finish!\n" << std::endl;
}

void GcodeGeneration::readGcodeFile(Eigen::MatrixXf& Gcode_Table, std::string FileName) {

    char targetFilename[1024];
    std::sprintf(targetFilename, "%s%s", "../DataSet/G_CODE/", FileName.c_str());
    FILE* fp; char linebuf[2048];
    double machine_X = 0.0, machine_Y = 0.0, machine_Z = 300.0, machine_B = 0.0, machine_C = 0.0;
    fp = fopen(targetFilename, "r");
    if (!fp) {
        printf("===============================================\n");
        printf("Can not open the data file - Gcode File Import!\n");
        printf("===============================================\n");
        return;
    }

    //get the num of lines in Gcode file.
    int lines = 0;
    while (true) {
        fgets(linebuf, 255, fp);
        if (feof(fp)) break;
        lines++;
    }
    std::fclose(fp);
    //std::cout << lines << std::endl;

    fp = fopen(targetFilename, "r");
    Gcode_Table = Eigen::MatrixXf::Zero(lines, 6);
    bool T3_flag = false;
    for (int i = 0; i < lines; i++) {

        double newLayerFlag = 0.0;// DOUBLE type is for the compactness of data structure

        fgets(linebuf, 255, fp);
        //std::cout << linebuf;

        std::string str = linebuf;
        std::string::size_type position_X = str.find("X");  std::string::size_type position_Y = str.find("Y");
        std::string::size_type position_Z = str.find("Z");
        std::string::size_type position_B = str.find("B");	std::string::size_type position_C = str.find("C");
        std::string::size_type position_E = str.find("A");	std::string::size_type position_F = str.find("F");

        //std::cout << position_X << " " << position_Y << " " << position_Z << " " << position_B << " " << position_C << std::endl;

        std::string::size_type GFlag = str.find("G");
        if (GFlag != std::string::npos) {
            // G1 X0.000 Y0.000 Z250.000 B0.000 C0.000 F2000
            if (position_X != str.npos && position_Y != str.npos && position_Z != str.npos
                && position_B != str.npos && position_C != str.npos
                && position_E == str.npos && position_F != str.npos) {

                std::string X_temp = str.substr(position_X + 1, position_Y - position_X - 2);
                std::string Y_temp = str.substr(position_Y + 1, position_Z - position_Y - 2);
                std::string Z_temp = str.substr(position_Z + 1, position_B - position_Z - 2);
                std::string B_temp = str.substr(position_B + 1, position_C - position_B - 2);
                std::string C_temp = str.substr(position_C + 1, position_F - position_C - 2);

                machine_X = atof(X_temp.c_str());	machine_Y = atof(Y_temp.c_str());	machine_Z = atof(Z_temp.c_str());
                machine_B = atof(B_temp.c_str());	machine_C = atof(C_temp.c_str());

            }
            // G1 X-2.207 Y88.771 Z114.490 B55.324 C-861.683 A782.472 F2000
            // the most common case
            else if (position_X != str.npos && position_Y != str.npos && position_Z != str.npos
                && position_B != str.npos && position_C != str.npos
                && position_E != str.npos && position_F != str.npos) {

                std::string X_temp = str.substr(position_X + 1, position_Y - position_X - 2);
                std::string Y_temp = str.substr(position_Y + 1, position_Z - position_Y - 2);
                std::string Z_temp = str.substr(position_Z + 1, position_B - position_Z - 2);
                std::string B_temp = str.substr(position_B + 1, position_C - position_B - 2);
                std::string C_temp = str.substr(position_C + 1, position_E - position_C - 2);

                machine_X = atof(X_temp.c_str());	machine_Y = atof(Y_temp.c_str());	machine_Z = atof(Z_temp.c_str());
                machine_B = atof(B_temp.c_str());	machine_C = atof(C_temp.c_str());

            }
            // G1 X2.003 Y87.702 B55.445 C51.857 F2000
            else if (position_X != str.npos && position_Y != str.npos && position_Z == str.npos
                && position_B != str.npos && position_C != str.npos
                && position_E == str.npos && position_F != str.npos) {

                std::string X_temp = str.substr(position_X + 1, position_Y - position_X - 2);
                std::string Y_temp = str.substr(position_Y + 1, position_B - position_Y - 2);
                std::string B_temp = str.substr(position_B + 1, position_C - position_B - 2);
                std::string C_temp = str.substr(position_C + 1, position_F - position_C - 2);

                machine_X = atof(X_temp.c_str());	machine_Y = atof(Y_temp.c_str());
                machine_B = atof(B_temp.c_str());	machine_C = atof(C_temp.c_str());
            }
            // G1 Z2.940 F4750
            else if (position_X == str.npos && position_Y == str.npos && position_Z != str.npos
                && position_B == str.npos && position_C == str.npos
                && position_E == str.npos && position_F != str.npos) {

                std::string Z_temp = str.substr(position_Z + 1, position_F - position_Z - 2);

                machine_Z = atof(Z_temp.c_str());
            }
            // G1 Z3.022 E562.31 F4750
            else if (position_X == str.npos && position_Y == str.npos && position_Z != str.npos
                && position_B == str.npos && position_C == str.npos
                && position_E != str.npos && position_F != str.npos) {

                std::string Z_temp = str.substr(position_Z + 1, position_E - position_Z - 2);

                machine_Z = atof(Z_temp.c_str());

            }
            // G1 F1500 // for new layer flag
            else if (position_X == str.npos && position_Y == str.npos && position_Z == str.npos
                && position_B == str.npos && position_C == str.npos
                && position_E == str.npos && position_F != str.npos) {

                newLayerFlag = 1.0;
            }
            // test for special case
            // else { cout << "------------------------------------------ some special case" << endl; }

        }
        // test for special case
        // else { cout << "------------------------------------------ some special case" << endl; }
        //std::cout << "X: " << machine_X << " Y: " << machine_Y << " Z: " << machine_Z
        //	<< " B: " << machine_B << " C: " << machine_C << std::endl << std::endl;

        Gcode_Table.row(i) << machine_X, machine_Y, machine_Z, -machine_B, -machine_C, newLayerFlag;
    }
    std::fclose(fp);

    std::cout << "Value range of X axis: [" << Gcode_Table.col(0).maxCoeff()
        << ", " << Gcode_Table.col(0).minCoeff() << "]" << std::endl;
    std::cout << "Value range of Y axis: [" << Gcode_Table.col(1).maxCoeff()
        << ", " << Gcode_Table.col(1).minCoeff() << "]" << std::endl;
    std::cout << "Value range of Z axis: [" << Gcode_Table.col(2).maxCoeff()
        << ", " << Gcode_Table.col(2).minCoeff() << "]" << std::endl;
    std::cout << "Value range of B axis: [" << Gcode_Table.col(3).maxCoeff()
        << ", " << Gcode_Table.col(3).minCoeff() << "]" << std::endl;
    std::cout << "Value range of C axis: [" << Gcode_Table.col(4).maxCoeff()
        << ", " << Gcode_Table.col(4).minCoeff() << "]" << std::endl;

    std::cout << "------------------------------------------- Gcode Load Finish!" << std::endl;

    this->_readCncData(2);
}

void GcodeGeneration::readGcodeFile_newConfig(Eigen::MatrixXf& Gcode_Table, std::string FileName) {

    char targetFilename[1024];
    std::sprintf(targetFilename, "%s%s", "../DataSet/G_CODE/", FileName.c_str());
    FILE* fp; char linebuf[2048];
    double machine_X = 0.0, machine_Y = 0.0, machine_Z = 300.0, machine_B = 0.0, machine_C = 0.0;
    fp = fopen(targetFilename, "r");
    if (!fp) {
        printf("===============================================\n");
        printf("Can not open the data file - Gcode File Import!\n");
        printf("===============================================\n");
        return;
    }

    //get the num of lines in Gcode file.
    int lines = 0;
    while (true) {
        fgets(linebuf, 255, fp);
        if (feof(fp)) break;
        lines++;
    }
    std::fclose(fp);
    //std::cout << lines << std::endl;

    fp = fopen(targetFilename, "r");
    Gcode_Table = Eigen::MatrixXf::Zero(lines, 6);
    bool T3_flag = false;
    for (int i = 0; i < lines; i++) {

        double newLayerFlag = 0.0;// DOUBLE type is for the compactness of data structure

        fgets(linebuf, 255, fp);
        //std::cout << linebuf;

        std::string str = linebuf;
        std::string::size_type position_X = str.find("X");  std::string::size_type position_Y = str.find("Y");
        std::string::size_type position_Z = str.find("Z");
        std::string::size_type position_B = str.find("B");	std::string::size_type position_C = str.find("C");
        std::string::size_type position_E = str.find("A");	std::string::size_type position_F = str.find("F");

        //std::cout << position_X << " " << position_Y << " " << position_Z << " " << position_B << " " << position_C << std::endl;

        std::string::size_type GFlag = str.find("G");
        if (GFlag != std::string::npos) {
            // G1 X0.000 Y0.000 Z250.000 B0.000 C0.000 F2000
            if (position_X != str.npos && position_Y != str.npos && position_Z != str.npos
                && position_B != str.npos && position_C != str.npos
                && position_E == str.npos && position_F != str.npos) {

                std::string X_temp = str.substr(position_X + 1, position_Y - position_X - 2);
                std::string Y_temp = str.substr(position_Y + 1, position_Z - position_Y - 2);
                std::string Z_temp = str.substr(position_Z + 1, position_B - position_Z - 2);
                std::string B_temp = str.substr(position_B + 1, position_C - position_B - 2);
                std::string C_temp = str.substr(position_C + 1, position_F - position_C - 2);

                machine_X = atof(X_temp.c_str());	machine_Y = atof(Y_temp.c_str());	machine_Z = atof(Z_temp.c_str());
                machine_B = atof(B_temp.c_str());	machine_C = atof(C_temp.c_str());

            }
            // G1 X-2.207 Y88.771 Z114.490 B55.324 C-861.683 A782.472 F2000
            // the most common case
            else if (position_X != str.npos && position_Y != str.npos && position_Z != str.npos
                && position_B != str.npos && position_C != str.npos
                && position_E != str.npos && position_F != str.npos) {

                std::string X_temp = str.substr(position_X + 1, position_Y - position_X - 2);
                std::string Y_temp = str.substr(position_Y + 1, position_Z - position_Y - 2);
                std::string Z_temp = str.substr(position_Z + 1, position_B - position_Z - 2);
                std::string B_temp = str.substr(position_B + 1, position_C - position_B - 2);
                std::string C_temp = str.substr(position_C + 1, position_E - position_C - 2);

                machine_X = atof(X_temp.c_str());	machine_Y = atof(Y_temp.c_str());	machine_Z = atof(Z_temp.c_str());
                machine_B = atof(B_temp.c_str());	machine_C = atof(C_temp.c_str());

            }
            // G1 X2.003 Y87.702 B55.445 C51.857 F2000
            else if (position_X != str.npos && position_Y != str.npos && position_Z == str.npos
                && position_B != str.npos && position_C != str.npos
                && position_E == str.npos && position_F != str.npos) {

                std::string X_temp = str.substr(position_X + 1, position_Y - position_X - 2);
                std::string Y_temp = str.substr(position_Y + 1, position_B - position_Y - 2);
                std::string B_temp = str.substr(position_B + 1, position_C - position_B - 2);
                std::string C_temp = str.substr(position_C + 1, position_F - position_C - 2);

                machine_X = atof(X_temp.c_str());	machine_Y = atof(Y_temp.c_str());
                machine_B = atof(B_temp.c_str());	machine_C = atof(C_temp.c_str());
            }
            // G1 Z2.940 F4750
            else if (position_X == str.npos && position_Y == str.npos && position_Z != str.npos
                && position_B == str.npos && position_C == str.npos
                && position_E == str.npos && position_F != str.npos) {

                std::string Z_temp = str.substr(position_Z + 1, position_F - position_Z - 2);

                machine_Z = atof(Z_temp.c_str());
            }
            // G1 Z3.022 E562.31 F4750
            else if (position_X == str.npos && position_Y == str.npos && position_Z != str.npos
                && position_B == str.npos && position_C == str.npos
                && position_E != str.npos && position_F != str.npos) {

                std::string Z_temp = str.substr(position_Z + 1, position_E - position_Z - 2);

                machine_Z = atof(Z_temp.c_str());

            }
            // G1 F1500 // for new layer flag
            else if (position_X == str.npos && position_Y == str.npos && position_Z == str.npos
                && position_B == str.npos && position_C == str.npos
                && position_E == str.npos && position_F != str.npos) {

                newLayerFlag = 1.0;
            }
            // test for special case
            // else { cout << "------------------------------------------ some special case" << endl; }

        }
        // test for special case
        // else { cout << "------------------------------------------ some special case" << endl; }
        //std::cout << "X: " << machine_X << " Y: " << machine_Y << " Z: " << machine_Z
        //	<< " B: " << machine_B << " C: " << machine_C << std::endl << std::endl;

        Gcode_Table.row(i) << machine_X, machine_Y, machine_Z, -machine_B, -machine_C, newLayerFlag;
    }
    std::fclose(fp);

    std::cout << "Value range of X axis: [" << Gcode_Table.col(0).maxCoeff()
        << ", " << Gcode_Table.col(0).minCoeff() << "]" << std::endl;
    std::cout << "Value range of Y axis: [" << Gcode_Table.col(1).maxCoeff()
        << ", " << Gcode_Table.col(1).minCoeff() << "]" << std::endl;
    std::cout << "Value range of Z axis: [" << Gcode_Table.col(2).maxCoeff()
        << ", " << Gcode_Table.col(2).minCoeff() << "]" << std::endl;
    std::cout << "Value range of B axis: [" << Gcode_Table.col(3).maxCoeff()
        << ", " << Gcode_Table.col(3).minCoeff() << "]" << std::endl;
    std::cout << "Value range of C axis: [" << Gcode_Table.col(4).maxCoeff()
        << ", " << Gcode_Table.col(4).minCoeff() << "]" << std::endl;

    std::cout << "------------------------------------------- Gcode Load Finish!" << std::endl;

    this->_readCncData(3);
}

// initial method only IK
void GcodeGeneration::cal_XYZBCE_test() {

    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        // give the message of XYZBC
        for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

            //cal E
            Node->m_XYZBCE(5) = 0.25 * Node->m_DHW(0);
            //cal BC
            double rad_B, rad_C;
            double nx, ny, nz;
            nx = Node->m_printNor(0); ny = Node->m_printNor(1); nz = Node->m_printNor(2);
            rad_B = -acos(nz);//rad
            rad_C = atan2(nx, ny); //rad
            //cal XYZ
            double px, py, pz;
            px = Node->m_printPos(0); py = Node->m_printPos(1); pz = Node->m_printPos(2);

            Node->m_XYZBCE(0) = px * cos(rad_C) - py * sin(rad_C);
            Node->m_XYZBCE(1) = px * sin(rad_C) + py * cos(rad_C) - h * sin(rad_B);
            Node->m_XYZBCE(2) = pz - h * (1 - cos(rad_B));
            //record BC_deg
            Node->m_XYZBCE(3) = ROTATE_TO_DEGREE(rad_B);
            Node->m_XYZBCE(4) = ROTATE_TO_DEGREE(rad_C);
        }
    }

    //absolute extrusion mode
    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        double absolute_E = 0.0;
        // give the message of absolute extrusion 
        for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

            Node->m_XYZBCE(5) = absolute_E + Node->m_XYZBCE(5);
            absolute_E = Node->m_XYZBCE(5);
        }
    }
    std::cout << "------------------------------------------- Method 1 :XYZBC calculation Finish!" << std::endl;
}