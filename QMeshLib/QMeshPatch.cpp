#define _CRT_SECURE_NO_DEPRECATE

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include "QMeshPatch.h"
#include "QMeshTetra.h"
#include "QMeshFace.h"
#include "QMeshEdge.h"
#include "QMeshNode.h"
#include <stddef.h>

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

QMeshPatch::QMeshPatch()
{
	indexno=0;
	for(int i=0;i<8;i++) flags[i]=false;
	nodeList.RemoveAll();
	edgeList.RemoveAll();
	faceList.RemoveAll();
	int num=faceList.GetCount();
}

QMeshPatch::~QMeshPatch()
{
	ClearAll();
}

//////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////

void QMeshPatch::ClearAll()
{
	GLKPOSITION Pos;

	for(Pos=faceList.GetHeadPosition();Pos!=NULL;) {
		QMeshFace* face=(QMeshFace*)(faceList.GetNext(Pos));
		delete face;
	}
	faceList.RemoveAll();

	for(Pos=edgeList.GetHeadPosition();Pos!=NULL;) {
		QMeshEdge* edge=(QMeshEdge*)(edgeList.GetNext(Pos));
		delete edge;
	}
	edgeList.RemoveAll();

	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
		QMeshNode* node=(QMeshNode*)(nodeList.GetNext(Pos));
		delete node;
	}
	nodeList.RemoveAll();
}

void QMeshPatch::InverseOrientation()
{
	GLKPOSITION Pos;
	QMeshEdge *edgeArray[MAX_EDGE_NUM];		bool edgeDir[MAX_EDGE_NUM];

	for(Pos=faceList.GetHeadPosition();Pos!=NULL;) {
		QMeshFace *face=(QMeshFace *)(faceList.GetNext(Pos));
		int i,eNum=face->GetEdgeNum();
		for(i=0;i<eNum;i++) {
			edgeArray[eNum-1-i]=face->GetEdgeRecordPtr(i);
			edgeDir[eNum-1-i]=face->IsNormalDirection(i);
		}
		for(i=0;i<eNum;i++) {
			face->SetEdgeRecordPtr(i,edgeArray[i]);
			face->SetDirectionFlag(i,!(edgeDir[i]));
		}
		face->CalPlaneEquation();
	}
	//------------------------------------------------------------------------------
	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
		QMeshNode *node=(QMeshNode *)(nodeList.GetNext(Pos));
		node->GetEdgeList().RemoveAll();
	}
	//------------------------------------------------------------------------------
	for(Pos=edgeList.GetHeadPosition();Pos!=NULL;) {
		QMeshEdge *edge=(QMeshEdge *)(edgeList.GetNext(Pos));
		edge->SetLeftFace(NULL);	edge->SetRightFace(NULL);
		edge->GetStartPoint()->AddEdge(edge);	edge->GetEndPoint()->AddEdge(edge);
	}
	//------------------------------------------------------------------------------
	for(Pos=faceList.GetHeadPosition();Pos!=NULL;) {
		QMeshFace *face=(QMeshFace *)(faceList.GetNext(Pos));
		int i,eNum=face->GetEdgeNum();
		for(i=0;i<eNum;i++) {
			if (face->IsNormalDirection(i))
				face->GetEdgeRecordPtr(i)->SetLeftFace(face);
			else
				face->GetEdgeRecordPtr(i)->SetRightFace(face);
		}
	}
}

bool QMeshPatch::inputOFFFile(char* filename, bool bOBTFile)
{
	FILE* fp;
	char buf[100];
	GLKPOSITION Pos;
	GLKPOSITION PosNode;
	QMeshNode* node, * startNode, * endNode;
	QMeshEdge* edge;
	QMeshFace* face;
	QMeshNode** nodeArray;
	float xx, yy, zz;
	int faceNum, nodeNum, i;

	fp = fopen(filename, "r");
	if (!fp) {
		printf("===============================================\n");
		printf("Can not open the data file - OFF File Import!\n");
		printf("===============================================\n");
		return false;
	}
	ClearAll();

	fscanf(fp, "%s\n", buf);
	fscanf(fp, "%d %d %d\n", &nodeNum, &faceNum, &i);

	for (i = 0; i < nodeNum; ++i) {
		fscanf(fp, "%f %f %f\n", &xx, &yy, &zz);
		node = new QMeshNode;
		node->SetMeshPatchPtr(this);
		node->SetCoord3D(xx, yy, zz);
		node->SetCoord3D_last(xx, yy, zz);
		node->SetIndexNo(nodeList.GetCount() + 1);
		nodeList.AddTail(node);
	}

	nodeArray = new QMeshNode * [nodeNum];
	i = 0;
	for (Pos = nodeList.GetHeadPosition(); Pos != NULL; ++i) {
		node = (QMeshNode*)(nodeList.GetNext(Pos));
		nodeArray[i] = node;
	}

	for (i = 0; i < faceNum; ++i) {
		int num, nodeIndex, tmp[3];
		fscanf(fp, "%d %d %d %d\n", &num, &tmp[0], &tmp[1], &tmp[2]);
		if (num == 1)
			break;

		if (num > 2) {
			face = new QMeshFace;
			face->SetMeshPatchPtr(this);
			face->SetIndexNo(faceList.GetCount() + 1);
			faceList.AddTail(face);

			for (int j = 0; j < num; ++j) {
				(face->GetAttachedList()).AddTail(nodeArray[tmp[j]]);
			}
		}
	}

	fclose(fp);

	delete[]nodeArray;

	//---------------------------------------------------------------------
	//	Build the topology
	//---------------------------------------------------------------------
	//	Step 1: build the edges
	for (Pos = faceList.GetHeadPosition(); Pos != NULL;) {
		face = (QMeshFace*)(faceList.GetNext(Pos));

		int edgeNum = (face->GetAttachedList()).GetCount();
		face->SetEdgeNum(edgeNum);

		//nodeArray=(QMeshNode**)new long[edgeNum];
		nodeArray = new QMeshNode * [edgeNum];

		i = 0;
		for (PosNode = (face->GetAttachedList()).GetHeadPosition(); PosNode != NULL; i++) {
			nodeArray[i] = (QMeshNode*)((face->GetAttachedList()).GetNext(PosNode));
			(nodeArray[i]->GetFaceList()).AddTail(face);
		}

		for (i = 0; i < edgeNum; i++) {
			edge = NULL;	startNode = nodeArray[i];	endNode = nodeArray[(i + 1) % edgeNum];
			bool bDir;
			for (PosNode = (startNode->GetEdgeList()).GetHeadPosition(); PosNode != NULL;) {
				QMeshEdge* temp = (QMeshEdge*)((startNode->GetEdgeList()).GetNext(PosNode));
				if ((temp->GetStartPoint() == startNode) && (temp->GetEndPoint() == endNode) && (temp->GetLeftFace() == NULL)) {
					edge = temp;	bDir = true;
				}
				else if ((temp->GetStartPoint() == endNode) && (temp->GetEndPoint() == startNode) && (temp->GetRightFace() == NULL)) {
					edge = temp;	bDir = false;
				}
			}
			if (edge && bDir) {
				face->SetEdgeRecordPtr(i, edge);
				face->SetDirectionFlag(i, true);
				edge->SetLeftFace(face);
			}
			else if (edge && (!bDir)) {
				face->SetEdgeRecordPtr(i, edge);
				face->SetDirectionFlag(i, false);
				edge->SetRightFace(face);
			}
			else {
				edge = new QMeshEdge;
				edge->SetMeshPatchPtr(this);
				edge->SetStartPoint(startNode);
				edge->SetEndPoint(endNode);
				edge->SetIndexNo(edgeList.GetCount() + 1);
				edgeList.AddTail(edge);

				edge->SetLeftFace(face);
				face->SetEdgeRecordPtr(i, edge);
				face->SetDirectionFlag(i, true);
				(startNode->GetEdgeList()).AddTail(edge);
				(endNode->GetEdgeList()).AddTail(edge);
			}
		}

		delete[]nodeArray;
		face->GetAttachedList().RemoveAll();
	}

	//---------------------------------------------------------------------
	//	Step 2: compute the normalf
	for (Pos = faceList.GetHeadPosition(); Pos != NULL;) {
		face = (QMeshFace*)(faceList.GetNext(Pos));
		face->CalPlaneEquation();
		double xx, yy, zz;
		face->CalCenterPos(xx, yy, zz);
		face->selected = false;
		face->m_nIdentifiedPatchIndex = -1;
	}
	for (Pos = edgeList.GetHeadPosition(); Pos != NULL;) {
		edge = (QMeshEdge*)(edgeList.GetNext(Pos));
		edge->selected = false;
		edge->CalLength();
		edge->Cal2DLength();
		if ((edge->GetLeftFace()) && (edge->GetRightFace())) continue;
		edge->SetAttribFlag(0);
		edge->GetStartPoint()->SetAttribFlag(0);
		edge->GetEndPoint()->SetAttribFlag(0);
	}
	//std::cout << "Finish input .off file" << std::endl;
	return true;
}


bool QMeshPatch::inputMFile(char* filename)
{
	FILE *fp;
	char linebuf[256],buf[100];
	GLKPOSITION Pos;
	GLKPOSITION PosNode;	
	int i,index,index1,index2,index3;
	QMeshNode *node,*startNode,*endNode;
	QMeshEdge *edge;
	QMeshFace *face;
	QMeshNode **nodeArray;
	float xx,yy,zz;
//	float minX,maxX,minY,maxY,minZ,maxZ;

	fp = fopen(filename, "r");
    if(!fp) {
	    printf("===============================================\n");
	    printf("Can not open the data file - OBJ File Import!\n");
	    printf("===============================================\n");
	    return false;
	}

	ClearAll();
	while(!feof(fp)) {
		sprintf(buf,"");
		sprintf(linebuf,"");
		fgets(linebuf, 255, fp);
		sscanf(linebuf,"%s",buf);
	
        if (strcmp(buf,"Vertex")==0 )
		{
			sscanf(linebuf, "%s %d %f %f %f \n", buf, &index, &xx, &yy, &zz);
//			xx=xx*100.0f;	yy=yy*100.0f;	zz=zz*100.0f;

			node=new QMeshNode;
			node->SetMeshPatchPtr(this);
			node->SetCoord3D(xx,yy,zz);
			node->SetIndexNo(nodeList.GetCount()+1);
			nodeList.AddTail(node);
		}
	}
	fclose(fp);

	int nodeNum=nodeList.GetCount();
	nodeArray=(QMeshNode**)new long[nodeNum];
	i=0;
	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;i++) {
		node=(QMeshNode*)(nodeList.GetNext(Pos));
		nodeArray[i]=node;
	}

	fp = fopen(filename, "r");
	while(!feof(fp)) {
		sprintf(buf,"");
		sprintf(linebuf,"");
		fgets(linebuf, 255, fp);
		sscanf(linebuf,"%s",buf);
		
        if ( strcmp(buf,"Face")==0 )
		{
			sscanf(linebuf, "%s %d %d %d %d \n", buf, &index, &index1, &index2, &index3);

			face=new QMeshFace;
			face->SetMeshPatchPtr(this);
			face->SetIndexNo(faceList.GetCount()+1);
			faceList.AddTail(face);
			
			(face->GetAttachedList()).AddTail(nodeArray[index1-1]);
			(face->GetAttachedList()).AddTail(nodeArray[index2-1]);
			(face->GetAttachedList()).AddTail(nodeArray[index3-1]);
		}
	}
	fclose(fp);

	delete [](QMeshNode**)nodeArray;

	//---------------------------------------------------------------------
	//	Build the topology
	//---------------------------------------------------------------------
	//	Step 1: build the edges
	for(Pos=faceList.GetHeadPosition();Pos!=NULL;) {
		face=(QMeshFace*)(faceList.GetNext(Pos));

		int edgeNum=(face->GetAttachedList()).GetCount();
		face->SetEdgeNum(edgeNum);

		nodeArray=(QMeshNode**)new long[edgeNum];
		i=0;
		for(PosNode=(face->GetAttachedList()).GetHeadPosition();PosNode!=NULL;i++) {
			nodeArray[i]=(QMeshNode*)((face->GetAttachedList()).GetNext(PosNode));
			(nodeArray[i]->GetFaceList()).AddTail(face);
		}
		for(i=0;i<edgeNum;i++) {
			edge=NULL;	startNode=nodeArray[i];	endNode=nodeArray[(i+1)%edgeNum];
			bool bDir;
			for(PosNode=(startNode->GetEdgeList()).GetHeadPosition();PosNode!=NULL;) {
				QMeshEdge *temp=(QMeshEdge *)((startNode->GetEdgeList()).GetNext(PosNode));
				if ((temp->GetStartPoint()==startNode) && (temp->GetEndPoint()==endNode)) {
					edge=temp;	bDir=true;
				}
				else if ((temp->GetStartPoint()==endNode) && (temp->GetEndPoint()==startNode)) {
					edge=temp;	bDir=false;
				}
			}
			if (edge) {
				face->SetEdgeRecordPtr(i,edge);
				if (bDir) {
					face->SetDirectionFlag(i,true);
					edge->SetLeftFace(face);
				}
				else {
					face->SetDirectionFlag(i,false);
					edge->SetRightFace(face);
				}
			}
			else {
				edge=new QMeshEdge;
				edge->SetMeshPatchPtr(this);
				edge->SetStartPoint(startNode);
				edge->SetEndPoint(endNode);
				edge->SetIndexNo(edgeList.GetCount()+1);
				edgeList.AddTail(edge);

				edge->SetLeftFace(face);
				face->SetEdgeRecordPtr(i,edge);
				face->SetDirectionFlag(i,true);
				(startNode->GetEdgeList()).AddTail(edge);
				(endNode->GetEdgeList()).AddTail(edge);
			}
		}

		delete [](QMeshNode**)nodeArray;

		face->GetAttachedList().RemoveAll();
	}
	//---------------------------------------------------------------------
	//	Step 2: compute the normal
	for(Pos=faceList.GetHeadPosition();Pos!=NULL;) {
		face=(QMeshFace*)(faceList.GetNext(Pos));
		face->CalPlaneEquation();
	}

	return true;
}

bool QMeshPatch::inputPLY2File(char* filename)
{
	FILE *fp;
	GLKPOSITION Pos;
	GLKPOSITION PosNode;	
	int i,j,nodeNum,faceNum,index,edgeNum;
	QMeshNode *node,*startNode,*endNode;
	QMeshEdge *edge;
	QMeshFace *face;
	QMeshNode **nodeArray;
	float xx,yy,zz;
//	float minX,maxX,minY,maxY,minZ,maxZ;

	fp = fopen(filename, "r");
    if(!fp) {
	    printf("===============================================\n");
	    printf("Can not open the data file - PLY2 File Import!\n");
	    printf("===============================================\n");
	    return false;
	}

	ClearAll();
	fscanf(fp,"%d\n",&nodeNum);
	fscanf(fp,"%d\n",&faceNum);

	nodeArray=(QMeshNode**)new long[nodeNum];
	for(i=0;i<nodeNum;i++) {
		fscanf(fp,"%f %f %f \n", &xx, &yy, &zz);

		node=new QMeshNode;
		node->SetMeshPatchPtr(this);
		node->SetCoord3D(xx,yy,zz);
		node->SetIndexNo(nodeList.GetCount()+1);
		nodeList.AddTail(node);
		nodeArray[i]=node;
	}

	for(i=0;i<faceNum;i++) {
		fscanf(fp,"%d ",&edgeNum);

		face=new QMeshFace;
		face->SetMeshPatchPtr(this);
		face->SetIndexNo(faceList.GetCount()+1);
		faceList.AddTail(face);

		for(j=0;j<edgeNum;j++) {
			fscanf(fp,"%d ",&index);
			(face->GetAttachedList()).AddTail(nodeArray[index]);
		}
	}

	fclose(fp);

	delete [](QMeshNode**)nodeArray;

	//---------------------------------------------------------------------
	//	Build the topology
	//---------------------------------------------------------------------
	//	Step 1: build the edges
	for(Pos=faceList.GetHeadPosition();Pos!=NULL;) {
		face=(QMeshFace*)(faceList.GetNext(Pos));

		int edgeNum=(face->GetAttachedList()).GetCount();
		face->SetEdgeNum(edgeNum);

		nodeArray=(QMeshNode**)new long[edgeNum];
		i=0;
		for(PosNode=(face->GetAttachedList()).GetHeadPosition();PosNode!=NULL;i++) {
			nodeArray[i]=(QMeshNode*)((face->GetAttachedList()).GetNext(PosNode));
			(nodeArray[i]->GetFaceList()).AddTail(face);
		}
		for(i=0;i<edgeNum;i++) {
			edge=NULL;	startNode=nodeArray[i];	endNode=nodeArray[(i+1)%edgeNum];
			bool bDir;
			for(PosNode=(startNode->GetEdgeList()).GetHeadPosition();PosNode!=NULL;) {
				QMeshEdge *temp=(QMeshEdge *)((startNode->GetEdgeList()).GetNext(PosNode));
				if ((temp->GetStartPoint()==startNode) && (temp->GetEndPoint()==endNode)) {
					edge=temp;	bDir=true;
				}
				else if ((temp->GetStartPoint()==endNode) && (temp->GetEndPoint()==startNode)) {
					edge=temp;	bDir=false;
				}
			}
			if (edge) {
				face->SetEdgeRecordPtr(i,edge);
				if (bDir) {
					face->SetDirectionFlag(i,true);
					edge->SetLeftFace(face);
				}
				else {
					face->SetDirectionFlag(i,false);
					edge->SetRightFace(face);
				}
			}
			else {
				edge=new QMeshEdge;
				edge->SetMeshPatchPtr(this);
				edge->SetStartPoint(startNode);
				edge->SetEndPoint(endNode);
				edge->SetIndexNo(edgeList.GetCount()+1);
				edgeList.AddTail(edge);

				edge->SetLeftFace(face);
				face->SetEdgeRecordPtr(i,edge);
				face->SetDirectionFlag(i,true);
				(startNode->GetEdgeList()).AddTail(edge);
				(endNode->GetEdgeList()).AddTail(edge);
			}
		}

		delete [](QMeshNode**)nodeArray;

		face->GetAttachedList().RemoveAll();
	}
	//---------------------------------------------------------------------
	//	Step 2: compute the normal
	for(Pos=faceList.GetHeadPosition();Pos!=NULL;) {
		face=(QMeshFace*)(faceList.GetNext(Pos));
		face->CalPlaneEquation();
	}
	for(Pos=edgeList.GetHeadPosition();Pos!=NULL;) {
		edge=(QMeshEdge*)(edgeList.GetNext(Pos));
		if ((edge->GetLeftFace()) && (edge->GetRightFace())) {
			edge->SetAttribFlag(0,false);
		}
		else {
			edge->SetAttribFlag(0,true);
			edge->GetStartPoint()->SetAttribFlag(0,true);
			edge->GetEndPoint()->SetAttribFlag(0,true);
		}
	}

	return true;
}

void QMeshPatch::constructionFromVerFaceTable(int nodeNum, float *nodeTable, int faceNum, unsigned int* faceTable)
{
	QMeshNode *node,*startNode,*endNode;
	QMeshEdge *edge;
	QMeshFace *face;
	QMeshNode **nodeArray;
	GLKPOSITION Pos;
	GLKPOSITION PosNode;
	int i;

	//nodeArray=(QMeshNode**)new long[nodeNum];
	nodeArray = new QMeshNode*[nodeNum];

	for(i=0;i<nodeNum;i++) {
		node=new QMeshNode;		nodeArray[i]=node;
		node->SetMeshPatchPtr(this);
		node->SetCoord3D(nodeTable[i*3],nodeTable[i*3+1],nodeTable[i*3+2]);
		node->SetCoord3D_last(nodeTable[i*3],nodeTable[i*3+1],nodeTable[i*3+2]);
		node->SetIndexNo(nodeList.GetCount()+1);
//		node->SetAttribFlag(4);
		nodeList.AddTail(node);
	}
//	delete [](QMeshNode**)nodeArray;	return;
	//--------------------------------------------------------------------------------------------------------
	for(i=0;i<faceNum;i++) {
		face=new QMeshFace;
		face->SetMeshPatchPtr(this);
		face->SetIndexNo(faceList.GetCount()+1);
		faceList.AddTail(face);

		(face->GetAttachedList()).AddTail(nodeArray[faceTable[i * 3 + 0]]);	//printf("%d ",faceTable[i*4+0]-1);
		(face->GetAttachedList()).AddTail(nodeArray[faceTable[i * 3 + 1]]);	//printf("%d ",faceTable[i*4+0]-1);
		(face->GetAttachedList()).AddTail(nodeArray[faceTable[i * 3 + 2]]);	//printf("%d ",faceTable[i*4+0]-1);

		//(face->GetAttachedList()).AddTail(nodeArray[faceTable[i*4+0]-1]);	//printf("%d ",faceTable[i*4+0]-1);
		//(face->GetAttachedList()).AddTail(nodeArray[faceTable[i*4+1]-1]);	//printf("%d ",faceTable[i*4+1]-1);
		//(face->GetAttachedList()).AddTail(nodeArray[faceTable[i*4+2]-1]);	//printf("%d ",faceTable[i*4+2]-1);
		//if (faceTable[i*4+3]==0) continue;
		//(face->GetAttachedList()).AddTail(nodeArray[faceTable[i*4+3]-1]);	//printf("%d ",faceTable[i*4+3]-1);
	}
	//delete [](QMeshNode**)nodeArray;	
	delete []nodeArray;

	//--------------------------------------------------------------------------------------------------------
	//	Build the topology
	//--------------------------------------------------------------------------------------------------------
	//	Step 1: build the edges
	int faceIndex=0;
	for(Pos=faceList.GetHeadPosition();Pos!=NULL;faceIndex++) {
		face=(QMeshFace*)(faceList.GetNext(Pos));

		int edgeNum=(face->GetAttachedList()).GetCount();
		face->SetEdgeNum(edgeNum);

		//nodeArray=(QMeshNode**)new long[edgeNum];
		nodeArray = new QMeshNode*[edgeNum];

		i=0;
		for(PosNode=(face->GetAttachedList()).GetHeadPosition();PosNode!=NULL;i++) {
			nodeArray[i]=(QMeshNode*)((face->GetAttachedList()).GetNext(PosNode));
			(nodeArray[i]->GetFaceList()).AddTail(face);
		}

		for(i=0;i<edgeNum;i++) {
			edge=NULL;	startNode=nodeArray[i];	endNode=nodeArray[(i+1)%edgeNum];
			bool bDir;
			for(PosNode=(startNode->GetEdgeList()).GetHeadPosition();PosNode!=NULL;) {
				QMeshEdge *temp=(QMeshEdge *)((startNode->GetEdgeList()).GetNext(PosNode));
				if ((temp->GetStartPoint()==startNode) && (temp->GetEndPoint()==endNode) && (temp->GetLeftFace()==NULL)) {
					edge=temp;	bDir=true;
				}
				else if ((temp->GetStartPoint()==endNode) && (temp->GetEndPoint()==startNode) && (temp->GetRightFace()==NULL)) {
					edge=temp;	bDir=false;
				}
			}
			if (edge && bDir) {
				face->SetEdgeRecordPtr(i,edge);
				face->SetDirectionFlag(i,true);
				edge->SetLeftFace(face);
			}
			else if (edge && (!bDir)) {
				face->SetEdgeRecordPtr(i,edge);
				face->SetDirectionFlag(i,false);
				edge->SetRightFace(face);
			}
			else {
				edge=new QMeshEdge;
				edge->SetMeshPatchPtr(this);
				edge->SetStartPoint(startNode);
				edge->SetEndPoint(endNode);
				edge->SetIndexNo(edgeList.GetCount()+1);
				edgeList.AddTail(edge);

				edge->SetLeftFace(face);
				face->SetEdgeRecordPtr(i,edge);
				face->SetDirectionFlag(i,true);
				(startNode->GetEdgeList()).AddTail(edge);
				(endNode->GetEdgeList()).AddTail(edge);
			}
		}

		//delete [](QMeshNode**)nodeArray;
		delete []nodeArray;

		face->GetAttachedList().RemoveAll();
	}
	//---------------------------------------------------------------------
	//	Step 2: compute the normal
	for(Pos=faceList.GetHeadPosition();Pos!=NULL;) {
		face=(QMeshFace*)(faceList.GetNext(Pos));
		face->CalPlaneEquation();
	}
	for(Pos=edgeList.GetHeadPosition();Pos!=NULL;) {
		edge=(QMeshEdge*)(edgeList.GetNext(Pos));
		if ((edge->GetLeftFace()) && (edge->GetRightFace())) continue;
		edge->SetAttribFlag(0);
		edge->GetStartPoint()->SetAttribFlag(0);
		edge->GetEndPoint()->SetAttribFlag(0);
	}
}

bool QMeshPatch::inputOBJFile(char* filename, bool bOBTFile)
{
    FILE *fp;
    char fields[MAX_EDGE_NUM][255];
    char linebuf[256],buf[100];
    GLKPOSITION Pos;
    GLKPOSITION PosNode;
    int i;
    QMeshNode *node,*startNode,*endNode;
    QMeshEdge *edge;
    QMeshFace *face;
    QMeshNode **nodeArray;
    float xx,yy,zz,ww;
//	float minX,maxX,minY,maxY,minZ,maxZ;

    fp = fopen(filename, "r");
    if(!fp) {
        printf("===============================================\n");
        printf("Can not open the data file - OBJ File Import!\n");
        printf("===============================================\n");
        return false;
    }

    ClearAll();
    while(!feof(fp)) {
        sprintf(buf,"");
        sprintf(linebuf,"");
        fgets(linebuf, 255, fp);
        sscanf(linebuf,"%s",buf);

        if ( (strlen(buf)==1) && (buf[0]=='v') )
        {
            float rr, gg, bb;
            rr=1.0; gg=1.0; bb=1.0;
            if (bOBTFile)
                sscanf(linebuf, "%s %f %f %f %f\n", buf, &xx, &yy, &zz, &ww);
            else
                sscanf(linebuf, "%s %f %f %f %f %f %f\n", buf, &xx, &yy, &zz, &rr, &gg, &bb);

            node=new QMeshNode;
            node->SetMeshPatchPtr(this);

            node->SetCoord3D(xx,yy,zz);
            node->SetCoord3D_last(xx,yy,zz);
            node->SetIndexNo(nodeList.GetCount()+1); 
            node->identifiedIndex = node->GetIndexNo();
            node->m_nIdentifiedPatchIndex = -1;
            node->selected = false;
            if (bOBTFile)
                node->SetWeight(ww);
            else{
                node->SetWeight(-1.0);
                node->SetColor(rr,gg,bb);
            }
            nodeList.AddTail(node);
        }
    }
    fclose(fp);


    int nodeNum=nodeList.GetCount();
	nodeArray = new QMeshNode*[nodeNum];
	i=0;
    for(Pos=nodeList.GetHeadPosition();Pos!=NULL;i++) {
        node=(QMeshNode*)(nodeList.GetNext(Pos));
        nodeArray[i]=node;
    }

    fp = fopen(filename, "r");
    while(!feof(fp)) {
        sprintf(buf,"");
        sprintf(linebuf,"");
        fgets(linebuf, 255, fp);
        sscanf(linebuf,"%s",buf);

        if ( (strlen(buf)==1) && (buf[0]=='f') )
        {
            char seps[]=" \r\n";
            char seps2[]="/";
            char *token;
            char linebuf2[255];
            strcpy(linebuf2,linebuf);

            int num=0;
            token = strtok(linebuf,seps);
            while(nullptr != token){
                token=strtok(nullptr,seps);
                num++;
            }
            num=num-1;

            if (num>MAX_EDGE_NUM) continue;
            if (num<1) continue;

            face=new QMeshFace;
            face->SetMeshPatchPtr(this);
            face->SetIndexNo(faceList.GetCount()+1);
            faceList.AddTail(face);
			face->inner = false; // it should be false for all surface mesh
								 // it is used for tet mesh

            token = strtok( linebuf2, seps );
            for(i=0;i<num;i++) {
                token = strtok( NULL, seps );
                strcpy(fields[i],token);
            }

            bool bValid=true;
            for(i=0;i<num;i++) {
                token = strtok( fields[i], seps2 );
                int nodeIndex=atoi(token);

//				double xc,yc,zc;
//				nodeArray[nodeIndex-1]->GetCoord3D(xc,yc,zc);
//				if (xc<0.0) bValid=false;

                (face->GetAttachedList()).AddTail(nodeArray[nodeIndex-1]);
//				(face->GetAttachedList()).AddHead(nodeArray[nodeIndex-1]);
            }
            if (!bValid) {delete face; faceList.RemoveTail(); continue;}

            bool bDegenerated=false;
            for(Pos=face->GetAttachedList().GetHeadPosition();Pos!=NULL;) {
                QMeshNode *pNode=(QMeshNode *)(face->GetAttachedList().GetNext(Pos));
                GLKPOSITION Pos2=Pos;
                for(;Pos2!=NULL;) {
                    QMeshNode *qNode=(QMeshNode *)(face->GetAttachedList().GetNext(Pos2));
                    if ((pNode==qNode)) {
                        bDegenerated=true;
                        break;
                    }
                }
                if (bDegenerated) break;
            }
            if (bDegenerated) {
                faceList.RemoveTail();
                delete face;
            }
        }
    }
    fclose(fp);

    delete []nodeArray;

    //---------------------------------------------------------------------
    //	Build the topology
    //---------------------------------------------------------------------
    //	Step 1: build the edges
    for(Pos=faceList.GetHeadPosition();Pos!=NULL;) {
        face=(QMeshFace*)(faceList.GetNext(Pos));

        int edgeNum=(face->GetAttachedList()).GetCount();
        face->SetEdgeNum(edgeNum);

        //nodeArray=(QMeshNode**)new long[edgeNum];
		nodeArray = new QMeshNode*[edgeNum];

        i=0;
        for(PosNode=(face->GetAttachedList()).GetHeadPosition();PosNode!=NULL;i++) {
            nodeArray[i]=(QMeshNode*)((face->GetAttachedList()).GetNext(PosNode));
            (nodeArray[i]->GetFaceList()).AddTail(face);
        }

        for(i=0;i<edgeNum;i++) {
            edge=NULL;	startNode=nodeArray[i];	endNode=nodeArray[(i+1)%edgeNum];
            bool bDir;
            for(PosNode=(startNode->GetEdgeList()).GetHeadPosition();PosNode!=NULL;) {
                QMeshEdge *temp=(QMeshEdge *)((startNode->GetEdgeList()).GetNext(PosNode));
                if ((temp->GetStartPoint()==startNode) && (temp->GetEndPoint()==endNode) && (temp->GetLeftFace()==NULL)) {
                    edge=temp;	bDir=true;
                }
                else if ((temp->GetStartPoint()==endNode) && (temp->GetEndPoint()==startNode) && (temp->GetRightFace()==NULL)) {
                    edge=temp;	bDir=false;
                }
            }
            if (edge && bDir) {
                face->SetEdgeRecordPtr(i,edge);
                face->SetDirectionFlag(i,true);
                edge->SetLeftFace(face);
            }
            else if (edge && (!bDir)) {
                face->SetEdgeRecordPtr(i,edge);
                face->SetDirectionFlag(i,false);
                edge->SetRightFace(face);
            }
            else {
                edge=new QMeshEdge;
                edge->SetMeshPatchPtr(this);
                edge->SetStartPoint(startNode);
                edge->SetEndPoint(endNode);
                edge->SetIndexNo(edgeList.GetCount()+1);
                edgeList.AddTail(edge);

                edge->SetLeftFace(face);
                face->SetEdgeRecordPtr(i,edge);
                face->SetDirectionFlag(i,true);
                (startNode->GetEdgeList()).AddTail(edge);
                (endNode->GetEdgeList()).AddTail(edge);
            }
        }

        delete []nodeArray;
        face->GetAttachedList().RemoveAll();
    }
    //---------------------------------------------------------------------
    //	Step 2: compute the normalf
    for(Pos=faceList.GetHeadPosition();Pos!=NULL;) {
        face=(QMeshFace*)(faceList.GetNext(Pos));
        face->CalPlaneEquation();
        double xx,yy,zz;
        face->CalCenterPos(xx,yy,zz);
        face->selected = false;
        face->m_nIdentifiedPatchIndex = -1;
    }
    for(Pos=edgeList.GetHeadPosition();Pos!=NULL;) {
        edge=(QMeshEdge*)(edgeList.GetNext(Pos));
		edge->selected = false;
        edge->CalLength();
        edge->Cal2DLength();
        if ((edge->GetLeftFace()) && (edge->GetRightFace())) continue;
        edge->SetAttribFlag(0);
        edge->GetStartPoint()->SetAttribFlag(0);
        edge->GetEndPoint()->SetAttribFlag(0);
    }
	std::cout << "Finish input obj" << std::endl;
    return true;
}

bool QMeshPatch::inputTETFile(char *filename, bool bOBTFile)
{
	FILE *fp;
	char linebuf[256], buf[100];
	int nodeNum, i;	float xx, yy, zz;
	int tetraNum;
	GLKPOSITION Pos;
	GLKPOSITION PosNode;
	QMeshNode *node, *startNode, *endNode;
	QMeshEdge *edge;
	QMeshFace *face;
	QMeshNode **nodeArray;

	fp = fopen(filename, "r");
	if (!fp) {
		printf("===============================================\n");
		printf("Can not open the data file - Tetra File Import!\n");
		printf("===============================================\n");
		return false;
	}
	ClearAll();

	printf("reading tet file.........");
	isVolume = true;

	sprintf(buf, ""); sprintf(linebuf, "");
	fgets(linebuf, 255, fp);
	sscanf(linebuf, "%d %s \n", &nodeNum, buf);
	if (strcmp(buf, "vertices") != 0) {
		std::cout << "Incorrect Tet format! Missing vertices number." << std::endl;
		fclose(fp);
		return false;
	}
	printf("%d vertices, ", nodeNum);
	sprintf(buf, ""); sprintf(linebuf, "");
	fgets(linebuf, 255, fp);
	sscanf(linebuf, "%d %s \n", &tetraNum, buf);
	if (strcmp(buf, "tets") != 0) {
		std::cout << "Incorrect Tet format! Missing Tetra number." << std::endl;
		fclose(fp);
		return false;
	}
	printf("%d tets\n", tetraNum);

	nodeArray = new QMeshNode*[nodeNum];

	//(1) ----------- Build Node List
	for (i = 0; i<nodeNum; i++) {
		sprintf(buf, ""); sprintf(linebuf, "");
		fgets(linebuf, 255, fp);
		sscanf(linebuf, "%f %f %f \n", &xx, &yy, &zz);
		node = new QMeshNode;
		node->SetMeshPatchPtr(this);
		node->SetCoord3D(xx, yy, zz);
		node->SetCoord3D_last(xx, yy, zz);
		node->SetIndexNo(i + 1);
		nodeList.AddTail(node);
		nodeArray[i] = node;
	}

	for (i = 0; i<tetraNum; i++) {
		sprintf(buf, ""); sprintf(linebuf, "");
		fgets(linebuf, 255, fp);
		int tet, n[4];
		sscanf(linebuf, "%d %d %d %d %d \n", &tet, &n[0], &n[1], &n[2], &n[3]);
		if (tet != 4) { std::cout<< "not a tetra?" << std::endl; fclose(fp); return false; }
		//recall the notation for face 1,2,3,4 - encoding for node index on each face
		int n_index[4][3] = {
			{ n[0], n[1], n[2] },
			{ n[1], n[3], n[2] },
			{ n[2], n[3], n[0] },
			{ n[3], n[1], n[0] }
		};

		//(2) ----------- Build Node List

		QMeshTetra *Tetra = new QMeshTetra;
		for(int j=0;j<4;j++){
			/*QMeshNode *Node = GetNodeRecordPtr(n[j]+1);
			(Tetra->GetNodeList()).AddTail(Node);*/
			Tetra->tet_node_index[j] = n[j];
		}
		Tetra->SetIndexNo(i + 1);
		Tetra->SetMeshSurfacePtr(this);
		tetraList.AddTail(Tetra);

		for (int j = 0; j<4; j++) {
			nodeArray[n[j]]->AddTetra(Tetra); //add this tetra to the node tetralist.
		}

		//for (int j=0; j<4; j++)
		//	Tetra->nodeindex[j] = n[j];

		QMeshFace *f[4];
		for (int j = 1; j <= 4; j++) {
			bool existed = false;
			for (GLKPOSITION Pos = nodeArray[n[j - 1]]->GetFaceList().GetHeadPosition(); Pos != NULL;) {
				QMeshFace *tmp_face = (QMeshFace*)nodeArray[n[j - 1]]->GetFaceList().GetNext(Pos);
				QMeshNode *tmp_node[3];
				int k = 0;
				for (GLKPOSITION Pos1 = tmp_face->GetAttachedList().GetHeadPosition(); Pos1 != NULL; k++)
					tmp_node[k] = (QMeshNode*)tmp_face->GetAttachedList().GetNext(Pos1);
				bool same[3] = { 0, 0, 0 };
				for (k = 0; k<3; k++) {
					for (int m = 0; m<3; m++) {
						if (tmp_node[k]->GetIndexNo() - 1 == n_index[j - 1][m]) {
							same[k] = true;
							break;
						}
					}
					if (!same[k]) break;
				}
				if (same[0] && same[1] && same[2]) {
					f[j - 1] = tmp_face;
					if (f[j - 1]->GetRightTetra() != NULL) printf("ERROR: f[%d-1]->GetRightTetra()!=NULL\n", j);
					f[j - 1]->SetRightTetra(Tetra);
					Tetra->SetDirectionFlag(j, false);
					existed = true; break;
				}
			}
			if (!existed) {
				f[j - 1] = new QMeshFace();
				f[j - 1]->SetMeshPatchPtr(this);
				f[j - 1]->SetIndexNo(faceList.GetCount() + 1);
				faceList.AddTail(f[j - 1]);
				for (int k = 0; k<3; k++) {
					f[j - 1]->GetAttachedList().AddTail(nodeArray[n_index[j - 1][k]]);
					nodeArray[n_index[j - 1][k]]->GetFaceList().AddTail(f[j - 1]);
				}
				if (f[j - 1]->GetLeftTetra() != NULL) printf("ERROR: f[%d-1]->GetLeftTetra()!=NULL\n", j);
				f[j - 1]->SetLeftTetra(Tetra);
				Tetra->SetDirectionFlag(j);
			}
			Tetra->SetFaceRecordPtr(j, f[j - 1]);
		}
	}

	/*for (Pos = nodeList.GetHeadPosition(); Pos != NULL;) {
		node = (QMeshNode*)(nodeList.GetNext(Pos));
		std::cout << node->GetIndexNo() << std::endl;
		int facen = node->GetFaceNumber();
		for (i = 0; i < facen; i++) std::cout << node->GetFaceRecordPtr(i+1)->GetIndexNo() << std::endl;
	}*/

	delete []nodeArray;
	SetAttribFlag(1);

	////Add node into Tetra nodelist, this is useful for further get node from tetra.
	//for (i = 0; i < tetraNum; i++) {
	//	sprintf(buf, ""); sprintf(linebuf, "");
	//	fgets(linebuf, 255, fp);
	//	int tet, n[4];
	//	sscanf(linebuf, "%d %d %d %d %d \n", &tet, &n[0], &n[1], &n[2], &n[3]);
	//	QMeshTetra *Tetra = GetTetraRecordPtr(i+1);
	//	for (int j = 0; j < 4; j++) {
	//		QMeshNode *Node = GetNodeRecordPtr(j + 1);
	//		(Tetra->GetNodeList()).AddTail(Node);
	//	}
	//}

	//std::cout << GetFaceNumber() << std::endl;

	//---------------------------------------------------------------------
	//	Build the topology
	//---------------------------------------------------------------------
	//	Step 1: build the edges
	for (Pos = faceList.GetHeadPosition(); Pos != NULL;) {
		QMeshFace*face = (QMeshFace*)(faceList.GetNext(Pos));

		if (face->GetLeftTetra() && face->GetRightTetra()) { face->inner = true; }
		else face->inner = false;
		face->selected = false;

		int edgeNum = (face->attachedList).GetCount();
		face->SetEdgeNum(edgeNum);
		nodeArray = new QMeshNode*[nodeNum];
		i = 0;
		for (PosNode = (face->attachedList).GetHeadPosition(); PosNode != NULL; i++) {
			nodeArray[i] = (QMeshNode*)((face->attachedList).GetNext(PosNode));
			//(nodeArray[i]->GetTrglFaceList()).AddTail(face);
		}
		for (i = 0; i<edgeNum; i++) {
			edge = NULL;	startNode = nodeArray[i];	endNode = nodeArray[(i + 1) % edgeNum];
			bool bDir;
			for (PosNode = (startNode->GetEdgeList()).GetHeadPosition(); PosNode != NULL;) {
				QMeshEdge *temp = (QMeshEdge *)((startNode->GetEdgeList()).GetNext(PosNode));
				if ((temp->GetStartPoint() == startNode) && (temp->GetEndPoint() == endNode)) {
					edge = temp;	bDir = true;
				}
				else if ((temp->GetStartPoint() == endNode) && (temp->GetEndPoint() == startNode)) {
					edge = temp;	bDir = false;
				}
			}
			if (edge) {
				face->SetEdgeRecordPtr(i, edge);
				edge->GetFaceList().AddTail(face);
				if (bDir) {
					face->SetDirectionFlag(i, true);
					if (!face->inner) {
						if (edge->GetLeftFace()) {
							printf("edge->GetLeftFace()!=NULL\n");
							face->m_nIdentifiedPatchIndex = 2;
							if (!edge->GetRightFace()) printf("but right face is null!\n");
							//bool t1 = face->GetLeftTetra(), t2 = face->GetRightTetra();
							//if (t1) deleteTetra(face->GetLeftTetra());
							//if (t2) deleteTetra(face->GetRightTetra());
							continue;
						}
						else edge->SetLeftFace(face);
					}
				}
				else {
					face->SetDirectionFlag(i, false);
					if (!face->inner) {
						if (edge->GetRightFace()) {
							printf("edge->GetRightFace()!=NULL\n");
							face->m_nIdentifiedPatchIndex = 2;
							if (!edge->GetLeftFace()) printf("but left face is null!\n");
							//bool t1 = face->GetLeftTetra(), t2 = face->GetRightTetra();
							//if (t1) deleteTetra(face->GetLeftTetra());
							//if (t2) deleteTetra(face->GetRightTetra());
							continue;
						}
						else edge->SetRightFace(face);
					}
				}
			}
			else {
				edge = new QMeshEdge;
				edge->SetMeshPatchPtr(this);
				edge->SetStartPoint(startNode);
				edge->SetEndPoint(endNode);
				edge->SetIndexNo(edgeList.GetCount() + 1);
				edgeList.AddTail(edge);

				edge->GetFaceList().AddTail(face);
				if (!face->inner) edge->SetLeftFace(face);
				face->SetEdgeRecordPtr(i, edge);
				face->SetDirectionFlag(i, true);
				(startNode->GetEdgeList()).AddTail(edge);
				(endNode->GetEdgeList()).AddTail(edge);
			}
		}

		delete[]nodeArray;
		face->attachedList.RemoveAll();
	}
	//for (Pos=trglTetraList.GetHeadPosition(); Pos!=NULL;){
	//	QMeshTetra *tetra = (QMeshTetra*) trglTetraList.GetNext(Pos);
	//	for (int j=1; j<=4; j++){
	//		bool exist = false;
	//		for (int i=0; i<4; i++){
	//			if (tetra->nodeindex[i]==tetra->GetNodeRecordPtr(j)->GetIndexNo()-1)
	//				exist=true;
	//		}
	//		if (!exist) printf("Not exist: %d tetra, %d node\n", tetra->GetIndexNo(), tetra->GetNodeRecordPtr(j)->GetIndexNo());
	//	}
	//}

	//---------------------------------------------------------------------
	//	Fill in the flags
	for (Pos = edgeList.GetHeadPosition(); Pos != NULL;) {
		QMeshEdge *edge = (QMeshEdge *)(edgeList.GetNext(Pos));
		edge->inner = true;
		edge->selected = false;
		for (GLKPOSITION Pos1 = edge->GetFaceList().GetHeadPosition(); Pos1 != NULL;) {
			QMeshFace *face = (QMeshFace *)edge->GetFaceList().GetNext(Pos1);
			if (!face->inner) { edge->inner = false; break; }
		}
	}
	for (Pos = nodeList.GetHeadPosition(); Pos != NULL;) {
		QMeshNode *node = (QMeshNode *)(nodeList.GetNext(Pos));
		node->inner = true;
		node->selected = false;
		for (GLKPOSITION Pos1 = node->GetFaceList().GetHeadPosition(); Pos1 != NULL;) {
			QMeshFace *face = (QMeshFace *)node->GetFaceList().GetNext(Pos1);
			face->m_nIdentifiedPatchIndex = -1;
			if (!face->inner) { node->inner = false; break; }
		}
	}
	for (Pos = edgeList.GetHeadPosition(); Pos != NULL;) {
		QMeshEdge *edge = (QMeshEdge *)(edgeList.GetNext(Pos));
		if (edge->inner) continue;
		if (edge->GetLeftFace() && edge->GetRightFace()) continue;
		// if edge is outer or not have both left and right face, then:
		edge->SetAttribFlag(0);
		edge->GetStartPoint()->SetAttribFlag(0);
		edge->GetEndPoint()->SetAttribFlag(0);
	}

	//---------------------------------------------------------------------
	//	Step 2: compute the normal
	for (Pos = nodeList.GetHeadPosition(); Pos != NULL;) {
		node = (QMeshNode*)(nodeList.GetNext(Pos));

		////Verify
		//if (node->GetIndexNo() == 2) {
		//	int facen = node->GetFaceNumber();
		//	for (i = 0; i < facen; i++) std::cout << node->GetFaceRecordPtr(i + 1)->GetIndexNo() << std::endl;
		//}

		if (node->inner) continue;
		node->CalNormal();
	}

	for (Pos = faceList.GetHeadPosition(); Pos != NULL;) {
		face = (QMeshFace*)(faceList.GetNext(Pos));
		//if (face->inner) continue;
		face->CalPlaneEquation();
		face->CalCenterPos();
		face->i_inner = face->inner;
	}

	std::cout << "finish input tet file." << std::endl;
	return true;
}

void QMeshPatch::outputTrglOBJFile(char* filename)
{
	FILE *fp;
	GLKPOSITION Pos;
	QMeshNode *node;
	QMeshFace *face;
	double xx,yy,zz;
	int i,num,index;

	fp = fopen(filename, "w");
    if(!fp)
	{
		printf("===============================================\n");
	    printf("Can not open the data file - OBJ File Export!\n");
		printf("===============================================\n");
	    return;
	}

	fprintf(fp,"# The units used in this file are meters.\n");
	
	i=1;
	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;i++) {
		node=(QMeshNode *)(nodeList.GetNext(Pos));
		node->GetCoord3D(xx,yy,zz);
		node->SetIndexNo(i);
//		fprintf(fp,"v %.5f %.5f %.5f\n",(float)yy,(float)zz,(float)xx);
		fprintf(fp,"v %.5f %.5f %.5f\n",(float)xx,(float)yy,(float)zz);
//		fprintf(fp,"v %.12f %.12f %.12f\n",(float)zz,(float)xx,(float)yy);
	}

	fprintf(fp,"\n");
	
	for(Pos=faceList.GetHeadPosition();Pos!=NULL;) {
		face=(QMeshFace *)(faceList.GetNext(Pos));
		num=face->GetEdgeNum();
		
		fprintf(fp,"f ");
		index=face->GetNodeRecordPtr(0)->GetIndexNo();
		fprintf(fp,"%d ",index);
		index=face->GetNodeRecordPtr(1)->GetIndexNo();
		fprintf(fp,"%d ",index);
		index=face->GetNodeRecordPtr(2)->GetIndexNo();
		fprintf(fp,"%d ",index);
		fprintf(fp,"\n");

		for(i=3;i<num;i++) {
			fprintf(fp,"f ");
			index=face->GetNodeRecordPtr(0)->GetIndexNo();
			fprintf(fp,"%d ",index);
			index=face->GetNodeRecordPtr(i-1)->GetIndexNo();
			fprintf(fp,"%d ",index);
			index=face->GetNodeRecordPtr(i)->GetIndexNo();
			fprintf(fp,"%d ",index);
			fprintf(fp,"\n");
		}
	}

	fclose(fp);
}

void QMeshPatch::outputOBJFile(char* filename, bool bOBTFile)
{
	FILE *fp;
	GLKPOSITION Pos;
	QMeshNode *node;
	QMeshFace *face;
	double xx,yy,zz;
	int i,num,index;

	fp = fopen(filename, "w");
    if(!fp)
	{
		printf("===============================================\n");
	    printf("Can not open the data file - OBJ File Export!\n");
		printf("===============================================\n");
	    return;
	}

	fprintf(fp,"# The units used in this file are meters.\n");
	
	i=1;
	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;i++) {
		node=(QMeshNode *)(nodeList.GetNext(Pos));
		node->GetCoord3D(xx,yy,zz);
		node->SetIndexNo(i);
        if (bOBTFile)
            fprintf(fp,"v %.12f %.12f %.12f\n",xx,yy,zz,node->GetWeight());
        else
            fprintf(fp,"v %.12f %.12f %.12f\n",xx,yy,zz);
	}

	fprintf(fp,"\n");
	
	for(Pos=faceList.GetHeadPosition();Pos!=NULL;) {
		face=(QMeshFace *)(faceList.GetNext(Pos));
		num=face->GetEdgeNum();
		fprintf(fp,"f ");
		for(i=0;i<num;i++) {
			index=face->GetNodeRecordPtr(i)->GetIndexNo();
			fprintf(fp,"%d ",index);
		}
		fprintf(fp,"\n");
	}

	fclose(fp);
}

bool QMeshPatch::GetAttribFlag( const int whichBit )
{
	return flags[whichBit];
}

void QMeshPatch::SetAttribFlag( const int whichBit, const bool toBe )
{
	flags[whichBit]=toBe;
}

int QMeshPatch:: GetIndexNo() //from 1 to n
{
	return indexno;
}

void QMeshPatch::SetIndexNo( const int _index )
{
	indexno=_index;
}

int QMeshPatch::GetTetraNumber()
{
	return tetraList.GetCount();	//from 1 to n
}

QMeshTetra* QMeshPatch::GetTetraRecordPtr(int No)	//from 1 to n
{
	if ((No < 1) || (No > tetraList.GetCount()))    return  NULL;
	return (QMeshTetra *)tetraList.GetAt(tetraList.FindIndex(No - 1));
}

GLKObList& QMeshPatch::GetTetraList()
{
	return tetraList;
}

int QMeshPatch::GetFaceNumber()
{
	return faceList.GetCount();	//from 1 to n
}

QMeshFace* QMeshPatch::GetFaceRecordPtr(int No)	//from 1 to n
{
	if( (No < 1) || (No > faceList.GetCount()))    return  NULL;
    return (QMeshFace *)faceList.GetAt(faceList.FindIndex(No-1));
}

GLKObList& QMeshPatch::GetFaceList()
{
	return faceList;
}

int QMeshPatch::GetEdgeNumber()	//from 1 to n
{
	return edgeList.GetCount();
}

QMeshEdge* QMeshPatch::GetEdgeRecordPtr(int No)	//from 1 to n
{
	if( (No < 1) || (No > edgeList.GetCount()))    return  NULL;
    return (QMeshEdge *)edgeList.GetAt(edgeList.FindIndex(No-1));
}

GLKObList& QMeshPatch::GetEdgeList() 
{
	return edgeList;
}

int QMeshPatch::GetNodeNumber()	//from 1 to n
{
	return nodeList.GetCount();
}

QMeshNode* QMeshPatch::GetNodeRecordPtr(int No)	//from 1 to n
{
	if( (No < 1) || (No > nodeList.GetCount()))    return  NULL;
    return (QMeshNode *)nodeList.GetAt(nodeList.FindIndex(No-1));
}

GLKObList& QMeshPatch::GetNodeList() 
{
	return nodeList;
}

GLKObList& QMeshPatch::GetAttrib_EdgeList()
{
    return Attrib_EdgeList;
}

QMeshPatch *QMeshPatch::CopyMesh()
{
    QMeshPatch *newPatch = new QMeshPatch;
    for (GLKPOSITION pos=nodeList.GetHeadPosition(); pos!=nullptr;){
        QMeshNode *node = (QMeshNode*)nodeList.GetNext(pos);
        double xx,yy,zz;
        node->GetCoord3D(xx,yy,zz);
        QMeshNode *newNode = new QMeshNode;
        newNode->SetMeshPatchPtr(newPatch);
        newNode->SetCoord3D(xx,yy,zz);
        newNode->SetIndexNo(newPatch->GetNodeList().GetCount()+1);
        newNode->m_nIdentifiedPatchIndex = node->m_nIdentifiedPatchIndex;
        newNode->identifiedIndex = node->identifiedIndex;
        newNode->SetIndexNo(node->GetIndexNo());
        newNode->selected = node->selected;
        newPatch->GetNodeList().AddTail(newNode);
    }
    int nodeNum=newPatch->GetNodeList().GetCount();
    QMeshNode **nodeArray=new QMeshNode*[nodeNum];
    int i=0;
    for(GLKPOSITION pos=newPatch->GetNodeList().GetHeadPosition();pos!=NULL;i++) {
        QMeshNode *node=(QMeshNode*)(newPatch->GetNodeList().GetNext(pos));
        nodeArray[i]=node;
    }
    for (GLKPOSITION pos=faceList.GetHeadPosition(); pos!=nullptr;){
        QMeshFace *face = (QMeshFace*)faceList.GetNext(pos);
        QMeshFace *newFace = new QMeshFace;
        newFace->SetMeshPatchPtr(newPatch);
        newFace->SetIndexNo(newPatch->GetFaceList().GetCount()+1);
        newFace->m_nIdentifiedPatchIndex = face->m_nIdentifiedPatchIndex;
        newFace->identifiedIndex = face->identifiedIndex;
        newFace->selected = face->selected;
        newPatch->GetFaceList().AddTail(newFace);
        int Id[3];
        for (int i=0; i<3; i++){
            QMeshNode *faceNode = face->GetNodeRecordPtr(i);
            Id[i] = faceNode->GetIndexNo()-1;
            newFace->GetAttachedList().AddTail(nodeArray[Id[i]]);
        }
    }
    delete []nodeArray;
    for(GLKPOSITION pos=newPatch->GetFaceList().GetHeadPosition();pos!=nullptr;) {
        QMeshFace *face=(QMeshFace*)(newPatch->GetFaceList().GetNext(pos));

        int edgeNum=(face->GetAttachedList()).GetCount();
        face->SetEdgeNum(edgeNum);

        nodeArray=new QMeshNode*[edgeNum];
        int i=0;
        for(GLKPOSITION PosNode=(face->GetAttachedList()).GetHeadPosition();PosNode!=nullptr;i++) {
            nodeArray[i]=(QMeshNode*)((face->GetAttachedList()).GetNext(PosNode));
            (nodeArray[i]->GetFaceList()).AddTail(face);
        }

        for(int i=0;i<edgeNum;i++) {
            QMeshEdge *edge=nullptr;	QMeshNode *startNode=nodeArray[i];	QMeshNode *endNode=nodeArray[(i+1)%edgeNum];
            bool bDir;
            for(GLKPOSITION PosNode=(startNode->GetEdgeList()).GetHeadPosition();PosNode!=nullptr;) {
                QMeshEdge *temp=(QMeshEdge *)((startNode->GetEdgeList()).GetNext(PosNode));
                if ((temp->GetStartPoint()==startNode) && (temp->GetEndPoint()==endNode) && (temp->GetLeftFace()==nullptr)) {
                    edge=temp;	bDir=true;
                }
                else if ((temp->GetStartPoint()==endNode) && (temp->GetEndPoint()==startNode) && (temp->GetRightFace()==nullptr)) {
                    edge=temp;	bDir=false;
                }
            }
            if (edge && bDir) {
                face->SetEdgeRecordPtr(i,edge);
                face->SetDirectionFlag(i,true);
                edge->SetLeftFace(face);
            }
            else if (edge && (!bDir)) {
                face->SetEdgeRecordPtr(i,edge);
                face->SetDirectionFlag(i,false);
                edge->SetRightFace(face);
            }
            else {
                edge=new QMeshEdge;
                edge->SetMeshPatchPtr(newPatch);
                edge->SetStartPoint(startNode);
                edge->SetEndPoint(endNode);
                edge->SetIndexNo(newPatch->GetEdgeList().GetCount()+1);
                newPatch->GetEdgeList().AddTail(edge);

                edge->SetLeftFace(face);
                face->SetEdgeRecordPtr(i,edge);
                face->SetDirectionFlag(i,true);
                (startNode->GetEdgeList()).AddTail(edge);
                (endNode->GetEdgeList()).AddTail(edge);
            }
        }

        delete [](QMeshNode**)nodeArray;
        face->GetAttachedList().RemoveAll();
    }
    //---------------------------------------------------------------------
    //	Step 2: compute the normal
    for(GLKPOSITION Pos=newPatch->GetFaceList().GetHeadPosition();Pos!=nullptr;) {
        QMeshFace *face=(QMeshFace*)(newPatch->GetFaceList().GetNext(Pos));
        face->CalPlaneEquation();
        //face->selected = false;
        //face->m_nIdentifiedPatchIndex = -1;
    }
    QMeshEdge **edgeArray = new QMeshEdge*[edgeList.GetCount()];
    int index = 0;
    for (GLKPOSITION pos=edgeList.GetHeadPosition(); pos!=nullptr; index++){
        QMeshEdge *edge = (QMeshEdge*)edgeList.GetNext(pos);
        edgeArray[index] = edge;
    }
    index = 0;
    for(GLKPOSITION Pos=newPatch->GetEdgeList().GetHeadPosition();Pos!=nullptr; index++) {
        QMeshEdge *edge=(QMeshEdge*)(newPatch->GetEdgeList().GetNext(Pos));
        //edge->seamIndex = edgeArray[index]->seamIndex;
        //edge->cableIndex = edgeArray[index]->cableIndex;
        edge->selected = edgeArray[index]->selected;
    }
    return newPatch;
}

void QMeshPatch::ComputeBoundingBox(double &xmin, double &ymin, double &zmin, double &xmax, double &ymax, double &zmax)
{
    GLKPOSITION Pos;
    GLKPOSITION PosNode;
    double cx,cy,cz;

    xmin=1.0e+32;	xmax=-1.0e+32;
    ymin=1.0e+32;	ymax=-1.0e+32;
    zmin=1.0e+32;	zmax=-1.0e+32;
    for(PosNode=nodeList.GetHeadPosition();PosNode!=NULL;) {
        QMeshNode *node=(QMeshNode *)nodeList.GetNext(PosNode);
        node->GetCoord3D(cx,cy,cz);

        if (cx>xmax) xmax=cx;
        if (cx<xmin) xmin=cx;
        if (cy>ymax) ymax=cy;
        if (cy<ymin) ymin=cy;
        if (cz>zmax) zmax=cz;
        if (cz<zmin) zmin=cz;
    }
}

//xmin xmax ymin ymax zmin zmax
void QMeshPatch::ComputeBoundingBox(double boundingBox[])
{
    GLKPOSITION PosMesh;
    GLKPOSITION Pos;
    double xx,yy,zz;

    boundingBox[0]=boundingBox[2]=boundingBox[4]=1.0e+32;
    boundingBox[1]=boundingBox[3]=boundingBox[5]=-1.0e+32;

    for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
        QMeshNode *node=(QMeshNode *)(nodeList.GetNext(Pos));
        node->GetCoord3D(xx,yy,zz);

        if (xx<boundingBox[0]) boundingBox[0]=xx;
        if (xx>boundingBox[1]) boundingBox[1]=xx;
        if (yy<boundingBox[2]) boundingBox[2]=yy;
        if (yy>boundingBox[3]) boundingBox[3]=yy;
        if (zz<boundingBox[4]) boundingBox[4]=zz;
        if (zz>boundingBox[5]) boundingBox[5]=zz;
    }
}

bool QMeshPatch::inputPosNorFile(char* filename, bool Yup2Zup) {

	FILE* fp;
	char linebuf[2048];
	GLKPOSITION Pos;
	QMeshNode* node, * startNode, * endNode;
	QMeshEdge* edge;
	QMeshNode** nodeArray;
	float xx, yy, zz, nx, ny, nz;

	fp = fopen(filename, "r");
	if (!fp) {
		printf("==================================================\n");
		printf("Can not open the data file - Waypoint File Import!\n");
		printf("==================================================\n");
		return false;
	}
	ClearAll();

	int negativeNormalNum = 0;
	while (true) {
		fgets(linebuf, 255, fp);
		if (feof(fp)) break;
		//cout << linebuf << endl;
		sscanf(linebuf, "%f %f %f %f %f %f\n", &xx, &yy, &zz, &nx, &ny, &nz);
		node = new QMeshNode;
		node->SetIndexNo(nodeList.GetCount()); // start from 0
		node->resampleChecked = true;
		node->SetMeshPatchPtr(this);
		node->SetCoord3D(xx, yy, zz);
		node->SetNormal(nx, ny, nz);

		if (Yup2Zup) {
			if (ny < 0) negativeNormalNum++;
		}
		else {
			if (nz < 0) negativeNormalNum++;
		}
		nodeList.AddTail(node);
	}
	fclose(fp);

	if ((double)negativeNormalNum / nodeList.GetCount() < 0.2) {
		//std::cout << filename << std::endl;
		for (Pos = nodeList.GetHeadPosition(); Pos != NULL;) {
			node = (QMeshNode*)(nodeList.GetNext(Pos));

			double temp_nx, temp_ny, temp_nz;
			node->GetNormal(temp_nx, temp_ny, temp_nz);
			node->SetNormal(-temp_nx, -temp_ny, -temp_nz);// flip the normal
		}
	}

	nodeArray = new QMeshNode * [nodeList.GetCount()];
	int i = 0;
	for (Pos = nodeList.GetHeadPosition(); Pos != NULL; ++i) {
		node = (QMeshNode*)(nodeList.GetNext(Pos));
		nodeArray[i] = node;
	}

	for (i = 0; i < nodeList.GetCount() - 1; i++) {
		edge = NULL;
		startNode = nodeArray[i];
		endNode = nodeArray[i + 1];
		edge = new QMeshEdge;
		edge->SetMeshPatchPtr(this);
		edge->SetStartPoint(startNode);
		edge->SetEndPoint(endNode);
		edge->SetIndexNo(edgeList.GetCount()); // start from 0
		edgeList.AddTail(edge);
		//std::cout << "edge length" << edge->CalLength() << std::endl;
	}

	delete[]nodeArray;
	fclose(fp);
	std::cout << "Finish input Waypoint" << std::endl;
	return true;
}

