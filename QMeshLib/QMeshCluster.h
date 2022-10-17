#pragma once

#include "../GLKLib/GLKObList.h"

class QMeshNode;
class QMeshFace;
class CCH_ATTRIB_EDGE;


class QMeshCluster : public GLKObject
{
public:
	QMeshCluster();
	virtual ~QMeshCluster();

public:

	int GetIndexNo();		//from 1 to n
	void SetIndexNo( const int _index = 1 );

	int GetClusterFaceNumber();
	int GetClusterNodeNumber();
    int GetClusterEdgeNumber();

	QMeshFace* GetClusterFaceRecordPtr(int No);//from 1 to n
	QMeshNode* GetClusterNodeRecordPtr(int No);//from 1 to n
	QMeshEdge* GetClusterEdgeRecordPtr(int No);//from 1 to n
    
	GLKObList& GetClusterFaceList();
	GLKObList& GetClusterNodeList();
	GLKObList& GetClusterEdgeList();
	GLKObList& GetBndCurveList();

	GLKObList& GetAnchorNodeList();
   
	QMeshFace* centroidFace;

private:
	int indexno;			// start from 1 to n  

	GLKObList clusterFaceList;	// a list of cluster's faces
	GLKObList clusterNodeList;	// a list of cluster's nodes
	GLKObList clusterEdgeList;	// a list of cluster's Edges
	GLKObList clusterAnchorNodeList; //a list of cluster's Anchor Nodes
	GLKObList clusterBndCurveList; //a list of cluster's Boundary curves
};

class QMeshClusterBndCurve : public GLKObject
{
public:
	QMeshClusterBndCurve();
	virtual ~QMeshClusterBndCurve();

	GLKObList *GetEdgeList() {return m_edgeList;};

	void SetLeftCluster(QMeshCluster *cluster) {m_leftCluster=cluster;};
	QMeshCluster *GetLeftCluster() {return m_leftCluster;};
	void SetRightCluster(QMeshCluster *cluster) {m_rightCluster=cluster;};
	QMeshCluster *GetRightCluster() {return m_rightCluster;};
	void SetGlobalIndex(int _Index) {GlobalIndex = _Index;};
	int GetGlobalIndex() {return GlobalIndex;};

	QMeshNode * GetStartPoint();
    void SetStartPoint(QMeshNode* _pStartPoint = nullptr );
	
	QMeshNode * GetEndPoint();
    void SetEndPoint(QMeshNode* _pEndPoint = nullptr );

	CCH_ATTRIB_EDGE* attrEdge;
private:
	int GlobalIndex;
	QMeshNode* pStartPoint, * pEndPoint;
	QMeshCluster* m_leftCluster, * m_rightCluster;
	GLKObList* m_edgeList;	//	Flag 4 of each edge is used here to indicate the direction of 
							//	this edge in this boundary curve
};
