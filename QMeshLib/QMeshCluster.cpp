#include "QMeshEdge.h"
#include "QMeshCluster.h"

QMeshCluster::QMeshCluster()
{
	clusterFaceList.RemoveAll();
	clusterNodeList.RemoveAll();
	clusterEdgeList.RemoveAll();
	clusterAnchorNodeList.RemoveAll();
	clusterBndCurveList.RemoveAll();
    centroidFace = nullptr;
}

QMeshCluster::~QMeshCluster()
{
	clusterFaceList.RemoveAll();
	clusterNodeList.RemoveAll();
	clusterEdgeList.RemoveAll();
	clusterAnchorNodeList.RemoveAll();
	clusterBndCurveList.RemoveAll();
    centroidFace = nullptr;
}

int QMeshCluster::GetIndexNo()
{
	return indexno;
}

void QMeshCluster::SetIndexNo( const int _index )
{
	indexno = _index;
}

int QMeshCluster::GetClusterFaceNumber() 
{
	return clusterFaceList.GetCount();
}

QMeshFace* QMeshCluster::GetClusterFaceRecordPtr(int No)  //from 1 to n
{
    if((No < 1)||(No>clusterFaceList.GetCount()))    return  nullptr;
    return (QMeshFace *)clusterFaceList.GetAt(clusterFaceList.FindIndex(No-1));
}

GLKObList& QMeshCluster::GetClusterFaceList()
{
	return clusterFaceList;
}


int QMeshCluster::GetClusterNodeNumber()
{
	return clusterNodeList.GetCount();
}

QMeshNode* QMeshCluster::GetClusterNodeRecordPtr(int No)//from 1 to n
{
    if((No<1)||(No>clusterNodeList.GetCount())) return  nullptr;
    return (QMeshNode *)clusterNodeList.GetAt(clusterNodeList.FindIndex(No-1));
}

GLKObList& QMeshCluster::GetClusterNodeList()
{
	return clusterNodeList;
}

int QMeshCluster::GetClusterEdgeNumber()
{
	return clusterEdgeList.GetCount();
}

QMeshEdge* QMeshCluster::GetClusterEdgeRecordPtr(int No)//from 1 to n
{
    if( (No < 1) || (No > clusterEdgeList.GetCount()))    return  nullptr;
    return (QMeshEdge *)clusterEdgeList.GetAt(clusterEdgeList.FindIndex(No-1));
}

GLKObList& QMeshCluster::GetClusterEdgeList()
{
	return clusterEdgeList;
}

GLKObList& QMeshCluster::GetAnchorNodeList()
{
	return clusterAnchorNodeList;
}
GLKObList& QMeshCluster::GetBndCurveList()
{
	return clusterBndCurveList;
}

QMeshClusterBndCurve::QMeshClusterBndCurve()
{
    m_leftCluster=nullptr;	m_rightCluster=nullptr; attrEdge = nullptr; GlobalIndex = -1;
	m_edgeList=new GLKObList;	m_edgeList->RemoveAll(); 
}

QMeshClusterBndCurve::~QMeshClusterBndCurve()
{
	delete m_edgeList;
// 	if (attrEdge)
// 	{
// 		delete attrEdge;
// 		attrEdge = NULL;
// 	}
}

QMeshNode * QMeshClusterBndCurve::GetStartPoint()
{
	return pStartPoint;
}

void QMeshClusterBndCurve::SetStartPoint( QMeshNode * _pStartPoint )
{
	pStartPoint = _pStartPoint;
}

QMeshNode * QMeshClusterBndCurve::GetEndPoint()
{
	return pEndPoint;
}

void QMeshClusterBndCurve::SetEndPoint( QMeshNode * _pEndPoint )
{
	pEndPoint = _pEndPoint;
}
