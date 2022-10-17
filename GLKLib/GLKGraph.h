// GLKGraph.h: interface for the GLKGraph class.
//
//////////////////////////////////////////////////////////////////////

#include "GLKObList.h"

#ifndef _GLKGRAPHNODE
#define _GLKGRAPHNODE

class GLKGraphNode : public GLKObject
{
public:
	GLKGraphNode() {edgeList.RemoveAll();attachedObj=NULL;};
	virtual ~GLKGraphNode() {};
	void *attachedObj;
	GLKObList edgeList;

	//---------------------------------------------------------------
	//	the following variables are for minimum cut
	double m_excess;
	int m_height;
	GLKGraphNode *nextNode;
};

#endif

#ifndef _GLKGRAPHEDGE
#define _GLKGRAPHEDGE

class GLKGraphEdge : public GLKObject
{
public:
	GLKGraphEdge() {startNode=NULL;	endNode=NULL;	m_weight=0.0;};
	virtual ~GLKGraphEdge() {};

	GLKGraphNode* startNode;
	GLKGraphNode* endNode;
	double m_weight;
	void *attachedObj;

	//---------------------------------------------------------------
	//	the following variables are for minimum cut
	double m_flow;
};

#endif

#ifndef _GLKGRAPH
#define _GLKGRAPH

class GLKGraphCutNode;

class GLKGraph  
{
public:
	GLKGraph();
	virtual ~GLKGraph();

	void AddNode(GLKGraphNode *node);
	void AddEdge(GLKGraphEdge *edge);
	void FillInEdgeLinkersOnNodes();

	void _Debug();

	//---------------------------------------------------------------------
	//	The following function is implemented by the relabel-to-front algorithm
public:
	double MinimumCut(GLKGraphNode *sourceNode, GLKGraphNode *targetNode, 
			GLKObList *sourceRegionNodeList, GLKObList *targetRegionNodeList, 
			bool bComputeMaxFlow=false);
private:
	void _initializePreflow(GLKGraphNode *sourceNode);
	void _discharge(GLKGraphNode *uNode);
	void _push(GLKGraphNode *uNode, GLKGraphEdge *edge);
	void _relable(GLKGraphNode *uNode);
	void _partitionByResidualGraph(GLKGraphNode *sourceNode, GLKGraphNode *targetNode, 
			GLKObList *sourceRegionNodeList, GLKObList *targetRegionNodeList);
	void _propagateInResidualGraph(GLKGraphNode *node, GLKObList *regionNodeList);
	double _computeMaxFlow();

private:
	void clearAll();

	GLKObList nodeList;
	GLKObList edgeList;
};

#endif
