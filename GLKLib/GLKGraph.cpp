// GLKGraph.cpp: implementation of the GLKGraph class.
//
//////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include "GLKGraph.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

GLKGraph::GLKGraph()
{
	nodeList.RemoveAll();	edgeList.RemoveAll();
}

GLKGraph::~GLKGraph()
{
	clearAll();
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

void GLKGraph::_Debug() 
{
	GLKGraphNode *node1;
	GLKGraphNode *node2;
	GLKGraphNode *node3;
	GLKGraphNode *node4;
	GLKGraphNode *node5;
	GLKGraphEdge *edge1;
	GLKGraphEdge *edge2;
	GLKGraphEdge *edge3;
	GLKGraphEdge *edge4;
	GLKGraphEdge *edge5;
	GLKGraphEdge *edge6;
	GLKGraphEdge *edge7;
	GLKObList srList,trList;

	node1=new GLKGraphNode;	AddNode(node1);
	node2=new GLKGraphNode;	AddNode(node2);
	node3=new GLKGraphNode;	AddNode(node3);
	node4=new GLKGraphNode;	AddNode(node4);
	node5=new GLKGraphNode;	AddNode(node5);

	edge1=new GLKGraphEdge;	edge1->startNode=node1;	edge1->endNode=node2; edge1->m_weight=12.0;
	AddEdge(edge1);
	edge2=new GLKGraphEdge;	edge2->startNode=node3;	edge2->endNode=node1; edge2->m_weight=14.0;
	AddEdge(edge2);
	edge3=new GLKGraphEdge;	edge3->startNode=node2;	edge3->endNode=node3; edge3->m_weight=5.0;
	AddEdge(edge3);
	edge4=new GLKGraphEdge;	edge4->startNode=node3;	edge4->endNode=node4; edge4->m_weight=8.0;
	AddEdge(edge4);
	edge5=new GLKGraphEdge;	edge5->startNode=node4;	edge5->endNode=node5; edge5->m_weight=10.0;
	AddEdge(edge5);
	edge6=new GLKGraphEdge;	edge6->startNode=node4;	edge6->endNode=node2; edge6->m_weight=7.0;
	AddEdge(edge6);
	edge7=new GLKGraphEdge;	edge7->startNode=node2;	edge7->endNode=node5; edge7->m_weight=16.0;
	AddEdge(edge7);

	FillInEdgeLinkersOnNodes();
	double maxFlow=MinimumCut(node1,node5,&srList,&trList,true);

	GLKPOSITION Pos;
	for(Pos=srList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphNode *node=(GLKGraphNode *)(srList.GetNext(Pos));
	}
}

double GLKGraph::MinimumCut(GLKGraphNode *sourceNode, GLKGraphNode *targetNode, 
							GLKObList *sourceRegionNodeList, GLKObList *targetRegionNodeList,
							bool bComputeMaxFlow)
{
	GLKObList linkList;
	GLKPOSITION Pos;

	_initializePreflow(sourceNode);

	linkList.RemoveAll();
	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphNode *node=(GLKGraphNode *)(nodeList.GetNext(Pos));
		if (node==sourceNode) continue;
		if (node==targetNode) continue;
		linkList.AddTail(node);
	}

	int oldHeight;	GLKGraphNode *uNode;
	Pos=linkList.GetHeadPosition();
	while(Pos!=NULL) {
		uNode=(GLKGraphNode *)(linkList.GetAt(Pos));
	
		oldHeight=uNode->m_height;
		
		_discharge(uNode);
		
		if (uNode->m_height>oldHeight) {
			linkList.RemoveAt(Pos);
			linkList.AddHead(uNode);
			Pos=linkList.GetHeadPosition();
		}
		
		linkList.GetNext(Pos);
	}

	_partitionByResidualGraph(sourceNode,targetNode,sourceRegionNodeList,targetRegionNodeList);
	if (bComputeMaxFlow) return _computeMaxFlow();
	return 0.0;
}

void GLKGraph::_partitionByResidualGraph(GLKGraphNode *sourceNode, GLKGraphNode *targetNode, 
										 GLKObList *sourceRegionNodeList, 
										 GLKObList *targetRegionNodeList)
{
	GLKPOSITION Pos;

	sourceRegionNodeList->RemoveAll();
	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphNode *node=(GLKGraphNode *)(nodeList.GetNext(Pos));
		node->m_height=0;
	}

	_propagateInResidualGraph(sourceNode,sourceRegionNodeList);

	targetRegionNodeList->RemoveAll();
	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphNode *node=(GLKGraphNode *)(nodeList.GetNext(Pos));
		if (node->m_height==0) targetRegionNodeList->AddTail(node);
	}
}

double GLKGraph::_computeMaxFlow()
{
	GLKPOSITION Pos;
	double value=0.0;

	for(Pos=edgeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphEdge *edge=(GLKGraphEdge *)(edgeList.GetNext(Pos));
		if ((edge->startNode->m_height==1) && (edge->endNode->m_height==0))
			value+=edge->m_weight;
		if ((edge->endNode->m_height==1) && (edge->startNode->m_height==0))
			value+=edge->m_weight;
	}

	return value;
}

void GLKGraph::_propagateInResidualGraph(GLKGraphNode *node, GLKObList *regionNodeList)
{
	GLKPOSITION Pos;
	GLKGraphNode *otherNode;

	regionNodeList->AddTail(node);
	node->m_height=1;	// to specify that it has been added into the list

	for(Pos=node->edgeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphEdge *edge=(GLKGraphEdge *)(node->edgeList.GetNext(Pos));
		double cf;

		if (edge->startNode==node) {
			cf=edge->m_weight-edge->m_flow;
			if (cf<=1.0e-8) continue;
			otherNode=edge->endNode;
		}
		else {
			cf=edge->m_weight+edge->m_flow;
			if (cf<=1.0e-8) continue;
			otherNode=edge->startNode;
		}

		if (otherNode->m_height==0) // to detect whether it has been added into the list
			_propagateInResidualGraph(otherNode, regionNodeList);
	}
}

void GLKGraph::_discharge(GLKGraphNode *uNode)
{
	GLKPOSITION Pos=uNode->edgeList.GetHeadPosition();

	while(uNode->m_excess>0.0) {
		if (Pos==NULL) {
			_relable(uNode);
			Pos=uNode->edgeList.GetHeadPosition();
		}
		else {
			GLKGraphEdge *edge=(GLKGraphEdge *)(uNode->edgeList.GetAt(Pos));
			_push(uNode,edge);
			uNode->edgeList.GetNext(Pos);
		}
	}
}

void GLKGraph::_push(GLKGraphNode *uNode, GLKGraphEdge *edge)
{
	GLKGraphNode *vNode;
	double df,cf;

	if (uNode->m_excess==0.0) return;

	if (uNode==edge->startNode) {
		vNode=edge->endNode;
		if (uNode->m_height!=(vNode->m_height+1)) return;
		cf=edge->m_weight-edge->m_flow;	
		if (cf<=0.0) return;

		df=uNode->m_excess;
		if (cf<df) df=cf;

		edge->m_flow=edge->m_flow+df;
		uNode->m_excess-=df;
		vNode->m_excess+=df;
	}
	else {	// the edge pointing to vNode to uNode but with some flow filled
		vNode=edge->startNode;
		if (uNode->m_height!=(vNode->m_height+1)) return;
		cf=edge->m_weight+edge->m_flow;
		if (cf<=0.0) return;

		df=uNode->m_excess;
		if (cf<df) df=cf;

		edge->m_flow=edge->m_flow-df;
		uNode->m_excess-=df;
		vNode->m_excess+=df;
	}
}

void GLKGraph::_relable(GLKGraphNode *uNode)
{
	if (uNode->m_excess==0.0) return;

	GLKGraphNode *vNode;
	GLKPOSITION Pos;	int minH;	double cf;	
	
	minH=-1;
	for(Pos=uNode->edgeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphEdge *edge=(GLKGraphEdge *)(uNode->edgeList.GetNext(Pos));
		if (edge->startNode==uNode) {
			vNode=edge->endNode;
			cf=edge->m_weight-edge->m_flow;
			if (cf<=0.0) continue;
//			if (uNode->m_height>vNode->m_height) continue;
			if ((minH<0) || (vNode->m_height<minH))	minH=vNode->m_height;
		}
		else {
			vNode=edge->startNode;
			cf=edge->m_weight+edge->m_flow;
			if (cf<=0.0) continue;
//			if (uNode->m_height>vNode->m_height) continue;
			if ((minH<0) || (vNode->m_height<minH))	minH=vNode->m_height;
		}
	}
	if (minH<0) return;

	uNode->m_height=1+minH;
}

void GLKGraph::_initializePreflow(GLKGraphNode *sourceNode)
{
	GLKPOSITION Pos;

	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphNode *node=(GLKGraphNode *)(nodeList.GetNext(Pos));
		node->m_height=0;
		node->m_excess=0.0;
	}
	for(Pos=edgeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphEdge *edge=(GLKGraphEdge *)(edgeList.GetNext(Pos));
		edge->m_flow=0.0;
	}
	sourceNode->m_height=nodeList.GetCount();

	for(Pos=sourceNode->edgeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphEdge *edge=(GLKGraphEdge *)(sourceNode->edgeList.GetNext(Pos));
		if (edge->startNode==sourceNode) {	// pointing out direction
			edge->m_flow=edge->m_weight; // capacity
			edge->endNode->m_excess=edge->m_weight;
			sourceNode->m_excess=sourceNode->m_excess-edge->m_weight;
		}
		else {	// pointing out direction
			edge->m_flow=-(edge->m_weight);
			edge->startNode->m_excess=edge->m_weight;
			sourceNode->m_excess=sourceNode->m_excess-edge->m_weight;
		}
	}
}

void GLKGraph::AddNode(GLKGraphNode *node)
{
	nodeList.AddTail(node);
}

void GLKGraph::FillInEdgeLinkersOnNodes()
{
	GLKPOSITION Pos;

	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphNode *node=(GLKGraphNode *)(nodeList.GetNext(Pos));
		node->edgeList.RemoveAll();
	}

	for(Pos=edgeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphEdge *edge=(GLKGraphEdge *)(edgeList.GetNext(Pos));
		edge->startNode->edgeList.AddTail(edge);
		edge->endNode->edgeList.AddTail(edge);
	}
}

void GLKGraph::AddEdge(GLKGraphEdge *edge)
{
	edgeList.AddTail(edge);
}

void GLKGraph::clearAll()
{
	GLKPOSITION Pos;

	for(Pos=nodeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphNode *node=(GLKGraphNode *)(nodeList.GetNext(Pos));
		delete node;
	}
	nodeList.RemoveAll();

	for(Pos=edgeList.GetHeadPosition();Pos!=NULL;) {
		GLKGraphEdge *edge=(GLKGraphEdge *)(edgeList.GetNext(Pos));
		delete edge;
	}
	edgeList.RemoveAll();
}
