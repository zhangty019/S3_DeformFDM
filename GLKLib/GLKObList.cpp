// GLKObList.cpp: implementation of the GLKObList class.
//
//////////////////////////////////////////////////////////////////////

#include "GLKObList.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

GLKObList::GLKObList()
{
    headPos=nullptr;	tailPos=nullptr;	nCount=0;
}

GLKObList::~GLKObList()
{
	RemoveAll();
}

//////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////

GLKPOSITION GLKObList::InsertBefore( GLKPOSITION rPosition, GLKObject* newElement )
{
	GLKPOSITION newPos;
	GLKObNode *newNode=new GLKObNode;	newNode->data=newElement;
	GLKObNode *currentNode=rPosition;
	if (currentNode)
	{
		currentNode->InsertBefore(newNode);
		newPos=newNode;
        if (newNode->prev==nullptr) headPos=newNode;
		nCount++;
		return newPos;
	}
	delete newNode;
    return nullptr;
}

GLKPOSITION GLKObList::InsertAfter( GLKPOSITION rPosition, GLKObject* newElement )
{
	GLKPOSITION newPos;
	GLKObNode *newNode=new GLKObNode;	newNode->data=newElement;
	GLKObNode *currentNode=rPosition;
	if (currentNode)
	{
		currentNode->InsertAfter(newNode);
		newPos=newNode;
        if (newNode->next==nullptr) tailPos=newNode;
		nCount++;
		return newPos;
	}
	delete newNode;
    return nullptr;
}

GLKPOSITION GLKObList::AddHead( GLKObject* newElement )
{
	GLKObNode *newNode=new GLKObNode;	newNode->data=newElement;
	if (headPos) headPos->InsertBefore(newNode);
	headPos=newNode;	nCount++;
	if (!tailPos) tailPos=newNode;

	return newNode;
}
	
GLKPOSITION GLKObList::AddTail( GLKObject* newElement )
{
	GLKObNode *newNode=new GLKObNode;	newNode->data=newElement;
	if (tailPos) tailPos->InsertAfter(newNode);
	tailPos=newNode;	nCount++;
	if (!headPos) headPos=newNode;

	return newNode;
}

GLKObject* GLKObList::GetHead() {return headPos->data;}
GLKObject* GLKObList::GetTail() {return tailPos->data;}

GLKObject* GLKObList::RemoveHead()
{
	GLKObNode* tempNode=headPos;
	if (headPos) {
		GLKObject* tempObj=tempNode->data;
		headPos=headPos->next;
        if (headPos) headPos->prev=nullptr;
		nCount--;
        if (nCount==0) tailPos=nullptr;
		delete tempNode;
		return tempObj;
	}
    return nullptr;
}
	
GLKObject* GLKObList::RemoveTail()
{
	GLKObNode* tempNode=tailPos;
	if (tailPos) {
		GLKObject* tempObj=tempNode->data;
		tailPos=tailPos->prev;
        if (tailPos) tailPos->next=nullptr;
		nCount--;
        if (nCount==0) headPos=nullptr;
		delete tempNode;
		return tempObj;
	}
    return nullptr;
}

GLKObject* GLKObList::RemoveAt(GLKPOSITION rPosition)
{
	GLKObNode* prevNode=rPosition->prev;
	GLKObNode* nextNode=rPosition->next;

    if ((nextNode==nullptr) && (prevNode==nullptr)) {
		GLKObject* tempObj=rPosition->data;
		RemoveAll();
		return tempObj;
	}

    if (nextNode==nullptr) return RemoveTail();
    if (prevNode==nullptr) return RemoveHead();

	GLKObject* tempObj=rPosition->data;
	prevNode->next=nextNode;
	nextNode->prev=prevNode;
	nCount--;

    if (nCount==0) {headPos=nullptr; tailPos=nullptr;}

	delete rPosition;

	return tempObj;
}

void GLKObList::Remove(GLKObject* element)
{
	GLKPOSITION Pos;

	Pos=Find(element);
	if (Pos) RemoveAt(Pos);
}

void GLKObList::RemoveAll()
{
	GLKObNode *node;
    for(node=headPos;node!=nullptr;)
	{
		GLKObNode* tempNode=node->next;
		delete node;
		node=tempNode;
	}

    nCount=0;	headPos=nullptr;	tailPos=nullptr;
}

void GLKObList::AttachListTail( GLKObList* pNewList )
{
	if (pNewList->IsEmpty()) return;

	GLKObNode *currentNode=tailPos;
	GLKObNode *nextNode=pNewList->GetHeadPosition();
		
	if (currentNode) {
		currentNode->next=pNewList->GetHeadPosition();
		if (nextNode) nextNode->prev=currentNode;
	}
	else {
		headPos=pNewList->GetHeadPosition();
	}
	tailPos=pNewList->GetTailPosition();
	nCount+=pNewList->GetCount();
}

void GLKObList::RemoveAllWithoutFreeMemory()
{
    nCount=0;	headPos=nullptr;	tailPos=nullptr;
}

GLKObject* GLKObList::GetNext( GLKPOSITION& rPosition )
{
	GLKPOSITION tempObj=rPosition;
	rPosition=tempObj->next;
	return tempObj->data;

}

GLKObject* GLKObList::GetAt( GLKPOSITION rPosition )
{
	return rPosition->data;
}
	
GLKObject* GLKObList::GetPrev( GLKPOSITION& rPosition )
{
	GLKPOSITION tempObj=rPosition;
	rPosition=tempObj->prev;
	return tempObj->data;
}

GLKPOSITION GLKObList::FindIndex(int index)
{
	GLKPOSITION Pos;	int n=0;

    for(Pos=GetHeadPosition();Pos!=nullptr;n++) {
		if (n==index) return Pos;
		GetNext(Pos);
	}
	
    return nullptr;
}

GLKPOSITION GLKObList::Find(GLKObject* element)
{
	GLKPOSITION Pos;	int n=0;

    for(Pos=GetHeadPosition();Pos!=nullptr;n++) {
		if (Pos->data==element) return Pos;
		GetNext(Pos);
	}

    return nullptr;
}

void GLKObList::AddHead( GLKObList* pNewList )
{
	GLKPOSITION Pos;
    for(Pos=pNewList->GetTailPosition();Pos!=nullptr;)
		this->AddHead(pNewList->GetPrev(Pos));
}

void GLKObList::AddTail( GLKObList* pNewList )
{
	GLKPOSITION Pos;
    for(Pos=pNewList->GetHeadPosition();Pos!=nullptr;)
		this->AddTail(pNewList->GetNext(Pos));
}





//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

GLKArray::GLKArray(int sz, int increaseStep, int type)
{
	m_type=type;
	size=0;		arraySize=sz;	step=increaseStep;

	switch(m_type)
	{
	case 0:{
			listType0=(void**)new long[arraySize];
		   }break;
	case 1:{
			listType1=new int[arraySize];
		   }break;
	case 2:{
			listType2=new float[arraySize];
		   }break;
	case 3:{
			listType3=new double[arraySize];
		   }
	}
}

GLKArray::~GLKArray()
{
	switch(m_type)
	{
	case 0:{
			delete [](void**)listType0;
		   }break;
	case 1:{
			delete [] listType1;
		   }break;
	case 2:{
			delete [] listType2;
		   }break;
	case 3:{
			delete [] listType3;
		   }break;
	}
}

//////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////

int GLKArray::GetSize()
{
	return size;
}

void GLKArray::RemoveAll()
{
	switch(m_type)
	{
	case 0:{
			delete [](void**)listType0;
			size=0;	arraySize=step;
			listType0=(void**)new long[arraySize];
		   }break;
	case 1:{
			delete [] listType1;
			size=0;	arraySize=step;
			listType1=new int[arraySize];
		   }break;
	case 2:{
			delete [] listType2;
			size=0;	arraySize=step;
			listType2=new float[arraySize];
		   }break;
	case 3:{
			delete [] listType3;
			size=0;	arraySize=step;
			listType3=new double[arraySize];
		   }break;
	}
}

void* GLKArray::RemoveAt(int i)
{
	int k;

	switch(m_type)
	{
	case 0:{
			void* returnObj=listType0[i];
			for(k=i;k<(size-1);k++) listType0[k]=listType0[k+1];
			size--;
			return returnObj;
		   }break;
	case 1:{
			for(k=i;k<(size-1);k++) listType1[k]=listType1[k+1];
			size--;
		   }break;
	case 2:{
			for(k=i;k<(size-1);k++) listType2[k]=listType2[k+1];
			size--;
		   }break;
	case 3:{
			for(k=i;k<(size-1);k++) listType3[k]=listType3[k+1];
			size--;
		   }break;
	}

    return nullptr;
}

void GLKArray::Add(void* data)
{
	if (m_type!=0) return;

	if (size==arraySize)	//	Resize
	{
		arraySize=arraySize+step;
		void** newlist=(void**)new long[arraySize];
		int n=size;
		void** srcpt=listType0;
		void** destpt=newlist;
		while(n--) *destpt++=*srcpt++;
		delete listType0;	listType0=newlist;
	}
	listType0[size]=data;	size++;
}

void GLKArray::Add(int data)
{
	if (m_type!=1) return;

	if (size==arraySize)	//	Resize
	{
		arraySize=arraySize+step;
		int* newlist=new int[arraySize];
		int n=size;
		int* srcpt=listType1;
		int* destpt=newlist;
		while(n--) *destpt++=*srcpt++;
		delete listType1;	listType1=newlist;
	}
	listType1[size]=data;	size++;
}

void GLKArray::Add(double data)
{
	if (m_type!=3) return;

	if (size==arraySize)	//	Resize
	{
		arraySize=arraySize+step;
		double* newlist=new double[arraySize];
		int n=size;
		double* srcpt=listType3;
		double* destpt=newlist;
		while(n--) *destpt++=*srcpt++;
		delete listType3;	listType3=newlist;
	}
	listType3[size]=data;	size++;
}

void GLKArray::Add(float data)
{
	if (m_type!=2) return;

	if (size==arraySize)	//	Resize
	{
		arraySize=arraySize+step;
		float* newlist=new float[arraySize];
		int n=size;
		float* srcpt=listType2;
		float* destpt=newlist;
		while(n--) *destpt++=*srcpt++;
		delete listType2;	listType2=newlist;
	}
	listType2[size]=data;	size++;
}

void GLKArray::InsertAt(int i, void* data)
{
	if (m_type!=0) return;
	Add(data);	
	for(int k=size-2;k>=i;k--) listType0[k+1]=listType0[k];
	listType0[i]=data;
}

void GLKArray::InsertAt(int i, int data)
{
	if (m_type!=1) return;
	Add(data);	
	for(int k=size-2;k>=i;k--) listType1[k+1]=listType1[k];
	listType1[i]=data;
}

void GLKArray::InsertAt(int i, float data)
{
	if (m_type!=2) return;
	Add(data);	
	for(int k=size-2;k>=i;k--) listType2[k+1]=listType2[k];
	listType2[i]=data;
}

void GLKArray::InsertAt(int i, double data)
{
	if (m_type!=3) return;
	Add(data);	
	for(int k=size-2;k>=i;k--) listType3[k+1]=listType3[k];
	listType3[i]=data;
}

void GLKArray::SetAt(int i, void* data)
{
	if (m_type!=0) return;
	listType0[i]=data;
}

void GLKArray::SetAt(int i, int data)
{
	if (m_type!=1) return;
	listType1[i]=data;
}

void GLKArray::SetAt(int i, float data)
{
	if (m_type!=2) return;
	listType2[i]=data;
}

void GLKArray::SetAt(int i, double data)
{
	if (m_type!=3) return;
	listType3[i]=data;
}


void* GLKArray::GetAt(int i)
{
    if (m_type!=0) return nullptr;
	return listType0[i];
}

int GLKArray::GetIntAt(int i)
{
	if (m_type!=1) return 0;
	return listType1[i];
}

float GLKArray::GetFloatAt(int i)
{
	if (m_type!=2) return 0;
	return listType2[i];
}

double GLKArray::GetDoubleAt(int i)
{
	if (m_type!=3) return 0;
	return listType3[i];
}
