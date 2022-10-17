// GLKObList.h: interface for the GLKObList class.
//
//////////////////////////////////////////////////////////////////////
#include <../ThirdPartyDependence/eigen3/Eigen/Dense> 
#include <vector>

#ifndef _GLKObject
#define _GLKObject

class GLKObject  
{
public:
	GLKObject () {};
	//int type;
};

#endif



#ifndef _GLKObNode
#define _GLKObNode

#define GLKPOSITION		GLKObNode*

class GLKObNode
{
public:
    GLKObNode(GLKObNode* ptrprev=nullptr,GLKObNode* ptrnext=nullptr)	{next=ptrnext;prev=ptrprev;};

	void InsertAfter(GLKObNode *p) {
		GLKObNode *oldNextNode=next;
		next=p;p->prev=this;
		if (oldNextNode) {oldNextNode->prev=p;p->next=oldNextNode;}
	};
	GLKObNode *DeleteAfter() {
		GLKObNode *tempObj=next;
        if (next==nullptr) return nullptr;
		next=tempObj->next;
		next->prev=this;
		return tempObj;
	};

	void InsertBefore(GLKObNode *p) {
		GLKObNode *oldPrevNode=prev;
		prev=p;p->next=this;
		if (oldPrevNode) {oldPrevNode->next=p;p->prev=oldPrevNode;}
	};
	GLKObNode *DeleteBefore() {
		GLKObNode *tempObj=prev;
        if (prev==nullptr) return nullptr;
		prev=tempObj->prev;
		prev->next=this;
		return tempObj;
	};

	GLKObNode *next;
	GLKObNode *prev;
	GLKObject *data;

};

#endif



#ifndef _GLKObList
#define _GLKObList

class GLKObList  
{
public:
	GLKObList();
	virtual ~GLKObList();

	GLKObject* GetHead();
	GLKObject* GetTail();
	GLKPOSITION GetHeadPosition() {return headPos;};
	GLKPOSITION GetTailPosition() {return tailPos;};
	GLKPOSITION FindIndex(int index);
	GLKPOSITION Find(GLKObject* element);

	GLKPOSITION AddHead( GLKObject* newElement );
	void AddHead( GLKObList* pNewList );
	GLKPOSITION AddTail( GLKObject* newElement );
	void AddTail( GLKObList* pNewList );
	GLKObject* RemoveHead();
	GLKObject* RemoveTail();
	GLKObject* RemoveAt(GLKPOSITION rPosition);
	void Remove(GLKObject* element);
	void RemoveAll();
	void RemoveAllWithoutFreeMemory();

	GLKObject* GetNext( GLKPOSITION& rPosition );
	GLKObject* GetPrev( GLKPOSITION& rPosition );

	GLKObject* GetAt( GLKPOSITION rPosition );

	GLKPOSITION InsertBefore( GLKPOSITION rPosition, GLKObject* newElement );
	GLKPOSITION InsertAfter( GLKPOSITION rPosition, GLKObject* newElement );

	void AttachListTail( GLKObList* pNewList );

	int GetCount() {return nCount;};

	bool IsEmpty() {return ((nCount==0)?true:false);};

private:
	GLKPOSITION headPos;
	GLKPOSITION tailPos;

	int nCount;
};

#endif




#ifndef _GLKARRAY
#define _GLKARRAY

class GLKArray  
{
public:
	GLKArray(int sx=50, int increaseStep=50, int type=0);	//	Type:	0 - (void*)
															//			1 - int
															//			2 - float 
															//			3 - double
	~GLKArray();

	int GetSize();

	void* RemoveAt(int i);
	void RemoveAll();
	
	void Add(void* data);
	void Add(int data);
	void Add(float data);
	void Add(double data);

	void SetAt(int i, void* data);
	void SetAt(int i, int data);
	void SetAt(int i, float data);
	void SetAt(int i, double data);

	void InsertAt(int i, void* data);
	void InsertAt(int i, int data);
	void InsertAt(int i, float data);
	void InsertAt(int i, double data);

	void* GetAt(int i);
	int GetIntAt(int i);
	float GetFloatAt(int i);
	double GetDoubleAt(int i);

private:
	void** listType0;
	int* listType1;
	float* listType2;
	double* listType3;

	int size;
	int arraySize;

	int step;
	int m_type;
};

#endif
