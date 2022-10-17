// GLKHeap.h: interface for the GLKHeap class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _GLKHEAPNODE
#define _GLKHEAPNODE

#include "GLKObList.h"

class GLKHeapNode : public GLKObject
{
public:
	GLKHeapNode() {index=0;};
	virtual ~GLKHeapNode() {};
	float GetValue() {return v;};
	void SetValue(float value) {v=value;};

	int index;	//	this is the index for locating HeapNode in a heap
	void* attachedObj; // who updates my weight

protected:
	float v; // weight
};

#endif


#ifndef _GLKHEAP
#define _GLKHEAP

class GLKHeap  
{
public:
	GLKHeap(int maxsize, bool minOrMax=true);	//	true - min Heap
												//	false - max Heap
	GLKHeap(GLKHeapNode** arr, int n, bool minOrMax=true);
	virtual ~GLKHeap();

	const GLKHeapNode* operator[] (int i);

	int ListSize();
	bool ListEmpty();
	bool ListFull();

	void SetKetOnMinOrMax(bool flag);
	bool IsKeyOnMinOrMax();		//	true	- Keyed on min value
								//	false	- Keyed in max value
	
	bool Insert(GLKHeapNode* item);
	GLKHeapNode* RemoveTop();
	GLKHeapNode* GetTop();
	void AdjustPosition(GLKHeapNode* item);
	void Remove(GLKHeapNode* item);
	void ClearList();

private:
	bool bMinMax;	//	true	- Keyed on min value
					//	false	- Keyed in max value

	// hlist points at the array which can be allocated by the constructor (inArray == 0)
	//	or passed as a parameter (inArray == 1)
	GLKHeapNode** hlist;

	// amx elements allowed and current size of heap
	int maxheapsize;
	int heapsize;		// identifies end of list

	// utility functions for Delete/Insert to restore heap
	void FilterDown(int i);
	void FilterUp(int i);

	void Expand();
};

#endif
