// GLKGLList.h: interface for the GLKGLList class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _GLKGLLIST
#define _GLKGLLIST

#include "GLKLib.h"
#include "GLKObList.h"

class GLKGLList : public GLKObject  
{
public:
	GLKGLList() {};
	virtual ~GLKGLList() {};
    virtual void draw(GLKLib *view) {};
};

#endif 
