#ifndef INTERACTIVETOOL_H
#define INTERACTIVETOOL_H

#include "GLKLib.h"
#include <QPoint>

enum select_type { NODE, EDGE, FACE, FIX, NHANDLE };

class InteractiveTool : public GLKMouseTool
{
public:
    InteractiveTool(GLKLib *pGLKLib, QMeshPatch *meshPatch_,  GLKMouseTool *mouseTool, select_type selectType, bool Select);
    InteractiveTool(GLKLib *pGLKLib, GLKObList *polygenMeshList,  GLKMouseTool *mouseTool, select_type selectType, bool Select);
    virtual ~InteractiveTool();

    virtual int process_event(QEvent *event, mouse_event_type even_type);

    select_type getSelectedType() {return type;};

private:
    GLKLib *pGLK;
    QMeshPatch *meshPatch;
    GLKObList *polygenList;
    select_type type;
    GLKMouseTool *mouse_tool;

    QPoint lastPos;
    QPoint pos;

	bool isSelect = true;

    bool bAltPressed;
    bool bShiftPressed;

    void _selectNodes(QEvent *event, mouse_event_type event_type);
    void _selectEdges(QEvent *event, mouse_event_type event_type);
    void _selectFaces(QEvent *event, mouse_event_type event_type);
	void _selectFixed(QEvent *event, mouse_event_type event_type);
	void _selectHandle(QEvent *event, mouse_event_type event_type);


    void GetVisibleMesh(QMeshPatch *patch);
};

//#include "GLKLib.h"

//class QMeshPatch;
//class QMeshNode;
//class QMeshFace;
//class InteractiveEntity;

//typedef enum select_type {NODE, FACE, NODECLUSTER, FACECLUSTER};

//class InteractiveTool : public GLKMouseTool
//{
//public:
//    InteractiveTool(GLKLib *pGLKLib, QMeshPatch *patch, GLKMouseTool *cameraTool, select_type selectType);
//    virtual ~InteractiveTool();

//    virtual int process_event(QEvent *event, mouse_event_type event_type);

//    GLKLib *pGLK;
//    QMeshPatch *m_patch;

//private:
//    GLKMouseTool *tool;
//    select_type type;

//    void _selectNode(QEvent *event, mouse_event_type event_type);
//    void _selectFace(QEvent *event, mouse_event_type event_type);

//    int m_totalNodeNum;
//    InteractiveEntity **m_node;
//    void _addNodeToDrawList();
//    void _deleteNodeFromDrawList();

//    QMeshFace* CalProjectedTrgl(double xx, double yy, double zz,
//               double nx, double ny, double nz, double& t,double& u,double& v,double& w);
//};

#endif // INTERACTIVETOOL_H

