#include <iostream>
#include <../ThirdPartyDependence/glut/glew.h>
#include <../ThirdPartyDependence/glut/GL.h>

//#pragma comment(lib, "glew32.lib")

#include <QOpenGLWidget>
#include <QVector>
#include <QPoint>
#include "GLKObList.h"

class GLKEntity;
class QMeshPatch;
class QMeshNode;
class GLKMouseTool;

#ifndef GLK_H
#define GLK_H

class GLKLib : public QOpenGLWidget
{
    Q_OBJECT
public:
    GLKLib(QWidget* parent = 0);
    ~GLKLib();

     void refresh(bool redrawAll=false);
     void AddDisplayObj(GLKEntity *entity, bool bRefresh=false);
     void DelDisplayObj(GLKEntity *entity);
     void DelDisplayObj2(GLKEntity *entity);
     void DelDisplayObj3(GLKEntity *entity);
     void ClearAll();
     void ClearDisplayObjList();
     void ClearDrawLine();
public:
    void Zoom_All_in_View();
    void Zoom(double ratio);
    void ZoomWindow();
    void GetSize(int &sx,int &sy) {sx=m_SizeX;sy=m_SizeY;};
    void setNavigation(short nDir);
    void setViewModel(short nDir);
    bool getViewModel(short nDir);

    void GetViewVector(double &x, double &y, double &z);
    void wcl_to_screen(double cx, double cy, double cz, double &sx, double &sy);
    void wcl_to_screen(double cx, double cy, double cz, double &sx, double &sy, double &sz);
    void screen_to_wcl(double sx, double sy, double &cx, double &cy, double &cz);
    void screen_to_wcl(double sx, double sy, double sz, double &cx, double &cy, double &cz);

    void GetRotation(float &rx, float &ry) 		{rx=m_xRotation;ry=m_yRotation;};
    void SetRotation(float rx, float ry) 		{m_xRotation=rx;m_yRotation=ry;};
    void GetTranslation(float &rx, float &ry, float &rz)  {rx=m_xTranslation;ry=m_yTranslation;rz=m_zTranslation;};
    void SetTranslation(float rx, float ry, float rz) 	{m_xTranslation=rx;m_yTranslation=ry;m_zTranslation=rz;};
    void GetScale(float &scale) {scale=m_Scaling;};
    void SetScale(float scale) {m_Scaling=scale;};
    void ReShape(int width, int height) {resizeGL(width,height);};
    void clear_tools();
    void set_tool(GLKMouseTool *tool);
    GLKMouseTool* GetCurrentTool() {return m_currentTool;};

    void draw_polyline_2d(int pointNum, const float pts[], bool bFill);
    bool pick_entity(QMouseEvent *event, unsigned int entityType, GLKEntity*& ent);
    void GLPickInit();
    void GLPickDraw(unsigned int nType);
    void GLPickDrawDisplayObjList(unsigned int nType);
    GLKEntity* GetPickObjByName(unsigned int nType,unsigned int name);
    GLKObList &GetDisplayList() {return m_displayObjList;};

    QVector<QPoint> m_drawPolylinePoints;
    void draw_polyline_2d(bool bClear);
    QVector<QPoint> m_drawLine;
    void draw_line_2d();

    GLubyte* GetColorImage(QMeshPatch *Patch, int w, int h, double *range, double *centroid);

public:
    float m_MappingScale;
    bool enterPress;
private:
    float m_xRotation;
    float m_yRotation;
    float m_zRotation;
    float m_xTranslation;
    float m_yTranslation;
    float m_zTranslation;
    float m_Scaling;
    float m_Range;
    int m_SizeX;
    int m_SizeY;
    QPoint lastPos;
    QPoint currPos;

    bool m_axisDisplay;
    bool m_Shading;
    bool m_Mesh;
    bool m_Node;
    bool m_Profile;
    bool m_FaceNormal;
    bool m_NodeNormal;

    GLdouble modelMatrix[16];
    GLdouble projMatrix[16];
    GLint viewport[4];

    GLKObList m_displayObjList;
    GLKMouseTool *m_currentTool;

protected:

     void mousePressEvent(QMouseEvent *event);
     void mouseMoveEvent(QMouseEvent *event);
     void wheelEvent(QWheelEvent *event);
     void mouseReleaseEvent(QMouseEvent *event);
     void keyPressEvent(QKeyEvent *event);
     void keyReleaseEvent(QKeyEvent *event);

     //Purely OpenGL
     void initializeGL();
     void resizeGL(int width, int height);

     //OpenGL(3D) + QPainter(2D)
     void paintEvent(QPaintEvent *event);
     void createBackground();
     void drawBackground(QPainter *painter);
     void drawOpenGL();

     QRadialGradient bgGradient;


private:
    void initValue();
    void setViewport();
    void setCamera();
    void GLEnableLight();
    void GLDisableLight();
    void GLDrawAxis();
    void GLDrawDisplayObjList(bool redrawAll);
};

#endif // GLK_H


#ifndef _GLKENTITY
#define _GLKENTITY

class GLKEntity : public GLKObject
{
public:
    GLKEntity() {bShow=true; entityType=0;};
    virtual ~GLKEntity() {};

    // Implement the virtual method which draw this entity
    //		TRUE - draw the shading mode
    //		FALSE - draw the mesh mode
    virtual void drawShade() {};
    virtual void drawMesh() {};
    virtual void drawNode() {};
    virtual void drawProfile() {};
    virtual void drawFaceNormal() {};
    virtual void drawNodeNormal() {};

    virtual void DeleteGLList() {};
    virtual void BuildGLList(bool bVertexNormalShading) {};
    // Implement the maximum distance to the original point of this entity
    virtual float getRange() {return 0.0;};

    bool bShow;
//    bool bHighLight;
    short entityType;
    bool bVertexNormalShading = true;
//    bool m_red,m_blue,m_green,m_GuassianCuv,m_Convex,m_Homogeneity,m_AspectRatio,m_IsoError;;
//protected:
//    float red, green, blue;
};

#endif


#ifndef _GLKMOUSETOOL
#define _GLKMOUSETOOL

typedef enum mouse_event_type {
    MOUSE_PRESS, MOUSE_RELEASE, MOUSE_MOVE, KEY_PRESS, KEY_RELEASE, MOUSE_WHEELE_MOVE
}mouse_event_type;

/////////////////////////////////////////////////////////////////////////////
//
//	The following definition are for the "nFlag " in the pick_event. (Defined by MFC)
//
//		MK_CONTROL   //Set if the CTRL key is down.
//		MK_LBUTTON   //Set if the left mouse button is down.
//		MK_MBUTTON   //Set if the middle mouse button is down.
//		MK_RBUTTON   //Set if the right mouse button is down.
//		MK_SHIFT	 //Set if the SHIFT key is down.

struct pick_event{
    double x,y;
    short nFlags;
    short nChar;	// if its value is negative, the key-in is by the special key func.
};

class GLKMouseTool : public GLKObject
{

public:
    GLKMouseTool() {orbit_type = 0;};
    virtual ~GLKMouseTool() {};

public:
    // Implement the virtual method which processes the button events
    // The default implementation maps the pick_event into a position
    // and then calls the process_position_event method
    //virtual int process_event(mouse_event_type even_type, const pick_event& pe) {return 0;};
    virtual int process_event(QEvent *event, mouse_event_type even_type) {return 0;};
    unsigned short tool_type; // 0 -- camera ; 1 -- interactiveTool
    unsigned short  orbit_type;

};

#endif
