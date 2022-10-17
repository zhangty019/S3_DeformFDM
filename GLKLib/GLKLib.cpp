#include "GLKLib.h"
#include "GLKCameraTool.h"
#include "InteractiveTool.h"
#include "../QMeshLib/QMeshPatch.h"
#include "../QMeshLib/QMeshNode.h"
#include "../QMeshLib/QMeshFace.h"
#include "GLKGeometry.h"
#include <gl/glu.h>
#include <QPainter>
#include <QtDebug>
#include <QGLWidget>
#include <QApplication>

#define Qt_PAINTER true
#define BUFSIZE	2048
#define DEGREE_TO_RATATE(x)		0.0174532922222*x

GLKLib::GLKLib(QWidget *parent) : QOpenGLWidget(parent)
{
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    setFormat(format);
    initValue();
    m_currentTool=NULL;

//    createBackground();

    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);

//    m_drawPolylinePointNum = 0;
//    if (!m_drawPolylinePoints)
//        delete []m_drawPolylinePoints;
    m_drawPolylinePoints.clear();

    enterPress = false;
}

GLKLib::~GLKLib()
{
    ClearAll();
}

void GLKLib::initValue()
{
    m_xRotation = 0.0f;
    m_yRotation = 0.0f;

    m_xTranslation = 0.0f;
    m_yTranslation = 0.0f;
    m_zTranslation = 0.0f;

    m_Scaling = 1.0f;
    m_Range = 1.0f;

    m_axisDisplay = true;

    m_Shading = true;
    m_Mesh = false;
    m_Node = false;
    m_Profile = false;
    m_FaceNormal = false;
    m_NodeNormal = false;
}

void GLKLib::ClearAll()
{
    clear_tools();

    GLKPOSITION Pos;
    for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;)
    {
        GLKEntity *entity=(GLKEntity *)(m_displayObjList.GetNext(Pos));
        if (entity) {delete entity; entity=NULL;}
    }
    ClearDisplayObjList();

//    m_drawPolylinePointNum = 0;
//    if (!m_drawPolylinePoints)
//        delete []m_drawPolylinePoints;
    m_drawPolylinePoints.clear();

    initValue();
    refresh(true);
}



void GLKLib::ClearDisplayObjList()
{
    m_displayObjList.RemoveAll();
    m_Range=1.0f;
}

void GLKLib::ClearDrawLine()
{
    GLKPOSITION Pos;

    for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;)
    {
        GLKEntity *tempEntity=(GLKEntity *)(m_displayObjList.GetNext(Pos));
        if (tempEntity->entityType >= 100)
        {
            m_displayObjList.Remove(tempEntity);
        }
    }
    //refresh();
}

bool GLKLib::getViewModel(short nDir)
{
    switch(nDir){
    case 0: return(m_Shading);
    case 1: return(m_Mesh);
    case 2: return(m_Node);
    case 3: return(m_Profile);
    case 4: return(m_FaceNormal);
    case 5: return(m_NodeNormal);
    }
    return false;
}

void GLKLib::setViewModel(short nDir)
{
    switch(nDir){
    case 0: m_Shading = !m_Shading; break;
    case 1: m_Mesh = !m_Mesh; break;
    case 2: m_Node = !m_Node; break;
    case 3: m_Profile = !m_Profile; break;
    case 4: m_FaceNormal = !m_FaceNormal; break;
    case 5: m_NodeNormal = !m_NodeNormal; break;
    }
    refresh();
}

void GLKLib::setNavigation(short nDir)
{
    switch(nDir)
    {
    case 0:{	//VD_FRONTVIEW
                m_xRotation=0.0;	m_yRotation=0.0;
                refresh();
           }break;
    case 1:{	//VD_BACKVIEW
                m_xRotation=0.0;	m_yRotation=180.0;
                refresh();
           }break;
    case 2:{	//VD_TOPVIEW
                m_xRotation=90.0;	m_yRotation=0.0;
                refresh();
           }break;
    case 3:{	//VD_BOTTOMVIEW
                m_xRotation=-90.0;	m_yRotation=0.0;
                refresh();
           }break;
    case 4:{	//VD_LEFTVIEW
                m_xRotation=0.0;	m_yRotation=90.0;
                refresh();
           }break;
    case 5:{	//VD_RIGHTVIEW
                m_xRotation=0.0;	m_yRotation=-90.0;
                refresh();
           }break;
    case 6:{	//VD_ISOMETRICVIEW
                //m_xRotation=27.0;	m_yRotation=-45.0;
                m_xRotation = -40.0;	m_yRotation = -40.0;
                refresh();
           }break;
    case 7:{
                Zoom(1.1);
           }break;
    case 8:{
                Zoom(0.9);
           }break;
    case 9:{
                Zoom_All_in_View();
           }break;
    case 10:{
                if (GetCurrentTool()->tool_type == 0)
                    ((GLKCameraTool*)GetCurrentTool())->setCameraType(ZOOMWINDOW);
                else
                    GetCurrentTool()->orbit_type = 1;
           }break;
    }

}

void GLKLib::Zoom(double ratio)
{
    m_Scaling*=ratio;
    if (m_Scaling<0.00001) m_Scaling=0.00001f;

    refresh();
}

void GLKLib::Zoom_All_in_View()
{
    float newRange,oldRange=m_Range;
    m_Range=1.0f;

    GLKPOSITION Pos;
    bool flag=true;
    for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;)
    {
        GLKEntity *tempEntity=(GLKEntity *)(m_displayObjList.GetNext(Pos));
        if (!tempEntity->bShow)
            continue;
        newRange=tempEntity->getRange();
        if ((newRange>m_Range) || (flag))
        {
            m_Range=newRange;
            flag=false;
        }
    }

    m_Scaling=1.0;
    m_xTranslation = 0.0f;
    m_yTranslation = 0.0f;
    m_zTranslation = 0.0f;

    refresh();
}

void GLKLib::GLDrawAxis()
{
    double axisLength=0.07*m_Range/m_Scaling;
    double scale = m_Range/m_Scaling;
    double f = m_MappingScale;

    glPushAttrib(GL_ALL_ATTRIB_BITS);

    GLUquadricObj *quadratic = gluNewQuadric();
    gluQuadricNormals(quadratic, GLU_SMOOTH);       // Create Smooth Normals ( NEW )
    double x,y,z;
    // screen_to_wcl(50,20,x,y,z);
    double yy = m_SizeY - 50;
    double  fix[16], pMat[16];
    fix[0] = 1.0; fix[1] = 0.0; fix[2] = 0.0; fix[3] = 0.0;
    fix[4] = 0.0; fix[5] = 1.0; fix[6] = 0.0; fix[7] = 0.0;
    fix[8] = 0.0; fix[9] = 0.0; fix[10] = 1.0; fix[11] = 0.0;
    fix[12] = 0.0; fix[13] = 0.0; fix[14] = 0.0; fix[15] = 1.0;
    glGetDoublev(GL_PROJECTION_MATRIX, pMat);
    gluUnProject(50, yy, 0.5, fix, pMat, viewport, &x, &y, &z);

    glPushMatrix();
    //glTranslated(-m_Range*1.3,-m_Range*0.8,m_Range*m_Scaling*0.5);
    glTranslated(x,-y, z);
    glRotatef(m_xRotation,1.0f,0.0f,0.0f);
    glRotatef(m_yRotation,0.0f,1.0f,0.0f);
    glScalef(m_Scaling,m_Scaling,m_Scaling);

    glColor3f(1.0,0.0,0.0);         //      x-axis
    glPushMatrix();
    glRotatef(90.0, 0.0, 1.0, 0.0);
    gluCylinder(quadratic,0.01f*scale,0.01f*scale,axisLength,8,1);
    glTranslated(0.0, 0.0, axisLength);
    //gluCylinder(quadratic,0.018f*scale,0.0f,0.02f*scale,8,1);
    gluCylinder(quadratic,0.02f*scale,0.0f,0.08f*scale,8,1);
    glPopMatrix();   // Draw Our Cylinder
    glPopMatrix();
	glPushMatrix();
    // glTranslated(-m_Range*1.3,-m_Range*0.8,m_Range*m_Scaling*0.5);
    glTranslated(x,-y,z);
    glRotatef(m_xRotation,1.0f,0.0f,0.0f);
    glRotatef(m_yRotation,0.0f,1.0f,0.0f);
    glScalef(m_Scaling,m_Scaling,m_Scaling);
    glColor3f(0.0,1.0,0.0);         //      y-axis
    glPushMatrix();
    glRotatef(-90.0, 1.0, 0.0, 0.0);
    gluCylinder(quadratic,0.01f*scale,0.01f*scale,axisLength,8,1);
    glTranslated(0.0, 0.0, axisLength);
    gluCylinder(quadratic,0.02f*scale,0.0f,0.08f*scale,8,1);
    //gluCylinder(quadratic,0.018f*scale,0.0f,0.02f*scale,8,1);
    glPopMatrix();	 // Draw Our Cylinder

    glColor3f(0.0,0.0,1.0);         //      z-axis
    glPushMatrix();
    gluCylinder(quadratic,0.01f*scale,0.01f*scale,axisLength,8,1);
    glTranslated(0.0, 0.0, axisLength);
    // gluCylinder(quadratic,0.018f*scale,0.0f,0.02f*scale,8,1);
    gluCylinder(quadratic,0.02f*scale,0.0f,0.08f*scale,8,1);
    glPopMatrix();
    glPopMatrix();

    gluDeleteQuadric(quadratic);
    glPopAttrib();
}

void GLKLib::refresh(bool redrawAll)
{
    if (redrawAll){
        GLDrawDisplayObjList(redrawAll);
        //Zoom_All_in_View();
    }

    setViewport();
    update();
}

void GLKLib::setCamera()
{
    glTranslatef(m_xTranslation,m_yTranslation,m_zTranslation);
    glRotatef(m_xRotation,1.0f,0.0f,0.0f);
    glRotatef(m_yRotation, 0.0f, 1.0f, 0.0f); //glRotatef(m_yRotation,0.0f,0.0f,1.0f); 
                                           //amazing change ^-^ tianyu 20210808
    glScalef(m_Scaling,m_Scaling,m_Scaling);
}

void GLKLib::setViewport()
{
    int cx=m_SizeX;
    int cy=m_SizeY;
    float scale=m_Scaling;

    if ((m_Range*scale)<0.5) scale=0.5/m_Range;

    glViewport(0,0,cx,cy);

    glMatrixMode(GL_PROJECTION);
    //glPushMatrix();
    glLoadIdentity();
    if (cx <= cy)
    {
        glOrtho (-m_Range, m_Range, -m_Range*(GLfloat)cy/(GLfloat)cx,
            m_Range*(GLfloat)cy/(GLfloat)cx,
            -m_Range*scale, m_Range*scale);
        m_MappingScale=cx/(m_Range*2.0);
    }
    else
    {
        glOrtho (-m_Range*(GLfloat)cx/(GLfloat)cy,
            m_Range*(GLfloat)cx/(GLfloat)cy, -m_Range, m_Range,
            -m_Range*scale, m_Range*scale);
        m_MappingScale=cy/(m_Range*2.0);
    }

    glMatrixMode(GL_MODELVIEW);
    //glPushMatrix();
    glLoadIdentity();
}

void GLKLib::GLEnableLight()
{
    glShadeModel(GL_SMOOTH);
    glEnable(GL_NORMALIZE);

    glEnable(GL_DEPTH_TEST);
    glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
//    glColorMaterial(GL_FRONT, GL_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

    GLfloat	ambientProperties[]  = {0.8f, 0.8f, 0.8f, 1.0f};
//    GLfloat	ambientProperties[]  = {0.7f, 0.7f, 0.2f, 1.0f};
    GLfloat	diffuseProperties[]  = {0.8f, 0.8f, 0.8f, 1.0f};
//    GLfloat	diffuseProperties[]  = {1.0f, 1.0f, 1.0f, 0.1f};
    GLfloat	specularProperties[] = {0.0f, 0.0f, 0.0f, 0.0f};

    glLightfv( GL_LIGHT0, GL_AMBIENT, ambientProperties);
    glLightfv( GL_LIGHT0, GL_DIFFUSE, diffuseProperties);
    glLightfv( GL_LIGHT0, GL_SPECULAR, specularProperties);
#ifdef CLIPPING
    glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, 0.0);
#else
    glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, 1.0);
#endif

    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
}

void GLKLib::GLDisableLight()
{
    glDisable(GL_LIGHT0);
    glDisable(GL_LIGHTING);
}


void GLKLib::initializeGL()
 {
    glClearColor(0.45f,0.45f,0.45f,1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    GLEnableLight();
 }

void GLKLib::resizeGL(int width, int height)
 {
    // multiply 2 for retina screen
//     m_SizeX = width*2;
//     m_SizeY = height*2;
    qreal deviceRatio = devicePixelRatio();

     m_SizeX = width*deviceRatio;
     m_SizeY = height*deviceRatio;

     setViewport();
 }

void GLKLib::AddDisplayObj(GLKEntity *entity, bool bRefresh)
{
    float newRange = entity->getRange();
    float oldRange = m_Range;

    if ((newRange>m_Range) || ((m_displayObjList.GetCount())==0)) {
        m_Range=newRange;
//        m_Scaling=m_Scaling*(newRange/oldRange);
    }
    m_displayObjList.AddTail(entity);
    refresh();
}

void GLKLib::DelDisplayObj(GLKEntity *entity)
{
    float newRange,oldRange=m_Range;
    m_Range=1.0f;

    GLKPOSITION Pos;	GLKObList tempList;
    tempList.RemoveAll();
    for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;)
    {
        GLKEntity *tempEntity=(GLKEntity *)(m_displayObjList.GetNext(Pos));
        if (tempEntity!=entity) tempList.AddTail(tempEntity);
    }
    m_displayObjList.RemoveAll();	m_displayObjList.AddTail(&tempList);

    bool flag=true;
    for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;)
    {
        GLKEntity *tempEntity=(GLKEntity *)(m_displayObjList.GetNext(Pos));
        newRange=tempEntity->getRange();
        if ((newRange>m_Range) || (flag))
        {
            m_Range=newRange;
            flag=false;
        }
    }

    m_Scaling=m_Scaling*(m_Range/oldRange);
    refresh();

    delete entity;
}

void GLKLib::DelDisplayObj2(GLKEntity *entity)
{
    GLKPOSITION Pos;	GLKObList tempList;
    tempList.RemoveAll();
    for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;)
    {
        GLKEntity *tempEntity=(GLKEntity *)(m_displayObjList.GetNext(Pos));
        if (tempEntity!=entity) tempList.AddTail(tempEntity);
    }
    m_displayObjList.RemoveAll();	m_displayObjList.AddTail(&tempList);
}

void GLKLib::DelDisplayObj3(GLKEntity *entity)
{
    float newRange,oldRange=m_Range;
    m_Range=1.0f;

    GLKPOSITION Pos;	GLKObList tempList;
    tempList.RemoveAll();
    for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;)
    {
        GLKEntity *tempEntity=(GLKEntity *)(m_displayObjList.GetNext(Pos));
        if (tempEntity!=entity) tempList.AddTail(tempEntity);
    }
    m_displayObjList.RemoveAll();	m_displayObjList.AddTail(&tempList);

    bool flag=true;
    for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;)
    {
        GLKEntity *tempEntity=(GLKEntity *)(m_displayObjList.GetNext(Pos));
        newRange=tempEntity->getRange();
        if ((newRange>m_Range) || (flag))
        {
            m_Range=newRange;
            flag=false;
        }
    }

    m_Scaling=m_Scaling*(m_Range/oldRange);
    refresh();
}


void GLKLib::GLDrawDisplayObjList(bool redrawAll)
{
    GLKPOSITION Pos;

    //glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
    //glEnable(GL_DEPTH_TEST);
    glEnable(GL_POLYGON_OFFSET_FILL);
    //glPolygonOffset(0.5,0.5);
    //glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
    glEnable(GL_POLYGON_OFFSET_LINE);
    glPolygonOffset(1.0,1.0);

    if (redrawAll) {
        for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;) {
            GLKEntity *entity=(GLKEntity *)(m_displayObjList.GetNext(Pos));
            if (!entity) continue;
            if (!(entity->bShow)) continue;
            entity->DeleteGLList();
            entity->BuildGLList(entity->bVertexNormalShading);
        }
    }

    else{
        if (m_Shading)
        {
            GLEnableLight();
            for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;)
            {
                GLKEntity *entity=(GLKEntity *)(m_displayObjList.GetNext(Pos));
                if (!entity) continue;
                if (!(entity->bShow)) continue;
                entity->drawShade();
            }
            GLDisableLight();
        }
        if (m_Mesh){
            for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;)
            {
                GLKEntity *entity=(GLKEntity *)(m_displayObjList.GetNext(Pos));
                if (!entity) continue;
                if (!(entity->bShow)) continue;
                entity->drawMesh();
            }
        }
        if (m_Node){
            for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;){
                GLKEntity *entity=(GLKEntity*)(m_displayObjList.GetNext(Pos));
                if (!entity) continue;
                if (!(entity->bShow)) continue;
                entity->drawNode();
            }
        }
        if (m_Profile){
            for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;)
            {
                GLKEntity *entity=(GLKEntity *)(m_displayObjList.GetNext(Pos));
                if (!entity) continue;
                if (!(entity->bShow)) continue;
                entity->drawProfile();
            }
        }
        if (m_FaceNormal){
            for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;)
            {
                GLKEntity *entity=(GLKEntity *)(m_displayObjList.GetNext(Pos));
                if (!entity) continue;
                if (!(entity->bShow)) continue;
                entity->drawFaceNormal();
            }
        }
        if (m_NodeNormal){
            for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;)
            {
                GLKEntity *entity=(GLKEntity *)(m_displayObjList.GetNext(Pos));
                if (!entity) continue;
                if (!(entity->bShow)) continue;
                entity->drawNodeNormal();
            }
        }
    }
}

void GLKLib::clear_tools()
{
    if (m_currentTool)
        delete m_currentTool;
    m_currentTool=NULL;
}

void GLKLib::set_tool(GLKMouseTool *tool)
{
    m_currentTool=tool;
}

void GLKLib::GetViewVector(double &x, double &y, double &z)
{
    GLKGeometry geo;
    double cx,cy,cz,d;
    double xx[3],yy[3],zz[3];

    screen_to_wcl(100,100,cx,cy,cz);
    xx[0]=cx;	yy[0]=cy;	zz[0]=cz;
    screen_to_wcl(200,200,cx,cy,cz);
    xx[1]=cx;	yy[1]=cy;	zz[1]=cz;
    screen_to_wcl(200,100,cx,cy,cz);
    xx[2]=cx;	yy[2]=cy;	zz[2]=cz;
    geo.CalPlaneEquation(x,y,z,d,xx,yy,zz);
}

void GLKLib::screen_to_wcl(double sx, double sy, double &cx, double &cy, double &cz)
{
    GLdouble objx, objy, objz;
    double y = m_SizeY - sy;
    gluUnProject(sx, y, 0.5, modelMatrix, projMatrix, viewport, &objx, &objy, &objz);

    cx=objx;	cy=objy;	cz=objz;
}

void GLKLib::screen_to_wcl(double sx, double sy, double sz, double &cx, double &cy, double &cz)
{
    GLdouble objx, objy, objz;
    double y = m_SizeY - sy;
    gluUnProject(sx, y, sz, modelMatrix, projMatrix, viewport, &objx, &objy, &objz);

    cx=objx;	cy=objy;	cz=objz;
}

void GLKLib::wcl_to_screen(double cx, double cy, double cz, double &sx, double &sy)
{
    GLdouble winx, winy, winz;

    gluProject(cx, cy, cz, modelMatrix, projMatrix, viewport, &winx, &winy, &winz);

    qreal deviceRatio = devicePixelRatio();
    sx=winx/(double)deviceRatio;
    sy=(m_SizeY-winy)/(double)deviceRatio;
}

void GLKLib::wcl_to_screen(double cx, double cy, double cz, double &sx, double &sy, double &sz)
{
    GLdouble winx, winy, winz;

    gluProject(cx, cy, cz, modelMatrix, projMatrix, viewport, &winx, &winy, &winz);

    sx=winx;
    sy=m_SizeY-winy;
    sz=winz;
}

void GLKLib::mousePressEvent(QMouseEvent *event)
{
    if (m_currentTool)
        m_currentTool->process_event(event,MOUSE_PRESS);
    lastPos = event->pos();
}

 void GLKLib::mouseMoveEvent(QMouseEvent *event)
 {
     if (m_currentTool)
        m_currentTool->process_event(event,MOUSE_MOVE);
     lastPos = event->pos();
 }

 void GLKLib::mouseReleaseEvent(QMouseEvent *event)
 {
    currPos = event->pos();
    if (m_currentTool)
        m_currentTool->process_event(event,MOUSE_RELEASE);
 }

 void GLKLib::keyReleaseEvent(QKeyEvent *e)
 {
     if (m_currentTool)
        m_currentTool->process_event(e,KEY_RELEASE);
 }

void GLKLib::keyPressEvent(QKeyEvent *e)
{
    if (m_currentTool)
        m_currentTool->process_event(e,KEY_PRESS);
    if (e->key() == Qt::Key_Enter || e->key() == Qt::Key_Enter)
        enterPress = true;
}

void GLKLib::wheelEvent(QWheelEvent * event )
{
    if (m_currentTool)
        m_currentTool->process_event(event,MOUSE_WHEELE_MOVE);
}

void GLKLib::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    drawBackground(&painter);

    painter.beginNativePainting();
    drawOpenGL();
    painter.endNativePainting();

    if (((GLKCameraTool*)GetCurrentTool())->getCameraType() == ZOOMWINDOW && !m_drawPolylinePoints.isEmpty())
        draw_polyline_2d(true);
    if (GetCurrentTool()->tool_type == 1 && m_drawPolylinePoints.size() > 1)
        draw_polyline_2d(false);
    if (GetCurrentTool()->tool_type == 1 && m_drawLine.size() > 0)
        draw_line_2d();
}

void GLKLib::draw_polyline_2d(bool bClear)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(Qt::black,1));

    QPainterPath path;
    path.moveTo(m_drawPolylinePoints.at(0).x(), m_drawPolylinePoints.at(0).y());
    for(int i=0;i<m_drawPolylinePoints.size();i++) {
        path.lineTo(m_drawPolylinePoints[i].x(), m_drawPolylinePoints[i].y());
        path.moveTo(m_drawPolylinePoints[i].x(), m_drawPolylinePoints[i].y());
    }
    painter.drawPath(path);
    if (bClear)
        m_drawPolylinePoints.clear();
}

void GLKLib::draw_line_2d()
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(Qt::black,1));

    painter.drawLine(m_drawLine.at(0),m_drawLine.at(1));
    m_drawLine.clear();
}

void GLKLib::draw_polyline_2d(int pointNum, const float pts[], bool bFill)
{               
    QPainter painter(this);
    drawBackground(&painter);

    painter.beginNativePainting();
    drawOpenGL();
    painter.endNativePainting();

    QPainterPath path;
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(Qt::black,2));

    path.moveTo(pts[0], pts[1]);
    for(int i=0;i<pointNum;i++) {
        path.lineTo(pts[i*2], pts[i*2+1]);
        path.moveTo(pts[i*2], pts[i*2+1]);
    }
    painter.drawPath(path);
}

void GLKLib::drawOpenGL()
{
    makeCurrent();
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glClear(GL_DEPTH_BUFFER_BIT);
    setViewport();
    GLEnableLight();

    if (m_axisDisplay){
        GLDisableLight();
        GLDrawAxis();
    }

    setCamera();

    //Draw Display Object
    GLDrawDisplayObjList(false);

    glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
    glGetIntegerv(GL_VIEWPORT, viewport);

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
}

void GLKLib::createBackground()
{
    bgGradient.setCoordinateMode(QGradient::ObjectBoundingMode);
    bgGradient.setCenter(0.45,0.5);
    bgGradient.setFocalPoint(0.4,0.45);
    bgGradient.setColorAt(0.0,QColor(105,146,182));
    bgGradient.setColorAt(0.4,QColor(81,113,150));
    bgGradient.setColorAt(0.8,QColor(18,52,86));
}

void GLKLib::drawBackground(QPainter *painter)
{
    painter->setPen(Qt::NoPen);
    painter->setBrush(bgGradient);
    painter->drawRect(rect());
}

GLubyte* GLKLib::GetColorImage(QMeshPatch *Patch, int w, int h, double *range, double *centroid)
{
    GLubyte *pixelColor = new GLubyte[w*h*3*sizeof(GLubyte)];
//	CDC *pDC1 = m_pWnd->GetWindowDC();
//	if (!pDC1) return pixelColor;
//	wglMakeCurrent(pDC1->m_hDC,m_hRC);
    makeCurrent();

    glDisable(GL_STENCIL_TEST);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    glViewport(0,0,w,h);
//    glViewport(0,0,m_SizeX,m_SizeY);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    float rx, ry;
    GetRotation(rx,ry);

    double boundingBox[8][3];
    int m = 0;
    for (int i=0; i<2; i++){
        for (int j=2; j<4; j++){
            for (int k=4; k<6; k++){
                boundingBox[m][0] = range[i]-centroid[0];
                boundingBox[m][1] = range[j]-centroid[1];
                boundingBox[m][2] = range[k]-centroid[2];
                m++;
            }
        }
    }

    double xmin, ymin, zmin, xmax, ymax, zmax;
    xmin=1.0e+32;	ymin=1.0e+32;	zmin=1.0e+32;
    xmax=-1.0e+32;	ymax=-1.0e+32;	zmax=-1.0e+32;
    double csy,sny,csx,snx;
    double ratio=3.1451592654/180.0;
    double xx,yy,zz,x1,y1,z1,x2,y2,z2;
    csy=cos(ry*ratio);		sny=sin(ry*ratio);
    csx=cos(rx*ratio);		snx=sin(rx*ratio);
    for (int i=0; i<8; i++){
        xx=boundingBox[i][0];
        yy=boundingBox[i][1];
        zz=boundingBox[i][2];
        x1=zz*sny+xx*csy;
        y1=yy;
        z1=zz*csy-xx*sny;
        x2=x1;
        y2=y1*csx-z1*snx;
        z2=y1*snx+z1*csx;
        if (x2 > xmax) xmax = x2;
        if (y2 > ymax) ymax = y2;
        if (z2 > zmax) zmax = z2;
        if (x2 < xmin) xmin = x2;
        if (y2 < ymin) ymin = y2;
        if (z2 < zmin) zmin = z2;
    }
    double r = 0.15;
    xmin -= r*(xmax-xmin); xmax += r*(xmax-xmin);
    ymin -= r*(ymax-ymin); ymax += r*(ymax-ymin);
    zmin -= r*(zmax-zmin); zmax += r*(zmax-zmin);

    //glOrtho(xmin,xmax,ymin,ymax,zmin,zmax);
    //glOrtho(xmin,xmax,ymin,ymax,-175,70);
    glOrtho(xmin,xmax,ymin,ymax,-zmax,-zmin);
    //glOrtho(-xmax,-xmin,-ymax,-ymin,-zmax,-zmin);
    //printf("%.5lf %.5lf %.5lf %.5lf %.5lf %.5lf ",xmin,xmax,ymin,ymax,zmax,zmin);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glClearDepth(1.0);

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glClearColor(1.0,1.0,1.0,1.0);

    glDepthFunc(GL_LESS);
    glPushMatrix();

    glRotatef(rx,1,0,0);
    glRotatef(ry,0,1,0);
    glTranslated(-centroid[0],-centroid[1],-centroid[2]);

    GLfloat rr, gg, bb;
    rr=0.0; gg=0.0; bb=-1.0;
    int index = 0;
    for (GLKPOSITION Pos=Patch->GetFaceList().GetHeadPosition(); Pos!=NULL;)
    {
        QMeshFace *temp=(QMeshFace *)(Patch->GetFaceList().GetNext(Pos));
        int i = temp->GetEdgeNum();
        glBegin(GL_POLYGON);
        const int max_edge_num = 10;
        QMeshNode *node[max_edge_num];
        int in = temp->GetIndexNo();
        bb += 1.0;
        if (bb > 255.0){
            gg+=1.0; bb=0.0;
            if (gg > 255.0){
                rr += 1.0; gg=0.0; bb=0.0;
            }
        }
        if (rr>255.0 && gg>255.0 && bb>255.0){
            printf("Color Index out of Memory");
            return pixelColor;
        }
        glColor3f(rr/255.0,gg/255.0,bb/255.0);
        for(int j = 0; j < i; j++){
            double x,y,z;
            node[j]=temp->GetNodeRecordPtr(j);
            node[j]->GetCoord3D(x,y,z);
            glVertex3d(x,y,z);
        }
        glEnd();
    }
    glReadPixels(0,0,w,h,GL_RGB,GL_UNSIGNED_BYTE, pixelColor);
//    glReadPixels(0,0,m_SizeX,m_SizeY,GL_RGB,GL_UNSIGNED_BYTE,pixelColor);

    glPopMatrix();
    glDisable(GL_DEPTH_TEST);
//	SwapBuffers(pDC1->m_hDC);
//	wglMakeCurrent(NULL, NULL);
//    update();
    update();
//    doneCurrent();

    return pixelColor;
}

void GLKLib::GLPickInit()
{
    int cx=m_SizeX;
    int cy=m_SizeY;
    float scale=m_Scaling;

    if ((m_Range*scale)<0.5) scale=0.5/m_Range;
    if (cx <= cy)
    {
        glOrtho (-m_Range, m_Range, -m_Range*(GLfloat)cy/(GLfloat)cx,
            m_Range*(GLfloat)cy/(GLfloat)cx, -m_Range*scale, m_Range*scale);
        m_MappingScale=cx/(m_Range*2.0);
    }
    else
    {
        glOrtho (-m_Range*(GLfloat)cx/(GLfloat)cy,
            m_Range*(GLfloat)cx/(GLfloat)cy, -m_Range, m_Range, -m_Range*scale, m_Range*scale);
        m_MappingScale=cy/(m_Range*2.0);
    }
}

void GLKLib::GLPickDraw(unsigned int nType)
{
    glClear(GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    glInitNames();
    glPushName(0);

    glTranslated(m_xTranslation,m_yTranslation,m_zTranslation);
    double ca=cos(DEGREE_TO_RATATE(m_yRotation));
    double sa=sin(DEGREE_TO_RATATE(m_yRotation));
    double cb=cos(DEGREE_TO_RATATE(m_xRotation));
    double sb=sin(DEGREE_TO_RATATE(m_xRotation));
    GLdouble R[16]={
        ca,sa*sb,-sa*cb,0,
        0,cb,sb,0,
        sa,-ca*sb,ca*cb,0,
        0,0,0,1
    };
    glMultMatrixd(R);
    glScalef(m_Scaling,m_Scaling,m_Scaling);

    GLDisableLight();
    ////////////////////////////////////////////////////////////////
    //	The following lines are drawing Object.
    //		Default rendering
    glColor3f(0.7f,0.7f,0.7f);
    GLPickDrawDisplayObjList(nType);

    glPopMatrix();
    glFlush();
}

void GLKLib::GLPickDrawDisplayObjList(unsigned int nType)
{
    unsigned int nameIndex=1;
    GLKPOSITION Pos;
    for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;)
    {
        GLKEntity *entity=(GLKEntity *)(m_displayObjList.GetNext(Pos));

        if (!entity) continue;
        if ((entity->bShow) && (entity->entityType==nType))
        {
            glPushMatrix();
            glLoadName(nameIndex++);
            if (m_Shading)
            {
                GLEnableLight();
                entity->drawShade();
                GLDisableLight();
            }
            if (m_Node)
                entity->drawNode();
            if (m_Mesh)
                entity->drawMesh();
            if (m_Profile)
                entity->drawProfile();
            glPopMatrix();
        }
    }
}

GLKEntity* GLKLib::GetPickObjByName(unsigned int nType,unsigned int name)
{
    unsigned int nameIndex=1;

    GLKPOSITION Pos;
    for(Pos=m_displayObjList.GetHeadPosition();Pos!=NULL;)
    {
        GLKEntity *entity=(GLKEntity *)(m_displayObjList.GetNext(Pos));

        if (!entity) continue;
        if ((entity->bShow) && (entity->entityType==nType))
        {
            if (name==nameIndex) return entity;
            nameIndex++;
        }
    }

    return NULL;
}

bool GLKLib::pick_entity(QMouseEvent *event, unsigned int entityType, GLKEntity*& ent)
{
    GLuint selectBuf[BUFSIZE];
    GLint viewport[4];
    GLint hits;
    QPoint pt;

    makeCurrent();
    pt = event->pos();
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glSelectBuffer(BUFSIZE, selectBuf);
    glGetIntegerv(GL_VIEWPORT, viewport);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glRenderMode(GL_SELECT);
    glLoadIdentity();

    gluPickMatrix((GLdouble)pt.x(),(GLdouble)(viewport[3]-pt.y()),10.0,10.0,viewport);
    GLPickInit();
    GLPickDraw(entityType);
    hits=glRenderMode(GL_RENDER);

    qDebug() << hits;


    GLuint *ptr=(GLuint*)selectBuf;
    GLuint names;
    unsigned int currentName=0,z1,z2,zmin,selectName,selectZ;

    for(int i=0;i<hits;i++)
    {
        names=*ptr;		ptr++;
        z1=*ptr;		ptr++;
        z2=*ptr;		ptr++;
        for(unsigned int j=0;j<names;j++)
        {
            currentName=*ptr;	ptr++;
        }
        if (z1<z2)
            zmin=z1;
        else
            zmin=z2;

        if (i==0)
        {
            selectName=currentName;
            selectZ=zmin;
        }
        else
        {
            if (zmin<selectZ)
            {
                selectName=currentName;
                selectZ=zmin;
            }
        }
    }

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopAttrib();

    if (currentName==0) return false;

    ent=GetPickObjByName(entityType,selectName);
    if (!ent) return false;

    return true;
}
