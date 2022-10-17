#include "GLKLib.h"
#include <QEvent>
#include <QMouseEvent>
#include <QPoint>
#include <math.h>
#ifndef _GLKCAMERATOOL
#define _GLKCAMERATOOL

typedef enum camera_type {ORBIT,PAN,ZOOM,ORBITPAN,ZOOMWINDOW};

class GLKCameraTool : public GLKMouseTool
{
public:
    GLKCameraTool(GLKLib *cView, camera_type ct)
    {
        pView=cView;
        m_ct=ct;
        tool_type = 0;
    }

    virtual ~GLKCameraTool() {};
    void setCameraType(camera_type ct) {m_ct = ct;};
    camera_type getCameraType() {return m_ct;}

private:
    GLKLib *pView;
    camera_type m_ct;
    QPoint lastPos;
    QPoint pt;
public:
    // Implement the virtual method which processes the button events
    // The default implementation maps the pick_event into a position
    // and then calls the process_position_event method
    virtual int process_event(QEvent *event, mouse_event_type event_type) {
        switch(m_ct) {
        case ORBITPAN	:{
            if (event_type == MOUSE_WHEELE_MOVE) {
                m_ct = ZOOM; break;}

            QMouseEvent *e = (QMouseEvent*)event;

            if (event_type == MOUSE_PRESS)
                lastPos = e->pos();
            if (event_type == MOUSE_MOVE && (e->buttons() & Qt::LeftButton))
            {
                float xR,yR;

                pView->GetRotation(xR,yR);

                double sx,sy;
                double cx,cy;
                pView->wcl_to_screen(0.0,0.0,0.0,sx,sy);
                pView->wcl_to_screen(0.0,1.0,0.0,cx,cy);
                if (cy>=sy)
                    yR += (float)(lastPos.x() - e->pos().x())/2;
                else
                    yR -= (float)(lastPos.x() - e->pos().x())/2;

                xR -= (float)(lastPos.y() - e->pos().y())/2;

                // printf("xR: %f yR: %f", xR, yR);

                pView->SetRotation(xR,yR);
                lastPos = e->pos();
                pView->refresh();
            }
            if (event_type == MOUSE_MOVE && (e->buttons() & Qt::RightButton))
            {
                float xR,yR,zR;
                float mappingScale=pView->m_MappingScale;

                pView->GetTranslation(xR,yR,zR);
                xR -= (float)(lastPos.x() - e->pos().x())/mappingScale;
                yR += (float)(lastPos.y() - e->pos().y())/mappingScale;
                pView->SetTranslation(xR,yR,zR);
                lastPos = e->pos();

                pView->refresh();
            }
            QKeyEvent *keyEvent = (QKeyEvent*)event;
            if (event_type == KEY_PRESS){
                float xR, yR;
                if (keyEvent->key() == Qt::Key_Left){
                    pView->GetRotation(xR, yR);
                    pView->SetRotation(xR, yR-5);
                }
                if (keyEvent->key() == Qt::Key_Right){
                    pView->GetRotation(xR, yR);
                    pView->SetRotation(xR, yR+5);
                }
                if (keyEvent->key() == Qt::Key_Up){
                    pView->GetRotation(xR, yR);
                    pView->SetRotation(xR-5, yR);
                }
                if (keyEvent->key() == Qt::Key_Down){
                    pView->GetRotation(xR, yR);
                    pView->SetRotation(xR+5, yR);
                }
                pView->refresh();
            }

        }break;
        case ORBIT		:{
            QMouseEvent *e = (QMouseEvent*)event;
            if (event_type == MOUSE_PRESS)
                lastPos = e->pos();
            if (event_type == MOUSE_MOVE && (e->buttons() & Qt::LeftButton))
            {
                float xR,yR;

                pView->GetRotation(xR,yR);

                double sx,sy;
                double cx,cy;
                pView->wcl_to_screen(0.0,0.0,0.0,sx,sy);
                pView->wcl_to_screen(0.0,1.0,0.0,cx,cy);
                if (cy>=sy)
                    yR += (float)(lastPos.x() - e->pos().x())/2;
                else
                    yR -= (float)(lastPos.x() - e->pos().x())/2;

                xR -= (float)(lastPos.y() - e->pos().y())/2;

                pView->SetRotation(xR,yR);
                lastPos = e->pos();
                pView->refresh();
            }
        }break;
        case PAN		:{
            QMouseEvent *e = (QMouseEvent*)event;
            if (event_type == MOUSE_PRESS)
                lastPos = e->pos();
            if (event_type == MOUSE_MOVE && (e->buttons() & Qt::RightButton))
            {
                float xR,yR,zR;
                float mappingScale=pView->m_MappingScale;

                pView->GetTranslation(xR,yR,zR);
                xR -= (float)(lastPos.x() - e->pos().x())/mappingScale;
                yR += (float)(lastPos.y() - e->pos().y())/mappingScale;
                pView->SetTranslation(xR,yR,zR);
                lastPos = e->pos();

                pView->refresh();
            }
        }break;
        case ZOOM		:{	if (event_type != MOUSE_WHEELE_MOVE) { m_ct = ORBITPAN; lastPos = ((QMouseEvent*)event)->pos();	break;}
            QWheelEvent *e = (QWheelEvent*)event;
            if (e->delta() >= 0.0)
                pView->Zoom(1.15);
            else
                pView->Zoom(0.85);
            pView->refresh();
        }break;
        case ZOOMWINDOW :{
            QMouseEvent *e = (QMouseEvent*)event;

            if (event_type == MOUSE_PRESS)
            {	lastPos = e->pos();	pt = e->pos();	}
            if (event_type == MOUSE_MOVE && (e->buttons() & Qt::LeftButton))
            {
                float pnts[10];
                pnts[0]=(float)lastPos.x();	pnts[1]=(float)lastPos.y();;
                pnts[2]=(float)pt.x();		pnts[3]=(float)lastPos.y();
                pnts[4]=(float)pt.x();		pnts[5]=(float)pt.y();
                pnts[6]=(float)lastPos.x();	pnts[7]=(float)pt.y();
                pnts[8]=(float)lastPos.x();	pnts[9]=(float)lastPos.y();

//                pView->draw_polyline_2d(5,pnts,false);
//                double xx, yy, zz;
//                glBegin(GL_LINE_STRIP);
//                glColor3f(0.75,0.75,0.75);
//                glLineWidth(10.0);
//                for(int i=0;i<10;i++) {
//                    pView->screen_to_wcl(pnts[i*2],pnts[i*2+1],xx,yy,zz);
//                    glVertex3d(xx,yy,zz);
//                }
//                glEnd();

//                pView->m_drawPolylinePointNum = 5;
//                pView->m_drawPolylinePoints = new float[pView->m_drawPolylinePointNum*2];
//                for (int i=0; i<pView->m_drawPolylinePointNum*2; i++)
//                    pView->m_drawPolylinePoints[i] = pnts[i];

                for (int i=0; i<5; i++)
                    pView->m_drawPolylinePoints << QPoint(pnts[2*i], pnts[2*i+1]);

                pt = e->pos();
                pView->update();
            }
            if (event_type == MOUSE_RELEASE)
            {
                double cx,cy,xx,yy;	int sx,sy;
                QPoint newpt;

                float xR,yR,zR;	float scale,sc;
                newpt = e->pos();
                cx = fabs(double(lastPos.x() - newpt.x()));		cy = fabs(double(lastPos.y() - newpt.y()));
                if ((cx>0) && (cy>0))
                {
                    pView->GetSize(sx,sy);
                    scale=(float)(sx/cx);		sc=(float)(sy/cy);
                    if (sc<scale) scale=sc;
                    pView->GetScale(sc);	sc=sc*scale;
                    if (sc<0.00001) sc=0.00001f;
                    //if (sc > 500000.0) sc=500000.0;
                    pView->SetScale(sc);
                    //qDebug("scale %d %d %f %f %f",sx,sy,cx,cy,sc);
                    float mappingScale=pView->m_MappingScale;

                    cx = (lastPos.x() + newpt.x())/2.0;		cy = (lastPos.y() + newpt.y())/2.0;
                    pView->GetTranslation(xR,yR,zR);
                    pView->wcl_to_screen(0.0,0.0,0.0,xx,yy);
                    xR -= (float)((cx-xx)*scale+xx-sx/2.0f)/mappingScale;
                    yR += (float)((cy-yy)*scale+yy-sy/2.0f)/mappingScale;
                    pView->SetTranslation(xR,yR,zR);

                    pView->ReShape(sx,sy);
                    pView->refresh();
                    m_ct=ORBITPAN;

//                    pView->m_drawPolylinePointNum = 0;
//                    delete []pView->m_drawPolylinePoints;
                    pView->m_drawPolylinePoints.clear();

                }
            }
        }break;
        }
        return 0;
    }
};


#endif

