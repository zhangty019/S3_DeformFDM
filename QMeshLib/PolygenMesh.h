#ifndef POLYGENMESH_H
#define POLYGENMESH_H

#include <string>

#include "../GLKLib/GLKLib.h"
#include "../GLKLib/GLKObList.h"

#include "../QMeshLib/QMeshTetra.h"
#include "../QMeshLib/QMeshPatch.h"
#include "../QMeshLib/QMeshFace.h"
#include "../QMeshLib/QMeshEdge.h"
#include "../QMeshLib/QMeshNode.h"

typedef enum mesh_type {
    UNDEFINED,
    TET_MODEL,
    SURFACE_MESH,
    CURVED_LAYER,
    TOOL_PATH,
    CNC_PRT,
    SUPPORT_RAY,
    SUPPORT_TET_MODEL
};

class PolygenMesh : public GLKEntity
{
public:

    PolygenMesh(mesh_type type);
    ~PolygenMesh();

    mesh_type meshType;

    void ImportOBJFile(char *filename, std::string modelName);
	void ImportTETFile(char *filename, std::string modelName);

    virtual void DeleteGLList();
    virtual void BuildGLList(bool bVertexNormalShading);

    virtual void drawShade();
    virtual void drawMesh();
    virtual void drawNode();
    virtual void drawProfile();
    virtual void drawFaceNormal();
    virtual void drawNodeNormal();
    virtual float getRange() {return m_range;}

    void drawBox(float xx, float yy, float zz, float r);

    void ClearAll();
    void computeRange();
    GLKObList &GetMeshList() {return meshList;};
    void CompBoundingBox(double boundingBox[]);

    int m_materialTypeNum;
    bool m_bVertexNormalShading = true;

public:
    GLKObList meshList;
    float m_range;
    int m_drawListID;
    int m_drawListNumber;

    void _buildDrawShadeList(bool bVertexNormalShading);
    void _buildDrawMeshList();
    void _buildDrawNodeList();
    void _buildDrawProfileList();
    void _buildDrawFaceNormalList();
    void _buildDrawNodeNormalList();
//    void _buildDrawPreMeshList();

    void drawSingleEdge(QMeshEdge* edge);
    void drawSingleNode(QMeshNode* node);
    void drawSingleFace(QMeshFace* face);
    void drawSingleArrayTip(double pp[3], double dir[3], double arrowLength);
	void drawOriginalCoordinate();

    void _changeValueToColor(int nType, float & nRed, float & nGreen, float & nBlue);
    void _changeValueToColor(double maxValue, double minValue, double Value,
                                 float & nRed, float & nGreen, float & nBlue);

    void setModelName(std::string name) {modelName=name;};
    void setTransparent() {isTransparent=true;};
    void setEdgeColor() {edgeColor=true;};
    std::string getModelName() {return modelName;};

private:
    std::string modelName;
    bool isTransparent;
    bool edgeColor=false;
};

#endif // POLYGENMESH_H
