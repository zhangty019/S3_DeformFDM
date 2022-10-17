#ifndef _CCL_POLYMESH_BODY
#define _CCL_POLYMESH_BODY

#include "../GLKLib/GLKLib.h"
#include "../GLKLib/GLKObList.h"
#include "../QMeshLib/QMeshPatch.h"

#define	UINT	unsigned int

#define CROSS(dest,v1,v2)                      \
              dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
              dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
              dest[2]=v1[0]*v2[1]-v1[1]*v2[0];

#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])

#define SUB(dest,v1,v2) dest[0]=v1[0]-v2[0]; dest[1]=v1[1]-v2[1]; dest[2]=v1[2]-v2[2]; 

#define ADD(dest,v1,v2) dest[0]=v1[0]+v2[0]; dest[1]=v1[1]+v2[1]; dest[2]=v1[2]+v2[2]; 

#define MULT(dest,v,factor) dest[0]=factor*v[0]; dest[1]=factor*v[1]; dest[2]=factor*v[2];

#define SET(dest,src) dest[0]=src[0]; dest[1]=src[1]; dest[2]=src[2]; 


//typedef struct cudaQuadTrglMesh {
//	int dev_nodeNum, dev_faceNum;
//	float *dev_nodeTable;			
//	float *dev_lastNodeTable;
//	UINT *dev_faceTable;	//	Note that: the index starts from '1'
//							//		when '0' is shown in the face table, it means that the vertex is not defined
//}CUDAQuadTrglMesh;

class QuadTrglMesh : public GLKObject
{
public:
	QuadTrglMesh(void);
	virtual ~QuadTrglMesh(void);

	void ClearAll();

	void MallocMemory(int nodeNum, int faceNum);
	void SetNodePos(int nodeIndex/*starting from 1*/, float pos[]);
	void SetFaceNodes(int faceIndex/*starting from 1*/, UINT verIndex1, UINT verIndex2, UINT verIndex3, UINT verIndex4);

	int GetFaceNumber();
	int GetNodeNumber();
	bool IsQuadFace(int faceIndex/*starting from 1*/);
	void GetFaceNodes(int faceIndex/*starting from 1*/, UINT &verIndex1, UINT &verIndex2, UINT &verIndex3, UINT &verIndex4);
	void GetNodePos(int nodeIndex/*starting from 1*/, float pos[]);
	float *GetNodeTablePtr() { return m_nodeTable; };
	UINT *GetFaceTablePtr() { return m_faceTable; };

	void CompNormal(int faceIndex/*starting from 1*/, float nv[]);
	void CompBoundingBox(float boundingBox[]);

	void TransformFromQMeshPatch(QMeshPatch* Patch);
	bool InputOBJFile(char *filename);
	bool OutputOBJFile(char *filename);

	bool InputMEBFile(char *filename);	// the binary file of QUAD/TRGL mesh object
	bool OutputMEBFile(char *filename);	// the binary file of QUAD/TRGL mesh object

	float* GetNodeArrayPtr() { return m_nodeTable; };

private:
	int m_nodeNum, m_faceNum;
	float *m_nodeTable;
	UINT *m_faceTable;	//	Note that: the index starts from '1'
						//		when '0' is shown in the face table, it means that the vertex is not defined
};

class PMBody : public GLKEntity
{
public:
	PMBody(void);
	virtual ~PMBody(void);

	void DeleteGLList(bool bShadeOrMesh);
	void BuildGLList(bool bShadeOrMesh);

	virtual void drawShade();
	virtual void drawMesh();
	virtual void drawProfile();
	virtual void drawPreMesh();
	virtual void drawHighLight();
	virtual float getRange() { return m_range; }

	void drawBox(float xx, float yy, float zz, float r);

	void ClearAll();
	void computeRange();

	GLKObList &GetMeshList() { return meshList; };

	void FlipModel(short nDir);
	void CompBoundingBox(float boundingBox[]);
	void Translation(float dx, float dy, float dz);
	void Scaling(float ratio);
	void Rotation(float axisVec[], float angle/*in degree*/);

private:
	GLKObList meshList;
	float m_range;
	int m_drawShadingListID;
	int m_drawMeshListID;

	void _buildDrawShadeList();
	void _buildDrawMeshList();
	void _changeValueToColor(int nType, float & nRed, float & nGreen, float & nBlue);
};

extern QuadTrglMesh* convexHullGeneration(QuadTrglMesh* inputMesh);

#endif