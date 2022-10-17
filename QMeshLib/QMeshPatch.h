// QMeshPatch.h: interface for the QMeshPatch class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _QMESHPATCH
#define _QMESHPATCH

#include "../GLKLib/GLKObList.h"
#include <string>

class QMeshTetra;
class QMeshFace;
class QMeshEdge;
class QMeshNode;


class QMeshPatch : public GLKObject  
{
public:
	QMeshPatch();
	virtual ~QMeshPatch();

public:
	void ClearAll();

	bool GetAttribFlag( const int whichBit );
	void SetAttribFlag( const int whichBit, const bool toBe = true );

	int GetIndexNo();		//from 1 to n
	void SetIndexNo( const int _index = 1 );

	int GetTetraNumber();
	QMeshTetra* GetTetraRecordPtr(int No);	//from 1 to n
	GLKObList& GetTetraList();

	int GetFaceNumber();
	QMeshFace* GetFaceRecordPtr(int No);	//from 1 to n
    GLKObList& GetFaceList();

    int GetEdgeNumber();
	QMeshEdge* GetEdgeRecordPtr(int No);	//from 1 to n
    GLKObList& GetEdgeList();

    int GetNodeNumber();
	QMeshNode* GetNodeRecordPtr(int No);	//from 1 to n
    GLKObList& GetNodeList();

	void SetMaterial(bool bDir, int material);
	int GetMaterial(bool bDir);

	bool inputOBJFile(char* filename, bool bOBTFile = false);
	bool inputTETFile(char* filename, bool bOBTFile = false);
	bool inputMFile(char* filename);
	bool inputPLY2File(char* filename);
	bool inputOFFFile(char* filename, bool bOBTFile = false);

	bool inputPosNorFile(char* filename, bool Yup2Zup);

    void outputOBJFile(char* filename, bool bOBTFile=false);
	void outputTrglOBJFile(char* filename);

	void InverseOrientation();
    void ComputeBoundingBox(double &xmin, double &ymin, double &zmin, double &xmax, double &ymax, double &zmax);
    void ComputeBoundingBox(double boundingBox[]);


	void constructionFromVerFaceTable(int nodeNum, float *nodeTable, int faceNum, unsigned int* faceTable);

    GLKObList& GetAttrib_EdgeList();

    QMeshPatch *CopyMesh();
    // for volume mesh
	bool isVolume;
	bool runTopOpt = false;
	// for S^3 DeformFDM
	bool drawThisPatch = true;
	bool drawStressField = false;
	bool drawOverhangSurface = false;
	bool drawSupportField = false;

	double minStressValue = 0, maxStressValue = 0; // raw data
	double isoSurfaceValue = 0.0;
	std::string patchName = "";
	bool thicknessChecked = false;
	// io operation
	int splitIndex;

	// supportTree
	int max_segment_NUM = -1; //record ployline segment num
	int max_branch_NUM = -1;//record max branch num of a support-structure tree
	int max_height = -1; //record max height of a support-structure tree

	bool is_SupportLayer = false;
	bool is_Slimmed_SupportLayer = false;
	int compatible_layer_Index = -1;

	QMeshPatch* attached_Layer = NULL; //toolpath on which layer
	bool isInstalled_toolpath = false;

	//Gcode generation
	QMeshPatch* rootPatch_jumpPatch = NULL;// the father Waypoint patch of each jump section
	bool drawSingularity = false;
	bool drawSolveSelect = false;

	//stress field
	double minPrincipleStressValue = 0, maxPrincipleStressValue = 0; // Principle data
	bool isStressLayer = false; // for stress guided toolpath generation

private:
	int indexno;			// start from 1 to n  

	bool flags[8];			// bit 0 -- TRUE for displaying the valence on nodes
							//			FALSE for NOT displaying the valence on nodes
							// bit 1 -- TRUE for displaying the tensile energy on edges
							//			FALSE for NOT displaying the tensile energy on edges
							// bit 2 -- TRUE for the 2D pattern has been determined
							//			FALSE for the 2D pattern has NOT been determined


	GLKObList tetraList;	// a list of mesh's tetras (QMeshTetra)
	GLKObList faceList;		// a list of mesh's faces (QMeshFace)
	GLKObList edgeList;		// a list of mesh's edges (QMeshEdge)
	GLKObList nodeList;		// a list of mesh's nodes (QMeshNode)


    GLKObList Attrib_EdgeList;
};

#endif
