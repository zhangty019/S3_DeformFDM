// GLKSparseMatrix.h: interface for the GLKSparseMatrix class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _GLKSPARSEMATRIX
#define _GLKSPARSEMATRIX

#include "GLKObList.h"

class GLKSparseMatrixNode
{
public:
	GLKSparseMatrixNode() {pNext=NULL;nColIndex=0;};
	virtual ~GLKSparseMatrixNode() {};

	GLKSparseMatrixNode *pNext;
	double data;	unsigned long nColIndex;	// index start from zero
};

class GLKSparseMatrix  
{
public:
	GLKSparseMatrix();
	virtual ~GLKSparseMatrix();

	void Initialization(unsigned long nRow, unsigned long nCol);

	void InputElement(unsigned long nRowIndex, unsigned long nColIndex, double data);
    void AddElement(unsigned long nRowIndex, unsigned long nColIndex, double delta);
	void InputComplete(bool bSort=true);	//	if the elements of each row are inserted by the
											//		order of their nColIndex (ascending),
											//	to reduce the computation time, we can set
											//		"bSort=false" as the input parameter.
	void EliminateRow(unsigned long nRowIndex);

	void InputPermutationTable(unsigned long *permutations);	// the number of element is the same as RowNumber
	void OutputPermutationTable(unsigned long *permutations);	// the number of element is the same as RowNumber
	unsigned long * GetPermutationPointer() {return m_permutations;};

	//-----------------------------------------------------------------------------------------
	//	Elements are reordered by the Cuthill-McKee algorithm, where the result is
	//		stored in the array - m_permutations[...] as the new index
	//	Note that: this only works well when the pattern of the matrix is symmetric
	void ReOrderingByCuthillMcKeeAlgorithm(bool bSymmetricMatrix=true);

	GLKSparseMatrixNode** GetRowRoots() {return m_rowRoots;};
	unsigned long GetRowNumber() {return m_nRow;};
	unsigned long GetColNumber() {return m_nCol;};

	unsigned long GetNonzeroElementNumber(); 

	void DEBUG_OutputPattern(char *filename);

private:
	void _clearAll();
	void _quickSort(GLKSparseMatrixNode **nodeArray, long left, long right, long max);

	int _pseudoPeripheralNodeSearch(GLKSparseMatrixNode** colRoots=NULL);
	void _levelStructuring(int rootIndex, int &levelNum, int &newRootIndex, 
							GLKSparseMatrixNode** colRoots=NULL);

private:
	unsigned long m_nRow,m_nCol;
	GLKSparseMatrixNode** m_rowRoots;
	unsigned long *m_rowElementNum;
	unsigned long *m_permutations;	// store the new index (re-ordered) of each element
};

void Mul_A_b(GLKSparseMatrix *sparseMat, double *b, double *x);
void Mul_At_b(GLKSparseMatrix *sparseMat, double *b, double *x);
void Mul_At_A(GLKSparseMatrix *sparseMat, GLKSparseMatrix *&outputMat);
void Mul_At_A1(GLKSparseMatrix *sparseMat, GLKSparseMatrix *&outputMat);
void Mul_A_At(GLKSparseMatrix *sparseMat, GLKSparseMatrix *&transposeMat, GLKSparseMatrix *&outputMat);
void Transpose(GLKSparseMatrix *sparseMat, GLKSparseMatrix *&outputMat);
bool Mul_A_B(GLKSparseMatrix *matA, GLKSparseMatrix *matB, GLKSparseMatrix *&outputMat);
void CopyMat(GLKSparseMatrix *sparseMat, GLKSparseMatrix *&outputMat);
void PrintMat(GLKSparseMatrix *sparseMat);

#endif
