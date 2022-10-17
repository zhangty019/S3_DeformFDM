// GLKSparseMatrix.cpp: implementation of the GLKSparseMatrix class.
//
//////////////////////////////////////////////////////////////////////
#define _CRT_SECURE_NO_DEPRECATE

#include <stdio.h>
#include "GLKHeap.h"
#include "GLKObList.h"
#include "GLKSparseMatrix.h"

/*//-------------------------------------------------------------------
//	Code in the following lines are for detecting memory leaks
#ifdef _DEBUG
#include <afx.h>
#define new DEBUG_NEW
#endif*/

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

GLKSparseMatrix::GLKSparseMatrix()
{
	m_nCol=0;	m_nRow=0;
}

GLKSparseMatrix::~GLKSparseMatrix()
{
	_clearAll();
}

//////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////

void GLKSparseMatrix::Initialization(unsigned long nRow, unsigned long nCol)
{
	unsigned long i;

	_clearAll();	
//	if (nCol>nRow) return;
	if (nRow>0) {
		m_nRow=nRow;	m_nCol=nCol;
		m_rowRoots=(GLKSparseMatrixNode**)new long[m_nRow];
		m_rowElementNum=new unsigned long[m_nRow];
		for(i=0;i<m_nRow;i++) {m_rowRoots[i]=NULL; m_rowElementNum[i]=0;}
		m_permutations=new unsigned long[m_nRow];
		for(i=0;i<m_nRow;i++) m_permutations[i]=i;
	}
}

void GLKSparseMatrix::InputElement(unsigned long nRowIndex, unsigned long nColIndex, double data)
{
	GLKSparseMatrixNode *newNode=new GLKSparseMatrixNode;
	newNode->nColIndex=nColIndex;	newNode->data=data;
	newNode->pNext=m_rowRoots[nRowIndex];
	m_rowRoots[nRowIndex]=newNode;
	m_rowElementNum[nRowIndex]++;
}

void GLKSparseMatrix::AddElement(unsigned long nRowIndex, unsigned long nColIndex, double delta)
{
    if(m_rowRoots[nRowIndex]){
        GLKSparseMatrixNode* currentNode = m_rowRoots[nRowIndex];
        for(;currentNode!=NULL;){
            GLKSparseMatrixNode* nextNode=currentNode->pNext;
            if(currentNode->nColIndex==nColIndex){
                currentNode->data+=delta;
                return;
            }
            currentNode=nextNode;
        }
    }
    InputElement(nRowIndex,nColIndex,delta);
}

void GLKSparseMatrix::EliminateRow(unsigned long nRowIndex)
{
	GLKSparseMatrixNode *currentNode=m_rowRoots[nRowIndex];
	for(;currentNode!=NULL;) {
		GLKSparseMatrixNode *nextNode=currentNode->pNext;
		delete currentNode;
		currentNode=nextNode;
	}
	m_rowRoots[nRowIndex]=NULL;
	m_rowElementNum[nRowIndex]=0;
}

unsigned long GLKSparseMatrix::GetNonzeroElementNumber()
{
	unsigned long i,nnzNum=0;

	for(i=0;i<m_nRow;i++) nnzNum+=m_rowElementNum[i];

	return nnzNum;
}

void GLKSparseMatrix::InputComplete(bool bSort)
{
	unsigned long i,j,num;

	//-------------------------------------------------------------------------
	//	resort the elements in each row
	for(i=0;i<m_nRow;i++) {
		GLKSparseMatrixNode *currentNode=m_rowRoots[i];

		num=m_rowElementNum[i];		if (num==0) continue;
		GLKSparseMatrixNode **nodeArray=(GLKSparseMatrixNode **)new long[num];

		if (bSort) {
			currentNode=m_rowRoots[i];
			j=0;
			for(;currentNode!=NULL;) {
				GLKSparseMatrixNode *nextNode=currentNode->pNext;
				nodeArray[j]=currentNode;
				currentNode=nextNode;	j++;
			}
			_quickSort(nodeArray,0,num-1,num);
		}
		else {
			currentNode=m_rowRoots[i];
			j=0;
			for(;currentNode!=NULL;) {
				GLKSparseMatrixNode *nextNode=currentNode->pNext;
				nodeArray[num-1-j]=currentNode;
				currentNode=nextNode;	j++;
			}
		}

		for(j=0;j<num-1;j++) nodeArray[j]->pNext=nodeArray[j+1]; 
		nodeArray[num-1]->pNext=NULL;	m_rowRoots[i]=nodeArray[0];

//		for(j=0;j<num;j++) printf("%ld (%lf) ",nodeArray[j]->nColIndex,nodeArray[j]->data);	printf("\n");
		delete [](GLKSparseMatrixNode **)nodeArray;
	}
}

void GLKSparseMatrix::InputPermutationTable(unsigned long *permutations)
{
	unsigned long i;
	for(i=0;i<m_nRow;i++) m_permutations[i]=permutations[i];
}

void GLKSparseMatrix::OutputPermutationTable(unsigned long *permutations)
{
	unsigned long i;
	for(i=0;i<m_nRow;i++) permutations[i]=m_permutations[i];
}

void GLKSparseMatrix::ReOrderingByCuthillMcKeeAlgorithm(bool bSymmetricMatrix)
{
	if (m_nCol!=m_nRow) return;	// the algorithm can only be applied to the square matrix

	bool *bInserted;	unsigned long *rSet;	unsigned long rSetNum=0;
	unsigned long i;

	bInserted=new bool[m_nRow];		rSet=new unsigned long[m_nRow];
	for(i=0;i<m_nRow;i++) bInserted[i]=false;

	GLKSparseMatrixNode** m_colRoots;
	GLKSparseMatrixNode** m_colTails;
	if (!bSymmetricMatrix) {
		m_colRoots=(GLKSparseMatrixNode**)new long[m_nCol];
		m_colTails=(GLKSparseMatrixNode**)new long[m_nCol];
		for(i=0;i<m_nCol;i++) {m_colTails[i]=NULL;m_colRoots[i]=NULL;}
		for(i=0;i<m_nRow;i++) {
			GLKSparseMatrixNode *currentNode=m_rowRoots[i];
			for(;currentNode!=NULL;) {
				GLKSparseMatrixNode *nextNode=currentNode->pNext;
				
				GLKSparseMatrixNode *newNode=new GLKSparseMatrixNode;
				newNode->nColIndex=i;
				unsigned long colIndex=currentNode->nColIndex;
				if (m_colTails[colIndex]) {
					m_colTails[colIndex]->pNext=newNode;
					m_colTails[colIndex]=newNode;
				}
				else {
					m_colRoots[colIndex]=newNode;
					m_colTails[colIndex]=newNode;
				}
				currentNode=nextNode;
			}
		}
		delete [](GLKSparseMatrixNode**)m_colTails;
	}

	//--------------------------------------------------------------------------------------
	//	Step 1: find the peripheral vertex and insert it into the rSet
	unsigned long peripheralVertexIndex;	
	unsigned long degree=m_nRow+1;
	for(i=0;i<m_nRow;i++) {
		if (m_rowElementNum[i]<degree) { degree=m_rowElementNum[i]; peripheralVertexIndex=i; }
	}
//	peripheralVertexIndex=_pseudoPeripheralNodeSearch();
	rSet[rSetNum++]=peripheralVertexIndex;	bInserted[peripheralVertexIndex]=true;

	//--------------------------------------------------------------------------------------
	//	Step 2: incrementally add more nodes into the rSet
	for(i=0;rSetNum<m_nRow;i++) {
		GLKSparseMatrixNode *currentNode=m_rowRoots[rSet[i]];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode *nextNode=currentNode->pNext;
			if (!(bInserted[currentNode->nColIndex])) {
				rSet[rSetNum++]=currentNode->nColIndex;
				bInserted[currentNode->nColIndex]=true;
			}
			currentNode=nextNode;
		}

		if (i==rSetNum-1) {	// insert another peripheral vertex
			unsigned long peripheralVertexIndex;	
			unsigned long degree=m_nRow+1;
			for(unsigned j=0;j<m_nRow;j++) {
				if (bInserted[j]) continue;
				if (m_rowElementNum[j]<degree) { degree=m_rowElementNum[j]; peripheralVertexIndex=j; }
			}
			rSet[rSetNum++]=peripheralVertexIndex;	bInserted[peripheralVertexIndex]=true;
		}

		if (bSymmetricMatrix) continue;

		currentNode=m_colRoots[rSet[i]];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode *nextNode=currentNode->pNext;
			if (!(bInserted[currentNode->nColIndex])) {
				rSet[rSetNum++]=currentNode->nColIndex;
				bInserted[currentNode->nColIndex]=true;
			}
			currentNode=nextNode;
		}
	}

	//--------------------------------------------------------------------------------------
	//	Step 3: build the permulation table
	for(i=0;i<rSetNum;i++) m_permutations[rSet[i]]=i;
//	for(i=0;i<rSetNum;i++) m_permutations[rSet[rSetNum-1-i]]=i;

	if (!bSymmetricMatrix) {
		for(i=0;i<m_nCol;i++) {
			GLKSparseMatrixNode *currentNode=m_colRoots[i];
			for(;currentNode!=NULL;) {
				GLKSparseMatrixNode *nextNode=currentNode->pNext;
				delete currentNode;
				currentNode=nextNode;
			}
		}
		delete [](GLKSparseMatrixNode**)m_colRoots;
	}

	delete []bInserted;		delete []rSet;
}

int GLKSparseMatrix::_pseudoPeripheralNodeSearch(GLKSparseMatrixNode** colRoots)
{
	int levelNum,newRootIndex,levelNum2,newRootIndex2;

	_levelStructuring((int)m_nRow/2,levelNum,newRootIndex,colRoots);
	while(true) {
		_levelStructuring(newRootIndex,levelNum2,newRootIndex2,colRoots);

		if (levelNum2<=levelNum) return newRootIndex;

		newRootIndex=newRootIndex2;	levelNum=levelNum2;
	}

	return 0;
}

void GLKSparseMatrix::_levelStructuring(int rootIndex, int &levelNum, int &newRootIndex, 
										GLKSparseMatrixNode** colRoots)
{
	unsigned long i,currentLevel;	

	GLKArray *bottomLevelIndexArray,*currentLevelIndexArray;
	bottomLevelIndexArray=new GLKArray(100,100,1);
	currentLevelIndexArray=new GLKArray(100,100,1);

	int* nodeLevels=new int[m_nRow];
	for(i=0;i<m_nRow;i++) nodeLevels[i]=-1;

	//-------------------------------------------------------------------------
	//	Build the level structure
	currentLevelIndexArray->Add(rootIndex);	nodeLevels[rootIndex]=0;
	currentLevel=0;
	do{
		bottomLevelIndexArray->RemoveAll();

		int j,num=currentLevelIndexArray->GetSize();
		for(j=0;j<num;j++) {
			int elementIndex=currentLevelIndexArray->GetIntAt(j);

			GLKSparseMatrixNode *currentNode=m_rowRoots[elementIndex];
			for(;currentNode!=NULL;) {
				GLKSparseMatrixNode *nextNode=currentNode->pNext;
				if (nodeLevels[currentNode->nColIndex]<0) {
					nodeLevels[currentNode->nColIndex]=currentLevel+1;
					bottomLevelIndexArray->Add((int)(currentNode->nColIndex));
				}
				currentNode=nextNode;
			}

			if (!colRoots) continue;

			currentNode=colRoots[elementIndex];
			for(;currentNode!=NULL;) {
				GLKSparseMatrixNode *nextNode=currentNode->pNext;
				if (nodeLevels[currentNode->nColIndex]<0) {
					nodeLevels[currentNode->nColIndex]=currentLevel+1;
					bottomLevelIndexArray->Add((int)(currentNode->nColIndex));
				}
				currentNode=nextNode;
			}
		}

		if (bottomLevelIndexArray->GetSize()==0) break;

		currentLevelIndexArray->RemoveAll();
		num=bottomLevelIndexArray->GetSize();
		for(j=0;j<num;j++) currentLevelIndexArray->Add(bottomLevelIndexArray->GetIntAt(j));

		currentLevel++;
	}while(true);

	//-------------------------------------------------------------------------
	//	Find the one with the minimum degree in the bottom level
	int num=currentLevelIndexArray->GetSize();	unsigned long degree=m_nRow;
	for(int j=0;j<num;j++) {
		int index=currentLevelIndexArray->GetIntAt(j);
		if (m_rowElementNum[index]<degree) {
			degree=m_rowElementNum[index];	newRootIndex=index;
		}
	}
	levelNum=currentLevel+1;

	delete bottomLevelIndexArray;	delete currentLevelIndexArray;	
	delete []nodeLevels;
}

void GLKSparseMatrix::_quickSort(GLKSparseMatrixNode **nodeArray, 
								 long left, long right, long max)
{
	long i,j;	GLKSparseMatrixNode *s;

	if (left < right){
		s = nodeArray[left];
		i = left;	j = right + 1; 
		while(true) {
			while( (i + 1 < max) && (nodeArray[++i]->nColIndex < s->nColIndex) ) ; 
			while( (j -1 > -1) && (nodeArray[--j]->nColIndex > s->nColIndex) ) ;
			if (i >= j) break; 

			GLKSparseMatrixNode * temp=nodeArray[i];
			nodeArray[i]=nodeArray[j];
			nodeArray[j]=temp;
		}
		nodeArray[left] = nodeArray[j];
		nodeArray[j] = s; 
		_quickSort(nodeArray, left, j-1, max);
		_quickSort(nodeArray, j+1, right, max);
	}
}

void GLKSparseMatrix::_clearAll()
{
	unsigned long i;
	if ((m_nRow==0) && (m_nCol==0)) return;

	for(i=0;i<m_nRow;i++) {
		GLKSparseMatrixNode *currentNode=m_rowRoots[i];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode *nextNode=currentNode->pNext;
			delete currentNode;
			currentNode=nextNode;
		}
	}
	delete [](GLKSparseMatrixNode **)m_rowRoots;
	delete []m_rowElementNum;
	delete []m_permutations;

	m_nRow=0;	m_nCol=0;
}

void GLKSparseMatrix::DEBUG_OutputPattern(char *filename)
{
	FILE *fp;	unsigned long i,j,fnum;	double xx,yy;

	fp=fopen(filename, "w");

	fnum=0;
	for(i=0;i<m_nRow;i++) {
		GLKSparseMatrixNode *currentNode=m_rowRoots[i];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode *nextNode=currentNode->pNext;
			j=currentNode->nColIndex;

			xx=0.1*(double)(m_permutations[j])-0.1*(double)(m_nCol)*0.5;	
			yy=0.1*(double)(m_nRow-1-m_permutations[i])-0.1*(double)(m_nRow)*0.5;
			fprintf(fp,"v %lf %lf 0.0\n",xx,yy);
			currentNode=nextNode;	fnum++;
		}
	}
	fprintf(fp,"v %lf %lf -1.0\n",-0.1*(double)(m_nCol)*0.5,-0.1*(double)(m_nRow)*0.5);
	fprintf(fp,"v %lf %lf -1.0\n",-0.1*(double)(m_nCol)*0.5,0.1*(double)(m_nRow)-0.1*(double)(m_nRow)*0.5);
	fprintf(fp,"v %lf %lf -1.0\n",0.1*(double)m_nCol*0.5,0.1*(double)(m_nRow)-0.1*(double)(m_nRow)*0.5);
	fprintf(fp,"v %lf %lf -1.0\n",0.1*(double)m_nCol*0.5,-0.1*(double)(m_nRow)*0.5);
	
	fprintf(fp,"f %ld %ld %ld %ld\n",fnum+1,fnum+2,fnum+3,fnum+4);

	fclose(fp);
}

void Mul_A_b(GLKSparseMatrix *sparseMat, double *b, double *x)
{
	unsigned long i;
	unsigned long nRow=sparseMat->GetRowNumber();

	for(i=0;i<nRow;i++) {
		x[i]=0.0;
		GLKSparseMatrixNode* currentNode=(sparseMat->GetRowRoots())[i];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode* nextNode=currentNode->pNext;
			x[i]+=b[currentNode->nColIndex]*(currentNode->data);
			currentNode=nextNode;
		}
	}
}

void Mul_At_b(GLKSparseMatrix *sparseMat, double *b, double *x)
{
	unsigned long i,nRow;
	GLKSparseMatrix *outputMat;

	Transpose(sparseMat,outputMat);

	nRow=outputMat->GetRowNumber();
	for(i=0;i<nRow;i++) {
		x[i]=0.0;
		GLKSparseMatrixNode* currentNode=(outputMat->GetRowRoots())[i];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode* nextNode=currentNode->pNext;
			x[i]+=b[currentNode->nColIndex]*(currentNode->data);
			currentNode=nextNode;
		}
	}

	delete outputMat;
}

void Mul_At_A1(GLKSparseMatrix *sparseMat, GLKSparseMatrix *&outputMat)
{
    unsigned long i, nRow, nCol, nRow1;

    outputMat=new GLKSparseMatrix;	nRow = nCol = sparseMat->GetColNumber();
    nRow1 = sparseMat->GetRowNumber();
    outputMat->Initialization(nRow,nCol);
    for(i=0;i<nRow1;i++){
        GLKSparseMatrixNode* currentnode1 = (sparseMat->GetRowRoots())[i];
        for(;currentnode1!=NULL;){
            GLKSparseMatrixNode* currentnode2 = (sparseMat->GetRowRoots())[i];
            for(;currentnode2!=NULL;){
                outputMat->AddElement(currentnode1->nColIndex, currentnode2->nColIndex, (currentnode1->data)*(currentnode2->data));
                currentnode2 = currentnode2->pNext;
            }
            currentnode1 = currentnode1->pNext;
        }
    }
    outputMat->InputComplete(false);
}

void Mul_At_A(GLKSparseMatrix *sparseMat, GLKSparseMatrix *&outputMat)
{
	unsigned long i,j,nRow,nCol;	double data;
	GLKSparseMatrix *transposeMat;

	Transpose(sparseMat,transposeMat);
	outputMat=new GLKSparseMatrix;	nRow=nCol=sparseMat->GetColNumber();
	outputMat->Initialization(nRow,nCol);

	for(i=0;i<nRow;i++) {
		for(j=0;j<nCol;j++) {
			GLKSparseMatrixNode* currentRowNode=(transposeMat->GetRowRoots())[i];
			GLKSparseMatrixNode* currentColNode=(transposeMat->GetRowRoots())[j];
			data=0.0;
			for(;(currentRowNode!=NULL)&&(currentColNode!=NULL);) {
				if (currentRowNode->nColIndex==currentColNode->nColIndex) {
					data+=(currentRowNode->data)*(currentColNode->data);
					currentRowNode=currentRowNode->pNext;
					currentColNode=currentColNode->pNext;
				}
				else if (currentRowNode->nColIndex<currentColNode->nColIndex) {
					currentRowNode=currentRowNode->pNext;
				}
				else {
					currentColNode=currentColNode->pNext;
				}
			}
			if (data!=0.0) outputMat->InputElement(i,j,data);
		}
	}
	outputMat->InputComplete(false);

	delete transposeMat;
}

void Mul_A_At(GLKSparseMatrix *sparseMat, GLKSparseMatrix *&transposeMat, GLKSparseMatrix *&outputMat)
{
    unsigned long i, nRow, nCol, nRow1;
    Transpose(sparseMat,transposeMat);

    outputMat=new GLKSparseMatrix;	nRow=nCol=transposeMat->GetColNumber();
    nRow1 = transposeMat->GetRowNumber();
    outputMat->Initialization(nRow,nCol);
    for(i=0;i<nRow1;i++){
        GLKSparseMatrixNode* currentnode1 = (transposeMat->GetRowRoots())[i];
        for(;currentnode1!=NULL;){
            GLKSparseMatrixNode* currentnode2 = (transposeMat->GetRowRoots())[i];
            for(;currentnode2!=NULL;){
                outputMat->AddElement(currentnode1->nColIndex, currentnode2->nColIndex, (currentnode1->data)*(currentnode2->data));
                currentnode2 = currentnode2->pNext;
            }
            currentnode1 = currentnode1->pNext;
        }
    }
    outputMat->InputComplete(false);
}

void CopyMat(GLKSparseMatrix *sparseMat, GLKSparseMatrix *&outputMat)
{
	unsigned long nRow,nCol,i;

	nRow=sparseMat->GetRowNumber();	nCol=sparseMat->GetColNumber();

	outputMat=new GLKSparseMatrix;	outputMat->Initialization(nRow,nCol);
	for(i=0;i<nRow;i++) {
		GLKSparseMatrixNode* currentNode=(sparseMat->GetRowRoots())[i];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode* nextNode=currentNode->pNext;
			outputMat->InputElement(i,currentNode->nColIndex,currentNode->data);
			currentNode=nextNode;
		}
	}
	outputMat->InputComplete(false);
}

void PrintMat(GLKSparseMatrix *sparseMat)
{
	int nRowNum=sparseMat->GetRowNumber();
	for(int i=0;i<nRowNum;i++) {
		GLKSparseMatrixNode *currentNode=(sparseMat->GetRowRoots())[i];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode *nextNode=currentNode->pNext;
			printf("rowIndex=%d    colIndex=%d    value=%lf\n",i,currentNode->nColIndex,currentNode->data);
			currentNode=nextNode;
		}
	}
}

bool Mul_A_B(GLKSparseMatrix *matA, GLKSparseMatrix *matB, GLKSparseMatrix *&outputMat)
{
	unsigned long aRow,aCol,bRow,bCol,i,j;	double data;

	aRow=matA->GetRowNumber();	aCol=matA->GetColNumber();
	bRow=matB->GetRowNumber();	bCol=matB->GetColNumber();
	if (aCol!=bRow) {outputMat=NULL; return false;}

	//--------------------------------------------------------------------------------
	//	Preparation
	GLKSparseMatrixNode **colRootNodes;	colRootNodes=(GLKSparseMatrixNode **)new long[bCol];
	GLKSparseMatrixNode **colTailNodes;	colTailNodes=(GLKSparseMatrixNode **)new long[bCol];
	for(j=0;j<bCol;j++) colRootNodes[j]=colTailNodes[j]=NULL;
	for(i=0;i<bRow;i++) {
		GLKSparseMatrixNode* currentNode=(matB->GetRowRoots())[i];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode* nextNode=currentNode->pNext;
			j=currentNode->nColIndex;

			GLKSparseMatrixNode* newNode=new GLKSparseMatrixNode;
			newNode->nColIndex=i;
			newNode->data=currentNode->data;
			if (colTailNodes[j]) {
				colTailNodes[j]->pNext=newNode; colTailNodes[j]=newNode;
			}
			else {
				colRootNodes[j]=newNode; colTailNodes[j]=newNode;
			}

			currentNode=nextNode;
		}
	}

	//--------------------------------------------------------------------------------
	//	Multiplication
	outputMat=new GLKSparseMatrix;	outputMat->Initialization(aRow,bCol);
	for(i=0;i<aRow;i++) {
		for(j=0;j<bCol;j++) {
			GLKSparseMatrixNode* currentRowNode=(matA->GetRowRoots())[i];
			GLKSparseMatrixNode* currentColNode=colRootNodes[j];
			data=0.0;	bool bFlag=false;
			for(;(currentRowNode!=NULL)&&(currentColNode!=NULL);) {
				if (currentRowNode->nColIndex==currentColNode->nColIndex) {
					data+=(currentRowNode->data)*(currentColNode->data);
					currentRowNode=currentRowNode->pNext;
					currentColNode=currentColNode->pNext;
					bFlag=true;
				}
				else if (currentRowNode->nColIndex<currentColNode->nColIndex) {
					currentRowNode=currentRowNode->pNext;
				}
				else {
					currentColNode=currentColNode->pNext;
				}
			}
			if (bFlag) outputMat->InputElement(i,j,data);
		}
	}
	outputMat->InputComplete(false);

	//--------------------------------------------------------------------------------
	//	Free the memory
	for(j=0;j<bCol;j++) {
		GLKSparseMatrixNode* currentNode=colRootNodes[j];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode* nextNode=currentNode->pNext;
			delete currentNode;
			currentNode=nextNode;
		}
	}
	delete [](GLKSparseMatrixNode **)colRootNodes; delete [](GLKSparseMatrixNode **)colTailNodes;

	return true;
}

void Transpose(GLKSparseMatrix *sparseMat, GLKSparseMatrix *&outputMat)
{
	unsigned long i,nRow;

	outputMat=new GLKSparseMatrix;
	outputMat->Initialization(sparseMat->GetColNumber(),sparseMat->GetRowNumber());
	nRow=sparseMat->GetRowNumber();
	for(i=0;i<nRow;i++) {
		GLKSparseMatrixNode* currentNode=(sparseMat->GetRowRoots())[i];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode* nextNode=currentNode->pNext;
			outputMat->InputElement(currentNode->nColIndex,i,currentNode->data);
			currentNode=nextNode;
		}
	}
	outputMat->InputComplete(false);
}
