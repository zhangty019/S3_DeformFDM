// GLKSparseMatrixDirectSolver.cpp: implementation of the GLKSparseMatrixDirectSolver class.
//
//////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <math.h>

#include "GLKMatrixLib.h"
#include "GLKSparseMatrix.h"
#include "GLKSparseMatrixDirectSolver.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

GLKSparseMatrixDirectSolver::GLKSparseMatrixDirectSolver()
{

}

GLKSparseMatrixDirectSolver::~GLKSparseMatrixDirectSolver()
{

}

//////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////

bool GLKSparseMatrixDirectSolver::BandGaussianElimination(GLKSparseMatrix *sparseMatrix, double *B)
{
	unsigned long nRowNum,nColNum,i;
	unsigned long rowIndex,colIndex;
	GLKSparseMatrixNode** rowRoots;
	unsigned long *permutation;

	//---------------------------------------------------------------------------------------------
	//	Step 1: preparation
	nRowNum=sparseMatrix->GetRowNumber();	nColNum=sparseMatrix->GetColNumber();
	permutation=sparseMatrix->GetPermutationPointer();
	rowRoots=sparseMatrix->GetRowRoots();

	//---------------------------------------------------------------------------------------------
	//	Step 2: compute the max semi-bandwidth
	unsigned long semiBandWidth=0,maxRow;
	for(i=0;i<nRowNum;i++) {
		unsigned long minIndex=nRowNum,maxIndex=0;
		unsigned long rowIndex=permutation[i];

		GLKSparseMatrixNode* currentNode=rowRoots[i];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode* nextNode=currentNode->pNext;
			if (permutation[currentNode->nColIndex]<minIndex)
				minIndex=permutation[currentNode->nColIndex];
			if (permutation[currentNode->nColIndex]>maxIndex)
				maxIndex=permutation[currentNode->nColIndex];
			currentNode=nextNode;
		}
		if ((maxIndex>rowIndex) && ((maxIndex-rowIndex)>semiBandWidth)) 
			{semiBandWidth=maxIndex-rowIndex;maxRow=rowIndex;}
		if ((minIndex<rowIndex) && ((rowIndex-minIndex)>semiBandWidth)) 
			{semiBandWidth=rowIndex-minIndex;maxRow=rowIndex;}
//		printf ("max=%ld  min=%ld  row=%ld band=%ld\n",maxIndex,minIndex,rowIndex,(maxIndex-minIndex));
	}
// 	printf("Semi-Bandwidth=%ld  maxRow=%ld  rowNum=%ld\n",semiBandWidth,maxRow,nRowNum);

	//---------------------------------------------------------------------------------------------
	//	Step 3: create the band matrix
	double *AA;	double *b;
	AA=new double[nRowNum*(semiBandWidth*2+1)];
	b=new double[nRowNum];
	for(i=0;i<nRowNum*(semiBandWidth*2+1);i++) AA[i]=0.0;

	for(i=0;i<nRowNum;i++) {
		GLKSparseMatrixNode* currentNode=rowRoots[i];
		rowIndex=permutation[i];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode* nextNode=currentNode->pNext;

			colIndex=permutation[currentNode->nColIndex];
			if (rowIndex>semiBandWidth)
				AA[rowIndex*(semiBandWidth*2+1)+colIndex-rowIndex+semiBandWidth]=currentNode->data;
			else
				AA[rowIndex*(semiBandWidth*2+1)+colIndex]=currentNode->data;

			currentNode=nextNode;
		}
		b[rowIndex]=B[i];
	}

	//---------------------------------------------------------------------------------------------
	//	Step 4: Gaussian Elimination
	int rc=aband(AA,b,nRowNum,semiBandWidth,semiBandWidth*2+1,1);

	//---------------------------------------------------------------------------------------------
	//	Step 6: Re-order the solution
	if (rc>0) {for(i=0;i<nRowNum;i++) B[i]=b[permutation[i]];}

	//---------------------------------------------------------------------------------------------
	//	Step 5: free the memory
	delete []AA;	delete []b;

	if (rc>0) return true;
	return false;
}

bool GLKSparseMatrixDirectSolver::BandGaussianElimination(GLKSparseMatrix *sparseMatrix, int colNum, double **B)
{
	unsigned long nRowNum,nColNum,i;
	unsigned long rowIndex,colIndex;
	GLKSparseMatrixNode** rowRoots;
	unsigned long *permutation;

	//---------------------------------------------------------------------------------------------
	//	Step 1: preparation
	nRowNum=sparseMatrix->GetRowNumber();	nColNum=sparseMatrix->GetColNumber();
	permutation=sparseMatrix->GetPermutationPointer();
	rowRoots=sparseMatrix->GetRowRoots();

	//---------------------------------------------------------------------------------------------
	//	Step 2: compute the max semi-bandwidth
	unsigned long semiBandWidth=0,maxRow;
	for(i=0;i<nRowNum;i++) {
		unsigned long minIndex=nRowNum,maxIndex=0;
		unsigned long rowIndex=permutation[i];

		GLKSparseMatrixNode* currentNode=rowRoots[i];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode* nextNode=currentNode->pNext;
			if (permutation[currentNode->nColIndex]<minIndex)
				minIndex=permutation[currentNode->nColIndex];
			if (permutation[currentNode->nColIndex]>maxIndex)
				maxIndex=permutation[currentNode->nColIndex];
			currentNode=nextNode;
		}
		if ((maxIndex>rowIndex) && ((maxIndex-rowIndex)>semiBandWidth)) 
			{semiBandWidth=maxIndex-rowIndex;maxRow=rowIndex;}
		if ((minIndex<rowIndex) && ((rowIndex-minIndex)>semiBandWidth)) 
			{semiBandWidth=rowIndex-minIndex;maxRow=rowIndex;}
	}

	//---------------------------------------------------------------------------------------------
	//	Step 3: create the band matrix
	double *AA;	double *b;
	AA=new double[nRowNum*(semiBandWidth*2+1)];
	b=new double[nRowNum*colNum];
	for(i=0;i<nRowNum*(semiBandWidth*2+1);i++) AA[i]=0.0;

	for(i=0;i<nRowNum;i++) {
		GLKSparseMatrixNode* currentNode=rowRoots[i];
		rowIndex=permutation[i];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode* nextNode=currentNode->pNext;

			colIndex=permutation[currentNode->nColIndex];
			if (rowIndex>semiBandWidth)
				AA[rowIndex*(semiBandWidth*2+1)+colIndex-rowIndex+semiBandWidth]=currentNode->data;
			else
				AA[rowIndex*(semiBandWidth*2+1)+colIndex]=currentNode->data;

			currentNode=nextNode;
		}
		for(int j=0;j<colNum;j++) b[rowIndex*colNum+j]=B[i][j];
	}

	//---------------------------------------------------------------------------------------------
	//	Step 4: Gaussian Elimination
	int rc=aband(AA,b,nRowNum,semiBandWidth,semiBandWidth*2+1,colNum);

	//---------------------------------------------------------------------------------------------
	//	Step 6: Re-order the solution
	if (rc>0) {
		for(i=0;i<nRowNum;i++) {
			for(int j=0;j<colNum;j++) B[i][j]=b[permutation[i]*colNum+j];
		}
	}

	//---------------------------------------------------------------------------------------------
	//	Step 5: free the memory
	delete []AA;	delete []b;

	if (rc>0) return true;
	return false;
}

int GLKSparseMatrixDirectSolver::aband(double* b, double* d, long n, long l, long il, long m)
{ 
	long ls,k,i,j,is,u,v;
	double p,t;

  if (il!=(2*l+1))
    { printf("fail\n"); return(-2);}
  ls=l;
  for (k=0;k<=n-2;k++)
    { p=0.0;
      for (i=k;i<=ls;i++)
        { t=fabs(b[i*il]);
          if (t>p) {p=t; is=i;}
        }
      if (p+1.0==1.0)
        { printf("fail 1 %ld %lf\n",k,p); return(0);}
      for (j=0;j<=m-1;j++)
        { u=k*m+j; v=is*m+j;
          t=d[u]; d[u]=d[v]; d[v]=t;
        }
      for (j=0;j<=il-1;j++)
        { u=k*il+j; v=is*il+j;
          t=b[u]; b[u]=b[v]; b[v]=t;
        }
      for (j=0;j<=m-1;j++)
        { u=k*m+j; d[u]=d[u]/b[k*il];}
      for (j=1;j<=il-1;j++)
        { u=k*il+j; b[u]=b[u]/b[k*il];}
      for (i=k+1;i<=ls;i++)
        { t=b[i*il];
          for (j=0;j<=m-1;j++)
            { u=i*m+j; v=k*m+j;
              d[u]=d[u]-t*d[v];
            }
          for (j=1;j<=il-1;j++)
            { u=i*il+j; v=k*il+j;
              b[u-1]=b[u]-t*b[v];
            }
          u=i*il+il-1; b[u]=0.0;
        }
      if (ls!=(n-1)) ls=ls+1;
    }
  p=b[(n-1)*il];
  if (fabs(p)+1.0==1.0)
    { printf("fail 2\n"); return(0);}
  for (j=0;j<=m-1;j++)
    { u=(n-1)*m+j; d[u]=d[u]/p;}
  ls=1;
  for (i=n-2;i>=0;i--)
    { for (k=0;k<=m-1;k++)
        { 
	  u=i*m+k;
          for (j=1;j<=ls;j++)
            { v=i*il+j; is=(i+j)*m+k;
              d[u]=d[u]-b[v]*d[is];
            }
        }
      if (ls!=(il-1)) ls=ls+1;
    }
  return(2);
}

