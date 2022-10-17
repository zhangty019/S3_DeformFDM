// GLKSparseMatrixLib.cpp: implementation of the GLKSparseMatrixLib class.
//
//////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <math.h>

#include "GLKSparseMatrix.h"
#include "GLKSparseMatrixIterativeSolver.h"

#define EPS		1.0e-8

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

GLKSparseMatrixIterativeSolver::GLKSparseMatrixIterativeSolver()
{

}

GLKSparseMatrixIterativeSolver::~GLKSparseMatrixIterativeSolver()
{

}

//////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////

void GLKSparseMatrixIterativeSolver::SparseMatrixIn_ZeroStartIndex(double **a, unsigned long n, double thresh, unsigned long nmax, 
									double *sa, unsigned long *ija)
{
	unsigned long i,j,k;

	for (j=1;j<=n;j++) sa[j]=a[j-1][j-1];	// Store diagonal elements.
	ija[1]=n+2;							// Index to 1st row off-diagonal element, if any.
	k=n+1;
	for (i=1;i<=n;i++) {				// Loop over rows.
		for (j=1;j<=n;j++) {			// Loop over columns.
			if (fabs(a[i-1][j-1]) >= thresh && i != j) {
				if (++k > nmax) {
					printf("SparseMatrixIn: nmax too small"); return;
				}
				sa[k]=a[i-1][j-1];			// Store off-diagonal elements and their columns.
				ija[k]=j;
			}
		}
		ija[i+1]=k+1;					// As each rowi s completed, store index to next. 
	}
}

void GLKSparseMatrixIterativeSolver::SparseMatrixIn(double **a, unsigned long n, double thresh, unsigned long nmax, 
										 double *sa, unsigned long *ija)
{
	unsigned long i,j,k;

	for (j=1;j<=n;j++) sa[j]=a[j][j];	// Store diagonal elements.
	ija[1]=n+2;							// Index to 1st row off-diagonal element, if any.
	k=n+1;
	for (i=1;i<=n;i++) {				// Loop over rows.
		for (j=1;j<=n;j++) {			// Loop over columns.
			if (fabs(a[i][j]) >= thresh && i != j) {
				if (++k > nmax) {
					printf("SparseMatrixIn: nmax too small"); return;
				}
				sa[k]=a[i][j];			// Store off-diagonal elements and their columns.
				ija[k]=j;
			}
		}
		ija[i+1]=k+1;					// As each rowi s completed, store index to next. 
	}
}

void GLKSparseMatrixIterativeSolver::SparseMatrixIn(double *a, unsigned long *aComIndex,
										unsigned long *aRowBeginIndex,
										unsigned long n, unsigned long nnz, 
										double *sa, unsigned long *ija, unsigned long nmax)
{
	unsigned long i,rowIndex,colIndex,elementIndex,stIndex,edIndex,nDiagIndex,lastIndex;

	elementIndex=n+2;		lastIndex=1;
	for(rowIndex=1;rowIndex<=n;rowIndex++) {
		stIndex=aRowBeginIndex[rowIndex];
		if (rowIndex==n) edIndex=nnz; else edIndex=aRowBeginIndex[rowIndex+1]-1;
		nDiagIndex=0;
		for(i=stIndex;i<=edIndex;i++) {
			colIndex=aComIndex[i];
			if (colIndex!=rowIndex) {
				ija[elementIndex]=colIndex;	sa[elementIndex]=a[i];	elementIndex++;
			}
			else {
				nDiagIndex=colIndex;
				ija[rowIndex]=lastIndex+1;	sa[rowIndex]=a[i];
			}
		}
		if (nDiagIndex==0) {ija[rowIndex]=lastIndex+1;sa[rowIndex]=0.0;}
		lastIndex=elementIndex-1;
	}
	ija[1]=n+2;		ija[n+1]=elementIndex;
}

double GLKSparseMatrixIterativeSolver::coefficientFromComRowByIndex(double *a, unsigned long *aComIndex, 
			unsigned long *aRowBeginIndex, unsigned long n, unsigned long nnz, unsigned long i, 
			unsigned long j)
{
	unsigned long colLowerBound,colUpperBound,k;

	if (aRowBeginIndex[i]==0) return 0.0; // empty row

	colLowerBound=aRowBeginIndex[i];	
	if (i==n) {
		colUpperBound=nnz;
	}
	else {
		unsigned long beginIndexNext=nnz+1;
		// find next non-zero row-begin-index
		for(k=i+1;k<=n;k++) {
			if (aRowBeginIndex[k]!=0) {beginIndexNext=aRowBeginIndex[i+1];break;}
		}
		colUpperBound=beginIndexNext-1;
	}

	for(k=colLowerBound;k<=colUpperBound;k++) 
		if (aComIndex[k]==j) return a[k];

	return 0.0;
}

void GLKSparseMatrixIterativeSolver::SparseMatrixMultiplyTransposedSparseMatrix(double *sa, unsigned long *ija,
		double *sb, unsigned long *ijb, double *sc, unsigned long *ijc) // pattern multiply
{
	unsigned long i,ijma,ijmb,j,m,ma,mb,mbb,mn;
	double sum;
	
	if (ija[1] != ijb[1] || ija[1] != ijc[1]) {
		printf("sprspm: sizes do not match");
		return;
	}
	
	for (i=1;i<=ijc[1]-2;i++) { // Loop over rows.
		j=m=i;	// Set up so that first pass through loop does the
				// diagonal component. 
		mn=ijc[i];
		sum=sa[i]*sb[i];
		for (;;) {	// Main loop over each component to be output.
			mb=ijb[j];
			for (ma=ija[i];ma<=ija[i+1]-1;ma++) {
				// Loop through elements in A¡¯s row. Convoluted logic, following, accounts for the
				// various combinations of diagonal and off-diagonal elements.
				ijma=ija[ma];
				if (ijma == j) sum += sa[ma]*sb[j];
				else {
					while (mb < ijb[j+1]) {
						ijmb=ijb[mb];
						if (ijmb == i) {
							sum += sa[i]*sb[mb++];
							continue;
						} else if (ijmb < ijma) {
							mb++;
							continue;
						} else if (ijmb == ijma) {
							sum += sa[ma]*sb[mb++];
							continue;
						}
						break;
					}
				}
			}
			for (mbb=mb;mbb<=ijb[j+1]-1;mbb++) { // Exhaust the remainder of B¡¯s row.
				if (ijb[mbb] == i) sum += sa[i]*sb[mbb];
			}
			sc[m]=sum;
			sum=0.0;	// Reset indices for next pass through loop.
			if (mn >= ijc[i+1]) break;
			j=ijc[m=mn++];
		}
	}
}

void GLKSparseMatrixIterativeSolver::SparseMatrixMultiplyTransposedSparseMatrix(double *sa, unsigned long *ija,
		double *sb, unsigned long *ijb, double thresh, unsigned long nmax, 
		double *sc, unsigned long *ijc) // threshold multiply
{
	unsigned long i,ijma,ijmb,j,k,ma,mb,mbb;
	double sum;

	if (ija[1] != ijb[1]) {
		printf("sprstm: sizes do not match");
		return;
	}

	ijc[1]=k=ija[1];
	for (i=1;i<=ija[1]-2;i++) {		// Loop over rows of A,
		for (j=1;j<=ijb[1]-2;j++) { // and rows of B.
			if (i == j) sum=sa[i]*sb[j]; else sum=0.0e0;
			mb=ijb[j];
			for (ma=ija[i];ma<=ija[i+1]-1;ma++) {
				// Loop through elements in A¡¯s row. Convoluted logic, following, accounts for the
				// various combinations of diagonal and off-diagonal elements.
				ijma=ija[ma];
				if (ijma == j) sum += sa[ma]*sb[j];
				else {
					while (mb < ijb[j+1]) {
						ijmb=ijb[mb];
						if (ijmb == i) {
							sum += sa[i]*sb[mb++];
							continue;
						} else if (ijmb < ijma) {
							mb++;
							continue;
						} else if (ijmb == ijma) {
							sum += sa[ma]*sb[mb++];
							continue;
						}
						break;
					}
				}
			}
			for (mbb=mb;mbb<=ijb[j+1]-1;mbb++) {	// Exhaust the remainder of B¡¯s row.
				if (ijb[mbb] == i) sum += sa[i]*sb[mbb];
			}
			if (i == j) sc[i]=sum;	// Where to put the answer...
			else if (fabs(sum) > thresh) {
				if (k > nmax) {
					printf("sprstm: nmax too small");	return;
				}
				sc[k]=sum;
				ijc[k++]=j;
			}
		}
		ijc[i+1]=k;
	}
}

void GLKSparseMatrixIterativeSolver::SparseMatrixMultiplyVector(double *sa, unsigned long *ija, double *x, 
													 double *b,	unsigned long n)
{
	unsigned long i,k;
	
	if (ija[1] != (n+2)) {printf("SparseMatrixMultiplyVector: mismatched vector and matrix");return;}
	for (i=1;i<=n;i++) {
		b[i]=sa[i]*x[i];					// Start with diagonal term.
		for (k=ija[i];k<=ija[i+1]-1;k++)	// Loop over off-diagonal terms.
			b[i] += sa[k]*x[ija[k]];
	}
}

void GLKSparseMatrixIterativeSolver::LinearPBCG(double *sa, unsigned long *ija, unsigned long n, double *b, double *x, 
									 int itol, double tol, int itmax, int &iter, double &err)
{
	unsigned long j;
	double ak,akden,bk,bkden,bknum,bnrm,dxnrm,xnrm,zm1nrm,znrm;
	double *p,*pp,*r,*rr,*z,*zz;			// Double precision is a good idea in this routine.
	
	p=new double[n+1];	for(j=0;j<=n;j++) p[j]=0.0;
	pp=new double[n+1];	for(j=0;j<=n;j++) pp[j]=0.0;
	r=new double[n+1];	for(j=0;j<=n;j++) r[j]=0.0;
	rr=new double[n+1];	for(j=0;j<=n;j++) rr[j]=0.0;
	z=new double[n+1];	for(j=0;j<=n;j++) z[j]=0.0;
	zz=new double[n+1];	for(j=0;j<=n;j++) zz[j]=0.0;

	// Calculate initial residual.
	iter=0;
	atimes(sa,ija,n,x,r,0);					// Input to atimes is x[1..n], output is r[1..n];
	for (j=1;j<=n;j++) {					// the final 0 indicates that the matrix (not its
		r[j]=b[j]-r[j];						// transpose) is to be used.
		rr[j]=r[j];
	}
	/* atimes(n,r,rr,0); */					// Uncomment this line to get the "minimum residual variant" of the algorithm.

	if (itol == 1) {
		bnrm=snrm(n,b,itol);
		asolve(sa,ija,n,r,z,0);				// Input to asolve is r[1..n], output is z[1..n];
	}										// the final 0 indicates that the matrix .A (not
	else if (itol == 2) {					// its transpose) is to be used.
		asolve(sa,ija,n,b,z,0);
		bnrm=snrm(n,z,itol);
		asolve(sa,ija,n,r,z,0);
	}
	else if (itol == 3 || itol == 4) {
		asolve(sa,ija,n,b,z,0);
		bnrm=snrm(n,z,itol);
		asolve(sa,ija,n,r,z,0);
		znrm=snrm(n,z,itol);
	} 
	else {printf("illegal itol in linbcg"); return;}
	
	while (iter <= itmax) {					// Main loop.
		++iter;
		asolve(sa,ija,n,rr,zz,1);			// Final 1 indicates use of transpose matrix (A')T.
		for (bknum=0.0,j=1;j<=n;j++) bknum += z[j]*rr[j];
		// Calculate coefficient bk and direction vectors p and pp.
		if (iter == 1) {
			for (j=1;j<=n;j++) {
				p[j]=z[j];
				pp[j]=zz[j];
			}
		}
		else {
			bk=bknum/bkden;
			for (j=1;j<=n;j++) {
				p[j]=bk*p[j]+z[j];
				pp[j]=bk*pp[j]+zz[j];
			}
		}
		bkden=bknum;					// Calculate coefficient ak, new iterate x, and new
		atimes(sa,ija,n,p,z,0);			// residuals r and rr.
		for (akden=0.0,j=1;j<=n;j++) akden += z[j]*pp[j];
		ak=bknum/akden;
		atimes(sa,ija,n,pp,zz,1);
		for (j=1;j<=n;j++) {
			x[j] += ak*p[j];
			r[j] -= ak*z[j];
			rr[j] -= ak*zz[j];
		}
		asolve(sa,ija,n,r,z,0);			// Solve A' z = r and check stopping criterion.
		if (itol == 1)
			err=snrm(n,r,itol)/bnrm;
		else if (itol == 2)
			err=snrm(n,z,itol)/bnrm;
		else if (itol == 3 || itol == 4) {
			zm1nrm=znrm;
			znrm=snrm(n,z,itol);
			if (fabs(zm1nrm-znrm) > EPS*znrm) {
				dxnrm=fabs(ak)*snrm(n,p,itol);
				err=znrm/fabs(zm1nrm-znrm)*dxnrm;
			} else {
				err=znrm/bnrm;				// Error may not be accurate, so loop again.
				continue;
			}
			xnrm=snrm(n,x,itol);	
			if (err <= 0.5*xnrm) 
				err /= xnrm;
			else {
				err=znrm/bnrm;				// Error may not be accurate, so loop again.
				continue;
			}
		}
//		printf("iter=%4d      err=%12.6f	bnrm=%lf	xnrm=%lf \n",iter,err,bnrm,xnrm);
		if (err <= tol) break;
	}
	
	delete []p;	delete []pp;	delete []r;	delete []rr;	delete []z; delete []zz;
}

void GLKSparseMatrixIterativeSolver::TransposeSparseMatrixMultiplyVector(double *sa, unsigned long *ija, 
															  double *x, double *b, unsigned long n)
{
	unsigned long i,j,k;

	if (ija[1] != (n+2)) {printf("mismatched vector and matrix in sprstx");return;}
	for (i=1;i<=n;i++) b[i]=sa[i]*x[i];		// Start with diagonal terms.
	for (i=1;i<=n;i++) {					// Loop over off-diagonal terms.
		for (k=ija[i];k<=ija[i+1]-1;k++) {
			j=ija[k];
			b[j] += sa[k]*x[i];
		}
	}
}

void GLKSparseMatrixIterativeSolver::asolve(double *sa, unsigned long *ija, unsigned long n, double *b, double *x, int itrnsp)
{
	unsigned long i;
	for(i=1;i<=n;i++) x[i]=((fabs(sa[i])> 1.0e-32 )?(b[i]/sa[i]):(b[i]));
	// The matrix A' is the diagonal part of A, stored in the first n elements of sa. Since the
	// transpose matrix has the same diagonal, the flag itrnsp is not used.
}

void GLKSparseMatrixIterativeSolver::atimes(double *sa, unsigned long *ija, unsigned long n, double *x, double *r, int itrnsp)
{
	if (itrnsp) 
		TransposeSparseMatrixMultiplyVector(sa,ija,x,r,n);
	else 
		SparseMatrixMultiplyVector(sa,ija,x,r,n);
}

double GLKSparseMatrixIterativeSolver::snrm(unsigned long n, double *sx, int itol)
{
	unsigned long i,isamax;
	double ans;
	
	if (itol <= 3) {
		ans = 0.0;
		for (i=1;i<=n;i++) 
			ans += sx[i]*sx[i];	// Vector magnitude norm.
		return sqrt(ans);
	} else {
		isamax=1;
		for (i=1;i<=n;i++) {					// Largest component norm.
		if (fabs(sx[i]) > fabs(sx[isamax])) isamax=i;
		}
		return fabs(sx[isamax]);
	}
}

void GLKSparseMatrixIterativeSolver::LinearPBCG(GLKSparseMatrix *sparseMatrix, double *B, double *X,
												int itol, double tol, int itmax, int &iter, double &err)
{
	double *sa,*x,*b;	
	unsigned long *ija;
	unsigned long i,nRow,nnzNum,index;
	double diagonalElement;

	//----------------------------------------------------------------------------------
	//	Build the input arrays for the LinearPBCG function
	nRow=sparseMatrix->GetRowNumber();		
	nnzNum=sparseMatrix->GetNonzeroElementNumber();
	//----------------------------------------------------------------------------------
	x=new double[nRow+1];	b=new double[nRow+1];
	sa=new double[nRow+nnzNum+2];	ija=new unsigned long[nRow+nnzNum+2];
	//----------------------------------------------------------------------------------
	for(i=0;i<nRow;i++) {b[i+1]=B[i];x[i+1]=X[i];}
	//----------------------------------------------------------------------------------
	index=nRow+2;
	for(i=0;i<nRow;i++) {
		diagonalElement=0.0;	ija[i+1]=index;

		GLKSparseMatrixNode *currentNode=(sparseMatrix->GetRowRoots())[i];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode *nextNode=currentNode->pNext;
			if (currentNode->nColIndex!=i) {
				ija[index]=currentNode->nColIndex+1; sa[index]=currentNode->data; index++;
			}
			else {
				diagonalElement=currentNode->data;
			}
			currentNode=nextNode;
		}
		sa[i+1]=diagonalElement;
	}
	ija[nRow+1]=index;

	//----------------------------------------------------------------------------------
	//	Solve the sparse linear equation system by the biconjugate gradient method
	LinearPBCG(sa, ija, nRow, b, x, itol, tol, itmax, iter, err);
	for(i=0;i<nRow;i++) X[i]=x[i+1];

	delete []x;		delete []b;		delete []ija;	delete []sa;
}

void GLKSparseMatrixIterativeSolver::LinearPBCG(GLKSparseMatrix *sparseMatrix, double *B,
												int itol, double tol, int itmax, 
												int &iter, double &err)
{
	double *sa,*x,*b;	
	unsigned long *ija;
	unsigned long i,nRow,nnzNum,index;
	double diagonalElement;

	//----------------------------------------------------------------------------------
	//	Build the input arrays for the LinearPBCG function
	nRow=sparseMatrix->GetRowNumber();		
	nnzNum=sparseMatrix->GetNonzeroElementNumber();
	//----------------------------------------------------------------------------------
	x=new double[nRow+1];	b=new double[nRow+1];
	sa=new double[nRow+nnzNum+2];	ija=new unsigned long[nRow+nnzNum+2];
	//----------------------------------------------------------------------------------
	for(i=0;i<nRow;i++) {b[i+1]=B[i];x[i+1]=B[i];}
	//----------------------------------------------------------------------------------
	index=nRow+2;
	for(i=0;i<nRow;i++) {
		diagonalElement=0.0;	ija[i+1]=index;

		GLKSparseMatrixNode *currentNode=(sparseMatrix->GetRowRoots())[i];
		for(;currentNode!=NULL;) {
			GLKSparseMatrixNode *nextNode=currentNode->pNext;
			if (currentNode->nColIndex!=i) {
				ija[index]=currentNode->nColIndex+1; sa[index]=currentNode->data; index++;
			}
			else {
				diagonalElement=currentNode->data;
			}
			currentNode=nextNode;
		}

		sa[i+1]=diagonalElement;
	}
	ija[nRow+1]=index;

	//----------------------------------------------------------------------------------
	//	Solve the sparse linear equation system by the biconjugate gradient method
	LinearPBCG(sa, ija, nRow, b, x, itol, tol, itmax, iter, err);
	for(i=0;i<nRow;i++) B[i]=x[i+1];

	delete []x;		delete []b;		delete []ija;	delete []sa;
}