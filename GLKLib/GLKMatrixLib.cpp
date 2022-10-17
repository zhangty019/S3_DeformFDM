// GLKMatrixLib.cpp: implementation of the GLKMatrixLib class.
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include <stdlib.h>

#include <math.h>
#include <stdio.h>

#include "GLKMatrixLib.h"

//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[] = __FILE__;
//#define new DEBUG_NEW
//#endif

//////////////////////////////////////////////////////////////////////
// For matrix container (GLKMatrix)
//////////////////////////////////////////////////////////////////////
GLKMatrix::GLKMatrix(int _m, int _n)
{
	mat = 0; m = 0; n = 0;
	CreateMatrix(_m, _n);
}

GLKMatrix::~GLKMatrix()
{
	DeleteMatrix();
}

void GLKMatrix::CreateMatrix(int _m, int _n) {
	if (_m <= 0 || _n <= 0) { printf("ERROR: Creating a %d x %d matrix!\n", _m, _n); return; }
	DeleteMatrix();

	m = _m; n = _n;
	GLKMatrixLib::CreateMatrix(mat, m, n);
}

void GLKMatrix::DeleteMatrix() {
	if (m>0 && n>0) GLKMatrixLib::DeleteMatrix(mat, m, n);
}

void GLKMatrix::PrintElements() {
	printf("GLKMatrix (%d x %d) Elements:\n", m, n);
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) printf("%f\t", mat[i][j]); printf("\n");
	}
}

GLKMatrix& GLKMatrix::operator=(const GLKMatrix &rhs) {
	if (m != rhs.m || n != rhs.n) {
		//printf("(GLKMatrix operator=)ERROR: dim different (%d %d) (%d %d)\n", m, rhs.m, n, rhs.n);
		//system("pause"); return *this;
		CreateMatrix(rhs.m, rhs.n);
	}
	for (int i = 0; i<m; i++) for (int j = 0; j< n; j++) mat[i][j] = rhs[i][j];
	return *this;  // Return a reference to myself.
}

GLKMatrix& GLKMatrix::operator=(const double &rhs) {
	for (int i = 0; i<m; i++) for (int j = 0; j<n; j++) mat[i][j] = rhs;
	return *this;  // Return a reference to myself.
}

GLKMatrix& GLKMatrix::operator=(double **rhs) {
	for (int i = 0; i<m; i++) for (int j = 0; j<n; j++) mat[i][j] = rhs[i][j];
	return *this;  // Return a reference to myself.
}

GLKMatrix GLKMatrix::operator+(GLKMatrix & rhs) {
	if (m != rhs.m || n != rhs.n) {
		printf("(GLKMatrix operator+)ERROR: dim different (%d %d) (%d %d)\n", m, rhs.m, n, rhs.n);
		system("pause"); return *this;
	}
	GLKMatrix result(m, n);
	for (int i = 0; i<m; i++) for (int j = 0; j<n; j++) result[i][j] = mat[i][j] + rhs[i][j];
	return result;
}

GLKMatrix GLKMatrix::operator*(const double &rhs) {
	GLKMatrix result(m, n);
	for (int i = 0; i<m; i++) for (int j = 0; j<n; j++) result[i][j] = mat[i][j] * rhs;
	return result;
}

GLKMatrix GLKMatrix::operator/(const double &rhs) {
	GLKMatrix result(m, n);
	for (int i = 0; i<m; i++) for (int j = 0; j<n; j++) result[i][j] = mat[i][j] / rhs;
	return result;
}

GLKMatrix& GLKMatrix::operator+=(GLKMatrix &rhs) {
	if (m != rhs.m || n != rhs.n) {
		printf("(GLKMatrix operator+=)ERROR: dim different (%d %d) (%d %d)\n", m, rhs.m, n, rhs.n);
		system("pause"); return *this;
	}
	for (int i = 0; i<m; i++) for (int j = 0; j<n; j++) mat[i][j] += rhs[i][j];
	return *this;
}

GLKMatrix& GLKMatrix::operator*=(const double &rhs) {
	for (int i = 0; i<m; i++) for (int j = 0; j<n; j++) mat[i][j] *= rhs;
	return *this;
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

GLKMatrixLib::GLKMatrixLib()
{

}

GLKMatrixLib::~GLKMatrixLib()
{

}

//////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////

void GLKMatrixLib::CreateMatrix(double** &a, int row, int col)
{
	int i,j;

	//a=(double**)new long[row];
	a = new double*[row];
	for(i=0;i<row;i++) {
		a[i]=new double[col];
		for(j=0;j<col;j++) 
			a[i][j]=0.0;
	}
}

void GLKMatrixLib::DeleteMatrix(double** &a, int row, int col)
{
	int i;

	for(i=0;i<row;i++) 
		delete [](double*)(a[i]);
	//delete [](double**)a;
	delete[]a;
	a = 0;
}

void GLKMatrixLib::CreateMatrix(bool** &a, int row, int col)
{
	int i,j;

	//a=(bool**)new long[row];
	a = new bool*[row];
	for(i=0;i<row;i++) {
		a[i]=new bool[col];
		for(j=0;j<col;j++) a[i][j]=false;
	}
}

void GLKMatrixLib::DeleteMatrix(bool** &a, int row, int col)
{
	int i;

	for(i=0;i<row;i++) delete [](bool*)(a[i]);
	delete [](bool**)a;
	a = 0;
}

void GLKMatrixLib::Transpose(double** inputMatrix, int n)
{
	int i,j;	double temp;

	for(i=0;i<n;i++) {
		for(j=i+1;j<n;j++) {
			temp=inputMatrix[i][j];
			inputMatrix[i][j]=inputMatrix[j][i];
			inputMatrix[j][i]=temp;
		}
	}
}

void GLKMatrixLib::Transpose(double** inputMatrix, int row, int col, 
							double** outputMatrix /* with (col x row) */)
{
	int i,j;

	for(i=0;i<row;i++)
		for(j=0;j<col;j++)
			outputMatrix[j][i]=inputMatrix[i][j];
}

bool GLKMatrixLib::Pseudoinverse(double** inputMatrix, int row, int col, 
								double** outputMatrix /* with (col x row) */)
{
	double **tA,**cA;
	int i,j;

	tA=(double**)new long[col];
	for(i=0;i<col;i++) tA[i]=new double[row];

	cA=(double**)new long[col];
	for(i=0;i<col;i++) cA[i]=new double[col];

	for(i=0;i<row;i++)
		for(j=0;j<col;j++)
			tA[j][i]=inputMatrix[i][j];

	Mul(tA,inputMatrix,col,row,col,cA);
	if (!Inverse(cA,col)) {
		for(i=0;i<col;i++) delete [](double*)(tA[i]);
		delete [](double**)tA;
		for(i=0;i<col;i++) delete [](double*)(cA[i]);
		delete [](double**)cA;
		return false;
	}
	Mul(cA,tA,col,col,row,outputMatrix);

	for(i=0;i<col;i++) delete [](double*)(tA[i]);
	delete [](double**)tA;
	for(i=0;i<col;i++) delete [](double*)(cA[i]);
	delete [](double**)cA;

	return true;
}

int GLKMatrixLib::Rank(double**a /*m by n*/, int m, int n)
{
	int i,j,k,nn,is,js,l;
    double q,d;

    nn=m;
    if (m>=n) nn=n;
    k=0;
    for (l=0; l<=nn-1; l++)
      { q=0.0;
        for (i=l; i<=m-1; i++)
        for (j=l; j<=n-1; j++)
          { d=fabs(a[i][j]);
	    if (d>q) { q=d; is=i; js=j;}
          }
        if (q+1.0==1.0) return(k);
        k=k+1;
        if (is!=l)
          { for (j=l; j<=n-1; j++)
              {
                d=a[l][j]; a[l][j]=a[is][j]; a[is][j]=d;
              }
          }
        if (js!=l)
          { for (i=l; i<=m-1; i++)
              {
                d=a[i][js]; a[i][js]=a[i][l]; a[i][l]=d;
              }
          }
        for (i=l+1; i<=n-1; i++)
          { d=a[i][l]/a[l][l];
            for (j=l+1; j<=n-1; j++)
              {
                a[i][j]=a[i][j]-d*a[l][j];
              }
          }
      }

    return k;
}

void GLKMatrixLib::Mul(double**a /*m by n*/, double*b  /*n rows*/, int m, int n, double*c /*m rows*/)
{
	int i,j;

	for(i=0;i<m;i++) {
		c[i]=0.0;
		for(j=0;j<n;j++) {
			c[i]=c[i]+a[i][j]*b[j];
		}
	}
}

void GLKMatrixLib::Mul(double**a /*m by n*/, double**b  /*n by k*/, int m, int n, int k, 
					double**c /*m by k*/)
{
	int i,j,l;

    for(i=0;i<m;i++) {
		for(j=0;j<k;j++) {
			c[i][j]=0.0;
			for (l=0;l<n;l++) c[i][j]=c[i][j]+a[i][l]*b[l][j];
		}
	}
}

void GLKMatrixLib::SwitchCol(double** a, int row, int col, int colIndex1, int colIndex2)
{
	double* tempV=new double[row];
	int i;

	for(i=0;i<row;i++) tempV[i]=a[i][colIndex1];
	for(i=0;i<row;i++) a[i][colIndex1]=a[i][colIndex2];
	for(i=0;i<row;i++) a[i][colIndex2]=tempV[i];

	delete []tempV;
}

void GLKMatrixLib::SwitchRow(double** a, int row, int col, int rowIndex1, int rowIndex2)
{
	double* tempV=new double[col];
	int j;

	for(j=0;j<col;j++) tempV[j]=a[rowIndex1][j];
	for(j=0;j<col;j++) a[rowIndex1][j]=a[rowIndex2][j];
	for(j=0;j<col;j++) a[rowIndex2][j]=tempV[j];

	delete []tempV;
}

void GLKMatrixLib::ComputeEigenvectorByEigenvalue(double**a, int n, 
												  double eigenvalue, double *eigenvector)
{
	double **AA,*b;	int i,j,k;	double factor,sum;

	b=new double[n]; CreateMatrix(AA,n,n);

	for(i=0;i<n;i++) {
		b[i]=0.0;
		for(j=0;j<n;j++) AA[i][j]=a[i][j];
		AA[i][i]=AA[i][i]-eigenvalue;
	}

	//--------------------------------------------------------------------
	//	forward elimination
	for(k=0;k<n-1;k++) {
		for(i=k+1;i<n;i++) {
			factor=AA[i][k]/AA[k][k];
			AA[i][k]=0.0;
			for(j=k+1;j<n;j++) {
				AA[i][j]=AA[i][j]-factor*AA[k][j];
			}
			b[i]=b[i]-factor*b[k];
		}
	}

/*	for(i=0;i<n;i++) {
		for(j=0;j<n;j++) {
			printf("%.5lf ",AA[i][j]);
		}
		printf("\n");
	}
	for(j=0;j<n;j++) printf("%.5lf ",b[j]);*/

	//--------------------------------------------------------------------
	//	backward substitution
	eigenvector[n-1]=1.0;
	for(i=n-2;i>=0;i--) {
		sum=0.0;
		for(j=i+1;j<n;j++) {
			sum=sum+AA[i][j]*eigenvector[j];
		}
		eigenvector[i]=(b[i]-sum)/AA[i][i];
	}

/*	for(i=0;i<n;i++) {
		b[i]=0.0;
		for(j=0;j<n;j++) {
			AA[i][j]=a[i][j];
			if (i==j) AA[i][j]-=eigenvalue;
		}
	}
	Mul(AA,eigenvector,5,5,b);
	for(j=0;j<n;j++) printf("%.5lf ",b[j]);*/

	//--------------------------------------------------------------------
	//	normalization
	sum=0.0;
	for(i=0;i<n;i++) sum+=eigenvector[i]*eigenvector[i];
	sum=sqrt(sum);
	for(i=0;i<n;i++) eigenvector[i]=eigenvector[i]/sum;

	delete []b; DeleteMatrix(AA,n,n);
}

bool GLKMatrixLib::HessenbergQREigenvaluesComputing(double**a, int n, 
													double *eigenvalues_Re, double *eigenvalues_Im,
													double eps, int maxIter)
{
	double *hMatrix;
	int i,j;

	hMatrix=new double[n*n];
	for(i=0;i<n;i++) for(j=0;j<n;j++) hMatrix[i*n+j]=a[i][j];

	_HessenbergConversion(hMatrix,n);
	bool bRC=_Hessenberg_QR_Eigen(hMatrix,n,eigenvalues_Re,eigenvalues_Im,eps,maxIter);

	delete []hMatrix;

	return bRC;
}

bool GLKMatrixLib::JacobianEigensystemSolver(double**a, int n, double **v, double *eigenvalues, double eps, int maxIter)
{
	int i,j,p,q,l;//u,w,t,s
	double fm,cn,sn,omega,x,y,d;
	
	l=1;
	for (i=0; i<=n-1; i++) { 
		v[i][i]=1.0;
		for (j=0; j<=n-1; j++)
			if (i!=j) v[i][j]=0.0;
    }

	while (1==1) { 
		fm=0.0;
		for (i=1; i<=n-1; i++)
			for (j=0; j<=i-1; j++) { 
				d=fabs(a[i][j]);
				if ((i!=j)&&(d>fm))	{ fm=d; p=i; q=j;}
			}
			if (fm<eps) {for(j=0;j<n;j++) eigenvalues[j]=a[j][j];return true;}
			if (l>maxIter)  return false;
			l=l+1;
			x=-a[p][q]; y=(a[q][q]-a[p][p])/2.0;
			omega=x/sqrt(x*x+y*y);
			if (y<0.0) omega=-omega;
			sn=1.0+sqrt(1.0-omega*omega);
			sn=omega/sqrt(2.0*sn);
			cn=sqrt(1.0-sn*sn);
			fm=a[p][p];
			a[p][p]=fm*cn*cn+a[q][q]*sn*sn+a[p][q]*omega;
			a[q][q]=fm*sn*sn+a[q][q]*cn*cn-a[p][q]*omega;
			a[p][q]=0.0; a[q][p]=0.0;
			
			for (j=0; j<=n-1; j++)
				if ((j!=p)&&(j!=q))
				{ 
					//u=p*n+j; w=q*n+j;
					fm=a[p][j];
					a[p][j]=fm*cn+a[q][j]*sn;
					a[q][j]=-fm*sn+a[q][j]*cn;
				}

			for (i=0; i<=n-1; i++)
				if ((i!=p)&&(i!=q))
				{ 
					//u=i*n+p; w=i*n+q;
					fm=a[i][p];
					a[i][p]=fm*cn+a[i][q]*sn;
					a[i][q]=-fm*sn+a[i][q]*cn;
				}

			for (i=0; i<=n-1; i++) { 
				//u=i*n+p; w=i*n+q;
				fm=v[i][p];
				v[i][p]=fm*cn+v[i][q]*sn;
				v[i][q]=-fm*sn+v[i][q]*cn;
			}
    }
	
	for(j=0;j<n;j++) eigenvalues[j]=a[j][j];
	return true;
}

void GLKMatrixLib::ConjugateGradientSolver(double** a, double* b, int n, double* x, double eps)
{
	int i,k;
    double *p,*r,*s,*q,alpha,beta,d,e;

    p=new double[n];
    r=new double[n];
    s=new double[n];
    q=new double[n];

    for (i=0; i<=n-1; i++)
      { x[i]=0.0; p[i]=b[i]; r[i]=b[i]; }
    i=0;
    while (i<=n-1) { 
		Mul(a,p,n,n,s);
        d=0.0; e=0.0;
        for (k=0; k<=n-1; k++)
           { d=d+p[k]*b[k]; e=e+p[k]*s[k]; }
        alpha=d/e;
        for (k=0; k<=n-1; k++)
           x[k]=x[k]+alpha*p[k];
		Mul(a,x,n,n,q);
        d=0.0;
        for (k=0; k<=n-1; k++)
          { r[k]=b[k]-q[k]; d=d+r[k]*s[k]; }
        beta=d/e; d=0.0;
        for (k=0; k<=n-1; k++) d=d+r[k]*r[k];
        d=sqrt(d);
        if (d<eps) { delete []p; delete []r; delete []s; delete []q; return;}
        for (k=0; k<=n-1; k++)
           p[k]=r[k]-beta*p[k];
        i=i+1;
      }
 
	delete []p; delete []r; delete []s; delete []q;
}

bool GLKMatrixLib::GaussSeidelSolver(double** a, double* b, int n, double* x, double eps)
{
	int i,j;
    double p,t,s,q;

    for (i=0; i<=n-1; i++) { 
		p=0.0; x[i]=0.0;
        for (j=0; j<=n-1; j++)
			if (i!=j) p=p+fabs(a[i][j]);
		if (p>=fabs(a[i][i])) return false;
	}
    p=eps+1.0;
    while (p>=eps){ 
		p=0.0;
        for (i=0; i<=n-1; i++)
          { t=x[i]; s=0.0;
            for (j=0; j<=n-1; j++)
              if (j!=i) s=s+a[i][j]*x[j];
            x[i]=(b[i]-s)/a[i][i];
            q=fabs(x[i]-t)/(1.0+fabs(x[i]));
            if (q>p) p=q;
          }
    }
	return true;
}
	
void GLKMatrixLib::SVDSolver(double** TT, int n, double* R, double criterion)
{
	double **UU,**UUT,**VV,**VVT,*X;
	int i;

	CreateMatrix(UU,n,n);	CreateMatrix(UUT,n,n);
	CreateMatrix(VV,n,n);	CreateMatrix(VVT,n,n);
	X=new double[n];

	SingularValueDecomposition(TT,n,n,UU,VVT);
	Transpose(UU,n,n,UUT);	Transpose(VVT,n,n,VV);
	for(i=0;i<n;i++) {
		for(int j=0;j<n;j++) {
			if (i!=j) {	TT[i][j]=0.0; }
			else {
				if (TT[i][j]<criterion) TT[i][j]=0.0; else TT[i][j]=1.0/TT[i][j];
			}
		}
	}
	Mul(UUT,R,n,n,X);	Mul(TT,X,n,n,R);	Mul(VV,R,n,n,X);
	for(i=0;i<n;i++) R[i]=X[i];
			
	delete []X;
	DeleteMatrix(UU,n,n);	DeleteMatrix(UUT,n,n);
	DeleteMatrix(VV,n,n);	DeleteMatrix(VVT,n,n);
}

bool GLKMatrixLib::CholeskyDecompositionSolver(double** a, int n, double* b)
{
	double **bb;	int i;	bool bFlag;

	CreateMatrix(bb,n,1);
	for(i=0;i<n;i++) bb[i][0]=b[i];

	bFlag=CholeskyDecompositionSolver(a,n,bb,1);

	for(i=0;i<n;i++) b[i]=bb[i][0];
	DeleteMatrix(bb,n,1);

	return bFlag;
}

bool GLKMatrixLib::CholeskyDecompositionSolver(double** a, int n, double** d, int m)
{
	int i,j,k;
	if ((a[0][0]+1.0==1.0)||(a[0][0]<0.0)) return false;
	a[0][0]=sqrt(a[0][0]);
	for (j=1; j<=n-1; j++) a[0][j]=a[0][j]/a[0][0];
	for (i=1; i<=n-1; i++) {
		for (j=1; j<=i; j++) {
			a[i][i]=a[i][i]-a[j-1][i]*a[j-1][i];
		}
	    if ((a[i][i]+1.0==1.0)||(a[i][i]<0.0)) return false;
		a[i][i]=sqrt(a[i][i]);
		if (i!=(n-1)) { 
			for (j=i+1; j<=n-1; j++) { 
				for (k=1; k<=i; k++) a[i][j]=a[i][j]-a[k-1][i]*a[k-1][j];
				a[i][j]=a[i][j]/a[i][i];
            }
        }
	}
	for (j=0; j<=m-1; j++) { 
		d[0][j]=d[0][j]/a[0][0];
		for (i=1; i<=n-1; i++) { 
			for (k=1; k<=i; k++) d[i][j]=d[i][j]-a[k-1][i]*d[k-1][j];
			d[i][j]=d[i][j]/a[i][i];
        }
    }
	for (j=0; j<=m-1; j++) { 
		d[n-1][j]=d[n-1][j]/a[n-1][n-1];
		for (k=n-1; k>=1; k--) { 
			for (i=k; i<=n-1; i++) d[k-1][j]=d[k-1][j]-a[k-1][i]*d[i][j];
			d[k-1][j]=d[k-1][j]/a[k-1][k-1];
        }
    }
	return true;
}

bool GLKMatrixLib::GaussJordanElimination(double** a, int n, double* b)
{
	double **bb;	int i;	bool bFlag;

	CreateMatrix(bb,n,1);
	for(i=0;i<n;i++) bb[i][0]=b[i];

	bFlag=GaussJordanElimination(a,n,bb,1);

	for(i=0;i<n;i++) b[i]=bb[i][0];
	DeleteMatrix(bb,n,1);

	return bFlag;
}

bool GLKMatrixLib::GaussJordanElimination(double** a, int n, double** b, int m)
{
	int *js,l,k,i,j,is;
    double d,t;

    js=new int [n];
    l=1;
    for (k=0;k<=n-1;k++)
      { d=0.0;
        for (i=k;i<=n-1;i++)
          for (j=k;j<=n-1;j++)
            { t=fabs(a[i][j]);
              if (t>d) { d=t; js[k]=j; is=i;}
            }
        if (d+1.0==1.0) l=0;
        else
          { if (js[k]!=k)
              for (i=0;i<=n-1;i++)
                { 
                  t=a[i][k]; a[i][k]=a[i][js[k]]; a[i][js[k]]=t;
                }
            if (is!=k)
              { for (j=k;j<=n-1;j++)
                  {
                    t=a[k][j]; a[k][j]=a[is][j]; a[is][j]=t;
                  }
                for (j=0;j<=m-1;j++)
                  { 
                    t=b[k][j]; b[k][j]=b[is][j]; b[is][j]=t;
                  }
              }
          }
        if (l==0)
          { delete []js;
            return false;
          }
        d=a[k][k];
        for (j=k+1;j<=n-1;j++)
          { a[k][j]=a[k][j]/d;}
        for (j=0;j<=m-1;j++)
          { b[k][j]=b[k][j]/d;}
        for (j=k+1;j<=n-1;j++)
          for (i=0;i<=n-1;i++)
            {
              if (i!=k)
                a[i][j]=a[i][j]-a[i][k]*a[k][j];
            }
        for (j=0;j<=m-1;j++)
        for (i=0;i<=n-1;i++)
          {
            if (i!=k)
              b[i][j]=b[i][j]-a[i][k]*b[k][j];
          }
      }
    for (k=n-1;k>=0;k--)
      if (js[k]!=k)
        for (j=0;j<=m-1;j++)
          { 
            t=b[k][j]; b[k][j]=b[js[k]][j]; b[js[k]][j]=t;
          }

    delete []js;
    return true;
}

bool GLKMatrixLib::Inverse(double**a, int n)
{
	int *is,*js,i,j,k;
    double d,p;

    is=new int[n];
    js=new int[n];

    for (k=0; k<=n-1; k++)
      { d=0.0;
        for (i=k; i<=n-1; i++)
        for (j=k; j<=n-1; j++)
          { 
			p=fabs(a[i][j]);
            if (p>d) { d=p; is[k]=i; js[k]=j;}
          }
        if (d+1.0==1.0)
          { 
			delete []is;	delete []js;
            return false;
          }
        if (is[k]!=k)
          for (j=0; j<=n-1; j++) { 
              p=a[k][j]; a[k][j]=a[is[k]][j]; a[is[k]][j]=p;
            }
        if (js[k]!=k)
          for (i=0; i<=n-1; i++) { 
              p=a[i][k]; a[i][k]=a[i][js[k]]; a[i][js[k]]=p;
            }

        a[k][k]=1.0/a[k][k];
        for (j=0; j<=n-1; j++)
          if (j!=k) {a[k][j]=a[k][j]*a[k][k];}

        for (i=0; i<=n-1; i++)
          if (i!=k)
            for (j=0; j<=n-1; j++)
              if (j!=k)
				  a[i][j]=a[i][j]-a[i][k]*a[k][j];

        for (i=0; i<=n-1; i++)
          if (i!=k) a[i][k]=-a[i][k]*a[k][k];
      }

    for (k=n-1; k>=0; k--)
      { if (js[k]!=k)
          for (j=0; j<=n-1; j++)
            { 
			  p=a[k][j]; a[k][j]=a[js[k]][j]; a[js[k]][j]=p;
            }
        if (is[k]!=k)
          for (i=0; i<=n-1; i++)
            { 
              p=a[i][k]; a[i][k]=a[i][is[k]]; a[i][is[k]]=p;
            }
      }

	delete []is;	delete []js;
    return true;
}

bool GLKMatrixLib::SingularValueDecomposition(double**a /*m by n*/, int m, int n, 
											  double**u /*m by m*/, double**v /*n by n*/, 
											  double eps)
{
	int i,j,k,l,it,ll,kk,mm,nn,m1,ks,ka;
    double d,dd,t,sm,sm1,em1,sk,ek,b,c,shh,fg[2],cs[2];
    double *s,*e,*w;

	ka=((m>n)?m:n)+1;
    
	s=new double[ka];
    e=new double[ka];
    w=new double[ka];
    
	it=60; k=n;
    if (m-1<n) k=m-1;
    l=m;
    if (n-2<m) l=n-2;
    if (l<0) l=0;
    ll=k;
    if (l>k) ll=l;
    if (ll>=1)
      { for (kk=1; kk<=ll; kk++)
          { if (kk<=k)
              { d=0.0;
                for (i=kk; i<=m; i++)
                  { d=d+a[i-1][kk-1]*a[i-1][kk-1];}
                s[kk-1]=sqrt(d);
                if (s[kk-1]!=0.0)
                  { 
                    if (a[kk-1][kk-1]!=0.0)
                      { s[kk-1]=fabs(s[kk-1]);
                        if (a[kk-1][kk-1]<0.0) s[kk-1]=-s[kk-1];
                      }
                    for (i=kk; i<=m; i++)
                      { 
                        a[i-1][kk-1]=a[i-1][kk-1]/s[kk-1];
                      }
                    a[kk-1][kk-1]=1.0+a[kk-1][kk-1];
                  }
                s[kk-1]=-s[kk-1];
              }
            if (n>=kk+1)
              { for (j=kk+1; j<=n; j++)
                  { if ((kk<=k)&&(s[kk-1]!=0.0))
                      { d=0.0;
                        for (i=kk; i<=m; i++)
                          {
                            d=d+a[i-1][kk-1]*a[i-1][j-1];
                          }
                        d=-d/a[kk-1][kk-1];
                        for (i=kk; i<=m; i++)
                          {
                            a[i-1][j-1]=a[i-1][j-1]+d*a[i-1][kk-1];
                          }
                      }
                    e[j-1]=a[kk-1][j-1];
                  }
              }
            if (kk<=k)
              { for (i=kk; i<=m; i++)
                  {
                    u[i-1][kk-1]=a[i-1][kk-1];
                  }
              }
            if (kk<=l)
              { d=0.0;
                for (i=kk+1; i<=n; i++)
                  d=d+e[i-1]*e[i-1];
                e[kk-1]=sqrt(d);
                if (e[kk-1]!=0.0)
                  { if (e[kk]!=0.0)
                      { e[kk-1]=fabs(e[kk-1]);
                        if (e[kk]<0.0) e[kk-1]=-e[kk-1];
                      }
                    for (i=kk+1; i<=n; i++)
                      e[i-1]=e[i-1]/e[kk-1];
                    e[kk]=1.0+e[kk];
                  }
                e[kk-1]=-e[kk-1];
                if ((kk+1<=m)&&(e[kk-1]!=0.0))
                  { for (i=kk+1; i<=m; i++) w[i-1]=0.0;
                    for (j=kk+1; j<=n; j++)
                      for (i=kk+1; i<=m; i++)
                        w[i-1]=w[i-1]+e[j-1]*a[i-1][j-1];
                    for (j=kk+1; j<=n; j++)
                      for (i=kk+1; i<=m; i++)
                        {
                          a[i-1][j-1]=a[i-1][j-1]-w[i-1]*e[j-1]/e[kk];
                        }
                  }
                for (i=kk+1; i<=n; i++)
                  v[i-1][kk-1]=e[i-1];
              }
          }
      }
    mm=n;
    if (m+1<n) mm=m+1;
    if (k<n) s[k]=a[k][k];
    if (m<mm) s[mm-1]=0.0;
    if (l+1<mm) e[l]=a[l][mm-1];
    e[mm-1]=0.0;
    nn=m;
    if (m>n) nn=n;
    if (nn>=k+1)
      { for (j=k+1; j<=nn; j++)
          { for (i=1; i<=m; i++)
              u[i-1][j-1]=0.0;
            u[j-1][j-1]=1.0;
          }
      }
    if (k>=1)
      { for (ll=1; ll<=k; ll++)
          { kk=k-ll+1;
            if (s[kk-1]!=0.0)
              { if (nn>=kk+1)
                  for (j=kk+1; j<=nn; j++)
                    { d=0.0;
                      for (i=kk; i<=m; i++)
                        {
                          d=d+u[i-1][kk-1]*u[i-1][j-1]/u[kk-1][kk-1];
                        }
                      d=-d;
                      for (i=kk; i<=m; i++)
                        { 
                          u[i-1][j-1]=u[i-1][j-1]+d*u[i-1][kk-1];
                        }
                    }
                  for (i=kk; i<=m; i++)
                    { u[i-1][kk-1]=-u[i-1][kk-1];}
                  u[kk-1][kk-1]=1.0+u[kk-1][kk-1];
                  if (kk-1>=1)
                    for (i=1; i<=kk-1; i++)
                      u[i-1][kk-1]=0.0;
              }
            else
              { for (i=1; i<=m; i++)
                  u[i-1][kk-1]=0.0;
                u[kk-1][kk-1]=1.0;
              }
          }
      }
    for (ll=1; ll<=n; ll++)
      { kk=n-ll+1;
        if ((kk<=l)&&(e[kk-1]!=0.0))
          { for (j=kk+1; j<=n; j++)
              { d=0.0;
                for (i=kk+1; i<=n; i++)
                  { 
                    d=d+v[i-1][kk-1]*v[i-1][j-1]/v[kk][kk-1];
                  }
                d=-d;
                for (i=kk+1; i<=n; i++)
                  { 
                    v[i-1][j-1]=v[i-1][j-1]+d*v[i-1][kk-1];
                  }
              }
          }
        for (i=1; i<=n; i++)
          v[i-1][kk-1]=0.0;
        v[kk-1][kk-1]=1.0;
      }
    for (i=1; i<=m; i++)
    for (j=1; j<=n; j++)
      a[i-1][j-1]=0.0;
    m1=mm; it=60;
    while (1==1)
      { if (mm==0)
          { ppp(a,e,s,v,m,n);
            delete []s; delete []e; delete []w; return true;
          }
        if (it==0)
          { ppp(a,e,s,v,m,n);
            delete []s; delete []e; delete []w; return false;
          }
        kk=mm-1;
	while ((kk!=0)&&(fabs(e[kk-1])!=0.0))
          { d=fabs(s[kk-1])+fabs(s[kk]);
            dd=fabs(e[kk-1]);
            if (dd>eps*d) kk=kk-1;
            else e[kk-1]=0.0;
          }
        if (kk==mm-1)
          { kk=kk+1;
            if (s[kk-1]<0.0)
              { s[kk-1]=-s[kk-1];
                for (i=1; i<=n; i++)
                  { v[i-1][kk-1]=-v[i-1][kk-1];}
              }
            while ((kk!=m1)&&(s[kk-1]<s[kk]))
              { d=s[kk-1]; s[kk-1]=s[kk]; s[kk]=d;
                if (kk<n)
                  for (i=1; i<=n; i++)
                    {
                      d=v[i-1][kk-1]; v[i-1][kk-1]=v[i-1][kk]; v[i-1][kk]=d;
                    }
                if (kk<m)
                  for (i=1; i<=m; i++)
                    {
                      d=u[i-1][kk-1]; u[i-1][kk-1]=u[i-1][kk]; u[i-1][kk]=d;
                    }
                kk=kk+1;
              }
            it=60;
            mm=mm-1;
          }
        else
          { ks=mm;
            while ((ks>kk)&&(fabs(s[ks-1])!=0.0))
              { d=0.0;
                if (ks!=mm) d=d+fabs(e[ks-1]);
                if (ks!=kk+1) d=d+fabs(e[ks-2]);
                dd=fabs(s[ks-1]);
                if (dd>eps*d) ks=ks-1;
                else s[ks-1]=0.0;
              }
            if (ks==kk)
              { kk=kk+1;
                d=fabs(s[mm-1]);
                t=fabs(s[mm-2]);
                if (t>d) d=t;
                t=fabs(e[mm-2]);
                if (t>d) d=t;
                t=fabs(s[kk-1]);
                if (t>d) d=t;
                t=fabs(e[kk-1]);
                if (t>d) d=t;
                sm=s[mm-1]/d; sm1=s[mm-2]/d;
                em1=e[mm-2]/d;
                sk=s[kk-1]/d; ek=e[kk-1]/d;
                b=((sm1+sm)*(sm1-sm)+em1*em1)/2.0;
                c=sm*em1; c=c*c; shh=0.0;
                if ((b!=0.0)||(c!=0.0))
                  { shh=sqrt(b*b+c);
                    if (b<0.0) shh=-shh;
                    shh=c/(b+shh);
                  }
                fg[0]=(sk+sm)*(sk-sm)-shh;
                fg[1]=sk*ek;
                for (i=kk; i<=mm-1; i++)
                  { sss(fg,cs);
                    if (i!=kk) e[i-2]=fg[0];
                    fg[0]=cs[0]*s[i-1]+cs[1]*e[i-1];
                    e[i-1]=cs[0]*e[i-1]-cs[1]*s[i-1];
                    fg[1]=cs[1]*s[i];
                    s[i]=cs[0]*s[i];
                    if ((cs[0]!=1.0)||(cs[1]!=0.0))
                      for (j=1; j<=n; j++)
                        {
                          d=cs[0]*v[j-1][i-1]+cs[1]*v[j-1][i];
                          v[j-1][i]=-cs[1]*v[j-1][i-1]+cs[0]*v[j-1][i];
                          v[j-1][i-1]=d;
                        }
                    sss(fg,cs);
                    s[i-1]=fg[0];
                    fg[0]=cs[0]*e[i-1]+cs[1]*s[i];
                    s[i]=-cs[1]*e[i-1]+cs[0]*s[i];
                    fg[1]=cs[1]*e[i];
                    e[i]=cs[0]*e[i];
                    if (i<m)
                      if ((cs[0]!=1.0)||(cs[1]!=0.0))
                        for (j=1; j<=m; j++)
                          {
                            d=cs[0]*u[j-1][i-1]+cs[1]*u[j-1][i];
                            u[j-1][i]=-cs[1]*u[j-1][i-1]+cs[0]*u[j-1][i];
                            u[j-1][i-1]=d;
                          }
                  }
                e[mm-2]=fg[0];
                it=it-1;
              }
            else
              { if (ks==mm)
                  { kk=kk+1;
                    fg[1]=e[mm-2]; e[mm-2]=0.0;
                    for (ll=kk; ll<=mm-1; ll++)
                      { i=mm+kk-ll-1;
                        fg[0]=s[i-1];
                        sss(fg,cs);
                        s[i-1]=fg[0];
                        if (i!=kk)
                          { fg[1]=-cs[1]*e[i-2];
                            e[i-2]=cs[0]*e[i-2];
                          }
                        if ((cs[0]!=1.0)||(cs[1]!=0.0))
                          for (j=1; j<=n; j++)
                            {
                              d=cs[0]*v[j-1][i-1]+cs[1]*v[j-1][mm-1];
                              v[j-1][mm-1]=-cs[1]*v[j-1][i-1]+cs[0]*v[j-1][mm-1];
                              v[j-1][i-1]=d;
                            }
                      }
                  }
                else
                  { kk=ks+1;
                    fg[1]=e[kk-2];
                    e[kk-2]=0.0;
                    for (i=kk; i<=mm; i++)
                      { fg[0]=s[i-1];
                        sss(fg,cs);
                        s[i-1]=fg[0];
                        fg[1]=-cs[1]*e[i-1];
                        e[i-1]=cs[0]*e[i-1];
                        if ((cs[0]!=1.0)||(cs[1]!=0.0))
                          for (j=1; j<=m; j++)
                            {
                              d=cs[0]*u[j-1][i-1]+cs[1]*u[j-1][kk-2];
                              u[j-1][kk-2]=-cs[1]*u[j-1][i-1]+cs[0]*u[j-1][kk-2];
                              u[j-1][i-1]=d;
                            }
                      }
                  }
              }
          }
      }

	  return true;
}

bool GLKMatrixLib::SingularValueDecomposition_2by2(double** a, double** u, double** v)
{
    double lambda[2], sigma[2];
    double v1[2], v2[2], u1[2], u2[2], ATA[2][2];
    ATA[0][0] = a[0][0]*a[0][0]+a[1][0]*a[1][0];
    ATA[0][1]= a[0][0]*a[0][1]+a[1][0]*a[1][1];
    ATA[1][0] = a[0][1]*a[0][0]+a[1][1]*a[1][0];
    ATA[1][1] = a[0][1]*a[0][1]+a[1][1]*a[1][1];
    EigenDecomposition_2by2(ATA, lambda[0], lambda[1], v1, v2);

    sigma[0] = sqrt(lambda[0]); sigma[1] = sqrt(lambda[1]);

    if(sigma[0]==0){
        u1[0] = u1[1] = 0.0;
    }
    else{
        u1[0] = (a[0][0]*v1[0]+a[0][1]*v1[1])/sigma[0];
        u1[1] = (a[1][0]*v1[0]+a[1][1]*v1[1])/sigma[0];
        _normalize(2, u1);
    }
    if(sigma[1]==0){
        u2[0] = u2[1] = 0.0;
    }
    else{
        u2[0] = (a[0][0]*v2[0]+a[0][1]*v2[1])/sigma[1];
        u2[1] = (a[1][0]*v2[0]+a[1][1]*v2[1])/sigma[1];
        _normalize(2, u2);
    }
    u[0][0] = u1[0]; u[0][1] = u2[0]; u[1][0] = u1[1]; u[1][1] = u2[1];
    v[0][0] = v1[0]; v[0][1] = v1[1]; v[1][0] = v2[0]; v[1][1] = v2[1];
    a[0][0] = sigma[0]; a[1][1] = sigma[1]; a[0][1] = a[1][0] = 0.0;
    return true;
}

bool GLKMatrixLib::EigenDecomposition_2by2(double ATA[2][2], double& eigenValue1, double& eigenValue2,
                                           double eignVec1[], double eignVec2[])
{
    double A, B, C, D;
    A = ATA[0][0]; B = ATA[0][1]; C = ATA[1][0]; D = ATA[1][1];
    if((B*C)<=1.0E-8){
        eigenValue1 = A; eignVec1[0] = 1.0; eignVec1[1] = 0.0;
        eigenValue2 = D; eignVec2[0] = 0.0; eignVec2[1] = 1.0;
        return true;
    }
    double tr = A+D;
    double det = A*D-B*C;
    double S = sqrt(fmax((tr/2.0)*(tr/2.0)-det, 0.0));
    eigenValue1 = tr/2.0+S;
    eigenValue2 = tr/2.0-S;

    double SS = sqrt(fmax(((A-D)/2)*((A-D)/2)+B*C, 0.0));
    if(A-D<0){
        eignVec1[0] = C;
        eignVec1[1] = -(A-D)/2+SS;
        eignVec2[0] = (A-D)/2-SS;
        eignVec2[1] = B;
    }
    else{
        eignVec2[0] = C;
        eignVec2[1] = -(A-D)/2-SS;
        eignVec1[0] = (A-D)/2+SS;
        eignVec1[1] = B;
    }
    _normalize(2, eignVec1); _normalize(2, eignVec2);
    return true;
}

bool GLKMatrixLib::_Hessenberg_QR_Eigen(double* a, int n, double *u, double *v, double eps, int jt)
{
	int m,it,i,j,k,l,ii,jj,kk,ll;
  double b,c,w,g,xy,p,q,r,x,s,e,f,z,y;
  it=0; m=n;
  while (m!=0)
    { l=m-1;
      while ((l>0)&&(fabs(a[l*n+l-1])>eps*
	      (fabs(a[(l-1)*n+l-1])+fabs(a[l*n+l])))) l=l-1;
      ii=(m-1)*n+m-1; jj=(m-1)*n+m-2;
      kk=(m-2)*n+m-1; ll=(m-2)*n+m-2;
      if (l==m-1)
        { u[m-1]=a[(m-1)*n+m-1]; v[m-1]=0.0;
          m=m-1; it=0;
        }
      else if (l==m-2)
        { b=-(a[ii]+a[ll]);
          c=a[ii]*a[ll]-a[jj]*a[kk];
          w=b*b-4.0*c;
          y=sqrt(fabs(w));
          if (w>0.0)
            { xy=1.0;
              if (b<0.0) xy=-1.0;
              u[m-1]=(-b-xy*y)/2.0;
              u[m-2]=c/u[m-1];
              v[m-1]=0.0; v[m-2]=0.0;
            }
          else
            { u[m-1]=-b/2.0; u[m-2]=u[m-1];
              v[m-1]=y/2.0; v[m-2]=-v[m-1];
            }
          m=m-2; it=0;
        }
      else
        { if (it>=jt)
            { 
              return false;
            }
          it=it+1;
          for (j=l+2; j<=m-1; j++)
            a[j*n+j-2]=0.0;
          for (j=l+3; j<=m-1; j++)
            a[j*n+j-3]=0.0;
          for (k=l; k<=m-2; k++)
            { if (k!=l)
                { p=a[k*n+k-1]; q=a[(k+1)*n+k-1];
                  r=0.0;
                  if (k!=m-2) r=a[(k+2)*n+k-1];
                }
              else
                { x=a[ii]+a[ll];
                  y=a[ll]*a[ii]-a[kk]*a[jj];
                  ii=l*n+l; jj=l*n+l+1;
                  kk=(l+1)*n+l; ll=(l+1)*n+l+1;
                  p=a[ii]*(a[ii]-x)+a[jj]*a[kk]+y;
                  q=a[kk]*(a[ii]+a[ll]-x);
                  r=a[kk]*a[(l+2)*n+l+1];
                }
              if ((fabs(p)+fabs(q)+fabs(r))!=0.0)
                { xy=1.0;
                  if (p<0.0) xy=-1.0;
                  s=xy*sqrt(p*p+q*q+r*r);
                  if (k!=l) a[k*n+k-1]=-s;
                  e=-q/s; f=-r/s; x=-p/s;
                  y=-x-f*r/(p+s);
                  g=e*r/(p+s);
                  z=-x-e*q/(p+s);
                  for (j=k; j<=m-1; j++)
                    { ii=k*n+j; jj=(k+1)*n+j;
                      p=x*a[ii]+e*a[jj];
                      q=e*a[ii]+y*a[jj];
                      r=f*a[ii]+g*a[jj];
                      if (k!=m-2)
                        { kk=(k+2)*n+j;
                          p=p+f*a[kk];
                          q=q+g*a[kk];
                          r=r+z*a[kk]; a[kk]=r;
                        }
                      a[jj]=q; a[ii]=p;
                    }
                  j=k+3;
                  if (j>=m-1) j=m-1;
                  for (i=l; i<=j; i++)
                    { ii=i*n+k; jj=i*n+k+1;
                      p=x*a[ii]+e*a[jj];
                      q=e*a[ii]+y*a[jj];
                      r=f*a[ii]+g*a[jj];
                      if (k!=m-2)
                        { kk=i*n+k+2;
                          p=p+f*a[kk];
                          q=q+g*a[kk];
                          r=r+z*a[kk]; a[kk]=r;
                        }
                      a[jj]=q; a[ii]=p;
                    }
                }
            }
        }
    }
  return true;
}

void GLKMatrixLib::_HessenbergConversion(double* a, int n)
{
	int i,j,k,u,v;
	double d,t;

  for (k=1; k<=n-2; k++)
    { d=0.0;
      for (j=k; j<=n-1; j++)
        { u=j*n+k-1; t=a[u];
          if (fabs(t)>fabs(d))
            { d=t; i=j;}
        }
      if (fabs(d)+1.0!=1.0)
        { if (i!=k)
            { for (j=k-1; j<=n-1; j++)
                { u=i*n+j; v=k*n+j;
                  t=a[u]; a[u]=a[v]; a[v]=t;
                }
              for (j=0; j<=n-1; j++)
                { u=j*n+i; v=j*n+k;
                  t=a[u]; a[u]=a[v]; a[v]=t;
                }
            }
          for (i=k+1; i<=n-1; i++)
            { u=i*n+k-1; t=a[u]/d; a[u]=0.0;
              for (j=k; j<=n-1; j++)
                { v=i*n+j;
                  a[v]=a[v]-t*a[k*n+j];
                }
              for (j=0; j<=n-1; j++)
                { v=j*n+k;
                  a[v]=a[v]+t*a[j*n+i];
                }
            }
        }
    }
}

void GLKMatrixLib::ppp(double** a,double* e, double* s, double** v, int m, int n)
{
	int i,j;
    double d;

    if (m>=n) i=n;
    else i=m;
    for (j=1; j<=i-1; j++)
      { a[(j-1)][j-1]=s[j-1];
        a[(j-1)][j]=e[j-1];
      }
    a[(i-1)][i-1]=s[i-1];
    if (m<n) a[(i-1)][i]=e[i-1];
    for (i=1; i<=n-1; i++)
    for (j=i+1; j<=n; j++)
      {
        d=v[i-1][j-1]; v[i-1][j-1]=v[j-1][i-1]; v[j-1][i-1]=d;
      }
}

void GLKMatrixLib::sss(double fg[], double cs[])
{
	double r,d;
    if ((fabs(fg[0])+fabs(fg[1]))==0.0)
      { cs[0]=1.0; cs[1]=0.0; d=0.0;}
    else 
      { d=sqrt(fg[0]*fg[0]+fg[1]*fg[1]);
        if (fabs(fg[0])>fabs(fg[1]))
          { d=fabs(d);
            if (fg[0]<0.0) d=-d;
          }
        if (fabs(fg[1])>=fabs(fg[0]))
          { d=fabs(d);
            if (fg[1]<0.0) d=-d;
          }
        cs[0]=fg[0]/d; cs[1]=fg[1]/d;
      }
    r=1.0;
    if (fabs(fg[0])>fabs(fg[1])) r=cs[1];
    else
      if (cs[0]!=0.0) r=1.0/cs[0];
    fg[0]=d; fg[1]=r;
}

bool GLKMatrixLib::_normalize(int m, double n[])
{
    int i;
    long double tt = 0.0;
    for(i=0;i<m;i++) tt += ((long double)n[i])*((long double)n[i]);
    tt = sqrt(tt);
    if(tt<1.0E-15){
        for(i=0;i<m;i++) n[i] = 0.0;
        return false;
    }
    else{
        for(i=0;i<m;i++) n[i] = n[i]/tt;
    }
    return true;
}
