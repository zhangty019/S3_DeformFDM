// GLKSparseMatrixDirectSolver.h: interface for the GLKSparseMatrixDirectSolver class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _GLKSPARSEMATRIX_DIRECT_SOLVER
#define _GLKSPARSEMATRIX_DIRECT_SOLVER

class GLKSparseMatrix;

class GLKSparseMatrixDirectSolver  
{
public:
	GLKSparseMatrixDirectSolver();
	virtual ~GLKSparseMatrixDirectSolver();

	static bool BandGaussianElimination(GLKSparseMatrix *sparseMatrix, double *B);
	static bool BandGaussianElimination(GLKSparseMatrix *sparseMatrix, int colNum, double **B);

private:
	static int aband(double* b, double* d, long n, long l, long il, long m);
};

#endif
