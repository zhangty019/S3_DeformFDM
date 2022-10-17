// GLKSparseMatrixLib.h: interface for the GLKSparseMatrixLib class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _GLKSPARSEMATRIXLIB
#define _GLKSPARSEMATRIXLIB

//////////////////////////////////////////////////////////////////////////////////////////////////
//
//	ATTENTION : The index of all matrices and vectors in this library start from 1 (NOT 0) !!!
//			The computation taken in this class are all based on the row-indexed sparse storage !!!
//

class GLKSparseMatrix;

class GLKSparseMatrixIterativeSolver  
{
public:
	GLKSparseMatrixIterativeSolver();
	virtual ~GLKSparseMatrixIterativeSolver();

	// ATTENTION: the index are all started from "ONE" !!!
	//		the computation taken in this class are all based on the row-indexed sparse storage !!!

	//--------------------------------------------------------------------------------------------
	// Converts a square matrix a[1..n][1..n] into row-indexed sparse storage mode. Only elements
	// of a with magnitude >=thresh are retained. Output is in two linear arrays with dimension
	// nmax (an input parameter): sa[1..] contains array values, indexed by ija[1..]. The
	// number of elements filled of sa and ija on output are both ija[ija[1]-1]-1 (see text).
	static void SparseMatrixIn(double **a, unsigned long n, double thresh, unsigned long nmax, 
									double *sa, unsigned long *ija);
	static void SparseMatrixIn_ZeroStartIndex(double **a, unsigned long n, double thresh, unsigned long nmax, 
									double *sa, unsigned long *ija);

	//--------------------------------------------------------------------------------------------
	// Convert a compressed row storage into row-indexed sparse storage mode.
	// Input: (all indices begin from 1)
	//		double *a;						// nonzero coefficients - range [1...nnz]
	//		unsigned long *aComIndex;		// colume indices - range [1...nnz]
	//		unsigned long *aRowBeginIndex;	// indicating the beginning of each row in the coefficients
	//										//		(zero means none coefficient in the row)
	//										//		 - range [1...n]
	//		unsigned long n;				// dimension of matrix
	//		unsigned long nnz;				// number of nonzero coefficients
	//		unsigned long nmax;				// upper-bound for the index of output arrays
	//
	// For example, the following matrix
	//		| 3 0 1 0 0 |
	//		| 0 4 0 0 0 |
	//		| 0 7 5 9 0 |
	//		| 0 0 0 0 2 |
	//		| 0 0 0 6 5 |
	// is stored as
	//		n = 5;	nnz = 9;
	//		a = { 0, 3, 1, 4, 7, 5, 9, 2, 6, 5 }
	//		aComIndex = { 0, 1, 3, 2, 2, 3, 4, 5, 4, 5 }
	//		aRowBeginIndex = { 0, 1, 3, 4, 7, 8 }
	//	
	// Output is in two linear arrays with dimension nmax (an input parameter): sa[1..] contains 
	// array values, indexed by ija[1..]. The number of elements filled of sa and ija on output 
	// are both ija[ija[1]-1]-1 (see text).
	static void SparseMatrixIn(double *a, unsigned long *aComIndex, unsigned long *aRowBeginIndex,
			unsigned long n, unsigned long nnz, double *sa, unsigned long *ija, unsigned long nmax);

	//--------------------------------------------------------------------------------------------
	// Multiply a matrix in row-index sparse storage arrays sa and ija by a vector x[1..n], giving
	// a vector b[1..n].
	static void SparseMatrixMultiplyVector(double *sa, unsigned long *ija, double *x, double *b,
									unsigned long n);

	//--------------------------------------------------------------------------------------------
	// Multiply the transpose of a matrix in row-index sparse storage arrays sa and ija by a vector
	// x[1..n], giving a vector b[1..n].
	static void TransposeSparseMatrixMultiplyVector(double *sa, unsigned long *ija, double *x, 
									double *b, unsigned long n);

	//--------------------------------------------------------------------------------------------
	// Pattern Multiply : C = A * BT
	//	Multiply a matrix A in row-index sparse storage arrays sa and ija by the transpose of a
	//		matrix B in row-index sparse storage arrays sb and iib, giving an output sparse matrix C
	//		in row-index sparse storage.
	//	Note: This routine computes only those components of the matrix product that are 
	//		pre-specified by the input index array ijc, which is not modified !!!
	//	A - sa[] and iia[]
	//	B - sb[] and iib[]
	//	C - sc[] and iic[]
	static void SparseMatrixMultiplyTransposedSparseMatrix(double *sa, unsigned long *ija,
		double *sb, unsigned long *ijb, double *sc, unsigned long *ijc);

	//--------------------------------------------------------------------------------------------
	// Threshold Multiply : C = threshold ( A * BT )
	//	Multiply a matrix A in row-index sparse storage arrays sa and ija by the transpose of a
	//		matrix B in row-index sparse storage arrays sb and iib, giving an output sparse matrix C
	//		in row-index sparse storage.
	//	The coefficients less than "thresh" are neglected.
	//	A - sa[] and iia[]
	//	B - sb[] and iib[]
	//	C - sc[] and iic[]
	static void SparseMatrixMultiplyTransposedSparseMatrix(double *sa, unsigned long *ija,
		double *sb, unsigned long *ijb, double thresh, unsigned long nmax, 
		double *sc, unsigned long *ijc);

	//--------------------------------------------------------------------------------------------
	// Solves A x = b for x[1..n], given b[1..n], by the iterative biconjugate gradient method.
	// On input x[1..n] should be set to an initial guess of the solution (or all ones); itol is 1,2,3,
	// or 4, specifying which convergence test is applied (see text); 
	//	(actually, itol=3 tests the "L-2" norm while itol=4 tests the "L-infinite" norm)
	// itmax is the maximum number of allowed iterations; and tol is the desired convergence tolerance. 
	// On output, x[1..n] is reset to the improved solution, iter is the number of iterations actually 
	// taken, and err is the estimated error. The matrix A is referenced only through the user-supplied 
	// routines atimes, which computes the product of either A or its transpose on a vector; and asolve, 
	// which solves A' x = b or (A')T x = b for some preconditioner matrix A'(possibly the trivial 
	// diagonal part of A).
	static void LinearPBCG(double *sa, unsigned long *ija, unsigned long n, double *b, double *x, 
									int itol, double tol, int itmax, int &iter, double &err);
	static void LinearPBCG(GLKSparseMatrix *sparseMatrix, double *B,
									int itol, double tol, int itmax, int &iter, double &err);
	static void LinearPBCG(GLKSparseMatrix *sparseMatrix, double *B, double *X,
									int itol, double tol, int itmax, int &iter, double &err);

private:
	//--------------------------------------------------------------------------------------------
	// Get coefficient by index from the compressed row storage
	static double coefficientFromComRowByIndex(double *a, unsigned long *aComIndex, 
			unsigned long *aRowBeginIndex, unsigned long n, unsigned long nnz, unsigned long i, 
			unsigned long j);

	//--------------------------------------------------------------------------------------------
	static void asolve(double *sa, unsigned long *ija, unsigned long n, double *b, double *x, 
									int itrnsp);

	//--------------------------------------------------------------------------------------------
	static void atimes(double *sa, unsigned long *ija, unsigned long n, double *x, double *r, 
									int itrnsp);
	
	//--------------------------------------------------------------------------------------------
	// Compute one of two norms for a vector sx[1..n], as signaled by itol. Used by LinearPBCG.
	static double snrm(unsigned long n, double *sx, int itol);
};

#endif
