static bool eigen_did_assert = false;
#define eigen_assert(X) if(!eigen_did_assert && !(X)){ std::cout << "### Assertion raised in " << __FILE__ << ":" << __LINE__ << ":\n" #X << "\n### The following would happen without assertions:\n"; eigen_did_assert = true;}

#include <iostream>
#include <Eigen/Eigen>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif


using namespace Eigen;
using namespace std;

int main(int, char**)
{
  cout.precision(3);
  MatrixXcd X = MatrixXcd::Random(4,4);
MatrixXcd A = X + X.adjoint();
cout << "Here is a random self-adjoint 4x4 matrix:" << endl << A << endl << endl;

Tridiagonalization<MatrixXcd> triOfA(A);
MatrixXd T = triOfA.matrixT();
cout << "The tridiagonal matrix T is:" << endl << T << endl << endl;

cout << "We can also extract the diagonals of T directly ..." << endl;
VectorXd diag = triOfA.diagonal();
cout << "The diagonal is:" << endl << diag << endl; 
VectorXd subdiag = triOfA.subDiagonal();
cout << "The subdiagonal is:" << endl << subdiag << endl;

  return 0;
}
