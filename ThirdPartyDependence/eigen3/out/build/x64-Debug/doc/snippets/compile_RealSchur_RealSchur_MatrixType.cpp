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
  MatrixXd A = MatrixXd::Random(6,6);
cout << "Here is a random 6x6 matrix, A:" << endl << A << endl << endl;

RealSchur<MatrixXd> schur(A);
cout << "The orthogonal matrix U is:" << endl << schur.matrixU() << endl;
cout << "The quasi-triangular matrix T is:" << endl << schur.matrixT() << endl << endl;

MatrixXd U = schur.matrixU();
MatrixXd T = schur.matrixT();
cout << "U * T * U^T = " << endl << U * T * U.transpose() << endl;

  return 0;
}
