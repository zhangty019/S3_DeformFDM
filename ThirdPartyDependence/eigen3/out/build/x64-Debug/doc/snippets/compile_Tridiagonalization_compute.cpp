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
  Tridiagonalization<MatrixXf> tri;
MatrixXf X = MatrixXf::Random(4,4);
MatrixXf A = X + X.transpose();
tri.compute(A);
cout << "The matrix T in the tridiagonal decomposition of A is: " << endl;
cout << tri.matrixT() << endl;
tri.compute(2*A); // re-use tri to compute eigenvalues of 2A
cout << "The matrix T in the tridiagonal decomposition of 2A is: " << endl;
cout << tri.matrixT() << endl;

  return 0;
}
