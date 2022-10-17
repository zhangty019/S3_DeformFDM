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
  MatrixXf A = MatrixXf::Random(4,4);
RealSchur<MatrixXf> schur(4);
schur.compute(A, /* computeU = */ false);
cout << "The matrix T in the decomposition of A is:" << endl << schur.matrixT() << endl;
schur.compute(A.inverse(), /* computeU = */ false);
cout << "The matrix T in the decomposition of A^(-1) is:" << endl << schur.matrixT() << endl;

  return 0;
}
