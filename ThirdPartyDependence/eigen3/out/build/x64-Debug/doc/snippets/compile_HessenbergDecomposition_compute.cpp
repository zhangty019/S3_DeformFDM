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
  MatrixXcf A = MatrixXcf::Random(4,4);
HessenbergDecomposition<MatrixXcf> hd(4);
hd.compute(A);
cout << "The matrix H in the decomposition of A is:" << endl << hd.matrixH() << endl;
hd.compute(2*A); // re-use hd to compute and store decomposition of 2A
cout << "The matrix H in the decomposition of 2A is:" << endl << hd.matrixH() << endl;

  return 0;
}
