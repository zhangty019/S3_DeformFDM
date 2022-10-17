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
  EigenSolver<MatrixXf> es;
MatrixXf A = MatrixXf::Random(4,4);
es.compute(A, /* computeEigenvectors = */ false);
cout << "The eigenvalues of A are: " << es.eigenvalues().transpose() << endl;
es.compute(A + MatrixXf::Identity(4,4), false); // re-use es to compute eigenvalues of A+I
cout << "The eigenvalues of A+I are: " << es.eigenvalues().transpose() << endl;

  return 0;
}
