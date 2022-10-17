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
  MatrixXcf ones = MatrixXcf::Ones(3,3);
ComplexEigenSolver<MatrixXcf> ces(ones, /* computeEigenvectors = */ false);
cout << "The eigenvalues of the 3x3 matrix of ones are:" 
     << endl << ces.eigenvalues() << endl;

  return 0;
}
