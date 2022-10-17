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
  GeneralizedEigenSolver<MatrixXf> ges;
MatrixXf A = MatrixXf::Random(4,4);
MatrixXf B = MatrixXf::Random(4,4);
ges.compute(A, B);
cout << "The (complex) numerators of the generalzied eigenvalues are: " << ges.alphas().transpose() << endl;
cout << "The (real) denominatore of the generalzied eigenvalues are: " << ges.betas().transpose() << endl;
cout << "The (complex) generalzied eigenvalues are (alphas./beta): " << ges.eigenvalues().transpose() << endl;

  return 0;
}
