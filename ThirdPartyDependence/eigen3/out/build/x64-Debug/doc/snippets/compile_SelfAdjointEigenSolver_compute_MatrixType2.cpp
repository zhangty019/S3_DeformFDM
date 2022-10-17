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
  MatrixXd X = MatrixXd::Random(5,5);
MatrixXd A = X * X.transpose();
X = MatrixXd::Random(5,5);
MatrixXd B = X * X.transpose();

GeneralizedSelfAdjointEigenSolver<MatrixXd> es(A,B,EigenvaluesOnly);
cout << "The eigenvalues of the pencil (A,B) are:" << endl << es.eigenvalues() << endl;
es.compute(B,A,false);
cout << "The eigenvalues of the pencil (B,A) are:" << endl << es.eigenvalues() << endl;

  return 0;
}
