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
  Matrix4f A = MatrixXf::Random(4,4);
cout << "Here is a random 4x4 matrix:" << endl << A << endl;
HessenbergDecomposition<MatrixXf> hessOfA(A);
MatrixXf H = hessOfA.matrixH();
cout << "The Hessenberg matrix H is:" << endl << H << endl;
MatrixXf Q = hessOfA.matrixQ();
cout << "The orthogonal matrix Q is:" << endl << Q << endl;
cout << "Q H Q^T is:" << endl << Q * H * Q.transpose() << endl;

  return 0;
}
