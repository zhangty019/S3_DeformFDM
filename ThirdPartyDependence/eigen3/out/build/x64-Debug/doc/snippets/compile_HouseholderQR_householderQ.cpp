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
  MatrixXf A(MatrixXf::Random(5,3)), thinQ(MatrixXf::Identity(5,3)), Q;
A.setRandom();
HouseholderQR<MatrixXf> qr(A);
Q = qr.householderQ();
thinQ = qr.householderQ() * thinQ;
std::cout << "The complete unitary matrix Q is:\n" << Q << "\n\n";
std::cout << "The thin matrix Q is:\n" << thinQ << "\n\n";

  return 0;
}
