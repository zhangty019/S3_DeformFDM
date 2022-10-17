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
  Matrix2f m = Matrix2f::Random();
m = (m + m.adjoint()).eval();
JacobiRotation<float> J;
J.makeJacobi(m, 0, 1);
cout << "Here is the matrix m:" << endl << m << endl;
m.applyOnTheLeft(0, 1, J.adjoint());
m.applyOnTheRight(0, 1, J);
cout << "Here is the matrix J' * m * J:" << endl << m << endl;
  return 0;
}
