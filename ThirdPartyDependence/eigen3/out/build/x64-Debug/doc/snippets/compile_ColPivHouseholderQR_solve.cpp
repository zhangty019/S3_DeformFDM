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
  Matrix3f m = Matrix3f::Random();
Matrix3f y = Matrix3f::Random();
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Here is the matrix y:" << endl << y << endl;
Matrix3f x;
x = m.colPivHouseholderQr().solve(y);
assert(y.isApprox(m*x));
cout << "Here is a solution x to the equation mx=y:" << endl << x << endl;

  return 0;
}
