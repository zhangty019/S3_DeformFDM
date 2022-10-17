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
  Matrix3d m = Matrix3d::Identity();
m(0,2) = 1e-4;
cout << "Here's the matrix m:" << endl << m << endl;
cout << "m.isUnitary() returns: " << m.isUnitary() << endl;
cout << "m.isUnitary(1e-3) returns: " << m.isUnitary(1e-3) << endl;

  return 0;
}
