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
  Matrix4i m = Matrix4i::Random();
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Here is m.bottomRightCorner(2, 2):" << endl;
cout << m.bottomRightCorner(2, 2) << endl;
m.bottomRightCorner(2, 2).setZero();
cout << "Now the matrix m is:" << endl << m << endl;

  return 0;
}
