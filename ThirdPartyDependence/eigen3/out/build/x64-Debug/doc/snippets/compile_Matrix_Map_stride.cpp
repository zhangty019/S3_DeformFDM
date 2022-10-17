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
  Matrix4i A;
A << 1,  2,  3,  4,
     5,  6,  7,  8,
     9, 10, 11, 12,
    13, 14, 15, 16;

std::cout << Matrix2i::Map(&A(1,1),Stride<8,2>()) << std::endl;

  return 0;
}
