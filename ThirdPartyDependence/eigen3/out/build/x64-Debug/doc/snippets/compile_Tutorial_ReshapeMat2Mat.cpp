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
  MatrixXf M1(2,6);    // Column-major storage
M1 << 1, 2, 3,  4,  5,  6,
      7, 8, 9, 10, 11, 12;

Map<MatrixXf> M2(M1.data(), 6,2);
cout << "M2:" << endl << M2 << endl;
  return 0;
}
