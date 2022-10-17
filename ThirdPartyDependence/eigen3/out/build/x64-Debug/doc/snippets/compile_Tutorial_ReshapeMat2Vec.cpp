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
  MatrixXf M1(3,3);    // Column-major storage
M1 << 1, 2, 3,
      4, 5, 6,
      7, 8, 9;

Map<RowVectorXf> v1(M1.data(), M1.size());
cout << "v1:" << endl << v1 << endl;

Matrix<float,Dynamic,Dynamic,RowMajor> M2(M1);
Map<RowVectorXf> v2(M2.data(), M2.size());
cout << "v2:" << endl << v2 << endl;
  return 0;
}
