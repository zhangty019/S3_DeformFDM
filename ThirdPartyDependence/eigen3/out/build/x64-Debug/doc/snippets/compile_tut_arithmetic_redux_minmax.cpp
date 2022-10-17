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
  std::ptrdiff_t i, j;
  float minOfM = m.minCoeff(&i,&j);
  cout << "Here is the matrix m:\n" << m << endl;
  cout << "Its minimum coefficient (" << minOfM 
       << ") is at position (" << i << "," << j << ")\n\n";

  RowVector4i v = RowVector4i::Random();
  int maxOfV = v.maxCoeff(&i);
  cout << "Here is the vector v: " << v << endl;
  cout << "Its maximum coefficient (" << maxOfV 
       << ") is at position " << i << endl;

  return 0;
}
