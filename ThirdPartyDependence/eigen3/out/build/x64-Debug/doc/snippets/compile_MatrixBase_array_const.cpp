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
  Vector3d v(-1,2,-3);
cout << "the absolute values:" << endl << v.array().abs() << endl;
cout << "the absolute values plus one:" << endl << v.array().abs()+1 << endl;
cout << "sum of the squares: " << v.array().square().sum() << endl;

  return 0;
}
