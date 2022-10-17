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
  Matrix3f m;
m = AngleAxisf(0.25*M_PI, Vector3f::UnitX())
  * AngleAxisf(0.5*M_PI,  Vector3f::UnitY())
  * AngleAxisf(0.33*M_PI, Vector3f::UnitZ());
cout << m << endl << "is unitary: " << m.isUnitary() << endl;

  return 0;
}
