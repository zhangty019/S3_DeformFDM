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
  Vector2f v = Vector2f::Random();
JacobiRotation<float> G;
G.makeGivens(v.x(), v.y());
cout << "Here is the vector v:" << endl << v << endl;
v.applyOnTheLeft(0, 1, G.adjoint());
cout << "Here is the vector J' * v:" << endl << v << endl;
  return 0;
}
