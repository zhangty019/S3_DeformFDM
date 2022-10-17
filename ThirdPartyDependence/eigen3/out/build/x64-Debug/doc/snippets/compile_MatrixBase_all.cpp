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
  Vector3f boxMin(Vector3f::Zero()), boxMax(Vector3f::Ones());
Vector3f p0 = Vector3f::Random(), p1 = Vector3f::Random().cwiseAbs();
// let's check if p0 and p1 are inside the axis aligned box defined by the corners boxMin,boxMax:
cout << "Is (" << p0.transpose() << ") inside the box: "
     << ((boxMin.array()<p0.array()).all() && (boxMax.array()>p0.array()).all()) << endl;
cout << "Is (" << p1.transpose() << ") inside the box: "
     << ((boxMin.array()<p1.array()).all() && (boxMax.array()>p1.array()).all()) << endl;

  return 0;
}
