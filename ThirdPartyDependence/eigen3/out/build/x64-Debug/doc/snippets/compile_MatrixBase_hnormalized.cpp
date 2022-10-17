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
  Vector4d v = Vector4d::Random();
Projective3d P(Matrix4d::Random());
cout << "v                   = " << v.transpose() << "]^T" << endl;
cout << "v.hnormalized()     = " << v.hnormalized().transpose() << "]^T" << endl;
cout << "P*v                 = " << (P*v).transpose() << "]^T" << endl;
cout << "(P*v).hnormalized() = " << (P*v).hnormalized().transpose() << "]^T" << endl;
  return 0;
}
