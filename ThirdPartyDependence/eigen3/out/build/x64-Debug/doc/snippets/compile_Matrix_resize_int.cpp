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
  VectorXd v(10);
v.resize(3);
RowVector3d w;
w.resize(3); // this is legal, but has no effect
cout << "v: " << v.rows() << " rows, " << v.cols() << " cols" << endl;
cout << "w: " << w.rows() << " rows, " << w.cols() << " cols" << endl;

  return 0;
}
