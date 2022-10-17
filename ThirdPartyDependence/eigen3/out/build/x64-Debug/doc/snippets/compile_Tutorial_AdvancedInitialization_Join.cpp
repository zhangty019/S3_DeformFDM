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
  RowVectorXd vec1(3);
vec1 << 1, 2, 3;
std::cout << "vec1 = " << vec1 << std::endl;

RowVectorXd vec2(4);
vec2 << 1, 4, 9, 16;
std::cout << "vec2 = " << vec2 << std::endl;

RowVectorXd joined(7);
joined << vec1, vec2;
std::cout << "joined = " << joined << std::endl;

  return 0;
}
