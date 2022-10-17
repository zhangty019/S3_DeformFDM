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
  Matrix3f A = Matrix3f::Random(3,3), B;
B << 0,1,0,  
     0,0,1,  
     1,0,0;
cout << "At start, A = " << endl << A << endl;
A *= B;
cout << "After A *= B, A = " << endl << A << endl;
A.applyOnTheRight(B);  // equivalent to A *= B
cout << "After applyOnTheRight, A = " << endl << A << endl;

  return 0;
}
