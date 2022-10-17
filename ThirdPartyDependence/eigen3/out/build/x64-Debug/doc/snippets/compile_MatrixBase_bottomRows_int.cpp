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
  Array44i a = Array44i::Random();
cout << "Here is the array a:" << endl << a << endl;
cout << "Here is a.bottomRows(2):" << endl;
cout << a.bottomRows(2) << endl;
a.bottomRows(2).setZero();
cout << "Now the array a is:" << endl << a << endl;

  return 0;
}
