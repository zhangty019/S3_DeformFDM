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
  int array[12];
for(int i = 0; i < 12; ++i) array[i] = i;
cout << Map<VectorXi, 0, InnerStride<2> >
         (array, 6) // the inner stride has already been passed as template parameter
     << endl;

  return 0;
}
