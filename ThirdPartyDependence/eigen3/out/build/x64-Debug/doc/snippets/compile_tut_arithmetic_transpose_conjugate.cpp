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
  MatrixXcf a = MatrixXcf::Random(2,2);
cout << "Here is the matrix a\n" << a << endl;

cout << "Here is the matrix a^T\n" << a.transpose() << endl;


cout << "Here is the conjugate of a\n" << a.conjugate() << endl;


cout << "Here is the matrix a^*\n" << a.adjoint() << endl;



  return 0;
}
