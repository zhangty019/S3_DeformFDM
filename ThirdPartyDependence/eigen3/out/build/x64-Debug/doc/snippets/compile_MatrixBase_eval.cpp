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
  Matrix2f M = Matrix2f::Random();
Matrix2f m;
m = M;
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Now we want to copy a column into a row." << endl;
cout << "If we do m.col(1) = m.row(0), then m becomes:" << endl;
m.col(1) = m.row(0);
cout << m << endl << "which is wrong!" << endl;
cout << "Now let us instead do m.col(1) = m.row(0).eval(). Then m becomes" << endl;
m = M;
m.col(1) = m.row(0).eval();
cout << m << endl << "which is right." << endl;

  return 0;
}
