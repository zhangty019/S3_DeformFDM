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
  SparseMatrix<double> A(3,3);
A.insert(1,2) = 0;
A.insert(0,1) = 1;
A.insert(2,0) = 2;
A.makeCompressed();
cout << "The matrix A is:" << endl << MatrixXd(A) << endl;
cout << "it has " << A.nonZeros() << " stored non zero coefficients that are: " << A.coeffs().transpose() << endl;
A.coeffs() += 10;
cout << "After adding 10 to every stored non zero coefficient, the matrix A is:" << endl << MatrixXd(A) << endl;

  return 0;
}
