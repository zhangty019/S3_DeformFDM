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
    int n = 10000;
  VectorXd x(n), b(n);
  SparseMatrix<double> A(n,n);
  /* ... fill A and b ... */ 
  BiCGSTAB<SparseMatrix<double> > solver(A);
  // start from a random solution
  x = VectorXd::Random(n);
  solver.setMaxIterations(1);
  int i = 0;
  do {
    x = solver.solveWithGuess(b,x);
    std::cout << i << " : " << solver.error() << std::endl;
    ++i;
  } while (solver.info()!=Success && i<100);
  return 0;
}
