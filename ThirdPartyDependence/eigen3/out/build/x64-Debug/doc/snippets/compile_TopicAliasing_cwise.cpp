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
  MatrixXf mat(2,2); 
mat << 1, 2,  4, 7;
cout << "Here is the matrix mat:\n" << mat << endl << endl;

mat = 2 * mat;
cout << "After 'mat = 2 * mat', mat = \n" << mat << endl << endl;


mat = mat - MatrixXf::Identity(2,2);
cout << "After the subtraction, it becomes\n" << mat << endl << endl;


ArrayXXf arr = mat;
arr = arr.square();
cout << "After squaring, it becomes\n" << arr << endl << endl;

// Combining all operations in one statement:
mat << 1, 2,  4, 7;
mat = (2 * mat - MatrixXf::Identity(2,2)).array().square();
cout << "Doing everything at once yields\n" << mat << endl << endl;

  return 0;
}
