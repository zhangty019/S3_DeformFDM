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
  MatrixXf M1 = MatrixXf::Random(3,8);
cout << "Column major input:" << endl << M1 << "\n";
Map<MatrixXf,0,OuterStride<> > M2(M1.data(), M1.rows(), (M1.cols()+2)/3, OuterStride<>(M1.outerStride()*3));
cout << "1 column over 3:" << endl << M2 << "\n";

typedef Matrix<float,Dynamic,Dynamic,RowMajor> RowMajorMatrixXf;
RowMajorMatrixXf M3(M1);
cout << "Row major input:" << endl << M3 << "\n";
Map<RowMajorMatrixXf,0,Stride<Dynamic,3> > M4(M3.data(), M3.rows(), (M3.cols()+2)/3,
                                              Stride<Dynamic,3>(M3.outerStride(),3));
cout << "1 column over 3:" << endl << M4 << "\n";
  return 0;
}
