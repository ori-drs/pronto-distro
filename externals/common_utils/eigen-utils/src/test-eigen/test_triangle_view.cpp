#include <eigen_utils/eigen_utils.hpp>

using namespace Eigen;
using namespace eigen_utils;
using namespace std;

int main(int argc, char * argv[])
{

  Matrix<double, 9, 1> nined;
//  nined.setLinSpaced(1, 9);

  Map<const Matrix3d> mated(nined.data());
  eigen_dump(mated);

  MatrixXd m(3, 3);
  m << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  eigen_dump(m);

  VectorXd flat = flattenSymmetric(m);

  MatrixXd recon = unflattenSymmetric(flat);

  eigen_dump(flat);
  eigen_dump(recon);

}
