#include <eigen_utils/eigen_utils.hpp>

using namespace Eigen;
using namespace eigen_utils;
using namespace std;

int main(int argc, char * argv[])
{

  int n = 5;
  int m = 2;

  MatrixXd A, QT;
  VectorXd b, x_star;
  A.setRandom(m, n);
  b.setRandom(m);
  x_star.resize(n);
  QT.setRandom(n, n);
  MatrixXd Q = QT.transpose() * QT;

  eigen_matlab_dump(A);
  eigen_matlab_dump(b);
  eigen_matlab_dump(Q);

  quadProgEliminationSolve(Q, A, b, x_star);
  eigen_matlab_dump(x_star);

}
