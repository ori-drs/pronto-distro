#include "eigen_utils_common.hpp"

namespace eigen_utils {

double atan2Vec(const Eigen::Vector2d & vec)
{
  return atan2(vec(1), vec(0));
}

Eigen::Vector2d angleToVec(double angle)
{
  Eigen::Vector2d unit_vec;
  unit_vec << cos(angle), sin(angle);
  return unit_vec;
}

void angleToVec(double angle, Eigen::Vector2d & unit_vec)
{
  unit_vec << cos(angle), sin(angle);
}

Eigen::VectorXd flattenSymmetric(Eigen::MatrixXd symm)
{
  //todo: other data types?
  //todo: safety/square matrix checking?
  int num_entries = symm.rows() * (symm.rows() + 1) / 2;
  Eigen::VectorXd flat(num_entries);
  int count = 0;
  for (int i = 0; i < symm.cols(); i++) {
    int ltcsz = symm.cols() - i;
    flat.block(count, 0, ltcsz, 1) = symm.block(i, i, ltcsz, 1);
    count += ltcsz;
  }
  return flat;
}

Eigen::MatrixXd unflattenSymmetric(Eigen::VectorXd flat)
{
  //todo: other data types?
  //todo: safety/size checking?

  int num_rows = (-1 + sqrt(1 + 4 * 2 * flat.rows())) / 2; //solve for number of rows using quadratic eq
  Eigen::MatrixXd symm(num_rows, num_rows);
  int count = 0;
  for (int i = 0; i < num_rows; i++) {
    int ltcsz = num_rows - i;
    symm.block(i, i, ltcsz, 1) = flat.block(count, 0, ltcsz, 1);
    symm.block(i, i, 1, ltcsz) = flat.block(count, 0, ltcsz, 1).transpose();
    count += ltcsz;
  }
  return symm;
}

bool hasNan(const Eigen::MatrixXd &m)
{
  for (int ii = 0; ii < m.rows(); ii++) {
    for (int jj = 0; jj < m.cols(); jj++) {
      if (isnan(m(ii, jj)))
        return true;
    }
  }
  return false;
}

void assertNoNan(const Eigen::MatrixXd &m)
{
  if (hasNan(m)) {
    std::cerr << "ERROR: m has a NAN!\n";
    eigen_dump(m);
    assert(false);
  }
}

}

