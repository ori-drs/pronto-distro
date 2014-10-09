#include "eigen_rand.hpp"
#include "eigen_utils_common.hpp"

namespace eigen_utils {

using namespace Eigen;

void fitParticles(const Eigen::MatrixXd & state_samples
    , const Eigen::VectorXd & weights , Eigen::VectorXd & mean
    ,Eigen::MatrixXd & covariance)
{

  int Nsamples = state_samples.cols();
  int Ndim = state_samples.rows();

  double sum_weights = weights.sum();
  mean = state_samples * weights.matrix() / sum_weights;

  Eigen::VectorXd diff(Ndim);
  covariance.setZero(Ndim, Ndim);
  for (int ii = 0; ii < Nsamples; ii++)
  {
    diff = mean - state_samples.col(ii);
    covariance += diff * diff.transpose() * weights(ii);
  }
  covariance = covariance / sum_weights;
}
}



