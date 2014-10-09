#ifndef __eigen_rand_h__
#define __eigen_rand_h__

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <math.h>
#include <bot_core/bot_core.h>

namespace eigen_utils {

//todo see about replacing templated functions with MatrixBase arguments and then checking for num rows and cols
/**
 * Fill an Eigen vector with gaussian random numbers with mu=0, Sigma = I
 */
template<typename scalarType, int N>
inline void randn_identity(Eigen::Matrix<scalarType, N, 1> & vec)
{
  for (int ii = 0; ii < vec.size(); ii++) {
    vec(ii) = bot_gauss_rand(0, 1);
  }
}

template<typename scalarType, int N>
inline Eigen::Matrix<scalarType, N, 1> randn_chol(const Eigen::Matrix<scalarType, N, N> & chol_decomp_cov)
{
  Eigen::Matrix<scalarType, N, 1> vec;
  randn_identity(vec);
  return chol_decomp_cov * vec;
}

template<typename scalarType, int N>
inline Eigen::Matrix<scalarType, N, 1> randn(const Eigen::Matrix<scalarType, N, N> & cov)
{
  Eigen::Matrix<scalarType, N, N> chol_decomp = cov.llt().matrixL();
  return randn_chol(chol_decomp);
}

/**
 * unnormalized log likelihood
 */
template<typename scalarType, int N>
inline scalarType loglike_unnormalized(const Eigen::Matrix<scalarType, N, 1> & x
    , const Eigen::Matrix<scalarType, N, 1> & mu
    , const Eigen::Matrix<scalarType, N, N> & sigma)
{
  Eigen::Matrix<scalarType, N, 1> diff = mu - x;
  return -0.5 * diff.transpose() * sigma.ldlt().solve(diff);
}

/**
 * unnormalized log likelhihood using information matrix
 */
template<typename scalarType, int N>
inline scalarType loglike_information_unnormalized(const Eigen::Matrix<scalarType, N, 1> & x
    ,const Eigen::Matrix<scalarType, N, 1> & mu , const Eigen::Matrix<scalarType, N, N> & sigma_inv)
{
  Eigen::Matrix<scalarType, N, 1> diff = mu - x;
  return -0.5 * diff.transpose() * sigma_inv * diff;
}

template<typename DerivedX, typename DerivedMU, typename DerivedSigma>
double loglike_normalized(const Eigen::MatrixBase<DerivedX> & x, const Eigen::MatrixBase<DerivedMU> & mu,
    const Eigen::MatrixBase<DerivedSigma> & sigma)
{
  Eigen::VectorXd diff = mu - x;
  return -log(sigma.determinant()) - diff.transpose() * sigma.ldlt().solve(diff);
}

template<typename scalarType, int N>
inline scalarType normpdf(const Eigen::Matrix<scalarType, N, 1> & x, const Eigen::Matrix<scalarType, N, 1> & mu
    , const Eigen::Matrix<scalarType, N, N> & sigma)
{
  Eigen::Matrix<scalarType, N, 1> diff = mu - x;

  scalarType exponent = -0.5 * diff.transpose() * sigma.ldlt().solve(diff);

  return exp(exponent) / (pow(2 * M_PI, ((scalarType) N) / 2.0) * pow(sigma.determinant(), 0.5));
}

void fitParticles(const Eigen::MatrixXd & state_samples
    , const Eigen::VectorXd & weights, Eigen::VectorXd & mean
    , Eigen::MatrixXd & covariance);

}
#endif
