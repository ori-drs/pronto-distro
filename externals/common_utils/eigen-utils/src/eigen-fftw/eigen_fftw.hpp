#ifndef __EIGEN_FFTW_HPP__
#define __EIGEN_FFTW_HPP__

#include <Eigen/Dense>
#include <complex>

namespace eigen_utils {
Eigen::MatrixXcd fftw_fwd2(const Eigen::MatrixXcd &timeM);
Eigen::MatrixXcf fftw_fwd2(const Eigen::MatrixXcf &timeM);

inline Eigen::MatrixXcd fftw_fwd2r(const Eigen::MatrixXd &timeM)
{
  Eigen::MatrixXcd ctimeM = timeM.cast<std::complex<double> >();
  return fftw_fwd2(ctimeM);
}
inline Eigen::MatrixXcf fftw_fwd2r(const Eigen::MatrixXf &timeM)
{
  Eigen::MatrixXcf ctimeM = timeM.cast<std::complex<float> >();
  return fftw_fwd2(ctimeM);
}

Eigen::MatrixXcd fftw_inv2(const Eigen::MatrixXcd &freqM);
Eigen::MatrixXcf fftw_inv2(const Eigen::MatrixXcf &freqM);

}

#endif /* __EIGEN_FFTW_HPP__ */
