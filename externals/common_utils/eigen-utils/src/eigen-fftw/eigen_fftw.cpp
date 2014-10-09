#include "eigen_fftw.hpp"
#include <fftw3.h>
#include <pthread.h>
#include <Eigen/Dense>

using namespace Eigen;

namespace eigen_utils{

//locking helpers. fftw planning is NOT thread safe
static pthread_mutex_t fftw_mutex = PTHREAD_MUTEX_INITIALIZER;
static void fftw_lock(void)
{
  pthread_mutex_lock(&fftw_mutex);
}
static void fftw_unlock(void)
{
  pthread_mutex_unlock(&fftw_mutex);
}


//casting helpers
inline fftw_complex * fftw_cast(const std::complex<double> * p)
{
  return const_cast<fftw_complex*>(reinterpret_cast<const fftw_complex*>(p));
}

inline fftwf_complex * fftw_cast(const std::complex<float> * p)
{
  return const_cast<fftwf_complex*>(reinterpret_cast<const fftwf_complex*>(p));
}







Eigen::MatrixXcd fftw_fwd2(const Eigen::MatrixXcd &timeM)
{
  //TODO: converting to RowMajor is inefficient
  Matrix<std::complex<double>, Dynamic, Dynamic, RowMajor> timeMRowMajor(timeM);
  Matrix<std::complex<double>, Dynamic, Dynamic, RowMajor> freqM(timeM.rows(), timeM.cols());


  /* Create plan */
  fftw_lock();
  fftw_plan plan = fftw_plan_dft_2d(timeMRowMajor.rows(), timeMRowMajor.cols(), fftw_cast(timeMRowMajor.data()),
      fftw_cast(freqM.data()), FFTW_FORWARD, FFTW_ESTIMATE);
  fftw_unlock();

  /* Compute forward DFT */
  fftw_execute(plan);

  /* Free plan memory */
  fftw_lock();
  fftw_destroy_plan(plan);
  fftw_unlock();

  return freqM;
}
Eigen::MatrixXcf fftw_fwd2(const Eigen::MatrixXcf &timeM)
{
  //TODO: converting to RowMajor is inefficient
  Matrix<std::complex<float>, Dynamic, Dynamic, RowMajor> timeMRowMajor(timeM);
  Matrix<std::complex<float>, Dynamic, Dynamic, RowMajor> freqM(timeM.rows(), timeM.cols());

  /* Create plan */
  fftw_lock();
  fftwf_plan plan = fftwf_plan_dft_2d(timeMRowMajor.rows(), timeMRowMajor.cols(), fftw_cast(timeMRowMajor.data()),
      fftw_cast(freqM.data()), FFTW_FORWARD, FFTW_ESTIMATE);
  fftw_unlock();

  /* Compute forward DFT */

  fftwf_execute(plan);


  /* Free plan memory */
  fftw_lock();
  fftwf_destroy_plan(plan); //TODO: do I need to lock for this?
  fftw_unlock();

  return freqM;
}

Eigen::MatrixXcd fftw_inv2(const Eigen::MatrixXcd &freqM)
{
  Matrix<std::complex<double>, Dynamic, Dynamic, RowMajor> freqMRowMajor(freqM);
  Matrix<std::complex<double>, Dynamic, Dynamic, RowMajor> timeM(freqM.rows(), freqM.cols());

  /* Create plan */
  fftw_lock();
  fftw_plan plan = fftw_plan_dft_2d(freqMRowMajor.rows(), freqMRowMajor.cols(), fftw_cast(freqMRowMajor.data()),
      fftw_cast(timeM.data()), FFTW_BACKWARD, FFTW_ESTIMATE);
  fftw_unlock();

  /* Compute forward DFT */
  fftw_execute(plan);

  /* Free plan memory */
  fftw_lock();
  fftw_destroy_plan(plan);
  fftw_unlock();

  timeM *= 1. / (timeM.size());

  return timeM;
}
Eigen::MatrixXcf fftw_inv2(const Eigen::MatrixXcf &freqM)
{
  Matrix<std::complex<float>, Dynamic, Dynamic, RowMajor> freqMRowMajor(freqM);
  Matrix<std::complex<float>, Dynamic, Dynamic, RowMajor> timeM(freqM.rows(), freqM.cols());

  /* Create plan */
  fftw_lock();
  fftwf_plan plan = fftwf_plan_dft_2d(freqMRowMajor.rows(), freqMRowMajor.cols(), fftw_cast(freqMRowMajor.data()),
      fftw_cast(timeM.data()), FFTW_BACKWARD, FFTW_ESTIMATE);
  fftw_unlock();

  /* Compute forward DFT */
  fftwf_execute(plan);

  /* Free plan memory */
  fftw_lock();
  fftwf_destroy_plan(plan);
  fftw_unlock();

  timeM *= 1. / (timeM.size());

  return timeM;
}

}
