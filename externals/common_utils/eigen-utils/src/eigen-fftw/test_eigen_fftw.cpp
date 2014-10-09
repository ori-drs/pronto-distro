#include "../eigen_utils_common.hpp"
#include "eigen_fftw.hpp"

#include <stdio.h>
#include <fftw3.h>

using namespace Eigen;
using namespace eigen_utils;
using namespace std;

// Print 1D complex array
void print_complex_1d(fftw_complex* arr, int size, int normalize)
{
  int i;

  for (i = 0; i < size; i++)
      {
    double a = arr[i][0] / (normalize ? size : 1);
    double b = arr[i][1] / (normalize ? size : 1);
    printf(" {%2.1f, %2.1f} \n", a, b);
  }
  printf("\n");
}

// Print 2D complex array
void print_complex_2d(fftw_complex* arr, int rows, int cols, int normalize)
{
  int i, j, k, size = rows * cols;

  for (i = 0, k = 0; i < rows; i++)
      {
    for (j = 0; j < cols; j++, k++)
        {
      double a = arr[k][0] / (normalize ? size : 1);
      double b = arr[k][1] / (normalize ? size : 1);
      printf(" {%2.3f, %2.3f} ", a, b);
    }
    printf("\n");
  }
  printf("\n");
}

// Print 1D real array
void print_real_1d(double* arr, int size, int normalize)
{
  int i;

  for (i = 0; i < size; i++)
    printf(" %2.1f ", arr[i] / (normalize ? size : 1));

  printf("\n\n");
}

// Print 2D real array
void print_real_2d(double* arr, int rows, int cols, int normalize)
{
  int i, j, k, size = rows * cols;

  for (i = 0, k = 0; i < rows; i++)
      {
    for (j = 0; j < cols; j++, k++)
      printf(" %2.1f ", arr[k] / (normalize ? size : 1));
    printf("\n");
  }

  printf("\n\n");
}

/**
 * Sample code to compute complex 2D DFT
 */
void complex_2d_dft(int M = 2, int N = 3)
{
  fftw_complex* a, *b, *c;
  fftw_plan plan_f, plan_b;
  int i, j, k;

  /* Allocate input/output arrays */
  a = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * M * N);
  b = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * M * N);
  c = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * M * N);

  /* Create plans */
  plan_f = fftw_plan_dft_2d(M, N, a, b, FFTW_FORWARD, FFTW_ESTIMATE);
  plan_b = fftw_plan_dft_2d(M, N, b, c, FFTW_BACKWARD, FFTW_ESTIMATE);

  /* Populate input data
   in row-major order */
  for (i = 0; i < M * N; i++)
      {
    a[i][0] = i * 0.1;
    a[i][1] = 0.;
  }

  /* Compute forward & inverse DFT */
  fftw_execute(plan_f);
  fftw_execute(plan_b);

  /* Print results */
  printf("Input:\n");
  print_complex_2d(a, M, N, 0);
  printf("DFT:\n");
  print_complex_2d(b, M, N, 0);
  printf("IDFT:\n");
  print_complex_2d(c, M, N, 1);

  /* Free memory */
  fftw_destroy_plan(plan_f);
  fftw_destroy_plan(plan_b);
  fftw_free(a);
  fftw_free(b);
  fftw_free(c);
}

//inline fftw_complex * fftw_cast(const std::complex<double> * p)
//{
//  return const_cast<fftw_complex*>(reinterpret_cast<const fftw_complex*>(p));
//}
//
//inline fftwf_complex * fftw_cast(const std::complex<float> * p)
//{
//  return const_cast<fftwf_complex*>(reinterpret_cast<const fftwf_complex*>(p));
//}
//
//MatrixXcd eigen_fft_fwd2(const MatrixXcd &timeM)
//{
//  //TODO: converting to RowMajor is inefficient
//  Matrix<std::complex<double>, Dynamic, Dynamic, RowMajor> timeMRowMajor(timeM);
//  Matrix<std::complex<double>, Dynamic, Dynamic, RowMajor> freqM(timeM.rows(), timeM.cols());
//
//  //TODO: THIS isn't threadsafe!
//  /* Create plan */
//  fftw_plan plan = fftw_plan_dft_2d(timeMRowMajor.rows(), timeMRowMajor.cols(), fftw_cast(timeMRowMajor.data()),
//      fftw_cast(freqM.data()), FFTW_FORWARD, FFTW_ESTIMATE);
//
//  /* Compute forward DFT */
//  fftw_execute(plan);
//
//  /* Free plan memory */
//  fftw_destroy_plan(plan);
//
//  return freqM;
//}
//
//MatrixXcd eigen_fft_inv2(const MatrixXcd &freqM)
//{
//  Matrix<std::complex<double>, Dynamic, Dynamic, RowMajor> freqMRowMajor(freqM);
//  Matrix<std::complex<double>, Dynamic, Dynamic, RowMajor> timeM(freqM.rows(), freqM.cols());
//
//  /* Create plan */
//  fftw_plan plan = fftw_plan_dft_2d(freqMRowMajor.rows(), freqMRowMajor.cols(), fftw_cast(freqMRowMajor.data()),
//      fftw_cast(timeM.data()), FFTW_BACKWARD, FFTW_ESTIMATE);
//
//  /* Compute forward DFT */
//  fftw_execute(plan);
//
//  /* Free plan memory */
//  fftw_destroy_plan(plan);
//
//  timeM *=1./(timeM.size());
//
//  return timeM;
//
//}

MatrixXd flattenComplex(const MatrixXcd &m)
{
  MatrixXd fm(m.rows() * 2, m.cols());
  fm << m.real(), m.imag();
  return fm;
}

int main(int argc, char * argv[])
{
  int r = 2;
  int c = 3;

  complex_2d_dft(r, c);

  MatrixXcd timeM(c, r); //we're gonna tranpose to get row major ordering
  for (int i = 0; i < timeM.size(); i++) {
    timeM(i) = i * 0.1;
  }
  timeM.transposeInPlace();

  eigen_matlab_dump(timeM.real());
  eigen_dump(timeM);
  MatrixXcd freqM = eigen_utils::fftw_fwd2(timeM);
  eigen_dump(freqM);
  eigen_dump(flattenComplex(freqM));

  MatrixXcd timeM2 = eigen_utils::fftw_inv2(freqM);
  eigen_dump(timeM2);

  MatrixXd timeMr = timeM.real();
  MatrixXcd freqMr = eigen_utils::fftw_fwd2r(timeMr);

  // manipulate freqM
//  fft.inv2( timeM,freqM);

}
