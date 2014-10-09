#ifndef __eigen_numerical_h__
#define __eigen_numerical_h__

#include <stdio.h>
#include <Eigen/Dense>
#include "eigen_utils_common.hpp"

namespace eigen_utils {

template<typename DerivedQ, typename DerivedA, typename Derivedb, typename Derivedx>
void quadProgEliminationSolve(const Eigen::MatrixBase<DerivedQ> & Q, const Eigen::MatrixBase<DerivedA> & A,
    const Eigen::MatrixBase<Derivedb> & b, Eigen::MatrixBase<Derivedx> & x_star)
{
  int m = A.rows();
  int n = A.cols();
  assert(n>=m);

  assert(Q.cols()==Q.rows());
  assert(Q.cols()==n);
  assert(b.rows()==m);
  assert(b.cols()==1);
  assert(x_star.cols()==1);
  assert(x_star.rows()==n);

  if (m == n) {

//    Eigen::ColPivHouseholderQR<Eigen::MatrixBase<DerivedA> > A_decomp;
//    A_decomp.compute(A);
//    if (!A_decomp.isInvertible()) {
//      fprintf(stderr,
//          "Warning: A matrix for fully constrained quadProgEliminationSolve in %s is not invertible, line %d\n",
//          __FILE__, __LINE__);
//    }
    //    x_star = A_decomp.solve(b);

    if (!A.colPivHouseholderQr().isInvertible()) {
      fprintf(stderr,
          "Warning: A matrix for fully constrained quadProgEliminationSolve in %s is not invertible, line %d\n",
          __FILE__, __LINE__);
    }
    x_star = A.colPivHouseholderQr().solve(b);
  }
  else {
    Eigen::MatrixXd B = A.leftCols(m);
    Eigen::MatrixXd R = A.rightCols(n - m);

    Eigen::MatrixXd Q_BB = Q.topLeftCorner(m, m);
    Eigen::MatrixXd Q_BR = Q.topRightCorner(m, n - m);
    Eigen::MatrixXd Q_RB = Q.bottomLeftCorner(n - m, m);
    Eigen::MatrixXd Q_RR = Q.bottomRightCorner(n - m, n - m);

    /*
     *  ColPivHouseholderQR (faster, less accurate)
     *  FullPivHouseholderQR (slower, more accurate)
     *  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> B_decomp;
     *  Eigen::FullPivHouseholderQR<Eigen::MatrixXd> B_decomp;
     */

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> B_decomp;
    B_decomp.compute(B);

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> B_transpose_decomp;
    B_transpose_decomp.compute(B.transpose());
    if (!B_decomp.isInvertible()) {
      fprintf(stderr, "Warning: B matrix in A=[B R] for quadProgEliminationSolve in %s is not invertible, line %d\n",
          __FILE__, __LINE__);
    }
    Eigen::MatrixXd B_inv_R = B_decomp.solve(R);

    //f_prime_RR=2*b'*(B'\(Q_BR-(Q_BB/B)*R));
    Eigen::VectorXd f_prime_RR;
//    eigen_dump(b);
//    eigen_dump(Q_BR);
//    eigen_dump(R);
//    eigen_dump(Q_BB);
//    eigen_dump(B_inv_R);
    f_prime_RR = (2 * b.transpose() * (B_transpose_decomp.solve(Q_BR - Q_BB * B_inv_R))).transpose();

    //    Q_prime_RR=...
    //          Q_RR+R'*(B'\Q_BB/B)*R...
    //          -(R'/B')*Q_BR...
    //          -(Q_RB/B)*R;
    Eigen::MatrixXd Q_prime_RR = Q_RR + B_inv_R.transpose() * Q_BB * B_inv_R - B_inv_R.transpose() * Q_BR
        - Q_RB * B_inv_R;

    /*
     *  ColPivHouseholderQR (faster, less accurate)
     *  FullPivHouseholderQR (slower, more accurate)
     *  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> Q_prime_RR_decomp;
     *  Eigen::FullPivHouseholderQR<Eigen::MatrixXd> Q_prime_RR_decomp;
     */
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> Q_prime_RR_decomp;
    Q_prime_RR_decomp.compute(Q_prime_RR);
    if (!Q_prime_RR_decomp.isInvertible()) {
      fprintf(stderr, "Warning: Q_primer_RR matrix in quadProgEliminationSolve in %s is not invertible, line %d\n",
          __FILE__, __LINE__);
    }

    //    x_R_star = -(2*Q_prime_RR)\f_prime_RR';
    //    x_B_star = B\(b-R*x_R_star);
    //    x_star = [x_B_star;x_R_star];
    x_star.bottomRows(n - m) = -0.5 * Q_prime_RR_decomp.solve(f_prime_RR);
    x_star.topRows(m) = B_decomp.solve(b - R * x_star.bottomRows(n - m));
  }
}

/**
 * fits a hyperplane to weighted particles by computing mean and covariance and aligning normal vec with the minimum variance axis in covariance
 *
 * data: columnwise data points
 * result_vec: orthoganol vector, scaled such that line satisfies result_vec.dot(y)=||result_vec||^2 for all points y on the line
 * weights: vector of weights for the fitting
 *
 * returns the standard deviation along that axis
 */
double fitHyperplaneLeastSquares(const Eigen::MatrixXd & data, const Eigen::VectorXd & weights,
    Eigen::VectorXd & result_vec);

/**
 * implements RANSAC for LS hyperplane fitting
 *
 * data: columnwise data points
 * num_iterations: number of ransac iterations
 * include_threshold: data point must lie within this distance of plane to be added to consensus set
 * min_num_include: miminum number of data points to be included to be considered a candidate for best fit
 *
 * result_vec: orthoganol vector, scaled such that line satisfies result_vec.dot(y)=||result_vec||^2 for all points y on the line
 * consensus_set: consensus_set(ind)==1 if data.col(ind) is included in best fit, 0 otherwise
 * returns: standard dev of best fit
 *
 */
double fitHyperPlaneRANSAC(const Eigen::MatrixXd & data, int num_iterations, double include_threshold,
    int min_num_include, Eigen::VectorXd & result_vec, Eigen::VectorXi & consensus_set);

}

#endif
