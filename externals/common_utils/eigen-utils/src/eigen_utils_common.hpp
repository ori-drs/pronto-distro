#ifndef __eigen_common_h__
#define __eigen_common_h__

#include <Eigen/Dense>
#include <iostream>
#include <algorithm>
#include <vector>
#include <utility>
// Define isnan() function for OSX
#if defined(__APPLE__)
#if (__GNUC__ >= 4)
//#define isnan(X) __inline_isnan(X)
#define isnan(X) std::isnan(X)
#else
#define isnan(X) __isnan(X)
#endif
#endif

namespace eigen_utils {
#define eigen_dump(MAT) std::cout << #MAT << std::endl << (MAT) << std::endl

#define eigen_matlab_dump(MAT) std::cout << #MAT << "=[" << (MAT) << "];\n"

#define var_dump(VAR) std::cout << #VAR << std::endl << (VAR) << std::endl

double atan2Vec(const Eigen::Vector2d & vec);

Eigen::Vector2d angleToVec(double angle);

void angleToVec(double angle, Eigen::Vector2d & unit_vec);

//utilities for flattening an symmetric matrices (ie cov matrices)
Eigen::VectorXd flattenSymmetric(Eigen::MatrixXd symm);
Eigen::MatrixXd unflattenSymmetric(Eigen::VectorXd flat);

/**
 * alpha is (time step)/(filter time constant)
 */
template<typename Type>
class ExponentialFilter {
public:

  ExponentialFilter(const Type & init_val, double alpha) :
      val(init_val), alpha()
  {
  }

  const Type & step(const Type & new_val)
  {
    val = alpha * new_val + (1 - alpha) * val;
  }

  const Type & operator()() const
  {
    return getVal();
  }

  const Type & getVal() const
  {
    return val;
  }

  const double & getAlpha() const
  {
    return alpha;
  }

  void setAlpha(double _alpha)
  {
    alpha = _alpha;
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  double alpha;
  Type val;
}
;

template<typename Derived>
bool hasNan(const Eigen::DenseBase<Derived> & m)
{
  for (int ii = 0; ii < m.rows(); ii++) {
    for (int jj = 0; jj < m.cols(); jj++) {
      if (isnan(m(ii, jj)))
        return true;
    }
  }
  return false;
}

template<typename Derived>
bool assertNoNan(const Eigen::DenseBase<Derived> & m)
{
  if (hasNan(m)) {
    std::cerr << "ERROR: m has a NAN!\n";
    eigen_dump(m);
    assert(false);
  }
}

template<typename Derived>
int numNonZeros(const Eigen::DenseBase<Derived> & m)
{
  return m.count();
}

template<typename Derived>
Eigen::ArrayXi findNonZeros(const Eigen::DenseBase<Derived> & arr)
{
  int nnz = numNonZeros(arr);
  Eigen::ArrayXi nz(nnz);
  int cnt = 0;
  for (int i = 0; i < arr.size(); i++) {
    if (arr(i))
      nz(cnt++) = i;
  }
  return nz;
}

/**
 * Compute the median of the values stored in this eigen array.
 *
 * This spares the copy required in the normal version, and therefore will be
 * faster for large datasets.
 *
 *  This Quickselect routine is based on the algorithm described in
 *  "Numerical recipes in C", Second Edition,
 *  Cambridge University Press, 1992, Section 8.5, ISBN 0-521-43108-5
 *  This code was originally by Nicolas Devillard - 1998. Public domain.
 *
 */
template<typename Derived>
typename Derived::Scalar median_non_const(Eigen::DenseBase<Derived> & arr)
{
  int low, high;
  int median;
  int middle, ll, hh;

  low = 0;
  high = arr.size() - 1;
  median = (low + high) / 2;
  while (1) {
    if (high <= low) /* One element only */
      return arr(median);

    if (high == low + 1) { /* Two elements only */
      if (arr(low) > arr(high))
        std::swap(arr(low), arr(high));
      return arr(median);
    }

    /* Find median of low, middle and high items; swap into position low */
    middle = (low + high) / 2;
    if (arr(middle) > arr(high))
      std::swap(arr(middle), arr(high));
    if (arr(low) > arr(high))
      std::swap(arr(low), arr(high));
    if (arr(middle) > arr(low))
      std::swap(arr(middle), arr(low));

    /* Swap low item (now in position middle) into position (low+1) */
    std::swap(arr(middle), arr(low + 1));

    /* Nibble from each end towards middle, swapping items when stuck */
    ll = low + 1;
    hh = high;
    while (1) {
      do
        ll++;
      while (arr(low) > arr(ll));
      do
        hh--;
      while (arr(hh) > arr(low));

      if (hh < ll)
        break;

      std::swap(arr(ll), arr(hh));
    }

    /* Swap middle item (in position low) back into correct position */
    std::swap(arr(low), arr(hh));

    /* Re-set active partition */
    if (hh <= median)
      low = ll;
    if (hh >= median)
      high = hh - 1;
  }
}

/**
 * Compute the median of the values stored in this eigen array.
 *
 */
template<typename Derived>
typename Derived::Scalar median(const Eigen::DenseBase<Derived> & const_arr)
{

  typename Derived::PlainObject arr = const_arr; //make a local copy... would be nice if this wasn't necessary :-/
  return median_non_const(arr);
}

template<typename Scalar>
Eigen::Array<Scalar, Eigen::Dynamic, 1> sort(const Eigen::Array<Scalar, Eigen::Dynamic, 1> & arr,
Eigen::ArrayXi &sorted_inds)
{
  typedef std::pair<Scalar, int> scalar_index_pair;
  std::vector<scalar_index_pair> sort_vec;
  sort_vec.reserve(arr.size());
  for (int i = 0; i < arr.size(); i++) {
    sort_vec.push_back(scalar_index_pair(arr(i), i));
  }
  std::sort(sort_vec.begin(), sort_vec.end());

  Eigen::Array<Scalar, Eigen::Dynamic, 1> sorted(arr.rows(), arr.cols());
  sorted_inds.resizeLike(arr);
  for (int i = 0; i < arr.size(); i++) {
    sorted(i) = sort_vec[i].first;
    sorted_inds(i) = sort_vec[i].second;
  }

  return sorted;
}

} //namespace eigen_utils
#endif
