#ifndef __EIGEN_LCM_HPP_
#define __EIGEN_LCM_HPP_

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/eigen_utils/eigen_dense_t.hpp>
#include <lcmtypes/eigen_utils/eigen_matrixxd_t.hpp>
#include <vector>
#include <string>
#include <iostream>

namespace eigen_utils {

template<typename Derived>
eigen_dense_t toLcmMsg(const Eigen::DenseBase<Derived> & mat);

template<typename Derived>
eigen_matrixxd_t toMatrixXdLcmMsg(const Eigen::DenseBase<Derived> & mat);

template<typename Derived>
typename Derived::PlainObject fromLcmMsg(const eigen_utils::eigen_dense_t * msg);

template<typename T> const std::string typenameToStr();


//include the actual implimentations
#define __EIGEN_LCM_DIRECT_INCLUDE__
#include "eigen_lcm.hxx"
}
#endif /* __EIGEN_LCM_HPP_ */
