/*
 * Functions for pulling out parts of an eigen matrix using an array of indices
 * This should be deprecated relatively soon once Eigen supports this behavior internally!
 */

#ifndef EIGEN_SELECT_BLOCK_HPP_
#define EIGEN_SELECT_BLOCK_HPP_

namespace eigen_utils {

//functions for pulling out parts of an eigen matrix by a vector/array of indices
template<typename DerivedData, typename DerivedInd>
typename DerivedData::PlainObject selectBlockByIndices(const Eigen::DenseBase<DerivedData> & m, const Eigen::DenseBase<
    DerivedInd> & rinds, const Eigen::DenseBase<DerivedInd> & cinds);

template<typename DerivedData, typename DerivedInd>
typename DerivedData::PlainObject selectRowsByIndices(const Eigen::DenseBase<DerivedData> & m, const Eigen::DenseBase<
    DerivedInd> & inds);

template<typename DerivedData, typename DerivedInd>
typename DerivedData::PlainObject selectColsByIndices(const Eigen::DenseBase<DerivedData> & m, const Eigen::DenseBase<
    DerivedInd> & inds);

//functions for pulling out parts of an eigen matrix by a indicator vector/array
template<typename DerivedData, typename DerivedInd>
typename DerivedData::PlainObject selectBlockByIndicators(const Eigen::DenseBase<DerivedData> & m,
    const Eigen::DenseBase<DerivedInd> & rinds, const Eigen::DenseBase<DerivedInd> & cinds);

template<typename DerivedData, typename DerivedInd>
typename DerivedData::PlainObject selectRowsByIndicator(const Eigen::DenseBase<DerivedData> & m,
    const Eigen::DenseBase<DerivedInd> & inds);

template<typename DerivedData, typename DerivedInd>
typename DerivedData::PlainObject selectColsByIndicator(const Eigen::DenseBase<DerivedData> & m,
    const Eigen::DenseBase<DerivedInd> & inds);

template<typename DerivedInd, typename DerivedAssign, typename DerivedData>
void assignRowsByIndices(const Eigen::DenseBase<DerivedInd> & inds, const Eigen::DenseBase<DerivedAssign> & assign_vals,
    Eigen::DenseBase<DerivedData> & m);

//include the actual implimentations
#define __EIGEN_SELECT_BLOCK_DIRECT_INCLUDE__
#include "eigen_select_block.hxx"

} //namespace eigen_utils


#endif /* EIGEN_SELECT_BLOCK_HPP_ */
