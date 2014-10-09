#ifndef __EIGEN_SELECT_BLOCK_DIRECT_INCLUDE__
#error "\nThis .hxx should not be included directly -- Include the .hpp instead!\n"
#endif

template<typename DerivedData, typename DerivedInd>
typename DerivedData::PlainObject selectRowsByIndices(const Eigen::DenseBase<DerivedData> & m, const Eigen::DenseBase<
    DerivedInd> & inds)
{
  typename DerivedData::PlainObject ret(inds.size(), m.cols());
  for (int i = 0; i < inds.size(); i++) {
    ret.row(i) = m.row(inds(i));
  }
  return ret;
}

template<typename DerivedData, typename DerivedInd>
typename DerivedData::PlainObject selectColsByIndices(const Eigen::DenseBase<DerivedData> & m, const Eigen::DenseBase<
    DerivedInd> & inds)
{
  typename DerivedData::PlainObject ret(m.rows(), inds.size());
  for (int i = 0; i < inds.size(); i++) {
    ret.col(i) = m.col(inds(i));
  }
  return ret;
}

template<typename DerivedData, typename DerivedInd>
typename DerivedData::PlainObject selectBlockByIndices(const Eigen::DenseBase<DerivedData> & m, const Eigen::DenseBase<
    DerivedInd> & rinds, const Eigen::DenseBase<DerivedInd> & cinds)
{
  typename DerivedData::PlainObject ret(rinds.rows(), cinds.rows());
  for (int r = 0; r < rinds.size(); r++) {
    for (int c = 0; c < cinds.size(); c++) {
      ret(r, c) = m(rinds(r), cinds(c));
    }
  }
  return ret;
}

//functions for pulling out parts of an eigen matrix by a indicator vector/array
template<typename DerivedData, typename DerivedInd>
typename DerivedData::PlainObject selectRowsByIndicator(const Eigen::DenseBase<DerivedData> & m,
    const Eigen::DenseBase<DerivedInd> & inds)
{
  eigen_assert(m.rows() == inds.size());
  typename DerivedData::PlainObject ret(numNonZeros(inds), m.cols());
  int cnt = 0;
  for (int i = 0; i < inds.size(); i++) {
    if (inds(i))
      ret.row(cnt++) = m.row(i);
  }
  return ret;
}

template<typename DerivedData, typename DerivedInd>
typename DerivedData::PlainObject selectColsByIndicator(const Eigen::DenseBase<DerivedData> & m,
    const Eigen::DenseBase<DerivedInd> & inds)
{
  eigen_assert(m.cols() == inds.size());
  typename DerivedData::PlainObject ret(m.rows(), numNonZeros(inds));
  int cnt = 0;
  for (int i = 0; i < inds.size(); i++) {

    if (inds(i)) {
      ret.col(cnt++) = m.col(i);
    }

  }

  return ret;
}

template<typename DerivedData, typename DerivedInd>
typename DerivedData::PlainObject selectBlockByIndicators(const Eigen::DenseBase<DerivedData> & m,
    const Eigen::DenseBase<DerivedInd> & rinds, const Eigen::DenseBase<DerivedInd> & cinds)
{
  eigen_assert(m.rows() == rinds.size());
  eigen_assert(m.cols() == cinds.size());
  typename DerivedData::PlainObject ret(numNonZeros(rinds), numNonZeros(cinds));
  int rcnt = 0;
  for (int r = 0; r < rinds.size(); r++) {
    if (!rinds(r))
      continue;
    int ccnt = 0;
    for (int c = 0; c < cinds.size(); c++) {
      if (cinds(c))
        ret(rcnt, ccnt++) = m(r, c);
    }
    rcnt++;
  }
  return ret;
}

template<typename DerivedInd, typename DerivedAssign, typename DerivedData>
void assignRowsByIndices(const Eigen::DenseBase<DerivedInd> & inds, const Eigen::DenseBase<DerivedAssign> & assign_vals,
    Eigen::DenseBase<DerivedData> & m)
{
  for (int i = 0; i < inds.size(); i++) {
    m.row(inds(i)) = assign_vals.row(i);
  }
}

