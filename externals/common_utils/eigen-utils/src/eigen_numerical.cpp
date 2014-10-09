#include "eigen_numerical.hpp"
#include "eigen_utils_common.hpp"
#include "eigen_rand.hpp"

namespace eigen_utils {

using namespace Eigen;

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
    Eigen::VectorXd & result_vec)
{

  using namespace Eigen;

  VectorXd mean;
  MatrixXd covariance;
  fitParticles(data, weights, mean, covariance);

  SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(covariance);
  VectorXd eig_vals = eigen_solver.eigenvalues();
  MatrixXd eig_vecs = eigen_solver.eigenvectors();

  int min_eig_ind;
  double min_var = eig_vals.minCoeff(&min_eig_ind);

  result_vec = eig_vecs.col(min_eig_ind).normalized();
  result_vec *= result_vec.dot(mean);

  return sqrt(min_var);
}

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
    int min_num_include, Eigen::VectorXd & result_vec, Eigen::VectorXi & consensus_set)
{

  int Ndata = data.cols();
  int Ndim = data.rows();

  VectorXi best_consensus_set = VectorXi::Zero(Ndata);
  VectorXd best_model = VectorXd::Zero(Ndim);
  consensus_set = best_consensus_set;
  result_vec = best_model;
  double best_error = INFINITY;

  if (Ndata < Ndim)
    return best_error;

  for (int ii = 0; ii < num_iterations; ii++) {
    VectorXi cur_consensus_set = VectorXi::Zero(Ndata);

    VectorXi init_consensus_inds = VectorXi(Ndim);

    init_consensus_inds(0) = (int) bot_randf_in_range(0, Ndata - .01); //hack to get and int in range
    cur_consensus_set(init_consensus_inds(0)) = 1;
    int num_consensus = 1;
    while (num_consensus < Ndim) {
      init_consensus_inds(num_consensus) = (int) bot_randf_in_range(0, Ndata - .01);
      if ((init_consensus_inds.head(num_consensus).array() == init_consensus_inds(num_consensus)).any())
        continue;

      cur_consensus_set(init_consensus_inds(num_consensus)) = 1;
      num_consensus++;
    }

    VectorXd model;
    fitHyperplaneLeastSquares(data, cur_consensus_set.cast<double>(), model);

    double offset = model.norm();
    model /= offset;

    for (int ii = 0; ii < Ndata; ii++) {
      if (cur_consensus_set(ii) == 1)
        continue;

      double resid = fabs(model.dot(data.col(ii)) - offset);
      if (resid < include_threshold) {
        cur_consensus_set(ii) = 1;
        num_consensus++;
      }
    }

    if (num_consensus > min_num_include) {
      double cur_error = fitHyperplaneLeastSquares(data, cur_consensus_set.cast<double>(), model);
      if (cur_error < best_error) {
        best_model = model;
        best_consensus_set = cur_consensus_set;
        best_error = cur_error;
      }
    }

  }

  consensus_set = best_consensus_set;
  result_vec = best_model;
  return best_error;
}

}

