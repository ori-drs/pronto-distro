#ifndef LcmMatlabRpc_HPP_
#define LcmMatlabRpc_HPP_
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/eigen_utils.hpp>
#include <Eigen/Dense>
#include <vector>

namespace eigen_utils {

class LcmMatlabRpc {
public:
  LcmMatlabRpc(const std::string &_name = "MATLAB_LCM_RPC", bool verbose=false);
  ~LcmMatlabRpc();

  //main function taking a variable number of input and output args
  int run(const std::string & command, const std::vector<Eigen::MatrixXd> & args, int numReturnVals,
      std::vector<Eigen::MatrixXd> &ret, int64_t timeout = -1);

  //conveniance functions taking a fixed number of arguments, and a single return argument
  Eigen::MatrixXd run(const std::string & command, const Eigen::MatrixXd & arg1, int64_t timeout = -1);
  Eigen::MatrixXd run(const std::string & command, const Eigen::MatrixXd & arg1, const Eigen::MatrixXd & arg2,
      int64_t timeout = -1);
  Eigen::MatrixXd run(const std::string & command, const Eigen::MatrixXd & arg1, const Eigen::MatrixXd & arg2,
      const Eigen::MatrixXd & arg3, int64_t timeout = -1);

private:
  void handleReturn(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
      const eigen_utils::matlab_rpc_return_t* msg);
  void handleAck(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eigen_utils::matlab_rpc_ack_t* msg);

  lcm::LCM lcm;
  bool received_ack;

  eigen_utils::matlab_rpc_return_t * ret_msg;
  int nonce;
  std::string name;
  lcm::Subscription * ret_sub;
  lcm::Subscription * ack_sub;
  bool verbose;
};

} /* namespace eigen_utils */
#endif /* LCMRPC_HPP_ */
