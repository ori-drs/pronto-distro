#include "eigen_lcm_matlab_rpc.hpp"
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/eigen_utils.hpp>
#include <bot_core/bot_core.h>
#include <eigen_utils/eigen_lcm.hpp>

namespace eigen_utils {

LcmMatlabRpc::LcmMatlabRpc(const std::string &_name, bool _verbose) :
    lcm(), ret_msg(NULL), received_ack(false), name(_name), verbose(_verbose)
{
  ret_sub = lcm.subscribe(name + "_RETURN", &LcmMatlabRpc::handleReturn, this);
  ack_sub = lcm.subscribe(name + "_CMD_ACK", &LcmMatlabRpc::handleAck, this);

}
LcmMatlabRpc::~LcmMatlabRpc()
{
  lcm.unsubscribe(ret_sub);
  lcm.unsubscribe(ack_sub);
}

void LcmMatlabRpc::handleReturn(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const eigen_utils::matlab_rpc_return_t* msg)
{
  if (msg->nonce == nonce) {
    if (verbose)
      fprintf(stderr, "received return!\n");
    ret_msg = new eigen_utils::matlab_rpc_return_t(*msg);
  }
  else if (verbose) {
    fprintf(stderr, "received duplicate? return %d\n", msg->nonce);
  }

  //send back an ack
  eigen_utils::matlab_rpc_ack_t ack_msg;
  ack_msg.nonce = msg->nonce;
  lcm.publish(name + "_RET_ACK", &ack_msg);
}
void LcmMatlabRpc::handleAck(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const eigen_utils::matlab_rpc_ack_t* msg)
{
  if (msg->nonce == nonce) {
    if (verbose)
      fprintf(stderr, "received ack!\n");
    received_ack = true;
  }
  else if (verbose) {
    fprintf(stderr, "received duplicate? ack %d\n", msg->nonce);
  }
}

int LcmMatlabRpc::run(const std::string & command, const std::vector<Eigen::MatrixXd> & args, int numReturnVals,
    std::vector<Eigen::MatrixXd> &ret, int64_t timeout)
{
  std::string chan = name + "_CMD";
  eigen_utils::matlab_rpc_command_t cmd;
  nonce = rand();
  cmd.nonce = nonce;
  cmd.command = command;
  cmd.numArgs = args.size();
  cmd.args.resize(cmd.numArgs);
  for (int i = 0; i < cmd.numArgs; i++) {
    cmd.args[i] = eigen_utils::toLcmMsg(args[i]);
  }
  cmd.numReturnVals = numReturnVals;

  ret_msg = NULL;
  received_ack = false;

  int64_t startTime = bot_timestamp_now();
  int64_t lastSendTime = 0;
  while (ret_msg == NULL) {
    if (!received_ack && bot_timestamp_now() - lastSendTime > 1e6) {
      if (verbose)
        fprintf(stderr, "sending command %s, %d\n", cmd.command.c_str(), cmd.nonce);
      lcm.publish(chan, &cmd);
      lastSendTime = bot_timestamp_now();
    }

    bot_lcm_handle_or_timeout(lcm.getUnderlyingLCM(), 1e5);
    if (timeout > 0 && bot_timestamp_now() - startTime > timeout)
      break;
  }

  if (ret_msg == NULL || ret_msg->return_status == 0) {
    fprintf(stderr, "matlab RPC failed with error:\n %s\n", ret_msg->error_msg.c_str());
    return 0;
  }

  ret.resize(ret_msg->numReturnVals);
  for (int i = 0; i < ret_msg->numReturnVals; i++) {
    ret[i] = eigen_utils::fromLcmMsg<Eigen::MatrixXd>(&ret_msg->returnVals[i]);
  }
  delete ret_msg;
  ret_msg = NULL;

  return 1;
}

Eigen::MatrixXd LcmMatlabRpc::run(const std::string & command, const Eigen::MatrixXd & arg1, int64_t timeout)
{
  std::vector<Eigen::MatrixXd> args, rets;
  args.push_back(arg1);
  int ret_val = run(command, args, 1, rets, timeout);
  if (!ret_val || rets.size() != 1) {
    return Eigen::MatrixXd(0, 0);
  }
  return rets[0];
}
Eigen::MatrixXd LcmMatlabRpc::run(const std::string & command, const Eigen::MatrixXd & arg1,
    const Eigen::MatrixXd & arg2, int64_t timeout)
{
  std::vector<Eigen::MatrixXd> args, rets;
  args.push_back(arg1);
  args.push_back(arg2);
  int ret_val = run(command, args, 1, rets, timeout);
  if (!ret_val || rets.size() != 1) {
    return Eigen::MatrixXd(0, 0);
  }
  return rets[0];
}
Eigen::MatrixXd LcmMatlabRpc::run(const std::string & command, const Eigen::MatrixXd & arg1,
    const Eigen::MatrixXd & arg2, const Eigen::MatrixXd & arg3, int64_t timeout)
{
  std::vector<Eigen::MatrixXd> args, rets;
  args.push_back(arg1);
  args.push_back(arg2);
  args.push_back(arg3);
  int ret_val = run(command, args, 1, rets, timeout);
  if (!ret_val || rets.size() != 1) {
    return Eigen::MatrixXd(0, 0);
  }
  return rets[0];
}

} /* namespace eigen_utils */

