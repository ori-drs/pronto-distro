#include "../eigen_lcm_matlab_rpc.hpp"
#include <iostream>

using namespace std;
using namespace eigen_utils;

int main(int argc, char ** argv)
{

  LcmMatlabRpc * matlab_rpc = new LcmMatlabRpc();

  vector<Eigen::MatrixXd> args;

  Eigen::MatrixXd a1(2, 3);
  Eigen::MatrixXd a2(2, 3);
  a1 << 5, 4, 234, 4, 1, 2;
  a2 << 1, 2, 3, 555, 4, 3;
  args.push_back(a1);
  args.push_back(a2);
  vector<Eigen::MatrixXd> ret;
  int ret_val = matlab_rpc->run("max", args, 1, ret);
  if (!ret_val)
    printf("matlab rpc failed!\n");
  else
    cerr << "ret:\n" << ret[0] << "\n";

  return 0;
}

