#include <eigen_utils/eigen_utils.hpp>
#include <lcm/lcm.h>


using namespace Eigen;
using namespace eigen_utils;
using namespace std;

int main(int argc, char * argv[])
{

  MatrixXf test = MatrixXf::Random(5, 5);
  eigen_dump(test);
  eigen_dense_t msg = toLcmMsg(test);
  eigen_dump(msg.type);
  MatrixXf test2 = fromLcmMsg<MatrixXf>(&msg);
  eigen_dump(test2);


  VectorXd vtest = VectorXd::Random(5);
  eigen_dump(vtest);
  eigen_dense_t msg2 = toLcmMsg(vtest);
  eigen_dump(msg2.type);

  VectorXd vtest2 = fromLcmMsg<VectorXd>(&msg2);
  eigen_dump(vtest2);

  ArrayXd atest2 = fromLcmMsg<ArrayXd>(&msg2);
  eigen_dump(atest2);

}
