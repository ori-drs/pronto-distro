#include <eigen_utils/eigen_utils.hpp>
#include <lcm/lcm.h>

using namespace Eigen;
using namespace eigen_utils;
using namespace std;

int main(int argc, char * argv[])
{

  RigidBodyState st;
  st.position() = Vector3d(1, 2, 3);
  st.velocity() = Vector3d(3, 4, 5);
  st.setQuatEulerAngles(Vector3d(0, M_PI / 3, 0));

//  VectorXd st;
//  st.setConstant(15,1);

  eigen_dump(st);

  return 0;

}
