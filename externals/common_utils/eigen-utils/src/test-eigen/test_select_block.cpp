#include <eigen_utils/eigen_utils.hpp>

using namespace Eigen;
using namespace eigen_utils;
using namespace std;

int main(int argc, char * argv[])
{

  ArrayXXd test = ArrayXXd::Random(5, 5);
  ArrayXXd test2 = ArrayXXd::Random(5, 5);
  ArrayXi indices = ArrayXi::LinSpaced(3, 0, 3);

  test.max(test2);


}
