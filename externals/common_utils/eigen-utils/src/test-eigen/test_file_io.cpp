#include "../eigen_file_io.hpp"
#include "../eigen_utils_common.hpp"

using namespace Eigen;
using namespace eigen_utils;
using namespace std;

int main(int argc, char * argv[])
{

  MatrixXd m = MatrixXd::Random(6, 8);
  writeToFile("testEigen.eg", m);
  eigen_dump(m);

  MatrixXd m2 = readFromFile<MatrixXd>("testEigen.eg");
  eigen_dump(m2);

  writeToFile("testEigenBlock.eg", m.block(0, 0, 3, 3));

  MatrixXd m2b = readFromFile<MatrixXd>("testEigenBlock.eg");
  eigen_dump(m2b);

  ofstream ofs("testMultiMat.eg", ios::binary);

  MatrixXd w1 = MatrixXd::Random(6, 8);
  MatrixXd w2 = MatrixXd::Random(2, 4);
  MatrixXd w3 = MatrixXd::Random(3, 8);
  eigen_dump(w1);
  eigen_dump(w2);
  eigen_dump(w3);

  std::vector<MatrixXd> wmats;
  wmats.push_back(w1);
  wmats.push_back(w2);
  wmats.push_back(w3);
  writeMultipleToFile("testMultiMat.eg", wmats);

  std::vector<ArrayXXd> wmats2;
  wmats2.push_back(w1);
  wmats2.push_back(w2);
  wmats2.push_back(w3);
  writeMultipleToFile("testMultiMat.eg", wmats2);

  std::vector<MatrixXd> rmats = readMultipleFromFile<MatrixXd>("testMultiMat.eg");
  for (int i = 0; i < rmats.size(); i++)
    eigen_dump(rmats[i]);
}
