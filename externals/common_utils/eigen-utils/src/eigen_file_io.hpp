#ifndef EIGEN_FILE_IO_HPP_
#define EIGEN_FILE_IO_HPP_

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include "eigen_lcm.hpp"
#include <vector>
namespace eigen_utils {

template<typename Derived>
void writeToFile(std::ofstream &ofs, const Eigen::DenseBase<Derived> & m)
{
  using namespace std;
  int32_t rows = m.rows();
  int32_t cols = m.cols();
  string type = typenameToStr<typename Derived::Scalar>();
  int32_t typelen = type.length();
//  cerr << "rows: " << rows << " cols: " << cols << " typelen: " << typelen << " type: " << type << "\n";

  ofs.write((char *) &rows, sizeof(int32_t));
  ofs.write((char *) &cols, sizeof(int32_t));
  ofs.write((char *) &typelen, sizeof(int32_t));
  ofs.write(type.c_str(), typelen);
  for (int j = 0; j < m.cols(); j++) {
    for (int i = 0; i < m.rows(); i++) {
      ofs.write((char *) &m(i, j), sizeof(typename Derived::Scalar));
    }
  }
}
template<typename Derived>
void writeToFile(const std::string & name, const Eigen::DenseBase<Derived> & m)
{
  using namespace std;
  ofstream ofs(name.c_str(), ios::binary);
  if (!ofs.good()) {
    cerr << "ERROR: Could not open file " << name << endl;
  }
  writeToFile(ofs, m);
  ofs.close();
}

//template<typename Derived>
template<typename vectorType>
void writeMultipleToFile(const std::string & name, const vectorType & mats)
{
  using namespace std;
  ofstream ofs(name.c_str(), ios::binary);
  if (!ofs.good()) {
    cerr << "ERROR: Could not open file " << name << endl;
  }
  for (int i = 0; i < mats.size(); i++) {
    writeToFile(ofs, mats[i]);
  }
  ofs.close();
}

template<typename Derived>
typename Derived::PlainObject readFromFile(std::ifstream &ifs)
{
  typename Derived::PlainObject m;
  using namespace std;
  int32_t rows = 0, cols = 0;
  int32_t typelen = 0;
  char type[256] = { 0 };
  ifs.read((char *) &rows, sizeof(int32_t));
  ifs.read((char *) &cols, sizeof(int32_t));
  ifs.read((char *) &typelen, sizeof(int32_t));
  ifs.read((char *) type, typelen);
//  cerr << "rows: " << rows << " cols: " << cols << " typelen: " << typelen << " type: " << type << "\n";
  if (type != typenameToStr<typename Derived::Scalar>()) {
    cerr << "ERROR: file has type: " << type << " which doesn't match template type of "
        << typenameToStr<typename Derived::Scalar>() << "\n";
    return m;
  }
  m.resize(rows, cols);
  ifs.read((char *) m.derived().data(), rows * cols * sizeof(typename Derived::Scalar));
}

template<typename Derived>
typename Derived::PlainObject readFromFile(const std::string & name)
{
  typename Derived::PlainObject m;
  using namespace std;
  ifstream ifs(name.c_str(), ios::binary);
  if (!ifs.good()) {
    cerr << "ERROR: Could not open file " << name << endl;
    return m;
  }
  m = readFromFile<Derived>(ifs);
  ifs.close();
  return m;
}

template<typename Derived>
typename std::vector<typename Derived::PlainObject> readMultipleFromFile(const std::string & name)
{
  typename std::vector<typename Derived::PlainObject> mats;
  using namespace std;
  ifstream ifs(name.c_str(), ios::binary);
  if (!ifs.good()) {
    cerr << "ERROR: Could not open file " << name << endl;
    return mats;
  }
  while (ifs.peek() != ifstream::traits_type::eof()) {
    mats.push_back(readFromFile<Derived>(ifs));
  }
  ifs.close();
  return mats;
}

}
#endif /* EIGEN_FILE_IO_HPP_ */
