#ifndef __stl_string_utils_hpp__
#define __stl_string_utils_hpp__
#include <sstream>

namespace stl_utils {

template<class T>
inline const std::string to_string(const T& t)
{
  std::stringstream ss;
  ss << std::boolalpha << std::showpoint << t;
  return ss.str();
}

//overload std::string and char to put quotes around them
template<>
inline const std::string to_string<std::string>(const std::string & t)
{
  std::stringstream ss;
  ss << "\"" << t << "\"";
  return ss.str();
}
template<>
inline const std::string to_string<char>(const char & t)
{
  std::stringstream ss;
  ss << "'" << t << "'";
  return ss.str();
}

}
#endif
