#ifndef __stl_fstream_utils_hpp__
#define __stl_fstream_utils_hpp__
#include <fstream>

namespace stl_utils {

bool fileExists(const std::string & fileName)
{
    std::ifstream infile(fileName.c_str());
    return infile.good();
}

}
#endif
