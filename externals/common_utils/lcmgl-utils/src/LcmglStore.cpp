#include "LcmglStore.hpp"
#include <iostream>
#include <stdlib.h>

namespace lcmgl_utils {

using namespace std;

LcmglStore::LcmglStore(lcm_t * lcm_, const std::string & prefix_) :
    lcm(lcm_), prefix(prefix_)
{

}
LcmglStore::LcmglStore() :
    lcm(NULL), prefix("uninitialized")
{

}

LcmglStore::~LcmglStore()
{
  std::map<string, bot_lcmgl_t *>::iterator it;
  for (it = lcmgls.begin(); it != lcmgls.end(); it++) {
    bot_lcmgl_t * lcmgl = it->second;
    bot_lcmgl_destroy(lcmgl);
  }
}

void LcmglStore::switchAllBuffers()
{
  std::map<string, bot_lcmgl_t *>::iterator it;
  for (it = lcmgls.begin(); it != lcmgls.end(); it++) {
    bot_lcmgl_t * lcmgl = it->second;
    bot_lcmgl_switch_buffer(lcmgl);
  }
}

bot_lcmgl_t * LcmglStore::getLcmgl(const std::string & name)
{
  if (lcm == NULL) {
    std::cerr << "ERROR: lcmglStore " << prefix << " lcm object is NULL\n";
    exit(1);
  }

  std::string full_name;
  if (prefix.size() > 0)
    full_name = prefix + "-" + name;
  else
    full_name = name;

  std::map<string, bot_lcmgl_t *>::iterator it;
  it = lcmgls.find(full_name);
  if (it == lcmgls.end()) {
    bot_lcmgl_t * lcmgl = bot_lcmgl_init(lcm, full_name.c_str());
    lcmgls.insert(pair<string, bot_lcmgl_t *>(full_name, lcmgl));
    return lcmgl;
  }
  else {
    return it->second;
  }
}

} /* namespace lcmgl_utils */
