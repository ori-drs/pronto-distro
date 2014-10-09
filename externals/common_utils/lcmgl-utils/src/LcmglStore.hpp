#ifndef LCMGLSTORE_HPP_
#define LCMGLSTORE_HPP_

#include <lcm/lcm.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <map>
#include <string>

namespace lcmgl_utils {

class LcmglStore {
public:
  LcmglStore(lcm_t * lcm, const std::string & prefix = "");
  LcmglStore();
  virtual ~LcmglStore();

  /**
   * Get an lcmgl object by name, creating it if necessary
   */
  // conveniance operators to get an lcmgl object
  bot_lcmgl_t * operator ()(const std::string & name)
  {
    return getLcmgl(name);
  }
  bot_lcmgl_t * operator [](const std::string & name)
  {
    return getLcmgl(name);
  }

  /**
   * Call bot_lcmgl_switch_buffer() on all of the managed lcmgl objects
   */
  void switchAllBuffers();

private:
  bot_lcmgl_t * getLcmgl(const std::string & name);
  lcm_t * lcm;
  std::string prefix;
  std::map<std::string, bot_lcmgl_t *> lcmgls;
};

} /* namespace lcmgl_utils */
#endif /* LCMGLSTORE_HPP_ */
