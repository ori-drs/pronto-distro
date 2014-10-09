#ifndef __stl_utils_hpp__
#define __stl_utils_hpp__
#include <map>

namespace stl_utils {

template<typename K, typename val>
bool stlmap_get_lower(std::map<K, val> & m,
K key,
typename std::map<K, val>::iterator &lb)
{
  lb = m.lower_bound(key);
  if (lb == m.end()) {
    return false;
  }

  if (key != lb->first) {
    if (lb == m.begin())
      return false;
    else
      lb--;
  }
  return true;
}

template<typename K, typename val>
bool stlmap_get_upper(std::map<K, val> & m,
const K & key,
typename std::map<K, val>::iterator &ub)
{
  ub = m.upper_bound(key);
  if (ub == m.end()) {
    return false;
  }
  return true;
}

//get iterators to the entry in the map such that lb <= key < ub
//returns false if those entries don't exist
template<typename K, typename val>
bool stlmap_get_bounding(std::map<K, val> & m,
const K & key,
typename std::map<K, val>::iterator &lb,
typename std::map<K, val>::iterator &ub)
{
  if (!stlmap_get_lower(m, key, lb))
    return false;
  return stlmap_get_upper(m, key, ub);
}



//multimap

template<typename K, typename val>
bool stlmultimap_get_lower(std::multimap<K, val> & m,
K key,
typename std::multimap<K, val>::iterator &lb)
{
  lb = m.lower_bound(key);
  if (lb == m.end()) {
    return false;
  }

  if (key != lb->first) {
    if (lb == m.begin())
      return false;
    else
      lb--;
  }
  return true;
}

template<typename K, typename val>
bool stlmultimap_get_upper(std::multimap<K, val> & m,
const K & key,
typename std::multimap<K, val>::iterator &ub)
{
  ub = m.upper_bound(key);
  if (ub == m.end()) {
    return false;
  }
  return true;
}

//get iterators to the entry in the multimap such that lb <= key < ub
//returns false if those entries don't exist
template<typename K, typename val>
bool stlmultimap_get_bounding(std::multimap<K, val> & m,
const K & key,
typename std::multimap<K, val>::iterator &lb,
typename std::multimap<K, val>::iterator &ub)
{
  if (!stlmultimap_get_lower(m, key, lb))
    return false;
  return stlmultimap_get_upper(m, key, ub);
}


}

#endif
