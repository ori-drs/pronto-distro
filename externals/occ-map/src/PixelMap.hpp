#ifndef __OCC_MAP_PIXEL_MAP_HPP__
#define __OCC_MAP_PIXEL_MAP_HPP__


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>


//#define NO_LCM
#ifndef NO_LCM
#include <lcmtypes/occ_map_pixel_map_t.h>
#include <zlib.h>
#include <fstream>
#include <typeinfo>
#endif


namespace occ_map {

template<class T>
class PixelMap {
public:
  //metadata
  double xy0[2], xy1[2];
  double metersPerPixel;
  int dimensions[2];
  int num_cells;
#ifndef NO_LCM
  occ_map_pixel_map_t * msg;
#endif
  int64_t utime;
  // the actual storage array
  T* data;

  // normal constructor
  PixelMap<T>(const double _xy0[2], const double _xy1[2], double mPP, T initValue = T(), bool allocate_data = true, bool align_to_pixels = true);

  // Copy Constructor
  template<class F>
  PixelMap<T>(const PixelMap<F> * to_copy, bool copyData=true, T(*transformFunc)(F) = NULL);

#ifndef NO_LCM
  // Constructor from an lcm message
  PixelMap<T>(const occ_map_pixel_map_t * _msg);

  // Constructor from a file (created with "saveToFile")
  PixelMap<T>(const std::string & name);
#endif

  ~PixelMap<T>();

  // set all values in the map to 0
  void reset(T resetVal = T());

  //get linear index into storage arrays
  inline int getInd(const int ixy[2]) const;
  inline int getInd(const double xy[2]) const;

  // Map backwards from an index to a location
  inline void indToLoc(int ind, int ixy[2]) const;
  inline void indToLoc(int ind, double xy[2]) const;

  // convert from world coordinates into the map
  inline void worldToTable(const double xy[2], int ixy[2]) const;
  // convert from the map coordinates to world coordinates
  inline void tableToWorld(const int ixy[2], double xy[2]) const;

  // check whether a location is inside the map bounds
  inline bool isInMap(const int ixy[2]) const;
  inline bool isInMap(const double xy[2]) const;

  //read the value contained in this cell
  inline T & readValue(const int ixy[2]) const;
  inline T & readValue(const double xy[2]) const;

  //write the value in the cell
  inline void writeValue(const int ixy[2], T val);
  inline void writeValue(const double xy[2], T val);

  //add the value to the cell with optional value clamping
  inline void updateValue(const int ixy[2], T inc, T clamp_bounds[2] = NULL);
  inline void updateValue(const double xy[2], T inc, T clamp_bounds[2] = NULL);

  //step along the line segment from start to end, updating with miss_inc along the way, and update by hit_inc at end
  void rayTrace(const int start[2], const int end[2], T miss_inc, T hit_inc, T clamp_bounds[2] = NULL);
  void rayTrace(const double start[2], const double end[2], T miss_inc, T hit_inc, T clamp_bounds[2] = NULL);

  //check whether any of the cells between start and end are greater than occ_thresh
  bool collisionCheck(const int start[2], const int end[2], T occ_thresh, int collisionPoint[2] = NULL) const;
  bool collisionCheck(const double start[2], const double end[2], T occ_thresh, double collisionPoint[2] = NULL) const;

#ifndef NO_LCM
  //convert the pixelmap into an LCM message
  const occ_map_pixel_map_t *get_pixel_map_t(int64_t utime);
  void set_from_pixel_map_t(const occ_map_pixel_map_t * _msg);

  //save the pixelmap to a file
  void saveToFile(const std::string & name);
  //load the pixelmap from a file
  void loadFromFile(const std::string & name);
#endif

private:
  template<class F>
  inline F clamp_value(F x, F min, F max) const;

};

#ifndef NO_LCM
//static function to load pixel_map message directly from a file
static occ_map_pixel_map_t * load_pixel_map_t_from_file(const std::string & name);
#endif

//typedefs for ease of use
typedef PixelMap<float> FloatPixelMap;
typedef PixelMap<int32_t> IntPixelMap;
typedef PixelMap<uint8_t> Uint8PixelMap;

//include the actual implimentations
#define __PIXELMAP_DIRECT_INCLUDE__
#include "PixelMap.hxx"

}

#endif /*GRIDMAP_H_*/
