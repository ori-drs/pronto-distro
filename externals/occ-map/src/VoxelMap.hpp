#ifndef __OCC_MAP_VOXELMAP_HPP__
#define __OCC_MAP_VOXELMAP_HPP__

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

//#define NO_LCM
#ifndef NO_LCM
#include <lcmtypes/occ_map_voxel_map_t.h>
#include <zlib.h>
#include <fstream>
#include <typeinfo>
#endif

namespace occ_map {

template<class T>
class VoxelMap {
public:
  //metadata
  double xyz0[3], xyz1[3];
  double metersPerPixel[3];
  int dimensions[3];
  int num_cells;

#ifndef NO_LCM
  occ_map_voxel_map_t *msg;
#endif
  int64_t utime;

  //the actual storage arrays
  T * data;

  // Normal Constructor
  VoxelMap<T>(const double _xyz0[3], const double _xyz1[3], const double _metersPerPixel[3], T initValue = T(),
      bool allocate_data = true);

  // Copy Constructor
  template<class F>
  VoxelMap<T>(const VoxelMap<F> * to_copy, bool copyData = true, T (*transformFunc)(F) = NULL);

#ifndef NO_LCM
  // Constructor from an LCM message
  VoxelMap<T>(const occ_map_voxel_map_t * _msg);

  // Constructor from a file (created with "saveToFile")
  VoxelMap<T>(const std::string & name);
#endif

  ~VoxelMap<T>();

  // set all values in the map to 0
  void reset(T resetVal = T());

  //get linear index into storage arrays
  inline int getInd(const int ixyz[3]) const;
  inline int getInd(const double xyz[3]) const;

  // Map backwards from an index to a location
  inline void indToLoc(int ind, int ixyz[3]) const;
  inline void indToLoc(int ind, double xyz[3]) const;

  // convert from world coordinates into the map
  inline bool worldToTable(const double xyz[3], int ixyz[3]) const;
  // convert from the map coordinates to world coordinates
  inline void tableToWorld(const int ixyz[3], double * xyz) const;

  // check whether a location is inside the map bounds
  inline bool isInMap(const int ixyz[3]) const;
  inline bool isInMap(const double xyz[3]) const;

  //read the value contained in this cell
  inline T readValue(const int ixyz[3]) const;
  inline float readValue(const double xyz[3]) const;

  //write the value in the cell
  inline void writeValue(const int ixyz[3], T value);
  inline void writeValue(const double xyz[3], T value);

  //add the value to the cell with optional value clamping
  inline void updateValue(const int ixyz[3], T value, const T clamp_bounds[2] = NULL);
  inline void updateValue(const double xyz[3], T value, const T clamp_bounds[2] = NULL);

  //step along the line segment from start to end, updating with miss_inc along the way, and update by hit_inc at end
  void raytrace(const int start[3], const int end[3], T miss_inc, T hit_inc, const T clamp_bounds[2] = NULL);
  void raytrace(const double start[3], const double end[3], T miss_inc, T hit_inc, const T clamp_bounds[2] = NULL);

  //check whether any of the cells between start and end are greater than occ_thresh
  bool collisionCheck(const int start[3], const int end[3], T occ_thresh, int collisionPoint[3] = NULL) const;
  bool collisionCheck(const double start[3], const double end[3], T occ_thresh, double collisionPoint[3] = NULL) const;

#ifndef NO_LCM
  //convert the voxelmap into an LCM message
  const occ_map_voxel_map_t * get_voxel_map_t(int64_t utime);
  void set_from_voxel_map_t(const occ_map_voxel_map_t * _msg);

  void saveToFile(const std::string & name);
  void loadFromFile(const std::string & name);
#endif

private:
  template<class F>
  inline F clamp_value(F x, F min, F max) const;
};

#ifndef NO_LCM
//static function to load pixel_map message directly from a file
static occ_map_voxel_map_t * load_voxel_map_t_from_file(const std::string & name);
#endif

typedef VoxelMap<float> FloatVoxelMap;
typedef VoxelMap<int32_t> IntVoxelMap;
typedef VoxelMap<uint8_t> Uint8VoxelMap;


//include the actual implimentations
#define __VOXELMAP_DIRECT_INCLUDE__
#include "VoxelMap.hxx"

}

#endif /*VOXELMAP_H_*/
