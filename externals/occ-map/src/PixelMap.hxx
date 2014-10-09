#ifndef __PIXELMAP_DIRECT_INCLUDE__
#error "PixelMap.hxx should not be included directly -- Include PixelMap.hpp instead!"
#endif

template<typename T>
PixelMap<T>::PixelMap(const double _xy0[2], const double _xy1[2], double mPP, T initValue, bool allocate_data,
    bool align_to_pixels) :
    metersPerPixel(mPP),  data(NULL), utime(0)
{
#ifndef NO_LCM
  msg=NULL;
#endif

  if (align_to_pixels) {
    // make bottom right align with pixels
    xy0[0] = floor((1.0 / metersPerPixel) * _xy0[0]) * metersPerPixel;
    xy0[1] = floor((1.0 / metersPerPixel) * _xy0[1]) * metersPerPixel;
  }
  else {
    xy0[0] = _xy0[0];
    xy0[1] = _xy0[1];
  }

  dimensions[0] = ceil(floor(100 * (1.0 / metersPerPixel) * (_xy1[0] - xy0[0])) / 100); //multiply by 100 and take floor to avoid machine
  dimensions[1] = ceil(floor(100 * (1.0 / metersPerPixel) * (_xy1[1] - xy0[1])) / 100); //precision causing different sized maps

  //make top right align with pixels
  xy1[0] = xy0[0] + dimensions[0] * metersPerPixel;
  xy1[1] = xy0[1] + dimensions[1] * metersPerPixel;

  if (dimensions[0] <= 0 || dimensions[1] < 0) {
    printf("ERROR:dimensions[0] or dimensions[1] is less than 0\n");
    return;
  }
  num_cells = dimensions[0] * dimensions[1];
  if (allocate_data) {
    data = new T[num_cells];
    reset(initValue);
  }
}

/*
 * Copy Constructor
 */
template<typename T>
template<class F>
PixelMap<T>::PixelMap(const PixelMap<F> * to_copy, bool copyData, T (*transformFunc)(F)) :
    metersPerPixel(to_copy->metersPerPixel), data(NULL), utime(0)
{
#ifndef NO_LCM
msg = NULL;
#endif

  memcpy(xy0, to_copy->xy0, 2 * sizeof(double));
  memcpy(xy1, to_copy->xy1, 2 * sizeof(double));

  memcpy(dimensions, to_copy->dimensions, 2 * sizeof(int));
  num_cells = to_copy->num_cells;
  data = new T[num_cells];
  if (copyData) {
    int ixy[2];
    for (ixy[1] = 0; ixy[1] < dimensions[1]; ixy[1]++) {
      for (ixy[0] = 0; ixy[0] < dimensions[0]; ixy[0]++) {
        if (transformFunc != NULL)
          writeValue(ixy, transformFunc(to_copy->readValue(ixy)));
        else
          writeValue(ixy, to_copy->readValue(ixy));
      }
    }
  }
}

#ifndef NO_LCM
/*
 * Constructor from a message
 */
template<typename T>
PixelMap<T>::PixelMap(const occ_map_pixel_map_t * _msg) :
    msg(NULL), data(NULL)
{
  set_from_pixel_map_t(_msg);
}

/*
 * Constructor from a file (created with "saveToFile")
 */
template<typename T>
PixelMap<T>::PixelMap(const std::string & name) :
    msg(NULL), data(NULL)
{
  loadFromFile(name);
}
#endif

template<typename T>
PixelMap<T>::~PixelMap()
{
  if (data != NULL)
    delete[] data;
#ifndef NO_LCM
  if (msg != NULL)
    occ_map_pixel_map_t_destroy (msg);
#endif
}

template<typename T>
void PixelMap<T>::reset(T resetVal)
{
  if (data != NULL)
    for (int i = 0; i < num_cells; i++)
      data[i] = resetVal;
}

template<typename T>
inline int PixelMap<T>::getInd(const int ixy[2]) const
    {
  return ixy[1] * dimensions[0] + ixy[0];
}

template<typename T>
inline int PixelMap<T>::getInd(const double xy[2]) const
    {
  int ixy[2];
  worldToTable(xy, ixy);
  return getInd(ixy);
}

template<typename T>
inline void PixelMap<T>::indToLoc(int ind, int ixy[2]) const
    {
  ixy[1] = ind / (dimensions[0]);
  ind -= ixy[1] * dimensions[0];
  ixy[0] = ind;
}
template<typename T>
inline void PixelMap<T>::indToLoc(int ind, double xy[2]) const
    {
  int ixy[2];
  indToLoc(ind, ixy);
  tableToWorld(ixy, xy);
}

template<typename T>
inline void PixelMap<T>::worldToTable(const double xy[2], int ixy[2]) const
    {
  ixy[0] = clamp_value(round((xy[0] - xy0[0]) / metersPerPixel), 0., (double) (dimensions[0] - 1));
  ixy[1] = clamp_value(round((xy[1] - xy0[1]) / metersPerPixel), 0., (double) (dimensions[1] - 1));
}

template<typename T>
inline void PixelMap<T>::tableToWorld(const int ixy[2], double xy[2]) const
    {
  //    *xy[0] = ((double)ixy[0]+0.5) * metersPerPixel + xy0[0]; //+.5 puts it in the center of the cell
  //    *xy[1] = ((double)ixy[1]+0.5) * metersPerPixel + xy0[1];
  xy[0] = ((double) ixy[0]) * metersPerPixel + xy0[0];
  xy[1] = ((double) ixy[1]) * metersPerPixel + xy0[1];

}
template<typename T>
inline bool PixelMap<T>::isInMap(const int ixy[2]) const
    {
  if (ixy[0] < 0 || ixy[1] < 0)
    return false;
  else if (ixy[0] >= dimensions[0] || ixy[1] >= dimensions[1])
    return false;
  else
    return true;
}

template<typename T>
inline bool PixelMap<T>::isInMap(const double xy[2]) const
    {
  if (xy[0] < xy0[0] || xy[0] > xy1[0])
    return false;
  else if (xy[1] < xy0[1] || xy[1] > xy1[1])
    return false;
  else
    return true;
}

template<typename T>
inline T & PixelMap<T>::readValue(const int ixy[2]) const
    {
  int ind = getInd(ixy);
  return data[ind];

}
template<typename T>
inline T & PixelMap<T>::readValue(const double xy[2]) const
    {
  int ixy[2];
  worldToTable(xy, ixy);
  return readValue(ixy);
}
template<typename T>
inline void PixelMap<T>::writeValue(const int ixy[2], T val)
{
  int ind = getInd(ixy);
  data[ind] = val;
}

template<typename T>
inline void PixelMap<T>::writeValue(const double xy[2], T val)
{
  int ixy[2];
  worldToTable(xy, ixy);
  writeValue(ixy, val);
}

template<typename T>
inline void PixelMap<T>::updateValue(const int ixy[2], T inc, T clamp_bounds[2])
{
  int ind = getInd(ixy);
  data[ind] += inc;
  if (clamp_bounds != NULL) {
    data[ind] = clamp_value(data[ind], clamp_bounds[0], clamp_bounds[1]);
  }
}

template<typename T>
inline void PixelMap<T>::updateValue(const double xy[2], T inc, T clamp_bounds[2])
{
  int ixy[2];
  worldToTable(xy, ixy);
  updateValue(ixy, inc, clamp_bounds);
}

template<typename T>
inline void PixelMap<T>::rayTrace(const double start[2], const double end[2], T miss_inc, T hit_inc, T clamp_bounds[2])
{
  int istart[2], iend[2];
  worldToTable(start, istart);
  worldToTable(end, iend);
  rayTrace(istart, iend, miss_inc, hit_inc, clamp_bounds);
}

/**
 * This function adapted from the Python Imaging Library
 */
template<typename T>
void PixelMap<T>::rayTrace(const int start[2], const int end[2], T miss_inc, T hit_inc, T clamp_bounds[2])
{
  int curr[2] = { start[0], start[1] };

  // normalize
  int xstep = 1;
  int ystep = 1;
  int dx = end[0] - start[0];
  if (dx < 0) {
    dx = -dx;
    xstep = -1;
  }
  int dy = end[1] - start[1];
  if (dy < 0) {
    dy = -dy;
    ystep = -1;
  }

  if (dx == 0) {
    // vertical
    for (int i = 0; i <= dy; i++) {
      int wasHit = curr[0] == end[0] && curr[1] == end[1];
      updateValue(curr, wasHit * hit_inc + (1 - wasHit) * miss_inc, clamp_bounds);
      curr[1] = curr[1] + ystep;
    }
  }
  else if (dy == 0) {
    // horizontal
    for (int i = 0; i <= dx; i++) {
      int wasHit = curr[0] == end[0] && curr[1] == end[1];
      updateValue(curr, wasHit * hit_inc + (1 - wasHit) * miss_inc, clamp_bounds);
      curr[0] += xstep;
    }
  }
  else if (dx > dy) {
    // bresenham, horizontal slope
    int n = dx;
    dy += dy;
    int e = dy - dx;
    dx += dx;

    for (int i = 0; i <= n; i++) {
      int wasHit = curr[0] == end[0] && curr[1] == end[1];
      updateValue(curr, wasHit * hit_inc + (1 - wasHit) * miss_inc, clamp_bounds);
      if (e >= 0) {
        curr[1] += ystep;
        e -= dx;
      }
      e += dy;
      curr[0] += xstep;
    }
  }
  else {
    // bresenham, vertical slope
    int n = dy;
    dx += dx;
    int e = dx - dy;
    dy += dy;

    for (int i = 0; i <= n; i++) {
      int wasHit = curr[0] == end[0] && curr[1] == end[1];
      updateValue(curr, wasHit * hit_inc + (1 - wasHit) * miss_inc, clamp_bounds);
      if (e >= 0) {
        curr[0] += xstep;
        e -= dy;
      }
      e += dx;
      curr[1] += ystep;
    }
  }
}

template<typename T>
inline bool PixelMap<T>::collisionCheck(const double start[2], const double end[2], T occ_thresh,
    double collisionPoint[2]) const
    {
  int istart[2], iend[2], icollision[2];
  worldToTable(start, istart);
  worldToTable(end, iend);
  bool collision = collisionCheck(istart, iend, occ_thresh, icollision);
  if (collision && collisionPoint != NULL)
    tableToWorld(icollision, collisionPoint);
  return collision;
}

template<typename T>
bool PixelMap<T>::collisionCheck(const int start[2], const int end[2], T occ_thresh, int collisionPoint[2]) const
    {
  int curr[2] = { start[0], start[1] };
  bool collision = false;

  // normalize
  int xstep = 1;
  int ystep = 1;
  int dx = end[0] - start[0];
  if (dx < 0) {
    dx = -dx;
    xstep = -1;
  }
  int dy = end[1] - start[1];
  if (dy < 0) {
    dy = -dy;
    ystep = -1;
  }

  if (dx == 0) {
    // vertical
    for (int i = 0; i <= dy; i++) {
      if (readValue(curr) > occ_thresh) {
        collision = true;
        break;
      }
      curr[1] = curr[1] + ystep;
    }
  }
  else if (dy == 0) {
    // horizontal
    for (int i = 0; i <= dx; i++) {
      if (readValue(curr) > occ_thresh) {
        collision = true;
        break;
      }
      curr[0] += xstep;
    }
  }
  else if (dx > dy) {
    // bresenham, horizontal slope
    int n = dx;
    dy += dy;
    int e = dy - dx;
    dx += dx;

    for (int i = 0; i <= n; i++) {
      if (readValue(curr) > occ_thresh) {
        collision = true;
        break;
      }
      if (e >= 0) {
        curr[1] += ystep;
        e -= dx;
      }
      e += dy;
      curr[0] += xstep;
    }
  }
  else {
    // bresenham, vertical slope
    int n = dy;
    dx += dx;
    int e = dx - dy;
    dy += dy;

    for (int i = 0; i <= n; i++) {
      if (readValue(curr) > occ_thresh) {
        collision = true;
        break;
      }
      if (e >= 0) {
        curr[0] += xstep;
        e -= dy;
      }
      e += dx;
      curr[1] += ystep;
    }
  }

  if (collisionPoint != NULL) {
    collisionPoint[0] = curr[0];
    collisionPoint[1] = curr[1];
  }
  return collision;
}


#ifndef NO_LCM
template<typename T>
const occ_map_pixel_map_t * PixelMap<T>::get_pixel_map_t(int64_t utime)
{
  if (msg == NULL)
    msg = (occ_map_pixel_map_t*) calloc(1, sizeof(occ_map_pixel_map_t));
  memcpy(msg->xy0, xy0, 2 * sizeof(double));
  memcpy(msg->xy1, xy1, 2 * sizeof(double));
  msg->mpp = metersPerPixel;
  memcpy(msg->dimensions, dimensions, 2 * sizeof(int));

  uLong uncompressed_size = num_cells * sizeof(T);

  uLong compress_buf_size = uncompressed_size * 1.01 + 12; //with extra space for zlib
  msg->mapData = (uint8_t *) realloc(msg->mapData, compress_buf_size);
  int compress_return = compress2((Bytef *) msg->mapData, &compress_buf_size, (Bytef *) data, uncompressed_size,
      Z_BEST_SPEED);
  if (compress_return != Z_OK) {
    fprintf(stderr, "ERROR: Could not compress voxel map!\n");
    return NULL;
  }
  //    fprintf(stderr, "uncompressed_size=%ld compressed_size=%ld\n", uncompressed_size, compress_buf_size);
  msg->datasize = compress_buf_size;
  msg->compressed = 1;

  //set the data_type
  const std::type_info& type = typeid(T);
  if (type == typeid(float))
    msg->data_type = OCC_MAP_PIXEL_MAP_T_TYPE_FLOAT;
  else if (type == typeid(uint8_t))
    msg->data_type = OCC_MAP_PIXEL_MAP_T_TYPE_UINT8;
  else if (type == typeid(double))
    msg->data_type = OCC_MAP_PIXEL_MAP_T_TYPE_DOUBLE;
  else if (type == typeid(int32_t))
    msg->data_type = OCC_MAP_PIXEL_MAP_T_TYPE_INT32;
  else if (type == typeid(uint32_t))
    msg->data_type = OCC_MAP_PIXEL_MAP_T_TYPE_UINT32;
  else if (type == typeid(int16_t))
    msg->data_type = OCC_MAP_PIXEL_MAP_T_TYPE_INT16;
  else if (type == typeid(uint16_t))
    msg->data_type = OCC_MAP_PIXEL_MAP_T_TYPE_UINT16;
  else if (type == typeid(int8_t))
    msg->data_type = OCC_MAP_PIXEL_MAP_T_TYPE_INT8;
  else
    msg->data_type = OCC_MAP_PIXEL_MAP_T_TYPE_UNKNOWN;

  msg->utime = utime;
  return msg;

}
template<typename T>
void PixelMap<T>::set_from_pixel_map_t(const occ_map_pixel_map_t * _msg)
{
  memcpy(xy0, _msg->xy0, 2 * sizeof(double));
  memcpy(xy1, _msg->xy1, 2 * sizeof(double));
  metersPerPixel = _msg->mpp;
  memcpy(dimensions, _msg->dimensions, 2 * sizeof(int));
  utime = _msg->utime;

  num_cells = dimensions[0] * dimensions[1];
  int type_size = 0;
  if (_msg->data_type > 0) {
    const std::type_info& type = typeid(T);
    if (_msg->data_type == OCC_MAP_PIXEL_MAP_T_TYPE_FLOAT && type != typeid(float)) {
      fprintf(stderr, "message has %d, not float data, not setting pixmap!\n", _msg->data_type);
      return;
    }
    else if (_msg->data_type == OCC_MAP_PIXEL_MAP_T_TYPE_UINT8 && type != typeid(uint8_t)) {
      fprintf(stderr, "message has %d, not uint8 data, not setting pixmap!\n", _msg->data_type);
      return;
    }
    else if (_msg->data_type == OCC_MAP_PIXEL_MAP_T_TYPE_DOUBLE && type != typeid(double)) {
      fprintf(stderr, "message has %d, not double data, not setting pixmap!\n", _msg->data_type);
      return;
    }
    else if (_msg->data_type == OCC_MAP_PIXEL_MAP_T_TYPE_INT32 && type != typeid(int32_t)) {
      fprintf(stderr, "message has %d, not int32 data, not setting pixmap!\n", _msg->data_type);
      return;
    }
    else if (_msg->data_type == OCC_MAP_PIXEL_MAP_T_TYPE_UINT32 && type != typeid(uint32_t)) {
      fprintf(stderr, "message has %d, not uint32 data, not setting pixmap!\n", _msg->data_type);
      return;
    }
    else if (_msg->data_type == OCC_MAP_PIXEL_MAP_T_TYPE_INT16 && type != typeid(int16_t)) {
      fprintf(stderr, "message has %d, not int16 data, not setting pixmap!\n", _msg->data_type);
      return;
    }
    else if (_msg->data_type == OCC_MAP_PIXEL_MAP_T_TYPE_UINT16 && type != typeid(uint16_t)) {
      fprintf(stderr, "message has %d, not uint16 data, not setting pixmap!\n", _msg->data_type);
      return;
    }
    else if (_msg->data_type == OCC_MAP_PIXEL_MAP_T_TYPE_INT8 && type != typeid(int8_t)) {
      fprintf(stderr, "message has %d, not int8 data, not setting pixmap!\n", _msg->data_type);
      return;
    }
    else if (_msg->data_type < 0 || _msg->data_type > OCC_MAP_PIXEL_MAP_T_TYPE_UINT8) {
      fprintf(stderr, "data type of %d is unknown, not setting pixmap!\n", _msg->data_type);
      return;
    }
  }
  else {
    fprintf(stderr, "data type is 0, lets hope it's correct!\n");
  }

  uLong uncompressed_size = num_cells * sizeof(T);
  data = (T*) realloc(data, uncompressed_size); //TODO: does this cause problems with the delete[] in the destructor??
  if (_msg->compressed) {
    uLong uncompress_size_result = uncompressed_size;
    uLong uncompress_return = uncompress((Bytef *) data, (uLong *) &uncompress_size_result, (Bytef *) _msg->mapData,
        (uLong) _msg->datasize);
    if (uncompress_return != Z_OK || uncompress_size_result != uncompressed_size) {
      fprintf(stderr, "ERROR uncompressing the map, ret = %lu\n", uncompress_return);
      return;
    }
  }
  else {
    assert((uLong) _msg->datasize == uncompressed_size);
    memcpy(data, _msg->mapData, uncompressed_size);
  }
}

template<typename T>
void PixelMap<T>::saveToFile(const std::string & name)
{
  const occ_map_pixel_map_t * msg = get_pixel_map_t(utime);
  int sz = occ_map_pixel_map_t_encoded_size(msg);
  char * buf = (char *) malloc(sz * sizeof(char));
  occ_map_pixel_map_t_encode(buf, 0, sz, msg);
  std::ofstream ofs(name.c_str(), std::ios::binary);
  ofs << sz;
  ofs.write(buf, sz);
  ofs.close();
  free(buf);
}

template<typename T>
void PixelMap<T>::loadFromFile(const std::string & name)
{
  occ_map_pixel_map_t * tmpmsg = load_pixel_map_t_from_file(name);
  set_from_pixel_map_t(tmpmsg);
  occ_map_pixel_map_t_destroy(tmpmsg);
}

occ_map_pixel_map_t * load_pixel_map_t_from_file(const std::string & name)
{
  std::ifstream ifs(name.c_str(), std::ios::binary);
  int sz;
  ifs >> sz;
  char * tmpdata = (char *) malloc(sz * sizeof(char));
  ifs.read(tmpdata, sz * sizeof(char));
  ifs.close();
  occ_map_pixel_map_t * ret_msg = (occ_map_pixel_map_t *) calloc(1, sizeof(occ_map_pixel_map_t));
  if (occ_map_pixel_map_t_decode(tmpdata, 0, sz, ret_msg) < 0) {
    fprintf(stderr,"ERROR decoding pixelmap from %s\n",name.c_str());
  }
  free(tmpdata);
  return ret_msg;
}

#endif




template<typename T>
template<class F>
inline F PixelMap<T>::clamp_value(F x, F min, F max) const
    {
  if (x < min)
    return min;
  if (x > max)
    return max;
  return x;
}
