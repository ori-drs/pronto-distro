#include <lcm/lcm.h>
#include <occ_map/PixelMap.hpp>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace occ_map;

static uint8_t floatToUint8(float v)
{
  return v * 255.0;
}

void usage(char * name)
{
  fprintf(stderr, "usage: %s map_fname img_fname \n", name);
  exit(1);
}

int main(int argc, char ** argv)
{
  if (argc != 3)
    usage(argv[0]);

  string filename = argv[1];
  string img_fname = argv[2];
  occ_map_pixel_map_t * pix_map_msg = load_pixel_map_t_from_file(filename.c_str());

  occ_map::Uint8PixelMap * map;
  if (pix_map_msg->data_type == OCC_MAP_PIXEL_MAP_T_TYPE_FLOAT) {
    FloatPixelMap * fmap = new FloatPixelMap(pix_map_msg);
    map = new occ_map::Uint8PixelMap(fmap, true, floatToUint8);
    delete fmap;
  }

  else if (pix_map_msg->data_type == OCC_MAP_PIXEL_MAP_T_TYPE_UINT8) {
    map = new occ_map::Uint8PixelMap(pix_map_msg);
  }

  CvMat cvm = cvMat(map->dimensions[1], map->dimensions[0], CV_8UC1, map->data);

  string out_fname = img_fname; //TODO: append resolution, and bounds?
  cvSaveImage(out_fname.c_str(), &cvm);

  FILE *f = fopen((out_fname+".m").c_str(),"w");
  fprintf(f,"xy0 = [%f %f]\n", map->xy0[0],map->xy0[1]);
  fprintf(f,"xy1 = [%f %f]\n", map->xy1[0],map->xy1[1]);
  fprintf(f,"metersPerPixel = %f\n", map->metersPerPixel);
  fclose(f);

}

