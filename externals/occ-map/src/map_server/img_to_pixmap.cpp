#include <lcm/lcm.h>
#include <occ_map/PixelMap.hpp>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace occ_map;

static float uint8ToFloat(uint8_t v)
{
  return (float) 1.0 - v / 255.0;
}

void usage(char * name)
{
  fprintf(stderr, "usage: %s img_fname pixmap_fname [orig_pixmap_fname]\n", name);
  exit(1);
}

int main(int argc, char ** argv)
{
  if (argc < 3)
    usage(argv[0]);

  string img_fname = argv[1];
  string filename = argv[2];

  string origMapFname;
  if (argc > 3)
    origMapFname = argv[3];

  //todo: get resolution and bounds from command line
  float resolution = 1;
  double xy0[2] = { 0, 0 };
  double xy1[2] = { 0, 0 };

  CvMat * cvm = cvLoadImageM(img_fname.c_str());

  float scaleFactor = 800.0 / (float) cvm->width;
  CvMat * dispM = cvCreateMat(cvm->height * scaleFactor, cvm->width * scaleFactor, cvm->type);
  cvResize(cvm, dispM);
  cvNamedWindow("img2map");
  cvShowImage("img2map", dispM);
  cvWaitKey(0);
  xy1[0] = xy0[0] + resolution * cvm->cols;
  xy1[1] = xy0[1] + resolution * cvm->rows;

  if (!origMapFname.empty()) {
    FloatPixelMap * orig_map = new FloatPixelMap(origMapFname);
    for (int i = 0; i < 2; i++) {
      xy0[i] = orig_map->xy0[i];
      xy1[i] = orig_map->xy1[i];
    }
    resolution = orig_map->metersPerPixel;
    int num_cells = cvm->cols * cvm->rows;
    if (num_cells != orig_map->num_cells) {
      fprintf(stderr, "ERROR: image has %d cells, but orig_map has %d\n", num_cells, orig_map->num_cells);
    }
  }

  Uint8PixelMap *u8map = new Uint8PixelMap(xy0, xy1, resolution, 0, true, false);
  int ixy[2];
  for (ixy[0] = 0; ixy[0] < u8map->dimensions[0]; ixy[0]++) {
    for (ixy[1] = 0; ixy[1] < u8map->dimensions[1]; ixy[1]++) {
      uint8_t val = *cvPtr2D(cvm, ixy[1], ixy[0], NULL);
      u8map->writeValue(ixy, val);
    }
  }
  FloatPixelMap * fmap = new FloatPixelMap(u8map, true, uint8ToFloat);
  fmap->saveToFile(filename.c_str());

}

