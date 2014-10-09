#include "octomap_util.hpp"
#include <bot_core/bot_core.h>
#include <sstream>
using namespace std;
using namespace octomap;
using namespace occ_map;
using namespace octomap_utils;

int main(int argc, char ** argv)
{

  double blur_sigma = .5;
  if (argc <2 || argc>3) {
    printf("Usage:\n");
    printf("%s <octomap_fname> <blur_radius>\n", argv[0]);
    exit(1);
  }

  if (argc>=3)
    blur_sigma = atof(argv[2]);

  string octomap_fname = argv[1];


  std::stringstream s;
  s <<octomap_fname << "_blurred_" << blur_sigma;
  string blurred_fname = s.str();

  printf("loading octomap from: %s\n", octomap_fname.c_str());
  octomap::OcTree * ocTree = new  octomap::OcTree(octomap_fname);

  ocTree->toMaxLikelihood();
  ocTree->expand();

  double minX, minY, minZ, maxX, maxY, maxZ;
  ocTree->getMetricMin(minX, minY, minZ);
  ocTree->getMetricMax(maxX, maxY, maxZ);
  printf("\nmap bounds: [%.2f, %.2f, %.2f] - [%.2f, %.2f, %.2f]  res: %f\n", minX, minY, minZ, maxX, maxY, maxZ,
      ocTree->getResolution());

  printf("blurring octomap\n");
  double minNegLogLike;
  octomap::OcTree * ocTree_blurred = octomapBlur(ocTree, blur_sigma, &minNegLogLike);
  printf("Saving blurred map to: %s\n", blurred_fname.c_str());
  saveOctomap(ocTree_blurred, blurred_fname.c_str(), minNegLogLike);

   return 0;
}
