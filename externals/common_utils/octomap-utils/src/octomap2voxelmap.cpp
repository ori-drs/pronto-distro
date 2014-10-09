#include "octomap_util.hpp"

using namespace std;
using namespace octomap;
using namespace occ_map;
using namespace octomap_utils;

int main(int argc, char ** argv)
{

  if (argc != 5) {
    printf("Usage:\n");
    printf("%s <octomap_fname> <voxmap_fname> <occupied_depth> <free_depth>\n", argv[0]);
    exit(1);
  }
  else if (strcmp(argv[1], argv[2]) == 0) {
    printf("input and output files must differ\n");
    exit(1);
  }
  const char * octomap_fname = argv[1];
  const char * voxmap_fname = argv[2];
  int occupied_depth = atoi(argv[3]);
  int free_depth = atoi(argv[4]);

  printf("loading octomap from: %s\n", octomap_fname);
  octomap::OcTree * ocTree = new OcTree(octomap_fname);
  printf("Converting to Voxmap\n");
  occ_map::FloatVoxelMap * voxmap = octomapToVoxelMap(ocTree, occupied_depth, free_depth);
  printf("Saving voxmap to: %s\n", voxmap_fname);
  voxmap->saveToFile(voxmap_fname);

 
  return 0;
}
