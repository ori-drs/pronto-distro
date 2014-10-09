#include <stdlib.h>
#include <list>
#include "octomap_util.hpp"
#include <laser_utils/laser_util.h>
#include <lcmtypes/octomap_file_t.h>
#include <iostream>
#include <fstream>
#include <bot_core/bot_core.h>
using namespace std;
using namespace occ_map;
using namespace octomap;

namespace octomap_utils {

occ_map::FloatVoxelMap * octomapToVoxelMap(octomap::OcTree * ocTree, int occupied_depth, int free_depth)
{
  ocTree->toMaxLikelihood(); //lets not care about likelihoods
  double xyz0[3];
  double xyz1[3];
  ocTree->getMetricMin(xyz0[0], xyz0[1], xyz0[2]);
  ocTree->getMetricMax(xyz1[0], xyz1[1], xyz1[2]);

  double resolution = ocTree->getResolution();
  double mpp[3] = { resolution, resolution, resolution };
  FloatVoxelMap *voxMap = new FloatVoxelMap(xyz0, xyz1, mpp, .5);

  std::list<octomap::OcTreeVolume> occupiedVoxels;
  ocTree->getOccupied(occupiedVoxels, occupied_depth);
  list<octomap::OcTreeVolume>::iterator it;
  //mark the free ones
  double xyz[3];
  for (it = occupiedVoxels.begin(); it != occupiedVoxels.end(); it++) {
    double cxyz[3] = { it->first.x(), it->first.y(), it->first.z() };
    double side_length = it->second;
    int side_cells = side_length / resolution;
    if (side_cells > 1)
      int foo = side_length + 2;
    for (int i = 0; i < side_cells; i++) {
      xyz[0] = cxyz[0] - side_length / 2 + i * resolution;
      for (int j = 0; j < side_cells; j++) {
        xyz[1] = cxyz[1] - side_length / 2 + j * resolution;
        for (int k = 0; k < side_cells; k++) {
          xyz[2] = cxyz[2] - side_length / 2 + k * resolution;
          voxMap->writeValue(xyz, 1);
        }
      }
    }
  }
  occupiedVoxels.clear();

  std::list<octomap::OcTreeVolume> freeVoxels;
  ocTree->getFreespace(freeVoxels, free_depth);
  for (it = freeVoxels.begin(); it != freeVoxels.end(); it++) {
    double cxyz[3] = { it->first.x(), it->first.y(), it->first.z() };
    double side_length = it->second;
    int side_cells = side_length / resolution;
    if (side_cells > 1)
      int foo = side_length + 2;
    for (int i = 0; i < side_cells; i++) {
      xyz[0] = cxyz[0] - side_length / 2 + i * resolution;
      for (int j = 0; j < side_cells; j++) {
        xyz[1] = cxyz[1] - side_length / 2 + j * resolution;
        for (int k = 0; k < side_cells; k++) {
          xyz[2] = cxyz[2] - side_length / 2 + k * resolution;
          voxMap->writeValue(xyz, 0);
        }
      }
    }
  }

  memset(xyz, 0, 3 * sizeof(double));
  int ixyz[3];
  voxMap->worldToTable(xyz, ixyz);
  for (ixyz[2] = 0; ixyz[2] < voxMap->dimensions[2]; ixyz[2]++) {
    float v = voxMap->readValue(ixyz);
    voxMap->tableToWorld(ixyz, xyz);
  }
  xyz[0] = -100;
  xyz[1] = 100;
  xyz[2] = 0;
  voxMap->worldToTable(xyz, ixyz);
  for (ixyz[2] = 0; ixyz[2] < voxMap->dimensions[2]; ixyz[2]++) {
    float v = voxMap->readValue(ixyz);
    voxMap->tableToWorld(ixyz, xyz);
  }
  return voxMap;
}


//TODO: add verbose flag/disable printing?
octomap::OcTree * octomapBlur(octomap::OcTree * ocTree, double blurSigma, double *minNegLogLike)
{

  float res = ocTree->getResolution();

  //normalize sigma from meteres to cell resolution
  double blurVar = blurSigma * blurSigma;
  //compute the size of the gaussian kernel assuming max likelihood of 255, and min of 32
  double det_var = pow(blurVar, 3); //covariance is diagonal
  double normalizer = pow(2 * M_PI, -3 / 2) * pow(det_var, -1 / 2);
//  int sz = 0;
//  float val = 1;
//  while (val > .1) {
//    sz++;
//    double d = sz * res;
//    val = normalizer * exp(-.5 * (d * d + d * d + d * d) / blurVar);
//  }
//  int kernel_size = 2 * sz;

  int kernel_size = 2 * blurSigma / res;

  printf("kernel size is %d\n", kernel_size);

  //create the gaussian kernel
  double kxzy0[3] = { -res * kernel_size, -res * kernel_size, -res * kernel_size };
  double kxzy1[3] = { res * kernel_size, res * kernel_size, res * kernel_size };
  double krez[3] = { res, res, res };
  FloatVoxelMap * blurKernel = new FloatVoxelMap(kxzy0, kxzy1, krez);
  VoxelMap<point3d> * indexTable = new VoxelMap<point3d>(kxzy0, kxzy1, krez);
  double xyz[3];
  int ixyz[3];
  double kernel_sum = 0;
  for (ixyz[2] = 0; ixyz[2] < blurKernel->dimensions[2]; ixyz[2]++) {
    for (ixyz[1] = 0; ixyz[1] < blurKernel->dimensions[1]; ixyz[1]++) {
      for (ixyz[0] = 0; ixyz[0] < blurKernel->dimensions[0]; ixyz[0]++) {
        blurKernel->tableToWorld(ixyz, xyz);
        double val = normalizer * exp(-.5 * (bot_sq(xyz[0]) + bot_sq(xyz[1]) + bot_sq(xyz[2])) / blurVar); //diagonal cov
        kernel_sum += val;
        blurKernel->writeValue(ixyz, val);
        indexTable->writeValue(ixyz, point3d(xyz[0], xyz[1], xyz[2]));
      }
    }
  }

  printf("kernel_sum = %f\n", kernel_sum);

  for (int i = 0; i < blurKernel->num_cells; i++) {
    blurKernel->data[i] /= kernel_sum;
  }

  double cross_section_sum = 0;
  xyz[0] = xyz[1] = xyz[2] = 0;
  blurKernel->worldToTable(xyz, ixyz);
  fprintf(stderr, "kernel = [\n");
  for (ixyz[1] = 0; ixyz[1] < blurKernel->dimensions[1]; ixyz[1]++) {
    for (ixyz[0] = 0; ixyz[0] < blurKernel->dimensions[0]; ixyz[0]++) {
      fprintf(stderr, "%f ", blurKernel->readValue(ixyz));
      cross_section_sum += blurKernel->readValue(ixyz);
    }
    fprintf(stderr, "\n");
  }
  fprintf(stderr, "];\n");

  printf("cross_section_sum = %f\n", cross_section_sum);

  printf("Creating Blurred Map\n");
  octomap::OcTree *ocTree_blurred = new OcTree(res);
  //set blurred map to occupancy probablity
  int numLeaves = ocTree->getNumLeafNodes();
  int count = 0;
  for (octomap::OcTree::leaf_iterator it = ocTree->begin_leafs(),
      end = ocTree->end_leafs(); it != end; ++it)
  {
    if (count % (numLeaves / 20) == 0) {
      printf("%d of %d\n", count, numLeaves);
    }
    count++;

    point3d center = it.getCoordinate();
    ocTree->search(it.getKey());
    if (!ocTree->isNodeOccupied(ocTree->search(it.getKey()))) {
//      fprintf(stderr, "skipping unoccupied node at %f, %f, %f\n", it.getCoordinate().x(),
//          it.getCoordinate().y(), it.getCoordinate().z());
      continue;
    }
    for (int i = 0; i < blurKernel->num_cells; i++) {
      OcTreeKey key;
      if (!ocTree_blurred->genKey(center + indexTable->data[i], key)) {
        fprintf(stderr, "Error: couldn't generate key in blurred map!\n");
      }
      ocTree_blurred->updateNode(key, blurKernel->data[i], true);
    }
  }

  printf("Updating inner occupancy!\n");
  ocTree_blurred->updateInnerOccupancy();

  //convert from probabilities to log odds, capping at the likelihood of a wall
  int numBlurLeaves = ocTree_blurred->getNumLeafNodes();
  printf("Converting to Log Odds\n");
  count = 0;
  for (octomap::OcTree::leaf_iterator it = ocTree_blurred->begin_leafs(),
      end = ocTree_blurred->end_leafs(); it != end; ++it)
  {
    octomap::OcTreeNode &node = *it;
    node.setValue(-log(fmin(cross_section_sum, node.getValue())));
  }
  *minNegLogLike = -log(cross_section_sum);
  return ocTree_blurred;
}

void saveOctomap(octomap::OcTree *ocTree, const char * fname, double minNegLogLike)
{

  octomap_file_t save_msg;

  ocTree->expand(); //make sure tree is full size
  save_msg.num_nodes = ocTree->getNumLeafNodes();
  save_msg.nodes = new octomap_node_t[save_msg.num_nodes];
  save_msg.resolution = ocTree->getResolution();
  save_msg.minNegLogLike = minNegLogLike;
  int count = 0;
  for (octomap::OcTree::leaf_iterator it = ocTree->begin_leafs(),
      end = ocTree->end_leafs(); it != end; ++it)
  {
    if (count > save_msg.num_nodes) {
      fprintf(stderr, "ERROR: iterator gave us too many nodes\n");
      break;
    }
    octomap::OcTreeNode &node = *it;
    save_msg.nodes[count].xyz[0] = it.getX();
    save_msg.nodes[count].xyz[1] = it.getY();
    save_msg.nodes[count].xyz[2] = it.getZ();
    save_msg.nodes[count].negLogLike = node.getValue();
    count++;
  }
  if (count != save_msg.num_nodes) {
    fprintf(stderr, "ERROR: iterator didn't give us enough nodes\n");
  }

  int sz = octomap_file_t_encoded_size(&save_msg);
  char * buf = (char *) malloc(sz * sizeof(char));
  octomap_file_t_encode(buf, 0, sz, &save_msg);
  std::ofstream ofs(fname, std::ios::binary);
  ofs << sz;
  ofs.write(buf, sz);
  ofs.close();
  free(buf);
  delete[] save_msg.nodes;

}

octomap::OcTree * loadOctomap(const char * fname, double * minNegLogLike)
{

  std::ifstream ifs(fname, std::ios::binary);

  if (!ifs.good()) {
    fprintf(stderr, "error: couldn't load octomap %s, dying!\n", fname);
    exit(1);
  }

  int sz;
  ifs >> sz;
  char * tmpdata = (char *) malloc(sz * sizeof(char));
  ifs.read(tmpdata, sz * sizeof(char));
  ifs.close();
  octomap_file_t * saved_msg = (octomap_file_t *) calloc(1, sizeof(octomap_file_t));
  octomap_file_t_decode(tmpdata, 0, sz, saved_msg);
  free(tmpdata);

  *minNegLogLike = saved_msg->minNegLogLike;
  octomap::OcTree *ocTree = new octomap::OcTree(saved_msg->resolution);
  ocTree->setClampingThresMax(1000);
  ocTree->setClampingThresMin(-1000);
  for (int i = 0; i < saved_msg->num_nodes; i++) {
    OcTreeKey key;
    point3d location(saved_msg->nodes[i].xyz[0], saved_msg->nodes[i].xyz[1], saved_msg->nodes[i].xyz[2]);
    if (!ocTree->genKey(location, key)) {
      fprintf(stderr, "Error: couldn't generate key in map for (%f,%f,%f)!\n", location.x(), location.y(),
          location.z());
      break;
    }
    ocTree->updateNode(key, saved_msg->nodes[i].negLogLike, true);
  }
  ocTree->updateInnerOccupancy();

  octomap_file_t_destroy(saved_msg);
  return ocTree;

}

octomap::OcTree * createZPlane(double resolution, double xy0[2], double xy1[2], double z_plane_height)
{
  octomap::OcTree * ocTree = new octomap::OcTree(resolution);

  double xy[0];
  for (xy[0] = xy0[0]; xy[0] < xy1[1]; xy[0] += resolution) {
    for (xy[1] = xy0[1]; xy[1] < xy1[1]; xy[1] += resolution) {
      ocTree->updateNode(point3d(xy[0], xy[1], z_plane_height), true, false);
    }
  }

  ocTree->toMaxLikelihood();
  ocTree->expand();
  return ocTree;
}

}
