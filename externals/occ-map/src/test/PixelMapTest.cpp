#include <occ_map/PixelMap.hpp>

int main(int argc, char ** argv){
  double xyz0[2] = {0,0};
  double xyz1[2] = {10,10};
  double mpp = .2;
  occ_map::FloatPixelMap fvm(xyz0,xyz1,mpp,0);
  double xy[2] ={3,4};
  int ixy[2];
  fvm.worldToTable(xy,ixy);
  printf("ixy=%d,%d\n",ixy[0],ixy[1]);
}
