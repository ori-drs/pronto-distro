#include <occ_map/VoxelMap.hpp>

int main(int argc, char ** argv)
{
  double xyz0[3] = { -20, -20, 0 };
  double xyz1[3] = { 20, 20, 5 };
  double mpp[3] = { .2, .2, .2 };
  occ_map::FloatVoxelMap fvm(xyz0, xyz1, mpp, 0);
  double ixyz[3];
  for (ixyz[2] = -5; ixyz[2] < 10; ixyz[2]+=.2) {
    for (ixyz[1] = -5; ixyz[1] < 10; ixyz[1]+=.2) {
      for (ixyz[0] = .5; ixyz[0] < 1; ixyz[0]+=.2) {
        fvm.writeValue(ixyz,0.99);
      }
    }
  }

  double xyzO[3] = { 0, 0, 0 };
  double xyzR[3] = { 0, 5, 2 };
  for (double x = -5; x < 5; x += .5) {
    xyzR[0] = x;
    fvm.raytrace(xyzO, xyzR, 1, .3);
    }

#ifndef NO_LCM
  const occ_map_voxel_map_t * msg = fvm.get_voxel_map_t(0);
  lcm_t * lcm = lcm_create(NULL);
  occ_map_voxel_map_t_publish(lcm, "VOXEL_MAP",msg);
#endif
}
