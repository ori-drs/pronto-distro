/*
 * renders a Octomap
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <gtk/gtk.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include "renderer_octomap.h"

#include <lcmtypes/octomap_raw_t.h>

#include <octomap/octomap.h>
#include <sstream>

#define RENDERER_NAME "Octomap"
#define PARAM_COLOR_MODE "Color Mode"
#define PARAM_COLOR_MODE_Z_MAX_Z "Red Height"
#define PARAM_COLOR_MODE_Z_MIN_Z "Blue Height"
#define PARAM_SHOW_FREE "Show Free"
#define PARAM_SHOW_OCC  "Show Occ"
//#define PARAM_SHOW_GRID "Show Grid"
#define PARAM_TREE_DEPTH "Tree Depth"
#define PARAM_POINT_SIZE "Point Size"
#define PARAM_Z_MAX "Z max"
#define PARAM_Z_MIN "Z min"

namespace octomap {

class MyOcTreeDrawer {
public:
  MyOcTreeDrawer();
  virtual ~MyOcTreeDrawer();
  void clear();
  void draw() const;
  /// sets a new OcTree that should be drawn by this drawer
  void setOcTree(const octomap::OcTree &octree, double minDrawZ, double maxDrawZ);

  /// sets a new selection of the current OcTree to be drawn
  void setOcTreeSelection(const std::list<octomap::OcTreeVolume>& selectedPoints);

  /// clear the visualization of the OcTree selection
  void clearOcTreeSelection();

  /// sets alpha level for occupied cells
  void setAlphaOccupied(double alpha);

  void enableOcTree(bool enabled = true);
  void enableOcTreeCells(bool enabled = true)
  {
    m_drawOccupied = enabled;
  }
  ;
  void enableFreespace(bool enabled = true)
  {
    m_drawFree = enabled;
  }
  ;
  void enableSelection(bool enabled = true)
  {
    m_drawSelection = enabled;
  }
  ;
  void setMax_tree_depth(unsigned int max_tree_depth)
  {
    m_max_tree_depth = max_tree_depth;
  }
  ;

public:
  //void clearOcTree();
  void clearOcTreeStructure();

  void drawOctreeGrid() const;
  void drawOccupiedVoxels() const;
  void drawFreeVoxels() const;
  void drawSelection() const;
  void drawCubes(GLfloat** cubeArray, unsigned int cubeArraySize, GLfloat* cubeColorArray = NULL) const;

  //! Initializes the OpenGL visualization for a list of OcTreeVolumes
  //! The array is cleared first, if needed
  void generateCubes(const std::list<octomap::OcTreeVolume>& points, GLfloat*** glArray, unsigned int& glArraySize,
      GLfloat** glColorArray = NULL);

  //! clear OpenGL visualization
  void clearCubes(GLfloat*** glArray, unsigned int& glArraySize, GLfloat** glColorArray = NULL);

  void initOctreeGridVis();

  //! OpenGL representation of Octree cells (cubes)

  GLfloat** m_occupiedThresArray;
  unsigned int m_occupiedThresSize;
  GLfloat** m_freeThresArray;
  unsigned int m_freeThresSize;
  GLfloat** m_occupiedArray;
  unsigned int m_occupiedSize;
  GLfloat** m_freeArray;
  unsigned int m_freeSize;
  GLfloat** m_selectionArray;
  unsigned int m_selectionSize;

  //! Color array for occupied cells (height)
  GLfloat* m_occupiedThresColorArray;
  GLfloat* m_occupiedColorArray;

  //! OpenGL representation of Octree (grid structure)
  // TODO: put in its own drawer object!
  GLfloat* octree_grid_vertex_array;
  unsigned int octree_grid_vertex_size;

  std::list<octomap::OcTreeVolume> m_grid_voxels;

  bool m_heightColorMode;
  double m_zMin, m_zMax;

  bool m_drawOccupied;
  bool m_drawOcTreeGrid;
  bool m_drawFree;
  bool m_drawSelection;
  bool m_octree_grid_vis_initialized;

  unsigned int m_max_tree_depth;
  double m_alphaOccupied;

  double minX, minY, minZ, maxX, maxY, maxZ;

  float m_ocTreeTransform[16];

};
}

namespace octomap {

MyOcTreeDrawer::MyOcTreeDrawer() :
    m_occupiedThresSize(0), m_freeThresSize(0), m_occupiedSize(0), m_freeSize(0), m_selectionSize(0),
        octree_grid_vertex_size(0), m_alphaOccupied(0.8)
{
  m_octree_grid_vis_initialized = false;
  m_drawOccupied = false;
  m_drawOcTreeGrid = false;
  m_drawFree = false;
  m_drawSelection = true;

  m_heightColorMode = false;
  m_zMin = -1;
  m_zMax = 12;

  m_occupiedArray = NULL;
  m_freeArray = NULL;
  m_occupiedThresArray = NULL;
  m_freeThresArray = NULL;
  m_occupiedColorArray = NULL;
  m_occupiedThresColorArray = NULL;
  m_selectionArray = NULL;
}

MyOcTreeDrawer::~MyOcTreeDrawer()
{
  clear();
}

void MyOcTreeDrawer::setAlphaOccupied(double alpha)
{
  m_alphaOccupied = alpha;
}

void MyOcTreeDrawer::setOcTree(const octomap::OcTree& octree, double minDrawZ, double maxDrawZ)
{

  octree.getMetricMin(minX, minY, minZ);
  octree.getMetricMax(maxX, maxY, maxZ);
  //printf("map bounds: [%.2f, %.2f, %.2f] - [%.2f, %.2f, %.2f]\n", minX, minY, minZ, maxX, maxY, maxZ);

  std::list<octomap::OcTreeVolume> occupiedThresVoxels;
  std::list<octomap::OcTreeVolume> freeThresVoxels;
  std::list<octomap::OcTreeVolume> occupiedVoxels;
  std::list<octomap::OcTreeVolume> freeVoxels;

  if (m_drawOccupied) {
    int numremoved = 0;
    octree.getOccupied(occupiedThresVoxels, occupiedVoxels, m_max_tree_depth);

    int size1 = occupiedVoxels.size();
    int size2 = occupiedThresVoxels.size();
    for (std::list<octomap::OcTreeVolume>::iterator it = occupiedVoxels.begin(); it != occupiedVoxels.end();) {
      if (it->first.z() > maxDrawZ || it->first.z() < minDrawZ) {
        it = occupiedVoxels.erase(it);
        numremoved++;
      }
      else
        it++;
    }
    for (std::list<octomap::OcTreeVolume>::iterator it = occupiedThresVoxels.begin(); it != occupiedThresVoxels.end();
        ) {
      if (it->first.z() > maxDrawZ || it->first.z() < minDrawZ) {
        it = occupiedThresVoxels.erase(it);
        numremoved++;
      }
      else
        it++;
    }
    int size3 = occupiedVoxels.size();
    int size4 = occupiedThresVoxels.size();
    //printf("thresh=[%f,%f], numremoved=%d, sizes %d %d %d %d\n", minDrawZ, maxDrawZ, numremoved, size1, size2, size3,
    //    size4);
  }

  if (m_drawFree) {
    octree.getFreespace(freeThresVoxels, freeVoxels, m_max_tree_depth);
    int numremoved = 0;
    int size1 = freeVoxels.size();
    int size2 = freeThresVoxels.size();
    for (std::list<octomap::OcTreeVolume>::iterator it = freeVoxels.begin(); it != freeVoxels.end();) {
      if (it->first.z() > maxDrawZ || it->first.z() < minDrawZ) {
        it = freeVoxels.erase(it);
        numremoved++;
      }
      else
        it++;
    }
    for (std::list<octomap::OcTreeVolume>::iterator it = freeThresVoxels.begin(); it != freeThresVoxels.end();
        ) {
      if (it->first.z() > maxDrawZ || it->first.z() < minDrawZ) {
        it = freeThresVoxels.erase(it);
        numremoved++;
      }
      else
        it++;
    }
    int size3 = freeVoxels.size();
    int size4 = freeThresVoxels.size();
    //printf("free numremoved=%d, sizes %d %d %d %d\n", numremoved, size1, size2, size3,
    //    size4);
  }

  m_octree_grid_vis_initialized = false;
//  if (m_drawOcTreeGrid && octree.size() < 10 * 1e6) {
//    octree.getVoxels(m_grid_voxels, m_max_tree_depth - 1); // octree structure not drawn at lowest level
//    initOctreeGridVis();
//  }

  // initialize visualization:
  generateCubes(occupiedThresVoxels, &m_occupiedThresArray, m_occupiedThresSize, &m_occupiedThresColorArray);
  generateCubes(freeThresVoxels, &m_freeThresArray, m_freeThresSize);

  generateCubes(occupiedVoxels, &m_occupiedArray, m_occupiedSize, &m_occupiedColorArray);
  generateCubes(freeVoxels, &m_freeArray, m_freeSize);
}

void MyOcTreeDrawer::setOcTreeSelection(const std::list<octomap::OcTreeVolume>& selectedVoxels)
{
  generateCubes(selectedVoxels, &m_selectionArray, m_selectionSize);

}

void MyOcTreeDrawer::clearOcTreeSelection()
{
  clearCubes(&m_selectionArray, m_selectionSize);
}

void MyOcTreeDrawer::generateCubes(const std::list<octomap::OcTreeVolume>& voxels, GLfloat*** glArray,
    unsigned int& glArraySize, GLfloat** glColorArray)
{

  // clear arrays first if needed:
  clearCubes(glArray, glArraySize, glColorArray);

  // now, allocate arrays:
  glArraySize = voxels.size() * 4 * 3;
  *glArray = new GLfloat*[6];

  for (unsigned i = 0; i < 6; ++i) {
    (*glArray)[i] = new GLfloat[glArraySize];
  }

  if (glColorArray != NULL
  )
    *glColorArray = new GLfloat[glArraySize * 4 * 4];

    // epsilon to be substracted from cube size so that neighboring planes don't overlap
    // seems to introduce strange artifacts nevertheless...
double  eps = 1e-5;

  // generate the cubes, 6 quads each
  // min and max-values are computed on-the-fly
  unsigned int i = 0;
  unsigned int colorIdx = 0;
  double x, y, z;

  for (std::list<octomap::OcTreeVolume>::const_iterator it = voxels.begin(); it != voxels.end(); it++) {

    double half_cube_size = GLfloat(it->second / 2.0 - eps);

    x = it->first.x();
    y = it->first.y();
    z = it->first.z();

    // Cube surfaces are in gl_array in order: back, front, top, down, left, right.
    // Arrays are filled in parrallel (increasing i for all at once)
    // One color array for all surfaces is filled when requested

    (*glArray)[0][i] = x + half_cube_size;
    (*glArray)[0][i + 1] = y + half_cube_size;
    (*glArray)[0][i + 2] = z - half_cube_size;

    (*glArray)[1][i] = x + half_cube_size;
    (*glArray)[1][i + 1] = y - half_cube_size;
    (*glArray)[1][i + 2] = z - half_cube_size;

    (*glArray)[2][i] = x + half_cube_size;
    (*glArray)[2][i + 1] = y + half_cube_size;
    (*glArray)[2][i + 2] = z - half_cube_size;

    (*glArray)[3][i] = x - half_cube_size;
    (*glArray)[3][i + 1] = y + half_cube_size;
    (*glArray)[3][i + 2] = z - half_cube_size;

    (*glArray)[4][i] = x + half_cube_size;
    (*glArray)[4][i + 1] = y + half_cube_size;
    (*glArray)[4][i + 2] = z - half_cube_size;

    (*glArray)[5][i] = x + half_cube_size;
    (*glArray)[5][i + 1] = y + half_cube_size;
    (*glArray)[5][i + 2] = z + half_cube_size;
    i += 3;

    (*glArray)[0][i] = x - half_cube_size;
    (*glArray)[0][i + 1] = y + half_cube_size;
    (*glArray)[0][i + 2] = z - half_cube_size;

    (*glArray)[1][i] = x - half_cube_size;
    (*glArray)[1][i + 1] = y - half_cube_size;
    (*glArray)[1][i + 2] = z - half_cube_size;

    (*glArray)[2][i] = x + half_cube_size;
    (*glArray)[2][i + 1] = y + half_cube_size;
    (*glArray)[2][i + 2] = z + half_cube_size;

    (*glArray)[3][i] = x - half_cube_size;
    (*glArray)[3][i + 1] = y + half_cube_size;
    (*glArray)[3][i + 2] = z + half_cube_size;

    (*glArray)[4][i] = x + half_cube_size;
    (*glArray)[4][i + 1] = y - half_cube_size;
    (*glArray)[4][i + 2] = z - half_cube_size;

    (*glArray)[5][i] = x + half_cube_size;
    (*glArray)[5][i + 1] = y - half_cube_size;
    (*glArray)[5][i + 2] = z + half_cube_size;
    i += 3;

    (*glArray)[0][i] = x - half_cube_size;
    (*glArray)[0][i + 1] = y + half_cube_size;
    (*glArray)[0][i + 2] = z + half_cube_size;

    (*glArray)[1][i] = x - half_cube_size;
    (*glArray)[1][i + 1] = y - half_cube_size;
    (*glArray)[1][i + 2] = z + half_cube_size;

    (*glArray)[2][i] = x + half_cube_size;
    (*glArray)[2][i + 1] = y - half_cube_size;
    (*glArray)[2][i + 2] = z + half_cube_size;

    (*glArray)[3][i] = x - half_cube_size;
    (*glArray)[3][i + 1] = y - half_cube_size;
    (*glArray)[3][i + 2] = z + half_cube_size;

    (*glArray)[4][i] = x - half_cube_size;
    (*glArray)[4][i + 1] = y - half_cube_size;
    (*glArray)[4][i + 2] = z - half_cube_size;

    (*glArray)[5][i] = x - half_cube_size;
    (*glArray)[5][i + 1] = y - half_cube_size;
    (*glArray)[5][i + 2] = z + half_cube_size;
    i += 3;

    (*glArray)[0][i] = x + half_cube_size;
    (*glArray)[0][i + 1] = y + half_cube_size;
    (*glArray)[0][i + 2] = z + half_cube_size;

    (*glArray)[1][i] = x + half_cube_size;
    (*glArray)[1][i + 1] = y - half_cube_size;
    (*glArray)[1][i + 2] = z + half_cube_size;

    (*glArray)[2][i] = x + half_cube_size;
    (*glArray)[2][i + 1] = y - half_cube_size;
    (*glArray)[2][i + 2] = z - half_cube_size;

    (*glArray)[3][i] = x - half_cube_size;
    (*glArray)[3][i + 1] = y - half_cube_size;
    (*glArray)[3][i + 2] = z - half_cube_size;

    (*glArray)[4][i] = x - half_cube_size;
    (*glArray)[4][i + 1] = y + half_cube_size;
    (*glArray)[4][i + 2] = z - half_cube_size;

    (*glArray)[5][i] = x - half_cube_size;
    (*glArray)[5][i + 1] = y + half_cube_size;
    (*glArray)[5][i + 2] = z + half_cube_size;
    i += 3;

    if (glColorArray != NULL) {
      // color for 4 vertices (same height)
      for (int k = 0; k < 4; ++k) {

        //          SceneObject::heightMapColor(z, *glColorArray + colorIdx);
        double z_norm = (z - m_zMin) / (m_zMax - m_zMin);
        float * jetC = bot_color_util_jet(z_norm);
        memcpy(*glColorArray + colorIdx, jetC, 3 * sizeof(float));
        // set Alpha value:
        (*glColorArray)[colorIdx + 3] = m_alphaOccupied;
        colorIdx += 4;
      }
    }

  }
}

void MyOcTreeDrawer::clearCubes(GLfloat*** glArray, unsigned int& glArraySize, GLfloat** glColorArray)
{
  if (glArraySize != 0) {
    for (unsigned i = 0; i < 6; ++i) {
      delete[] (*glArray)[i];
    }
    delete[] *glArray;
    *glArray = NULL;
    glArraySize = 0;
  }

  if (glColorArray != NULL && *glColorArray != NULL) {
    delete[] *glColorArray;
    *glColorArray = NULL;
  }
}

void MyOcTreeDrawer::initOctreeGridVis()
{

  if (m_octree_grid_vis_initialized)
    return;

  clearOcTreeStructure();
  // allocate arrays for octree grid visualization
  octree_grid_vertex_size = m_grid_voxels.size() * 12 * 2 * 3;
  octree_grid_vertex_array = new GLfloat[octree_grid_vertex_size];

  // generate the cubes, 12 lines each

  std::list<octomap::OcTreeVolume>::iterator it_rec;
  unsigned int i = 0;
  double x, y, z;
  for (it_rec = m_grid_voxels.begin(); it_rec != m_grid_voxels.end(); it_rec++) {

    x = it_rec->first.x();
    y = it_rec->first.y();
    z = it_rec->first.z();

    double half_voxel_size = it_rec->second / 2.0;

    // ----
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    // ----
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    // ----
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    // ----
  }

  m_octree_grid_vis_initialized = true;
}

void MyOcTreeDrawer::clearOcTreeStructure()
{
  if (octree_grid_vertex_size != 0) {
    delete[] octree_grid_vertex_array;
    octree_grid_vertex_size = 0;
  }

  m_octree_grid_vis_initialized = false;
}

void MyOcTreeDrawer::clear()
{
  //clearOcTree();
  clearCubes(&m_occupiedArray, m_occupiedSize, &m_occupiedColorArray);
  clearCubes(&m_occupiedThresArray, m_occupiedThresSize, &m_occupiedThresColorArray);
  clearCubes(&m_freeArray, m_freeSize);
  clearCubes(&m_freeThresArray, m_freeThresSize);
  clearCubes(&m_selectionArray, m_selectionSize);

  clearOcTreeStructure();
}

void MyOcTreeDrawer::draw() const
{
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glPushMatrix();
  glMultMatrixf(m_ocTreeTransform);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  //        glEnable(GL_LIGHTING);

  glEnableClientState(GL_VERTEX_ARRAY);

  if (m_drawOccupied)
    drawOccupiedVoxels();
  if (m_drawFree)
    drawFreeVoxels();
  if (m_drawOcTreeGrid)
    drawOctreeGrid();
  if (m_drawSelection)
    drawSelection();

  glDisableClientState(GL_VERTEX_ARRAY);

  glPopMatrix();
  glPopAttrib();

}

void MyOcTreeDrawer::drawOccupiedVoxels() const
{

  // draw binary occupied cells
  if (m_occupiedThresSize != 0) {
    glColor4f(0.0, 0.0, 0.0, m_alphaOccupied);
    drawCubes(m_occupiedThresArray, m_occupiedThresSize, m_occupiedThresColorArray);
  }

  // draw delta occupied cells
  if (m_occupiedSize != 0) {
    glColor4f(0.2, 0.7, 1.0, m_alphaOccupied);
    drawCubes(m_occupiedArray, m_occupiedSize, m_occupiedColorArray);
  }

}

void MyOcTreeDrawer::drawFreeVoxels() const
{

  // draw binary freespace cells
  if (m_freeThresSize != 0) {
    glColor4f(0.0, 1.0, 0., 0.3);
    drawCubes(m_freeThresArray, m_freeThresSize);
  }

  // draw delta freespace cells
  if (m_freeSize != 0) {
    glColor4f(0.5, 1.0, 0.1, 0.3);
    drawCubes(m_freeArray, m_freeSize);
  }
}

void MyOcTreeDrawer::drawSelection() const
{
  if (m_selectionSize != 0) {
    glColor4f(1.0, 0.0, 0.0, 0.5);
    drawCubes(m_selectionArray, m_selectionSize);
  }
}

void MyOcTreeDrawer::drawCubes(GLfloat** cubeArray, unsigned int cubeArraySize, GLfloat* cubeColorArray) const
    {
  if (cubeArraySize == 0 || cubeArray == NULL) {
    std::cerr << "Warning: GLfloat array to draw cubes appears to be empty, nothing drawn.\n";
    return;
  }

  // save current color
  GLfloat* curcol = new GLfloat[4];
  glGetFloatv(GL_CURRENT_COLOR, curcol);

  // enable color pointer when heightColorMode is enabled:

  if (m_heightColorMode && cubeColorArray != NULL) {
    glEnableClientState(GL_COLOR_ARRAY);
    glColorPointer(4, GL_FLOAT, 0, cubeColorArray);
  }
  else{
    glColor4f(.1,.1,.1,.5);
  }

  // top surfaces:
  glNormal3f(0.0f, 1.0f, 0.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[0]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
  // bottom surfaces:
  glNormal3f(0.0f, -1.0f, 0.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[1]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
  // right surfaces:
  glNormal3f(1.0f, 0.0f, 0.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[2]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
  // left surfaces:
  glNormal3f(-1.0f, 0.0f, 0.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[3]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
  // back surfaces:
  glNormal3f(0.0f, 0.0f, -1.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[4]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
  // front surfaces:
  glNormal3f(0.0f, 0.0f, 1.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[5]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);

  if (m_heightColorMode && cubeColorArray != NULL) {
    glDisableClientState(GL_COLOR_ARRAY);
  }

  // reset color
  glColor4fv(curcol);
  delete[] curcol;
}

void MyOcTreeDrawer::drawOctreeGrid() const
{
  if (!m_octree_grid_vis_initialized)
    return;

  if (octree_grid_vertex_size == 0)
    return;

  glDisable(GL_LIGHTING);
  glEnable(GL_LINE_SMOOTH);

  glLineWidth(1.);
  glVertexPointer(3, GL_FLOAT, 0, octree_grid_vertex_array);
  glColor3f(0.0, 0.0, 0.0);
  glDrawArrays(GL_LINES, 0, octree_grid_vertex_size / 3);

  glDisable(GL_LINE_SMOOTH);
  glEnable(GL_LIGHTING);
}

void MyOcTreeDrawer::enableOcTree(bool enabled)
{
  m_drawOcTreeGrid = enabled;
  if (m_drawOcTreeGrid && !m_octree_grid_vis_initialized) {
    initOctreeGridVis();
  }
}

}

using namespace octomap;

typedef enum _color_mode_t {
  COLOR_MODE_DRAB, COLOR_MODE_Z,
} color_mode_t;

typedef struct _BotRendererOctomap BotRendererOctomap;

struct _BotRendererOctomap {
  BotRenderer renderer;
  BotEventHandler ehandler;
  lcm_t *lc;
  BotGtkParamWidget *pw;
  GtkWidget *label;
  BotViewer *viewer;

  OcTree * ocTree;
  MyOcTreeDrawer * octd;

};

static void Octomap_draw(BotViewer *viewer, BotRenderer *renderer)
{
  BotRendererOctomap *self = (BotRendererOctomap*) renderer;
  if (self->octd->m_drawOccupied || self->octd->m_drawFree || self->octd->m_drawOcTreeGrid) {
    if (self->ocTree == NULL) {
      return;
    }
    self->octd->draw();
  }

}

static void Octomap_free(BotRenderer *renderer)
{
  //TODO: don't LEAK!

  free(renderer);
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  BotRendererOctomap *self = (BotRendererOctomap*) user;
  if (self->octd == NULL
  )
    return;

  int color_mod = bot_gtk_param_widget_get_enum(self->pw, PARAM_COLOR_MODE);
  switch (color_mod) {
  case COLOR_MODE_Z:
    {
      self->octd->m_heightColorMode = true;
    }
    break;
  case COLOR_MODE_DRAB:
    default:
    self->octd->m_heightColorMode = false;
    break;
  }
  self->octd->m_zMin = bot_gtk_param_widget_get_double(self->pw, PARAM_COLOR_MODE_Z_MIN_Z);
  self->octd->m_zMax = bot_gtk_param_widget_get_double(self->pw, PARAM_COLOR_MODE_Z_MAX_Z);
  self->octd->setMax_tree_depth(bot_gtk_param_widget_get_int(self->pw, PARAM_TREE_DEPTH));

  self->octd->enableFreespace(bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_FREE));
  self->octd->enableOcTreeCells(bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_OCC));
//  self->octd->enableOcTree(bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_GRID));

  if (self->ocTree != NULL && (strcmp(name, PARAM_COLOR_MODE_Z_MIN_Z) == 0 || strcmp(name, PARAM_COLOR_MODE_Z_MAX_Z)
      == 0 || strcmp(name, PARAM_TREE_DEPTH) == 0 || strcmp(name, PARAM_SHOW_FREE) || strcmp(name, PARAM_SHOW_OCC)))
//      || strcmp(name, PARAM_SHOW_GRID)))
    self->octd->setOcTree(*self->ocTree, bot_gtk_param_widget_get_double(self->pw, PARAM_Z_MIN),
        bot_gtk_param_widget_get_double(self->pw, PARAM_Z_MAX)); //regenerate cubes etc..

  bot_viewer_request_redraw(self->viewer);
}

static void on_clear_button(GtkWidget *button, BotRendererOctomap *self)
{
  if (!self->viewer)
    return;

  if (self->ocTree != NULL
  )
    delete self->ocTree;
  self->ocTree = NULL;

  bot_viewer_request_redraw(self->viewer);
}

static void on_load_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user)
{
  if (!viewer)
    return;

  BotRendererOctomap *self = (BotRendererOctomap*) user;
  bot_gtk_param_widget_load_from_key_file(self->pw, keyfile, RENDERER_NAME);
}

static void on_save_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user)
{
  if (!viewer)
    return;

  BotRendererOctomap *self = (BotRendererOctomap*) user;
  bot_gtk_param_widget_save_to_key_file(self->pw, keyfile, RENDERER_NAME);
}

static void on_octomap(const lcm_recv_buf_t *rbuf, const char *channel, const octomap_raw_t *msg, void *user_data)
{
  BotRendererOctomap *self = (BotRendererOctomap*) user_data;
  if (self->ocTree != NULL)
    delete self->ocTree;
  self->octd->clear();

  // set transform
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      self->octd->m_ocTreeTransform[j*4+i] = msg->transform[i][j];
    }
  }

  std::stringstream datastream;
  datastream.write((const char*) msg->data, msg->length);
  self->ocTree = new octomap::OcTree(1); //resolution will be set by data from message
  self->ocTree->readBinary(datastream);
  double minX, minY, minZ, maxX, maxY, maxZ;
  self->ocTree->getMetricMin(minX, minY, minZ);
  self->ocTree->getMetricMax(maxX, maxY, maxZ);
  //printf("\nmap bounds: [%.2f, %.2f, %.2f] - [%.2f, %.2f, %.2f]\n", minX, minY, minZ, maxX, maxY, maxZ);
  self->octd->setOcTree(*self->ocTree, bot_gtk_param_widget_get_double(self->pw, PARAM_Z_MIN),
      bot_gtk_param_widget_get_double(self->pw, PARAM_Z_MAX));
}

static void on_find_button(GtkWidget *button, BotRendererOctomap *self)
{
  BotViewHandler *vhandler = self->viewer->view_handler;

  double eye[3];
  double lookat[3];
  double up[3];

  double octomap_center[3] = { (self->octd->minX + self->octd->maxX) / 2,
      (self->octd->minY + self->octd->maxY) / 2,
      (self->octd->minZ + self->octd->maxZ) / 2 };

  vhandler->get_eye_look(vhandler, eye, lookat, up);
  double diff[3];
  bot_vector_subtract_3d(eye, lookat, diff);

  bot_vector_add_3d(octomap_center, diff, eye);

  vhandler->set_look_at(vhandler, eye, octomap_center, up);

  bot_viewer_request_redraw(self->viewer);
}

BotRenderer *renderer_octomap_new(BotViewer *viewer, int render_priority, lcm_t * lcm)
{
  BotRendererOctomap *self = (BotRendererOctomap*) calloc(1, sizeof(BotRendererOctomap));
  BotRenderer *renderer = &self->renderer;
  self->viewer = viewer;
  self->lc = lcm;

  self->ocTree = NULL;
  self->octd = new MyOcTreeDrawer();

  octomap_raw_t_subscribe(self->lc, "OCTOMAP", on_octomap, self);

  renderer->draw = Octomap_draw;
  renderer->destroy = Octomap_free;
  renderer->widget = gtk_vbox_new(FALSE, 0);
  renderer->name = (char *) RENDERER_NAME;
  renderer->user = self;
  renderer->enabled = 1;

  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
  renderer->widget = gtk_vbox_new(FALSE, 0);
  gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
  bot_gtk_param_widget_add_enum(self->pw, PARAM_COLOR_MODE, BOT_GTK_PARAM_WIDGET_MENU, COLOR_MODE_Z, "Height",
      COLOR_MODE_Z, "Drab", COLOR_MODE_DRAB, NULL);

  //  bot_gtk_param_widget_add_int(self->pw, PARAM_POINT_SIZE, BOT_GTK_PARAM_WIDGET_SLIDER, 1, 20, 1, 4);

  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_OCC, 0, NULL);
  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_FREE, 0, NULL);
//  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_GRID, 0, NULL);

  bot_gtk_param_widget_add_int(self->pw, PARAM_TREE_DEPTH, BOT_GTK_PARAM_WIDGET_SPINBOX, 1, 16, 1, 16);

  bot_gtk_param_widget_add_double(self->pw, PARAM_COLOR_MODE_Z_MIN_Z, BOT_GTK_PARAM_WIDGET_SPINBOX, round(
      self->octd->m_zMin - 2), round(self->octd->m_zMax + 5), .5, round(self->octd->m_zMin));
  bot_gtk_param_widget_add_double(self->pw, PARAM_COLOR_MODE_Z_MAX_Z, BOT_GTK_PARAM_WIDGET_SPINBOX, round(
      self->octd->m_zMin - 2), round(self->octd->m_zMax + 5), .5, round(self->octd->m_zMax));

  bot_gtk_param_widget_add_double(self->pw, PARAM_Z_MIN, BOT_GTK_PARAM_WIDGET_SPINBOX,
      round(self->octd->m_zMin - 5), round(self->octd->m_zMax + 2), .05, round(self->octd->m_zMin));

  bot_gtk_param_widget_add_double(self->pw, PARAM_Z_MAX, BOT_GTK_PARAM_WIDGET_SPINBOX,
      round(self->octd->m_zMin - 2), round(self->octd->m_zMax + 5), .05, round(self->octd->m_zMax));

  GtkWidget *find_button = gtk_button_new_with_label("Find");
  gtk_box_pack_start(GTK_BOX(renderer->widget), find_button, FALSE, FALSE, 0);
  g_signal_connect(G_OBJECT(find_button), "clicked", G_CALLBACK(on_find_button), self);

  GtkWidget *clear_button = gtk_button_new_with_label("Clear memory");
  gtk_box_pack_start(GTK_BOX(renderer->widget), clear_button, FALSE, FALSE, 0);
  g_signal_connect(G_OBJECT(clear_button), "clicked", G_CALLBACK(on_clear_button), self);

  gtk_widget_show_all(renderer->widget);
  g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  g_signal_connect(G_OBJECT(viewer), "load-preferences", G_CALLBACK(on_load_preferences), self);
  g_signal_connect(G_OBJECT(viewer), "save-preferences", G_CALLBACK(on_save_preferences), self);

  return &self->renderer;
}

extern "C" void add_octomap_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t * lcm)
{
  bot_viewer_add_renderer(viewer, renderer_octomap_new(viewer, render_priority, lcm), render_priority);
}
