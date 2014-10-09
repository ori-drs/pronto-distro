/*
 * laser_utils.h
 *
 *  Created on: Jun 27, 2009
 *      Author: abachrac
 */

#ifndef LASER_UTILS_H_
#define LASER_UTILS_H_

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <geom_utils/geometry.h>


#ifdef __cplusplus
extern "C" {
#endif

  /*
   * structure that holds parameters needed for projecting a laser scan
   */
  typedef struct {
    BotFrames * bot_frames;
    BotParam *param;

    char * laser_name;
    char * coord_frame;

    double max_range; /* max sensor range (from param) */
    double max_range_free_dist; /*project max_range readings out this far (from param) */
    double min_range; /* min sensor range (from param) */

    int heightDownRegion[2]; /*begining and end of beams deflected down by mirror (from param)*/
    int heightUpRegion[2]; /*begining and end of beams deflected down by mirror (from param)*/
    int surroundRegion[2]; /*begining and end of beams not deflected by mirror (from param)*/
    int project_height;
    double laser_frequency; /*frequency with which we should be expecting the laser msgs*/
    char * laser_type;
  } Laser_projector;

  typedef enum{
    laser_surround=0,
    laser_height_down=1,
    laser_height_up=2,
    laser_valid_projection=3, //less than this is a valid point
    laser_max_range=4,
    laser_min_range=5,
    laser_invalid_projection=6
  }laser_point_projection_status;

  /*
   * structure that a projected laser scan
   */
  typedef struct _laser_projected_scan {
    Laser_projector * projector;
    bot_core_planar_lidar_t * rawScan; /*copy of original scan message*/

    BotTrans body;    /* local frame body pose */
    BotTrans origin;  /* scan origin relative to requested dest frame*/
    point3d_t *points; /* points relative to requested dest frame */
    uint8_t *point_status; /* status of point. One of laser_point_projection_status enum values */
    int npoints; /* total number of points in arrays above */
    int numValidPoints; /*number of points that are valid*/

    int projection_status; /*1 if scan was projected using a valid transform, 0 otherwise (timestamp off end of tranform buffer)*/

    double aveSurroundRange; /*average range of Surround points */
    double stddevSurroundRange; /*std-dev of the range of Surround points */

    int64_t utime; /* time when data associated w/ this scan was received */
  } laser_projected_scan;

  /*
   * create a new structure that holds all the necessary info to project the
   * laser ranges from this laser
   * it takes care of max-range, min-range, surround region etc...
   */
  Laser_projector * laser_projector_new(BotParam *param, BotFrames * frames, const char * laser_name, int project_height);
  void laser_projector_destroy(Laser_projector * to_destroy);

  /*
   * take a planar_lidar_t message, and project it into frame %dest_frame for rendering/whatever...
   */
  laser_projected_scan *laser_create_projected_scan_from_planar_lidar(Laser_projector * projector,
      const bot_core_planar_lidar_t *msg, const char * dest_frame);

  /*
   * motion corrects the projection for laser_angular_rate and laser_angular_velocity expressed in laser frame
   * mfallon, march 2014: I did not think this was very accurate, specifically because of bot_trans_set_from_velocities
   */
  laser_projected_scan *laser_create_projected_scan_from_planar_lidar_with_motion(Laser_projector * projector,
      const bot_core_planar_lidar_t *msg, const char * dest_frame, const double laser_angular_rate[3],
      const double laser_velocity[3]);

  /*
   * motion corrects the projection for laser_angular_rate and laser_angular_velocity expressed in laser frame
   * mfallon, march 2014: I did not think this was very accurate, specifically because of bot_trans_set_from_velocities
   */
  int laser_update_projected_scan_with_motion(Laser_projector * projector, laser_projected_scan * proj_scan,
      const char * dest_frame, const double laser_angular_rate[3], const double laser_velocity[3]);


  ///////////////////////////////// NEW NEW NEW NEW NEW NEW //////////////////////////////////
  /*
   * interpolation corrects the projection due to motion expressed in laser frame using bot frames
   * add by mfallon, march 2014: Uses bot-frames to determine required bottrans of start and end of scan 
   * And then interpolates that. It does NOT take into account the motion of the body for that duration
   * that could be supported. I'd like to merge this *with_motion and provide optional arguments as the code is duplicative
   */
  laser_projected_scan *laser_create_projected_scan_from_planar_lidar_with_interpolation(Laser_projector * projector,
      const bot_core_planar_lidar_t *msg, const char * dest_frame);

  /*
   * interpolation corrects the projection due to motion expressed in laser frame using bot frames
   * added by mfallon, march 2014: ses bot-frames to determine required transform of start of scan
   */
  int laser_update_projected_scan_with_interpolation(Laser_projector * projector, laser_projected_scan * proj_scan,
      const char * dest_frame);
  
  /*
   * update the scan with the current transform...
   */
  int laser_update_projected_scan(Laser_projector * projector, laser_projected_scan * proj_scan, const char * dest_frame);
  
  
  void laser_decimate_projected_scan(laser_projected_scan * lscan, int beam_skip, double spatial_decimation_min, double spatial_decimation_max);

  void laser_destroy_projected_scan(laser_projected_scan * proj_scan);

#ifdef __cplusplus
}
#endif

#endif /* LASER_UTILS_H_ */
