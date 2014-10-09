#ifndef __bot2_core_h__
#define __bot2_core_h__

#include "math_util.h"
#include "small_linalg.h"
#include "camtrans.h"
#include "circular.h"
#include "ctrans.h"
#include "fasttrig.h"
#include "fileutils.h"
#include "glib_util.h"
#include "gps_linearize.h"
#include "lcm_util.h"
#include "minheap.h"
#include "ppm.h"
#include "ptr_circular.h"
#include "rotations.h"
#include "serial.h"
#include "set.h"
#include "signal_pipe.h"
#include "ssocket.h"
#include "tictoc.h"
#include "timespec.h"
#include "timestamp.h"
#include "trans.h"
#include "color_util.h"
#include "rand_util.h"
#include "ringbuf.h"

#include <lcmtypes/bot_core_image_t.h>
#include <lcmtypes/bot_core_image_sync_t.h>
#include <lcmtypes/bot_core_planar_lidar_t.h>
#include <lcmtypes/bot_core_pose_t.h>
#include <lcmtypes/bot_core_raw_t.h>
#include <lcmtypes/bot_core_rigid_transform_t.h>

/**
 * @defgroup BotCore Bot Core
 * @brief Core data structures and functions
 */

/**
 * @defgroup BotCoreMathGeom Math / Geometry
 * @ingroup BotCore
 * @brief Functions and data structures for math and geometry
 */

/**
 * @defgroup BotCoreDataStructures Data Structures
 * @ingroup BotCore
 * @brief General purpose data structures
 */

/**
 * @defgroup BotCoreIO I/O
 * @ingroup BotCore
 * @brief I/O routines (files, sockets, etc.) and reading/writing certain file formats.
 */

/**
 * @defgroup BotCoreTime Time
 * @ingroup BotCore
 * @brief Time keeping
 */

#endif
