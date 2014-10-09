#ifndef __ar_bot_frames_h__
#define __ar_bot_frames_h__

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * SECTION:BotFrames:
 * @title: Bot Frames
 * @short description: Frames for kinematic models
 * @include: bot_param/param_client.h
 *
 * BotFrames is intended to be a one-stop shop for coordinate frame
 * transformations.
 *
 * It assumes that there is a block in the param file specifying the layout of
 * the coordinate frames.
 * The coordinate frames are constructed from the "coordinate_frames" block of
 * the param file. The param block should be laid out as shown below.
 *
 * frames can be updated by one of three ways:
 *      1) publishing a bot frames update using the bot_frames_update_frame() function
 *      2) defining an update_channel name, which should receive bot_core_rigid_transform_t messages
 *      3) defining a pose_update_channel, where bot_core_pose_t messages will be listened for
 *
 *
 * It assumes that there is a block in the param file specifying the layout of the coordinate frames.
 * For example:
 *
 coordinate_frames {
   root_frame = "local";                 #a root_frame must be defined

   body {
     relative_to = "local";
     history = 1000;                    #number of past transforms to keep around,
     pose_update_channel = "POSE";      #bot_core_pose_t messages will be listened for this channel
     initial_transform{
       translation = [ 0, 0, 0 ];       #(x,y,z) translation vector
       quat = [ 1, 0, 0, 0 ];           #may be specified as a quaternion, rpy, rodrigues, or axis-angle
     }
   }
   laser {
     relative_to = "body";
     history = 0;
     update_channel = "LASER_TO_BODY";               #specify an update_channel to listen on
     initial_transform{
       translation = [ 0, 0, 0 ];
       rpy = [ 0, 0, 0 ];
     }
   }
   camera {
     relative_to = "body";
     history = 0;
     initial_transform{
       translation = [ 0, 0, 0 ];
       rodrigues = [ 0, 0, 0 ];
     }
   }
  #etc...
}
 *
 *@{
 */
typedef struct _BotFrames BotFrames;

/**
 * bot_frames_new
 *
 * allocates and initializes a new BotFrames pointer with a bot_param
 * structure generated from either a server or file with a "coordinate_frames"
 * parameter described above. Uses the "update_channel" to subscribe to
 * channels within LCM
 *
 * bot_param: pointer to a BotParam structure that contains a
 *	"coordinate_frames" key
 * lcm: pointer to an LCM structure that is listening for changes to the
 *	frames described on the "coordinate_frames.'frame'.update_channel
 *	string
 *
 * returns a newly allocated/initialized pointer to a BotFrames structure
 * 
 */
BotFrames * bot_frames_new(lcm_t *lcm, BotParam *bot_param);

/**
 * bot_frames_destroy
 *
 * free's a BotFrames structure allocated by bot_frames_new
 */
void bot_frames_destroy(BotFrames * bot_frames);

/**
 * bot_frames_get_global
 *
 * Conveniance function to get a globally shared frames object
 */
BotFrames * bot_frames_get_global(lcm_t *lcm, BotParam *bot_param);


/**
 * bot_frames_get_latest_timestamp
 * bot_frames: pointer to a BotFrames structure that is to be modified
 * 	frames
 * from_frame: source coordinate frame
 * to_frame: destination coordinate frame
 * timestamp: output parameter
 *
 * Retrieves the timestamp of the most recent rigid body transformation 
 * relating two coordinate frames.
 *
 * Since there may be many links relating two coordinate frames, this 
 * retrieves the timestamp of the most recently updated link.  For information
 * about a specific link, use bot_ctrans_link_get_nth_trans()
 * 
 * Returns: 1 on success, 0 on failure.
 */
int bot_frames_get_latest_timestamp(BotFrames * bot_frames, 
                                        const char *from_frame, const char *to_frame, int64_t *timestamp);

/**
 * bot_frames_update_frame
 *
 * bot_frames: pointer to a BotFrames structure that is to be modified
 * 	frames
 * frame_name: name of the frame to update
 * relative_to: name of the frame this new frame is relative to
 * trans: transformation: from (frame_name) to (relative_to)
 * utime: timestamp
 *
 * Publish a frame update message
 */

void bot_frames_update_frame(BotFrames * bot_frames, const char * frame_name,
    const char * relative_to, const BotTrans * trans, int64_t utime);

/** 
 * bot_frames_link_update_handler_t
 *
 * LCM handler function template for a BotFrames callback
 *
 * bot_frames: BotFrames structure that will be updated
 * frame: name of the frame that is updated
 * relative_to: name of the frame that this is updated relative to
 * user: user data that was passed to the function
 */
typedef void(bot_frames_link_update_handler_t)(BotFrames *bot_frames,
             const char *frame, const char * relative_to, int64_t utime, void *user);

/**
 * bot_frames_add_update_subscriber
 *
 * add a callback handler to get called when a link is updated
 * the template of the callback handler is described above
 *
 * bot_frames: the BotFrames structure that should have updates
 * callback_func: function to call when new data is recieved
 * user: user data to be passed to the function
 */
void bot_frames_add_update_subscriber(BotFrames *bot_frames,
    bot_frames_link_update_handler_t * callback_func, void * user);


/**
 * bot_frames_get_trans
 *
 * compute the latest rigid body transformation from one coordinate frame to
 * another.
 *
 * bot_frames: BotFrames structure to get transforms
 * from_frame: string of the name of the frame at the start of the transform
 * to_frame: string of the name of the frame at the end of the transformation
 * result: resulting transformation
 *
 * Returns: 1 on success, 0 on failure
 */
int bot_frames_get_trans(BotFrames *bot_frames, const char *from_frame,
        const char *to_frame, BotTrans *result);

/**
 * bot_frames_get_trans_with_utime
 *
 * compute the rigid body transformation from one coordinate frame
 * to another.
 *
 * bot_frames: BotFrames structure to get transforms
 * from_frame: string of the name of the frame at the start of the transform
 * to_frame: string of the name of the frame at the end of the transform
 * utime: time (in microseconds) of the transform
 * result: resulting transformation
 *
 * Returns: 1 on success, 0 on failure
 */
int bot_frames_get_trans_with_utime(BotFrames *bot_frames, const char *from_frame,
        const char *to_frame, int64_t utime, BotTrans *result);


/**
 * bot_frames_get_trans_latest_timestamp
 *
 * Get the timestamp of the most recent transform
 *
 * bot_frames: BotFrames structure to get transforms
 * from_frame: string of the name of the frame at the start of the transform
 * to_frame: string of then name of the frame at the end of the transfrom

 *
 * Returns: 1 if the requested transformation is availabe, 0 if not
 */
int
bot_frames_get_trans_latest_timestamp(BotFrames *bot_frames, const char *from_frame, const char *to_frame, int64_t *timestamp);


/**
 * bot_frames_have_trans
 *
 * Returns: 1 if the requested transformation is availabe, 0 if not
 *
 */
int bot_frames_have_trans(BotFrames *bot_frames, const char *from_frame,
        const char *to_frame);

/**
 * bot_frames_get_trans_mat_3x4
 *
 * convenience function: returns a 3 x 4 matrix representing a transfrom
 * 	from "from_frame" to "to_frame"
 *
 */
int bot_frames_get_trans_mat_3x4(BotFrames *bot_frames, const char *from_frame,
        const char *to_frame, double mat[12]);
/**
 * bot_frames_get_trans_mat_3x4_with_utime
 *
 * convenience function: returns a 3 x 4 matrix representing a transfrom
 * 	from "from_frame" to "to_frame" as well as a time stamp
 *
 */
int bot_frames_get_trans_mat_3x4_with_utime(BotFrames *bot_frames,
        const char *from_frame, const char *to_frame, int64_t utime,
        double mat[12]);

// convenience function
int bot_frames_get_trans_mat_4x4(BotFrames *bot_frames, const char *from_frame,
        const char *to_frame, double mat[16]);

// convenience function
int bot_frames_get_trans_mat_4x4_with_utime(BotFrames *bot_frames,
        const char *from_frame, const char *to_frame, int64_t utime,
        double mat[16]);


/**
 * Transforms a vector from one coordinate frame to another
 *
 * Returns: 1 on success, 0 on failure
 */
int bot_frames_transform_vec(BotFrames *bot_frames, const char *from_frame,
        const char *to_frame, const double src[3], double dst[3]);

/**
 * Rotates a vector from one coordinate frame to another.  Does not apply
 * any translations.
 *
 * Returns: 1 on success, 0 on failure
 */
int bot_frames_rotate_vec(BotFrames *bot_frames, const char *from_frame,
        const char *to_frame, const double src[3], double dst[3]);

/**
 * Retrieves the number of transformations available for the specified link.
 * Only valid for <from_frame, to_frame> pairs that are directly linked.  e.g.
 * <body, local> is valid, but <camera, local> is not.
 */
int bot_frames_get_n_trans(BotFrames *bot_frames, const char *from_frame,
        const char *to_frame, int nth_from_latest);

/**
 * @nth_from_latest:  0 corresponds to most recent transformation update
 * @btrans: may be NULL
 * @timestamp: may be NULL
 *
 * Returns: 1 on success, 0 on failure
 */
int bot_frames_get_nth_trans(BotFrames *bot_frames, const char *from_frame,
        const char *to_frame, int nth_from_latest,
        BotTrans *btrans, int64_t *timestamp);

/**
 * bot_frames_get_relative_to
 *
 * Retrieves the name of the frame that (frame_name) is relative to
 *
 * Returns: the relative_to value on success, or NULL if failure
 *
 */
const char * bot_frames_get_relative_to(BotFrames *bot_frames, const char *frame_name);

/**
 * Returns: the number of frames managed by this instance
 */
int bot_frames_get_num_frames(BotFrames * bot_frames);

/**
 *
 * Returns: a newly allocated array of strings containing the names of the frames
 *              the strings and the array must be freed!
 */

char ** bot_frames_get_frame_names(BotFrames * bot_frames);

/**
 *
 * Returns: a string containing the name of the root
 *              coordinate frame
 */
const char * bot_frames_get_root_name(BotFrames * bot_frames);



#ifdef __cplusplus
}
#endif
/**
 * @}
 */
#endif
