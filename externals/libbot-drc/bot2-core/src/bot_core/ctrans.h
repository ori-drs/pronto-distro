#ifndef __bot_ctrans_h__
#define __bot_ctrans_h__

#include <stdint.h>

#include "trans.h"

/**
 * @defgroup BotCoreCTrans CTrans
 * @ingroup BotCoreMathGeom
 * @brief Transforming between multiple coordinate frames.
 * @include: bot_core/bot_core.h
 *
 * Represents a set of cartesian coordinate frames and the possibly
 * time-varying rigid-body transformations between the coordinate frames.
 *
 * Internally, a graph is used to track the coordinate frames.  Each edge in
 * the graph represents a transformation.  To find the transformation from one
 * coordinate frame (source) to another (target), a path is computed in the
 * graph.  The path is then traversed from source to target, and the rigid body
 * transformations are composed together to form a single transformation.
 *
 * Linking: `pkg-config --libs bot2-core`
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _BotCTrans BotCTrans;

/**
 * BotCTransLink:
 *
 * Represents a possibly time-varying rigid body transformation that directly
 * relates two cartesian coordinate frames.
 */
typedef struct _BotCTransLink BotCTransLink;

/**
 * bot_ctrans_new:
 *
 * Constructor.
 * Returns: a newly allocated BotCTrans.  
 */
BotCTrans * bot_ctrans_new(void);

/**
 * bot_ctrans_destroy:
 *
 * Releases memory used by a BotCTrans.  Automatically destroys all
 * BotCTransFrame and BotCTransLink structures related to the BotCTrans.
 */
void bot_ctrans_destroy(BotCTrans *ctrans);

/**
 * bot_ctrans_add_frame:
 * Adds a new coordinate frame.
 *
 * Returns: 1 on success, 0 if the coordinate frame already exists
 */
int bot_ctrans_add_frame(BotCTrans * ctrans, const char *id);

/**
 * bot_ctrans_get_trans:
 *
 * Retrieves the rigid body transformation relating two coordinate frames at
 * the specified time.  The transformation is calculated by interpolating
 * the nearest transformations in time.
 *
 *
 * Returns: 1 on success, 0 on failure
 */
int bot_ctrans_get_trans(BotCTrans *ctrans, const char *from_frame,
        const char *to_frame, int64_t timestamp, BotTrans *result);

/**
 * bot_ctrans_get_trans_latest:
 *
 * Retrieves the most recent rigid body transformation relating two coordinate
 * frames.
 *
 * Returns: 1 on success, 0 on failure
 */
int bot_ctrans_get_trans_latest(BotCTrans *ctrans, const char *from_frame,
        const char *to_frame, BotTrans *result);

/**
 * bot_ctrans_have_trans:
 *
 * Returns: 1 if the specified transformation is available, 0 if not.
 */
int bot_ctrans_have_trans(BotCTrans *ctrans, const char *from_frame,
        const char *to_frame);

/**
 * bot_ctrans_get_trans_latest_timestamp:
 * @ctrans: The CTrans object
 * @from_frame: source coordinate frame
 * @to_frame: destination coordinate frame
 * @timestamp: output parameter
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
int bot_ctrans_get_trans_latest_timestamp(BotCTrans *ctrans, 
        const char *from_frame, const char *to_frame, int64_t *timestamp);

/**
 * bot_ctrans_link_frames:
 *
 * Creates an link between two coordinate frames to indicate the presence of a
 * rigid-body transformation that directly relates the two frames.  The
 * resulting link can then be used to update the transformation.
 *
 * Returns: the BotCTransLink that represents the transformation from
 * %from_frame_id to %to_frame_id, or NULL if either of the coordinate frames
 * is invalid.
 */
BotCTransLink * bot_ctrans_link_frames(BotCTrans * ctrans, 
        const char *from_frame_id, const char *to_frame_id, 
        int history_maxlen);

/**
 * bot_ctrans_get_link:
 *
 * Searches for and retrieves the BotCTransLink that represents the
 * rigid body transformation that directly relates two coordinate frames.
 *
 * The link must have previously been created by a call to
 * bot_ctrans_link_frames.  If you just want the latest rigid-body
 * transformation relating the two frames, then use bot_ctrans_get_trans or
 * bot_ctrans_get_trans_latest.
 *
 * Returns: the BotCTransLink that represents the link between the two
 * coordinate frames, or NULL if there is no such link.
 */
BotCTransLink * bot_ctrans_get_link(BotCTrans * ctrans,
        const char *frame_a_id, const char * frame_b_id);

/**
 * bot_ctrans_link_update:
 * @link: The link to update
 * @transformation: The rigid body transformation describing the link
 * @timestamp:  timestamp of the transformation. 
 *
 * Updates the link between two coordinate frames.  If the specified timestamp
 * is older than the timestamp for any previous update, then the entire
 * transformation history is discarded before saving the update.
 */
void bot_ctrans_link_update(BotCTransLink * link,
        const BotTrans *transformation, int64_t timestamp);

/**
 * bot_ctrans_link_get_n_trans:
 *
 * Returns: the number of transformations stored for this link.
 */
int bot_ctrans_link_get_n_trans(const BotCTransLink * link);

/**
 * bot_ctrans_link_get_nth_trans:
 * @index: transformation index.  0 is most recent, higher indices correspond
 *         to older transformations.
 * @transformation: Output parameter.  If not NULL, the desired transformation
 *                  will be stored here.
 * @timestamp: Output parameter.  If not NULL, the timestamp corresponding to 
 *             the desired transformattion will be stored here.
 *
 * Retrieves the nth most recent transformation for this link.
 *
 * Returns: 1 on success, 0 if the requested transformation is not available.
 */
int bot_ctrans_link_get_nth_trans(BotCTransLink * link,
        int index, BotTrans *transformation, int64_t *timestamp);

const char * bot_ctrans_link_get_from_frame(BotCTransLink *link);
const char * bot_ctrans_link_get_to_frame(BotCTransLink *link);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
