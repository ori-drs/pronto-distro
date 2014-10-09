#include <stdio.h>

#include <glib.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <lcm/lcm.h>
#include "bot_frames.h"

#include <bot_param/param_util.h>
#include <lcmtypes/bot_frames_update_t.h>

#define BOT_FRAMES_UPDATE_CHANNEL "BOT_FRAMES_UPDATE"
#define DEFAULT_HISTORY_LEN 100

typedef struct {
  int frame_num;
  char * frame_name;
  char * relative_to;
  char * update_channel;
  bot_core_rigid_transform_t_subscription_t * transform_subscription;
  bot_core_pose_t_subscription_t * pose_subscription;

  BotCTransLink * ctrans_link;
  int was_updated;
} frame_handle_t;

typedef struct {
  bot_frames_link_update_handler_t * callback_func;
  void * user;
} update_handler_t;

static void frame_handle_destroy(lcm_t * lcm, frame_handle_t * fh)
{
  if (fh->frame_name != NULL)
    free(fh->frame_name);
  if (fh->relative_to != NULL)
    free(fh->relative_to);
  if (fh->update_channel != NULL)
    free(fh->update_channel);
  if (fh->transform_subscription != NULL)
    bot_core_rigid_transform_t_unsubscribe(lcm, fh->transform_subscription);
  if (fh->pose_subscription != NULL)
    bot_core_pose_t_unsubscribe(lcm, fh->pose_subscription);
  free(fh);
}

struct _BotFrames {
  BotCTrans * ctrans;
  lcm_t *lcm;
  BotParam *bot_param;

  GMutex * mutex;
  int num_frames;
  char * root_name;
  GHashTable* frame_handles_by_name;
  GHashTable* frame_handles_by_channel;

  bot_frames_update_t_subscription_t * update_subscription;
  GList * update_callbacks;

};

static void _dispatch_update_callbacks(BotFrames * bot_frames,const char * frame_name, const char * relative_to,
    int64_t utime)
{
  GList * p = bot_frames->update_callbacks;
  for ( ; p != NULL; p = g_list_next(p)) {
    update_handler_t * uh = (update_handler_t *) p->data;
    uh->callback_func(bot_frames, frame_name, relative_to,utime, uh->user);
  }
}

static void on_transform_update(const lcm_recv_buf_t *rbuf, const char *channel, const bot_core_rigid_transform_t *msg,
    void *user_data)
{
  BotFrames * bot_frames = (BotFrames *) user_data;
  g_mutex_lock(bot_frames->mutex);
  BotTrans link_transf;
  bot_trans_set_from_quat_trans(&link_transf, msg->quat, msg->trans);

  frame_handle_t * frame_handle = (frame_handle_t *) g_hash_table_lookup(bot_frames->frame_handles_by_channel, channel);
  assert(frame_handle != NULL);
  frame_handle->was_updated = 1;
  bot_ctrans_link_update(frame_handle->ctrans_link, &link_transf, msg->utime);
  g_mutex_unlock(bot_frames->mutex);

  _dispatch_update_callbacks(bot_frames, frame_handle->frame_name, frame_handle->relative_to,msg->utime);

}

static void on_pose_update(const lcm_recv_buf_t *rbuf, const char *channel, const bot_core_pose_t *msg,
    void *user_data)
{
  BotFrames * bot_frames = (BotFrames *) user_data;
  g_mutex_lock(bot_frames->mutex);
  BotTrans link_transf;
  bot_trans_set_from_quat_trans(&link_transf, msg->orientation, msg->pos);

  frame_handle_t * frame_handle = (frame_handle_t *) g_hash_table_lookup(bot_frames->frame_handles_by_channel, channel);
  assert(frame_handle != NULL);
  frame_handle->was_updated = 1;
  bot_ctrans_link_update(frame_handle->ctrans_link, &link_transf, msg->utime);
  g_mutex_unlock(bot_frames->mutex);

  _dispatch_update_callbacks(bot_frames, frame_handle->frame_name, frame_handle->relative_to,msg->utime);

}

static void on_frames_update(const lcm_recv_buf_t *rbuf, const char *channel, const bot_frames_update_t *msg,
    void *user_data)
{
  BotFrames * bot_frames = (BotFrames *) user_data;
  g_mutex_lock(bot_frames->mutex);
  BotTrans link_transf;
  bot_trans_set_from_quat_trans(&link_transf, msg->quat, msg->trans);

  frame_handle_t * frame_handle = (frame_handle_t *) g_hash_table_lookup(bot_frames->frame_handles_by_name, msg->frame);
  if (frame_handle == NULL) {
    fprintf(stderr, "Received frame update for unknown frame, adding link %s->%s to BotFrames\n", msg->frame, msg->relative_to);
    frame_handle = (frame_handle_t *) calloc(1, sizeof(frame_handle_t));
    frame_handle->ctrans_link = bot_ctrans_link_frames(bot_frames->ctrans, msg->frame, msg->relative_to, DEFAULT_HISTORY_LEN);
    bot_ctrans_link_update(frame_handle->ctrans_link, &link_transf, msg->utime);
    frame_handle->was_updated = 1;
    frame_handle->frame_name = strdup(msg->frame);
    frame_handle->relative_to = strdup(msg->relative_to);
    g_hash_table_insert(bot_frames->frame_handles_by_name, (gpointer) frame_handle->frame_name, (gpointer) frame_handle);
  }
  else if(strcmp(msg->relative_to, frame_handle->relative_to) == 0){
    //update the existing frame
    frame_handle->was_updated = 1;
    bot_ctrans_link_update(frame_handle->ctrans_link, &link_transf, msg->utime);
  }
  else {
    //invalid update TODO: rate limit spewing? probably not worth it
    fprintf(stderr, "Ignoring link update %s->%s, frame was constructed relative to %s\n", msg->frame,
        msg->relative_to, frame_handle->relative_to);
  }
  g_mutex_unlock(bot_frames->mutex);

  _dispatch_update_callbacks(bot_frames, frame_handle->frame_name, frame_handle->relative_to, msg->utime);
}


BotFrames *
bot_frames_new(lcm_t *lcm, BotParam *bot_param)
{
  BotFrames *self = g_slice_new0(BotFrames);

  self->lcm = lcm;
  self->bot_param = bot_param;
  self->mutex = g_mutex_new();
  self->num_frames =0;
  g_mutex_lock(self->mutex);
  // setup the coordinate frame graph
  self->ctrans = bot_ctrans_new();

  //allocate frame handles
  self->frame_handles_by_name = g_hash_table_new(g_str_hash, g_str_equal);
  self->frame_handles_by_channel = g_hash_table_new(g_str_hash, g_str_equal);

  //create the callback lists
  self->update_callbacks = NULL;

  int num_frames = bot_param_get_num_subkeys(self->bot_param, "coordinate_frames");
  if (num_frames <= 0) {
    fprintf(stderr, "BotFrames Error: param file does not contain a 'coordinate_frames' block\n");
    bot_frames_destroy(self);
    return NULL;
  }

  int ret = bot_param_get_str(self->bot_param, "coordinate_frames.root_frame", &self->root_name);
  if (ret < 0) {
    fprintf(stderr, "BotFrames Error: root_frame not defined!\n");
    goto fail;
  }
  bot_ctrans_add_frame(self->ctrans, self->root_name);

  //create a frame_handle for the root frame
  frame_handle_t * root_handle = (frame_handle_t *) calloc(1, sizeof(frame_handle_t));
  root_handle->frame_name = strdup(self->root_name);
  root_handle->frame_num =self->num_frames++;
  g_hash_table_insert(self->frame_handles_by_name, (gpointer) self->root_name, (gpointer) root_handle);

  char ** frame_names = bot_param_get_subkeys(self->bot_param, "coordinate_frames");

  for (int i = 0; i < num_frames; i++) {
    char * frame_name = strdup(frame_names[i]);
    //add this frame to ctrans
    bot_ctrans_add_frame(self->ctrans, frame_name);

    char param_key[2048];
    sprintf(param_key, "coordinate_frames.%s", frame_name);
    int num_sub_keys = bot_param_get_num_subkeys(self->bot_param, param_key);
    if (num_sub_keys == 0) {
      continue; // probably the root_frame definition
    }
    //setup the link parameters if this isn't a root frame

    //get which frame this is relative to
    sprintf(param_key, "coordinate_frames.%s.relative_to", frame_name);
    char * relative_to;
    int ret = bot_param_get_str(self->bot_param, param_key, &relative_to);
    if (ret < 0) {
      fprintf(stderr, "BotFrames Error: frame %s does not have a 'relative_to' field block\n", frame_name);
      goto fail;
    }

    //get the history size
    sprintf(param_key, "coordinate_frames.%s.history", frame_name);
    int history;
    ret = bot_param_get_int(self->bot_param, param_key, &history);
    if (ret < 0) {
      history = DEFAULT_HISTORY_LEN;
    }

    //get the initial transform
    sprintf(param_key, "coordinate_frames.%s.initial_transform", frame_name);
    if (bot_param_get_num_subkeys(self->bot_param, param_key) != 2) {
      fprintf(stderr,
          "BotFrames Error: frame %s does not have the right number of fields in the 'initial_transform' block\n",
          frame_name);
      goto fail;
    }
    BotTrans init_trans;
    ret = bot_param_get_trans(self->bot_param, param_key, &init_trans);
    if (ret < 0) {
      fprintf(stderr, "BotFrames Error: could not get 'initial_transform' for frame %s\n", frame_name);
      goto fail;
    }

    //create and initialize the link
    BotCTransLink *link = bot_ctrans_link_frames(self->ctrans, frame_name, relative_to, history + 1);
    bot_ctrans_link_update(link, &init_trans, 0);

    //add the frame to the hash table
    frame_handle_t * frame_handle = (frame_handle_t *) g_hash_table_lookup(self->frame_handles_by_name, frame_name);
    if (frame_handle != NULL) {
      fprintf(stderr, "BotFrames Error: frame %s duplicated\n", frame_name);
      goto fail;
    }

    //create the frame_handle
    frame_handle = (frame_handle_t *) calloc(1, sizeof(frame_handle_t));
    frame_handle->frame_num = self->num_frames++;
    frame_handle->ctrans_link = link;
    frame_handle->frame_name = frame_name;
    frame_handle->relative_to = relative_to;
    g_hash_table_insert(self->frame_handles_by_name, (gpointer) frame_name, (gpointer) frame_handle);

    //get the update channel
    char * update_channel = NULL;
    sprintf(param_key, "coordinate_frames.%s.update_channel", frame_name);
    ret = bot_param_get_str(self->bot_param, param_key, &update_channel);
    int pose_update_chan = 0;
    if (ret < 0) {
      sprintf(param_key, "coordinate_frames.%s.pose_update_channel", frame_name);
      ret = bot_param_get_str(self->bot_param, param_key, &update_channel);
      if (ret == 0) {
        pose_update_chan=1;
      }
    }

    //add the entry to the update_channel hash table
    if (update_channel!=NULL){
    frame_handle_t * entry = (frame_handle_t *) g_hash_table_lookup(self->frame_handles_by_channel, update_channel);
    if (entry != NULL) {
      fprintf(stderr, "BotFrames Error: update_channel %s for frame %s already used for frame %s\n", update_channel,
          frame_name, entry->frame_name);
      goto fail;
    }
    //first time around, allocate and set the timer goin...
    frame_handle->update_channel = update_channel;
    if (!pose_update_chan){
      frame_handle->transform_subscription = bot_core_rigid_transform_t_subscribe(self->lcm, update_channel, on_transform_update,
          (void*) self);
    }
    else{
      frame_handle->pose_subscription = bot_core_pose_t_subscribe(self->lcm, update_channel, on_pose_update,
                (void*) self);
    }
    g_hash_table_insert(self->frame_handles_by_channel, (gpointer) update_channel, (gpointer) frame_handle);
    }


  }

  //subscribe to the default update handler
  self->update_subscription = bot_frames_update_t_subscribe(self->lcm, BOT_FRAMES_UPDATE_CHANNEL, on_frames_update, (void*) self);

  g_strfreev(frame_names);
  g_mutex_unlock(self->mutex);
  return self;

  fail: g_mutex_unlock(self->mutex);
  bot_frames_destroy(self);
  return NULL;

}

static void _update_handler_t_destroy(void * data, void * user)
{
  g_slice_free(update_handler_t, data);
}

void bot_frames_destroy(BotFrames * bot_frames)
{

  g_mutex_lock(bot_frames->mutex);

  bot_ctrans_destroy(bot_frames->ctrans);
  if(bot_frames->update_subscription!=NULL)
    bot_frames_update_t_unsubscribe(bot_frames->lcm,bot_frames->update_subscription);

  GHashTableIter iter;
  gpointer key, value;
  g_hash_table_iter_init(&iter, bot_frames->frame_handles_by_name);
  int frame_num;
  while (g_hash_table_iter_next(&iter, &key, &value)) {
    frame_handle_t * han = (frame_handle_t *) value;
    frame_handle_destroy(bot_frames->lcm, han);
  }
  g_hash_table_destroy(bot_frames->frame_handles_by_name);
  g_hash_table_destroy(bot_frames->frame_handles_by_channel);
  free(bot_frames->root_name);

  if (bot_frames->update_callbacks != NULL) {
    g_list_foreach(bot_frames->update_callbacks, _update_handler_t_destroy, NULL);
    g_list_free(bot_frames->update_callbacks);
  }
  g_mutex_unlock(bot_frames->mutex);
  g_mutex_free(bot_frames->mutex);
  g_slice_free(BotFrames, bot_frames);
}

void bot_frames_update_frame(BotFrames * bot_frames, const char * frame_name, const char * relative_to,
    const BotTrans * trans, int64_t utime)
{
  bot_frames_update_t msg;
  msg.frame = (char *)frame_name;
  msg.relative_to = (char *)relative_to;
  msg.utime = utime;
  memcpy(msg.trans, trans->trans_vec, 3 * sizeof(double));
  memcpy(msg.quat, trans->rot_quat, 4 * sizeof(double));
  bot_frames_update_t_publish(bot_frames->lcm, BOT_FRAMES_UPDATE_CHANNEL, &msg); //lcm object is threadsafe
}

void bot_frames_add_update_subscriber(BotFrames *bot_frames, bot_frames_link_update_handler_t * callback_func,
    void * user)
{
  update_handler_t * uh = g_slice_new0(update_handler_t);
  uh->callback_func = callback_func;
  uh->user = user;
  g_mutex_lock(bot_frames->mutex);
  bot_frames->update_callbacks = g_list_append(bot_frames->update_callbacks, uh);
  g_mutex_unlock(bot_frames->mutex);

}

int bot_frames_get_latest_timestamp(BotFrames * bot_frames, 
                                    const char *from_frame, const char *to_frame, int64_t *timestamp){

    g_mutex_lock(bot_frames->mutex);
    int status = bot_ctrans_get_trans_latest_timestamp(bot_frames->ctrans, from_frame, to_frame, timestamp);
    g_mutex_unlock(bot_frames->mutex);
    return status;
}

int bot_frames_get_trans_with_utime(BotFrames *bot_frames, const char *from_frame, const char *to_frame, int64_t utime,
    BotTrans *result)
{
  g_mutex_lock(bot_frames->mutex);
  int status = bot_ctrans_get_trans(bot_frames->ctrans, from_frame, to_frame, utime, result);
  g_mutex_unlock(bot_frames->mutex);
  return status;
}

int bot_frames_get_trans(BotFrames *bot_frames, const char *from_frame, const char *to_frame, BotTrans *result)
{
  g_mutex_lock(bot_frames->mutex);
  int status = bot_ctrans_get_trans_latest(bot_frames->ctrans, from_frame, to_frame, result);
  g_mutex_unlock(bot_frames->mutex);
  return status;
}

int bot_frames_get_trans_mat_3x4(BotFrames *bot_frames, const char *from_frame, const char *to_frame, double mat[12])
{
  BotTrans bt;
  if (!bot_frames_get_trans(bot_frames, from_frame, to_frame, &bt))
    return 0;
  bot_trans_get_mat_3x4(&bt, mat);
  return 1;
}

int bot_frames_get_trans_mat_3x4_with_utime(BotFrames *bot_frames, const char *from_frame, const char *to_frame,
    int64_t utime, double mat[12])
{
  BotTrans bt;
  if (!bot_frames_get_trans_with_utime(bot_frames, from_frame, to_frame, utime, &bt))
    return 0;
  bot_trans_get_mat_3x4(&bt, mat);
  return 1;
}

int bot_frames_get_trans_mat_4x4(BotFrames *bot_frames, const char *from_frame, const char *to_frame, double mat[12])
{
  BotTrans bt;
  if (!bot_frames_get_trans(bot_frames, from_frame, to_frame, &bt))
    return 0;
  bot_trans_get_mat_4x4(&bt, mat);
  return 1;
}

int bot_frames_get_trans_mat_4x4_with_utime(BotFrames *bot_frames, const char *from_frame, const char *to_frame,
    int64_t utime, double mat[12])
{
  BotTrans bt;
  if (!bot_frames_get_trans_with_utime(bot_frames, from_frame, to_frame, utime, &bt))
    return 0;
  bot_trans_get_mat_4x4(&bt, mat);
  return 1;
}

int bot_frames_get_trans_latest_timestamp(BotFrames *bot_frames, const char *from_frame, const char *to_frame,
    int64_t *timestamp)
{
  g_mutex_lock(bot_frames->mutex);
  int status = bot_ctrans_get_trans_latest_timestamp(bot_frames->ctrans, from_frame, to_frame, timestamp);
  g_mutex_unlock(bot_frames->mutex);
  return status;
}

int bot_frames_have_trans(BotFrames *bot_frames, const char *from_frame, const char *to_frame)
{
  g_mutex_lock(bot_frames->mutex);
  int status = bot_ctrans_have_trans(bot_frames->ctrans, from_frame, to_frame);
  g_mutex_unlock(bot_frames->mutex);
  return status;
}

int bot_frames_transform_vec(BotFrames *bot_frames, const char *from_frame, const char *to_frame, const double src[3],
    double dst[3])
{
  BotTrans rbtrans;
  if (!bot_frames_get_trans(bot_frames, from_frame, to_frame, &rbtrans))
    return 0;
  bot_trans_apply_vec(&rbtrans, src, dst);
  return 1;
}

int bot_frames_rotate_vec(BotFrames *bot_frames, const char *from_frame, const char *to_frame, const double src[3],
    double dst[3])
{
  BotTrans rbtrans;
  if (!bot_frames_get_trans(bot_frames, from_frame, to_frame, &rbtrans))
    return 0;
  bot_trans_rotate_vec(&rbtrans, src, dst);
  return 1;
}

int bot_frames_get_n_trans(BotFrames *bot_frames, const char *from_frame, const char *to_frame, int nth_from_latest)
{
  g_mutex_lock(bot_frames->mutex);
  BotCTransLink *link = bot_ctrans_get_link(bot_frames->ctrans, from_frame, to_frame);
  int n_trans;
  if (!link)
    n_trans = 0;
  else
    n_trans = bot_ctrans_link_get_n_trans(link);
  g_mutex_unlock(bot_frames->mutex);

  return n_trans;
}

/**
 * Returns: 1 on success, 0 on failure
 */
int bot_frames_get_nth_trans(BotFrames *bot_frames, const char *from_frame, const char *to_frame, int nth_from_latest,
    BotTrans *btrans, int64_t *timestamp)
{
  g_mutex_lock(bot_frames->mutex);
  BotCTransLink *link = bot_ctrans_get_link(bot_frames->ctrans, from_frame, to_frame);
  int status;
  if (!link)
    status =0;
  else{
    int status = bot_ctrans_link_get_nth_trans(link, nth_from_latest, btrans, timestamp);
    if (status && btrans && 0 != strcmp(to_frame, bot_ctrans_link_get_to_frame(link))) {
      bot_trans_invert(btrans);
    }
  }
  g_mutex_unlock(bot_frames->mutex);
  return status;
}
const char * bot_frames_get_relative_to(BotFrames * bot_frames, const char * frame_name)
{
  const char * rel_to = NULL;
  g_mutex_lock(bot_frames->mutex);
  //get a reference to the frame_handle
  frame_handle_t * frame_handle = (frame_handle_t *) g_hash_table_lookup(bot_frames->frame_handles_by_name, frame_name);
  //check to see if it was successful
  if (frame_handle != NULL)
    rel_to = frame_handle->relative_to;
  g_mutex_unlock(bot_frames->mutex);
  return rel_to;
}

const char * bot_frames_get_root_name(BotFrames * bot_frames)
{
  return bot_frames->root_name; //root frame is read-only
}

int bot_frames_get_num_frames(BotFrames * bot_frames)
{
  g_mutex_lock(bot_frames->mutex);
  int num_frames = bot_frames->num_frames;
  g_mutex_unlock(bot_frames->mutex);
  return num_frames;
}

char ** bot_frames_get_frame_names(BotFrames * bot_frames)
{
  int num_frames = bot_frames_get_num_frames(bot_frames);

  g_mutex_lock(bot_frames->mutex);
  char ** frames = calloc(num_frames + 1, sizeof(char*));

  GHashTableIter iter;
  gpointer key, value;

  g_hash_table_iter_init(&iter, bot_frames->frame_handles_by_name);
  while (g_hash_table_iter_next(&iter, &key, &value)) {
    frame_handle_t * han = (frame_handle_t *) value;
    frames[han->frame_num] = strdup(han->frame_name);
  }
  g_mutex_unlock(bot_frames->mutex);
  return frames;
}

static BotFrames *global_bot_frames = NULL;
static GStaticMutex bot_frames_global_mutex = G_STATIC_MUTEX_INIT;

BotFrames*
bot_frames_get_global(lcm_t *lcm, BotParam *bot_param)
{
  if (lcm == NULL)
    lcm = bot_lcm_get_global(NULL);
  if (bot_param == NULL)
    bot_param = bot_param_get_global(lcm, 0);

  g_static_mutex_lock(&bot_frames_global_mutex);
  if (global_bot_frames == NULL) {
    global_bot_frames = bot_frames_new(lcm, bot_param);
    if (!global_bot_frames)
      goto fail;
  }

  BotFrames *result = global_bot_frames;
  g_static_mutex_unlock(&bot_frames_global_mutex);
  return result;

  fail: g_static_mutex_unlock(&bot_frames_global_mutex);
  fprintf(stderr, "ERROR: Could not get global bot_frames!\n");
  return NULL;
}
