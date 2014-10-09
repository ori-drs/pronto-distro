#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <lcmtypes/bot_lcmgl_data_t.h>

#include "lcmgl_decode.h"
#include "lcmgl_bot_renderer.h"

typedef struct
{
    GPtrArray *backbuffer;
    GPtrArray *frontbuffer;
    int enabled;
} lcmgl_channel_t;

typedef struct _BotLcmglRenderer {
    BotRenderer renderer;
    BotGtkParamWidget *pw;
    BotViewer   *viewer;
    lcm_t     *lcm;

    GHashTable *channels;

} BotLcmglRenderer;

static void my_free( BotRenderer *renderer )
{
    BotLcmglRenderer *self = (BotLcmglRenderer*) renderer;

    free( self );
}

static void my_draw( BotViewer *viewer, BotRenderer *renderer )
{
    //Enable transparency
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);

    BotLcmglRenderer *self = (BotLcmglRenderer*) renderer->user;

    // iterate over each channel
    GList *keys = bot_g_hash_table_get_keys(self->channels);

    for (GList *kiter = keys; kiter; kiter=kiter->next) {
        lcmgl_channel_t *chan = g_hash_table_lookup(self->channels,
                                                   kiter->data);
        glPushMatrix();
        glPushAttrib(GL_ENABLE_BIT);

        if (chan->enabled) {
            // iterate over all the messages received for this channel
            for (int i = 0; i < chan->frontbuffer->len; i++) {
                bot_lcmgl_data_t *data =
                    g_ptr_array_index(chan->frontbuffer, i);

                bot_lcmgl_decode(data->data, data->datalen);
            }
        }
        glPopAttrib ();
        glPopMatrix();
    }
    g_list_free (keys);
}

static void on_lcmgl_data (const lcm_recv_buf_t *rbuf, const char *channel,
        const bot_lcmgl_data_t *_msg, void *user_data )
{
    BotLcmglRenderer *self = (BotLcmglRenderer*) user_data;

    lcmgl_channel_t *chan = g_hash_table_lookup(self->channels, _msg->name);

    if (!chan) {
        chan = (lcmgl_channel_t*) calloc(1, sizeof(lcmgl_channel_t));
        chan->enabled=1;
        //chan->backbuffer = g_ptr_array_new();
        chan->frontbuffer = g_ptr_array_new();
        g_hash_table_insert(self->channels, strdup(_msg->name), chan);
        bot_gtk_param_widget_add_booleans (self->pw,
                0, strdup(_msg->name), 1, NULL);
    }

#if 0
    int current_scene = -1;
    if (chan->backbuffer->len > 0) {
        bot_lcmgl_data_t *ld = g_ptr_array_index(chan->backbuffer, 0);
        current_scene = ld->scene;
    }

    // new scene?
    if (current_scene != _msg->scene) {

        // free objects in foreground buffer
        for (int i = 0; i < chan->frontbuffer->len; i++)
            bot_lcmgl_data_t_destroy(g_ptr_array_index(chan->frontbuffer, i));
        g_ptr_array_set_size(chan->frontbuffer, 0);

        // swap front and back buffers
        GPtrArray *tmp = chan->backbuffer;
        chan->backbuffer = chan->frontbuffer;
        chan->frontbuffer = tmp;

        bot_viewer_request_redraw( self->viewer );
    }
#endif

    for (int i = 0; i < chan->frontbuffer->len; i++)
        bot_lcmgl_data_t_destroy(g_ptr_array_index(chan->frontbuffer, i));
    g_ptr_array_set_size (chan->frontbuffer, 0);
    g_ptr_array_add(chan->frontbuffer, bot_lcmgl_data_t_copy(_msg));
    bot_viewer_request_redraw( self->viewer );
}

static void on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    BotLcmglRenderer *self = (BotLcmglRenderer*) user;

    // iterate over each channel
    GList *keys = bot_g_hash_table_get_keys(self->channels);

    for (GList *kiter=keys; kiter; kiter=kiter->next) {
        lcmgl_channel_t *chan = g_hash_table_lookup(self->channels,
                                                   kiter->data);

        chan->enabled = bot_gtk_param_widget_get_bool (pw, kiter->data);
    }
    g_list_free (keys);

    bot_viewer_request_redraw(self->viewer);
}

static void on_clear_button(GtkWidget *button, BotLcmglRenderer *self)
{
    if (!self->viewer)
        return;

    // iterate over each channel
    GList *keys = bot_g_hash_table_get_keys(self->channels);
    for (GList *kiter = keys; kiter; kiter = kiter->next) {
        lcmgl_channel_t *chan = g_hash_table_lookup(self->channels, kiter->data);
        // iterate over all the messages received for this channel
        for (int i = 0; i < chan->frontbuffer->len; i++)
            bot_lcmgl_data_t_destroy(g_ptr_array_index(chan->frontbuffer, i));
        g_ptr_array_set_size(chan->frontbuffer, 0);

    }
    g_list_free(keys);

    bot_viewer_request_redraw(self->viewer);
}

void 
bot_lcmgl_add_renderer_to_viewer(BotViewer* viewer, lcm_t* lcm, int priority)
{
    BotLcmglRenderer *self =
        (BotLcmglRenderer*) calloc(1, sizeof(BotLcmglRenderer));

    BotRenderer *renderer = &self->renderer;

    self->lcm = lcm;
    self->viewer = viewer;
    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    renderer->draw = my_draw;
    renderer->destroy = my_free;
    renderer->name = "LCM GL";
    renderer->widget = GTK_WIDGET(self->pw);
    renderer->enabled = 1;
    renderer->user = self;

    self->channels = g_hash_table_new(g_str_hash, g_str_equal);

    g_signal_connect (G_OBJECT (self->pw), "changed",
                      G_CALLBACK (on_param_widget_changed), self);

    bot_lcmgl_data_t_subscribe(self->lcm, "LCMGL.*", on_lcmgl_data, self);

    bot_viewer_add_renderer(viewer, renderer, priority);

    GtkWidget *clear_button = gtk_button_new_with_label("Clear All");
    gtk_box_pack_start(GTK_BOX(renderer->widget), clear_button, FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(clear_button), "clicked", G_CALLBACK(on_clear_button), self);
    gtk_widget_show_all(renderer->widget);
}

/*
 * setup_renderer:
 * Generic renderer add function for use as a dynamically loaded plugin
 */
void add_renderer_to_plugin_viewer(BotViewer *viewer, int render_priority){
  lcm_t * lcm = bot_lcm_get_global(NULL);
  bot_lcmgl_add_renderer_to_viewer(viewer,lcm, render_priority);
}
