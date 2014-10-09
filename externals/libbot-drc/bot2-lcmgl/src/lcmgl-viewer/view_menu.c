#include <gtk/gtk.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>

void setup_view_menu(BotViewer *viewer);

static void
on_setview_y_up_item(GtkMenuItem *mi, void *user)
{
    BotViewer *viewer = (BotViewer*)user;
    double eye[] = { 0, 0, 50 };
    double lookat[] = { 0, 0, 0 };
    double up[] = { 0, 1, 0 };
    if(viewer->view_handler)
        viewer->view_handler->set_look_at(viewer->view_handler, eye, lookat, up);
}

static void
on_setview_x_up_item(GtkMenuItem *mi, void *user)
{
    BotViewer *viewer = (BotViewer*)user;
    double eye[] = { 0, 0, 50 };
    double lookat[] = { 0, 0, 0 };
    double up[] = { 1, 0, 0 };
    if(viewer->view_handler)
        viewer->view_handler->set_look_at(viewer->view_handler, eye, lookat, up);
}

void
setup_view_menu(BotViewer *viewer)
{
    GtkWidget *view_menu = viewer->view_menu;

    // bookmarks

    gtk_menu_append(GTK_MENU(view_menu), gtk_separator_menu_item_new());

    // predefined viewpoints
    GtkWidget *setview_y_up_item = gtk_menu_item_new_with_mnemonic("Y axis up");
    gtk_menu_append(GTK_MENU(view_menu), setview_y_up_item);
    g_signal_connect(G_OBJECT(setview_y_up_item), "activate", 
            G_CALLBACK(on_setview_y_up_item), viewer);

    // predefined viewpoints
    GtkWidget *setview_x_up_item = gtk_menu_item_new_with_mnemonic("X axis up");
    gtk_menu_append(GTK_MENU(view_menu), setview_x_up_item);
    g_signal_connect(G_OBJECT(setview_x_up_item), "activate", 
            G_CALLBACK(on_setview_x_up_item), viewer);

    gtk_widget_show_all(view_menu);
}
