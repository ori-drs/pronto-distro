#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <glib.h>
#include <glib-object.h>
#include <gtk/gtk.h>
#include <gtk/gtksignal.h>

#include "param_widget.h"

//#define dbg(args...) fprintf(stderr, args)
#define dbg(args...)
#define err(args...) fprintf(stderr, args)

struct _BotGtkParamWidget
{
    GtkVBox vbox;

    /* private */
    GHashTable *params;
    GHashTable *widget_to_param;

    GList *widgets;

    char *estr;
};

struct _BotGtkParamWidgetClass
{
    GtkVBoxClass parent_class;
};

typedef struct _param_data {
    char *name;
    GtkWidget *widget;

    GType data_type;

    double min_double;
    double max_double;
} param_data_t;

static param_data_t * 
param_data_new(const char *name, GtkWidget *widget, GType data_type)
{
    param_data_t *pdata = g_slice_new(param_data_t);
    pdata->name = strdup(name);
    pdata->widget = widget;
    pdata->data_type = data_type;
    return pdata;
}

static void
param_data_free(param_data_t *pdata)
{
    free(pdata->name);
    g_slice_free(param_data_t, pdata);
}

enum {
    CHANGED_SIGNAL,
    LAST_SIGNAL
};

static void bot_gtk_param_widget_class_init (BotGtkParamWidgetClass *klass);
static void bot_gtk_param_widget_init (BotGtkParamWidget *pw);
static void bot_gtk_param_widget_finalize (GObject *obj);

static guint bot_gtk_param_widget_signals[LAST_SIGNAL] = { 0 };

G_DEFINE_TYPE (BotGtkParamWidget, bot_gtk_param_widget, GTK_TYPE_VBOX);

static void
bot_gtk_param_widget_class_init (BotGtkParamWidgetClass *klass)
{
    GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
    gobject_class->finalize = bot_gtk_param_widget_finalize;

    bot_gtk_param_widget_signals[CHANGED_SIGNAL] = g_signal_new("changed",
            G_TYPE_FROM_CLASS(klass),
            G_SIGNAL_RUN_FIRST | G_SIGNAL_ACTION,
            0,
            NULL,
            NULL,
            gtk_marshal_VOID__STRING,
            G_TYPE_NONE, 1, G_TYPE_STRING);
}

static void
bot_gtk_param_widget_init (BotGtkParamWidget *pw)
{
    dbg ("%s:%d %s\n", __FILE__, __LINE__, __FUNCTION__);

    pw->widgets = NULL;
    pw->params = g_hash_table_new(g_str_hash, g_str_equal);
    pw->widget_to_param = g_hash_table_new_full(g_direct_hash, g_direct_equal,
            NULL, (GDestroyNotify)param_data_free);
    pw->estr = NULL;
}

static void
bot_gtk_param_widget_finalize (GObject *obj)
{
    BotGtkParamWidget *pw = BOT_GTK_PARAM_WIDGET (obj);
    dbg ("%s:%d %s\n", __FILE__, __LINE__, __FUNCTION__);

    if (pw->estr) {
        free (pw->estr);
        pw->estr = NULL;
    }

    G_OBJECT_CLASS (bot_gtk_param_widget_parent_class)->finalize(obj);
}

GtkWidget *
bot_gtk_param_widget_new (void)
{
    return GTK_WIDGET (g_object_new (BOT_GTK_TYPE_PARAM_WIDGET, NULL));
}

static inline param_data_t * 
get_param_data(BotGtkParamWidget * pw, const char *name)
{
    GtkWidget *w = g_hash_table_lookup (pw->params, name);
    if(!w)
        return NULL;
    param_data_t *pd = 
        (param_data_t*) g_hash_table_lookup (pw->widget_to_param, w);
    return pd;
}

static inline param_data_t * 
get_param_data_check_type(BotGtkParamWidget * pw, const char *name, GType data_type)
{
    GtkWidget *w = g_hash_table_lookup (pw->params, name);
    if(!w)
        return NULL;
    param_data_t *pd = 
        (param_data_t*) g_hash_table_lookup (pw->widget_to_param, w);
    if(pd->data_type != data_type) {
        g_warning("bot_gtk_param_widget:  param [%s] is not type %s\n",
                name, g_type_name(data_type));
        return NULL;
    }
    return pd;
}

static int
have_parameter_key (BotGtkParamWidget *pw, const char *name)
{
    return g_hash_table_lookup (pw->params, name) != NULL;
}

static void
generic_widget_changed (GtkWidget *w, gpointer user_data)
{
    BotGtkParamWidget *pw = BOT_GTK_PARAM_WIDGET(user_data);
    param_data_t *pd = 
        (param_data_t*) g_hash_table_lookup (pw->widget_to_param, w);
    g_signal_emit (G_OBJECT (pw), 
            bot_gtk_param_widget_signals[CHANGED_SIGNAL], 0, pd->name);
}

static param_data_t * 
add_row (BotGtkParamWidget *pw, const char *name,
        GtkWidget *w, const char *signal_name, GType data_type)
{
    GtkWidget *hb = gtk_hbox_new (FALSE, 0);
    GtkWidget *lbl = gtk_label_new (name);
    gtk_box_pack_start (GTK_BOX (hb), lbl, FALSE, FALSE, 0);
    gtk_box_pack_start (GTK_BOX (hb), w, TRUE, TRUE, 0);
    gtk_box_pack_start (GTK_BOX (&pw->vbox), hb, TRUE, TRUE, 0);

    pw->widgets = g_list_append (pw->widgets, hb);
    pw->widgets = g_list_append (pw->widgets, lbl);
    pw->widgets = g_list_append (pw->widgets, w);

    g_hash_table_insert (pw->params, (gpointer)name, w);

    param_data_t *pdata = param_data_new(name, w, data_type);
    g_hash_table_insert (pw->widget_to_param, w, pdata);

    if (signal_name && strlen (signal_name)) {
        g_signal_connect (G_OBJECT(w), signal_name,
                G_CALLBACK (generic_widget_changed), pw);
    }

    gtk_widget_show_all (hb);
    return pdata;
}

int
bot_gtk_param_widget_add_int(BotGtkParamWidget *pw, const char *name, 
        BotGtkParamWidgetUIHint ui_hints,
        int min, int max, int increment, int initial_value)
{
    if (have_parameter_key (pw, name)) return -1;
    if (min >= max || initial_value < min || initial_value > max) {
        err("WARNING: param_widget_add_int - invalid args\n");
        return -1;
    }

    GtkWidget *w = NULL;
    switch(ui_hints) {
        case BOT_GTK_PARAM_WIDGET_SPINBOX:
            w = gtk_spin_button_new_with_range (min, max, increment);
            gtk_spin_button_set_value (GTK_SPIN_BUTTON(w), initial_value);
            break;
        case BOT_GTK_PARAM_WIDGET_SLIDER:
        case BOT_GTK_PARAM_WIDGET_DEFAULTS:
            w = gtk_hscale_new_with_range (min, max, increment);
            gtk_range_set_value (GTK_RANGE(w), initial_value);
            break;
        default:
            err("ERROR: param_widget_add_int - bad ui_hints\n");
            return -1;
    } 
    g_object_set_data (G_OBJECT(w), "data-type", "int");
    add_row (pw, name, w, "value-changed", G_TYPE_INT);
    return 0;
}

double bot_gtk_param_widget_get_min_double(BotGtkParamWidget *pw,
        const char *name);
double 
bot_gtk_param_widget_get_min_double(BotGtkParamWidget *pw,
        const char *name)
{
    param_data_t *pdata = get_param_data_check_type(pw, name, G_TYPE_DOUBLE);
    if(!pdata)
        return NAN;
    return pdata->min_double;
}

double bot_gtk_param_widget_get_max_double(BotGtkParamWidget *pw,
        const char *name);
double 
bot_gtk_param_widget_get_max_double(BotGtkParamWidget *pw,
        const char *name)
{
    param_data_t *pdata = get_param_data_check_type(pw, name, G_TYPE_DOUBLE);
    if(!pdata)
        return NAN;
    return pdata->max_double;
}

int bot_gtk_param_widget_add_double (BotGtkParamWidget *pw,
        const char *name, BotGtkParamWidgetUIHint ui_hints, 
        double min, double max, double increment, double initial_value)
{
    if (have_parameter_key (pw, name)) return -1;
    if (min >= max || initial_value < min || initial_value > max) {
        err("WARNING: param_widget_add_double - invalid args\n");
        return -1;
    }

    GtkWidget *w = NULL;
    switch(ui_hints) {
        case BOT_GTK_PARAM_WIDGET_SPINBOX:
            w = gtk_spin_button_new_with_range (min, max, increment);
            gtk_spin_button_set_value (GTK_SPIN_BUTTON(w), initial_value);
            break;
        case BOT_GTK_PARAM_WIDGET_SLIDER:
        case BOT_GTK_PARAM_WIDGET_DEFAULTS:
            w = gtk_hscale_new_with_range (min, max, increment);
            gtk_range_set_value (GTK_RANGE(w), initial_value);
            break;
        default:
            err("ERROR: param_widget_add_double - bad ui_hints\n");
            return -1;
    } 
    param_data_t * pdata = add_row (pw, name, w, "value-changed", 
            G_TYPE_DOUBLE);
    pdata->min_double = min;
    pdata->max_double = max;
    g_object_set_data (G_OBJECT(w), "data-type", "double");
    return 0;
}

int bot_gtk_param_widget_add_text_entry (BotGtkParamWidget *pw,
        const char *name, BotGtkParamWidgetUIHint ui_hints,
        const char *initial_value)
{
    if (have_parameter_key (pw, name)) return -1;
    
    
    GtkWidget *w = NULL;
    switch(ui_hints) {
        case BOT_GTK_PARAM_WIDGET_ENTRY:
            w = gtk_entry_new ();
            gtk_entry_set_text (GTK_SPIN_BUTTON(w), initial_value);
            break;
        default:
            err("ERROR: param_widget_add_text_entry - bad ui_hints\n");
            return -1;
    } 
    param_data_t * pdata = add_row (pw, name, w, "value-changed", 
            G_TYPE_STRING);
    g_object_set_data (G_OBJECT(w), "data-type", "string");
    return 0;
}

static int
add_checkboxes_helper (BotGtkParamWidget *pw, GtkBox *box, const char *name, 
        int checked)
{
    if (have_parameter_key (pw, name)) return -1;
    GtkWidget *cb = gtk_check_button_new_with_label (name);
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (cb), checked);

    gtk_box_pack_start (GTK_BOX (box), cb, FALSE, FALSE, 0);

    pw->widgets = g_list_append (pw->widgets, cb);

    g_hash_table_insert (pw->params, (gpointer)name, cb);
    g_object_set_data (G_OBJECT(cb), "data-type", "boolean");

    param_data_t *pd = param_data_new(name, cb, G_TYPE_BOOLEAN);
    g_hash_table_insert (pw->widget_to_param, cb, pd);

    g_signal_connect (G_OBJECT(cb), "toggled",
            G_CALLBACK (generic_widget_changed), pw);
    return 0;
}

static int
add_checkboxes (BotGtkParamWidget *pw, 
        const char *name1, int initally_checked1,
        va_list ap)
{
    GtkBox *hb = GTK_BOX (gtk_hbox_new (FALSE, 0));

    int status;
    status = add_checkboxes_helper (pw, hb, name1, initally_checked1);
    if (0 != status) return status;

    int i;
    for (i=0; i<10000; i++) {
        const char *name = va_arg (ap, const char *);
        if (! name || strlen (name) == 0) break;

        int checked = va_arg (ap, int);

        status = add_checkboxes_helper (pw, hb, name, checked);
        if (0 != status) {
            return status;
        }
    }

    gtk_widget_show_all (GTK_WIDGET (hb));

    gtk_box_pack_start (GTK_BOX(&pw->vbox), GTK_WIDGET(hb), TRUE, TRUE, 0);
    return 0;
}

static int
add_togglebuttons_helper (BotGtkParamWidget *pw, GtkBox *box, const char *name, 
        int checked)
{
    if (have_parameter_key (pw, name)) return -1;
    GtkWidget *tb = gtk_toggle_button_new_with_label (name);
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (tb), checked);

    gtk_box_pack_start (GTK_BOX (box), tb, FALSE, FALSE, 0);

    pw->widgets = g_list_append (pw->widgets, tb);

    g_hash_table_insert (pw->params, (gpointer)name, tb);
    g_object_set_data (G_OBJECT(tb), "data-type", "boolean");

    param_data_t *pd = param_data_new(name, tb, G_TYPE_BOOLEAN);
    g_hash_table_insert (pw->widget_to_param, tb, pd);

    g_signal_connect (G_OBJECT(tb), "toggled",
            G_CALLBACK (generic_widget_changed), pw);
    return 0;
}

static int
add_togglebuttons (BotGtkParamWidget *pw, 
        const char *name1, int initally_checked1,
        va_list ap)
{
    GtkBox *hb = GTK_BOX (gtk_hbox_new (FALSE, 0));

    int status;
    status = add_togglebuttons_helper (pw, hb, name1, initally_checked1);
    if (0 != status) return status;

    int i;
    for (i=0; i<10000; i++) {
        const char *name = va_arg (ap, const char *);
        if (! name || strlen (name) == 0) break;

        int checked = va_arg (ap, int);

        status = add_togglebuttons_helper (pw, hb, name, checked);
        if (0 != status) {
            return status;
        }
    }

    gtk_widget_show_all (GTK_WIDGET (hb));

    gtk_box_pack_start (GTK_BOX(&pw->vbox), GTK_WIDGET(hb), TRUE, TRUE, 0);
    return 0;
}

int bot_gtk_param_widget_add_booleans (BotGtkParamWidget *pw, 
        BotGtkParamWidgetUIHint ui_hints,
        const char *name1, int initally_checked1,
        ...)
{
    int status = -1;
    va_list ap;
    va_start (ap, initally_checked1);

    switch (ui_hints) {
        case BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON:
            status = add_togglebuttons (pw, name1, initally_checked1, ap);
            break;
        case BOT_GTK_PARAM_WIDGET_CHECKBOX:
        case BOT_GTK_PARAM_WIDGET_DEFAULTS:
            status = add_checkboxes(pw, name1, initally_checked1, ap);
            break;
        default:
            err("ERROR: param_widget_add_booleans - bad ui_hints\n");
    }
    va_end (ap);
    return status;
}

static int
add_buttons_helper (BotGtkParamWidget *pw, GtkBox *box, const char *name)
{
    if (have_parameter_key (pw, name)) return -1;
    GtkWidget *b = gtk_button_new_with_label (name);

    gtk_box_pack_start (GTK_BOX (box), b, FALSE, FALSE, 0);

    pw->widgets = g_list_append (pw->widgets, b);

    g_hash_table_insert (pw->params, (gpointer)name, b);
    g_object_set_data (G_OBJECT(b), "data-type", "button");

    param_data_t *pd = param_data_new(name, b, G_TYPE_NONE);
    g_hash_table_insert (pw->widget_to_param, b, pd);

    g_signal_connect (G_OBJECT(b), "clicked",
            G_CALLBACK (generic_widget_changed), pw);
    return 0;
}

static int
add_buttons (BotGtkParamWidget *pw, 
        const char *name1, 
        va_list ap)
{
    GtkBox *hb = GTK_BOX (gtk_hbox_new (FALSE, 0));

    int status;
    status = add_buttons_helper (pw, hb, name1);
    if (0 != status) return status;

    int i;
    for (i=0; i<10000; i++) {
        const char *name = va_arg (ap, const char *);
        if (! name || strlen (name) == 0) break;

        status = add_buttons_helper (pw, hb, name);
        if (0 != status) {
            return status;
        }
    }

    gtk_widget_show_all (GTK_WIDGET (hb));

    gtk_box_pack_start (GTK_BOX(&pw->vbox), GTK_WIDGET(hb), TRUE, TRUE, 0);
    return 0;
}

int bot_gtk_param_widget_add_buttons (BotGtkParamWidget *pw, 
        const char *name1, ...)
{
    int status = -1;
    va_list ap;
    va_start (ap, name1);
    status = add_buttons (pw, name1, ap);
    va_end (ap);
    return status;
}

static int
_add_menu (BotGtkParamWidget *pw, const char *name, int initial_value,
        int noptions, const char **names, const int *values)
{
    GtkTreeIter iter;

    GtkListStore * store = gtk_list_store_new (2, G_TYPE_STRING, G_TYPE_INT);
    int selected = -1;

    int i;
    for (i=0; i<noptions; i++) {
        gtk_list_store_append (store, &iter);
        gtk_list_store_set (store, &iter, 0, names[i], 1, values[i], -1);
        if (values[i] == initial_value) selected = i;
    }

    GtkWidget *mb = gtk_combo_box_new_with_model (GTK_TREE_MODEL (store));
    GtkCellRenderer * renderer = gtk_cell_renderer_text_new();
    gtk_cell_layout_pack_start (GTK_CELL_LAYOUT (mb), renderer, TRUE);
    gtk_cell_layout_add_attribute (GTK_CELL_LAYOUT (mb), renderer, "text", 0);
    g_object_set (G_OBJECT (renderer), "font", "monospace", NULL);
    g_object_unref (store);
    gtk_combo_box_set_active (GTK_COMBO_BOX(mb), selected);

    add_row (pw, name, mb, "changed", G_TYPE_ENUM);
    g_object_set_data (G_OBJECT(mb), "data-type", "enum");
    return 0;
}

int 
bot_gtk_param_widget_add_enumv (BotGtkParamWidget *pw,
        const char *name, BotGtkParamWidgetUIHint ui_hints, int initial_value,
        int noptions, const char **names, const int *values)
{
    switch (ui_hints) {
        case BOT_GTK_PARAM_WIDGET_MENU:
        case BOT_GTK_PARAM_WIDGET_DEFAULTS:
            return _add_menu (pw, name, initial_value, 
                    noptions, names, values);
            break;
        default:
            err("ERROR: param_widget_add_enum - bad ui_hints\n");
    }
    return -1;
}

int 
bot_gtk_param_widget_add_enum (BotGtkParamWidget *pw,
        const char *name, BotGtkParamWidgetUIHint ui_hints, int initial_value,
        const char *string1, int value1, ...)
{
    va_list ap;
    va_start (ap, value1);

    GPtrArray *names = g_ptr_array_new ();
    GArray *values = g_array_new (FALSE, FALSE, sizeof (int));
    g_ptr_array_add (names, strdup (string1));
    g_array_append_val (values, value1);

    while (1) {
        const char *str = va_arg (ap, const char *);
        if (! str || strlen (str) == 0) break;
        int val = va_arg (ap, int);

        g_ptr_array_add (names, strdup (str));
        g_array_append_val (values, val);
    }
    va_end (ap);

    int status = bot_gtk_param_widget_add_enumv (pw, name, ui_hints, 
            initial_value, names->len, (const char **)names->pdata, 
            (int*) values->data);
    for (int i=0; i<values->len; i++) free (g_ptr_array_index (names, i));
    g_ptr_array_free (names, TRUE);
    g_array_free (values, TRUE);

    return status;
}

void 
bot_gtk_param_widget_add_separator (BotGtkParamWidget *pw, const char *text)
{
    if (!text) {
        GtkWidget *sep = gtk_hseparator_new();
        gtk_box_pack_start (GTK_BOX (&pw->vbox), sep , FALSE, FALSE, 0);
        gtk_widget_show_all(sep);
    } else {
        GtkWidget *b = gtk_hbox_new(FALSE, 0);
        gtk_box_pack_start(GTK_BOX(b), gtk_hseparator_new(), TRUE, TRUE, 0);
        gtk_box_pack_start(GTK_BOX(b), gtk_label_new(text), FALSE, FALSE, 0);
        gtk_box_pack_start(GTK_BOX(b), gtk_hseparator_new(), TRUE, TRUE, 0);
        gtk_box_pack_start(GTK_BOX(&pw->vbox), GTK_WIDGET(b), FALSE, FALSE, 0);
        gtk_widget_show_all(GTK_WIDGET(b));
    }
}

int bot_gtk_param_widget_get_int (BotGtkParamWidget *pw, const char *name)
{
    if (! have_parameter_key (pw, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return 0;
    }
    GtkWidget *w = g_hash_table_lookup (pw->params, name);

    GType type = G_OBJECT_TYPE (w);

    if (GTK_TYPE_HSCALE == type) {
        return (int) gtk_range_get_value (GTK_RANGE (w));
    } else if ( GTK_TYPE_SPIN_BUTTON == type) {
        return (int) gtk_spin_button_get_value (GTK_SPIN_BUTTON (w));
    } else {
        fprintf(stderr, "param_widget:  can't retrieve parameter [%s] "
                "as integer.\n", name);
        return 0;
    }

    return 0;
}

double bot_gtk_param_widget_get_double (BotGtkParamWidget *pw, const char *name)
{
    if (! have_parameter_key (pw, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return 0;
    }
    GtkWidget *w = g_hash_table_lookup (pw->params, name);

    GType type = G_OBJECT_TYPE (w);

    if (GTK_TYPE_HSCALE == type) {
        return gtk_range_get_value (GTK_RANGE (w));
    } else if ( GTK_TYPE_SPIN_BUTTON == type) {
        return gtk_spin_button_get_value (GTK_SPIN_BUTTON (w));
    } else {
        fprintf(stderr, "param_widget:  can't retrieve parameter [%s] "
                "as double.\n", name);
        return 0;
    }

    return 0;
}

const gchar * bot_gtk_param_widget_get_text_entry (BotGtkParamWidget *pw, const char *name)
{
    if (! have_parameter_key (pw, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return 0;
    }
    GtkWidget *w = g_hash_table_lookup (pw->params, name);

    GType type = G_OBJECT_TYPE (w);

    if ( GTK_TYPE_ENTRY == type) {
        return gtk_entry_get_text (GTK_ENTRY (w));
    } else {
        fprintf(stderr, "param_widget:  can't retrieve parameter [%s] "
                "as text.\n", name);
        return 0;
    }

    return 0;
}


int bot_gtk_param_widget_get_bool (BotGtkParamWidget *pw, const char *name)
{
    if (! have_parameter_key (pw, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return 0;
    }
    GtkWidget *w = g_hash_table_lookup (pw->params, name);

    if (GTK_IS_TOGGLE_BUTTON (w)) {
        return gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON (w));
    } else {
        fprintf(stderr, "param_widget:  can't retrieve parameter [%s] "
                "as boolean.\n", name);
        return 0;
    }

    return 0;
}

const char *
bot_gtk_param_widget_get_enum_str (BotGtkParamWidget *pw, const char *name)
{
    if (! have_parameter_key (pw, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return NULL;
    }
    GtkWidget *w = g_hash_table_lookup (pw->params, name);

    GType type = G_OBJECT_TYPE (w);
    if (pw->estr) {
        free (pw->estr);
        pw->estr = NULL;
    }

    if (GTK_TYPE_COMBO_BOX == type) {
        GtkTreeIter iter;
        gtk_combo_box_get_active_iter (GTK_COMBO_BOX(w), &iter);
        GtkTreeModel *model = gtk_combo_box_get_model (GTK_COMBO_BOX(w));
        gtk_tree_model_get (model, &iter, 0, &pw->estr, -1);
        return pw->estr;
    } else {
        fprintf(stderr, "param_widget:  can't retrieve parameter [%s] "
                "as string.\n", name);
        return NULL;
    }
    return NULL;
}

int 
bot_gtk_param_widget_get_enum (BotGtkParamWidget *pw, const char *name)
{
    if (! have_parameter_key (pw, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return -1;
    }
    GtkWidget *w = g_hash_table_lookup (pw->params, name);

    int result;
    GType type = G_OBJECT_TYPE (w);

    if (GTK_TYPE_COMBO_BOX == type) {
        GtkTreeIter iter;
        gtk_combo_box_get_active_iter (GTK_COMBO_BOX(w), &iter);
        GtkTreeModel *model = gtk_combo_box_get_model (GTK_COMBO_BOX(w));
        gtk_tree_model_get (model, &iter, 1, &result, -1);
        return result;
    } else {
        fprintf(stderr, "param_widget:  can't retrieve parameter [%s] "
                "as string.\n", name);
        return -1;
    }
    return -1;
}

void 
bot_gtk_param_widget_set_int (BotGtkParamWidget *pw, const char *name, 
        int val)
{
    if (! have_parameter_key (pw, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return;
    }
    GtkWidget *w = g_hash_table_lookup (pw->params, name);
    GType type = G_OBJECT_TYPE (w);

    if (GTK_TYPE_SPIN_BUTTON == type) {
        gtk_spin_button_set_value (GTK_SPIN_BUTTON(w), val);
    } else if (GTK_TYPE_HSCALE == type) {
        gtk_range_set_value (GTK_RANGE(w), val);
    }
}

void 
bot_gtk_param_widget_set_double (BotGtkParamWidget *pw, const char *name,
        double val)
{
    param_data_t *pdata = get_param_data_check_type(pw, name, G_TYPE_DOUBLE);
    if(!pdata) {
        g_warning("%s: invalid parameter [%s]\n", __FUNCTION__, name);
        return;
    }
    val = fmin(fmax(val, pdata->min_double), pdata->max_double);
    GType type = G_OBJECT_TYPE(pdata->widget);
    if (GTK_TYPE_SPIN_BUTTON == type) {
        gtk_spin_button_set_value (GTK_SPIN_BUTTON(pdata->widget), val);
    } else if (GTK_TYPE_HSCALE == type) {
        gtk_range_set_value (GTK_RANGE(pdata->widget), val);
    }
}

void 
bot_gtk_param_widget_set_bool (BotGtkParamWidget *pw, const char *name,
        int val)
{
    if (! have_parameter_key (pw, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return;
    }
    GtkWidget *w = g_hash_table_lookup (pw->params, name);
    GType type = G_OBJECT_TYPE (w);
    if (GTK_TYPE_CHECK_BUTTON == type) {
        gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON(w), val);
    }
}

typedef struct _tree_model_match_data {
    int val;
    GtkComboBox *combo;
} tree_model_match_data_t;

static gboolean
activate_if_matched (GtkTreeModel *model, GtkTreePath *path, GtkTreeIter *iter,
        void *user_data)
{
    tree_model_match_data_t *md = (tree_model_match_data_t*) user_data;

    int entry_val;
    gtk_tree_model_get (model, iter, 1, &entry_val, -1);
    
    if (entry_val == md->val) {
        gtk_combo_box_set_active_iter (md->combo, iter);
        return TRUE;
    }
    return FALSE;
}

void 
bot_gtk_param_widget_set_enum (BotGtkParamWidget *pw, const char *name,
        int val)
{
    if (! have_parameter_key (pw, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return;
    }
    GtkWidget *w = g_hash_table_lookup (pw->params, name);
    GType type = G_OBJECT_TYPE (w);
    if (GTK_TYPE_COMBO_BOX == type) {
        GtkTreeModel *model = gtk_combo_box_get_model (GTK_COMBO_BOX(w));
        tree_model_match_data_t matchdata = {
            val, 
            GTK_COMBO_BOX(w)
        };
        gtk_tree_model_foreach (model, activate_if_matched, &matchdata);
    }
}

void 
bot_gtk_param_widget_set_enabled (BotGtkParamWidget *pw, const char *name,
        int enabled)
{
    if (! have_parameter_key (pw, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return;
    }
    GtkWidget *w = g_hash_table_lookup (pw->params, name);
    gtk_widget_set_sensitive(w, enabled);
}

typedef struct {
    GKeyFile *keyfile;
    const char *group_name;
    BotGtkParamWidget *pw;
} _keyfile_data_t;

static void
_param_load_from_keyfile (void *key, void *value, void *user_data)
{
    GtkWidget *w = GTK_WIDGET (value);
    _keyfile_data_t *data = user_data;
    const char *param_name = key;
    const char *type = g_object_get_data (G_OBJECT(w), "data-type");
    GError *gerr = NULL;
    if (!strcmp (type, "boolean")) {
        gboolean val = g_key_file_get_boolean (data->keyfile, data->group_name,
                param_name, &gerr);
        if (!gerr) {
            bot_gtk_param_widget_set_bool (data->pw, param_name, val);
        }
    } else if (!strcmp (type, "double")) {
        double val = g_key_file_get_double (data->keyfile, data->group_name,
                param_name, &gerr);
        if (!gerr) {
            bot_gtk_param_widget_set_double (data->pw, param_name, val);
        }
    } else if (!strcmp (type, "int")) {
        int val = g_key_file_get_integer (data->keyfile, data->group_name,
                param_name, &gerr);
        if (!gerr) {
            bot_gtk_param_widget_set_int (data->pw, param_name, val);
        }
    } else if (!strcmp (type, "enum")) {
        int val = g_key_file_get_integer (data->keyfile, data->group_name,
                param_name, &gerr);
        if (!gerr) {
            bot_gtk_param_widget_set_enum (data->pw, param_name, val);
        }
    } else {
        // TODO
    }
    if (gerr) {
//        g_warning ("couldn't load param %s from GKeyFile\n", param_name);
//        g_warning ("%s\n", gerr->message);
        g_error_free (gerr);
    }
}

void 
bot_gtk_param_widget_load_from_key_file (BotGtkParamWidget *pw, 
        GKeyFile *keyfile, const char *group_name)
{
    _keyfile_data_t data = {
        .keyfile = keyfile,
        .group_name = group_name,
        .pw = pw
    };
    g_hash_table_foreach (pw->params, _param_load_from_keyfile, &data);
}

static void
_param_save_to_keyfile (void *key, void *value, void *user_data)
{
    GtkWidget *w = GTK_WIDGET (value);
    _keyfile_data_t *data = user_data;
    const char *param_name = key;
    const char *type = g_object_get_data (G_OBJECT(w), "data-type");
    if (!strcmp (type, "boolean")) {
        int val = bot_gtk_param_widget_get_bool (data->pw, param_name);
        g_key_file_set_boolean (data->keyfile, data->group_name,
                param_name, val);
    } else if (!strcmp (type, "double")) {
        double val = bot_gtk_param_widget_get_double (data->pw, param_name);
        g_key_file_set_double (data->keyfile, data->group_name,
                param_name, val);
    } else if (!strcmp (type, "int")) {
        int val = bot_gtk_param_widget_get_int (data->pw, param_name);
        g_key_file_set_integer (data->keyfile, data->group_name,
                param_name, val);
    } else if (!strcmp (type, "enum")) {
        int val = bot_gtk_param_widget_get_enum (data->pw, param_name);
        g_key_file_set_integer (data->keyfile, data->group_name,
                param_name, val);
    } else {
        // TODO
    }
}

void 
bot_gtk_param_widget_save_to_key_file (BotGtkParamWidget *pw,
        GKeyFile *keyfile, const char *group_name)
{
    _keyfile_data_t data = {
        .keyfile = keyfile,
        .group_name = group_name,
        .pw = pw
    };
    g_hash_table_foreach (pw->params, _param_save_to_keyfile, &data);
}

int 
bot_gtk_param_widget_modify_int(BotGtkParamWidget *pw,
        const char *name, int min, int max, int increment, int value)
{
    if (! have_parameter_key (pw, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return -1;
    }
    GtkWidget *w = g_hash_table_lookup (pw->params, name);

    GType type = G_OBJECT_TYPE (w);

    if (GTK_TYPE_HSCALE == type) {
        gtk_range_set_range(GTK_RANGE(w), min, max);
        gtk_range_set_increments(GTK_RANGE(w), increment, increment * 10);
        gtk_range_set_value(GTK_RANGE(w), value);
    } else if (GTK_TYPE_SPIN_BUTTON == type) {
        GtkSpinButton *sb = GTK_SPIN_BUTTON(w);
        gtk_spin_button_set_range(sb, min, max);
        gtk_spin_button_set_increments(sb, increment, increment * 10);
        gtk_spin_button_set_value(sb, value);
    } else {
        fprintf(stderr, "param_widget:  can't modify parameter [%s] "
                "as integer.\n", name);
        return -1;
    }

    return 0;
}


int
bot_gtk_param_widget_modify_double(BotGtkParamWidget *pw,
        const char *name, double min, double max, double increment, double value)
{
    if (! have_parameter_key (pw, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return -1;
    }
    GtkWidget *w = g_hash_table_lookup (pw->params, name);

    GType type = G_OBJECT_TYPE (w);

    if (GTK_TYPE_HSCALE == type) {
        gtk_range_set_range(GTK_RANGE(w), min, max);
        gtk_range_set_increments(GTK_RANGE(w), increment, increment * 10);
        gtk_range_set_value(GTK_RANGE(w), value);
    } else if (GTK_TYPE_SPIN_BUTTON == type) {
        GtkSpinButton *sb = GTK_SPIN_BUTTON(w);
        gtk_spin_button_set_range(sb, min, max);
        gtk_spin_button_set_increments(sb, increment, increment * 10);
        gtk_spin_button_set_value(sb, value);
    } else {
        fprintf(stderr, "param_widget:  can't modify parameter [%s] "
                "as double.\n", name);
        return -1;
    }

    return 0;
}



int bot_gtk_param_widget_modify_enum(BotGtkParamWidget *pw, const char *name,
        const char *label, const int value)
{
    if (! have_parameter_key (pw, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return -1;
    }
    GtkWidget *w = g_hash_table_lookup (pw->params, name);

    GType type = G_OBJECT_TYPE (w);

    if (GTK_TYPE_COMBO_BOX == type) {
        GtkTreeIter iter;
        gtk_combo_box_get_active_iter (GTK_COMBO_BOX(w), &iter);
    	GtkListStore * store = gtk_list_store_new (2, G_TYPE_STRING, G_TYPE_INT);
        store = (GtkListStore *) gtk_combo_box_get_model (GTK_COMBO_BOX(w));
	gtk_list_store_append(store, &iter);
	gtk_list_store_set(store, &iter, 0, label, 1, value, -1);
	gtk_combo_box_set_model(GTK_COMBO_BOX(w), GTK_TREE_MODEL(store));
    	gtk_combo_box_set_active_iter (GTK_COMBO_BOX(w), &iter);
    }
    return 0;
}

void bot_gtk_param_widget_clear_enum(BotGtkParamWidget *pw, const char *name)
{
    if (! have_parameter_key (pw, name)) {
        fprintf(stderr, "param_widget: invalid parameter [%s]\n", name);
        return;
    }
    GtkWidget *w = g_hash_table_lookup (pw->params, name);

    GType type = G_OBJECT_TYPE (w);

    if (GTK_TYPE_COMBO_BOX == type) {
    	GtkListStore * store = gtk_list_store_new (2, G_TYPE_STRING, G_TYPE_INT);
	gtk_combo_box_set_model(GTK_COMBO_BOX(w), GTK_TREE_MODEL(store));
        GtkTreeIter iter;
	gtk_list_store_append(store, &iter);
	gtk_list_store_set(store, &iter, 0, "None", 1, 0, -1);
    	gtk_combo_box_set_active_iter (GTK_COMBO_BOX(w), &iter);
    }
    return;
}

