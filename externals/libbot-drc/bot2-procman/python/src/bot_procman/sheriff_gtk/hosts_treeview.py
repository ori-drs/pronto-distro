import time
import gobject
import gtk

import bot_procman.sheriff as sheriff
import bot_procman.sheriff_gtk.command_model as cm
import bot_procman.sheriff_gtk.sheriff_dialogs as sd

class SheriffHostModel(gtk.ListStore):
    COL_OBJ, \
    COL_NAME, \
    COL_LAST_UPDATE, \
    COL_LOAD, \
    NUM_ROWS = range(5)

    def __init__(self, _sheriff):
        super(SheriffHostModel, self).__init__(gobject.TYPE_PYOBJECT,
                gobject.TYPE_STRING, # deputy name
                gobject.TYPE_STRING, # last update time
                gobject.TYPE_STRING, # load
                )
        self.sheriff = _sheriff

    def update(self):
        to_update = set(self.sheriff.get_deputies ())
        to_remove = []

        def _deputy_last_update_str (dep):
            if dep.last_update_utime:
                now_utime = time.time () * 1000000
                return "%.1f seconds ago" % ((now_utime-dep.last_update_utime) * 1e-6)
            else:
                return "<never>"

        def _update_host_row (model, path, model_iter, user_data):
            deputy = model.get_value (model_iter, SheriffHostModel.COL_OBJ)
            if deputy in to_update:
                model.set (model_iter,
                        SheriffHostModel.COL_LAST_UPDATE,
                        _deputy_last_update_str (deputy),
                        SheriffHostModel.COL_LOAD,
                        "%f" % deputy.cpu_load,
                        )
                to_update.remove (deputy)
            else:
                to_remove.append (gtk.TreeRowReference (model, path))

        self.foreach (_update_host_row, None)

        for trr in to_remove:
            self.remove (self.get_iter (trr.get_path()))

        for deputy in to_update:
#            print "adding %s to treeview" % deputy.name
            new_row = (deputy, deputy.name, _deputy_last_update_str (deputy),
                    "%f" % deputy.cpu_load,
                    )
            self.append (new_row)


class SheriffHostTreeView(gtk.TreeView):
    def __init__(self, _sheriff, hosts_ts):
        super(SheriffHostTreeView, self).__init__(hosts_ts)
        self.sheriff = _sheriff
        self.hosts_ts = hosts_ts

        plain_tr = gtk.CellRendererText ()
        col = gtk.TreeViewColumn ("Deputy", plain_tr, text=SheriffHostModel.COL_NAME)
        col.set_sort_column_id (1)
        col.set_resizable (True)
        self.append_column (col)

        last_update_tr = gtk.CellRendererText()
        col = gtk.TreeViewColumn ("Last update", last_update_tr, text=SheriffHostModel.COL_LAST_UPDATE)
#        col.set_sort_column_id (2) # XXX this triggers really weird bugs...
        col.set_resizable (True)
        col.set_cell_data_func(last_update_tr, self._deputy_last_update_cell_data_func)
        self.append_column (col)

        col = gtk.TreeViewColumn ("Load", plain_tr, text=SheriffHostModel.COL_LOAD)
        col.set_resizable (True)
        self.append_column (col)

        self.connect ("button-press-event",
                self._on_hosts_tv_button_press_event)

        # hosts treeview context menu
        self.hosts_ctxt_menu = gtk.Menu ()

        self.cleanup_hosts_ctxt_mi = gtk.MenuItem ("_Cleanup")
        self.hosts_ctxt_menu.append (self.cleanup_hosts_ctxt_mi)
        self.cleanup_hosts_ctxt_mi.connect ("activate",
                self._cleanup_hosts)
        self.hosts_ctxt_menu.show_all()

#        # set some default appearance parameters
#        self.base_color = gtk.gdk.Color(65535, 65535, 65535)
#        self.text_color = gtk.gdk.Color(0, 0, 0)
#        self.set_background_color(self.base_color)
#        self.set_text_color(self.text_color)

    def _on_hosts_tv_button_press_event (self, treeview, event):
        if event.type == gtk.gdk.BUTTON_PRESS and event.button == 3:
            self.hosts_ctxt_menu.popup (None, None, None, event.button, event.time)
            return True

    def _cleanup_hosts(self, *args):
        self.sheriff.purge_useless_deputies()
        self.hosts_ts.update()

    def _deputy_last_update_cell_data_func (self, column, cell, model, model_iter):
        # bit of a hack to pull out the last update time
        try:
            last_update = float(model.get_value(model_iter, SheriffHostModel.COL_LAST_UPDATE).split()[0])
        except:
            last_update = None
        if last_update is None or last_update > 5:
            cell.set_property("cell-background-set", True)
            cell.set_property("cell-background", "Red")
#            cell.set_property("foreground", "Black")
        elif last_update > 2:
            cell.set_property("cell-background-set", True)
            cell.set_property("cell-background", "Yellow")
#            cell.set_property("foreground", "Black")
        else:
            cell.set_property("cell-background-set", False)
#            cell.set_property("foreground-set", False)

#    def get_background_color(self):
#        return self.base_color
#
#    def get_text_color(self):
#        return self.text_color
#
#    def set_background_color(self, color):
#        self.base_color = color
#        self.modify_base(gtk.STATE_NORMAL, color)
#        self.modify_base(gtk.STATE_ACTIVE, color)
#        self.modify_base(gtk.STATE_PRELIGHT, color)
#
#    def set_text_color(self, color):
#        self.text_color = color
#        self.modify_text(gtk.STATE_NORMAL, color)
#        self.modify_text(gtk.STATE_ACTIVE, color)
#        self.modify_text(gtk.STATE_PRELIGHT, color)

    def save_settings(self, save_map):
        pass
#        save_map["hosts_treeview_background_color"] = self.base_color.to_string()
#        save_map["hosts_treeview_text_color"] = self.text_color.to_string()

    def load_settings(self, save_map):
        pass
#        if "hosts_treeview_background_color" in save_map:
#            self.set_background_color(gtk.gdk.Color(save_map["hosts_treeview_background_color"]))
#
#        if "hosts_treeview_text_color" in save_map:
#            self.set_text_color(gtk.gdk.Color(save_map["hosts_treeview_text_color"]))

#    def _get_selected_hosts (self):
#        model, rows = self.hosts_tv.get_selection ().get_selected_rows ()
#        return [ model.get_value (model.get_iter(path), 0) \
#                for path in rows ]

