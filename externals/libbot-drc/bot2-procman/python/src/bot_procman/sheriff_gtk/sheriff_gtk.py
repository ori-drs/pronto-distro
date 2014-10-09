#!/usr/bin/env python

import os
import sys
import time
import traceback
import getopt
import subprocess
import signal
import pickle

import glib
import gobject
import gtk
import pango

from lcm import LCM

from bot_procman.orders_t import orders_t
import bot_procman.sheriff as sheriff
import bot_procman.sheriff_config as sheriff_config

import bot_procman.sheriff_gtk.command_model as cm
import bot_procman.sheriff_gtk.command_treeview as ctv
import bot_procman.sheriff_gtk.sheriff_dialogs as sd
import bot_procman.sheriff_gtk.command_console as cc
import bot_procman.sheriff_gtk.hosts_treeview as ht

try:
    from bot_procman.build_prefix import BUILD_PREFIX
except ImportError:
    BUILD_PREFIX = None

def find_bot_procman_deputy_cmd():
    search_path = []
    if BUILD_PREFIX is not None:
        search_path.append("%s/bin" % BUILD_PREFIX)
    search_path.extend(os.getenv("PATH").split(":"))
    for dirname in search_path:
        fname = "%s/bot-procman-deputy" % dirname
        if os.path.exists(fname) and os.path.isfile(fname):
            return fname
    return None

def find_bot_procman_glade():
    search_path = []
    if BUILD_PREFIX:
        search_path.append(os.path.join(BUILD_PREFIX, "share", "bot_procman"))
    search_path.append("/usr/share/bot_procman")
    search_path.append("/usr/local/share/bot_procman")
    for spath in search_path:
        fname = os.path.join(spath, "procman-sheriff.glade")
        if os.path.isfile(fname):
            return fname
    sys.stderr.write("ERROR!  Unable to find procman-sheriff.glade\n")
    sys.stderr.write("Locations checked:\n")
    for spath in search_path:
        sys.stderr.write("    %s\n" % spath)
    sys.exit(1)

def split_script_name(name):
    name = name.strip("/")
    while name.find("//") >= 0:
        name = name.replace("//", "/")
    return name.split("/")

class SheriffGtk(object):
    def __init__ (self, lc):
        self.lc = lc
        self.cmds_update_scheduled = False
        self.config_filename = None
        self.script_done_action = None

        # deputy spawned by the sheriff
        self.spawned_deputy = None

        # create sheriff and subscribe to events
        self.sheriff = sheriff.Sheriff (self.lc)
        self.sheriff.command_added.connect(self._schedule_cmds_update)
        self.sheriff.command_removed.connect(self._schedule_cmds_update)
        self.sheriff.command_status_changed.connect(self._schedule_cmds_update)
        self.sheriff.command_group_changed.connect(self._schedule_cmds_update)
        self.sheriff.script_started.connect(self._on_script_started)
        self.sheriff.script_action_executing.connect(self._on_script_action_executing)
        self.sheriff.script_finished.connect(self._on_script_finished)
        self.sheriff.script_added.connect(self._on_script_added)
        self.sheriff.script_removed.connect(self._on_script_removed)

        # update very soon
        gobject.timeout_add(100, lambda *s: self.hosts_ts.update() and False)
        gobject.timeout_add(100, lambda *s: self._schedule_cmds_update() and False)

        # and then periodically
        gobject.timeout_add (1000, self._maybe_send_orders)
        gobject.timeout_add (1000,
                lambda *s: self._schedule_cmds_update () or True)

        self.lc.subscribe ("PMD_ORDERS", self.on_procman_orders)

        # setup GUI

        self.builder = gtk.Builder()
        self.builder.add_from_file(find_bot_procman_glade())
        self.builder.connect_signals(self)

        self.window = self.builder.get_object("main_window")

        self.cmds_ts = cm.SheriffCommandModel(self.sheriff)
        self.cmds_tv = ctv.SheriffCommandTreeView(self.sheriff, self.cmds_ts)

        # load save menu
        self.load_cfg_mi = self.builder.get_object("load_cfg_mi")
        self.save_cfg_mi = self.builder.get_object("save_cfg_mi")
        self.load_dlg = None
        self.save_dlg = None
        self.load_save_dir = None
        if BUILD_PREFIX and os.path.exists("%s/data/procman" % BUILD_PREFIX):
            self.load_save_dir = "%s/data/procman" % BUILD_PREFIX

        # options menu
        self.is_observer_cmi = self.builder.get_object("is_observer_cmi")
        self.spawn_deputy_mi = self.builder.get_object("spawn_deputy_mi")
        self.terminate_spawned_deputy_mi = self.builder.get_object("terminate_spawned_deputy_mi")

        self.bot_procman_deputy_cmd = find_bot_procman_deputy_cmd()
        if not self.bot_procman_deputy_cmd:
            sys.stderr.write("Can't find bot-procman-deputy.  Spawn Deputy disabled")
            self.spawn_deputy_mi.set_sensitive(False)

        # commands menu
        self.start_cmd_mi = self.builder.get_object("start_cmd_mi")
        self.stop_cmd_mi = self.builder.get_object("stop_cmd_mi")
        self.printf_cmd_mi = self.builder.get_object("printf_cmd_mi")
        self.remove_cmd_mi = self.builder.get_object("remove_cmd_mi")
        self.edit_cmd_mi = self.builder.get_object("edit_cmd_mi")
        self.new_cmd_mi = self.builder.get_object("new_cmd_mi")

        # scripts menu
        self.abort_script_mi = self.builder.get_object("abort_script_mi")
        self.edit_script_mi = self.builder.get_object("edit_script_mi")
        self.remove_script_mi = self.builder.get_object("remove_script_mi")
        self.scripts_menu = self.builder.get_object("scripts_menu")
        self.edit_scripts_menu = self.builder.get_object("edit_scripts_menu")
        self.remove_scripts_menu = self.builder.get_object("remove_scripts_menu")

        vpane = self.builder.get_object("vpaned")

        sw = gtk.ScrolledWindow()
        sw.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        hpane = self.builder.get_object("hpaned")
        hpane.pack1(sw, resize = True)
        sw.add(self.cmds_tv)

        cmds_sel = self.cmds_tv.get_selection()
        cmds_sel.connect ("changed", self._on_cmds_selection_changed)

        # create a checkable item in the View menu for each column to toggle
        # its visibility in the treeview
        view_menu = self.builder.get_object("view_menu")
        for col in self.cmds_tv.get_columns():
            name = col.get_title ()
            col_cmi = gtk.CheckMenuItem(name)
            col_cmi.set_active(col.get_visible())
            def on_activate(cmi, col_):
                should_be_visible = cmi.get_active()
                if col_.get_visible() != should_be_visible:
                    col_.set_visible(should_be_visible)
                    if col_ == self.cmds_tv.columns[0]:
                        self.cmds_ts.set_populate_exec_with_group_name(not should_be_visible)
                        self.cmds_ts.repopulate()
            def on_visibility_changed(col_, param, cmi_):
                is_visible = col_.get_visible()
                if is_visible != cmi_.get_active():
                    cmi_.set_active(is_visible)
            col_cmi.connect("activate", on_activate, col)
            col.connect("notify::visible", on_visibility_changed, col_cmi)
            view_menu.append(col_cmi)

        # setup the hosts treeview
        self.hosts_ts = ht.SheriffHostModel(self.sheriff)
        self.hosts_tv = ht.SheriffHostTreeView(self.sheriff, self.hosts_ts)
        sw = gtk.ScrolledWindow ()
        sw.set_policy (gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        hpane.pack2 (sw, resize = False)
        sw.add (self.hosts_tv)

        gobject.timeout_add (1000, lambda *s: self.hosts_ts.update() or True)

        # stdout textview
        self.cmd_console = cc.SheriffCommandConsole(self.sheriff, self.lc)
        vpane.add2(self.cmd_console)

        # status bar
        self.statusbar = self.builder.get_object("statusbar")
        self.statusbar_context_script = self.statusbar.get_context_id("script")
        self.statusbar_context_main = self.statusbar.get_context_id("main")
        self.statusbar_context_script_msg = None

        config_dir = os.path.join(glib.get_user_config_dir(), "procman-sheriff")
        if not os.path.exists(config_dir):
            os.makedirs(config_dir)
        self.config_fname = os.path.join(config_dir, "config")
        self.load_settings()

        self.window.show_all()

    def on_preferences_mi_activate(self, *args):
        sd.do_preferences_dialog(self, self.window)

    def on_quit_requested(self, *args):
        gtk.main_quit()

    def on_start_cmd_mi_activate(self, *args):
        self.cmds_tv._start_selected_commands()

    def on_stop_cmd_mi_activate(self, *args):
        self.cmds_tv._stop_selected_commands()

    def on_restart_cmd_mi_activate(self, *args):
        self.cmds_tv._restart_selected_commands()

    def on_printf_cmd_mi_activate(self, *args):
        self.cmds_tv._printf_selected_commands()

    def on_remove_cmd_mi_activate(self, *args):
        self.cmds_tv._remove_selected_commands()

    def on_edit_cmd_mi_activate(self, *args):
        self.cmds_tv._edit_selected_command()

    def on_new_cmd_mi_activate(self, *args):
        sd.do_add_command_dialog(self.sheriff, self.cmds_ts, self.window)

    def cleanup(self):
        self._terminate_spawned_deputy()
        self.save_settings()

    def load_settings(self):
        if not os.path.exists(self.config_fname):
            return
        try:
            d = pickle.load(open(self.config_fname, "r"))
        except Exception, err:
            print err
            return

        self.cmds_tv.load_settings(d)
        self.cmd_console.load_settings(d)
        self.hosts_tv.load_settings(d)

    def save_settings(self):
        config_dir = os.path.join(glib.get_user_config_dir(), "procman-sheriff")
        if not os.path.exists(config_dir):
            os.makedirs(config_dir)
        self.config_fname = os.path.join(config_dir, "config")
        d = {}

        self.cmds_tv.save_settings(d)
        self.cmd_console.save_settings(d)
        self.hosts_tv.save_settings(d)

        try:
            pickle.dump(d, open(self.config_fname, "w"))
        except Exception, err:
            print err

    def _do_repopulate(self):
        self.cmds_ts.repopulate()
        self.cmds_update_scheduled = False

    def _schedule_cmds_update(self, *unused):
        if not self.cmds_update_scheduled:
            gobject.timeout_add(100, self._do_repopulate)
        return True

    def _terminate_spawned_deputy(self):
        if self.spawned_deputy:
            try:
                self.spawned_deputy.terminate()
            except AttributeError: # python 2.4, 2.5 don't have Popen.terminate()
                os.kill(self.spawned_deputy.pid, signal.SIGTERM)
                self.spawned_deputy.wait()
        self.spawned_deputy = None

    def _check_spawned_deputy(self):
        if not self.spawned_deputy:
            return

        self.spawned_deputy.poll()
        if self.spawned_deputy.returncode is None:
            return

        returncode_msgs = { \
                0 : "Terminated",
                1 : "OS or other networking error",
                2 : "Conflicting deputy with same name already exists" }

        msg = returncode_msgs.get(self.spawned_deputy.returncode, "Unknown error")

        self.spawn_deputy_mi.set_sensitive(True)
        self.terminate_spawned_deputy_mi.set_sensitive(False)
        self.spawned_deputy = None

        dialog = gtk.MessageDialog(self.window, 0, gtk.MESSAGE_ERROR,
                gtk.BUTTONS_OK, "Spawned deputy exited prematurely: %s" % msg)
        dialog.run()
        dialog.destroy()

    def set_observer (self, is_observer):
        self.sheriff.set_observer (is_observer)

        self._update_menu_item_sensitivities ()

        if is_observer: self.window.set_title ("Procman Observer")
        else: self.window.set_title ("Procman Sheriff")

        if self.is_observer_cmi != is_observer:
            self.is_observer_cmi.set_active (is_observer)

    def run_script(self, menuitem, script, script_done_action = None):
        self.script_done_action = script_done_action
        errors = self.sheriff.execute_script(script)
        if errors:
            msgdlg = gtk.MessageDialog (self.window,
                    gtk.DIALOG_MODAL|gtk.DIALOG_DESTROY_WITH_PARENT,
                    gtk.MESSAGE_ERROR, gtk.BUTTONS_CLOSE,
                    "Script failed to run.  Errors detected:\n" + \
                    "\n".join(errors))
            msgdlg.run ()
            msgdlg.destroy ()

    def _on_script_started(self, script):
        self._update_menu_item_sensitivities()
        cid = self.statusbar_context_script
        if self.statusbar_context_script_msg is not None:
            self.statusbar.pop(cid)
            self.statusbar_context_script_msg = self.statusbar.push(cid, \
                    "Script %s: start" % script.name)

    def _on_script_action_executing(self, script, action):
        cid = self.statusbar_context_script
        self.statusbar.pop(cid)
        msg = "Action: %s" % str(action)
        self.statusbar_context_script_msg = self.statusbar.push(cid, msg)

    def _on_script_finished(self, script):
        self._update_menu_item_sensitivities()
        cid = self.statusbar_context_script
        self.statusbar.pop(cid)
        self.statusbar_context_script_msg = self.statusbar.push(cid, \
                "Script %s: finished" % script.name)
        def _remove_msg_func(msg_id):
            return lambda *s: msg_id == self.statusbar_context_script_msg and self.statusbar.pop(cid)
        gobject.timeout_add(6000, _remove_msg_func(self.statusbar_context_script_msg))
        if self.script_done_action == "exit":
            gtk.main_quit()
        elif self.script_done_action == "observe":
            self.set_observer(True)

    def on_abort_script_mi_activate(self, menuitem):
        self.sheriff.abort_script()

    def on_new_script_mi_activate(self, menuitem):
        sd.do_add_script_dialog(self.sheriff, self.window)

    def _get_script_menuitem(self, menu, script, name_parts, create):
        assert name_parts
        partname = name_parts[0]
        if len(name_parts) == 1:
            insert_point = 0
            for i, smi in enumerate(menu.get_children()):
                other_script = smi.get_data("sheriff-script")
                if other_script is script:
                    return smi
                if other_script is None:
                    break
                if other_script.name < script.name:
                    insert_point += 1
            if create:
                mi = gtk.MenuItem(partname, use_underline=False)
                mi.set_data("sheriff-script", script)
                menu.insert(mi, insert_point)
                mi.show()
                return mi
            return None
        else:
            insert_point = 0
            for i, smi in enumerate(menu.get_children()):
                if not smi.get_data("sheriff-script-submenu"):
                    continue
                submenu_name = smi.get_label()
                if submenu_name == partname:
                    return self._get_script_menuitem(smi.get_submenu(), script, name_parts[1:], create)
                elif submenu_name < partname:
                    insert_point = i

            if create:
                smi = gtk.MenuItem(partname)
                submenu = gtk.Menu()
                smi.set_submenu(submenu)
                smi.set_data("sheriff-script-submenu", True)
                menu.insert(smi, insert_point)
                smi.show()
                return self._get_script_menuitem(submenu, script, name_parts[1:], create)

    def _remove_script_menuitems(self, menu, script, name_parts):
        assert name_parts
        partname = name_parts[0]

        if len(name_parts) == 1:
            for smi in menu.get_children():
                if script == smi.get_data("sheriff-script"):
                    menu.remove(smi)
                    return
        else:
            for smi in menu.get_children():
                if not smi.get_data("sheriff-script-submenu"):
                    continue
                submenu_name = smi.get_label()
                submenu = smi.get_submenu()
                if submenu_name == partname:
                    self._remove_script_menuitems(submenu, script, name_parts[1:])
                    if not submenu.get_children():
                        smi.remove_submenu()
                        menu.remove(smi)
                    return

    def _maybe_add_script_menu_item(self, script):
        name_parts = split_script_name(script.name)

        # make menu items for executing, editing, and removing the script
        run_mi = self._get_script_menuitem(self.scripts_menu, script, name_parts, True)
        run_mi.connect("activate", self.run_script, script)

        edit_mi = self._get_script_menuitem(self.edit_scripts_menu, script, name_parts, True)
        edit_mi.connect("activate",
                lambda mi: sd.do_edit_script_dialog(self.sheriff, self.window, script))

        remove_mi = self._get_script_menuitem(self.remove_scripts_menu, script, name_parts, True)
        remove_mi.connect("activate",
                lambda mi: self.sheriff.remove_script(mi.get_data("sheriff-script")))

        self.edit_script_mi.set_sensitive(True)
        self.remove_script_mi.set_sensitive(True)

    def _on_script_added(self, script):
        self._maybe_add_script_menu_item(script)

    def _on_script_removed(self, script):
        name_parts = split_script_name(script.name)
        for menu in [ self.scripts_menu, self.edit_scripts_menu, self.remove_scripts_menu ]:
            self._remove_script_menuitems(menu, script, name_parts)

        if not self.sheriff.get_scripts():
            self.edit_script_mi.set_sensitive(False)
            self.remove_script_mi.set_sensitive(False)

    def load_config(self, cfg):
        self.sheriff.load_config(cfg, False)

    # GTK signal handlers
    def on_load_cfg_mi_activate(self, *args):
        if not self.load_dlg:
            self.load_dlg = gtk.FileChooserDialog ("Load Config", self.window,
                    buttons = (gtk.STOCK_OPEN, gtk.RESPONSE_ACCEPT,
                        gtk.STOCK_CANCEL, gtk.RESPONSE_REJECT))
        if self.load_save_dir:
            self.load_dlg.set_current_folder(self.load_save_dir)
        if gtk.RESPONSE_ACCEPT == self.load_dlg.run ():
            self.config_filename = self.load_dlg.get_filename ()
            self.load_save_dir = os.path.dirname(self.config_filename)
            try:
                cfg = sheriff_config.config_from_filename (self.config_filename)
            except Exception:
                msgdlg = gtk.MessageDialog (self.window,
                        gtk.DIALOG_MODAL|gtk.DIALOG_DESTROY_WITH_PARENT,
                        gtk.MESSAGE_ERROR, gtk.BUTTONS_CLOSE,
                        traceback.format_exc ())
                msgdlg.run ()
                msgdlg.destroy ()
            else:
                self.load_config (cfg)
        self.load_dlg.hide()
        self.load_dlg.destroy()
        self.load_dlg = None

    def on_save_cfg_mi_activate(self, *args):
        if not self.save_dlg:
            self.save_dlg = gtk.FileChooserDialog ("Save Config", self.window,
                    action = gtk.FILE_CHOOSER_ACTION_SAVE,
                    buttons = (gtk.STOCK_SAVE, gtk.RESPONSE_ACCEPT,
                        gtk.STOCK_CANCEL, gtk.RESPONSE_REJECT))
        if self.load_save_dir:
            self.save_dlg.set_current_folder(self.load_save_dir)
        if self.config_filename is not None:
            self.save_dlg.set_filename (self.config_filename)
        if gtk.RESPONSE_ACCEPT == self.save_dlg.run ():
            self.config_filename = self.save_dlg.get_filename ()
            self.load_save_dir = os.path.dirname(self.config_filename)
            try:
                self.sheriff.save_config (file (self.config_filename, "w"))
            except IOError, e:
                msgdlg = gtk.MessageDialog (self.window,
                        gtk.DIALOG_MODAL|gtk.DIALOG_DESTROY_WITH_PARENT,
                        gtk.MESSAGE_ERROR, gtk.BUTTONS_CLOSE, str (e))
                msgdlg.run ()
                msgdlg.destroy ()
        self.save_dlg.hide ()
        self.save_dlg.destroy()
        self.save_dlg = None

    def on_is_observer_cmi_toggled(self, menu_item):
        self.set_observer(menu_item.get_active ())

    def on_spawn_deputy_mi_activate(self, *args):
        print("Spawn deputy!")
        self._terminate_spawned_deputy()
        args = [ self.bot_procman_deputy_cmd, "-n", "localhost" ]
        self.spawned_deputy = subprocess.Popen(args)
        # TODO disable
        self.spawn_deputy_mi.set_sensitive(False)
        self.terminate_spawned_deputy_mi.set_sensitive(True)

    def on_terminate_spawned_deputy_mi_activate(self, *args):
        print("Terminate!")
        self._terminate_spawned_deputy()
        self.spawn_deputy_mi.set_sensitive(True)
        self.terminate_spawned_deputy_mi.set_sensitive(False)

    def _update_menu_item_sensitivities (self):
        # enable/disable menu options based on sheriff state and user selection
        selected_cmds = self.cmds_tv.get_selected_commands ()
        script_active = self.sheriff.get_active_script() is not None
        can_modify = len(selected_cmds) > 0 and \
                not self.sheriff.is_observer () and \
                not script_active
        can_add_load = not self.sheriff.is_observer () and \
                not script_active

        self.start_cmd_mi.set_sensitive (can_modify)
        self.stop_cmd_mi.set_sensitive (can_modify)
        #self.restart_cmd_mi.set_sensitive (can_modify)
        self.printf_cmd_mi.set_sensitive (can_modify)
        self.remove_cmd_mi.set_sensitive (can_modify)
        self.edit_cmd_mi.set_sensitive (can_modify)

        self.new_cmd_mi.set_sensitive (can_add_load)
        self.load_cfg_mi.set_sensitive (can_add_load)

        self.abort_script_mi.set_sensitive(script_active)

    def _on_cmds_selection_changed (self, selection):
        selected_cmds = self.cmds_tv.get_selected_commands ()
        if len (selected_cmds) == 1:
            self.cmd_console.show_command_buffer(list(selected_cmds)[0])
        elif len (selected_cmds) == 0:
            self.cmd_console.show_sheriff_buffer()
        self._update_menu_item_sensitivities ()

    def _maybe_send_orders (self):
        self._check_spawned_deputy()
        if not self.sheriff.is_observer ():
            self.sheriff.send_orders ()
        return True

    # LCM handlers
    def on_procman_orders (self, channel, data):
        msg = orders_t.decode (data)
        if not self.sheriff.is_observer () and \
                self.sheriff.get_name() != msg.sheriff_name:
            # detected the presence of another sheriff that is not this one.
            # self-demote to prevent command thrashing
            self.set_observer (True)

            self.statusbar.push (self.statusbar.get_context_id ("main"),
                    "WARNING: multiple sheriffs detected!  Switching to observer mode");
            gobject.timeout_add (6000,
                    lambda *s: self.statusbar.pop (self.statusbar.get_context_id ("main")))

class SheriffHeadless(object):
    def __init__(self, lc, config, spawn_deputy, script_name, script_done_action):
        self.sheriff = sheriff.Sheriff(lc)
        self.spawn_deputy = spawn_deputy
        self.spawned_deputy = None
        self.config = config
        self.script_name = script_name
        self.script = None
        self.mainloop = None
        self.lc = lc
        self.lc.subscribe ("PMD_ORDERS", self._on_procman_orders)
        if script_done_action is None:
            self.script_done_action = "exit"
        else:
            self.script_done_action = script_done_action

    def _terminate_spawned_deputy(self):
        if not self.spawned_deputy:
            return

        print("Terminating local deputy..")
        try:
            self.spawned_deputy.terminate()
        except AttributeError: # python 2.4, 2.5 don't have Popen.terminate()
            os.kill(self.spawned_deputy.pid, signal.SIGTERM)
            self.spawned_deputy.wait()
        self.spawned_deputy = None

    def _start_script(self):
        if not self.script:
            return False
        print("Running script %s" % self.script_name)
        errors = self.sheriff.execute_script(self.script)
        if errors:
            print("Script failed to run.  Errors detected:\n" + "\n".join(errors))
            self._terminate_spawned_deputy()
            sys.exit(1)
        return False

    def _on_script_finished(self, *args):
        if self.script_done_action == "exit":
            print("Script \"%s\" finished.  Exiting" % self.script_name)
            self.mainloop.quit()
        elif self.script_done_action == "observe":
            print("Script \"%s\" finished.  Self-demoting to observer" % self.script_name)
            self.sheriff.set_observer(True)

    def _maybe_send_orders(self):
        if not self.sheriff.is_observer():
            self.sheriff.send_orders()
        return True

    def _on_procman_orders(self, channel, data):
        if self.sheriff.is_observer():
            return

        msg = orders_t.decode(data)
        if self.sheriff.name != msg.sheriff_name:
            # detected the presence of another sheriff that is not this one.
            # self-demote to prevent command thrashing
            self.sheriff.set_observer(True)

    def run(self):
        self.mainloop = glib.MainLoop()

        # parse the config file
        if self.config is not None:
            self.sheriff.load_config(self.config, False)

        # start a local deputy?
        if self.spawn_deputy:
            bot_procman_deputy_cmd = find_bot_procman_deputy_cmd()
            args = [ bot_procman_deputy_cmd, "-n", "localhost" ]
            if not bot_procman_deputy_cmd:
                sys.stderr.write("Can't find bot-procman-deputy.")
                sys.exit(1)
            self.spawned_deputy = subprocess.Popen(args)
        else:
            self.spawned_deputy = None

        # run a script
        if self.script_name:
            self.script = self.sheriff.get_script(self.script_name)
            if not self.script:
                print "No such script: %s" % self.script_name
                self._terminate_spawned_deputy()
                sys.exit(1)
            errors = self.sheriff.check_script_for_errors(self.script)
            if errors:
                print "Unable to run script.  Errors were detected:\n\n"
                print "\n    ".join(errors)
                self._terminate_spawned_deputy()
                sys.exit(1)

            self.sheriff.script_finished.connect(self._on_script_finished)

            # delay script execution by 200 ms.
            gobject.timeout_add(200, self._start_script)

        signal.signal(signal.SIGINT, lambda *s: mainloop.quit())
        signal.signal(signal.SIGTERM, lambda *s: mainloop.quit())
        signal.signal(signal.SIGHUP, lambda *s: mainloop.quit())
        gobject.timeout_add(1000, self._maybe_send_orders)

        try:
            self.mainloop.run()
        except KeyboardInterrupt:
            pass
        print("Sheriff terminating..")
        self._terminate_spawned_deputy()
        return 0

def usage():
    sys.stdout.write(
"""usage: %s [options] [<procman_config_file> [<script_name>]]

Process management operating console.

Options:
  -l, --lone-ranger   Automatically run a deputy within the sheriff process
                      This deputy terminates with the sheriff, along with
                      all the commands it hosts.

  -n, --no-gui        Runs in headless mode (no GUI).

  -o, --observer      Runs in observer mode on startup.  This prevents the
                      sheriff from sending any commands, and is useful for
                      monitoring existing procman sheriff and/or deputy
                      instances.

  --on-script-complete <exit|observe>
                      Only valid if a script is specified.  If set to "exit",
                      then the sheriff exits when the script is done executing.
                      If set to "observe", then the sheriff self-demotes to
                      observer mode.

  -h, --help          Shows this help text

If <procman_config_file> is specified, then the sheriff tries to load
deputy commands from the file.

If <script_name> is additionally specified, then the sheriff executes the
named script once the config file is loaded.

""" % os.path.basename(sys.argv[0]))
    sys.exit(1)

def main():
    try:
        opts, args = getopt.getopt( sys.argv[1:], 'hlon',
                ['help','lone-ranger', 'on-script-complete=', 'no-gui', 'observer'] )
    except getopt.GetoptError:
        usage()
        sys.exit(2)

    spawn_deputy = False
    use_gui = True
    script_done_action = None
    observer = False

    for optval, argval in opts:
        if optval in [ '-l', '--lone-ranger' ]:
            spawn_deputy = True
        elif optval in [ '-n', '--no-gui' ]:
            use_gui = False
        elif optval in [ '-o', '--observer' ]:
            observer = True
        elif optval in [ '--on-script-complete' ]:
            script_done_action = argval
            if argval not in [ "exit", "observe" ]:
                usage()
        elif optval in [ '-h', '--help' ]:
            usage()

    cfg = None
    script_name = None
    if len(args) > 0:
        try:
            cfg = sheriff_config.config_from_filename(args[0])
        except Exception, xcp:
            print "Unable to load config file."
            print xcp
            sys.exit(1)
    if len(args) > 1:
        script_name = args[1]

    if observer:
        if cfg:
            print "Loading a config file is not allowed when starting in observer mode."
            sys.exit(1)
        if not use_gui:
            print "Refusing to start an observer without a gui -- that would be useless."
            sys.exit(1)
        if spawn_deputy:
            print "Lone ranger mode and observer mode are mutually exclusive."
            sys.exit(1)

    lc = LCM()
    def handle(*a):
        try:
            lc.handle()
        except Exception:
            traceback.print_exc()
        return True
    gobject.io_add_watch(lc, gobject.IO_IN, handle)

    if use_gui:
        gui = SheriffGtk(lc)
        if observer:
            gui.set_observer(True)
        if spawn_deputy:
            gui.on_spawn_deputy_mi_activate()
        if cfg is not None:
            gui.load_config(cfg)
            gui.load_save_dir = os.path.dirname(args[0])

        if script_name:
            script = gui.sheriff.get_script(script_name)
            if not script:
                print "No such script: %s" % script_name
                gui._terminate_spawned_deputy()
                sys.exit(1)
            errors = gui.sheriff.check_script_for_errors(script)
            if errors:
                print "Unable to run script.  Errors were detected:\n\n"
                print "\n    ".join(errors)
                gui._terminate_spawned_deputy()
                sys.exit(1)
            gobject.timeout_add(200, lambda *s: gui.run_script(None, script, script_done_action))

        signal.signal(signal.SIGINT, lambda *s: gtk.main_quit())
        signal.signal(signal.SIGTERM, lambda *s: gtk.main_quit())
        signal.signal(signal.SIGHUP, lambda *s: gtk.main_quit())
        try:
            gtk.main ()
        except KeyboardInterrupt:
            print("Exiting")
        gui.cleanup()
    else:
        if not script_name:
            print("No script specified and running in headless mode.  Exiting")
            sys.exit(1)
        SheriffHeadless(lc, cfg, spawn_deputy, script_name, script_done_action).run()

if __name__ == "__main__":
    main()
