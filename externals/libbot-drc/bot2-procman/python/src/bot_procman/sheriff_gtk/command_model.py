import gobject
import gtk

import bot_procman.sheriff as sheriff

COL_CMDS_TV_OBJ, \
COL_CMDS_TV_EXEC, \
COL_CMDS_TV_FULL_GROUP, \
COL_CMDS_TV_COMMAND_ID, \
COL_CMDS_TV_HOST, \
COL_CMDS_TV_STATUS_ACTUAL, \
COL_CMDS_TV_CPU_USAGE, \
COL_CMDS_TV_MEM_VSIZE, \
COL_CMDS_TV_AUTO_RESPAWN, \
NUM_CMDS_ROWS = range(10)

class SheriffCommandModel(gtk.TreeStore):
    def __init__(self, _sheriff):
        super(SheriffCommandModel, self).__init__( \
                gobject.TYPE_PYOBJECT,
                gobject.TYPE_STRING, # command executable
                gobject.TYPE_STRING, # group name
                gobject.TYPE_STRING, # display name
                gobject.TYPE_STRING, # host name
                gobject.TYPE_STRING, # status actual
                gobject.TYPE_STRING, # CPU usage
                gobject.TYPE_INT,    # memory vsize
                gobject.TYPE_BOOLEAN,# auto-respawn
                )

        self.sheriff = _sheriff
        self.group_row_references = {}
        self.populate_exec_with_group_name = False

        self.set_sort_column_id(COL_CMDS_TV_COMMAND_ID, gtk.SORT_ASCENDING)

    def _find_or_make_group_row_reference(self, group_name):
        if not group_name:
            return None
        if group_name in self.group_row_references:
            return self.group_row_references[group_name]
        else:
            name_parts = group_name.split("/")
            if len(name_parts) > 1:
                parent_name = "/".join(name_parts[:-1])
                parent_row_ref = self._find_or_make_group_row_reference(parent_name)
                parent = self.get_iter(parent_row_ref.get_path())
            else:
                parent = None

            if self.populate_exec_with_group_name:
                exec_val = name_parts[-1]
            else:
                exec_val = ""

            new_row = (None,                    # COL_CMDS_TV_OBJ
                      exec_val,                 # COL_CMDS_TV_EXEC
                      group_name,               # COL_CMDS_TV_FULL_GROUP
                      name_parts[-1],           # COL_CMDS_TV_COMMAND_ID
                      "",                       # COL_CMDS_TV_HOST
                      "",                       # COL_CMDS_TV_STATUS_ACTUAL
                      "",                       # COL_CMDS_TV_CPU_USAGE
                      0,                        # COL_CMDS_TV_MEM_VSIZE
                      False,                    # COL_CMDS_TV_AUTO_RESPAWN
                      )
            ts_iter = self.append(parent, new_row)
            trr = gtk.TreeRowReference (self, self.get_path (ts_iter))
            self.group_row_references[group_name] = trr
            return trr

    def get_known_group_names (self):
        return self.group_row_references.keys()

    def set_populate_exec_with_group_name(self, val):
        self.populate_exec_with_group_name = val

    def _delete_group_row_reference(self, trr):
        model_iter = self.get_iter(trr.get_path())
        group_name = self.get_value(model_iter, COL_CMDS_TV_FULL_GROUP)
        del self.group_row_references[group_name]
        self.remove(model_iter)

    def _is_group_row(self, model_iter):
        return self.iter_to_command(model_iter) is None

    def _update_cmd_row(self, model_rr, cmd_deps, to_reparent):
        path = model_rr.get_path()
        model_iter = self.get_iter(path)
        cmd = self.iter_to_command(model_iter)
        cpu_str = "%.2f" % (cmd.cpu_usage * 100)
        mem_usage = int(cmd.mem_vsize_bytes / 1024)

        if cmd.command_id.strip():
            command_id = cmd.command_id
        else:
            command_id = "<unnamed>"

        self.set(model_iter,
                COL_CMDS_TV_EXEC, cmd.exec_str,
                COL_CMDS_TV_COMMAND_ID, command_id,
                COL_CMDS_TV_STATUS_ACTUAL, cmd.status (),
                COL_CMDS_TV_HOST, cmd_deps[cmd].name,
                COL_CMDS_TV_CPU_USAGE, cpu_str,
                COL_CMDS_TV_MEM_VSIZE, mem_usage,
                COL_CMDS_TV_AUTO_RESPAWN, cmd.auto_respawn)

        # get a row reference to the model since
        # adding a group may invalidate the iterators
        model_rr = gtk.TreeRowReference(self, path)

        # check that the command is in the correct group in the
        # treemodel
        correct_grr = self._find_or_make_group_row_reference(cmd.group)
        correct_parent_iter = None
        correct_parent_path = None
        actual_parent_path = None
        if correct_grr and correct_grr.get_path() is not None:
            correct_parent_iter = self.get_iter(correct_grr.get_path())
        actual_parent_iter = self.iter_parent(self.get_iter(model_rr.get_path())) # use the model_rr in case model_iter was invalidated

        if correct_parent_iter:
            correct_parent_path = self.get_path(correct_parent_iter)
        if actual_parent_iter:
            actual_parent_path = self.get_path(actual_parent_iter)

        if correct_parent_path != actual_parent_path:
            # schedule the command to be moved
            to_reparent.append((model_rr, correct_grr))
#                print "moving %s (%s) (%s)" % (cmd.name,
#                        correct_parent_path, actual_parent_path)

    def _update_group_row(self, group_rr, cmd_deps):
        model_iter = self.get_iter(group_rr.get_path())
        # row represents a procman group
        children = self.get_group_row_child_commands_recursive(model_iter)
        if not children:
            return

        # aggregate command status
        statuses = [ child.status () for child in children ]
        stopped_statuses = [sheriff.STOPPED_OK, sheriff.STOPPED_ERROR]
        if all ([s == statuses[0] for s in statuses]):
            status_str = statuses[0]
        elif all ([s in stopped_statuses for s in statuses]):
            status_str = "Stopped (Mixed)"
        else:
            status_str = "Mixed"

        # aggregate host information
        child_deps = set([ cmd_deps[child] for child in children \
                if child in cmd_deps ])
        if len(child_deps) == 1:
            dep_str = child_deps.pop().name
        else:
            dep_str = "Mixed"

        # aggregate CPU and memory usage
        cpu_total = sum ([cmd.cpu_usage for cmd in children])
        mem_total = sum ([cmd.mem_vsize_bytes / 1024 \
                for cmd in children])
        cpu_str = "%.2f" % (cpu_total * 100)

        # display group name in command column?
        if self.populate_exec_with_group_name:
            exec_val = self.get_value(model_iter, COL_CMDS_TV_COMMAND_ID)
        else:
            exec_val = ""

        self.set (model_iter,
                COL_CMDS_TV_STATUS_ACTUAL, status_str,
                COL_CMDS_TV_EXEC, exec_val,
                COL_CMDS_TV_HOST, dep_str,
                COL_CMDS_TV_CPU_USAGE, cpu_str,
                COL_CMDS_TV_MEM_VSIZE, mem_total)

    def _dispatch_row_changes(self, model, path, model_iter, user_data):
        cmds_to_add, cmd_rows_to_remove, cmds_rows_to_update, group_rows_to_update = user_data
        cmd = self.iter_to_command(model_iter)
        if cmd:
            if cmd in cmds_to_add:
                cmds_rows_to_update.append(gtk.TreeRowReference(model, path))
                cmds_to_add.remove(cmd)
            else:
                cmd_rows_to_remove.append(gtk.TreeRowReference(model, path))
        else:
            group_rows_to_update.append(gtk.TreeRowReference(model, path))

    def repopulate(self):
        cmds_to_add = set()
        cmd_deps = {}
        for deputy in self.sheriff.get_deputies ():
            for cmd in deputy.get_commands ():
                cmd_deps [cmd] = deputy
                cmds_to_add.add (cmd)
        cmd_rows_to_remove = []
        cmd_rows_to_reparent = []
        cmds_rows_to_update = []
        group_rows_to_update = []

        # Figure out which rows should be added/updated/removed etc...
        # On return, the cmds_to_add set will
        # contain commands that were not updated (i.e., commands that need to
        # be added into the model)
        self.foreach(self._dispatch_row_changes,
                (cmds_to_add, cmd_rows_to_remove, cmds_rows_to_update, group_rows_to_update))

        # update the command rows that should be updated
        for trr in cmds_rows_to_update:
            self._update_cmd_row(trr, cmd_deps, cmd_rows_to_reparent)

        # update the group rows that should be updated
        for trr in group_rows_to_update:
            self._update_group_row(trr, cmd_deps)


        # reparent rows that are in the wrong group
        for trr, newparent_rr in cmd_rows_to_reparent:
            orig_iter = self.get_iter(trr.get_path ())
            rowdata = self.get (orig_iter, *range(NUM_CMDS_ROWS))
            self.remove (orig_iter)

            newparent_iter = None
            if newparent_rr:
                newparent_iter = self.get_iter(newparent_rr.get_path())
            self.append(newparent_iter, rowdata)

        # remove rows that have been marked for deletion
        for trr in cmd_rows_to_remove:
            self.remove(self.get_iter(trr.get_path()))

        # remove group rows with no children
        groups_to_remove = []
        def _check_for_lonely_groups(model, path, model_iter, user_data):
            is_group = self._is_group_row(model_iter)
            if is_group and not model.iter_has_child(model_iter):
                groups_to_remove.append(gtk.TreeRowReference(model, path))
        self.foreach(_check_for_lonely_groups, None)
        for trr in groups_to_remove:
            self._delete_group_row_reference(trr)

        # create new rows for new commands
        for cmd in cmds_to_add:
            deputy = cmd_deps[cmd]
            parent = self._find_or_make_group_row_reference(cmd.group)

            new_row = (cmd,        # COL_CMDS_TV_OBJ
                cmd.exec_str,      # COL_CMDS_TV_EXEC
                "",                # COL_CMDS_TV_FULL_GROUP
                cmd.command_id,    # COL_CMDS_TV_COMMAND_ID
                deputy.name,       # COL_CMDS_TV_HOST
                cmd.status (),     # COL_CMDS_TV_STATUS_ACTUAL
                "0",               # COL_CMDS_TV_CPU_USAGE
                0,                 # COL_CMDS_TV_MEM_VSIZE
                cmd.auto_respawn,  # COL_CMDS_TV_AUTO_RESPAWN
                )
            if parent:
                self.append(self.get_iter(parent.get_path()), new_row)
            else:
                self.append(None, new_row)

    def rows_to_commands(self, rows):
        col = COL_CMDS_TV_OBJ
        selected = set()
        for path in rows:
            cmds_iter = self.get_iter (path)
            cmd = self.get_value(cmds_iter, col)
            if cmd:
                selected.add(cmd)
            else:
                for child in self.get_group_row_child_commands_recursive(cmds_iter):
                    selected.add(child)
        return selected

    def iter_to_command(self, model_iter):
        return self.get_value(model_iter, COL_CMDS_TV_OBJ)

    def path_to_command(self, path):
        return self.iter_to_command(self.get_iter(path))

    def get_group_row_child_commands_recursive(self, group_iter):
        child_iter = self.iter_children(group_iter)
        children = []
        while child_iter:
            child_cmd = self.iter_to_command(child_iter)
            if child_cmd:
                children.append (child_cmd)
            else:
                children += self.get_group_row_child_commands_recursive(child_iter)
            child_iter = self.iter_next(child_iter)
        return children
