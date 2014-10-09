from bot_procman.sheriff_config import ScriptNode, WaitStatusActionNode, WaitMsActionNode, StartStopRestartActionNode, RunScriptActionNode, escape_str

class StartStopRestartAction(object):
    """Script action to start, stop, or restart a command or group.

    \ingroup python_api

    """
    def __init__(self, action_type, ident_type, ident, wait_status):
        assert action_type in ["start", "stop", "restart"]
        assert ident_type in [ "everything", "group", "cmd" ]
        self.action_type = action_type
        self.ident_type = ident_type
        self.wait_status = wait_status
        if self.ident_type == "everything":
            self.ident = None
        else:
            self.ident = ident
            assert self.ident is not None

    def toScriptNode(self):
        return StartStopRestartActionNode(self.action_type,
                self.ident_type, self.ident, self.wait_status)

    def __str__(self):
        if self.ident_type == "everything":
            ident_str = self.ident_type
        else:
            ident_str = "%s \"%s\"" % (self.ident_type, escape_str(self.ident))
        if self.wait_status is not None:
            return "%s %s wait \"%s\";" % (self.action_type,
                    ident_str, self.wait_status)
        else:
            return "%s %s;" % (self.action_type, ident_str)

class WaitMsAction(object):
    """Script action to wait a fixed number of milliseconds.

    \ingroup python_api

    """
    def __init__(self, delay_ms):
        self.delay_ms = delay_ms
        self.action_type = "wait_ms"

    def toScriptNode(self):
        return WaitMsAction(self.delay_ms)

    def __str__(self):
        return "wait ms %d;" % self.delay_ms

class WaitStatusAction(object):
    """Script action to wait for a command or group to change status.

    \ingroup python_api

    """
    def __init__(self, ident_type, ident, wait_status):
        self.ident_type = ident_type
        self.ident = ident
        self.wait_status = wait_status
        self.action_type = "wait_status"

    def toScriptNode(self):
        return WaitStatusActionNode(self.ident_type,
                self.ident, self.wait_status)

    def __str__(self):
        return "wait %s \"%s\" status \"%s\";" % \
                (self.ident_type, escape_str(self.ident), self.wait_status)

class RunScriptAction(object):
    """Script action to run a subscript.

    \ingroup python_api

    """
    def __init__(self, script_name):
        self.script_name = script_name
        self.action_type = "run_script"

    def toScriptNode(self):
        return RunScriptActionNode(self.script_name)

    def __str__(self):
        return "run_script \"%s\";" % escape_str(self.script_name)

class SheriffScript(object):
    """A simple script that can be executed by the Sheriff.

    \ingroup python_api

    """
    def __init__(self, name):
        self.name = name
        self.actions = []

    def add_action(self, action):
        self.actions.append(action)

    def toScriptNode(self):
        node = ScriptNode(self.name)
        for action in self.actions:
            node.add_action(action.toScriptNode())
        return node

    def __str__(self):
        val = "script \"%s\" {" % escape_str(self.name)
        for action in self.actions:
            val = val + "\n    " + str(action)
        val = val + "\n}\n"
        return val

    @staticmethod
    def from_script_node(node):
        script = SheriffScript(node.name)
        for action_node in node.actions:
            if action_node.action_type in [ "start", "stop", "restart" ]:
                action = StartStopRestartAction(action_node.action_type,
                        action_node.ident_type,
                        action_node.ident,
                        action_node.wait_status)
            elif action_node.action_type == "wait_ms":
                action = WaitMsAction(action_node.delay_ms)
            elif action_node.action_type == "wait_status":
                action = WaitStatusAction(action_node.ident_type,
                        action_node.ident,
                        action_node.wait_status)
            elif action_node.action_type == "run_script":
                action = RunScriptAction(action_node.script_name)
            else:
                raise ValueError("unrecognized action %s" % \
                        action_node.action_type)
            script.add_action(action)
        return script
