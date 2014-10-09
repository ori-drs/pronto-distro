TokIdentifier = "Identifier"
TokOpenStruct = "OpenStruct"
TokCloseStruct = "CloseStruct"
TokAssign = "Assign"
TokEndStatement = "EndStatement"
TokString = "String"
TokEOF = "EOF"
TokComment = "Comment"
TokInteger = "Integer"

class Token(object):
    def __init__ (self, type, val):
        self.type = type
        self.val = val

class ParseError (ValueError):
    def __init__ (self, lineno, line_pos, line_text, tokenval, msg):
        self.lineno = lineno
        self.offset = line_pos
        self.text = line_text
        self.token = tokenval
        self.msg = msg

    def __str__ (self):
        ntabs = self.text.count ("\t")
        tokenstr = ""
        if self.token is not None:
            tokenstr = "token %s" % self.token
        s = """%s

line %d col %s %s
%s
""" % (self.msg, self.lineno, self.offset, tokenstr, self.text)
        s += " " * (self.offset - ntabs - 1) + "\t" * ntabs + "^"
        return s

class Tokenizer(object):
    def __init__ (self, f):
        self.f = f
        self.unget_char = None
        self.line_pos = 0
        self.line_len = 0
        self.line_buf = ""
        self.line_num = 1
        self.tok_pos = 0
        self.prev_tok_pos = 0

    def _next_char (self):
        if self.unget_char is not None:
            c = self.unget_char
            self.unget_char = None
            return c
        else:
            if self.line_pos == self.line_len:
                self.line_buf = self.f.readline ()
                if not len (self.line_buf):
                    return ''
                self.line_len = len (self.line_buf)
                self.line_pos = 0

            c = self.line_buf[self.line_pos]
            self.line_pos += 1

        if c == '\n':
            self.line_num += 1
        return c

    def _ungetc (self, c):
        if not c: return
        self.unget_char = c

    def _unescape (self, c):
        d = { "n": "\n",
              "r": "\r",
              "t": "\t" }
        if c in d: return d[c]
        return c

    def next_token (self):
        c = self._next_char ()

        while c and c.isspace ():
            c = self._next_char ()
        if not c: return Token (TokEOF, "")

        self.prev_tok_pos = self.tok_pos
        self.tok_pos = self.line_pos

        simple_tokens = { \
                "=" : TokAssign,
                ";" : TokEndStatement,
                "{" : TokOpenStruct,
                "}" : TokCloseStruct
                }
        if c in simple_tokens:
            return Token (simple_tokens[c], c)

        tok_chars = [ c ]

        if c == "#":
            while True:
                c = self._next_char ()
                if not c or c == "\n":
                    return Token (TokComment, "".join (tok_chars))
                tok_chars.append (c)

        if c == "\"":
            tok_chars = []
            while True:
                c = self._next_char ()
                if c == "\n":
                    raise ParseError (self.line_num, self.tok_pos,
                        self.line_buf, None, "Unterminated string constant")
                if c == "\\":   c = self._unescape (self._next_char ())
                elif not c or c == "\"":
                    return Token (TokString, "".join (tok_chars))
                tok_chars.append (c)

        if c.isalpha () or c == "_":
            while True:
                c = self._next_char ()
                if not c.isalnum () and c not in "_-":
                    self._ungetc (c)
                    return Token (TokIdentifier, "".join (tok_chars))
                tok_chars.append (c)

        if c.isdigit():
            while True:
                c = self._next_char()
                if not c.isdigit():
                    self._ungetc(c)
                    return Token(TokInteger, "".join(tok_chars))
                tok_chars.append(c)

        raise ParseError (self.line_num, self.line_pos,
                self.line_buf, None, "Invalid character")

def escape_str(text):
    def escape_char(c):
        if c in r'\"':
            return '\\' + c
        return c

    return "".join([ escape_char(c) for c in text ])

class CommandNode(object):
    def __init__ (self):
        self.attributes = { \
                "exec" : None,
                "host" : None,
                "group" : "",
                "nickname" : "",
                "stop_signal" : 0,
                "stop_time_allowed" : 0
                }

    def to_config_string(self, indent = 0):
        s = "    " * indent
        lines = []
        nickname = self.attributes["nickname"]
        if len(nickname):
            lines.append (s + "cmd \"%s\" {" % escape_str(nickname))
        else:
            lines.append (s + "cmd {")
        pairs = self.attributes.items()
        pairs.sort()
        for key, val in pairs:
            if not val:
                continue
            if key in [ "group", "nickname" ]:
                continue
            lines.append (s + "    %s = \"%s\";" % (key, escape_str(val)))
        lines.append (s + "}")
        return ("\n".join (lines))

    def __str__ (self):
        return self.to_config_string()

class GroupNode(object):
    def __init__ (self, name):
        self.name = name
        self.commands = []
        self.subgroups = {}

    def add_command (self, command):
        command.attributes["group"] = self.name
        self.commands.append (command)

    def get_subgroup(self, name_parts, create=False):
        if not name_parts:
            return self
        next_name = name_parts[0]
        if next_name in self.subgroups:
            return self.subgroups[next_name].get_subgroup(name_parts[1:], create)
        elif create:
            subgroup = GroupNode(next_name)
            self.subgroups[next_name] = subgroup
            return subgroup.get_subgroup(name_parts[1:], create)
        else:
            raise KeyError()

    def to_config_string(self, indent=0):
        s = "    " * indent
        if self.name == "":
            assert indent == 0
            val = "\n".join([group.to_config_string(0) for group in self.subgroups.values()])
            val = val + "\n".join([cmd.to_config_string(0) for cmd in self.commands]) + "\n"
        else:
            val = "%sgroup \"%s\" {\n" % (s, self.name)
            val = val + "\n".join([group.to_config_string(indent+1) for group in self.subgroups.values()])
            val = val + "\n".join([cmd.to_config_string(indent+1) for cmd in self.commands])
            val = val + "\n%s}\n" % s
        return val

    def __str__ (self):
        return self.to_config_string(0)

class StartStopRestartActionNode(object):
    def __init__(self, action_type, ident_type, ident, wait_status):
        assert action_type in ["start", "stop", "restart"]
        assert ident_type in [ "everything", "group", "cmd" ]
        self.action_type = action_type
        self.ident_type = ident_type
        self.wait_status = wait_status
        assert wait_status in [None, "running", "stopped"]
        if self.ident_type == "everything":
            self.ident = None
        else:
            self.ident = ident
            assert self.ident is not None

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

class WaitMsActionNode(object):
    def __init__(self, delay_ms):
        self.delay_ms = delay_ms
        self.action_type = "wait_ms"

    def __str__(self):
        return "wait ms %d;" % self.delay_ms

class WaitStatusActionNode(object):
    def __init__(self, ident_type, ident, wait_status):
        self.ident_type = ident_type
        self.ident = ident
        self.wait_status = wait_status
        self.action_type = "wait_status"
        assert wait_status in ["running", "stopped"]

    def __str__(self):
        return "wait %s \"%s\" status \"%s\";" % \
                (self.ident_type, escape_str(self.ident), self.wait_status)

class RunScriptActionNode(object):
    def __init__(self, script_name):
        self.script_name = script_name
        self.action_type = "run_script"

    def __str__(self):
        return "run_script \"%s\";" % escape_str(self.script_name)

class ScriptNode(object):
    def __init__(self, name):
        self.name = name
        self.actions = []

    def add_action(self, action):
        assert action is not None
        assert hasattr(action, "action_type")
        self.actions.append(action)

    def __str__(self):
        val = "script \"%s\" {" % escape_str(self.name)
        for action in self.actions:
            val = val + "\n    " + str(action)
        val = val + "\n}\n"
        return val

class ConfigNode(object):
    def __init__ (self):
        self.scripts = {}
        self.root_group = GroupNode("")

    def _normalize_group_name(self, name):
        if not name.startswith("/"):
            name = "/" + name
        while name.find("//") >= 0:
            name = name.replace("//", "/")
        return name.rstrip("/")

    def has_group(self, group_name):
        name = self._normalize_group_name(group_name)
        parts = group_name.split("/")
        group = self.root_group
        assert group.name == parts[0]
        for part in parts:
            if part in group.subgroups:
                group = group.subgroups[part]
            else:
                return False
        return True

    def get_group (self, group_name, create=False):
        name = self._normalize_group_name(group_name)
        parts = name.split("/")
        group = self.root_group
        return group.get_subgroup(parts[1:], create)

    def add_script (self, script):
        assert script.name not in self.scripts
        self.scripts[script.name] = script

    def __str__ (self):
        val = self.root_group.to_config_string()
        scripts = sorted(self.scripts.values(), key=lambda s: s.name.lower())
        val += "\n" + "\n".join([str(script) for script in scripts])
        return val

class Parser:
    def __init__ (self):
        self.tokenizer = None
        self._cur_tok = None
        self._next_tok = None

    def _get_token (self):
        self._cur_tok = self._next_tok
        self._next_tok = self.tokenizer.next_token ()
        while self._next_tok.type == TokComment:
            self._next_tok = self.tokenizer.next_token ()
        return self._cur_tok

    def _eat_token (self, tok_type):
        if self._next_tok and self._next_tok.type == tok_type:
            self._get_token ()
            return True
        return False

    def _fail (self, msg):
        raise ParseError (self.tokenizer.line_num,
            self.tokenizer.prev_tok_pos,
            self.tokenizer.line_buf,
            self._cur_tok.val, msg)

    def _fail_next_token (self, msg):
        raise ParseError (self.tokenizer.line_num,
            self.tokenizer.tok_pos,
            self.tokenizer.line_buf,
            self._next_tok.val, msg)

    def _eat_token_or_fail(self, tok_type, err_msg):
        if not self._eat_token(tok_type):
            self._fail_next_token(err_msg)
        return self._cur_tok.val

    def _expect_identifier(self, identifier, err_msg = None):
        if err_msg is None:
            err_msg = "Expected %s" % identifier
        self._eat_token_or_fail(TokIdentifier, err_msg)
        if self._cur_tok.val != identifier:
            self._fail(err_msg)

    def _parse_identifier_one_of(self, valid_identifiers):
        err_msg = "Expected one of %s" % str(valid_identifiers)
        self._eat_token_or_fail(TokIdentifier, err_msg)
        result = self._cur_tok.val
        if result not in valid_identifiers:
            self._fail(err_msg)
        return result

    def _parse_string_one_of(self, valid_strings):
        err_msg = "Expected one of %s" % str(valid_strings)
        self._eat_token_or_fail(TokString, err_msg)
        result = self._cur_tok.val
        if result not in valid_strings:
            self._fail(err_msg)
        return result

    def _parse_string_or_fail(self):
        self._eat_token_or_fail(TokString, "Expected string literal")
        return self._cur_tok.val

    def _parse_command_param_list (self, cmd):
        if not self._eat_token (TokIdentifier):
            return
        attrib_name = self._cur_tok.val

        attribs = { "exec" : TokString,
                "host" : TokString,
                "auto_respawn" : TokString,
                "group" : TokString,
                "stop_signal" : TokInteger,
                "stop_time_allowed" : TokInteger }

        if attrib_name not in attribs:
            self._fail("Unrecognized attribute %s" % attrib_name)

        self._eat_token_or_fail(TokAssign, "Expected '='")
        if attribs[attrib_name] == TokString:
            attrib_val = self._parse_string_or_fail()
        else:
            self._eat_token_or_fail(TokInteger, "Expected integer literal")
            attrib_val = int(self._cur_tok.val)
        self._eat_token_or_fail(TokEndStatement, "Expected ';'")
        if attrib_name == "stop_signal" and attrib_val < 1:
            self._fail("Invalid value specified for command attribute 'stop_signal'")
        elif attrib_name == "stop_time_allowed" and attrib_val < 1:
            self._fail("Invalid value specified for command attribute 'stop_time_allwoed'")
        cmd.attributes[attrib_name] = attrib_val

        return self._parse_command_param_list (cmd)

    def _parse_command (self):
        cmd = CommandNode ()
        if self._eat_token(TokString):
            cmd.attributes["nickname"] = self._cur_tok.val
            if "/" in self._cur_tok.val:
                self._fail("'/' character not allowed in command name")
        self._eat_token_or_fail (TokOpenStruct, "Expected '{'")
        self._parse_command_param_list (cmd)
        self._eat_token_or_fail (TokCloseStruct, "Expected '}'")
        if not cmd.attributes["exec"]:
            self._fail ("Invalid command defined -- no executable specified")
        return cmd

    def _parse_group(self, parent_group):
        self._eat_token_or_fail (TokString, "Expected group name string")
        if "/" in self._cur_tok.val:
            self._fail("'/' character is not allowed in group name")
        elif not self._cur_tok.val.strip():
            self._fail("Empty group name is not allowed")
        name = self._cur_tok.val
        group = parent_group.get_subgroup([name], True)
        self._eat_token_or_fail (TokOpenStruct, "Expected '{'")
        while self._eat_token(TokIdentifier):
            if self._cur_tok.val == "cmd":
                group.add_command(self._parse_command())
            elif self._cur_tok.val == "group":
                self._parse_group(group)
            else:
                self._fail("Expected one of [group, cmd]")
        self._eat_token_or_fail(TokCloseStruct, "Expected '}'")

    def _parse_start_stop_restart_action(self, action_type):
        valid_ident_types = [ "everything", "cmd", "group" ]
        ident_type = self._parse_identifier_one_of(valid_ident_types)
        ident = None
        if ident_type != "everything":
            ident = self._parse_string_or_fail()
        if self._eat_token(TokEndStatement):
            return StartStopRestartActionNode(action_type, ident_type, ident,
                    None)
        self._expect_identifier("wait", "Expected ';' or 'wait'")
        wait_status = self._parse_string_one_of(["running", "stopped"])
        self._eat_token_or_fail(TokEndStatement, "Expected ';'")
        return StartStopRestartActionNode(action_type, ident_type, ident,
                wait_status)

    def _parse_wait_action(self):
        wait_type = self._parse_identifier_one_of(["ms", "cmd", "group"])
        if wait_type == "ms":
            err_msg = "Expected integer constant"
            delay_ms = int(self._eat_token_or_fail(TokInteger, err_msg))
            self._eat_token_or_fail(TokEndStatement, "Expected ';'")
            return WaitMsActionNode(delay_ms)
        else:
            ident = self._parse_string_or_fail()
            self._expect_identifier("status")
            wait_status = self._parse_string_one_of(["running", "stopped"])
            self._eat_token_or_fail(TokEndStatement, "Expected ';'")
            return WaitStatusActionNode(wait_type, ident, wait_status)

    def _parse_run_script(self):
        script_name = self._eat_token_or_fail(TokString, "expected script name")
        self._eat_token_or_fail(TokEndStatement, "Expected ';'")
        return RunScriptActionNode(script_name)

    def _parse_script_action_list(self):
        self._eat_token_or_fail (TokOpenStruct, "Expected '{'")
        actions = []
        while self._eat_token(TokIdentifier):
            action_type = self._cur_tok.val
            if action_type in [ "start", "stop", "restart" ]:
                action = self._parse_start_stop_restart_action(action_type)
                actions.append(action)
            elif action_type == "wait":
                actions.append(self._parse_wait_action())
            elif action_type == "run_script":
                actions.append(self._parse_run_script())
            else:
                self._fail("Unexpected token %s" % action_type)
        self._eat_token_or_fail(TokCloseStruct, "Unexpected token")
        return actions

    def _parse_script(self):
        name = self._eat_token_or_fail(TokString, "expected script name")
        script_node = ScriptNode(name)
        for action in self._parse_script_action_list():
            script_node.add_action(action)
        self._node.add_script(script_node)

    def _parse_listdecl(self):
        while True:
            if self._eat_token(TokEOF):
                return
            ident_type = self._parse_identifier_one_of(["cmd", "group", "script"])
            if ident_type == "cmd":
                self._node.root_group.add_command(self._parse_command())
            if ident_type == "group":
                self._parse_group(self._node.root_group)
            if ident_type == "script":
                self._parse_script()

    def parse (self, f):
        self.tokenizer = Tokenizer (f)
        self._cur_tok = None
        self._next_tok = None
        self._get_token ()
        self._node = ConfigNode()
        self._parse_listdecl()
        return self._node

def config_from_filename (fname):
    return Parser ().parse (file (fname))

if __name__ == "__main__":
    import sys
    try:
        fname = sys.argv[1]
    except IndexError:
        print "usage: sheriff_config.py <fname>"
        sys.exit (1)

    print config_from_filename (fname)
