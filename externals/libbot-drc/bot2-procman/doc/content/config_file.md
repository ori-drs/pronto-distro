Configuration files {#procman_config_file}
===================

[TOC]

# Overview {#procman_config_file_overview}

When working within a single project, you'll typically want to load the same
set of commands into procman for managing.  The sheriff configuration files
provide a mechanism for:
- storing a set of commands that can be loaded into procman sheriff.
- writing simple scripts to sequence starting and stopping commands.

A simple configuration file might look like:

\code
# a hash mark indicates a comment.

# commands can be listed individually.  This command will have ID "web-browser".
cmd "web-browser" {
    # When started from procman, the executable named "firefox" is invoked.
    exec = "firefox";
    # The command is assigned to the deputy specified by the "host" attribute.
    host = "deputy_name";
}

# Commands can be grouped.  This group is named "shells"
group "shells" {
    cmd "terminal 1" {
        exec = "xterm";
        host = "deputy_name";
    }
    cmd "terminal 2" {
        exec = "rxvt";
        host = "deputy_name";
    }
    # more commands in "group_name_0" here.
}
# more groups...
\endcode
`simple_config.procman`

# Loading a configuration file {#procman_config_file_loading}

You can load this configuration file into procman by passing it as a command
line argument:
\code
$ bot-procman-sheriff simple_config.procman
\endcode

Additionally, the procman sheriff GUI provides a menu option to save and load
configuration files.

\note The procman sheriff GUI config file save operation does not preserve
comments.  So if you load a config file that you manually entered comments
into, and then save it out again, you will lose the comments.  If you want to
preserve comments, then you need to stick with hand-edited config files.

# Configuration file structure {#procman_config_file_structure}

The general structure of a configuration file takes the form of a list of
elements.  Each element can be:
- A command, indicated by "cmd".
- A group, indicated by "group".
- A script, indicated by "script".

# Commands {#procman_config_file_commands}

Commands are the central concept in procman, and correspond to what you might
type into a shell to start a process running.  A command starts with the
keyword "cmd" followed by the command ID, and is described using a list of
attributes surrounded by curly braces.  Each attribute be expressed as a
separate line of the form:
\code
    attribute = value;
\endcode

Values are either quoted strings, or integers.

- "exec"
  - String.  The name of the command that will be run when the command is started.  This
    should follow bash-style shell syntax.  This attribute is __required__.
- "host"
  - String.  The name of the deputy that the command will be assigned to.  This attribute is __required__.
- "auto_respawn"
  - String.  Must be either "true" or "false".  If true, then a deputy will
    automatically try to restart the command if it stops, as long as the
    desired command status is set to running.  In other words, if you order the
    command to stop via the sheriff, then it will not be automatically restarted.
    If not specified, defaults to "false".
- "stop_signal"
  - Integer.  When ordering a command to stop, this specifies the numerical
    OS-level signal to send the command to request a clean exit.  Most of the
    time, this should be either 2 or 15 (corresponding to SIGINT and SIGTERM on
    POSIX systems).  If the command does not exit after an alloted amount of
    time passes, then it is sent a SIGKILL.  If not specified, this defaults to
    2 (SIGINT).
- "stop_time_allowed"
  - Integer.  When ordering a command to stop, a deputy first sends
  `stop_signal` and waits `stop_time_allowed` seconds for the command to exit.
  If it is still running after `stop_time_allowed` seconds elapses, then the
  command is immediately sent a SIGNKILL.  If not specified, this defaults to
  7.

Some examples:
\code
cmd "MATLAB" {
    # Start MATLAB and have it run the script 'run_my_robot.m'
    exec = "matlab -nodisplay -r run_my_robot";

    # Run it on deputy name
    host = "deputy_name";

    # MATLAB may not exit on SIGINT, but does respond to SIGTERM.
    stop_signal = 15;

    # may need a little extra time to exit cleanly..
    stop_time_allowed = 20;
}
cmd "hardware interface" {
    exec = "hardware_driver";
    host = "deputy_name";
    # Automatically restart this command if it crashes.
    auto_respawn = "true";
}
\endcode

## Environment variables. {#procman_config_file_commands_environment_variables}

Procman supports setting and using environment variables in the executable
specification for a command.  Procman generally follows bash-style syntax.  In
particular, environment variables can be set by prefixing the command with
"NAME=VALUE" pairs.  There cannot be any spaces surrounding the "=" symbol in
the environment variable specifications.  For example:

\code
cmd "terminal" {
    # runs 'xterm' and set the environment variable 'DISPLAY' to the value ':1.0'
    exec = "DISPLAY=:1.0  xterm";
    host = "deputy_name";
}
\endcode

In addition, environment variables can be referenced from an executable
specification.  Note that environment variables are always evaluated in the
deputy process at the time a command is started, and not on the sheriff.  For example:

\code
cmd "list home directory {
    exec = "ls ${HOME}";
    host = "deputy_name";
}
\endcode

# Groups {#procman_config_file_groups}

In the same way that commands can be grouped together in the procman sheriff
GUI, they can also be grouped together in configuration files using the 'group'
specifier.  This generally takes the form of:

\code
group "group_name" {
    # Command specifiers...
    # Every command enclosed by the surrounding curly braces will be treated as
    # part of this group.
}
\endcode

Groups can also be nested:
\code
group "group_name" {
    group "subgroup_a" {
        group "subsubgroup_0" {
            # Command specifiers...
            cmd "terminal" {
                exec = "xterm";
                host = "deputy_name";
            }
        }
        cmd "web browser" {
            exec = "firefox";
            host = "other_deputy";
        }
    }
}
\endcode

# Scripts {#procman_config_file_scripts}

Procman sheriff supports a very simple scripting language that can be useful
for sequencing the starting and stopping of commands.  The scripting language
allows you to specify a deterministic sequence of actions that are run one
after the other.

A configuration file with a simple script might look like:
\code
cmd "load configuration" {
    # This command loads the robot configuration, and then exits.
    exec = "configure_robot";
    host = "robot";
}
cmd "hardware interface" {
    exec = "hw_interface";
    host = "robot";
}
group "planning and perception" {
    cmd "planner" {
        exec = "planner";
        host = "robot";
    }
    cmd "perception" {
        exec = "perception";
        host = "robot";
    }
}

script "go" {
    # Start the 'load configuration' command, and then pause the script until
    # the command exits.
    start cmd "load configuration" wait "stopped";

    # Now start the hardware interface processes
    start cmd "hardware interface";

    # wait 1000 milliseconds
    wait ms 1000;

    # Start all of the commands in the group 'planning and perception'
    start group "planning and perception";
}
\endcode

This configuration file has a single script named "go".  If you load the config
file into the procman sheriff GUI, then you'll see "go" listed under the
scripts menu, which you can then use to run the script.

## Script actions {#procman_config_file_script_actions}

A script is composed of a sequence of actions.  The valid actions are:
### "start"
Usage: `start {cmd|group} TARGET_ID [ wait {"running","stopped"} ]`

Orders a command or a group to start running.  Examples:
\code
# Orders the command "planner" to start running.
start cmd "planner";

# Orders the command "load configuration" to start running and then waits for
# it to exit.  Essentially, run it once through to completion.
start cmd "load configuration" wait "stopped";

# Order the entire group "planning and perception" to start running
start group "planning and perception";

# order the command "hw_interface" to start running and waits for it to
# have actually started running (i.e., for the deputy to report that it has
# started the child process).
start cmd "hw_interface" wait "running";
#
\endcode

If "wait" is used on a group, then script execution only continues when all
commands in the group achieve the specified status.  A script can wait
indefinitely, and does not timeout or fail.

If "wait" is not specified, then script execution continues immediately.  This
way, it is possible to effectively order many commands and groups to start
running all at once.

### "stop"
Usage: `stop {cmd|group} TARGET_ID [ wait "stopped" ]`

This is the opposite of "start", and orders a single command or a group of
commands to stop execution.  Commands that have the "auto_respawn" attribute
will also be stopped and they will not be automatically respawned.

Some example:
\code
# Order a single command to stop.
stop cmd "hw_interface";

# Order an entire group to stop, and wait for all commands in the group to stop
# before continuing.
stop group "planning and perception" wait "stopped";
#
\endcode

### "restart"
Usage: `restart {cmd|group} TARGET_ID [ wait {"running", "stopped"} ]`

The restart action first stops a command or group of commands, and then orders
them to start.  Using this script action is usually faster than using a "stop"
followed by a "start", as the restart action is executed entirely by the
deputy.

Otherwise, the usage is similar to "start" and "stop".

### "wait ms"
Usage: `wait ms MILLISECONDS`

This action simply pauses script execution for the specified number of
milliseconds.  This can be useful if you need a quick and dirty way to wait for
an external device to settle, or do not have another way of synchronizing
script execution.

Use this script action with caution, as it's almost always better to find a
more robust way of sequencing script execution than by inserting arbitrary
delays.

### "wait status"
Usage: `wait {cmd|group} status {"running", "stopped"}`

Waits for a single command, or a group of commands to all achieve the specified
status.  For example:

\code
# Order a bunch of commands to stop.
stop cmd "abc";
stop cmd "def";
stop cmd "ghi";
# Now wait for them all to actually stop.
wait cmd "abc" status "stopped";
wait cmd "def" status "stopped";
wait cmd "ghi" status "stopped";
\endcode

### "run_script"

Usage: `run_script OTHER_SCRIPT_NAME`

A configuration file can have multiple scripts.  This script action invokes
another script listed in the configuration file and waits the other script to
finish execution before continuing.

## Running scripts {#procman_config_file_script_running}

Scripts can be generally be run in one of several ways:
- load a configuration file into the procman sheriff GUI, and select the script
  from the "script" menu.
- pass the configuration file and script name as command line arguments to `bot-procman-sheriff`.  For example:
\code
$ bot-procman-sheriff config_file.procman go
\endcode

## Referencing nested groups {#procman_config_file_script_nested_groups}

The syntax for identifying subgroups (i.e., groups nested within another group)
is to join the group names with slashes.  For example, if group "cameras" is a
subgroup of group "hardware drivers", then a script would refer to the inner
group by the name "hardware drivers/cameras".

\code
group "hardware drivers" {
    group "cameras" {
        cmd "left camera" {
            exec = "left_camera";
            host = "robot";
        }
        cmd "right camera" {
            exec = "right_camera";
            host = "robot";
        }
    }
}

script "go" {
    start group "hardware drivers/cameras";
}

For this reason, slashes are not allowed in group names.
\endcode

