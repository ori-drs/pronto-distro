Quick Start Tutorial {#procman_tutorial}
====================

This section provides a brief introduction to Procman and controlling processes
with a deputy and a sheriff.

There are two types of programs in Procman - the deputy (\p bot-procman-deputy)
and the sheriff (\p bot-procman-sheriff).

- Deputies control processes, and there must be at least one deputy.
- Sheriffs control deputies, and there can be at most one sheriff.

To get started, we'll show you how to run the deputy and the sheriff, and create
and manage commands.

- \ref procman_tutorial_starting
- \ref procman_tutorial_command_creation
- \ref procman_tutorial_command_managing
- \ref procman_tutorial_config

# Starting a deputy and a sheriff {#procman_tutorial_starting}

First, open up your favorite terminal program and run the deputy command:

\code
bot-procman-deputy
\endcode

Then, in another terminal, launch the sheriff GUI.

\code
bot-procman-sheriff
\endcode

When you run the sheriff, you should see a GUI that looks something like this:

\image html sheriff-gui-empty.png "bot-procman-sheriff with one deputy and no commands."

There are three panes in the sheriff GUI:
- Top left: the command pane
- Top right: the deputy pane
- Bottom: the console pane

The _command pane_ shows all the commands managed by Procman.  Since we haven't
created any commands, there isn't anything to see there yet.

The _deputy pane_ shows all the deputies detected by the sheriff.  We've started
one deputy, so it shows up in the top right.  Deputies are always named, and
the default name is the computer hostname where the deputy process is running.
In this case, the hostname is "contact", so the deputy is named "contact" as well.

The _console pane_ shows console output (stdout and stderr) from running
commands, and also status information from the sheriff.

# Creating a command {#procman_tutorial_command_creation}

To create a command, select the menu item "Commands -> New command".

\image html sheriff-gui-add-command-menu.png "Adding a command using the menu bar."

Following that, a dialog box should appear with several options to fill out.

\image html sheriff-gui-add-command.png "Add command dialog."

The options are:
- \c Deputy - which deputy should manage the command.
- \c Command - the command to execute and any command-line arguments.
- \c Name - a name for the command, used for display purposes and to identify the command.
- \c Group - commands can be grouped together, or not.
- \c Auto-restart - if checked, deputies will automatically restart commands when they terminate.

In the image above, we've set the command to \c xterm, a fairly common terminal
emulator.  If you don't have \c xterm installed, replace the command with
something you do have on your system.  We've also named the command "terminal
emulator".

# Managing commands {#procman_tutorial_command_managing}
Now that we have a command, we can run it.  Click on the command in the command
pane so that it is highlighted.  Once the command is selected, click on the menu
bar item "Commands -> Start".

\image html sheriff-gui-start-command-menu.png

You should now see the \c xterm terminal emulator popup, and the status of the command
should change to "Running".

\image html sheriff-gui-command-running.png

And that's basically it.

You can stop, restart, move, and edit the selected command using the
"Commands" menu.

There are a bunch more features to Procman, but creating, starting, and
stopping processes is its core functionality.

# Saving and loading configurations {#procman_tutorial_config}

It's often useful to save the commands you've created so that you can easily
load them and run them again later on.  To do this, select "File -> Save config
as", and then select a filename.

The configuration files are human-readable, so if you save the configuration
above and opened up the file in a text editor, you'd see something like the
following:

\verbatim
cmd "terminal emulator" {
    exec = "xterm";
    host = "contact";
}
\endverbatim

It should be pretty straightforward to figure out the format, and you can edit
the configuration files directly with a text editor.  For more information on
the configuration file format, see the [documentation on configuration files](\ref procman_config_file).

To load a configuration, click on the menu bar item "File -> load config".
