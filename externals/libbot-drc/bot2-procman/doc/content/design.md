Procman design overview {#procman_design}
=====================

This page explains the design of procman and how the different components work
together to control processes.

# Deputies and Sheriffs {#procman_design_deputies_and_sheriffs}

There are two types of processes in procman: deputies and sheriffs.

_Deputies_ host and control processes directly.  A deputy can:
- start child processes
- stop child processes
- monitor and report the status of hosted processes

The deputy is essentially a daemon process that manages other commands.  It is
not interactive, does not have a GUI, and simply carries out orders that it
receives from a sheriff.


_Sheriffs_ tell deputies what to do.  A single sheriff can command many
deputies.  Any process can be a sheriff as long as it implements the
sheriff/deputy communications protocol, but the most commonly used sheriff is
the `bot-procman-sheriff` GUI tool provided with Procman.  The Procman Python API
can also be used to implement a custom sheriff.

Sheriffs and deputies [communicate via LCM](\ref procman_comms), a UDP
multicast-based communications protocol.  All communications are stateless,
which enables sheriffs and deputies to work together more easily in the
presence of network and communication dropouts.

## bot-procman-sheriff {#procman_design_bot_procman_sheriff}

`bot-procman-sheriff` is the primary implementation of Procman sheriff, and can
be used to communicate with and command deputies.  The sheriff forms a global
view of all deputies and their commands.  The sheriff sends commands to the
deputies, and specifies which commands a deputy should be managing, and the
desired state of those commands.

The sheriff has an interactive GUI through which a user can modify commands and
their desired statuses.  It also has a scripting facility that can be useful
for starting multiple commands at once, sequencing a startup procedure, or
running simple scripts in general.

The sheriff can also be run from the command line without a GUI.

\image html procman-sheriff-screenshot.png "bot-procman-sheriff screenshot"

### Observer mode {#procman_design_bot_procman_sheriff_observer_mode}

`bot-procman-sheriff` can be switched to observer mode, where it stops
transmitting commands, and simply displays the state of the deputies.  Observer
mode is useful in situations where you want to simply observe the state of a
running system.  Examples of this include situations where the active sheriff
is running without a GUI, and also when replaying an LCM log file that contains
deputy status message (using the LCM log playback tools).
