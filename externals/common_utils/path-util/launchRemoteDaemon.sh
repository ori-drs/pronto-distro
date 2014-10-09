#!/bin/bash
# Launch a remote daemon on another machine

function kill_daemon {
    if [ ! -z "$KILLED" ]; then #prevent multipe calls... 
	return
    fi
    KILLED=TRUE
    #end the remote screen session
    echo -e "\nKilling session $SESSION_NAME on host $REMOTE_HOST"
    nohup ssh -f -o BatchMode=yes $REMOTE_HOST "bash -c 'kill \`cat $PID_FILE\` && rm $DAEMON_OUTFILE $PID_FILE' " &>/dev/null </dev/null &
    disown
    ps &> /dev/null #dunno why, but calling ps seems to be necessary for ssh to background properly :-/
    echo "kill-ssh exited with code $?" 
}

function usage {
	# Display usage message on standard error
	echo "Usage:"
	echo "   $(basename $0) host remote_workingdir daemon [args]" 1>&2
	exit 1
}

if [ $# -lt 3 ]; then
	usage
fi
SESSION_NAME=session_${RANDOM}
REMOTE_HOST=$1
WORKING_DIR=$2
DAEMON=$3
shift;shift;shift
DAEMON_ARGS="$@"

DAEMON_OUTFILE=/tmp/${SESSION_NAME}_OUT
PID_FILE=/tmp/${SESSION_NAME}_PID

#trap signals so we can cleanup later
trap kill_daemon SIGHUP SIGINT SIGTERM

#launch the daemon on the remote host
echo "Launching $DAEMON from $WORKING_DIR on remote host: $REMOTE_HOST"
echo "Using session name $SESSION_NAME"
ssh -o BatchMode=yes $REMOTE_HOST "bash -c 'cd $WORKING_DIR; ./$DAEMON $DAEMON_ARGS &>$DAEMON_OUTFILE < /dev/null & echo \$! > $PID_FILE' " 

echo "launch-ssh exited with code $?"

#monitor it using autossh for persistance
echo -e "Monitoring output:\n"
autossh -M 0 -- -t -o BatchMode=yes -o ServerAliveInterval=5 -o serverAliveCountMax=3 $REMOTE_HOST "tail -f $DAEMON_OUTFILE"

echo "monitor-ssh exited with code $?" 

kill_daemon