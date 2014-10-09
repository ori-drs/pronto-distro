#ifndef __procman_engine_h__
#define __procman_engine_h__

// procman provides a set of data structures and functions for managing a
// number of separate processes

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <glib.h>

typedef enum {
    PROCMAN_CMD_STOPPED = 0,
    PROCMAN_CMD_RUNNING
} procman_cmd_status_t;

typedef struct _procman procman_t;

typedef struct _procman_params {
    char config_file[1024];  // the configuration file listing all commands

    char bin_path[1024];  // commands that are not specified as an absolute
                          // path have this prepended to their path
    int verbose;
} procman_params_t;

typedef struct _procman_cmd {
    int32_t cmd_id;   // unique to the containing instance of procman_t

    GString *cmd; // the command to execute.  Do not modify directly

    char* cmd_name;  // a user-assigned name for the command.  Do not modify directly

    int pid;      // pid of process when running.  0 otherwise

    int stdin_fd;  // when the process is running, writing to this pipe
                   // writes to stdin of the process
    int stdout_fd; // and reading from this pipe reads from stdout of the proc

    int exit_status;

    int envc;    //number of environment variables
    char ***envp; //environment variables to set

    int argc;    //number of arguments, shouldn't be needed
    char **argv; // don't touch this

    GArray* descendants_to_kill; // Used internally when killing a process.

    void *user;  // use this for application-specific data
} procman_cmd_t;


void procman_params_init_defaults (procman_params_t *params,
        int argc, char **argv);

// constructor
procman_t *procman_create (const procman_params_t *params);

// destructor
void procman_destroy (procman_t *pm);

// returns a doubly linked list, where each data element is a procman_cmd_t
//
// Do not modify this list, or it's contents!
const GList* procman_get_cmds (procman_t *pm);

/**
 * Sets a variable.  Variable expansion will be performed on commands with
 * variables of the form $VARNAME or ${VARNAME}, for variable names in this
 * hash table.
 */
void procman_set_variable(procman_t* pm, const char* name, const char* val);

/**
 * Removes a variable from the variable expansion table.
 */
void procman_remove_variable(procman_t* pm, const char* name);

/**
 * Removes all variables from the variable expansion table.
 */
void procman_remove_all_variables(procman_t* pm);

int procman_start_cmd (procman_t *pm, procman_cmd_t *cmd);
int procman_stop_cmd (procman_t *pm, procman_cmd_t *p);
int procman_kill_cmd (procman_t *pm, procman_cmd_t *cmd, int signum);

// convenience functions
int procman_start_all_cmds (procman_t *pm);
int procman_stop_all_cmds (procman_t *pm);

/* adds a command to be managed by procman.  returns a pointer to a newly
 * created procman_cmd_t, or NULL on failure
 *
 * The command is not started.  To start a command running, use
 * procman_start_cmd
 */
procman_cmd_t* procman_add_cmd (procman_t *pm, const char *cmd_str, const char* cmd_name);

/* Removes a command from management by procman.  The command must already be
 * stopped and reaped by procman_check_for_dead_children.  Otherwise, this
 * function will fail.  On success, the %cmd structure is destroyed and no
 * longer available for use.
 *
 * returns 0 on success, -1 on failure.
 */
int procman_remove_cmd (procman_t *pm, procman_cmd_t *cmd);

/* searches for a command.  returns a pointer to the corresponding
 * procman_cmd_t on success, NULL on failure
 */
procman_cmd_t *procman_find_cmd (procman_t *pm, const char *cmd_str);

/* searches for a command.  returns a pointer to the corresponding
 * procman_cmd_t on success, NULL on failure
 */
procman_cmd_t *procman_find_cmd_by_id (procman_t *pm, int32_t cmd_id);

/* checks to see if any processes spawned by procman_start_cmd have died
 *
 * dead_child should point to an unused procman_cmd_t *
 *
 * on return, if a child process has died, then it is reaped and a pointer to
 * the procman_cmd_t is placed in dead_child.  If no children have died, then
 * dead_child points to NULL on return.
 *
 * This function does not block
 *
 * returns 0 on success, -1 on failure
 */
int procman_check_for_dead_children (procman_t *pm,
        procman_cmd_t **dead_child);

int procman_close_dead_pipes (procman_t *pm, procman_cmd_t *cmd);

/* returns 0  TODO
 */
int32_t procman_get_cmd_status (procman_t *pm, procman_cmd_t *cmd);

/* Changes the command that will be executed for a procman_cmd_t
 * no effect until the command is started again (if it's currently running)
 */
void procman_cmd_change_str (procman_cmd_t *cmd, const char *cmd_str);

/**
 * Sets the command name.
 */
void procman_cmd_set_name(procman_cmd_t* cmd, const char* cmd_name);

#define PROCMAN_MAX_MESSAGE_AGE_USEC 60000000LL

#ifdef __cplusplus
}
#endif

#endif
