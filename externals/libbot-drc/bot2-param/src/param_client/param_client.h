#ifndef __bot_param_client_h__
#define __bot_param_client_h__

#include <stdio.h>
#include <lcm/lcm.h>

/**
 * @defgroup: BotParam Bot Param
 * @brief Hierarchical key/value configuration files
 * @include: bot_param/param_client.h
 *
 * TODO
 *
 * Linking: -lparam_client
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _BotParam BotParam;

/**
 * bot_param_new_from_server:
 * @param lcm         Handle to the lcm object to be used.
 * @param keep_update Keep listening for parameter updates.
 *
 * Gets the params for the param-server, and parse them. Returns a handle to the contents.
 * Checks the environment variable BOT_PARAM_SERVER_NAME. If no parameters are received within 
 * 5seconds, returns with an error.
 *
 * WARNING: This calls lcm_handle internally, so make sure that you create the param_client
 * BEFORE you subscribe with handlers that may use it!
 *
 * @return A handle to a newly-allocated %BotParam or %NULL on error.
 */
BotParam *
bot_param_new_from_server (lcm_t * lcm, int keep_updated);


/**
 * bot_param_new_from_named_server:
 * @param lcm         Handle to the lcm object to be used.
 * @param server_name Name of param server
 * @param keep_update Keep listening for parameter updates.
 *
 * Gets the params for the named param-server, and parse them. Returns a handle to the contents.  
 * If server_name is NULL, checks for environment variable BOT_PARAM_SERVER_NAME. If no parameters 
 * are received within 5seconds, returns with an error.
 *
 * WARNING: This calls lcm_handle internally, so make sure that you create the param_client
 * BEFORE you subscribe with handlers that may use it!
 *
 * @return A handle to a newly-allocated %BotParam or %NULL on error.
 */
BotParam *
bot_param_new_from_named_server (lcm_t * lcm, const char * server_name, int keep_updated);


/**
 * bot_param_update_handler_t
 *
 * Handler function template for a BotParam callback
 *
 * old_param: The previous BotParam structure
 * new_param: The new BotParam structure with updated parameters from the server
 * user: user data that was passed to the function
 */
typedef void(bot_param_update_handler_t)(BotParam * old_param,BotParam * new_param, int64_t utime, void *user);

/**
 * bot_param_add_update_subscriber
 *
 * add a callback handler to get called when a parameter is updated
 * the template of the callback handler is described above
 *
 * param: the BotParam structure that should have updates
 * callback_func: function to call when new params are recieved
 * user: user data to be passed to the function
 */
void bot_param_add_update_subscriber(BotParam *param,
    bot_param_update_handler_t * callback_func, void * user);


/**
 * bot_param_new_from_file:
 * @param filename The name of the file.
 *
 * Parses a file and returns a handle to the contents.
 *
 * Use the INCLUDE keyword directive to link in other param files. For
 * example, add the following to your config to additionally source
 * file1.cfg and file2.cfg:<p>
 *
 \verbatim
 INCLUDE = [
     "pathA/file1.cfg",
     "pathB/file2.cfg",
 ];
 \endverbatim
 *
 * where pathA and pathB can either be absolute or relative path names
 * with respect to filename.
 *
 * @note Namespace clashes are possible with INCLUDE configs keys.  To
 * avoid config key namespace clashes wrap sourced config files in
 * their own unique namespace.
 *
 * @return A handle to a newly-allocated %BotParam or %NULL on parse error.
 */
BotParam *
bot_param_new_from_file (const char * filename);

/**
 * bot_param_new_from_string:
 * @param string A character array containing the config file.
 * @param length The length of the character array.
 *
 * Parses the string and returns a handle to the contents.
 *
 * @return A handle to a newly-allocated %BotParam or %NULL on parse error.
 */
BotParam *
bot_param_new_from_string (const char * string, int length);

/**
 * bot_param_alloc:
 *
 * Creates a new, meaningless, empty config struct.
 *
 * @return An uninitialized %BotParam object.
 */
BotParam *
bot_param_new( void );

/**
 * bot_param_destroy:
 * @param param The %BotParam to free.
 *
 * Frees the memory used by a BotParam and destroys its
 * constituents.
 */
void
bot_param_destroy (BotParam * param);

/**
 * bot_param_write:
 * @param param The configuration to write.
 * @param f     The file handle to write to.
 *
 * Writes the contents of a parsed configuration file to file handle f.
 *
 * @return -1 on error, 0 on success.
 */
int
bot_param_write (BotParam * param, FILE * f);

/**
 * bot_param_write_to_string:
 * @param param The configuration to write.
 * @param s     String to write to.
 *
 * The string will be allocated by the library, but must
 * be freed by the user.
 *
 * return -1 on error, 0 on success.
 */
int
bot_param_write_to_string (BotParam * param, char ** s);

/**
 * bot_param_print:
 * @param param The configuration to print.
 *
 * Prints the contents of a parsed configuration file.
 *
 * @return -1 on error, 0 on success.
 */
static inline int
bot_param_print (BotParam * param){
  return bot_param_write (param, stdout);
}

/**
 * bot_param_has_key:
 * @param param The configuration.
 * @param key   The key to check the existence of.
 *
 * Checks if a key named key exists in the file that was read in by param.
 *
 * @return 1 if the key is present, 0 if not
 */
int bot_param_has_key (BotParam *param, const char *key);

/**
 * bot_param_get_num_subkeys:
 * @param param        The configuration.
 * @param containerKey The key to check.
 *
 * Finds the number of sub keys of @containerKey that also have sub keys.
 *
 * @return The number of top-level keys in container named by containerKey or
 * -1 if container key not found.
 */
int
bot_param_get_num_subkeys (BotParam * param, const char * containerKey);

/**
 * bot_param_get_subkeys:
 * @param param          The configuration.
 * @param containerKey   The key to check.
 *
 * Fetch all top-level keys in container named by containerKey.
 *
 * @return a newly allocated, %NULL-terminated, array of strings.  This array
 * should be freed by the caller (e.g. with g_strfreev)
 */
char **
bot_param_get_subkeys (BotParam * param, const char * containerKey);

/**
 * bot_param_get_int:
 * @param param The configuration.
 * @param key   The key to get the value for.
 * @param val   Return value.
 *
 * This searches for a key (i.e. "sick.front.pos") and fetches the value
 * associated with it into %val, converting it to an %int.  If the conversion
 * is not possible or the key is not found, -1 is returned.  If the key
 * contains an array of multiple values, the first value is used.
 *
 * @return 0 on success, -1 on failure.
 */
int
bot_param_get_int (BotParam * param, const char * key, int * val);

/**
 * bot_param_get_boolean:
 * @param param The configuration.
 * @param key   The key to get the value for.
 * @param val   Return value.
 *
 * Same as bot_param_get_int(), only for a boolean.
 *
 * @return 0 on success, -1, on failure.
 */
int
bot_param_get_boolean (BotParam * param, const char * key, int * val);

/**
 * bot_param_get_double:
 * @param param The configuration.
 * @param key   The key to get the value for.
 * @param val   Return value.
 *
 * Same as bot_param_get_int(), only for a double.
 *
 * @return 0 on success, -1, on failure.
 */
int
bot_param_get_double (BotParam * param, const char * key, double * val);

/**
 * bot_param_get_str:
 * @param param The configuration.
 * @param key   The key to get the value for.
 * @param val   Return value.
 *
 * Same as bot_param_get_int(), only for a string.
 * NOTE: this function returns a DUPLICATE of the string value, which should be freed!
 *
 * @return 0 on success, -1, on failure.
 */
int
bot_param_get_str (BotParam * param, const char * key, char ** val);


/**
 * bot_param_get_int_or_fail:
 * @param param The configuration.
 * @param key   The key to get the value for.
 *
 * Same as bot_param_get_int(), except, on error, this will call abort().
 *
 * @return The int value at @key.
 */
int
bot_param_get_int_or_fail(BotParam *param, const char *key);

/**
 * bot_param_get_boolean_or_fail:
 * @param param The configuration.
 * @param key   The key to get the value for.
 *
 * Same as bot_param_get_boolean(), except, on error, this will call abort().
 *
 * @return The boolean value (1 or 0) at @key.
 */
int
bot_param_get_boolean_or_fail(BotParam * param, const char * key);

/**
 * bot_param_get_double_or_fail:
 * @param param The configuration.
 * @param key   The key to get the value for.
 *
 * Same as bot_param_get_double(), except, on error, this will call abort().
 *
 * @return The double value at @key.
 */
double bot_param_get_double_or_fail (BotParam *param, const char *key);

/**
 * bot_param_get_str_or_fail:
 * @param param The configuration.
 * @param key   The key to get the string value for.
 *
 * Like bot_param_get_double_or_fail(), except with a string instead.
 * NOTE: this function returns a DUPLICATE of the string value, which should be freed!
 *
 * @return The string value at @key.
 */
char
*bot_param_get_str_or_fail (BotParam *param, const char *key);

/**
 * bot_param_get_int_array:
 * @param param The configuration:
 * @param key   The key to look for an array of values.
 * @param vals  An array of ints (return values).
 * @param len   Number of elements in %vals array.
 *
 * Same as bot_param_get_int(), except for an array.
 *
 * @return Number of elements read or -1 on error.
 */
int
bot_param_get_int_array (BotParam * param, const char * key, int * vals, int len);

/**
 * bot_param_get_int_array_or_fail:
 * @param param The configuration:
 * @param key   The key to look for an array of values.
 * @param vals  An array of ints (return values).
 * @param len   Number of elements in %vals array.
 *
 * Same as bot_param_get_int_or_fail(), except for an array. Calls abort() if
 * the number of elements read is less than len.
 *
 */
void
bot_param_get_int_array_or_fail (BotParam * param, const char * key, int * vals, int len);


/**
 * bot_param_get_boolean_array:
 * @param param The configuration:
 * @param key   The key to look for an array of values.
 * @param vals  An array of booleans (return values).
 * @param len   Number of elements in %vals array.
 *
 * Same as bot_param_get_boolean(), except for an array.
 *
 * @return Number of elements read or -1 on error.
 */
int
bot_param_get_boolean_array (BotParam * param, const char * key, int * vals, int len);


/**
 * bot_param_get_boolean_array_or_fail:
 * @param param The configuration:
 * @param key   The key to look for an array of values.
 * @param vals  An array of booleans (return values).
 * @param len   Number of elements in %vals array.
 *
 * Same as bot_param_get_boolean_or_fail(), except for an array. Calls abort() if
 * the number of elements read is less than len. 
 *
 */
void
bot_param_get_boolean_array_or_fail (BotParam * param, const char * key, int * vals, int len);


/**
 * bot_param_get_double_array:
 * @param param The configuration:
 * @param key   The key to look for an array of values.
 * @param vals  An array of doubles (return values).
 * @param len   Number of elements in %vals array.
 *
 * Same as bot_param_get_double(), except for an array.
 *
 * @return Number of elements read or -1 on error.
 */
int
bot_param_get_double_array (BotParam * param, const char * key, double * vals, int len);


/**
 * bot_param_get_double_array_or_fail:
 * @param param The configuration:
 * @param key   The key to look for an array of values.
 * @param vals  An array of doubles (return values).
 * @param len   Number of elements in %vals array.
 *
 * Same as bot_param_get_double(), except for an array. Calls abort() if
 * the number of elements read is less than len. 
 *
 */
void
bot_param_get_double_array_or_fail (BotParam * param, const char * key, double * vals, int len);


/**
 * bot_param_get_array_len:
 * @param param The configuration.
 * @param key   The key to look for a value.
 *
 * Gets the length of the array at key, or -1 if that key doesn't exist.
 *
 * @return the number of elements in the specified array, or -1 if key
 * does not correspond to an array
 */
int bot_param_get_array_len (BotParam *param, const char * key);

/**
 * bot_param_get_str_array_alloc:
 * @param param The configuration.
 * @param key   The key to look for a value.
 *
 * Allocates and returns a %NULL-terminated array of strings.  Free it with
 * bot_param_str_array_free().
 *
 * @return A newly-allocated, %NULL-terminated array of strings.
 */
char **
bot_param_get_str_array_alloc (BotParam * param, const char * key);

/**
 * bot_param_str_array_free:
 * @param data The %NULL-terminated string array to free.
 *
 * Frees a %NULL-terminated array of strings.
 */
void bot_param_str_array_free ( char **data);


/**
 * bot_param_get_global:
 * @param lcm          The lcm object for the global_param client to use
 * @param keep_updated Set to 1 to keep the BotParam updated
 *
 * Upon first being called, this function instantiates and returns a new
 * BotParam instance. Subsequent calls return the same BotParam instance.
 *
 * <b>WARNING</b>: Creating the param_client calls lcm_handle internally,
 * so make sure that you create the param_client <i>BEFORE</i>
 * you subscribe handlers that may use it!
 *
 * @return pointer to BotParam
 */
BotParam*
bot_param_get_global(lcm_t * lcm,int keep_updated);



/**
 * bot_param_local_override:
 * @param param The param object to modify
 * @param key   The param key to set
 * @param val   The value to set it to
 *
 * Note: This method is ONLY valid if the param object is not subscribing to updates!
 */
int bot_param_override_local_param(BotParam * param, const char * key, const char * val); //TODO: add typed versions?

/**
 * bot_param_override_local_params:
 * @param param The param object to modify
 * @param override_params   a list of key=value pairs separated by '|'
 *
 * This function should make it easier to script programs that
 * depend on botParam to get most of their settings
 *
 * Note: This method is ONLY valid if the param object is not subscribing to updates!
 */
int bot_param_override_local_params(BotParam * param, const char * override_params);

int64_t bot_param_get_server_id(BotParam * param);

int bot_param_get_seqno(BotParam * param);



#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
