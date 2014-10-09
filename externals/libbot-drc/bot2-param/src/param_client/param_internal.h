#include "param_client.h"

#define BOT_PARAM_UPDATE_CHANNEL "PARAM_UPDATE"
#define BOT_PARAM_REQUEST_CHANNEL "PARAM_REQUEST"
#define BOT_PARAM_SET_CHANNEL "PARAM_SET"
#define BOT_PARAM_INCLUDE_KEYWORD "INCLUDE"

/**
 * bot_param_set_int:
 * @param: The configuration.
 * @key: The key to look for a value.
 * @val: The value to set it to.
 *
 * This function searches for a key (or creates it if it does not exist yet)
 * and stores a single value (@val) associated with that key.
 *
 * Returns: 0 on success, -1 on failure.
 */
int
bot_param_set_int (BotParam * param,
                const char * key,
                int val);

/**
 * bot_param_set_boolean:
 * @param: The configuration.
 * @key: The key to look for a value.
 * @val: The value to set it to.
 *
 * This function searches for a key (or creates it if it does not exist yet)
 * and stores a single value (@val) associated with that key.
 *
 * Returns: 0 on success, -1 on failure.
 */
int
bot_param_set_boolean (BotParam * param,
                    const char * key,
                    int val);

/**
 * bot_param_set_double:
 * @param: The configuration.
 * @key: The key to look for a value.
 * @val: The value to set it to.
 *
 * This function searches for a key (or creates it if it does not exist yet)
 * and stores a single value (@val) associated with that key.
 *
 * Returns: 0 on success, -1 on failure.
 */
int
bot_param_set_double (BotParam * param,
                   const char * key,
                   double val);

/**
 * bot_param_set_str:
 * @param: The configuration.
 * @key: The key to look for a value.
 * @val: The value to set it to.
 *
 * This function searches for a key (or creates it if it does not exist yet)
 * and stores a single value (@val) associated with that key.
 *
 * Returns: 0 on success, -1 on failure.
 */
int
bot_param_set_str (BotParam * param,
                const char * key,
                const char * val);

/**
 * bot_param_set_int_array:
 * @param: The configuration.
 * @key: The key to look up (or create).
 * @vals: Array of values to set.
 * @len: Number of members in @vals.
 *
 * This function searches for a key, or creates it if it does not exist.  It
 * stores the entire input array @vals at that key.
 *
 * Returns: 0 on success, -1 on failure.
 */
int
bot_param_set_int_array (BotParam * param,
                      const char * key,
                      int * vals,
                      int len);

/**
 * bot_param_set_boolean_array:
 * @param: The configuration.
 * @key: The key to look up (or create).
 * @vals: Array of values to set.
 * @len: Number of members in @vals.
 *
 * This function searches for a key, or creates it if it does not exist.  It
 * stores the entire input array @vals at that key.
 *
 * Returns: 0 on success, -1 on failure.
 */
int
bot_param_set_boolean_array (BotParam * param,
                          const char * key,
                          int * vals,
                          int len);

/**
 * bot_param_set_double_array:
 * @param: The configuration.
 * @key: The key to look up (or create).
 * @vals: Array of values to set.
 * @len: Number of members in @vals.
 *
 * This function searches for a key, or creates it if it does not exist.  It
 * stores the entire input array @vals at that key.
 *
 * Returns: 0 on success, -1 on failure.
 */
int
bot_param_set_double_array (BotParam * param,
                         const char * key,
                         double * vals,
                         int len);

/**
 * bot_param_set_str_array:
 * @param: The configuration.
 * @key: The key to look up (or create).
 * @vals: Array of values to set.
 * @len: Number of members in @vals.
 *
 * This function searches for a key, or creates it if it does not exist.  It
 * stores the entire input array @vals at that key.
 *
 * Returns: 0 on success, -1 on failure.
 */
int
bot_param_set_str_array (BotParam * param,
                      const char * key,
                      const char ** vals,
                      int len);

