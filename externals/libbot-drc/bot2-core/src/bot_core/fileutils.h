#ifndef __bot_fileutils_h__
#define __bot_fileutils_h__

#include <stdio.h>

/**
 * @defgroup BotCoreFileUtils File Utilities
 * @brief Convenience functions for reading/writing files
 * @ingroup BotCoreIO
 * @include: bot_core/bot_core.h
 *
 * Linking: `pkg-config --libs bot2-core`
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * bot_fileutils_get_unique_filename:
 *
 * Returns an unused filename that can be used for writing without fear
 * of overwriting another file.  The returned string must be freed with
 * free() when no longer needed.  The new file is not opened.  The
 * filename is constructed as path/[date-]basename.XX.extension where
 * XX is an integer chosen to make it unique.  date- is only included
 * if @time_prefix is 1.
 *
 * @path: The path to the directory where the file should be stored.
 *    Pass NULL to use the current working directory or if the path
 *    is already included in @basename.
 * @basename: The desired name of the file without any trailing extension.
 *    This may include a leading path, but only if @time_prefix is zero.
 * @time_prefix: Set to 1 if the basename should be prefixed with the
 *    current date in the format YYYY-MM-DD-.
 * @extension: Filename extension to be used.  Use NULL if no extension
 *    is desired.
 */
char *
bot_fileutils_get_unique_filename (const char * path, const char * basename,
        uint8_t time_prefix, const char * extension);

int bot_fileutils_write_fully(int fd, const void *b, int len);
int bot_fileutils_read_fully(int fd, void *b, int len);
int bot_fileutils_read_timeout(int fd, void *buf, int maxlen, int msTimeout);
int bot_fileutils_read_fully_timeout(int fd, void *bufin, int len, int msTimeout);
int bot_fileutils_read_line_timeout(int fd, void *buf, int maxlen, int msTimeout);
int bot_fileutils_read_line_timeout_ex(int fd, void *buf_in, int maxlen, int msTimeout, int *timed_out);
int bot_fileutils_read_available(int fd);
void bot_fileutils_read_flush(int fd);

int bot_fileutils_fwrite32(FILE *f, int32_t v);
int bot_fileutils_fwrite64(FILE *f, int64_t v64);
int bot_fileutils_fread32(FILE *f, int32_t *v32);
int bot_fileutils_fread64(FILE *f, int64_t *v64);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
