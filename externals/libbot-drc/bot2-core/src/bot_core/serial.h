#ifndef __bot_serial_h__
#define __bot_serial_h__

/**
 * @defgroup BotCoreSerial Serial ports
 * @brief Reading and writing from serial ports
 * @ingroup BotCoreIO
 * @include: bot_core/bot_core.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs bot2-core`
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/** Creates a basic fd, setting baud to 9600, raw data i/o (no flow
    control, no fancy character handling. Configures it for blocking
    reads.  8 data bits, 1 stop bit, no parity.

    Returns the fd, -1 on error
**/
int bot_serial_open(const char *port, int baud, int blocking);

/** Set the baud rate, where the baudrate is just the integer value
    desired.

    Returns non-zero on error.
**/
int bot_serial_setbaud(int fd, int baudrate);

/** Enable cts/rts flow control.
    Returns non-zero on error.
**/
int bot_serial_enablectsrts(int fd);
/** Enable xon/xoff flow control.
    Returns non-zero on error.
**/
int bot_serial_enablexon(int fd);

/** Set the port to 8 data bits, 2 stop bits, no parity.
    Returns non-zero on error.
 **/
int bot_serial_set_N82 (int fd);

int bot_serial_close(int fd);

/**
 * Get the number of bytes waiting to be read
 */
int bot_serial_bytes_available(int fd);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
