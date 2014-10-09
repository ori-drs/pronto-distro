#include <termios.h> // terminal io (serial port) interface
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <assert.h>
#include <signal.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <lcm/lcm.h>
#include <lcmtypes/microstrain_ins_t.h>

#include <ConciseArgs>
#define ACC_ANG_MAG 0xCB
#define LENGTH_ACC_ANG_MAG 43

#define ACC_STAB (0xD2)
#define LENGTH_ACC_STAB (43)

#define DANG_DVEL_MAG 0xD3
#define LENGTH_DANG_DVEL_MAG 43

#define ACC_ANG_MAG_ROT 0xCC
#define LENGTH_ACC_ANG_MAG_ROT 79

#define CONTINUOUS_MODE_COMMAND  0xC4
#define LENGTH_CONTINUOUS_MODE      4
#define LENGTH_CONTINUOUS_MODE_ECHO 8

#define COMMS_SETTINGS_COMMAND   0xD9
#define LENGTH_COMMS_SETTINGS      11
#define LENGTH_COMMS_SETTINGS_ECHO 10

#define SAMPLING_SETTINGS_COMMAND   0xDB
#define LENGTH_SAMPLING_SETTINGS      20
#define LENGTH_SAMPLING_SETTINGS_ECHO 19

#define DATA_RATE_DEFAULT  100
#define DATA_RATE_MED      500
#define DATA_RATE_HIGH    1000

#define BAUD_RATE_DEFAULT 115200
#define BAUD_RATE_MED     460800
#define BAUD_RATE_HIGH    921600

#define DELTA_ANG_VEL_DT_DEFAULT 0.010
#define DELTA_ANG_VEL_DT_MED     0.002
#define DELTA_ANG_VEL_DT_HIGH    0.001

#define FILTER_WINDOW_SIZE_DEFAULT 15
#define FILTER_WINDOW_SIZE_MIN      1
#define FILTER_WINDOW_SIZE_MAX     32

#define GRAVITY 9.80665

typedef unsigned char Byte;
using namespace std;

#define INPUT_BUFFER_SIZE 2000

// global reference to Glib main loop
static GMainLoop* mainloop = NULL;

//self structure
class app_t {
public:
  // communication variables
  int comm, status;
  char comm_port_name[255];
  unsigned int baud_rate, data_rate;

  // dt for angular rate and acceleration computation
  double delta_t;

  // window size for gyro and accelerometer digital filter
  int filter_window_size;

  // input buffer variables
  Byte input_buffer[INPUT_BUFFER_SIZE];
  Byte message_mode;
  BotRingBuf * read_buffer;
  char current_segment;
  int expected_segment_length;

  // packet variables
  int message_size;
  int message_start_byte;

  // state flags (not currently used, but may be in future)
  bool changed_baud_rate;
  bool changed_data_rate;
  bool in_continuous_mode;

  // boolean setting flags
  bool quiet, verbose;

  bool little_endian;

  int64_t utime_prev;

  BotParam * param;

  lcm_t * lcm;
  string channel;

  bot_timestamp_sync_state * sync;
  bool do_sync;
};

bool systemLittleEndianCheck()
{
  short int word = 0x0001;
  char *bytes = (char *) &word;
  if (bytes[0] == 0)
    return false;
  else
    return true;
}

unsigned int make32UnsignedInt(Byte* pBytes, bool little_endian)
{
  unsigned int i;

  if (little_endian) {
    ((Byte*) (&i))[0] = pBytes[3];
    ((Byte*) (&i))[1] = pBytes[2];
    ((Byte*) (&i))[2] = pBytes[1];
    ((Byte*) (&i))[3] = pBytes[0];
  }
  else {
    ((Byte*) (&i))[0] = pBytes[0];
    ((Byte*) (&i))[1] = pBytes[1];
    ((Byte*) (&i))[2] = pBytes[2];
    ((Byte*) (&i))[3] = pBytes[3];
  }

  return i;
}

unsigned int make16UnsignedInt(const Byte* pBytes, bool little_endian)
{
  unsigned int i;

  if (little_endian) {
    ((Byte*) (&i))[0] = pBytes[1];
    ((Byte*) (&i))[1] = pBytes[0];

  }
  else {
    ((Byte*) (&i))[0] = pBytes[0];
    ((Byte*) (&i))[1] = pBytes[1];
  }

  return i;
}

void makeUnsignedInt16(unsigned int val, Byte* high, Byte* low)
{
  *low  = static_cast<Byte>(val);
  *high = static_cast<Byte>(val >> 8);
}

void makeUnsignedInt32(unsigned int val, Byte* byte3, Byte* byte2, Byte* byte1, Byte* byte0)
{
  *byte0 = static_cast<Byte>(val);
  *byte1 = static_cast<Byte>(val >>  8);
  *byte2 = static_cast<Byte>(val >> 16);
  *byte3 = static_cast<Byte>(val >> 24);
}

float make32bitFloat(const Byte* pBytes, bool little_endian)
{
  float f = 0;

  if (little_endian) {
    ((Byte*) (&f))[0] = pBytes[3];
    ((Byte*) (&f))[1] = pBytes[2];
    ((Byte*) (&f))[2] = pBytes[1];
    ((Byte*) (&f))[3] = pBytes[0];
  }
  else {
    ((Byte*) (&f))[0] = pBytes[0];
    ((Byte*) (&f))[1] = pBytes[1];
    ((Byte*) (&f))[2] = pBytes[2];
    ((Byte*) (&f))[3] = pBytes[3];
  }

  return f;
}

void unpack32BitFloats(float * dest, const Byte * source_bytes, int length, bool little_endian)
{
  int ii;
  int byte_ind = 0;
  for (ii = 0; ii < length; ii++) {
    dest[ii] = make32bitFloat(&source_bytes[byte_ind], little_endian);
    byte_ind += 4;
  }
}

void convertFloatToDouble(double * dest, const float * source, int length)
{
  for (int ii = 0; ii < length; ii++) {
    dest[ii] = (double) source[ii];
  }
}

static void sig_action(int signal, siginfo_t *s, void *user)
{
  // kill the glib main loop...
  if (g_main_loop_is_running(mainloop)) {
    g_main_loop_quit(mainloop);
  }
}

void install_signal_handler()
{
  struct sigaction action;
  action.sa_sigaction = sig_action;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;

  sigaction(SIGINT,  &action, NULL);
  sigaction(SIGTERM, &action, NULL);
  sigaction(SIGKILL, &action, NULL);
  sigaction(SIGHUP,  &action, NULL);
}

// ------------- Functions taken from microstrain driver for opening commport with proper settings and finding an attached microstrain device -----------

//scandev
//finds attached microstrain devices and prompts user for choice then returns selected portname
bool scandev(char * comm_port_name)
{

  FILE *instream;
  char devnames[255][255]; //allows for up to 256 devices with path links up to 255 characters long each
  int devct = 0; //counter for number of devices
  int i = 0;
  int j = 0;
  int userchoice = 0;

  char command[] = "find /dev/serial -print | grep -i microstrain"; //search /dev/serial for microstrain devices

  printf("Searching for devices...\n");

  instream = popen(command, "r"); //execute piped command in read mode

  if (!instream) { //SOMETHING WRONG WITH THE SYSTEM COMMAND PIPE...EXITING
    printf("ERROR BROKEN PIPELINE %s\n", command);
    return false;
  }

  for (i = 0; i < 255 && (fgets(devnames[i], sizeof(devnames[i]), instream)); i++) { //load char array of device addresses
    ++devct;
  }

  for (i = 0; i < devct; i++) {
    for (j = 0; j < sizeof(devnames[i]); j++) {
      if (devnames[i][j] == '\n') {
        devnames[i][j] = '\0'; //replaces newline inserted by pipe reader with char array terminator character
        break; //breaks loop after replacement
      }
    }
    printf("Device Found:\n%d: %s\n", i, devnames[i]);
  }

  //CHOOSE DEVICE TO CONNECT TO AND CONNECT TO IT (IF THERE ARE CONNECTED DEVICES)

  if (devct > 0) {
    printf("Number of devices = %d\n", devct);
    if (devct > 1) {
      printf("Please choose the number of the device to connect to (0 to %i):\n", devct - 1);
      while (scanf("%i", &userchoice) == 0 || userchoice < 0 || userchoice > devct - 1) { //check that there's input and in the correct range
        printf("Invalid choice...Please choose again between 0 and %d:\n", devct - 1);
        getchar(); //clear carriage return from keyboard buffer after invalid choice
      }
    }
    strcpy(comm_port_name, devnames[userchoice]);
    return true;
  }
  else {
    printf("No MicroStrain devices found.\n");
    return false;
  }

}

int setup_com_port(int comPort, speed_t baudRate)
{
  //Get the current options for the port...
  struct termios options;
  tcgetattr(comPort, &options);

  // set the desired baud rate (default for MicroStrain is 115200)
  // int baudRate = B115200;
  cfsetospeed(&options, baudRate);
  cfsetispeed(&options, baudRate);

  //set the number of data bits.
  options.c_cflag &= ~CSIZE; // Mask the character size bits
  options.c_cflag |= CS8;

  //set the number of stop bits to 1
  options.c_cflag &= ~CSTOPB;

  //Set parity to None
  options.c_cflag &= ~PARENB;

  //set for non-canonical (raw processing, no echo, etc.)
  options.c_iflag = IGNPAR; // ignore parity check close_port(int
  options.c_oflag = 0; // raw output
  options.c_lflag = 0; // raw input

  //Time-Outs -- won't work with NDELAY option in the call to open
  options.c_cc[VMIN] = 0; // block reading until RX x characers. If x = 0, it is non-blocking.
  options.c_cc[VTIME] = 100; // Inter-Character Timer -- i.e. timeout= x*.1 s

  //Set local mode and enable the receiver
  options.c_cflag |= (CLOCAL | CREAD);

  tcflush(comPort, TCIOFLUSH);

  //Set the new options for the port...
  int status = tcsetattr(comPort, TCSANOW, &options);

  if (status != 0) { //For error message

    printf("Configuring comport failed\n");
    return status;

  }

  //Purge serial port buffers
  tcflush(comPort, TCIOFLUSH);

  return comPort;
}

// Opens a com port with the correct settings for communicating with a MicroStrain 3DM-GX3-25 sensor
// Tweaked - split into two, added baud rate argument
int open_com_port(const char* comPortPath, speed_t baudRate)
{
  int comPort = open(comPortPath, O_RDWR | O_NOCTTY);

  if (comPort == -1) { //Opening of port failed

    printf("Unable to open com Port %s\n Errno = %i\n", comPortPath, errno);
    return -1;

  }

  return setup_com_port(comPort, baudRate);
}

/*
 * prints byte arrays in hex
 */
void print_array_char_hex(const unsigned char * array, int length)
{
  int ii;
  for (ii = 0; ii < length; ii++) {
    fprintf(stderr, "%02X ", (unsigned char) array[ii]);
  }
  fprintf(stderr, "\n");
}

/*
 * checksum function
 */
unsigned short cksum(const Byte * packet_bytes, int packet_length)
{
  unsigned short check_sum_val = 0;
  for (int ii = 0; ii < packet_length - 2; ii++) {
    check_sum_val += packet_bytes[ii];
  }
  return check_sum_val;
}

/*
 * sets continuous streaming operation
 */
bool set_continuous_mode(app_t* app)
{
  char set_mode_string[] = { 0xC4, 0xC1, 0x29, app->message_mode };

  cout << "Setting continuous mode" << endl;

  if (write(app->comm, set_mode_string, LENGTH_CONTINUOUS_MODE) != LENGTH_CONTINUOUS_MODE) {
    cerr << "Error writing command to set continuous mode" << endl;
    return false;
  }

  return true;
}

/*
 * stops continous streaming - command does not generate a response
 */
void stop_continuous_mode(app_t* app)
{
  char stop_mode_string[] = { 0xFA, 0x75, 0xB4 };

  if (write(app->comm, stop_mode_string, 3) != 3) {
    cerr << "Error writing command to stop continuous mode" << endl;
  }
}

/*
 * soft device reset - return to default settings
 */
void soft_reset(app_t* app)
{
  char soft_reset_string[] = { 0xFE, 0x9E, 0x3A };

  if (write(app->comm, soft_reset_string, 3) != 3) {
    cerr << "Error writing command to stop continuous mode" << endl;
  }
}

/*
 * sets/changes comms baud rate
 */
bool set_comms_baud_rate(app_t* app)
{
  Byte baud0, baud1, baud2, baud3;

  makeUnsignedInt32(app->baud_rate, &baud3, &baud2, &baud1, &baud0);

  char set_comms_baud_rate_string[] =
    { COMMS_SETTINGS_COMMAND, // Byte  1  : command
      0xC3,                   // Bytes 2-3: confirm intent
      0x55,
      0x01,                   // Byte  4  : port selector
      0x01,                   // Byte  5  : temporary change
      baud3,                  // Bytes 6-9: baud rate
      baud2,
      baud1,
      baud0,
      0x02,                   // Byte  10 : port config
      0x00                    // Byte  11 : reserved (zero)
    };

  if (app->verbose)
    cout << "Setting baud rate" << endl;

  if (write(app->comm, set_comms_baud_rate_string, LENGTH_COMMS_SETTINGS) != LENGTH_COMMS_SETTINGS) {
    cerr << "Error writing command to set comms baud rate" << endl;
    return false;
  }

  return true;
}

/*
 * sets/changes sensor update (data) rate and filter window size
 */
bool set_sampling_settings(app_t* app)
{
  Byte ocsb;       // orientation, coning & sculling byte
  Byte decu, decl; // decimation value upper & lower bytes
  Byte wndb;       // digital filter window size byte

  unsigned int decimation = 1000/app->data_rate;
  makeUnsignedInt16(decimation, &decu, &decl);

  // don't compute orientation if we're running at max rate
  if (app->data_rate == DATA_RATE_HIGH)
    ocsb = 0x02;
  else
    ocsb = 0x03;

  wndb = static_cast<Byte>(app->filter_window_size);

  char set_sampling_params_string[] =
    { SAMPLING_SETTINGS_COMMAND, // Byte  1    : command
      0xA8,                      // Bytes 2-3  : confirm intent
      0xB9,
      0x01,                      // Byte  4    : change params
      decu,                      // Bytes 5-6  : decimation value
      decl,
      0x00,                      // Bytes 7-8  : flags - orient
      ocsb,
      wndb,                      // Byte  9    : gyro/accel window size
      0x11,                      // Byte  10   : magneto window size
      0x00,                      // Byte  11-12: up compensation
      0x0A,
      0x00,                      // Byte  13-14: north compensation
      0x0A,
      0x01,                      // Byte  15   : low magneto power
      0x00,                      // Bytes 16-20: reserved (zeros)
      0x00,
      0x00,
      0x00,
      0x00
     };

  if (app->verbose)
    cout << "Setting sampling settings" << endl;

  if (write(app->comm, set_sampling_params_string, LENGTH_SAMPLING_SETTINGS) != LENGTH_SAMPLING_SETTINGS) {
    cerr << "Error writing command to set sampling settings" << endl;
    return false;
  }

  return true;
}

bool handle_message(app_t* app)
{
  microstrain_ins_t ins_message;
  memset(&ins_message, 0, sizeof(ins_message));
  int ins_timer;
  int64_t utime = bot_timestamp_now();

  float vals[9];

  if (app->verbose) {
    fprintf(stderr, "Received data packet:\n");
    print_array_char_hex((unsigned char *) app->input_buffer, app->message_size);
  }

  bool got_quat = false;
  bool success = true;

  switch (app->message_start_byte) {
  case ACC_ANG_MAG_ROT:
    {
      if (app->message_mode != ACC_ANG_MAG_ROT && !app->quiet)
        printf("error, received ACC_ANG_MAG_ROT instead of ACC_ANG_MAG\n");

      double rot[9];
      unpack32BitFloats(vals, &app->input_buffer[37], 9, app->little_endian);
      convertFloatToDouble(rot, vals, 9);

      // This libbot2 function is faulty:
      // bot_matrix_to_quat(rot, ins_message.quat);
      // Workaround (from mfallon, oct2011)
      double ms_rpy[] = { 0, 0, 0 };
      ms_rpy[0] = atan2(rot[5], rot[8]); // roll
      ms_rpy[1] = asin(-rot[2]); // pitch
      ms_rpy[2] = atan2(rot[1], rot[0]); // yaw
      bot_roll_pitch_yaw_to_quat(ms_rpy, ins_message.quat);

      got_quat = true;
      //fall into standard ins message handling
    }
  case ACC_ANG_MAG:
    {
      if (app->message_mode == ACC_ANG_MAG_ROT && !got_quat && !app->quiet)
        printf("error, received ACC_ANG_MAG instead of ACC_ANG_MAG_ROT (no quat received)\n");

      //get the data we care about
      unpack32BitFloats(vals, &app->input_buffer[1], 3, app->little_endian);
      convertFloatToDouble(ins_message.accel, vals, 3);
      bot_vector_scale_3d(ins_message.accel, GRAVITY);

      unpack32BitFloats(vals, &app->input_buffer[13], 3, app->little_endian);
      convertFloatToDouble(ins_message.gyro, vals, 3);

      unpack32BitFloats(vals, &app->input_buffer[25], 3, app->little_endian);
      convertFloatToDouble(ins_message.mag, vals, 3);

      //ins internal timer, currently not used
      ins_timer = make32UnsignedInt(&app->input_buffer[37], app->little_endian);

      ins_message.device_time = ((double) ins_timer) / 62500.0;

      if (app->do_sync) {
        ins_message.utime = bot_timestamp_sync(app->sync, ins_timer, utime);
      }
      else {
        ins_message.utime = utime;
      }

      microstrain_ins_t_publish(app->lcm, app->channel.c_str(), &ins_message);
      break;
    }

  case ACC_STAB:
    {
      if (app->message_mode != ACC_STAB && !app->quiet)
        printf("error: received unexpected ACC_STAB message\n");

      //get the data we care about
      unpack32BitFloats(vals, &app->input_buffer[1], 3, app->little_endian);
      convertFloatToDouble(ins_message.accel, vals, 3);
      bot_vector_scale_3d(ins_message.accel, GRAVITY);

      unpack32BitFloats(vals, &app->input_buffer[13], 3, app->little_endian);
      convertFloatToDouble(ins_message.gyro, vals, 3);

      unpack32BitFloats(vals, &app->input_buffer[25], 3, app->little_endian);
      convertFloatToDouble(ins_message.mag, vals, 3);

      //ins internal timer, currently not used
      ins_timer = make32UnsignedInt(&app->input_buffer[37], app->little_endian);

      ins_message.device_time = ((double) ins_timer) / 62500.0;

      if (app->do_sync) {
        ins_message.utime = bot_timestamp_sync(app->sync, ins_timer, utime);
      }
      else {
        ins_message.utime = utime;
      }
      microstrain_ins_t_publish(app->lcm, app->channel.c_str(), &ins_message);
      break;
    }

  case DANG_DVEL_MAG:
    {
      if (app->message_mode != DANG_DVEL_MAG && !app->quiet)
        printf("error: received unexpected DANG_DVEL_MAG message\n");

      //get the data we care about
      unpack32BitFloats(vals, &app->input_buffer[1], 3, app->little_endian);
      convertFloatToDouble(ins_message.gyro, vals, 3);
      bot_vector_scale_3d(ins_message.gyro, 1 / app->delta_t);

      unpack32BitFloats(vals, &app->input_buffer[13], 3, app->little_endian);
      convertFloatToDouble(ins_message.accel, vals, 3);
      bot_vector_scale_3d(ins_message.accel, 1 / app->delta_t);
      bot_vector_scale_3d(ins_message.accel, GRAVITY);

      unpack32BitFloats(vals, &app->input_buffer[25], 3, app->little_endian);
      convertFloatToDouble(ins_message.mag, vals, 3);

      //ins internal timer, currently not used
      ins_timer = make32UnsignedInt(&app->input_buffer[37], app->little_endian);

      ins_message.device_time = ins_timer * 16; // 1e6 / 62500.0;

      if (app->do_sync) {
        ins_message.utime = bot_timestamp_sync(app->sync, ins_timer, utime);
      }
      else {
        ins_message.utime = utime;
      }
      microstrain_ins_t_publish(app->lcm, app->channel.c_str(), &ins_message);
      break;
    }
  case CONTINUOUS_MODE_COMMAND:
    {
      fprintf(stderr, "Received continuous mode command echo\n");
      app->in_continuous_mode = true;
      break;
    }
  case SAMPLING_SETTINGS_COMMAND:
    {
      if (app->verbose)
        fprintf(stderr, "Recieved sampling settings command echo\n");

      if (app->data_rate != DATA_RATE_DEFAULT) {
        app->changed_data_rate = true;
      }

      success = set_continuous_mode(app);
      break;
    }
  case COMMS_SETTINGS_COMMAND:
    {
      if (app->verbose)
        fprintf(stderr, "Recieved comms baud rate command echo\n");

      // received echo at the current baud rate, now switch to desired baud rate
      if(setup_com_port(app->comm, app->baud_rate) < 0) {
        success = false;
      }
      else {
        app->changed_baud_rate = true;
        success = set_sampling_settings(app);
      }
      break;
    }
  default:
    {
      if (!app->quiet)
        fprintf(stderr, "Unknown message start byte: %d\n", app->message_start_byte);
      break;
    }
  }

  return success;
}

/*
 * gets data packets out of circular buffer.
 *
 * has 2 states, either looking for the header, or looking for the data + checksum bytes.  it waits until it
 * has all expected bytes before taking appropriate action.
 */
void unpack_packets(app_t * app)
{
  while (bot_ringbuf_available(app->read_buffer) >= app->expected_segment_length) {
    switch (app->current_segment) {
    case 's':
      bot_ringbuf_peek(app->read_buffer, 1, (uint8_t *) &app->message_start_byte);

      if (app->verbose)
        fprintf(stderr, "received message start byte: id=%d\n", app->message_start_byte);

      app->current_segment = 'p';

      switch (app->message_start_byte) {
      case ACC_ANG_MAG:
        app->expected_segment_length = LENGTH_ACC_ANG_MAG;
        break;
      case ACC_ANG_MAG_ROT:
        app->expected_segment_length = LENGTH_ACC_ANG_MAG_ROT;
        break;
      case ACC_STAB:
        app->expected_segment_length = LENGTH_ACC_STAB;
        break;
      case DANG_DVEL_MAG:
        app->expected_segment_length = LENGTH_DANG_DVEL_MAG;
        break;
      case CONTINUOUS_MODE_COMMAND:
        app->expected_segment_length = LENGTH_CONTINUOUS_MODE_ECHO;
        break;
      case SAMPLING_SETTINGS_COMMAND:
        app->expected_segment_length = LENGTH_SAMPLING_SETTINGS_ECHO;
        break;
      case COMMS_SETTINGS_COMMAND:
        app->expected_segment_length = LENGTH_COMMS_SETTINGS_ECHO;
        break;
      default:
        if (!app->quiet) {
          fprintf(stderr, "no match for message start byte %d\n", app->message_start_byte);
        }
        //read a byte and continue if we don't have a match
        bot_ringbuf_read(app->read_buffer, 1, (uint8_t *) &app->message_start_byte);
        app->current_segment = 's';
        break;
      }
      break;
    case 'p':
      bot_ringbuf_read(app->read_buffer, app->expected_segment_length, app->input_buffer);
      unsigned short transmitted_cksum = make16UnsignedInt(&app->input_buffer[app->expected_segment_length - 2],
          app->little_endian);
      unsigned short computed_cksum = cksum(app->input_buffer, app->expected_segment_length);
      if (computed_cksum != transmitted_cksum) {
        if (!app->quiet)
          fprintf(stderr, "Failed check sum! got: %d, expected: %d\n", transmitted_cksum, computed_cksum);
        break;
      }

      if (app->verbose)
        fprintf(stderr, "Passed checksum, handling message\n");

      bool message_success = handle_message(app);
      if (!message_success && !app->quiet)
        fprintf(stderr, "Message handling failed\n");

      app->current_segment = 's';
      app->expected_segment_length = 1;
      break;
    }

  }
}

/*
 * reads serial bytes from ardu as they become available from g_io_watch and then writes them into the circular buffer
 * and calls unpack_packets
 *
 */
static gboolean serial_read_handler(GIOChannel * source, GIOCondition condition, void * user)
{
  app_t * app = (app_t *) user;

  static uint8_t middle_buffer[INPUT_BUFFER_SIZE];

  //get number of bytes available
  int available = 0;

  if (ioctl(app->comm, FIONREAD, &available) != 0) {
    if (!app->quiet)
      fprintf(stderr, "ioctl check for bytes available didn't return 0, breaking read\n");
    return TRUE;
  }

  if (available > INPUT_BUFFER_SIZE) {
    if (!app->quiet)
      fprintf(stderr, "too many bytes available: %d, flushing input buffer\n", available);
    tcflush(app->comm, TCIFLUSH);
    return TRUE;
  }

  int num_read = read(app->comm, middle_buffer, available);

  if (num_read != available) {
    if (!app->quiet)
      fprintf(stderr, "warning, read %d of %d available bytes\n", num_read, available);
  }

  if (num_read > 0) {
    bot_ringbuf_write(app->read_buffer, num_read, middle_buffer);
  }

  unpack_packets(app);

  return TRUE;
}

int main(int argc, char **argv)
{
  app_t * app = new app_t();
  app->little_endian = systemLittleEndianCheck();

  // default settings
  app->verbose = 0;
  app->quiet = 0;
  app->message_mode = DANG_DVEL_MAG;
  app->data_rate = DATA_RATE_DEFAULT;
  app->baud_rate = BAUD_RATE_DEFAULT;
  app->delta_t = DELTA_ANG_VEL_DT_DEFAULT;
  app->filter_window_size = FILTER_WINDOW_SIZE_DEFAULT;
  app->do_sync = true;
  app->channel = "MICROSTRAIN_INS";

  app->changed_baud_rate = false;
  app->changed_data_rate = false;
  app->in_continuous_mode = false;

  string user_comm_port_name;
  bool auto_comm = true;
  char data_rate = 'l';

  bool acc_ang_mag_rot = false;
  bool acc_ang_mag = false;
  bool acc_stab = false;

  ConciseArgs opt(argc, argv);
  opt.add(app->verbose, "v", "verbose");
  opt.add(app->quiet, "q", "quiet");
  opt.add(user_comm_port_name, "d", "dev", "Device file to connect to (default is automatic scan)");
  opt.add(data_rate, "b", "rate", "Update rate: 'l'-low (100 Hz), 'm'-medium (500 Hz), 'h'-high (1000 Hz)");
  opt.add(app->filter_window_size, "w", "window", "Gyro & accel digital filter window size: 1-32");
  opt.add(acc_ang_mag_rot, "r", "quat");
  opt.add(acc_ang_mag, "n", "no_delta");
  opt.add(acc_stab, "f", "filter");
  opt.add(app->channel, "c", "channel", "LCM message output channel");
  opt.add(app->do_sync, "s", "time_sync");
  opt.parse();

  if (opt.wasParsed("dev"))
    auto_comm = false;

  // data rate (which also determines baud rate)
  if (opt.wasParsed("rate")) {
    switch (data_rate) {
    case 'l':
      // Default
      break;
    case 'm':
      app->data_rate = DATA_RATE_MED;
      app->baud_rate = BAUD_RATE_MED;
      app->delta_t = DELTA_ANG_VEL_DT_MED;
      break;
    case 'h':
      app->data_rate = DATA_RATE_HIGH;
      app->baud_rate = BAUD_RATE_HIGH;
      app->delta_t = DELTA_ANG_VEL_DT_HIGH;
      break;
    default:
      cerr << "Unknown update rate flag - using default rate" << endl;
      break;
    }
  }

  if(!app->quiet)
    cout << "Setting data rate to " << app->data_rate << " Hz" << endl;

  // make sure filter window size isn't too big or small
  if (opt.wasParsed("window")) {
    if (app->filter_window_size < FILTER_WINDOW_SIZE_MIN) {
      app->filter_window_size = FILTER_WINDOW_SIZE_DEFAULT;
      cerr << "Digital filter window size too small, using default size" << endl;
    }
    else if (app->filter_window_size > FILTER_WINDOW_SIZE_MAX) {
      app->filter_window_size = FILTER_WINDOW_SIZE_DEFAULT;
      cerr << "Digital filter window size too large, using default size" << endl;
    }

    if (!app->quiet)
      cout << "Setting digital filter window size to " << app->filter_window_size << endl;
  }

  // modes are mutually exlusive
  if (acc_stab) {
    app->message_mode = ACC_STAB;
  }
  else if (acc_ang_mag) {
    app->message_mode = ACC_ANG_MAG;
  }
  else if (acc_ang_mag_rot) {
    if(app->data_rate != DATA_RATE_HIGH) {
      app->message_mode = ACC_ANG_MAG_ROT;
    }
    else {
      cout << "Can't compute orientation at high speed, using delta angle + delta velocity mode instead" << endl;
      app->message_mode = DANG_DVEL_MAG;
    }
  }

  if (!app->quiet)
    fprintf(stderr, "Little endian = %d\n", (int) app->little_endian);

  mainloop = g_main_loop_new(NULL, FALSE);
  app->lcm = bot_lcm_get_global(NULL);
  app->utime_prev = bot_timestamp_now();
  app->sync = bot_timestamp_sync_init(62500, (int64_t) 68719 * 62500, 1.001);
  //  app->param = bot_param_new_from_server(app->lcm, 1);

  app->read_buffer = bot_ringbuf_create(INPUT_BUFFER_SIZE);

  if (auto_comm)
    scandev(app->comm_port_name);
  else
    strcpy(app->comm_port_name, user_comm_port_name.c_str());

  // initialize comm port at default baud rate
  app->comm = open_com_port(app->comm_port_name, BAUD_RATE_DEFAULT);
  if (app->comm < 0) {
    exit(1);
  }

  // install signal handler
  install_signal_handler();

  // simple state machine
  if (app->data_rate == DATA_RATE_DEFAULT && app->filter_window_size == FILTER_WINDOW_SIZE_DEFAULT) {
    // set continous mode and we're done
    if (!set_continuous_mode(app)) {
      exit(1);
    }
  }
  else if (app->data_rate == DATA_RATE_DEFAULT) {
    // set filter window size, then set continous mode
    if(!set_sampling_settings(app)) {
      exit(1);
    }
  }
  else {
    // set baud rate, then sampling settings, then continuous mode
    if(!set_comms_baud_rate(app)) {
      exit(1);
    }
  }

  app->current_segment = 's';
  app->expected_segment_length = 1;
  GIOChannel * ioc = g_io_channel_unix_new(app->comm);
  g_io_add_watch_full(ioc, G_PRIORITY_HIGH, G_IO_IN, (GIOFunc) serial_read_handler, (void *) app, NULL);

  g_main_loop_run(mainloop);

  // received signal - soft reset to cleanup before quitting
  soft_reset(app);

  close(app->comm);
  return 0;
}
