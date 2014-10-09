/*
 *
 * LCM vicon publisher
 *
 * origina author Brian J. Julian
 */

// includes
#include <lcm/lcm.h>
#include <sys/time.h>
#include <unistd.h>
#include <lcmtypes/vicon_body_t.h>


#include "data_stream_client.hpp"

// defaults
#define DEFAULT_MOCAP_HOST_ADDR "192.168.20.99"
//#define DEFAULT_MOCAP_HOST_ADDR "192.168.1.104"
#define DEFAULT_MOCAP_HOST_PORT "801"

#define dbg(...) fprintf(stderr, __VA_ARGS__)

static int64_t _timestamp_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


// namespaces
using namespace ViconDataStreamSDK::CPP;

namespace {
std::string Adapt(const bool i_Value)
{
  return i_Value ? "True" : "False";
}

std::string Adapt(const Direction::Enum i_Direction)
{
  switch (i_Direction) {
  case Direction::Forward:
    return "Forward";
  case Direction::Backward:
    return "Backward";
  case Direction::Left:
    return "Left";
  case Direction::Right:
    return "Right";
  case Direction::Up:
    return "Up";
  case Direction::Down:
    return "Down";
  default:
    return "Unknown";
  }
}

std::string Adapt(const DeviceType::Enum i_DeviceType)
{
  switch (i_DeviceType) {
  case DeviceType::ForcePlate:
    return "ForcePlate";
  case DeviceType::Unknown:
  default:
    return "Unknown";
  }
}

std::string Adapt(const Unit::Enum i_Unit)
{
  switch (i_Unit) {
  case Unit::Meter:
    return "Meter";
  case Unit::Volt:
    return "Volt";
  case Unit::NewtonMeter:
    return "NewtonMeter";
  case Unit::Newton:
    return "Newton";
  case Unit::Unknown:
  default:
    return "Unknown";
  }
}
}

// class
class DataStreamClient {
public:

  // constructor
  DataStreamClient(lcm_t * lcm, std::string vicon_hostname);

  // destructor
  ~DataStreamClient();

  void run();
  void stop();

private:
  //lcm
  lcm_t * _lcm;

  // data stream client
  ViconDataStreamSDK::CPP::Client _vicon_client;

  // mocap tcp client
  std::string mocap_host_addr_;
  std::string mocap_host_port_;

};

// class constructor
DataStreamClient::DataStreamClient(lcm_t * lcm, std::string vicon_hostname) :
  _lcm(lcm)
{
  // local stack
  Output_GetAxisMapping axis_mapping;
  Output_GetVersion version_number;

  _vicon_client.Connect(vicon_hostname);
  if (!_vicon_client.IsConnected().Connected) {
    fprintf(stderr, "Error connecting to %s\n", vicon_hostname.c_str());
    exit(1);
  }

  // enable segment data
  _vicon_client.EnableSegmentData();
  if (!_vicon_client.IsSegmentDataEnabled().Enabled) {
    fprintf(stderr, "Error enabling segment data\n");
    _vicon_client.Disconnect();
    return;
  }

  // set streaming mode
  _vicon_client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);
  // _vicon_client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch);
  // _vicon_client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);

  // set global axes
  _vicon_client.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up);
  axis_mapping = _vicon_client.GetAxisMapping();
  dbg("Axis Mapping: X-%s Y-%s Z-%s\n", Adapt(axis_mapping.XAxis).c_str(), Adapt(axis_mapping.YAxis).c_str(), Adapt(axis_mapping.ZAxis).c_str());

}

// class destructor
DataStreamClient::~DataStreamClient(void)
{
  //TODO: cleanup
}

// mocap position thread
void DataStreamClient::run(void)
{

  while (true) //TODO: exit cleanly?
  {

    // get frame
    while (_vicon_client.GetFrame().Result != Result::Success) {
      dbg("Couldn't GetFrame()\n");
      usleep(1000);
    }

    // get frame number
    Output_GetFrameNumber frame_number = _vicon_client.GetFrameNumber();

    // get timecode and populate subject message
    //    Output_GetTimecode time_code = _vicon_client.GetTimecode();
    vicon_body_t msg;
    msg.utime = _timestamp_now();

    // get subject count
    uint subject_count = _vicon_client.GetSubjectCount().SubjectCount;

    // loop through subjects
    for (uint subject_index = 0; subject_index < subject_count; subject_index++) {
      // get subject name
      std::string subject_name = _vicon_client.GetSubjectName(subject_index).SubjectName;

      // get root segment name
      std::string root_segment_name = _vicon_client.GetSubjectRootSegmentName(subject_name).SegmentName;

      // get number of segments
      uint segment_count = _vicon_client.GetSegmentCount(subject_name).SegmentCount;

      // loop through segments
      for (uint segment_index = 0; segment_index < segment_count; segment_index++) {
        // get segment name
        std::string segment_name = _vicon_client.GetSegmentName(subject_name, segment_index).SegmentName;

        // check if root segment
        if (segment_name == root_segment_name) { //TODO: handle articulated bodies
          // get segment translation
          // Output_GetSegmentStaticTranslation segment_translation = _vicon_client.GetSegmentStaticTranslation(subject_name, segment_name);
          Output_GetSegmentGlobalTranslation segment_translation = _vicon_client.GetSegmentGlobalTranslation(
              subject_name, segment_name);

          // get segment rotation
          //Output_GetSegmentStaticRotationQuaternion segment_rotation = _vicon_client.GetSegmentStaticRotationQuaternion(subject_name, segment_name);
          Output_GetSegmentGlobalRotationQuaternion segment_rotation =
              _vicon_client.GetSegmentGlobalRotationQuaternion(subject_name, segment_name);
          //           Output_GetSegmentGlobalRotationEulerXYZ   segment_rotation = _vicon_client.GetSegmentGlobalRotationEulerXYZ(subject_name, segment_name);

          // populate message with position
          for (int i = 0; i < 3; i++)
            msg.trans[i] = segment_translation.Translation[i]/1000.0; //vicon data is in mm
	  msg.quat[0] = segment_rotation.Rotation[3]; //vicon is x,y,z,w
	  msg.quat[1] = segment_rotation.Rotation[0];
	  msg.quat[2] = segment_rotation.Rotation[1];
	  msg.quat[3] = segment_rotation.Rotation[2];


          std::string channel = "VICON_" + subject_name;
          vicon_body_t_publish(_lcm, channel.c_str(), &msg);

          // break from segment for loop
          break;
        }
      }
    }

  }

  // disconnect client
  dbg("Subject position thread terminated properly, disconnecting client\n");
  _vicon_client.Disconnect();
}

// main function
int main(int argc, char **argv)
{
  setlinebuf(stdout);

  lcm_t * lcm = lcm_create(NULL);

  //TODO: get host and port from command line
  std::string mocap_host_addr = DEFAULT_MOCAP_HOST_ADDR;
  std::string mocap_host_port = DEFAULT_MOCAP_HOST_PORT;

  if(argc > 1) {
      mocap_host_addr = argv[1];
  }

  std::string vicon_host = mocap_host_addr + ":" + mocap_host_port;
  printf("Vicon adddress: %s\n", vicon_host.c_str());

  // create class instance/ connect to server
  DataStreamClient *data_stream_client = new DataStreamClient(lcm, vicon_host);

  data_stream_client->run();

  // return success
  return (0);
}
