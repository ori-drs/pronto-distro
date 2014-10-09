#include <lcm/lcm.h>
#include <occ_map/PixelMap.hpp>
#include <string>

using namespace std;
using namespace occ_map;

class app_t {
public:
  lcm_t * lcm;
  string filename;
};

static void pixel_map_handler(const lcm_recv_buf_t *rbuf, const char *channel, const occ_map_pixel_map_t *msg,
    void *user)
{
  app_t *self = (app_t*) user;
  if (msg->data_type == OCC_MAP_PIXEL_MAP_T_TYPE_FLOAT) {
    FloatPixelMap * pix_map = new FloatPixelMap(msg);
    fprintf(stderr, "saving float map\n");
    pix_map->saveToFile(self->filename.c_str());
    delete pix_map;
  }
  else if (msg->data_type == OCC_MAP_PIXEL_MAP_T_TYPE_UINT8) {
    Uint8PixelMap * pix_map = new Uint8PixelMap(msg);
    fprintf(stderr, "saving uint8 map\n");
    pix_map->saveToFile(self->filename.c_str());
    delete pix_map;
  }
  else if (msg->data_type == 0) {
    fprintf(stderr, "Warning, pixmap datatype = 0, assuming it's a float!\n");
    FloatPixelMap * pix_map = new FloatPixelMap(msg);
    pix_map->saveToFile(self->filename.c_str());
    delete pix_map;
  }
  else {
    fprintf(stderr, "pixmap datatype %d not handled\n", msg->data_type);
  }

}

void usage(char * name)
{
  fprintf(stderr, "usage: %s filename channel [logfile]\n", name);
  exit(1);
}

int main(int argc, char ** argv)
{
  app_t app;
  if (argc < 3 || argc >4)
    usage(argv[0]);

  app.filename = argv[1];
  char * channel = argv[2];

  const  char * provider = NULL;
  string logfile,lcmurl;
  if (argc>3){
    logfile = argv[3];
    lcmurl = "file://" + logfile + "?speed=0";
    provider = lcmurl.c_str();
  }
  app.lcm = lcm_create(provider);


  occ_map_pixel_map_t_subscribe(app.lcm, channel, pixel_map_handler, &app);

  while (true){
    int ret = lcm_handle(app.lcm);
    if (ret!=0)
      break;
  }
}

