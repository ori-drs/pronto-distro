#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <lcm/lcm.h>
#include <ConciseArgs>
#include <string>
#include <map>
using namespace std;

bool tokenizeChangeMap(const std::string &changeMapStr, std::map<std::string, std::string> & changeMap, bool quiet)
{
  size_t tokStart = 0, tokEnd = 0;
  while (tokEnd != string::npos)
  {
    tokEnd = changeMapStr.find("|", tokStart);
    size_t tokLen = tokEnd - tokStart;
    if (tokEnd == string::npos) //handle special case of last token
      tokLen = string::npos;
    string token = changeMapStr.substr(tokStart, tokLen);

    //split token on equals sign
    size_t eqSplit = token.find("=");
    if (eqSplit == string::npos) {
      fprintf(stderr, "ERROR: malformed change map format... no '=' found in token %s!\n", token.c_str());
      return false;
    }
    string inChan = token.substr(0, eqSplit);
    string outChan = token.substr(eqSplit + 1, string::npos);
    changeMap[inChan] = outChan;
    if (!quiet) {
      fprintf(stderr, "Mapping %s -> %s\n", inChan.c_str(), outChan.c_str());
    }

    // If at end, use start=maxSize.  Else use start=end+delimiter.
    tokStart = tokEnd + 1;
  }

  return true;

}

int main(int argc, char ** argv)
{
  bool quiet = false;
  lcm_eventlog_t * read_log;
  lcm_eventlog_t * write_log;

  string changeMapStr;
  string outLogFname;

  ConciseArgs opt(argc, argv, "log_name");
  opt.add(outLogFname, "o", "out_file", "output to this file instead of overwriting input");
  opt.add(changeMapStr, "c", "change_map", "Channels to change in format: INCHAN1=OUTCHAN1|INCHAN2=OUTCHAN2|...", true);
  opt.add(quiet, "q", "quiet");

  string inLogFname;
  opt.parse(inLogFname);
  if (!opt.wasParsed("o"))
    outLogFname = inLogFname;

  string tmpLogFname = outLogFname + "_tmp";

  map<string, string> changeMap;
  if (!tokenizeChangeMap(changeMapStr, changeMap, quiet)) {
    opt.usage(true);
  }

  read_log = lcm_eventlog_create(inLogFname.c_str(), "r");
  if (!read_log) {
    fprintf(stderr, "couldn't open read log file\n");
    return 1;
  }

  write_log = lcm_eventlog_create(tmpLogFname.c_str(), "w");
  if (!write_log) {
    fprintf(stderr, "couldn't open write log file\n");
    return 1;
  }

  int numRenamed = 0;
  while (1) {
    int i;
    lcm_eventlog_event_t *event = lcm_eventlog_read_next_event(read_log);
    if (!event)
      break;

    map<string, string>::iterator it = changeMap.find(event->channel);
    if (it != changeMap.end()) {
      free(event->channel);
      event->channellen = it->second.size();
      event->channel = strdup(it->second.c_str());
      numRenamed++;
    }
    lcm_eventlog_write_event(write_log, event);
    lcm_eventlog_free_event(event);
  }
  lcm_eventlog_destroy(write_log);
  lcm_eventlog_destroy(read_log);

  int result;
  result = rename(tmpLogFname.c_str(), outLogFname.c_str());
  if (result != 0)
    perror("Error renaming file");
  if (!quiet)
    fprintf(stderr, "All done. Renamed %d messages\n", numRenamed);
  return 0;
}
