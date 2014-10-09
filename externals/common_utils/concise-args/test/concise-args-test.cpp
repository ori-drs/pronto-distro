#include <stdio.h>
#include <ConciseArgs>

int main(int argc, char ** argv)
{
  ConciseArgs parser(argc, argv, "map-name");

  bool b = false;
  int i = -8;
  float f = -9;
  double d = 3.1459;
  char c = 'z';
  std::string s = "asdf";
  parser.add(b, "b", "bools", "do bools work?");
  parser.add(i, "i", "ints", "do ints work?", true);
  parser.addUsageSeperator("\n-------------------------------------------");
  parser.addUsageSeperator("   Other Options");
  parser.addUsageSeperator("-------------------------------------------\n");
  parser.add(f, "f", "floats"); //I'm too lazy to provide a description
  parser.add(d, "d"); //I'm too lazy to even put in a longname
  parser.add(s, "s", "strings", "do strings work ?");
  parser.add(c, "c", "chars", "do chars work ?");

  parser.usage(false);
  std::cerr << "\n\n";

  std::string req1;
  int req2;
  parser.parse(req1, req2);
  std::cerr << "req1 is :" << req1 << " req2 is :" << req2 << "\n";
  std::cerr << "i:" << i << " b:" << b << " f:" << f << " d:" << d << " s:" << s << " c:" << c << "\n";

  if (parser.wasParsed("b")) {
    std::cerr << "the bool was parsed!\n";
  }
  else {
    std::cerr << "the bool was NOT parsed!\n";
  }

//  //this should force the usage to get printed
//  if (parser.wasParsed("not there")) {
//    std::cerr << "uhh... oh, something went wrong! THIS SHOULDN'T HAVE BEEN PRINTED \n";
//  }

  return 0;
}
