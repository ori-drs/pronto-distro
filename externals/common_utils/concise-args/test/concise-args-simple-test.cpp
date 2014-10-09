#include <ConciseArgs>
int main(int argc, char ** argv)
{
  bool bl = false;  int in;  float flt = -9; double dbl=3.14;
  ConciseArgs parser(argc, argv);
  parser.add(bl,  "b", "bools", "do bools work?"); // Parse -b/--bools, setting bl accordingly
  parser.add(in,  "i", "ints",  "do ints work?", true); // The "true" means this argument is mandatory
  parser.add(flt, "f", "floats"); //I'm too lazy to provide a description
  parser.add(dbl, "d"); //I'm too lazy to even provide a longName
  parser.parse();
  return 0;
}
