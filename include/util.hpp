#ifndef UTIL_HPP
#define UTIL_HPP
#include <cmath>
/*
utility class which defines angle wrapping and stuff like that
*/
double wrapAng(double th) {
  while (th >= M_PI) th -= 2 * M_PI;
  while (th < -M_PI) th += 2 * M_PI;
  return th;
}
#endif