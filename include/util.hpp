#ifndef UTIL_HPP
#define UTIL_HPP
#include <cmath>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

#include "Line.hpp"
#include "pose.hpp"

/*
utility class which defines angle wrapping and stuff like that
*/
double wrapAng(double th) {
  while (th >= M_PI) th -= 2 * M_PI;
  while (th < -M_PI) th += 2 * M_PI;
  return th;
}

Line Point2_to_Line(gtsam::Point2 p) {
  return Line(p.x(), p.y());
}

Pose Pose2_to_Pose(gtsam::Pose2 p) {
  return Pose(p.x(), p.y(), p.theta());
}


#endif
