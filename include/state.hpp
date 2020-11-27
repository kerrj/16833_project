#ifndef STATE_HPP
#define STATE_HPP

#include <cmath>
#include <memory>
#include "Line.hpp"
#include "LogReader.hpp"
#include "pose.hpp"
#include "util.hpp"
namespace Project {

class State {
public:
  std::vector<Line> landmarks;
  Pose pose;

  void set_landmarks(std::vector<Line> updated_landmarks) {
    landmarks = updated_landmarks;
  }

  void update_pose(std::shared_ptr<Odometry> o) {
    double RT = pose.beta;
    double opt1 = RT + o->th;
    double opt2 = o->x / 6.0;
    double opt3 = o->th / 2.0;
    double opt4 = RT + opt3;
    double dx = opt2 * (std::cos(RT) + 4.0 * std::cos(opt4) + std::cos(opt1));
    double dy = opt2 * (std::sin(RT) + 4.0 * std::sin(opt4) + std::sin(opt1));
    pose.x += dx;
    pose.y += dy;
    pose.beta = opt1;
  }
};

}  // namespace Project
#endif
