#ifndef POSE_HPP
#define POSE_HPP

#include <gtsam/geometry/Pose2.h>

namespace Project {
class Pose {
 public:
  Pose(double _x, double _y, double _beta) : x(_x), y(_y), beta(_beta){};
  // destructor
  virtual ~Pose() {}

  Pose(const Pose &p) {
    x = p.x;
    y = p.y;
    beta = p.beta;
  }
  Pose() : x(0), y(0), beta(0) {}
  double x, y, beta;

  gtsam::Pose2 to_Pose2() {
    return gtsam::Pose2(x, y, beta);
  }

};

Project::Pose Pose2_to_Pose(gtsam::Pose2 p) {
  return Project::Pose(p.x(), p.y(), p.theta());
}

std::ostream& operator << (std::ostream& outs, const Pose& p) {
  return outs << "Pose(" << p.x << "," << p.y << "," << p.beta << ")";
}

}  // namespace Project

#endif
