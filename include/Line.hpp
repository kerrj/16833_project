#ifndef LINE_HPP
#define LINE_HPP
#include <cmath>
#include "pose.hpp"
#include "util.hpp"
#include <gtsam/geometry/Point2.h>

namespace Project {
class Line {
public:
  Pose ref_frame;
  double r, th;
  constexpr static const double R_VAR = 0.005, TH_VAR = .003;

  Line(Pose _p, double _r, double _th) : ref_frame(_p), r(_r), th(_th){};
  Line() : ref_frame(Pose()), r(0), th(0) {}

  virtual ~Line() {}

  double distance(Line other) {
    Line self_prime = convert_coords(other.ref_frame);
    double r_dist = std::pow(self_prime.r - other.r, 2);
    double th_dist = std::pow(wrapAng(self_prime.th - other.th), 2);
    return std::sqrt(r_dist + th_dist);
  }

  // distance from this line to a point (x,y) that's also relative to ref_frame
  double distance_to_point(double x, double y) {
    double u_x = std::cos(th);
    double u_y = std::sin(th);
    return std::abs(u_x*x + u_y*y - r);
  }

  /*
  Converts the coordinates of the line to the new frame given
  */
  Line convert_coords(Pose new_frame) {
    double th_prime = wrapAng(th + ref_frame.beta - new_frame.beta);
    double dr = (new_frame.x - ref_frame.x) * std::cos(ref_frame.beta + th) +
                (new_frame.y - ref_frame.y) * std::sin(ref_frame.beta + th);
    double r_prime = r - dr;
    if (r_prime < 0) {
      // always make sure that the radius is positive
      r_prime = -r_prime;
      th_prime = wrapAng(th_prime + M_PI);
    }
    return Line(new_frame, r_prime, th_prime);
  }

  gtsam::Point2 to_Point2() {
    return gtsam::Point2(r, th);
  }

};

Project::Line Point2_to_Line(gtsam::Point2 line, Pose ref_pose) {
  return Project::Line(ref_pose, line.x(), line.y());
}

std::ostream& operator << (std::ostream& outs, const Line& l) {
  return outs << "Line(" << l.ref_frame << " r,th=(" << l.r << "," << l.th << ")";
}

}  // namespace Project

#endif
