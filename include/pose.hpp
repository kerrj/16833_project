#ifndef POSE_HPP
#define POSE_HPP
namespace Project {
class Pose {
 public:
  Pose(double _x, double _y, double _beta) : x(_x), y(_y), beta(_beta){};
  Pose(const Pose &p) {
    x = p.x;
    y = p.y;
    beta = p.beta;
  }
  void print(bool line = false) {
    std::cout << "Pose(" << x << "," << y << "," << beta << ")";
    if (line) std::cout << std::endl;
  }
  Pose() : x(0), y(0), beta(0) {}
  double x, y, beta;
};
}  // namespace Project
#endif