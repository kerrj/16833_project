#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include "util.hpp"

namespace gs = gtsam::symbol_shorthand;


class Solver {
  protected:
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    gtsam::LevenbergMarquardtParams params;

  Solver() {

  }

  public:

  void update_graph(
      Pose new_pose,
      std::vector<std::pair<int, int> > matches,
      std::vector<Line> > new_lines
      ) {

    // add new_pose variable
    // for all matched lines, we add the factor
    // for any unmatched lines, we add a new landmark variable

    for (auto& n: new_lines) {
      values.insert(gs::L(lid), line_to_p2(n));
    }

  }

      matches, new_lines) {
std::pair<, )
  }

}

