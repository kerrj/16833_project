#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <map>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include "LineFactor.hpp"

namespace Project {

class Solver {
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  gtsam::NonlinearISAM isam = gtsam::NonlinearISAM(25);

  gtsam::SharedNoiseModel odometry_noise = 
    gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1)); // TODO: change covariances
  gtsam::SharedNoiseModel measurement_noise = 
    gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.2, 0.2)); // TODO: change covariances

  int num_poses = 0;
  int num_landmarks = 0;
  std::map<int, int> landmark_to_ref_pose = std::map<int, int>();
  gtsam::Pose2 prev_pose;

  Solver() {
    // add a pose variable with a prior at (0, 0, 0)
    gtsam::Key pose_key = gtsam::symbol('P', num_poses++);
    values.insert(pose_key, gtsam::Pose2());
    graph.add(gtsam::PriorFactor<gtsam::Pose2>(pose_key, gtsam::Pose2(), odometry_noise));
  }

  public:

  void update(
      std::vector<std::pair<Line, int> > matches,
      std::vector<Line> new_lines
    ) {

    if (new_lines.size() == 0) return;

    Pose new_pose_narnia = new_lines[0].ref_frame;
    gtsam::Pose2 new_pose = new_pose_narnia.to_Pose2();

    // add new_pose variable
    gtsam::Key prev_pose_key = gtsam::symbol('P', num_poses-1);
    gtsam::Key next_pose_key = gtsam::symbol('P', num_poses++);
    values.insert(next_pose_key, new_pose);

    // add factor between new and previous pose
    gtsam::Pose2 odometry(new_pose.x() - prev_pose.x(), new_pose.y() - prev_pose.y(), new_pose.theta() - prev_pose.theta());
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(prev_pose_key, next_pose_key, odometry, odometry_noise));

    // update prev_pose for the next valid call to update_graph
    prev_pose = new_pose;

    // add factor for all matched lines
    for (auto match : matches) {
      Line line = match.first;
      gtsam::Key landmark_key = gtsam::symbol('L', match.second);
      gtsam::Key ref_pose_key = gtsam::symbol('P', landmark_to_ref_pose[match.second]);
      graph.add(LineFactor(ref_pose_key, next_pose_key, landmark_key, line.r, line.th, measurement_noise));
    }

    // add variables and factors for all newly observed lines
    for (auto new_line_narnia : new_lines) {
      gtsam::Key new_line_key = gtsam::symbol('L', num_landmarks++);
      gtsam::Point2 new_line = new_line_narnia.to_Point2();
      values.insert(new_line_key, new_line);
      graph.add(LineFactor(next_pose_key, next_pose_key, new_line_key, new_line_narnia.r, new_line_narnia.th, measurement_noise));
      landmark_to_ref_pose[num_landmarks] = num_poses;
    }

    isam.update(graph, values);
  }

};
} // namespace Project

#endif 

