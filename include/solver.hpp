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
#include <iostream>

namespace Project {

class Solver {
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  gtsam::NonlinearISAM isam = gtsam::NonlinearISAM(50);

  gtsam::SharedNoiseModel odometry_noise =
    gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.05, 0.05, 0.1)); // TODO: change covariances
  gtsam::SharedNoiseModel measurement_noise =
    gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(Line::R_VAR, Line::TH_VAR)); // TODO: change covariances

  int num_poses = 0;
  int num_landmarks = 0;
  std::map<int, int> landmark_to_ref_pose = std::map<int, int>();
  gtsam::Pose2 prev_pose;

  public:
  Solver() {
    // add a pose variable with a prior at (0, 0, 0)
    gtsam::Key pose_key = gtsam::symbol('P', num_poses++);
    values.insert(pose_key, gtsam::Pose2());
    graph.add(gtsam::PriorFactor<gtsam::Pose2>(pose_key, gtsam::Pose2(), odometry_noise));
    prev_pose = gtsam::Pose2();
  }


  void update(
      Pose new_pose_narnia,
      std::vector<std::pair<Line, int> > matches,
      std::vector<Line> new_lines
    ) {

    gtsam::Pose2 new_pose = new_pose_narnia.to_Pose2();

    // add new_pose variable
    gtsam::Key prev_pose_key = gtsam::symbol('P', num_poses-1);
    if (num_poses > 2) prev_pose = isam.estimate().at<gtsam::Pose2>(prev_pose_key); // TODO: use this for prev_pose? could also pass in odometry directly
    gtsam::Key next_pose_key = gtsam::symbol('P', num_poses++);
    values.insert(next_pose_key, new_pose);

    // add factor between new and previous pose
    gtsam::Pose2 odometry(new_pose.x() - prev_pose.x(), new_pose.y() - prev_pose.y(), wrapAng(new_pose.theta() - prev_pose.theta()));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(prev_pose_key, next_pose_key, odometry, odometry_noise));

    // update prev_pose for the next valid call to update_graph
    prev_pose = new_pose;

    // add factor for all matched lines
    // for (auto match : matches) {
    //   Line line = match.first;
    //   gtsam::Key landmark_key = gtsam::symbol('L', match.second);
    //   gtsam::Key ref_pose_key = gtsam::symbol('P', landmark_to_ref_pose[match.second]);

    //   graph.add(LineFactor(ref_pose_key, next_pose_key, landmark_key, line.r, line.th, measurement_noise));
    // }

    // // add variables and factors for all newly observed lines
    // //std::cout << "narnina loop" << std::endl;
    // for (auto new_line_narnia : new_lines) {
    //   landmark_to_ref_pose[num_landmarks] = num_poses - 1;
    //   gtsam::Key new_line_key = gtsam::symbol('L', num_landmarks++);
    //   gtsam::Point2 new_line = new_line_narnia.to_Point2();

    //   values.insert(new_line_key, new_line);
    //   graph.add(gtsam::PriorFactor<gtsam::Point2>(new_line_key,new_line,measurement_noise));
    //   // LineFactor factor(next_pose_key, next_pose_key, new_line_key, new_line_narnia.r, new_line_narnia.th, measurement_noise);
    //   // graph.add(factor);
    // }

    // TODO: get rid of all these debug prints
    if (num_poses > 2) isam.estimate().at<gtsam::Pose2>(prev_pose_key).print();
    values.at<gtsam::Pose2>(next_pose_key).print();
    isam.update(graph, values);
    odometry.print();
    isam.estimate().at<gtsam::Pose2>(prev_pose_key).print();
    isam.estimate().at<gtsam::Pose2>(next_pose_key).print();
    std::cout << std::endl;
    graph.resize(0);
    values.clear();
  }

  std::vector<Line> get_landmark_values() {
    std::vector<Line> updated_landmarks;

    gtsam::Values estimate_values = isam.estimate();

    for (auto k: estimate_values.keys()) {
      // if landmark value
      if (gtsam::symbolChr(k) == 'L') {
        int opt_land_idx = gtsam::symbolIndex(k);
        gtsam::Point2 opt_land = estimate_values.at<gtsam::Point2>(k);;
        gtsam::Key opt_pose_key = gtsam::symbol('P', landmark_to_ref_pose[opt_land_idx]);
        assert (estimate_values.exists(opt_pose_key));
        gtsam::Pose2 opt_pose = estimate_values.at<gtsam::Pose2>(opt_pose_key);
        updated_landmarks.push_back(Point2_to_Line(opt_land, Pose2_to_Pose(opt_pose)));
      }
    }

    return updated_landmarks;
  }

  Pose get_last_pose() {
    gtsam::Values estimate_values = isam.estimate();
    gtsam::Key pose_key = gtsam::symbol('P',num_poses-1);
    gtsam::Pose2 p = estimate_values.at<gtsam::Pose2>(pose_key);
    return Pose2_to_Pose(p);
  }
};
} // namespace Project

#endif

