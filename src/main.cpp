#include <iostream>
#include <fstream>
#include "../include/data_association.hpp"
#include "../include/LogReader.hpp"
#include "../include/line_detector.hpp"
#include "../include/state.hpp"
#include "../include/solver.hpp"

double th_min = 0.0;
double th_max = 2.0 * M_PI;
double r_min = 0.0;
double r_max = 5.0;
double r_step = .01;
double th_step = .01;
int vote_thresh = 120; //120;

int main(int argc, char** argv) {
  using namespace std;

  if (argc < 2) {
    cout << "Please pass the name of the log file" << endl;
    return 0;
  }

  // output log file for visualization
  ofstream vis_log;
  vis_log.open("./scripts/output.txt");
  vis_log << fixed; cout << fixed; // this makes it print all precision of doubles

  LogReader logReader(argv[1]);
  shared_ptr<Reading> r;
  Project::State state;

  Project::Pose origin;

  Project::LineDetector line_detector(th_min, th_max, r_min, r_max, vote_thresh,
                                      r_step, th_step);

  Project::Solver solver;

  int scan_count = 0;
  while ((r = logReader.getNext()) !=
         nullptr) {  // terminate when we have no more lines left
    if (r->type == 'O') {
      // odometry reading
      shared_ptr<Odometry> odom = static_pointer_cast<Odometry>(r);
      // cout<<"Processing odom at time " << odom->t << endl;

      state.add_odom(odom);

    } else if (r->type == 'S') {
      // scan reading
      state.integrate_odom();//changed log files to make this happen up front
      shared_ptr<Scan> scan = static_pointer_cast<Scan>(r);
      cout<<"Processing scan at time " << scan->t << endl;

      // detect lines
      vector<Project::Line> detected_lines =
        line_detector.detect_lines(scan, state.pose);

      // match lines with landmarks
      pair<vector<pair<Project::Line, int> >, vector<Project::Line> >
        matches = Project::associate_data(detected_lines, state.landmarks);

      // optimize with new lines
      solver.update(state.pose, matches.first, matches.second);
      vector<Project::Line> updated_landmarks = solver.get_landmark_values();
      //update the landmark estimates
      state.set_landmarks(updated_landmarks);
      // update the scan pose estimate
      state.pose = solver.get_last_pose();

      // prints the current reference frame of the scan
      vis_log << state.pose << endl;

      // print all landmarks
      vis_log << "Landmarks:" << state.landmarks.size() << endl;
      for (auto& l : state.landmarks) {
        Project::Line line_in_world = l.convert_coords(origin);
        vis_log << "  " << line_in_world << endl;
      }

      // print all detected lines
      vis_log << "Detected Lines:" << detected_lines.size() << endl;
      for (auto& l : detected_lines) {
        Project::Line line_in_world = l.convert_coords(origin);
        vis_log << "  " << line_in_world << endl;
      }
      if (scan_count++ >= 1000) break;
    }
  }  
  vis_log.close();
  //after all the estimation, print out a final estimate file in the same format
  //so we can visualize the final estimate
  std::ofstream final_log;
  std::vector<Project::Line> final_landmarks = solver.get_landmark_values();
  final_log.open("./scripts/final_output.txt");
  for(int i=0;i<solver.get_num_poses();i++){
    Project::Pose p = solver.get_pose(i);
    final_log<<p<<std::endl;
    final_log<<"Landmarks: "<<final_landmarks.size()<<std::endl;
    for (auto& l : final_landmarks) {
        Project::Line line_in_world = l.convert_coords(origin);
        final_log << "  " << line_in_world << std::endl;
      }
  }
  final_log.close();
}
