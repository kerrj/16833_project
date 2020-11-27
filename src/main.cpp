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
double r_max = 4.0;
double r_step = .01;
double th_step = .02;
int vote_thresh = 60; //120;

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
  // This is necessary to keep track of since the reference from of each scan
  // is in the frame of the end of the previous scan
  Project::Pose last_scan_pose;
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

      state.update_pose(odom);

    } else if (r->type == 'S') {
      // scan reading
      shared_ptr<Scan> scan = static_pointer_cast<Scan>(r);
      cout<<"Processing scan at time " << scan->t << endl;

      // detect lines
      vector<Project::Line> detected_lines =
        line_detector.detect_lines(scan, last_scan_pose);

      // match lines with landmarks
      pair<vector<pair<Project::Line, int> >, vector<Project::Line> >
        matches = Project::associate_data(detected_lines, state.landmarks);

      // optimize with new lines
      solver.update(matches.first, matches.second);
      vector<Project::Line> updated_landmarks = solver.get_landmark_values();

      state.set_landmarks(updated_landmarks);

      // prints the current reference frame of the scan
      vis_log << last_scan_pose << endl;

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

      last_scan_pose = state.pose;
      if (scan_count++ >= 300) break;
    }
  }
  vis_log.close();
}
