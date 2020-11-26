#include <iostream>
#include "data_association.hpp"
#include "LogReader.hpp"
#include "line_detector.hpp"
#include "state.hpp"
#include "solver.hpp"

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
  LogReader logReader(argv[1]);
  shared_ptr<Reading> r;
  cout << fixed;  // this makes it print all precision of doubles
  Project::State state;
  // This is necessary to keep track of since the reference from of each scan
  // is in the frame of the end of the previous scan
  Project::Pose lastScanPose;

  Project::LineDetector line_detector(th_min, th_max, r_min, r_max, vote_thresh,
                                      r_step, th_step);
  int scan_count = 0;
  while ((r = logReader.getNext()) !=
         nullptr) {  // terminate when we have no more lines left
    if (r->type == 'O') {
      // odometry reading
      shared_ptr<Odometry> odom = static_pointer_cast<Odometry>(r);
      // cout<<"Processing odom at time "<<odom->t<<endl;
      state.update_pose(odom);
    } else if (r->type == 'S') {
      // scan reading
      shared_ptr<Scan> scan = static_pointer_cast<Scan>(r);
      // cout<<"Processing scan at time "<<scan->t<<endl;
      std::vector<Project::Line> detected_lines =
          line_detector.detect_lines(scan, lastScanPose);

      // std::cout << "Detected lines: \n";
      // for (auto& l: detected_lines) {
      // 	std::cout<<"  ";
      // 	l.print(true);
      // }
      std::pair<std::vector<std::pair<Project::Line, int> >, std::vector<Project::Line> >
          matches = Project::associate_data(detected_lines, state.landmarks);
      state.update_landmarks(matches.second);
      // std::cout<<"Found matches:\n";
      // for(int i=0;i<matches.first.size();i++){
      // 	std::pair<int,int> corresp=(matches.first)[i];
      // 	std::cout<<"  detected id -> landmark id:
      // "<<corresp.first<<"->"<<corresp.second<<std::endl;
      // }
      lastScanPose.print(
          true);  // prints the current reference frame of the scan
      std::cout << "Landmarks:" << state.landmarks.size() << std::endl;
      // std::cout<<"Landmarks:"<<detected_lines.size()<<std::endl;
      Project::Pose origin;
      for (auto& l : state.landmarks) {
        // for(auto& l: detected_lines){
        std::cout << "  ";
        Project::Line line_in_world = l.convert_coords(origin);
        line_in_world.print(true);
      }
      lastScanPose = state.p;
      if(scan_count++>=300)break;
    }
  }
}
