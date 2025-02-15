#include <iostream>
#include <fstream>
#include <chrono>
#include "../include/data_association.hpp"
#include "../include/LogReader.hpp"
#include "../include/line_detector.hpp"
#include "../include/state.hpp"
#include "../include/solver.hpp"
#include <algorithm>
double th_min = 0.0;
double th_max = 2.0 * M_PI;
double r_min = 0.0;
double r_max = 5.0;
double r_step = .01;
double th_step = .01;
int vote_thresh = 32; 
bool sortbysecond(std::pair<Project::Line, int> a,std::pair<Project::Line, int> b){
  return a.second<b.second;
}
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
  // Project::LineDetectorP line_detector(th_min, th_max, r_min, r_max, 60, 0.1, r_step, th_step);

  Project::Solver solver;

  std::chrono::time_point<std::chrono::high_resolution_clock> start;
  std::chrono::time_point<std::chrono::high_resolution_clock> beginning = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> line_detection_time;
  std::chrono::duration<double> logging_time;
  std::chrono::duration<double> data_association_time;
  std::chrono::duration<double> solver_time;

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
      start = std::chrono::high_resolution_clock::now();
      vector<Project::Line> detected_lines =
        line_detector.detect_lines(scan, state.pose);
      line_detection_time += (std::chrono::high_resolution_clock::now() - start);

      // match lines with landmarks
      start = std::chrono::high_resolution_clock::now();
      //get the high conf landmarks and low conf landmarks
      std::vector<Project::Line> high_landmarks = solver.get_landmark_values(true);
      std::vector<Project::Line> low_landmarks  = solver.get_landmark_values(false,true);
      //find matches with high conf landmarks
      pair<vector<pair<Project::Line, int> >, vector<Project::Line> >
                        high_matches = Project::associate_data(detected_lines, high_landmarks);
      //find matches with low conf landmarks with remaining lines if any
      pair<vector<pair<Project::Line, int> >, vector<Project::Line> >
                        low_matches  = Project::associate_data(high_matches.second, low_landmarks);
      //new lines are ones that weren't matched in either round
      std::vector<Project::Line> new_lines = low_matches.second;
      //find the new associations, being careful to preserve index into original landmarks
      std::vector<pair<Project::Line, int> > new_assocs;
      //we can essentially merge the high and low associations
      int high_id=0;
      int low_id=0;
      std::vector<pair<Project::Line, int> > &high_assocs=high_matches.first;
      std::vector<pair<Project::Line, int> > &low_assocs=low_matches.first;
      //sort both of these by their index
      std::sort(high_assocs.begin(),high_assocs.end(),sortbysecond);
      std::sort(low_assocs.begin(),low_assocs.end(),sortbysecond);
      std::vector<Project::Line> all_landmarks = state.landmarks;
      for(int i=0;i<all_landmarks.size();i++){
        Project::Line landmark=all_landmarks[i];
        bool foundhigh=false;
        if(high_id<high_assocs.size() && 
            high_landmarks[high_assocs[high_id].second].distance(landmark)<.002){
          foundhigh=true;
          new_assocs.emplace_back(high_assocs[high_id++].first,i);
        }
        if(low_id<low_assocs.size() && 
            low_landmarks[low_assocs[low_id].second].distance(landmark)<.002){
          if(foundhigh)throw std::runtime_error("matched both high and low to same landmark\n");
          new_assocs.emplace_back(low_assocs[low_id++].first,i);
        }
      }
      if(new_assocs.size()!=high_assocs.size()+low_assocs.size()){
        throw std::runtime_error("sizes dont match\\n");
      }
      pair<vector<pair<Project::Line, int> >, vector<Project::Line> >
              matches = make_pair(new_assocs,new_lines);
      //below 2 lines is old technique
      // pair<vector<pair<Project::Line, int> >, vector<Project::Line> >
      //                   matches = Project::associate_data(detected_lines, state.landmarks);
      data_association_time += (std::chrono::high_resolution_clock::now() - start);

      // optimize with new lines
      start = std::chrono::high_resolution_clock::now();
      solver.update(state.pose, matches.first, matches.second);
      solver_time += (std::chrono::high_resolution_clock::now() - start);

      //update the landmark estimates
      vector<Project::Line> updated_landmarks = solver.get_landmark_values();
      state.set_landmarks(updated_landmarks);

      // update the scan pose estimate
      state.pose = solver.get_last_pose();

      start = std::chrono::high_resolution_clock::now();
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
      logging_time += (std::chrono::high_resolution_clock::now() - start);

      if (scan_count++ >= 1000) break;
    }
  }  
  vis_log.close();
  //after all the estimation, print out a final estimate file in the same format
  //so we can visualize the final estimate
  start = std::chrono::high_resolution_clock::now();
  std::ofstream final_log;
  std::vector<Project::Line> final_landmarks = solver.get_landmark_values(true);
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
  logging_time += (std::chrono::high_resolution_clock::now() - start);
  std::cout << "total line detection time: " << line_detection_time.count() << std::endl;
  std::cout << "total solver time: " << solver_time.count() << std::endl;
  std::cout << "total visualization logging time: " << logging_time.count() << std::endl;
  std::cout << "total data association time: " << data_association_time.count() << std::endl;
  std::chrono::duration<double> total_time = (std::chrono::high_resolution_clock::now() - beginning);
  std::cout << "total time: " << total_time.count() << std::endl;
}
