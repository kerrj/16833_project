#ifndef LINE_DETECTOR_HPP
#define LINE_DETECTOR_HPP

#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include "Line.hpp"
#include "LogReader.hpp"
#include "pose.hpp"
#include "util.hpp"

namespace Project {

class LineDetector {
  // parameter space in meters x radians
 public:

  std::vector<std::vector<int16_t> > param_space;
  double th_min, th_max, r_min, r_max, th_range, r_range, r_step, th_step;
  int vote_thresh, param_th_range, param_r_range;

  // define th_min_ and th_max_ in radians
  // r_step is the distance between buckets along r axis
  LineDetector(double th_min_, double th_max_, double r_min_, double r_max_,
               int vote_thresh_, double r_step_, double th_step_) {
    th_min = th_min_;
    th_max = th_max_;
    r_min = r_min_;
    r_max = r_max_;
    vote_thresh = vote_thresh_;

    th_range = th_max - th_min;
    r_range  = r_max - r_min;
    r_step   = r_step_;
    th_step  = th_step_;

    param_th_range = int(th_range / th_step);
    param_r_range = int(r_range / r_step);
    // std::cout<<"size of space (th,r): "<<param_th_range<<"
    // "<<param_r_range<<std::endl;
    for (int i = 0; i < param_r_range; i++) {
      std::vector<int16_t> row(param_th_range, 0);
      param_space.push_back(row);
    }
  }

  std::vector<Line> max_votes(Pose p,std::vector<int> &votes) {
    // vector of 2d vectors
    std::vector<Line> hough_lines;

    for (int u = 0; u < param_space.size(); ++u) {
      for (int v = 0; v < param_space[0].size(); ++v) {
        if (param_space[u][v] >= vote_thresh) {
          bool notmax = false;
          for (int r = -5; r <= 5; r++) {
            for (int c = -5; c <= 5; c++) {
              int uo = u + r;
              int vo = v + c;

              if (uo >= 0 && uo < param_space.size() && vo >= 0 &&
                  vo < param_space[0].size()) {
                if (param_space[uo][vo] > param_space[u][v]) {
                  notmax = true;
                  break;
                }
              }
            }
            if (notmax) break;
          }
          if (notmax) continue;
          Line new_line(p, ((double)u) * r_step, ((double)v) * th_step);
          hough_lines.push_back(new_line);
          votes.push_back(param_space[u][v]);
        }
      }
    }

    return hough_lines;
  }
  void downsample_scan(std::shared_ptr<Scan> scan, std::vector<double> &xs, std::vector<double> &ys){
    xs.push_back(scan->xs[0]);
    ys.push_back(scan->ys[0]);
    // Downsample the points to a certain max density
    const double min_distance = .025;
    // Iterate through the scan radially, reject
    for(int i=1;i<scan->xs.size();i++){
      double lastx = xs[xs.size()-1];
      double lasty = ys[ys.size()-1];
      double dist = std::hypot(lastx - scan->xs[i],lasty - scan->ys[i]);
      if(dist<min_distance)continue;
      xs.push_back(scan->xs[i]);
      ys.push_back(scan->ys[i]);
    }
  }
  std::vector<Line> detect_lines(std::shared_ptr<Scan> scan, Pose p) {
    // clear param_space
    for (auto& r : param_space) {
      std::fill(r.begin(), r.end(), 0);
    }
    std::vector<double> xs;
    std::vector<double> ys;
    downsample_scan(scan,xs,ys);
    std::cout<<"Original size: "<<scan->xs.size()<<" downsampled size: "<<xs.size()<<std::endl;
    int num_pts = xs.size();
    // populate parameter space with votes
    for (int i = 0; i < num_pts; ++i) {
      double x = xs[i];
      double y = ys[i];
      for (int th_index = 0; th_index < param_space[0].size(); th_index++) {
        double th = wrapAng(((double)th_index * th_step) - M_PI) + M_PI;
        double rho = x * std::cos(th) + y * std::sin(th);
        if (rho < 0) {
          continue;
        }
        int rho_index = (int)(rho / r_step);
        if (rho_index >= param_space.size()) continue;
        param_space[rho_index][th_index] += 1;
      }
    }
    // get max votes
    std::vector<int> votes;
    std::vector<Line> hough_lines = max_votes(p,votes);
    // prune things out with NMS
    const double reject_dist=1;
    for(int i = hough_lines.size()-1;i >= 0;i--){
      for(int j = i-1;j >= 0;j--){
        double dist = hough_lines[i].distance(hough_lines[j]);
        if(dist<reject_dist && votes[i]<=votes[j]){
          hough_lines.erase(hough_lines.begin()+i);
          votes.erase(votes.begin()+i);
          break;
        }
      }
    }
    return hough_lines;
  }
};

// TODO: resolution for radius
// based on Robust Detection of Lines Using the Progressive Probabilistic Hough Transform
class LineDetectorP {
  // parameter space in meters x degrees
 public:

  std::vector<std::vector<int16_t> > param_space;
  double th_min, th_max, r_min, r_max, th_range, r_range, r_step, th_step, distance_thresh;
  int vote_thresh, param_th_range, param_r_range;

  // define th_min_ and th_max_ in radians
  // r_step is the distance between buckets along r axis
  LineDetectorP(double th_min_, double th_max_, double r_min_, double r_max_,
                int vote_thresh_, double distance_thresh_,
                double r_step_, double th_step_) {
    th_min = th_min_;
    th_max = th_max_;
    r_min = r_min_;
    r_max = r_max_;
    vote_thresh = vote_thresh_;
    distance_thresh = distance_thresh_;

    th_range = th_max - th_min;
    r_range  = r_max - r_min;
    r_step   = r_step_;
    th_step  = th_step_;

    param_th_range = int(th_range / th_step);
    param_r_range = int(r_range / r_step);

    for (int i = 0; i < param_r_range; i++) {
      std::vector<int16_t> row(param_th_range, 0);
      param_space.push_back(row);
    }

    /* initialize random seed: */
    srand(0);
    // srand(time(NULL));
  }

private:

  bool vote(double x, double y, Pose &p, Line &detected_line, int increment = 1) {
    bool detected = false;
    for (int th_index = 0; th_index < param_space[0].size(); th_index++) {
      double th = wrapAng(((double)th_index * th_step) - M_PI) + M_PI;
      double rho = x * std::cos(th) + y * std::sin(th);

      // verify valid rho
      if (rho < 0) continue;
      int rho_index = (int)(rho / r_step);
      if (rho_index >= param_space.size()) continue;

      param_space[rho_index][th_index] += increment;
      if (!detected && increment == 1 && param_space[rho_index][th_index] >= vote_thresh) { 
        detected_line = Line(p, rho, th);
        detected = true;
      }
    }

    return detected;
  }

  void unvote(double x, double y) {
    Pose p;
    Line detected_line;
    vote(x, y, p, detected_line, -1);
  }

public:

  // NOTE: this destroys the scan argument
  std::vector<Line> detect_lines(std::shared_ptr<Scan> scan, Pose p) {
    // clear param_space
    for (auto& r : param_space) {
      std::fill(r.begin(), r.end(), 0);
    }

    std::vector<Line> detected_lines;
    int num_pts_in_param_space = 0;

    for (int num_pts = scan->xs.size(); num_pts > 0; ) {
      // choose a random point
      int i = rand() % num_pts;
      double x = scan->xs[i];
      double y = scan->ys[i];

      if (x == 0 && y == 0) {
        scan->xs[i] = scan->xs[num_pts-1];
        scan->ys[i] = scan->ys[num_pts-1];
        scan->xs[num_pts-1] = scan->xs[num_pts+num_pts_in_param_space-1];
        scan->ys[num_pts-1] = scan->ys[num_pts+num_pts_in_param_space-1];
        num_pts--;
        continue;
      }

      // move it to the "voted" region of scan
      double temp_x = scan->xs[i];
      double temp_y = scan->ys[i];
      scan->xs[i] = scan->xs[num_pts-1];
      scan->ys[i] = scan->ys[num_pts-1];
      scan->xs[num_pts-1] = temp_x;
      scan->ys[num_pts-1] = temp_y;
      num_pts_in_param_space++;
      num_pts--;

      Line detected_line;
      if (vote(x, y, p, detected_line)) {
        detected_lines.push_back(detected_line);

        int num_close = 0;
        // iterate through "voted" region of scan
        for (int j = num_pts; j < num_pts+num_pts_in_param_space; j++) {
          double x1 = scan->xs[j];
          double y1 = scan->ys[j];

          // unvote if close to detected line
          if (detected_line.distance_to_point(x1, y1) <= distance_thresh) {
            num_close++;
            unvote(x1, y1);
            // "remove" voted point by overriding it with last element in "voted" region of scan
            scan->xs[j] = scan->xs[num_pts+num_pts_in_param_space-1];
            scan->ys[j] = scan->ys[num_pts+num_pts_in_param_space-1];
            num_pts_in_param_space--;
            j--;
          }
        }
        // iterate through "unvoted" region of scan
        for (int j = 0; j < num_pts; j++) {
          double x1 = scan->xs[j];
          double y1 = scan->ys[j];

          // mark as done if close to detected line
          if (detected_line.distance_to_point(x1, y1) <= distance_thresh) {
            num_close++;
            // mark as done
            scan->xs[j] = 0;
            scan->ys[j] = 0;
          }
        }
      }
    }

    return detected_lines;
  }
};

}  // namespace Project

#endif
