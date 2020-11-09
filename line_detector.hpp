#ifndef LINE_DETECTOR_HPP
#define LINE_DETECTOR_HPP

#include "Line.hpp"
#include "pose.hpp"
#include "LogReader.hpp"
#include "util.hpp"
#include <memory>
#include <vector>
#include <assert.h>
#include <iostream>

namespace Project{

    // TODO: resolution for radius
    class LineDetector {
        // parameter space in meters x degrees
        public:
            // define th_min_ and th_max_ in radians
            // 0 -> 2*pi radians, 0 -> 800 cm
            LineDetector(double th_min_, double th_max_, double r_min_,
                         double r_max_, int vote_thresh_) {

                th_min = th_min_;
                th_max = th_max_;
                r_min  = r_min_;
                r_max  = r_max_;
                vote_thresh = vote_thresh_;

                th_range = th_max - th_min;
                r_range  = r_max - r_min;

                param_th_range = int(th_range * 180.0 / M_PI);
                param_r_range  = int(r_range * 100);

                for (int i = 0; i < param_r_range; i++) {
                    std::vector<int> row(param_th_range, 0);
                    param_space.push_back(row);
                }
            }

            std::vector<std::vector<int>> param_space;
            double th_min, th_max, r_min, r_max, th_range, r_range;
            int vote_thresh, param_th_range, param_r_range;

            std::vector<Line> max_votes(Pose p) {
                // vector of 2d vectors
                std::vector<Line> hough_lines;

                for (int u = 0; u < param_space.size(); ++u) {
                    for (int v = 0; v < param_space[0].size(); ++v) {
                        if (param_space[u][v] >= vote_thresh) {
                            Line new_line(p, u, v);
                            hough_lines.push_back(new_line);
                        }
                    }
                }

                return hough_lines;
            }

            void non_max_suppress() {
                for (int u = 0; u < param_space.size(); ++u) {
                    for (int v = 0; v < param_space[0].size(); ++v) {
                        for (int r = -1; r <= 1; r++) {
                            for (int c = -1; c <= 1; c++) {
                                int uo = u + r;
                                int vo = v + c;

                                if (uo >= 0 && uo < param_space.size()
                                        && vo >= 0 && vo < param_space[0].size()) {

                                    if (param_space[uo][vo] > param_space[u][v]) {
                                        param_space[u][v] = 0;
                                        break;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            std::vector<Line> detect_lines(std::shared_ptr<Scan> scan, Pose p) {
                // clear param_space
                for (auto& r: param_space) {
                    std::fill(r.begin(), r.end(), 0);
                }

                int num_pts = scan->xs.size();

                // populate parameter space with votes
                for (int i = 0; i < num_pts; ++i) {
                    double x = scan->xs[i];
                    double y = scan->ys[i];

                    int th_min_deg = th_min * 180 / M_PI;
                    int th_max_deg = th_max * 180 / M_PI;

                    for (int th_index = th_min_deg; th_index < th_max_deg; th_index++) {
                        double th = wrapAng(((double)th_index * M_PI / 180.0) - M_PI) + M_PI;
                        //std::cout << th << std::endl;
                        double rho = x*std::cos(th) + y*std::sin(th);
                        if (rho < 0) {
                           continue;
                        }
                        int rho_index = int(rho * 100);
                        param_space[rho_index][th_index] += 1;
                    }

                }
                non_max_suppress();

                // get max votes
                std::vector<Line> hough_lines = max_votes(p);
                return hough_lines;
            }

        private:

    };
}

#endif
