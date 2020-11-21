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
            //r_step is the distance between buckets along r axis
            LineDetector(double th_min_, double th_max_, double r_min_,
                         double r_max_, int vote_thresh_, double r_step_, double th_step_) {

                th_min = th_min_;
                th_max = th_max_;
                r_min  = r_min_;
                r_max  = r_max_;
                vote_thresh = vote_thresh_;

                th_range = th_max - th_min;
                r_range  = r_max - r_min;
                r_step=r_step_;
                th_step=th_step_;

                param_th_range = int(th_range / th_step);
                param_r_range  = int(r_range / r_step);
                // std::cout<<"size of space (th,r): "<<param_th_range<<" "<<param_r_range<<std::endl;
                for (int i = 0; i < param_r_range; i++) {
                    std::vector<int16_t> row(param_th_range, 0);
                    param_space.push_back(row);
                }
            }

            std::vector<std::vector<int16_t> > param_space;
            double th_min, th_max, r_min, r_max, th_range, r_range,r_step,th_step;
            int vote_thresh, param_th_range, param_r_range;

            std::vector<Line> max_votes(Pose p) {
                // vector of 2d vectors
                std::vector<Line> hough_lines;

                for (int u = 0; u < param_space.size(); ++u) {
                    for (int v = 0; v < param_space[0].size(); ++v) {
                        if (param_space[u][v] >= vote_thresh) {
                        	bool notmax=false;
                        	for (int r = -10; r <= 10; r++) {
		                        for (int c = -10; c <= 10; c++) {
		                            int uo = u + r;
		                            int vo = v + c;

		                            if (uo >= 0 && uo < param_space.size()
		                                    && vo >= 0 && vo < param_space[0].size()) {
		                                if (param_space[uo][vo] > param_space[u][v]) {
		                                    notmax=true;
		                                    break;
		                                }
		                            }
		                        }
		                        if(notmax)break;
		                    }
		                    if(notmax)continue;
                            Line new_line(p, ((double)u)*r_step, ((double)v)*th_step);
                            hough_lines.push_back(new_line);
                        }
                    }
                }

                return hough_lines;
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
                    for (int th_index = 0; th_index < param_space[0].size(); th_index++) {
                        double th = wrapAng(((double)th_index * th_step) - M_PI) + M_PI;
                        double rho = x*std::cos(th) + y*std::sin(th);
                        if (rho < 0) {
                           continue;
                        }
                        int rho_index = (int)(rho / r_step);
                        if(rho_index>=param_space.size())continue;
                        param_space[rho_index][th_index] += 1;
                    }

                }
                // get max votes
                std::vector<Line> hough_lines = max_votes(p);
                return hough_lines;
            }

        private:

    };
}

#endif
