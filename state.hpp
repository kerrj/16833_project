#ifndef STATE_HPP
#define STATE_HPP

#include “Pose.hpp”
#include “Line.hpp”
namespace Project{
class State {
public:
	std::vector<Line> landmarks;
	Pose p;
	void update_landmarks(std::vector<Line> new_lines); // TODO: input argument will change to gtsam state
	void update_pose(Odometry o);
	std::vector<Line> landmarks();
std::vector<Line> pose();
};
}