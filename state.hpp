#ifndef STATE_HPP
#define STATE_HPP

#include "Pose.hpp"
#include "Line.hpp"
#include "LogReader.hpp"
#include <memory>
#include <cmath>
#include "util.hpp"
namespace Project{
class State {
public:
	void update_landmarks(std::vector<Line> new_lines){
		// TODO: input argument will change to gtsam state
		//inserts the lines into the landmark vector
		landmarks.insert(landmarks.end(),new_lines.begin(),new_lines.end());
	}
	void update_pose(shared_ptr<Odometry> o){
		double half_th = 
	}
	std::vector<Line> landmarks(){return landmarks;}
	std::vector<Line> pose(){return p;}
	std::vector<Line> landmarks;
	Pose p;
};
}