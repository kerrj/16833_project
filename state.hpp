#ifndef STATE_HPP
#define STATE_HPP

#include "pose.hpp"
#include "line.hpp"
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
	void update_pose(std::shared_ptr<Odometry> o){
		double RT=p.beta;
		double opt1=RT+o->th;
		double opt2=o->x/6.0;
		double opt3=o->th/2.0;
		double opt4=RT+opt3;
		double dx=opt2*(std::cos(RT)+4.0*std::cos(opt4)+std::cos(opt1));
		double dy=opt2*(std::sin(RT)+4.0*std::sin(opt4)+std::sin(opt1));
		p.x+=dx;
		p.y+=dy;
		p.beta=opt1;
	}
	std::vector<Line> landmarks;
	Pose p;
};
}
#endif
