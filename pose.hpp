#ifndef POSE_HPP
#define POSE_HPP
namespace Project{
class Pose{
public:
	Pose(double _x,double _y,double _beta):x(_x),y(_y),beta(_beta){};
	Pose(const Pose &p){
		x = p.x;
		y = p.y;
		beta = p.beta;
	}
	double x,y,beta;
};
}
#endif