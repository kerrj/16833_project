#ifndef LINE_HPP
#define LINE_HPP
#include "pose.hpp"
#include <cmath>
namespace Project{
class Line{
public:
	Line(Pose _p, double _r, double _th):ref_frame(_p),r(_r),th(_th){};
	double distance(Line other){
		Line self_prime = convert_coords(other.ref_frame);
		double r_dist = pow(self_prime.r - other.r,2);
		double th_dist= pow(self_prime.th - other.th,2);
		return r_dist + th_dist;
	}
	Pose ref_frame;
	double r,th;
private:
	/*
	Converts the coordinates of the line to the new frame given
	*/
	Line convert_coords(Pose new_frame){
		double th_prime = th + ref_frame.beta - new_frame.beta;
		double dr = (new_frame.x-ref_frame.x)*std::cos(ref_frame.beta + th) + 
					(new_frame.y-ref_frame.y)*std::sin(ref_frame.beta + th);
		double r_prime  = r  - dr;
		return Line(new_frame,r_prime,th_prime);
	}
};
}
#endif
