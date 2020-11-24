#ifndef LINE_HPP
#define LINE_HPP
#include "pose.hpp"
#include "util.hpp"
#include <cmath>
namespace Project{
class Line{
public:
	Line(Pose _p, double _r, double _th):ref_frame(_p),r(_r),th(_th){};
	double distance(Line other){
		Line self_prime = convert_coords(other.ref_frame);
		double r_dist = std::pow(self_prime.r - other.r,2);
		double th_dist= std::pow(wrapAng(self_prime.th - other.th),2);
		return std::sqrt(r_dist + th_dist);
	}
	void print(bool line=false){
		std::cout<<"Line(";
		ref_frame.print();
		std::cout<<" r,th=("<<r<<","<<th<<")";
		if(line)std::cout<<std::endl;
	}
	Pose ref_frame;
	double r,th;
	/*
	Converts the coordinates of the line to the new frame given
	*/
	Line convert_coords(Pose new_frame){
		double th_prime = wrapAng(th + ref_frame.beta - new_frame.beta);
		double dr = (new_frame.x-ref_frame.x)*std::cos(ref_frame.beta + th) + 
					(new_frame.y-ref_frame.y)*std::sin(ref_frame.beta + th);
		double r_prime  = r  - dr;
		if(r_prime<0){
			//always make sure that the radius is positive
			r_prime = -r_prime;
			th_prime = wrapAng(th_prime + M_PI);
		}
		return Line(new_frame,r_prime,th_prime);
	}
};
}
#endif
