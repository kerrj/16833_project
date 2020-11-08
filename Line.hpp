#ifndef LINE_HPP
#define LINE_HPP
#include "pose.hpp"
namespace Project{
class Line{
public:
	Line(Pose _p, double _r, double _th):ref_frame(_p),r(_r),th(_th){};
	double distance(Line other){

	}
	Pose ref_frame;
	double r,th;
private:
	/*
	Converts the coordinates of the line to the new frame given
	*/
	Line convert_coords(Pose new_frame){
		
	}
};
}
#endif
