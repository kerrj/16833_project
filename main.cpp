#include "LogReader.hpp"
#include <iostream>
#include "data_association.hpp"
#include "state.hpp"
#include "line_detector.hpp"
#include "data_association.hpp"

double th_min = 0.0;
double th_max = 2.0 * M_PI;
double r_min  = 0.0;
double r_max  = 10.0;
double r_step = .01;
double th_step= .01;
int vote_thresh = 50;

int main(int argc,char** argv){
	using namespace std;
	if(argc<2){cout<<"Please pass the name of the log file"<<endl;return 0;}
	LogReader logReader(argv[1]);
	shared_ptr<Reading> r;
	cout<<fixed;//this makes it print all precision of doubles
	Project::State state;

	Project::LineDetector line_detector(th_min, th_max, r_min, r_max, vote_thresh,r_step,th_step);
	while((r=logReader.getNext())!=nullptr){//terminate when we have no more lines left
		if(r->type=='O'){
			//odometry reading
			shared_ptr<Odometry> odom=static_pointer_cast<Odometry>(r);
			cout<<"Processing odom at time "<<odom->t<<endl;
			state.update_pose(odom);
		}else if(r->type=='S'){
			//scan reading
			shared_ptr<Scan> scan=static_pointer_cast<Scan>(r);
			cout<<"Processing scan at time "<<scan->t<<endl;
			std::vector<Project::Line> detected_lines = line_detector.detect_lines(scan, state.p);

			std::cout << "Num detected lines: " << detected_lines.size() << std::endl;
			for (auto& l: detected_lines) {
				std::cout << l.r << " " << l.th << std::endl;
				//std::cout << l.ref_frame.x << " " << l.ref_frame.y << " " << l.ref_frame.beta << std::endl;
			}
			std::pair<std::vector<std::pair<int, int> >, std::vector<Project::Line> > matches =
				Project::associate_data(detected_lines, state.landmarks);
			break;
		}
	}
}
