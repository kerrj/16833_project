#include "LogReader.hpp"
#include <iostream>
#include "state.hpp"
int main(int argc,char** argv){
	using namespace std;
	if(argc<2){cout<<"Please pass the name of the log file"<<endl;return 0;}
	LogReader logReader(argv[1]);
	shared_ptr<Reading> r;
	cout<<fixed;//this makes it print all precision of doubles
	Project::State state;
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
		}
	}
}