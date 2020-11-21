#ifndef LOG_READER_HPP
#define LOG_READER_HPP
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <memory>
/*
simple class to iterate line by line through a text file and return the data
*/
class Reading{
public:
	double t;//timestamp
	char type;
};
class Odometry : public Reading{
public:
	Odometry(double x_,double y_,double th_){
		x=x_;
		y=y_;
		th=th_;
	}
	double x,y,th;
	//x,y,th in robot frame of the odometry (difference from previous pose)
};
class Scan : public Reading{
public:
	std::vector<double> xs;
	std::vector<double> ys;
};
class LogReader{
public:
	LogReader(std::string logName){
		fileStream.open(logName);
	}
	~LogReader(){
		fileStream.close();
	}
	/*
	Returns a shared_ptr to a reading struct, null if no lines left
	*/
	std::shared_ptr<Reading> getNext(){
		double t;
		if(!(fileStream >> t)){
			return nullptr;
		}
		char type;
		fileStream>>type;
		if(type=='S'){
			int len;
			fileStream>>len;
			std::shared_ptr<Scan> sPtr=std::make_shared<Scan>();
			sPtr->t=t;
			sPtr->type='S';
			for(int i=0;i<len;i++){
				double x,y;
				fileStream>>x;
				fileStream>>y;
				sPtr->xs.push_back(x);
				sPtr->ys.push_back(y);
			}
			return std::static_pointer_cast<Reading>(sPtr);
		}else if(type=='O'){
			double x,y,th;
			fileStream>>x;
			fileStream>>y;
			fileStream>>th;
			std::shared_ptr<Reading> oPtr=std::make_shared<Odometry>(x,y,th);
			oPtr->type='O';
			oPtr->t=t;
			return oPtr;
		}
		return nullptr;
	}
private:
	std::ifstream fileStream;
};
#endif