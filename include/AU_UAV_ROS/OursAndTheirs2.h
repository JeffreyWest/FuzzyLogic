#ifndef OURSANDTHEIRS2_H
#define OURSANDTHEIRS2_H

#include "ros/ros.h"
#include <cmath>
#include "fuzzylite/FuzzyLite.h"
#include <string>
#include <sstream>

namespace AU_UAV_ROS{
	class OursAndTheirs2{
    public:
       
	fl::FuzzyEngine* engine;
	fl::InputLVar* distanceBetweenPlanes;
	fl::InputLVar* ourBearingAngle;
	fl::InputLVar* theirBearingAngle;
	fl::OutputLVar* changeHeading;
	fl::RuleBlock* block;
        
  	public:
        //default constructor
        OursAndTheirs2(void);

        //PlanePose(int planePoseID, double planePoseX, double planePoseY, double planePoseZ, double planePoseHeading, double planePoseVelocity);

        double process(double distance, double ourAngle, double theirAngle);

};
}
#endif
