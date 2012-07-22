#ifndef FUZZYLOGICONE_H
#define FUZZYLOGICONE_H

#include "ros/ros.h"
#include <cmath>
#include "fuzzylite/FuzzyLite.h"
#include <string>
#include <sstream>

namespace AU_UAV_ROS{
	class FuzzyLogicOne{
    public:
       
	fl::FuzzyEngine engine;
	fl::InputLVar* aMinusB;
	fl::InputLVar* distanceToCollision;
	fl::OutputLVar* collImminence;
	fl::RuleBlock* block;
        
  	public:
        //default constructor
        FuzzyLogicOne(void);

        //PlanePose(int planePoseID, double planePoseX, double planePoseY, double planePoseZ, double planePoseHeading, double planePoseVelocity);

        double process(double distToCollision, double oDist);

};
}
#endif
