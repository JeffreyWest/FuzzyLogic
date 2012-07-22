#ifndef OTWOWBHTIGHTER_H
#define OTWOWBHTIGHTER_H

#include "ros/ros.h"
#include <cmath>
#include "fuzzylite/FuzzyLite.h"
#include <string>
#include <sstream>

namespace AU_UAV_ROS{
	class OTWOWBHtighter{
    public:
       
	fl::FuzzyEngine* engine;
	fl::InputLVar* distanceBetweenPlanes;
	fl::InputLVar* ourBearingAngle;
	fl::InputLVar* theirBearingAngle;
	fl::OutputLVar* changeHeading;
	fl::RuleBlock* block;
        
  	public:
        //default constructor
        OTWOWBHtighter(void);

        double process(double distance, double ourAngle, double theirAngle);

};
}
#endif
