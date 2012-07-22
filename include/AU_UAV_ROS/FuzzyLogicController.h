#ifndef FUZZYLOGICCONTROLLER_H
#define FUZZYLOGICCONTROLLER_H

#include "ros/ros.h"
#include "fuzzylite/FuzzyLite.h"
#include <string>
#include <sstream>



namespace AU_UAV_ROS{
	class FuzzyLogicController{
        private:
        public:
            
        double FuzzyLogicOne(double in1, double in2);
        double FuzzyLogicTwo(double distance, double angle);
        double FLJeffOne(double distance, double angle);
        double tryturningintoplanes(double distance, double angle);
        double oursandtheirs(double distance, double ourAngle, double theirAngle);
        double sevenOutputs(double distance, double ourAngle, double theirAngle);
        double oursandtheirs2(double distance, double ourAngle, double theirAngle);
        double OTWOWBHtighter(double distance, double ourAngle, double theirAngle);
        double smallDistance(double ourAngle, double theirAngle);
        double smallDistance2(double ourAngle, double theirAngle);
        double lowerTurnsHighDists(double ourAngle, double theirAngle);
        double semiLowerTurnsHighDists(double ourAngle, double theirAngle);
        double copyCat(double distance, double ourAngle);
        double ZEM(double dist, double time);

	
//	FuzzyLogicTwo(void);

    };
};
#endif
