/*
collisionAvoidance

This file receives telemetry updates from all UAVs and sends waypoints to these UAVs
 based on Fuzzy Logic Collision Avoidance Algorithm
*/

//standard C++ headers
#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>

//ROS, FL, Dubins headers
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"
#include "AU_UAV_ROS/standardFuncs.h"
#include "AU_UAV_ROS/FuzzyLogicController.h"
#include "AU_UAV_ROS/dubins.h"
#include "AU_UAV_ROS/FuzzyLogicOne.h"
#include "AU_UAV_ROS/OursAndTheirs2.h"
#include "AU_UAV_ROS/OTWOWBHtighter.h"

#define rho 15/(22.5*(M_PI/180.0))
#define TIMESTEP 1 
#define UAV_AIRSPEED 11.176

//collisionAvoidance does not have a header file, define several methods / variables here
AU_UAV_ROS::OursAndTheirs2* fuzzyLogic;
AU_UAV_ROS::OTWOWBHtighter* OTWOWBHtighter;

//ROS service clients for calling a service from the coordinator
ros::ServiceClient goToWaypointClient;
ros::ServiceClient requestInfoClient;

//initialize some cool variables
std::map<int,AU_UAV_ROS::PlanePose> planeMap;
AU_UAV_ROS::FuzzyLogicController fuzzy1;
std::ofstream myfile;
double distToWP;
double currentHeading = 0.0;
double fuzzyHeading = 0.0;
AU_UAV_ROS::fuzzyParams fuzzyParams;

//Initilize deez pointers
AU_UAV_ROS::PlanePose* currentUAV;
AU_UAV_ROS::PlanePose* oldUAV;
AU_UAV_ROS::PlanePose* closestUAV;

//this function is run everytime new telemetry information from any plane is recieved
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
//planeLatLongAlt -> current UAV's waypoint (Lat, Long, Alt)
//currentPose -> current UAV's position (X, Y, Z)
    	AU_UAV_ROS::waypoint planeLatLongAlt;
    	planeLatLongAlt.longitude = msg->currentLongitude;
    	planeLatLongAlt.latitude = msg->currentLatitude;
    	planeLatLongAlt.altitude = msg->currentAltitude;
        AU_UAV_ROS::position currentPose = getXYZ(planeLatLongAlt);
    
//If planeMap is empty, add current telemetry information
	if(planeMap.count(msg->planeID)==0)
    	{
            currentUAV = new AU_UAV_ROS::PlanePose;
            currentUAV->setID(msg->planeID);
            currentUAV->setX(currentPose.x_coordinate);
            currentUAV->setY(currentPose.y_coordinate);
            currentUAV->setZ(currentPose.altitude);
            currentUAV->setVelocity(11.176);
            currentUAV->setHeading(0.0);
            planeMap[msg->planeID] = *currentUAV;
            delete currentUAV;
    	}
    
//After initial planeMap creation, don't update planeMap until planes have been sent a new Waypoint
    else if (planeMap.count(msg->planeID)!=0 && msg->currentWaypointIndex == -1)
    {
        /* Don't do anything here!
           This function is super important, because the UAVs should 
           not do anything until waypoints are received */
    }    
    else
    {
/*----------------------------------------------------------------------------------*/
//After initial map update, we can now update things like heading during second time through
        currentUAV = new AU_UAV_ROS::PlanePose;
        currentUAV->setID(msg->planeID);
        currentUAV->setX(currentPose.x_coordinate);
        currentUAV->setY(currentPose.y_coordinate);
        currentUAV->setZ(currentPose.altitude);
        oldUAV = &planeMap.find(msg->planeID)->second;
        double currentHeading = getNewHeading(oldUAV->getPosition(), currentUAV->getPosition());
        currentUAV->setHeading(currentHeading);
        
/*----------------------------------------------------------------------------------*/
//Find the Closest UAV and the distance between the two
        int closestPlane = getClosestPlane(msg->planeID, planeMap);
        closestUAV = &planeMap.find(closestPlane)->second;
        double distBtwnPlanes = getDist(currentUAV->getPosition(), closestUAV->getPosition());
/*----------------------------------------------------------------------------------*/
        AU_UAV_ROS::waypoint nextWaypoint;
        //ENTER Collision Avoidance Mode:
        if ((closestPlane != -1) && (distBtwnPlanes < 100)) {
            //grab necessary parameters for the Fuzzy Logic:
            fuzzyParams = getFuzzyParams(currentUAV, closestUAV);
            //Process Fuzzy Logic to return a change in heading (fuzzyHeading)
            fuzzyHeading = fuzzyLogic->process(distBtwnPlanes, fuzzyParams.ourBearingAngle, fuzzyParams.theirBearingAngle);
            //add the change in heading to the actual heading (for real, guys)
            double nextHeadingForReal = fuzzyHeading + currentUAV->getHeading();
            //use this heading and find a waypoint (lat, long, alt) to send
            nextWaypoint = getCAWaypoint(nextHeadingForReal, currentPose);
        }
        //Did not enter Collision Avoidance Mode:
        else {
            //Find the next goal waypoint
            AU_UAV_ROS::waypoint nextGoal;
            AU_UAV_ROS::RequestWaypointInfo srv;
            srv.request.planeID=msg->planeID;
            srv.request.isAvoidanceWaypoint = false;
            srv.request.positionInQueue = 0;
            if (requestInfoClient.call(srv)){
                //ROS_INFO("clean");
            }
            else{
                ROS_INFO("error");
            }
            nextGoal.latitude = srv.response.latitude;
            nextGoal.longitude = srv.response.longitude;
            nextGoal.altitude = srv.response.altitude;
            //Go to the next goal waypoint
            nextWaypoint = nextGoal;            
        }
/*----------------------------------------------------------------------------------*/
        //go to nextWaypoint everytime
        //service request to go to the waypoint determined by fuzzy logicness OR normal waypoint
        AU_UAV_ROS::GoToWaypoint gotosrv;
        gotosrv.request.planeID = msg->planeID;
        gotosrv.request.latitude = nextWaypoint.latitude;
        gotosrv.request.longitude = nextWaypoint.longitude;
        gotosrv.request.isAvoidanceManeuver = true;
        gotosrv.request.isNewQueue = true;
        if(goToWaypointClient.call(gotosrv)){
            //ROS_INFO("clean");
        }
        else{
            ROS_ERROR("error");
        }
        //update map with current Plane Pose, Heading (currentUAV) everytime
        planeMap[msg->planeID] = *currentUAV; 

        delete currentUAV;
    }
}

int main(int argc, char **argv)
{
	//standard ROS startup
	ros::init(argc, argv, "collisionAvoidance");
	ros::NodeHandle n;
	
	//subscribe to telemetry outputs and create client for the avoid collision service
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	goToWaypointClient = n.serviceClient<AU_UAV_ROS::GoToWaypoint>("go_to_waypoint");
    requestInfoClient = n.serviceClient<AU_UAV_ROS::RequestWaypointInfo>("request_waypoint_info");
	
	//create Fuzzy Logic Controllers for use in telemetryCallback
    fuzzyLogic = new AU_UAV_ROS::OursAndTheirs2();

	//random seed for if statement in telemetryCallback, remove when collision avoidance work begins
	srand(time(NULL));

	//needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}
