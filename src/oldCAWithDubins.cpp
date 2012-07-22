/*
 collisionAvoidance
 This is where students will be able to program in a collision avoidance algorithm.  The telemetry callback
 is already setup along with a dummy version of how the service request would work.
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

#define rho 15/(22.5*(M_PI/180.0)) //TODO-check if units are correct, this is currently in meters/radian
#define TIMESTEP 1 
#define UAV_AIRSPEED 11.176

//collisionAvoidance does not have a header file, define methods here
bool firstFuzzyEngine(double distanceToCollision, double overlapDistance, double distToWP, double distBtwnPlanes);
double secondFuzzyEngine(double distBtwnPlanes, double ourBearingAngle, double theirBearingAngle);
void isNewWaypoint(AU_UAV_ROS::PlanePose* myUAV, int myPlaneID);

//ROS service client for calling a service from the coordinator
ros::ServiceClient goToWaypointClient;
ros::ServiceClient requestInfoClient;

//initialize some cool variables
std::map<int,AU_UAV_ROS::PlanePose> planeMap;
AU_UAV_ROS::FuzzyLogicController fuzzy1;
std::ofstream myfile;
//double distBtwnPlanes = -1;
double distToWP;
double currentHeading = 0.0;
int counter = 0;
AU_UAV_ROS::position nextGoalXY;
AU_UAV_ROS::fuzzyParams fuzzyParams;
//Initilize deez pointers
AU_UAV_ROS::PlanePose* currentUAV;
AU_UAV_ROS::PlanePose* oldUAV;
AU_UAV_ROS::PlanePose* closestUAV;

/* this method returns a dubins path for a UAV that would take it to the next waypoint
 should be called whenever a path needs to be calculated */
DubinsPath* setupDubins(AU_UAV_ROS::PlanePose* myUAV,int myPlaneID) {
    DubinsPath* myDubinsPath = new DubinsPath;
    //starting point
    double q0[3];
    //ending point
    double q1[3];
    
    q0[0]=myUAV->getX();
    q0[1]=myUAV->getY();
    q0[2]=90.0 - myUAV->getHeading();
    q0[2] = q0[2]*DEGREES_TO_RADIANS;
    AU_UAV_ROS::position waypoints[2];
    
    isNewWaypoint(myUAV, myPlaneID);
    
    //grab x, y, z positions of next two waypoints
    waypoints[0] = getXYZ(myUAV->goalWaypoints[0]);
    waypoints[1] = getXYZ(myUAV->goalWaypoints[1]);
    ROS_INFO("Waypoints0 is %f", waypoints[0].x_coordinate);
    ROS_INFO("Waypoints1 is %f", waypoints[1].x_coordinate);
    
    //set x and y to the next waypoint as the endpoint of this dubins path
    q1[0]=waypoints[0].x_coordinate;
    q1[1]=waypoints[0].y_coordinate;
    
    //get bearing for the plane to be at when it reaches the end of path
    double deltaX = waypoints[1].x_coordinate - waypoints[0].x_coordinate;
    double deltaY = waypoints[1].y_coordinate - waypoints[0].y_coordinate;
    
    q1[2]=atan2(deltaY,deltaX); //in radians
    //create path
    //ROS_INFO("q0[0]=%f  q0[1]=%f  q0[2]=%f",q0[0],q0[1],q0[2]);
    //ROS_INFO("q1[0]=%f  q1[1]=%f  q1[2]=%f",q1[0],q1[1],q1[2]);
    dubins_init(q0,q1,rho,myDubinsPath);
    return myDubinsPath;
}

void isNewWaypoint(AU_UAV_ROS::PlanePose* myUAV, int myPlaneID){
    int number = myUAV->goalWaypoints.size() - 1;
    
    AU_UAV_ROS::RequestWaypointInfo srv;
    srv.request.planeID=myPlaneID;
    srv.request.isAvoidanceWaypoint = false;
    srv.request.positionInQueue = number;
    requestInfoClient.call(srv);
    
    if (srv.response.latitude < 0)
    {
        //bump here!
        myUAV->goalWaypoints.erase(myUAV->goalWaypoints.begin());
    }
}

//this function is run everytime new telemetry information from any plane is recieved
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
    //add one to the counter
    if (msg->planeID == 0) {
        counter++;
    }
    
    //planeLatLongAlt -> current UAV's waypoint (Lat, Long, Alt)
    //currentPose -> current UAV's position (XYZ)
    AU_UAV_ROS::waypoint planeLatLongAlt;
    planeLatLongAlt.longitude = msg->currentLongitude;
    planeLatLongAlt.latitude = msg->currentLatitude;
    planeLatLongAlt.altitude = msg->currentAltitude;
    AU_UAV_ROS::position currentPose = getXYZ(planeLatLongAlt);
    
    //If planeMap is empty, add current information
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
        //ROS_INFO("Plane Map is Empty");
    }
    
    //After initial planeMap creation, don't update planeMap until planes have been sent a new Waypoint
    else if (planeMap.count(msg->planeID)!=0 && msg->currentWaypointIndex == -1)
    {
        //Don't do anything here!
    }    
    //After initial update, we can now update things like newHeading during second time through
    else
    {
        currentUAV = new AU_UAV_ROS::PlanePose;
        currentUAV->setID(msg->planeID);
        currentUAV->setX(currentPose.x_coordinate);
        currentUAV->setY(currentPose.y_coordinate);
        currentUAV->setZ(currentPose.altitude);
        currentUAV->setVelocity(11.176);
        //update heading
        oldUAV = &planeMap.find(msg->planeID)->second;
        double currentHeading = getNewHeading(oldUAV->getPosition(), currentUAV->getPosition());
        currentUAV->setHeading(currentHeading);
        
        AU_UAV_ROS::waypoint nextGoal;
        AU_UAV_ROS::RequestWaypointInfo srv;
        if (oldUAV->goalWaypoints.empty()) {
            //Grab all of this plane's waypoints and store
            int i = 0;
            srv.request.planeID=msg->planeID;
            srv.request.isAvoidanceWaypoint = false;
            srv.request.positionInQueue = 0;
            if (requestInfoClient.call(srv)) {
                //ROS_INFO("good to go");
            }
            else{
                //ROS_INFO("oh noes");
            }
            while (srv.response.latitude > 0) {
                nextGoal.latitude = srv.response.latitude;
                nextGoal.longitude = srv.response.longitude;
                nextGoal.altitude = srv.response.altitude;
                currentUAV->goalWaypoints.push_back(nextGoal);
                i++;
                srv.request.planeID=msg->planeID;
                srv.request.isAvoidanceWaypoint = false;
                srv.request.positionInQueue = i;
                if (requestInfoClient.call(srv)) {
                    //ROS_INFO("clean");
                }
                else {
                    //ROS_INFO("error");
                }
            }
        }
        else
        {
            currentUAV->goalWaypoints = oldUAV->goalWaypoints;
        }
        /*----------------------------------------------------------------------------------*/
        //This section stores a waypoint and position struct of where the current Plane would
        //travel if collision avoidance was NOT initiated (dubins path)
        AU_UAV_ROS::waypoint nextWaypoint;
        //check if dubins Vector is empty
        if (oldUAV->dubinsPoints.empty()) //currentUAV->dubinsPoints.empty())
        {
            //create a new dubins path that goes from current position to next waypoint
            DubinsPath* newDubinspath = setupDubins(currentUAV, msg->planeID);
            int i=0;
            double q[3];
            int step = 1;
            while (i==0)
            {
                //step along dubins path and create waypoints at these points
                i = dubins_path_sample(newDubinspath,(UAV_AIRSPEED+.5)*step,q);
                AU_UAV_ROS::position newDubinsPoint;
                newDubinsPoint.x_coordinate = q[0];
                newDubinsPoint.y_coordinate = q[1];
                newDubinsPoint.altitude = msg->currentAltitude;
                currentUAV->dubinsPoints.push_back(newDubinsPoint);
                
                //print out dubins stuff
                /*
                 std::stringstream ss; 
                 ss<<"/Users/Jeffrey/Documents/data/dubins" << msg->planeID << ".txt";      
                 std::string str(ss.str());
                 myfile.open(str.c_str(), std::ofstream::app);
                 myfile << newDubinsPoint.x_coordinate << " " << newDubinsPoint.y_coordinate << " " << "\n";
                 myfile.close();
                 */
                
                //ROS_INFO("\n ******X is %f \n *******Y is %f", newDubinsPoint.x_coordinate, newDubinsPoint.y_coordinate);
                step++;
            }
            //            currentUAV->dubinsPoints.pop_back(); //there seems to be 1 extra waypoint at end of avoidancepoints
            delete newDubinspath; 
        }
        else
        {
            //set current dubins to old dubins
            currentUAV->dubinsPoints = oldUAV->dubinsPoints;
        }
        
        //AU_UAV_ROS::waypoint temp = convertPositionToWaypoint(currentUAV->dubinsPoints[0]);
        
        //figure out would-be heading without CA:
    	double wouldBeHeading = getNewHeading(currentPose, currentUAV->dubinsPoints[0]);
        /*----------------------------------------------------------------------------------*/
        //Find FUZZY LOGIC PARAMETERS FOR CLOSEST PLANE:    
        //get closest plane to current plane here ----> maybe change this function to get "most dangerous" plane
        int closestPlane = getClosestPlane(msg->planeID, planeMap);
        //ROS_INFO("%d, %d", msg->planeID, closestPlane);
        closestUAV = &planeMap.find(closestPlane)->second;
        fuzzyParams = getFuzzyParams(currentUAV, closestUAV);
        /*----------------------------------------------------------------------------------*/
        nextGoal = currentUAV->goalWaypoints[0];
        //ROS_INFO("next goalWaypoints is %f  %f", nextGoal.latitude, nextGoal.longitude);
        nextGoalXY = getXYZ(nextGoal);
        
        //find distToWP
        distToWP = getActualDistance(planeLatLongAlt, nextGoal);
        //find distBtwnPlanes
        //distBtwnPlanes = getDist(currentUAV->getPosition(), closestUAV->getPosition());
        
        //Decide to enter fuzzy logic??????
        bool enterCA = firstFuzzyEngine(fuzzyParams.distanceToCollision, fuzzyParams.overlapDistance, distToWP, fuzzyParams.distBtwnPlanes);
        //ENTER CA:
        if (enterCA)
        {
            ROS_INFO("OurBA is %f", fuzzyParams.ourBearingAngle);
            ROS_INFO("TheirBA is %f", fuzzyParams.theirBearingAngle);
            ROS_INFO("Dist is %f", fuzzyParams.distBtwnPlanes);
            ROS_INFO("Wouldbbe Heaind gis %f", wouldBeHeading);
            //fuzzyHeading is just change in heading, so we need to add it to the wouldBeHeading
            double fuzzyHeading = secondFuzzyEngine(fuzzyParams.distBtwnPlanes, fuzzyParams.ourBearingAngle, fuzzyParams.theirBearingAngle);
            //ROS_INFO("first fuzzy is %f and currentPose is %f %f", fuzzyHeading, currentPose.x_coordinate, currentPose.y_coordinate);
            fuzzyHeading = fuzzyHeading;// + wouldBeHeading;
            //convert fuzzyHeading into a waypoint for the current Plane to go to
            //ROS_INFO("fuzzy is %f and currentPose is %f %f", fuzzyHeading, currentPose.x_coordinate, currentPose.y_coordinate);
            nextWaypoint = getCAWaypoint(fuzzyHeading, currentPose);
            
            //erase dubins
            currentUAV->dubinsPoints.clear();
        }
        //DIDN'T ENTER CA:
        else
        {
            nextWaypoint = convertPositionToWaypoint(currentUAV->dubinsPoints.front());
            //pop dubins
            currentUAV->dubinsPoints.erase(currentUAV->dubinsPoints.begin());
        }
        /*----------------------------------------------------------------------------------*/
        //go to nextWaypoint everytime
        //service request to go to the waypoint determined by fuzzy logicness OR normal waypoint
        //ROS_INFO("nextwaypoint is %f / %f / %f", nextWaypoint.latitude, nextWaypoint.longitude, nextWaypoint.altitude);
        AU_UAV_ROS::GoToWaypoint gotosrv;
        gotosrv.request.planeID = msg->planeID;
        gotosrv.request.latitude = nextWaypoint.latitude;
        gotosrv.request.longitude = nextWaypoint.longitude;
        
        //these settings mean it is an avoidance maneuver waypoint AND to clear the avoidance queue
        gotosrv.request.isAvoidanceManeuver = true;
        gotosrv.request.isNewQueue = true;
        if(goToWaypointClient.call(gotosrv))
        {
            //ROS_INFO("Received Response");
        }
        else
        {
            ROS_ERROR("Did not receive response");
        }
        //update map with current Plane Pose, Heading (currentUAV) everytime
        planeMap[msg->planeID] = *currentUAV; 
        /*----------------------------------------------------------------------------------*/
        //Write some Outputs (distance to closest object and distance to waypoint) to some files)
        
        std::stringstream ss; 
        ss<<"/Users/Jeffrey/Documents/data/UAV" << msg->planeID << ".txt";      
        //ss<<"/Users/Jeffrey/github/local/Team-IV/ros/AU_UAV_stack/AU_UAV_ROS/data/distanceToClosestPlane" << msg->planeID << ".txt";
        std::string str(ss.str());
        myfile.open(str.c_str(), std::ofstream::app);
        myfile << counter << " " << currentUAV->getX() << " " << currentUAV->getY() << " " <<
        nextGoalXY.x_coordinate << " " << nextGoalXY.y_coordinate << "\n";
        myfile.close();
        
        delete currentUAV;
    }
}


//This function will take inputs of min(A,teB) and A-B and output true or false to enter the CA algorithm where A is the distance for the current plane to the collision point and B is the distance to collision point for the closest plane to current plane. 
bool firstFuzzyEngine(double distanceToCollision, double overlapDistance, double distToWP, double distBtwnPlanes)
{
	bool collisionPotential = false; 	
	double output = fuzzy1.FuzzyLogicOne(distanceToCollision, overlapDistance);
	//ROS_INFO("for dToColl= %f and overlap= %f", distanceToCollision, overlapDistance);
	//ROS_INFO("fuzzy logic output = %f", output);
	
	if(output >= 0.4){
		collisionPotential = true; 
	}
	return collisionPotential;
    
}

//This function will take inputs of min(A,B), A-B, bearing angle and output the heading
double secondFuzzyEngine(double distBtwnPlanes, double ourBearingAngle, double theirBearingAngle)//double distanceToCollision, double overlapDistance, double bearingAngle)
{
	double changeInHeading = fuzzy1.oursandtheirs(distBtwnPlanes, ourBearingAngle, theirBearingAngle);
	//ROS_INFO("for distBtwnPlanes= %f and bearingAngle= %f ", distBtwnPlanes, bearingAngle); 	
	//ROS_INFO("Change in heading: %f", changeInHeading);
    return changeInHeading;
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
	//fl1 = AU_UAV_ROS::FuzzyLogicController();
    
	//random seed for if statement in telemetryCallback, remove when collision avoidance work begins
	srand(time(NULL));
    
	//needed for ROS to wait for callbacks
	ros::spin();	
    
	return 0;
}
