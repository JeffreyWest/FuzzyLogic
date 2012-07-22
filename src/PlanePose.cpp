
#include "ros/ros.h"
#include "AU_UAV_ROS/standardFuncs.h"
#include "AU_UAV_ROS/PlanePose.h"

//Define a waypoint
struct waypoint
{
	double latitude;
	double longitude;
	double altitude;
};


//default constructor
AU_UAV_ROS::PlanePose::PlanePose(void){
	this->planePoseID = 0;
	this->planePoseX = 0;
	this->planePoseY = 0;
	this->planePoseZ = 0;
	this->planePoseHeading = 0;
	this->planePoseVelocity = 0;
}

AU_UAV_ROS::PlanePose::PlanePose(int id, double x, double y, double z, double heading, double velocity){
	this->planePoseID = id;
	this->planePoseX = x;
	this->planePoseY = y;
	this->planePoseZ = z;
	this->planePoseHeading = heading;
	this->planePoseVelocity = velocity;
}

void AU_UAV_ROS::PlanePose::setID(int newID){
	this->planePoseID = newID;
}
void AU_UAV_ROS::PlanePose::setX(double newX){
	this->planePoseX = newX; 
}
void AU_UAV_ROS::PlanePose::setY(double newY){
	this->planePoseY = newY; 
}
void AU_UAV_ROS::PlanePose::setZ(double newZ){
	this->planePoseZ = newZ; 
}
void AU_UAV_ROS::PlanePose::setHeading(double newHeading){
	this->planePoseHeading = newHeading;
}
void AU_UAV_ROS::PlanePose::setVelocity(double newVelocity){
	this->planePoseVelocity = newVelocity;
}
int AU_UAV_ROS::PlanePose::getID(){
	return this->planePoseID;
}
double AU_UAV_ROS::PlanePose::getX(){
	return this->planePoseX;
}
double AU_UAV_ROS::PlanePose::getY(){
	return this->planePoseY; 
}
double AU_UAV_ROS::PlanePose::getZ(){
	return this->planePoseZ; 
}
double AU_UAV_ROS::PlanePose::getHeading(){
	return this->planePoseHeading; 
}
double AU_UAV_ROS::PlanePose::getVelocity(){
	return this->planePoseVelocity;
}
AU_UAV_ROS::position AU_UAV_ROS::PlanePose::getPosition(){
    AU_UAV_ROS::position currentPose;
    currentPose.x_coordinate = this->getX();
    currentPose.y_coordinate = this->getY();
    currentPose.altitude = this->getZ();
    
    return currentPose;
}

void AU_UAV_ROS::PlanePose::update(const AU_UAV_ROS::TelemetryUpdate &msg){
	AU_UAV_ROS::waypoint telemetryInfo;
	telemetryInfo.latitude = msg.currentLatitude;
	telemetryInfo.longitude = msg.currentLongitude;
	telemetryInfo.altitude = msg.currentAltitude;	

	AU_UAV_ROS::position newPoseData = getXYZ(telemetryInfo);	/* call inherited function */
    
	this->setX(newPoseData.x_coordinate); 
	this->setY(newPoseData.y_coordinate); 
	this->setZ(newPoseData.altitude);

}
