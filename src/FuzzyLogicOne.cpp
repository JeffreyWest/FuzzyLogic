
/*

Fuzzy logic controllers for detecting collisions and
for determining appropriate collision avoidance maneuver

@authors Michelle Hromatka
         Jeffrey West
[
*/


#include "ros/ros.h"
#include <cmath>
#include "fuzzylite/FuzzyLite.h"
#include "AU_UAV_ROS/FuzzyLogicOne.h"
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

//default constructor
AU_UAV_ROS::FuzzyLogicOne::FuzzyLogicOne(void){
	ROS_INFO("CREATED*********************************************************************");
        fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
	ROS_INFO("OPER*********************************************************************");
    this->engine = fl::FuzzyEngine("Collison-Detection", op);

	ROS_INFO("INPUTDISTcOLL*********************************************************************");
        this->distanceToCollision = new fl::InputLVar("CollDist");
        distanceToCollision->addTerm(new fl::ShoulderTerm("VERYCLOSE", 24.0, 36.0, true));
        distanceToCollision->addTerm(new fl::TriangularTerm("CLOSE", 30.0, 60.0));
       	distanceToCollision->addTerm(new fl::TriangularTerm("FAR", 54.0, 96.0));
        distanceToCollision->addTerm(new fl::ShoulderTerm("VERYFAR", 90.0, 108.0, false));
        engine.addInputLVar(distanceToCollision);
      
        this->aMinusB = new fl::InputLVar("OverlapDistance");
        aMinusB->addTerm(new fl::ShoulderTerm("VERYNEG", -32.0, -20.0, true));
        aMinusB->addTerm(new fl::TriangularTerm("NEG", -24.0, -12.0));
        aMinusB->addTerm(new fl::TriangularTerm("ZERO", -18.0, 18.0));
        aMinusB->addTerm(new fl::TriangularTerm("POS", 12.0, 24.0));
        aMinusB->addTerm(new fl::ShoulderTerm("VERYPOS", 20.0, 32.0, false));
        engine.addInputLVar(aMinusB);

        this->collImminence = new fl::OutputLVar("CollisionImminence");
        collImminence->addTerm(new fl::ShoulderTerm("SAFE", 0.0, 0.4, true));
        collImminence->addTerm(new fl::TriangularTerm("POSSIBLE", 0.25, 0.75));
        collImminence->addTerm(new fl::ShoulderTerm("DANGER", 0.6, 1.0, false));
        engine.addOutputLVar(collImminence);
	ROS_INFO("BLOCK*********************************************************************");
      
	this->block = new fl::RuleBlock();
    ROS_INFO("hereeee");
	this->block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is ZERO then CollisionImminence is DANGER", engine));
    ROS_INFO("here?");
	fl::MamdaniRule* test = new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine);
	ROS_INFO("blah");
    this->block->addRule(test);
    ROS_INFO("blah2.0");
    ROS_INFO("%s", this->block->toString().c_str());
/*
        this->block->addRule(new fl::MamdaniRule("if OverlapDistance is VERYNEG or OverlapDistance is VERYPOS then CollisionImminence is SAFE", this->engine));
	ROS_INFO("RULE*********************************************************************");        
	block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is ZERO then CollisionImminence is DANGER", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is NEG then CollisionImminence is DANGER", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is POS then CollisionImminence is DANGER", engine)); 
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is ZERO then CollisionImminence is DANGER", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is POS then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is ZERO then CollisionImminence is POSSIBLE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is POS then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is ZERO then CollisionImminence is POSSIBLE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is POS then CollisionImminence is POSSIBLE", engine));
  */     
    ROS_INFO("BLOa;lsdkfjsCK*********************************************************************");

	engine.addRuleBlock(this->block);
    ROS_INFO("endA;SLDKFJAO;KDSJF*********************************************************************");

}

double AU_UAV_ROS::FuzzyLogicOne::process(double distToCollision, double oDist){
	ROS_INFO("1************************************************************************");
	this->distanceToCollision->setInput(distToCollision);
	ROS_INFO("2************************************************************************");
    this->aMinusB->setInput(oDist);
    this->engine.process();
       
	return this->collImminence->output().defuzzify();
	
}
