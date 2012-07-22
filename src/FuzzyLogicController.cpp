
/*

Fuzzy logic controllers for detecting collisions and
for determining appropriate collision avoidance maneuver

@authors Michelle Hromatka
         Jeffrey West

*/

#include "ros/ros.h"
#include "fuzzylite/FuzzyLite.h"
#include "AU_UAV_ROS/FuzzyLogicController.h"
#include <string>
#include <sstream>

double AU_UAV_ROS::FuzzyLogicController::FuzzyLogicOne(double cDist, double oDist){
        fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
        fl::FuzzyEngine engine("Collison-Detection", op);

        fl::InputLVar* distanceToCollision = new fl::InputLVar("CollDist");
        distanceToCollision->addTerm(new fl::ShoulderTerm("VERYCLOSE", 12.0, 26.0, true));
        distanceToCollision->addTerm(new fl::TriangularTerm("CLOSE", 18.0, 48.0));
       	distanceToCollision->addTerm(new fl::TriangularTerm("FAR", 42.0, 78.0));//42
        distanceToCollision->addTerm(new fl::ShoulderTerm("VERYFAR", 70.0, 78.0, false));
        engine.addInputLVar(distanceToCollision);
      
        //aMinusB, where A is the distance to collision point for the plane of interest
        //and B is dist to Collision for the nearest plane, aMinusB determines whether a collision
        //will actually happen or if the planes will be at that collision point at different times
        fl::InputLVar* aMinusB = new fl::InputLVar("OverlapDistance");
        aMinusB->addTerm(new fl::ShoulderTerm("VERYNEG", -24.0, -16.0, true));
        aMinusB->addTerm(new fl::TriangularTerm("NEG", -20.0, -8.0));
        aMinusB->addTerm(new fl::TriangularTerm("ZERO", -12.0, 12.0));
        aMinusB->addTerm(new fl::TriangularTerm("POS", 8.0, 20.0));
        aMinusB->addTerm(new fl::ShoulderTerm("VERYPOS", 16.0, 24.0, false));
        engine.addInputLVar(aMinusB);

        fl::OutputLVar* collImminence = new fl::OutputLVar("CollisionImminence");
        collImminence->addTerm(new fl::TriangularTerm("SAFE", 0.0, 0.4));
        collImminence->addTerm(new fl::TriangularTerm("POSSIBLE", 0.25, 0.75));
        collImminence->addTerm(new fl::TriangularTerm("DANGER", 0.6, 1.0));
        engine.addOutputLVar(collImminence);

        fl::RuleBlock* block = new fl::RuleBlock();
        block->addRule(new fl::MamdaniRule("if OverlapDistance is VERYNEG or OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));
//        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is VERYNEG then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is ZERO then CollisionImminence is DANGER", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is NEG then CollisionImminence is DANGER", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is POS then CollisionImminence is DANGER", engine));
 //	block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));
//	block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is VERYNEG then CollisionImminence is SAFE", engine)); 
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is ZERO then CollisionImminence is DANGER", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is POS then CollisionImminence is POSSIBLE", engine));
 //	block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));
//	block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is VERYNEG then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is ZERO then CollisionImminence is POSSIBLE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is POS then CollisionImminence is POSSIBLE", engine));
 //	block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));
//	block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is VERYNEG then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is ZERO then CollisionImminence is POSSIBLE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is POS then CollisionImminence is POSSIBLE", engine));
//	block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));

    engine.addRuleBlock(block);
    distanceToCollision->setInput(cDist);
    aMinusB->setInput(oDist);
    engine.process();
    double output = collImminence->output().defuzzify();
    
    //delete block;
    //delete aMinusB;
    //delete distanceToCollision;
    //delete collImminence;
    
    return output;
}

double AU_UAV_ROS::FuzzyLogicController::ZEM(double dist, double time){
    fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
    fl::FuzzyEngine engine("Collison-Detection", op);
    
    fl::InputLVar* ZEMDist = new fl::InputLVar("ZEMDist");
    ZEMDist->addTerm(new fl::ShoulderTerm("DANGER", 18.0, 24.0, true));
    ZEMDist->addTerm(new fl::TriangularTerm("ISH", 18.0, 30.0));
    ZEMDist->addTerm(new fl::ShoulderTerm("SAFE", 24.0, 30.0, false));
    engine.addInputLVar(ZEMDist);
    
    //aMinusB, where A is the distance to collision point for the plane of interest
    //and B is dist to Collision for the nearest plane, aMinusB determines whether a collision
    //will actually happen or if the planes will be at that collision point at different times
    fl::InputLVar* ZEMTime = new fl::InputLVar("ZEMTime");
    ZEMTime->addTerm(new fl::ShoulderTerm("DANGER", 4.0, 6.0, true));
    ZEMTime->addTerm(new fl::TriangularTerm("ISH", 4.0, 8.0));
    ZEMTime->addTerm(new fl::ShoulderTerm("SAFE", 6.0, 8.0, false));
    engine.addInputLVar(ZEMTime);
    
    fl::OutputLVar* collImminence = new fl::OutputLVar("CollisionImminence");
    collImminence->addTerm(new fl::TriangularTerm("SAFE", 0.0, 0.4));
    collImminence->addTerm(new fl::TriangularTerm("POSSIBLE", 0.25, 0.75));
    collImminence->addTerm(new fl::TriangularTerm("DANGER", 0.6, 1.0));
    engine.addOutputLVar(collImminence);
    
    fl::RuleBlock* block = new fl::RuleBlock();
	block->addRule(new fl::MamdaniRule("if ZEMDist is DANGER and ZEMTime is DANGER then CollisionImminence is DANGER", engine));
	block->addRule(new fl::MamdaniRule("if ZEMDist is DANGER and ZEMTime is ISH then CollisionImminence is DANGER", engine));
	block->addRule(new fl::MamdaniRule("if ZEMDist is DANGER and ZEMTime is SAFE then CollisionImminence is POSSIBLE", engine));
    
	block->addRule(new fl::MamdaniRule("if ZEMDist is ISH and ZEMTime is DANGER then CollisionImminence is DANGER", engine));
	block->addRule(new fl::MamdaniRule("if ZEMDist is ISH and ZEMTime is ISH then CollisionImminence is POSSIBLE", engine));
	block->addRule(new fl::MamdaniRule("if ZEMDist is ISH and ZEMTime is SAFE then CollisionImminence is POSSIBLE", engine));

	block->addRule(new fl::MamdaniRule("if ZEMDist is SAFE and ZEMTime is DANGER then CollisionImminence is POSSIBLE", engine));
	block->addRule(new fl::MamdaniRule("if ZEMDist is SAFE and ZEMTime is ISH then CollisionImminence is POSSIBLE", engine));
	block->addRule(new fl::MamdaniRule("if ZEMDist is SAFE and ZEMTime is SAFE then CollisionImminence is SAFE", engine));
    
    
    engine.addRuleBlock(block);
    ZEMDist->setInput(dist);
    ZEMTime->setInput(time);
    engine.process();
    double output = collImminence->output().defuzzify();
    
    //delete block;
    //delete aMinusB;
    //delete distanceToCollision;
    //delete collImminence;
    
    return output;
}



double AU_UAV_ROS::FuzzyLogicController::FuzzyLogicTwo(double distance, double angle){
        fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
        fl::FuzzyEngine engine("Heading-Change", op);

        fl::InputLVar* distanceBetweenPlanes = new fl::InputLVar("PlaneDist");
        distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYCLOSE", 12.0, 26.0, true));
        distanceBetweenPlanes->addTerm(new fl::TriangularTerm("CLOSE", 18.0, 48.0));
        distanceBetweenPlanes->addTerm(new fl::TriangularTerm("FAR", 42.0, 78.0));
        distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYFAR", 70.0, 78.0, false));
        engine.addInputLVar(distanceBetweenPlanes);
        
        //aMinusB, where A is the distance to collision point for the plane of interest
        //and B is dist to Collision for the nearest plane, aMinusB determines whether a collision
        //will actually happen or if the planes will be at that collision point at different times
        fl::InputLVar* bearingAngle = new fl::InputLVar("BearingAngle");
        bearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -45.0, true));
        bearingAngle->addTerm(new fl::TriangularTerm("NEG", -89, -22.5));
        bearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -45.0, 2.0));
        bearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", 2.0, 45.0));
        bearingAngle->addTerm(new fl::TriangularTerm("POS", 22.5, 89.0));
        bearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 45, 90.0, false));
        engine.addInputLVar(bearingAngle);
        fl::OutputLVar* changeHeading = new fl::OutputLVar("ChangeInHeading");
        changeHeading->addTerm(new fl::ShoulderTerm("VERYLEFT", -45, 0.0, true));
        changeHeading->addTerm(new fl::TriangularTerm("LEFT", -22.5, 0.0));
        changeHeading->addTerm(new fl::TriangularTerm("NOCHANGE", -22.5, 22.5));
        changeHeading->addTerm(new fl::TriangularTerm("RIGHT", 0.0, 22.5));
        changeHeading->addTerm(new fl::ShoulderTerm("VERYRIGHT", 0.0, 45, false));    
        engine.addOutputLVar(changeHeading);

        fl::RuleBlock* block = new fl::RuleBlock();
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is VERYNEG then ChangeInHeading is VERYRIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is NEG then ChangeInHeading is VERYRIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is LOWPOS then ChangeInHeading is VERYLEFT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is POS then ChangeInHeading is VERYLEFT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is VERYPOS then ChangeInHeading is VERYLEFT", engine));
        
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is VERYNEG then ChangeInHeading is RIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is NEG then ChangeInHeading is RIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is LOWPOS then ChangeInHeading is VERYLEFT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is POS then ChangeInHeading is LEFT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is VERYPOS then ChangeInHeading is LEFT", engine));    
        
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is NEG then ChangeInHeading is RIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is LOWNEG then ChangeInHeading is RIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is LOWPOS then ChangeInHeading is LEFT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is POS then ChangeInHeading is LEFT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
        
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is NEG then ChangeInHeading is NOCHANGE", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is LOWNEG then ChangeInHeading is RIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is LOWPOS then ChangeInHeading is LEFT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is POS then ChangeInHeading is NOCHANGE", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
        engine.addRuleBlock(block); 
 	
       
	distanceBetweenPlanes->setInput(distance);
	bearingAngle->setInput(angle);
        engine.process();
	return changeHeading->output().defuzzify();
}

double AU_UAV_ROS::FuzzyLogicController::FLJeffOne(double distance, double angle){
    fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
    fl::FuzzyEngine engine("Heading-Change", op);
    
    fl::InputLVar* distanceBetweenPlanes = new fl::InputLVar("PlaneDist");
    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYCLOSE", 24.0, 50.0, true));
    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("CLOSE", 24.0, 65.0));
    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("FAR", 35.0, 76.0));
    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYFAR", 50.0, 76.0, false));
    engine.addInputLVar(distanceBetweenPlanes);
    
    //aMinusB, where A is the distance to collision point for the plane of interest
    //and B is dist to Collision for the nearest plane, aMinusB determines whether a collision
    //will actually happen or if the planes will be at that collision point at different times
    fl::InputLVar* bearingAngle = new fl::InputLVar("BearingAngle");
    bearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -45.0, true));
    bearingAngle->addTerm(new fl::TriangularTerm("NEG", -89, -22.5));
    bearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -45.0, 2.0));
    bearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", 2.0, 45.0));
    bearingAngle->addTerm(new fl::TriangularTerm("POS", 22.5, 89.0));
    bearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 45, 90.0, false));
    engine.addInputLVar(bearingAngle);
    fl::OutputLVar* changeHeading = new fl::OutputLVar("ChangeInHeading");
    changeHeading->addTerm(new fl::ShoulderTerm("VERYLEFT", -45, 0.0, true));
    changeHeading->addTerm(new fl::TriangularTerm("LEFT", -22.5, 0.0));
    changeHeading->addTerm(new fl::TriangularTerm("NOCHANGE", -22.5, 22.5));
    changeHeading->addTerm(new fl::TriangularTerm("RIGHT", 0.0, 22.5));
    changeHeading->addTerm(new fl::ShoulderTerm("VERYRIGHT", 0.0, 45, false));    
    engine.addOutputLVar(changeHeading);
    
    fl::RuleBlock* block = new fl::RuleBlock();
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is VERYNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is NEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is LOWPOS then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is POS then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is VERYPOS then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is VERYNEG then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is NEG then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is LOWPOS then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is POS then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is VERYPOS then ChangeInHeading is LEFT", engine));    
    
    block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is NEG then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is LOWNEG then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is LOWPOS then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is POS then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
    
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is NEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is LOWNEG then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is LOWPOS then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is POS then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
    engine.addRuleBlock(block); 
 	
    
	distanceBetweenPlanes->setInput(distance);
	bearingAngle->setInput(angle);
    engine.process();
	return changeHeading->output().defuzzify();
}

double AU_UAV_ROS::FuzzyLogicController::tryturningintoplanes(double distance, double angle){
    fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
    fl::FuzzyEngine engine("Heading-Change", op);
    
    fl::InputLVar* distanceBetweenPlanes = new fl::InputLVar("PlaneDist");
    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYCLOSE", 24.0, 48.0, true));
    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("CLOSE", 40.0, 100.0));
    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("FAR", 90.0, 170.0));
    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYFAR", 150.0, 250.0, false));
    engine.addInputLVar(distanceBetweenPlanes);
    
    //aMinusB, where A is the distance to collision point for the plane of interest
    //and B is dist to Collision for the nearest plane, aMinusB determines whether a collision
    //will actually happen or if the planes will be at that collision point at different times
    fl::InputLVar* bearingAngle = new fl::InputLVar("BearingAngle");
    bearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -45.0, true));
    bearingAngle->addTerm(new fl::TriangularTerm("NEG", -89, -44.0));
    bearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -45.0, 2.0));
    bearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", 1.0, 45.0));
    bearingAngle->addTerm(new fl::TriangularTerm("POS", 44, 89.0));
    bearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 45, 90.0, false));
    engine.addInputLVar(bearingAngle);
    fl::OutputLVar* changeHeading = new fl::OutputLVar("ChangeInHeading");
    changeHeading->addTerm(new fl::ShoulderTerm("VERYLEFT", -45, 0.0, true));
    changeHeading->addTerm(new fl::TriangularTerm("LEFT", -22.5, 0.0));
    changeHeading->addTerm(new fl::TriangularTerm("NOCHANGE", -22.5, 22.5));
    changeHeading->addTerm(new fl::TriangularTerm("RIGHT", 0.0, 22.5));
    changeHeading->addTerm(new fl::ShoulderTerm("VERYRIGHT", 0.0, 45, false));    
    engine.addOutputLVar(changeHeading);
    
    fl::RuleBlock* block = new fl::RuleBlock();
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is VERYNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is NEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is LOWNEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is LOWPOS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is POS then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is VERYPOS then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is VERYNEG then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is NEG then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is LOWNEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is LOWPOS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is POS then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is VERYPOS then ChangeInHeading is LEFT", engine));    
    
    block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is NEG then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is LOWNEG then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is LOWPOS then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is POS then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
    
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is NEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is LOWNEG then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is LOWPOS then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is POS then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
    engine.addRuleBlock(block); 
 	
    
	distanceBetweenPlanes->setInput(distance);
	bearingAngle->setInput(angle);
    engine.process();
	return changeHeading->output().defuzzify();
}



double AU_UAV_ROS::FuzzyLogicController::oursandtheirs(double distance, double ourAngle, double theirAngle){
    fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
    fl::FuzzyEngine engine("Heading-Change", op);
    
    //possible consider changing this to ZEM
    fl::InputLVar* distanceBetweenPlanes = new fl::InputLVar("PlaneDist");
    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYCLOSE", 24.0, 48.0, true));
    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("CLOSE", 40.0, 100.0));
    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("FAR", 90.0, 170.0));
    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYFAR", 150.0, 250.0, false));
    engine.addInputLVar(distanceBetweenPlanes);
    
    fl::InputLVar* ourBearingAngle = new fl::InputLVar("OurBearingAngle");
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -45.0, true));
    ourBearingAngle->addTerm(new fl::TriangularTerm("NEG", -89, -44.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -45.0, 2.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", 1.0, 45.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("POS", 44, 89.0));
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 45, 90.0, false));
    engine.addInputLVar(ourBearingAngle);
    
    fl::InputLVar* theirBearingAngle = new fl::InputLVar("TheirBearingAngle");
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -45.0, true));
    theirBearingAngle->addTerm(new fl::TriangularTerm("NEG", -89, -44.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -45.0, 2.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", 1.0, 45.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("POS", 44, 89.0));
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 45, 90.0, false));
    engine.addInputLVar(theirBearingAngle);
    
    fl::OutputLVar* changeHeading = new fl::OutputLVar("ChangeInHeading");
    changeHeading->addTerm(new fl::TriangularTerm("VERYLEFT", -45, 0.0));//, true));
    changeHeading->addTerm(new fl::TriangularTerm("LEFT", -22.5, 0.0));
    changeHeading->addTerm(new fl::TriangularTerm("NOCHANGE", -22.5, 22.5));
    changeHeading->addTerm(new fl::TriangularTerm("RIGHT", 0.0, 22.5));
    changeHeading->addTerm(new fl::TriangularTerm("VERYRIGHT", 0.0, 45));//, false));    
    engine.addOutputLVar(changeHeading);
    
    fl::RuleBlock* block = new fl::RuleBlock();
    //PlaneDist is VERYCLOSE
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));

    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is LEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is LEFT", engine));
    
    //PlaneDist is CLOSE
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is RIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", engine));
    
    //PlaneDist is FAR
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is RIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is NOCHANGE", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is NOCHANGE", engine));
    
    //PlaneDist is VERYFAR
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is RIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", engine)); 
    
    engine.addRuleBlock(block); 

	distanceBetweenPlanes->setInput(distance);
	ourBearingAngle->setInput(ourAngle);
    theirBearingAngle->setInput(theirAngle);
    engine.process();
    double output = changeHeading->output().defuzzify();
    //delete block;
    //delete distanceBetweenPlanes;
    //delete ourBearingAngle;
    //delete theirBearingAngle;
    //delete changeHeading;
	return output;
}

double AU_UAV_ROS::FuzzyLogicController::oursandtheirs2(double distance, double ourAngle, double theirAngle){
    fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
    fl::FuzzyEngine engine("Heading-Change", op);
    
    //possible consider changing this to ZEM
    fl::InputLVar* distanceBetweenPlanes = new fl::InputLVar("PlaneDist");
    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYCLOSE", 24.0, 50.0, true));
    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("CLOSE", 24.0, 65.0));
    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("FAR", 35.0, 76.0));
    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYFAR", 50.0, 76.0, false));
//    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYCLOSE", 24.0, 30.0, true));
//    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("CLOSE", 24.0, 60.0));
//    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("FAR", 40.0, 76.0));
//    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYFAR", 70.0, 76.0, false));
    engine.addInputLVar(distanceBetweenPlanes);
    
    fl::InputLVar* ourBearingAngle = new fl::InputLVar("OurBearingAngle");
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -45.0, true));
    ourBearingAngle->addTerm(new fl::TriangularTerm("NEG", -89, -44.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -45.0, 2.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", 1.0, 45.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("POS", 44, 89.0));
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 45, 90.0, false));
    engine.addInputLVar(ourBearingAngle);
    
    fl::InputLVar* theirBearingAngle = new fl::InputLVar("TheirBearingAngle");
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -45.0, true));
    theirBearingAngle->addTerm(new fl::TriangularTerm("NEG", -89, -44.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -45.0, 2.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", 1.0, 45.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("POS", 44, 89.0));
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 45, 90.0, false));
    engine.addInputLVar(theirBearingAngle);
    
    fl::OutputLVar* changeHeading = new fl::OutputLVar("ChangeInHeading");
    changeHeading->addTerm(new fl::TriangularTerm("VERYLEFT", -45, 0.0));//, true));
    changeHeading->addTerm(new fl::TriangularTerm("LEFT", -22.5, 0.0));
    changeHeading->addTerm(new fl::TriangularTerm("NOCHANGE", -22.5, 22.5));
    changeHeading->addTerm(new fl::TriangularTerm("RIGHT", 0.0, 22.5));
    changeHeading->addTerm(new fl::TriangularTerm("VERYRIGHT", 0.0, 45));//, false));    
    engine.addOutputLVar(changeHeading);
    
    fl::RuleBlock* block = new fl::RuleBlock();
    //PlaneDist is VERYCLOSE
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is LEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is LEFT", engine));
    
    //PlaneDist is CLOSE
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is RIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", engine));
    
    //PlaneDist is FAR
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is RIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is NOCHANGE", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is NOCHANGE", engine));
    
    //PlaneDist is VERYFAR
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is RIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", engine)); 
    
    engine.addRuleBlock(block); 
    
	distanceBetweenPlanes->setInput(distance);
	ourBearingAngle->setInput(ourAngle);
    theirBearingAngle->setInput(theirAngle);
    engine.process();
    double output = changeHeading->output().defuzzify();
    //delete block;
    //delete distanceBetweenPlanes;
    //delete ourBearingAngle;
    //delete theirBearingAngle;
    //delete changeHeading;
	return output;
}


double AU_UAV_ROS::FuzzyLogicController::sevenOutputs(double distance, double ourAngle, double theirAngle){
    fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
    fl::FuzzyEngine engine("Heading-Change", op);
    
    //possible consider changing this to ZEM
    fl::InputLVar* distanceBetweenPlanes = new fl::InputLVar("PlaneDist");
    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("CLOSE", 24.0, 50.0, true));
    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("MEDIUM", 24.0, 65.0));
    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("FAR", 50.0, 76.0, false));
    engine.addInputLVar(distanceBetweenPlanes);
    
    fl::InputLVar* ourBearingAngle = new fl::InputLVar("OurBearingAngle");
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -60.0, true));
    ourBearingAngle->addTerm(new fl::TriangularTerm("NEG", -70, -20.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -21.0, 1.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", -1.0, 21.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("POS", 20, 70.0));
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 60, 90.0, false));
    engine.addInputLVar(ourBearingAngle);
    
    fl::InputLVar* theirBearingAngle = new fl::InputLVar("TheirBearingAngle");
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -60.0, true));
    theirBearingAngle->addTerm(new fl::TriangularTerm("NEG", -70, -20.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -21.0, 1.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", -1.0, 21.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("POS", 20, 70.0));
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 60, 90.0, false));
    engine.addInputLVar(theirBearingAngle);
    
    fl::OutputLVar* changeHeading = new fl::OutputLVar("ChangeInHeading");
    changeHeading->addTerm(new fl::TriangularTerm("VERYLEFT", -32.5, 12.5));
    changeHeading->addTerm(new fl::TriangularTerm("LEFT", -21.25, -1.25));
    changeHeading->addTerm(new fl::TriangularTerm("SLIGHTLEFT", -10.5, 0.0));
    changeHeading->addTerm(new fl::TriangularTerm("NOCHANGE", -10.0, 10.0));
    changeHeading->addTerm(new fl::TriangularTerm("SLIGHTRIGHT", 0.0, 10.5));
    changeHeading->addTerm(new fl::TriangularTerm("RIGHT", 1.25, 21.25));
    changeHeading->addTerm(new fl::TriangularTerm("VERYRIGHT", 12.5, 32.5));     
    engine.addOutputLVar(changeHeading);
    
    fl::RuleBlock* block = new fl::RuleBlock();        
    //PlaneDist is CLOSE
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is LEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is SLIGHTLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is SLIGHTLEFT", engine));
    
    //PlaneDist is MEDIUM
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG and PlaneDist is MEDIUM then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG and PlaneDist is MEDIUM then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG and PlaneDist is MEDIUM then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS and PlaneDist is MEDIUM then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS and PlaneDist is MEDIUM then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS and PlaneDist is MEDIUM then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG and PlaneDist is MEDIUM then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG and PlaneDist is MEDIUM then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG and PlaneDist is MEDIUM then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS and PlaneDist is MEDIUM then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS and PlaneDist is MEDIUM then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS and PlaneDist is MEDIUM then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG and PlaneDist is MEDIUM then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG and PlaneDist is MEDIUM then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG and PlaneDist is MEDIUM then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS and PlaneDist is MEDIUM then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS and PlaneDist is MEDIUM then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS and PlaneDist is MEDIUM then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG and PlaneDist is MEDIUM then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG and PlaneDist is MEDIUM then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG and PlaneDist is MEDIUM then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS and PlaneDist is MEDIUM then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS and PlaneDist is MEDIUM then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS and PlaneDist is MEDIUM then ChangeInHeading is LEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG and PlaneDist is MEDIUM then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG and PlaneDist is MEDIUM then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG and PlaneDist is MEDIUM then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS and PlaneDist is MEDIUM then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS and PlaneDist is MEDIUM then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS and PlaneDist is MEDIUM then ChangeInHeading is SLIGHTLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG and PlaneDist is MEDIUM then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG and PlaneDist is MEDIUM then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG and PlaneDist is MEDIUM then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS and PlaneDist is MEDIUM then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS and PlaneDist is MEDIUM then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS and PlaneDist is MEDIUM then ChangeInHeading is SLIGHTLEFT", engine)); 
    
    //PlaneDist is FAR
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is LEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is SLIGHTLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is SLIGHTLEFT", engine));
    engine.addRuleBlock(block); 
    
	distanceBetweenPlanes->setInput(distance);
	ourBearingAngle->setInput(ourAngle);
    theirBearingAngle->setInput(theirAngle);
    engine.process();
    double output = changeHeading->output().defuzzify();
	return output;
}

double AU_UAV_ROS::FuzzyLogicController::smallDistance(double ourAngle, double theirAngle){
    fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
    fl::FuzzyEngine engine("Heading-Change", op);
    
    fl::InputLVar* ourBearingAngle = new fl::InputLVar("OurBearingAngle");
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -60.0, true));
    ourBearingAngle->addTerm(new fl::TriangularTerm("NEG", -70, -20.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -21.0, 1.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", -1.0, 21.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("POS", 20, 70.0));
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 60, 90.0, false));
    engine.addInputLVar(ourBearingAngle);
    
    fl::InputLVar* theirBearingAngle = new fl::InputLVar("TheirBearingAngle");
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -60.0, true));
    theirBearingAngle->addTerm(new fl::TriangularTerm("NEG", -70, -20.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -21.0, 1.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", -1.0, 21.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("POS", 20, 70.0));
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 60, 90.0, false));
    engine.addInputLVar(theirBearingAngle);
    
    fl::OutputLVar* changeHeading = new fl::OutputLVar("ChangeInHeading");
    changeHeading->addTerm(new fl::TriangularTerm("VERYLEFT", -32.5, 12.5));
    changeHeading->addTerm(new fl::TriangularTerm("LEFT", -21.25, -1.25));
    changeHeading->addTerm(new fl::TriangularTerm("SLIGHTLEFT", -10.5, 0.0));
    changeHeading->addTerm(new fl::TriangularTerm("NOCHANGE", -10.0, 10.0));
    changeHeading->addTerm(new fl::TriangularTerm("SLIGHTRIGHT", 0.0, 10.5));
    changeHeading->addTerm(new fl::TriangularTerm("RIGHT", 1.25, 21.25));
    changeHeading->addTerm(new fl::TriangularTerm("VERYRIGHT", 12.5, 32.5));     
    engine.addOutputLVar(changeHeading);
    
    fl::RuleBlock* block = new fl::RuleBlock();        
    //PlaneDist is CLOSE
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS then ChangeInHeading is RIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS then ChangeInHeading is SLIGHTLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG then ChangeInHeading is LEFT  ", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
    
    engine.addRuleBlock(block); 
	ourBearingAngle->setInput(ourAngle);
    theirBearingAngle->setInput(theirAngle);
    engine.process();
    double output = changeHeading->output().defuzzify();
	return output;
}

double AU_UAV_ROS::FuzzyLogicController::smallDistance2(double ourAngle, double theirAngle){
    fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
    fl::FuzzyEngine engine("Heading-Change", op);
    
    fl::InputLVar* ourBearingAngle = new fl::InputLVar("OurBearingAngle");
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -40.0, true));
    ourBearingAngle->addTerm(new fl::TriangularTerm("NEG", -50, -10.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -20.0, 1.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", -1.0, 20.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("POS", 10, 50.0));
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 40, 90.0, false));
    engine.addInputLVar(ourBearingAngle);
    
    fl::InputLVar* theirBearingAngle = new fl::InputLVar("TheirBearingAngle");
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -40.0, true));
    theirBearingAngle->addTerm(new fl::TriangularTerm("NEG", -50, -10.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -20.0, 1.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", -1.0, 20.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("POS", 10, 50.0));
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 40, 90.0, false));
    engine.addInputLVar(theirBearingAngle);
    
    fl::OutputLVar* changeHeading = new fl::OutputLVar("ChangeInHeading");
    changeHeading->addTerm(new fl::TriangularTerm("VERYLEFT", -32.5, 12.5));
    changeHeading->addTerm(new fl::TriangularTerm("LEFT", -21.25, -1.25));
    changeHeading->addTerm(new fl::TriangularTerm("SLIGHTLEFT", -10.5, 0.0));
    changeHeading->addTerm(new fl::TriangularTerm("NOCHANGE", -10.0, 10.0));
    changeHeading->addTerm(new fl::TriangularTerm("SLIGHTRIGHT", 0.0, 10.5));
    changeHeading->addTerm(new fl::TriangularTerm("RIGHT", 1.25, 21.25));
    changeHeading->addTerm(new fl::TriangularTerm("VERYRIGHT", 12.5, 32.5));     
    engine.addOutputLVar(changeHeading);
    
    fl::RuleBlock* block = new fl::RuleBlock();        
    //PlaneDist is CLOSE
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS then ChangeInHeading is RIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS then ChangeInHeading is SLIGHTLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG then ChangeInHeading is LEFT  ", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
    
    engine.addRuleBlock(block); 
	ourBearingAngle->setInput(ourAngle);
    theirBearingAngle->setInput(theirAngle);
    engine.process();
    double output = changeHeading->output().defuzzify();
	return output;
}
/*
double AU_UAV_ROS::FuzzyLogicController::ZEMStuff(double ourAngle, double theirAngle){
    fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
    fl::FuzzyEngine engine("Heading-Change", op);
    
    fl::InputLVar* ourBearingAngle = new fl::InputLVar("OurBearingAngle");
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -40.0, true));
    ourBearingAngle->addTerm(new fl::TriangularTerm("NEG", -50, -10.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -20.0, 1.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", -1.0, 20.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("POS", 10, 50.0));
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 40, 90.0, false));
    engine.addInputLVar(ourBearingAngle);
    
    fl::InputLVar* theirBearingAngle = new fl::InputLVar("TheirBearingAngle");
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -40.0, true));
    theirBearingAngle->addTerm(new fl::TriangularTerm("NEG", -50, -10.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -20.0, 1.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", -1.0, 20.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("POS", 10, 50.0));
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 40, 90.0, false));
    engine.addInputLVar(theirBearingAngle);
    
    fl::OutputLVar* changeHeading = new fl::OutputLVar("ChangeInHeading");
    changeHeading->addTerm(new fl::TriangularTerm("VERYLEFT", -32.5, 12.5));
    changeHeading->addTerm(new fl::TriangularTerm("LEFT", -21.25, -1.25));
    changeHeading->addTerm(new fl::TriangularTerm("SLIGHTLEFT", -10.5, 0.0));
    changeHeading->addTerm(new fl::TriangularTerm("NOCHANGE", -10.0, 10.0));
    changeHeading->addTerm(new fl::TriangularTerm("SLIGHTRIGHT", 0.0, 10.5));
    changeHeading->addTerm(new fl::TriangularTerm("RIGHT", 1.25, 21.25));
    changeHeading->addTerm(new fl::TriangularTerm("VERYRIGHT", 12.5, 32.5));     
    engine.addOutputLVar(changeHeading);
    
    fl::RuleBlock* block = new fl::RuleBlock();        
    //PlaneDist is CLOSE
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS then ChangeInHeading is RIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS then ChangeInHeading is SLIGHTLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG then ChangeInHeading is LEFT  ", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
    
    engine.addRuleBlock(block); 
	ourBearingAngle->setInput(ourAngle);
    theirBearingAngle->setInput(theirAngle);
    engine.process();
    double output = changeHeading->output().defuzzify();
	return output;
}
 */

double AU_UAV_ROS::FuzzyLogicController::copyCat(double distance, double ourAngle){
    fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
    fl::FuzzyEngine engine("Heading-Change", op);
    
    fl::InputLVar* distanceBetweenPlanes = new fl::InputLVar("PlaneDist");
    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("DANGER", 12.0, 26.0, true));
    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("ALERT", 18.0, 48.0));
    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("MEDIUM", 42.0, 78.0));
    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("SAFE", 70.0, 78.0, false));
    engine.addInputLVar(distanceBetweenPlanes);
    
    fl::InputLVar* ourBearingAngle = new fl::InputLVar("OurBearingAngle");
    fl::TriangularTerm *posleave = new fl::TriangularTerm("POSLEAVE", -185.0, -45.0);
    posleave->setB(-90.0);
    ourBearingAngle->addTerm(posleave);
    ourBearingAngle->addTerm(new fl::TriangularTerm("POSFIT", -90.0, 0.0));
    fl::TriangularTerm *poshead = new fl::TriangularTerm("POSHEAD", -45.0, 0.0);
    poshead->setB(-0.01);
    ourBearingAngle->addTerm(poshead);
    fl::TriangularTerm *neghead = new fl::TriangularTerm("NEGHEAD", 0.0, 45.0);
    neghead->setB(0.01);
    ourBearingAngle->addTerm(neghead);
    ourBearingAngle->addTerm(new fl::TriangularTerm("NEGFIT", 0.0, 90.0));
    fl::TriangularTerm *negleave = new fl::TriangularTerm("NEGLEAVE", 45.0, 185.0);
    negleave->setB(90.0);
    ourBearingAngle->addTerm(negleave);
    engine.addInputLVar(ourBearingAngle);
    
    fl::OutputLVar* changeHeading = new fl::OutputLVar("ChangeInHeading");
    changeHeading->addTerm(new fl::TriangularTerm("POSLARGE", -32.5, 12.5));
    changeHeading->addTerm(new fl::TriangularTerm("POSMEDIUM", -21.25, -1.25));
    changeHeading->addTerm(new fl::TriangularTerm("POSSMALL", -10.5, 0.0));
    changeHeading->addTerm(new fl::TriangularTerm("NOCHANGE", -10.0, 10.0));
    changeHeading->addTerm(new fl::TriangularTerm("NEGSMALL", 0.0, 10.5));
    changeHeading->addTerm(new fl::TriangularTerm("NEGMEDIUM", 1.25, 21.25));
    changeHeading->addTerm(new fl::TriangularTerm("NEGLARGE", 12.5, 32.5));     
    engine.addOutputLVar(changeHeading);
    
    fl::RuleBlock* block = new fl::RuleBlock();        
    //PlaneDist is CLOSE
    block->addRule(new fl::MamdaniRule("if PlaneDist is SAFE and OurBearingAngle is POSLEAVE then ChangeInHeading is POSSMALL", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is SAFE and OurBearingAngle is POSFIT then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is SAFE and OurBearingAngle is POSHEAD then ChangeInHeading is NEGSMALL", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is SAFE and OurBearingAngle is NEGHEAD then ChangeInHeading is POSSMALL", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is SAFE and OurBearingAngle is NEGFIT then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is SAFE and OurBearingAngle is NEGLEAVE then ChangeInHeading is NEGSMALL", engine));

    block->addRule(new fl::MamdaniRule("if PlaneDist is MEDIUM and OurBearingAngle is POSLEAVE then ChangeInHeading is POSMALL", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is MEDIUM and OurBearingAngle is POSFIT then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is MEDIUM and OurBearingAngle is POSHEAD then ChangeInHeading is NEGMEDIUM", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is MEDIUM and OurBearingAngle is NEGHEAD then ChangeInHeading is POSMEDIUM", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is MEDIUM and OurBearingAngle is NEGFIT then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is MEDIUM and OurBearingAngle is NEGLEAVE then ChangeInHeading is NEGSMALL", engine));
    
    block->addRule(new fl::MamdaniRule("if PlaneDist is ALERT and OurBearingAngle is POSLEAVE then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is ALERT and OurBearingAngle is POSFIT then ChangeInHeading is NEGSMALL", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is ALERT and OurBearingAngle is POSHEAD then ChangeInHeading is NEGLARGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is ALERT and OurBearingAngle is NEGHEAD then ChangeInHeading is POSLARGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is ALERT and OurBearingAngle is NEGFIT then ChangeInHeading is POSSMALL", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is ALERT and OurBearingAngle is NEGLEAVE then ChangeInHeading is NOCHANGE", engine));
    
    block->addRule(new fl::MamdaniRule("if PlaneDist is DANGER and OurBearingAngle is POSLEAVE then ChangeInHeading is NEGSMALL", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is DANGER and OurBearingAngle is POSFIT then ChangeInHeading is NEGMEDIUM", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is DANGER and OurBearingAngle is POSHEAD then ChangeInHeading is NEGLARGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is DANGER and OurBearingAngle is NEGHEAD then ChangeInHeading is POSLARGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is DANGER and OurBearingAngle is NEGFIT then ChangeInHeading is POSMEDIUM", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is DANGER and OurBearingAngle is NEGLEAVE then ChangeInHeading is POSSMALL", engine));
    
    engine.addRuleBlock(block); 
	ourBearingAngle->setInput(ourAngle);
    engine.process();
    double output = changeHeading->output().defuzzify();
	return output;
}

double AU_UAV_ROS::FuzzyLogicController::lowerTurnsHighDists(double ourAngle, double theirAngle){
    fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
    fl::FuzzyEngine engine("Heading-Change", op);
    
    fl::InputLVar* ourBearingAngle = new fl::InputLVar("OurBearingAngle");
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -40.0, true));
    ourBearingAngle->addTerm(new fl::TriangularTerm("NEG", -50, -10.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -20.0, 1.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", -1.0, 20.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("POS", 10, 50.0));
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 40, 90.0, false));
    engine.addInputLVar(ourBearingAngle);
    
    fl::InputLVar* theirBearingAngle = new fl::InputLVar("TheirBearingAngle");
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -40.0, true));
    theirBearingAngle->addTerm(new fl::TriangularTerm("NEG", -50, -10.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -20.0, 1.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", -1.0, 20.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("POS", 10, 50.0));
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 40, 90.0, false));
    engine.addInputLVar(theirBearingAngle);
    
    fl::OutputLVar* changeHeading = new fl::OutputLVar("ChangeInHeading");
    changeHeading->addTerm(new fl::TriangularTerm("VERYLEFT", -15.0, 10.0));
    changeHeading->addTerm(new fl::TriangularTerm("LEFT", -15.0, -1.0));
    changeHeading->addTerm(new fl::TriangularTerm("SLIGHTLEFT", -5.0, 0.0));
    changeHeading->addTerm(new fl::TriangularTerm("NOCHANGE", -3.0, 3.0));
    changeHeading->addTerm(new fl::TriangularTerm("SLIGHTRIGHT", 0.0, 5.0));
    changeHeading->addTerm(new fl::TriangularTerm("RIGHT", 1.0, 15.0));
    changeHeading->addTerm(new fl::TriangularTerm("VERYRIGHT", 10.0, 15.0));     
    engine.addOutputLVar(changeHeading);
    
    fl::RuleBlock* block = new fl::RuleBlock();        
    //PlaneDist is CLOSE
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS then ChangeInHeading is RIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS then ChangeInHeading is SLIGHTLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG then ChangeInHeading is LEFT  ", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
    
    engine.addRuleBlock(block); 
	ourBearingAngle->setInput(ourAngle);
    theirBearingAngle->setInput(theirAngle);
    engine.process();
    double output = changeHeading->output().defuzzify();
	return output;
}

double AU_UAV_ROS::FuzzyLogicController::semiLowerTurnsHighDists(double ourAngle, double theirAngle){
    fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
    fl::FuzzyEngine engine("Heading-Change", op);
    
    fl::InputLVar* ourBearingAngle = new fl::InputLVar("OurBearingAngle");
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -40.0, true));
    ourBearingAngle->addTerm(new fl::TriangularTerm("NEG", -50, -10.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -20.0, 1.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", -1.0, 20.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("POS", 10, 50.0));
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 40, 90.0, false));
    engine.addInputLVar(ourBearingAngle);
    
    fl::InputLVar* theirBearingAngle = new fl::InputLVar("TheirBearingAngle");
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -40.0, true));
    theirBearingAngle->addTerm(new fl::TriangularTerm("NEG", -50, -10.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -20.0, 1.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", -1.0, 20.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("POS", 10, 50.0));
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 40, 90.0, false));
    engine.addInputLVar(theirBearingAngle);
    
    fl::OutputLVar* changeHeading = new fl::OutputLVar("ChangeInHeading");
    changeHeading->addTerm(new fl::TriangularTerm("VERYLEFT", -15.0, 10.0));
    changeHeading->addTerm(new fl::TriangularTerm("LEFT", -15.0, -1.0));
    changeHeading->addTerm(new fl::TriangularTerm("SLIGHTLEFT", -5.0, 0.0));
    changeHeading->addTerm(new fl::TriangularTerm("NOCHANGE", -3.0, 3.0));
    changeHeading->addTerm(new fl::TriangularTerm("SLIGHTRIGHT", 0.0, 5.0));
    changeHeading->addTerm(new fl::TriangularTerm("RIGHT", 1.0, 15.0));
    changeHeading->addTerm(new fl::TriangularTerm("VERYRIGHT", 10.0, 15.0));     
    engine.addOutputLVar(changeHeading);
    
    fl::RuleBlock* block = new fl::RuleBlock();        
    //PlaneDist is CLOSE
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS then ChangeInHeading is RIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS then ChangeInHeading is VERYRIGHT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG then ChangeInHeading is SLIGHTRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS then ChangeInHeading is VERYLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS then ChangeInHeading is SLIGHTLEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG then ChangeInHeading is VERYLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG then ChangeInHeading is LEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG then ChangeInHeading is LEFT  ", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS then ChangeInHeading is SLIGHTLEFT", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
    
    engine.addRuleBlock(block); 
	ourBearingAngle->setInput(ourAngle);
    theirBearingAngle->setInput(theirAngle);
    engine.process();
    double output = changeHeading->output().defuzzify();
	return output;
}
