
/*

Fuzzy logic controller used in determining appropriate 
collision avoidance maneuver. Based on inputs of 
current plane bearing angle, closest plane bearing
angle and distance between planes

@authors Michelle Hromatka
         Jeffrey West



*/

#include "ros/ros.h"
#include <cmath>
#include "fuzzylite/FuzzyLite.h"
#include "AU_UAV_ROS/OursAndTheirs2.h"
#include <string>
#include <sstream>

//default constructor
AU_UAV_ROS::OursAndTheirs2::OursAndTheirs2(void){
        fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
        this->engine = new fl::FuzzyEngine("Heading-Change", op);
    
    distanceBetweenPlanes = new fl::InputLVar("PlaneDist");
    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYCLOSE", 24.0, 50.0, true));
    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("CLOSE", 24.0, 65.0));
    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("FAR", 35.0, 76.0));
    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYFAR", 50.0, 76.0, false));
    engine->addInputLVar(distanceBetweenPlanes);
    
    ourBearingAngle = new fl::InputLVar("OurBearingAngle");
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -45.0, true));
    ourBearingAngle->addTerm(new fl::TriangularTerm("NEG", -89, -44.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -45.0, 2.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", 1.0, 45.0));
    ourBearingAngle->addTerm(new fl::TriangularTerm("POS", 44, 89.0));
    ourBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 45, 90.0, false));
    engine->addInputLVar(ourBearingAngle);
    
    theirBearingAngle = new fl::InputLVar("TheirBearingAngle");
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -45.0, true));
    theirBearingAngle->addTerm(new fl::TriangularTerm("NEG", -89, -44.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -45.0, 2.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", 1.0, 45.0));
    theirBearingAngle->addTerm(new fl::TriangularTerm("POS", 44, 89.0));
    theirBearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 45, 90.0, false));
    engine->addInputLVar(theirBearingAngle);
    
    changeHeading = new fl::OutputLVar("ChangeInHeading");
    changeHeading->addTerm(new fl::TriangularTerm("VERYLEFT", -45, 0.0));//, true));
    changeHeading->addTerm(new fl::TriangularTerm("LEFT", -22.5, 0.0));
    changeHeading->addTerm(new fl::TriangularTerm("NOCHANGE", -22.5, 22.5));
    changeHeading->addTerm(new fl::TriangularTerm("RIGHT", 0.0, 22.5));
    changeHeading->addTerm(new fl::TriangularTerm("VERYRIGHT", 0.0, 45));//, false));    
    engine->addOutputLVar(changeHeading);
    
    block = new fl::RuleBlock();
    //PlaneDist is VERYCLOSE
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is LEFT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS and PlaneDist is VERYCLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS and PlaneDist is VERYCLOSE then ChangeInHeading is LEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS and PlaneDist is VERYCLOSE then ChangeInHeading is LEFT", *engine));
    
    //PlaneDist is CLOSE
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is RIGHT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is LEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG and PlaneDist is CLOSE then ChangeInHeading is LEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG and PlaneDist is CLOSE then ChangeInHeading is LEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG and PlaneDist is CLOSE then ChangeInHeading is LEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS and PlaneDist is CLOSE then ChangeInHeading is LEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS and PlaneDist is CLOSE then ChangeInHeading is NOCHANGE", *engine));
    
    //PlaneDist is FAR
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is NOCHANGE", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is NOCHANGE", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is RIGHT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is NOCHANGE", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is LEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is NOCHANGE", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG and PlaneDist is FAR then ChangeInHeading is LEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG and PlaneDist is FAR then ChangeInHeading is LEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG and PlaneDist is FAR then ChangeInHeading is LEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS and PlaneDist is FAR then ChangeInHeading is LEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS and PlaneDist is FAR then ChangeInHeading is NOCHANGE", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS and PlaneDist is FAR then ChangeInHeading is NOCHANGE", *engine));
    
    //PlaneDist is VERYFAR
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYNEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is RIGHT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is RIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is NEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWNEG and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYRIGHT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is LOWPOS and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is VERYLEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is LEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is POS and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", *engine));
    
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYNEG and PlaneDist is VERYFAR then ChangeInHeading is LEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is NEG and PlaneDist is VERYFAR then ChangeInHeading is LEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWNEG and PlaneDist is VERYFAR then ChangeInHeading is LEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is LOWPOS and PlaneDist is VERYFAR then ChangeInHeading is LEFT", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is POS and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", *engine));
    block->addRule(new fl::MamdaniRule("if OurBearingAngle is VERYPOS and TheirBearingAngle is VERYPOS and PlaneDist is VERYFAR then ChangeInHeading is NOCHANGE", *engine)); 
    
    engine->addRuleBlock(block); 
    
}

double AU_UAV_ROS::OursAndTheirs2::process(double distance, double ourAngle, double theirAngle){
	
	this->distanceBetweenPlanes->setInput(distance);
	this->ourBearingAngle->setInput(ourAngle);
	this->theirBearingAngle->setInput(theirAngle);
        this->engine->process();
        return this->changeHeading->output().defuzzify();

}
