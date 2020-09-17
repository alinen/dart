#ifndef Anthropometrics_H_
#define Anthropometrics_H_

#include <map>
#include <string>
#include "ATransform.h"
#include "ASkeleton.h"
#include "AGLM.h"

class AJoint;

// Lengths are in meters, weight in kgs, density is kgs/litre
class Anthropometrics
{
public:
    Anthropometrics();
    virtual ~Anthropometrics();

    // scale factor: conversion factor needed for converting the meters
    void init(const ASkeleton& skeleton, double factor = 1.0);

    // Average weight and density
    double getWeight(double height);
    double getBodyDensity(double height, double weight);

    // Segment properties
    enum Segment {Hand, Forearm, UpperArm, Foot, Shank, Thigh, HeadNeck, Trunk};
    double getMass(Segment s, double totalMass);
    double getCOMProximal(Segment s);
    double getCOMDistal(Segment s);
    double getDensity(Segment s, double bodyDensity);
    glm::vec3 getDimensions(const ASkeleton& skeleton) const;        
    double estimateHeight(const ASkeleton& skeleton, int upidx) const;

    typedef std::pair<Segment, double> BodyData;
    std::map<std::string, BodyData> CMU_Mapping;
    std::map<std::string, BodyData> MB_Mapping;

    // bounding spheres for self-collision testing
    typedef std::pair<glm::vec3, double> BSphere;
    std::map<std::string, BSphere> bspheres;

public:
    typedef std::pair<double,double> Limits;  // min,max for joint in radians
    class DOF 
    {
    public:
        DOF(Limits l) : limits(l) {}
        Anthropometrics::Limits limits; 
        glm::vec3 axis; // depending on the joint type, this can mean a direction vector, or a rotation vector
    };
    // Depending on the number of DOFs, a joint could be a ball, swing, or hinge joint
    std::map<std::string, std::vector<DOF>> jointDOFs; 

    //bool isBall(Joint* joint);
    //bool isSwing(Joint* joint);
    //bool isHinge(Joint* joint);
    //void addBallJoint(const std::string& name, Limits twist, Limits swing, Limits phi);
    //void addSwingJoint(const std::string& name, Limits swing, Limits phi);
    //void addHingeJoint(const std::string& name, Limits phi);

    //Quaternion getBallJointQuaternion(Joint* joint, double twistRad, double swingRad, double phiRad);
    //Quaternion getSwingJointQuaternion(Joint* joint, double swingRad, double phiRad);
    //Quaternion getHingeJointQuaternion(Joint* joint, double phiRad);

    //void getBallJointDecomposition(Joint* joint, double& twistRad, double& swingRad, double& phiRad);
    //void getSwingJointDecomposition(Joint* joint, double& swingRad, double& phiRad);
    //void getHingeJointDecomposition(Joint* joint, double& phiRad);

    //void initJointDOFAxes(Skeleton& skeleton);

protected:

    double _height2weightB; // y-intercept
    double _height2weightM; // slope

    double _HandDensityB;
    double _HandDensityM;
    double _ForearmDensityB;
    double _ForearmDensityM;
    double _UpperArmDensityB;
    double _UpperArmDensityM;
    double _FootDensityB;
    double _FootDensityM;
    double _ShankDensityB;
    double _ShankDensityM;
    double _ThighDensityB;
    double _ThighDensityM;
    double _HeadNeckDensityB;
    double _HeadNeckDensityM;
    double _TrunkDensityB;
    double _TrunkDensityM;

    std::map<Segment, double> _mass;
    std::map<Segment, double> _comProximal;
    std::map<Segment, double> _rogProximal;
};

#endif
