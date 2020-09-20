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

    // mappings from joints to metrics
    enum Segment {Hand, Forearm, UpperArm, Foot, Shank, Thigh, HeadNeck, Trunk};
    typedef std::pair<Segment, double> BodyData;
    typedef std::map<std::string, BodyData> BodyMap;
    BodyMap CMU_Mapping;
    BodyMap MB_Mapping;
    BodyMap ASL_Mapping;

    // scale factor: conversion factor needed for converting the meters
    void init(const ASkeleton& skeleton, double factor = 1.0);
    void init(const ASkeleton& skeleton, double height, double weight, double factor = 1.0);

    // Average weight and density
    double getWeight(double height);
    double getBodyDensity(double height, double weight);

    // Segment properties
    double getMass(Segment s, double totalMass);
    double getCOMProximal(Segment s);
    double getCOMDistal(Segment s);
    double getDensity(Segment s, double bodyDensity);

    // joint properties
    double getRadius(const std::string& name);
    double getMass(const std::string& name);
    double getDensity(const std::string& name);
    double getCOMProximal(const std::string& name);

    // skeleton properties
    glm::vec3 getDimensions(const ASkeleton& skeleton) const;        
    double estimateHeight(const ASkeleton& skeleton, int upidx) const;

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

   void setupBoneShapes(const ASkeleton& skeleton, double h, double totalMass);
   //void setupBoneShapes(const ASkeleton& skeleton, const BodyMap& map, double h, double totalMass);
   void computeMass(const ASkeleton& skeleton);

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

    // body shape parameters
    double _height;
    double _totalMass;
    std::map<std::string,double> _aspx; // radius of bone
    std::map<std::string,double> _jmass; // masses
    std::map<std::string,double> _jmassCombo; // masses as sum of children
    std::map<std::string,double> _jdensity; // masses as sum of children
    // COM offset from start of each bone, e.g. comPos = comOffset * joint->LocalTranslation()
    std::map<std::string,double> _comOffset; 

    std::map<std::string,glm::mat3> _jI; // momments of inertia
    std::map<std::string,glm::mat3> _jIlocal; // momments of inertia

};

#endif
