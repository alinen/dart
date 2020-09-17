#include "Anthropometrics.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <utility>
#include <algorithm>
#include "AJoint.h"

using namespace glm;

// initialize constants: alternate models can be tried by overriding these values
Anthropometrics::Anthropometrics()
{
    // From http://www.rush.edu/rumc/page-1108048103230.html
    _height2weightB = -61.7542; // y-intercept
    _height2weightM = 73.0766; // slope

    // From Winter Biomechanics Book, Fourth Edition, 2007, Chpt 4
    _mass[Hand] = 0.006;
    _mass[Forearm] = 0.016;
    _mass[UpperArm] = 0.028;
    _mass[Foot] = 0.0145;
    _mass[Shank] = 0.0465;
    _mass[Thigh] = 0.1;
    _mass[HeadNeck] = 0.081;
    _mass[Trunk] = 0.497;

    _comProximal[Hand] = 0.506;
    _comProximal[Forearm] = 0.430;
    _comProximal[UpperArm] = 0.436;
    _comProximal[Foot] = 0.5;
    _comProximal[Shank] = 0.433;
    _comProximal[Thigh] = 0.433;
    _comProximal[HeadNeck] = 0.5; 
    _comProximal[Trunk] = 0.5;

    _rogProximal[Hand] = 0.297; // radius of gyration
    _rogProximal[Forearm] = 0.303;
    _rogProximal[UpperArm] = 0.322;
    _rogProximal[Foot] = 0.475;
    _rogProximal[Shank] = 0.302;
    _rogProximal[Thigh] = 0.323;
    _rogProximal[HeadNeck] = 0.495; 
    _rogProximal[Trunk] = 0.5;

    _HandDensityB = -0.44;
    _HandDensityM = 1.5;
    _ForearmDensityB = -0.0675;
    _ForearmDensityM = 1.125;
    _UpperArmDensityB = 0.4225;
    _UpperArmDensityM = 0.625;
    _FootDensityB = 0.3933;
    _FootDensityM = 0.6667;
    _ShankDensityB = 0.555;
    _ShankDensityM = 0.5;
    _ThighDensityB = 0.3533;
    _ThighDensityM = 0.6667;
    _HeadNeckDensityB = 1.11; //constant for all body densities
    _HeadNeckDensityM = 0;
    _TrunkDensityB = 1.03; // constant
    _TrunkDensityM = 0.0; 

    /* TODO: Move to unit test
    double wgt = 190 * WeightTable[LB][KG];
    double height = 73 * MoUnitTable[INCH][M];
    double d = GetBodyDensity(height, wgt);
    double mass = wgt; // wgt = mass * gravity
    double volume = mass / d;
    double test1 =  GetDensity(Hand, d); 
    double test2 =  GetDensity(Forearm, d); 
    double test3 =  GetDensity(UpperArm, d); 
    double test4 =  GetDensity(Foot, d); 
    double test5 =  GetDensity(Shank, d); 
    double test6 =  GetDensity(Thigh, d); 
    double test7 =  GetDensity(HeadNeck, d); 
    double test8 =  GetDensity(Trunk, d); 

    std::vector<double> volumes(8,0);
    volumes[0] =  2*(GetMass(Hand, mass) / test1); // note: density * vol = mass
    volumes[1] =  2*(GetMass(Forearm, mass) / test2); // note: density * vol = mass
    volumes[2] =  2*(GetMass(UpperArm, mass) / test3); // note: density * vol = mass
    volumes[3] =  2*(GetMass(Foot, mass) / test4); // note: density * vol = mass
    volumes[4] =  2*(GetMass(Shank, mass) / test5); // note: density * vol = mass
    volumes[5] =  2*(GetMass(Thigh, mass) / test6); // note: density * vol = mass
    volumes[6] =  GetMass(HeadNeck, mass) / test7; // note: density * vol = mass
    volumes[7] =  GetMass(Trunk, mass) / test8; // note: density * vol = mass

    double sum = 0;
    for (int i = 0; i < volumes.size(); i++) sum += volumes[i];
    std::cout << sum << " " << volume << std::endl;
    */

    MB_Mapping["Hips"] = BodyData(Trunk, 0.0);
    MB_Mapping["LeftUpLeg"] = BodyData(Trunk, 0.15);
    MB_Mapping["LeftLeg"] = BodyData(Thigh, 1.0);
    MB_Mapping["LeftFoot"] = BodyData(Shank, 1.0);
    MB_Mapping["LeftToeBase"] = BodyData(Foot, 0.50);
    MB_Mapping["LeftToeBaseSite"] = BodyData(Foot, 0.50);

    MB_Mapping["RightUpLeg"] = BodyData(Trunk, 0.15);
    MB_Mapping["RightLeg"] = BodyData(Thigh, 1.0);
    MB_Mapping["RightFoot"] = BodyData(Shank, 1.0);
    MB_Mapping["RightToeBase"] = BodyData(Foot, 0.50);
    MB_Mapping["RightToeBaseSite"] = BodyData(Foot, 0.50);

    MB_Mapping["Spine"] = BodyData(Trunk, 0.1);
    MB_Mapping["Spine1"] = BodyData(Trunk, 0.2);
    MB_Mapping["Spine2"] = BodyData(Trunk, 0.2);
    MB_Mapping["Spine3"] = BodyData(Trunk, 0.0);
    MB_Mapping["Spine4"] = BodyData(Trunk, 0.0);
    MB_Mapping["Neck"] = BodyData(HeadNeck, 0.1);
    MB_Mapping["Head"] = BodyData(HeadNeck, 0.1);
    MB_Mapping["HeadSite"] = BodyData(HeadNeck, 0.8);

    MB_Mapping["LeftShoulder"] = BodyData(Trunk, 0.05);
    MB_Mapping["LeftArm"] = BodyData(Trunk, 0.05);
    MB_Mapping["LeftForeArm"] = BodyData(UpperArm, 1.0);
    MB_Mapping["LeftHand"] = BodyData(Forearm, 1.0);
    MB_Mapping["LeftHandSite"] = BodyData(Hand, 1.0);

    MB_Mapping["RightShoulder"] = BodyData(Trunk, 0.05);
    MB_Mapping["RightArm"] = BodyData(Trunk, 0.05);
    MB_Mapping["RightForeArm"] = BodyData(UpperArm, 1.0);
    MB_Mapping["RightHand"] = BodyData(Forearm, 1.0);
    MB_Mapping["RightHandSite"] = BodyData(Hand, 1.0);

    CMU_Mapping["root"]=BodyData(Trunk, 0.0);
    CMU_Mapping["lowerback"]=BodyData(Trunk, 0.0); // no length for lowerback
    CMU_Mapping["upperback"]=BodyData(Trunk, 0.3);
    CMU_Mapping["thorax"]=BodyData(Trunk, 0.3);
    CMU_Mapping["lowerneck"]=BodyData(Trunk, 0.2);
    CMU_Mapping["upperneck"]=BodyData(Trunk, 0.05);
    CMU_Mapping["head"]=BodyData(HeadNeck, 0.2);
    CMU_Mapping["headSite"]=BodyData(HeadNeck, 0.75);

    CMU_Mapping["lfemur"]= BodyData(Thigh, 1.0);
    CMU_Mapping["ltibia"]=BodyData(Shank, 1.0);
    CMU_Mapping["lfoot"]=BodyData(Foot, 0.8);
    CMU_Mapping["ltoes"]=BodyData(Foot, 0.1);
    CMU_Mapping["ltoesSite"]=BodyData(Foot, 0.1);

    CMU_Mapping["rfemur"]=BodyData(Thigh, 1.0);
    CMU_Mapping["rtibia"]=BodyData(Shank, 1.0);
    CMU_Mapping["rfoot"]=BodyData(Foot, 0.8);
    CMU_Mapping["rtoes"]=BodyData(Foot, 0.1);
    CMU_Mapping["rtoesSite"]=BodyData(Foot, 0.1);

    CMU_Mapping["lclavicle"]=BodyData(Trunk, 0.1);
    CMU_Mapping["lhumerus"]=BodyData(UpperArm, 1.0);
    CMU_Mapping["lradius"]=BodyData(Forearm, 1.0);
    CMU_Mapping["lwrist"]=BodyData(Hand, 0.6);
    CMU_Mapping["lhand"]=BodyData(Hand, 0.2);
    CMU_Mapping["lthumb"]=BodyData(Hand, 0.05);
    CMU_Mapping["lthumbSite"]=BodyData(Hand, 0.05);
    CMU_Mapping["lfingers"]=BodyData(Hand, 0.05);
    CMU_Mapping["lfingersSite"]=BodyData(Hand, 0.05);

    CMU_Mapping["rclavicle"]=BodyData(Trunk, 0.1);
    CMU_Mapping["rhumerus"]=BodyData(UpperArm, 1.0);
    CMU_Mapping["rradius"]=BodyData(Forearm, 1.0);
    CMU_Mapping["rwrist"]=BodyData(Hand, 0.6);
    CMU_Mapping["rhand"]=BodyData(Hand, 0.2);
    CMU_Mapping["rthumb"]=BodyData(Hand, 0.05);
    CMU_Mapping["rthumbSite"]=BodyData(Hand, 0.05);
    CMU_Mapping["rfingers"]=BodyData(Hand, 0.05);
    CMU_Mapping["rfingersSite"]= BodyData(Hand, 0.05);

    // TODO: ASL mapping
}

Anthropometrics::~Anthropometrics()
{
}

void Anthropometrics::init(const ASkeleton& inSkeleton, double factor) 
{
    ASkeleton skeleton = inSkeleton;

    // Zero out joints
    skeleton.getRoot()->setLocalTranslation(vec3(0,0,0));
    for (int i = 0; i < skeleton.getNumJoints(); i++)
    {
        AJoint* joint = skeleton.getByID(i); 
        joint->setLocalRotation(IdentityQ);
        joint->setLocalTranslation(joint->getLocalTranslation() * (float) factor);
    }
    skeleton.fk(); 

    // Find up direction
    int upidx = 2; 
    vec3 dim = getDimensions(skeleton);
    if (dim[1] > dim[2]) upidx = 1;

    double height = estimateHeight(skeleton, upidx);
    double mass = getWeight(height);
    //setupBoneShapes(skeleton, height, mass);
    std::cout << "Height: " << height << " " << mass << std::endl; 
}

// cast insensitive search for joint containing any name from names
AJoint* findJoint(const ASkeleton& skeleton, const std::vector<std::string> names)
{
   for (unsigned int j = 0; j < names.size(); j++)
   {
      std::string name = names[j];
      std::transform(name.begin(), name.end(), name.begin(), ::tolower);
      //std::cout << name << std::endl;

      for (int i = 0; i < skeleton.getNumJoints(); i++)
      {
         AJoint* current = skeleton.getByID(i);
         std::string jointName = current->getName();
         std::transform(jointName.begin(), jointName.end(), jointName.begin(), ::tolower);
         
         //std::cout << "JOINT: " << jointName << std::endl;
         if (jointName.find(name) != std::string::npos)
         {
            return current;
         }
      }
   }


   return 0;
}

// cast insensitive search for end effector of joint containing "name"
AJoint* findEndEffector(const ASkeleton& skeleton, const std::string& name)
{
   std::string lName = name;
   std::transform(name.begin(), name.end(), lName.begin(), ::tolower);
   for (int i = 0; i < skeleton.getNumJoints(); i++)
   {
      AJoint* current = skeleton.getByID(i);
      std::string jointName = current->getName();
      std::transform(jointName.begin(), jointName.end(), jointName.begin(), ::tolower);
      //std::cout << lName << " " << jointName << std::endl;
      if (jointName.find(lName) != std::string::npos)
      {
         // get end effector
         AJoint* child = current;
         while (child->getNumChildren() > 0)
         {
            child = child->getChildAt(0);
         }
         return child;
      }
   } 

   return 0;
}

double Anthropometrics::estimateHeight(const ASkeleton& skeleton, int upidx) const
{
    AJoint* head = findEndEffector(skeleton, "Head");
    if (!head) std::cout << "Cannot find head joint\n";


    AJoint* foot = findJoint(skeleton, {"Heel", "Toe", "Foot"});
    if (!foot) std::cout << "Cannot find left foot joint\n";

    double min = 99999999.0;
    double max = -99999999.0;
    if (head && foot)
    {
        min = foot->getGlobalTranslation()[upidx];
        max = head->getGlobalTranslation()[upidx];
    }
    else
    {
        // look for max vertical distance
       for (int i = 0; i < skeleton.getNumJoints(); i++)
       {
            AJoint* joint = skeleton.getByID(i);
            vec3 pos = joint->getGlobalTranslation();
            min = std::min<double>(pos[upidx], min);
            max = std::max<double>(pos[upidx], max);
        }
    }

    return max - min;
}

vec3 Anthropometrics::getDimensions(const ASkeleton& skeleton) const
{
    vec3 min(9999999999.0,9999999999.0,9999999999.0);
    vec3 max(-9999999999.0,-9999999999.0,-9999999999.0);
    for (int i = 1; i < skeleton.getNumJoints(); i++)
    {
        vec3 pos = skeleton.getByID(i)->getGlobalTranslation();
        min[0] = std::min<float>(min[0], pos[0]);
        min[1] = std::min<float>(min[1], pos[1]);
        min[2] = std::min<float>(min[2], pos[2]);

        max[0] = std::max<float>(max[0], pos[0]);
        max[1] = std::max<float>(max[1], pos[1]);
        max[2] = std::max<float>(max[2], pos[2]);
    }

    vec3 dim(0,0,0);
    dim = max - min;
    return dim; 
}

double Anthropometrics::getWeight(double height)
{
    return _height2weightB + _height2weightM * height;
}

double Anthropometrics::getBodyDensity(double height, double weight)
{
    // From Winter Biomechanics Book, Fourth Edition, 2007, Chpt 4
    // via Contini 1972, Body Segment Parameters, Part II
    double c = height / pow(weight, 0.333333333333);
    return 0.69 + 0.9 * c; 
}

double Anthropometrics::getMass(Anthropometrics::Segment s, double totalMass)
{
    return _mass[s] * totalMass;
}

double Anthropometrics::getCOMProximal(Anthropometrics::Segment s)
{
    return _comProximal[s];
}

double Anthropometrics::getCOMDistal(Anthropometrics::Segment s)
{
    return 1.0 - _comProximal[s];
}

double Anthropometrics::getDensity(Anthropometrics::Segment s, double bodyDensity)
{
    switch(s)
    {
    case Hand: return _HandDensityB + _HandDensityM * bodyDensity;
    case Forearm: return _ForearmDensityB + _ForearmDensityM * bodyDensity;
    case UpperArm: return _UpperArmDensityB + _UpperArmDensityM * bodyDensity;
    case Foot: return _FootDensityB + _FootDensityM * bodyDensity;
    case Shank: return _ShankDensityB + _ShankDensityM * bodyDensity;
    case Thigh: return _ThighDensityB + _ThighDensityM * bodyDensity;
    case HeadNeck: return _HeadNeckDensityB + _HeadNeckDensityM * bodyDensity;
    case Trunk: return _TrunkDensityB + _TrunkDensityM * bodyDensity;
    }

    return 1.0; 
}

/*
#define DFLTLIM Limits(-90*Deg2Rad, 90*Deg2Rad)
void Anthropometrics::initJointDOFAxes(ASkeleton& skeleton)
{
    for (int i = 0; i < skeleton.getNumJoints(); i++)
    {
        Joint* joint = skeleton.getByID(i);
        joint->setLocalRotation(identity3D);
    }
    skeleton.UpdateFK();

    //AddBallJoint("lhumerus", DFLTLIM, DFLTLIM, DFLTLIM);
    //AddBallJoint("rhumerus", DFLTLIM, DFLTLIM, DFLTLIM);
    //AddSwingJoint("rfoot", DFLTLIM, DFLTLIM);
    addHingeJoint("rtibia", Limits(-5 * Deg2Rad, 155 * Deg2Rad));
    addHingeJoint("ltibia", Limits(-5 * Deg2Rad, 155 * Deg2Rad));
    addHingeJoint("lradius", Limits(-5 * Deg2Rad, 140 * Deg2Rad));
    addHingeJoint("rradius", Limits(-5 * Deg2Rad, 140 * Deg2Rad));
    addHingeJoint("rfoot", Limits(-160 * Deg2Rad, 10 * Deg2Rad));
    addHingeJoint("lfoot", Limits(-160 * Deg2Rad, 10 * Deg2Rad));

    // The following assumes that the skeleton is in a zero pose
    for (int i = 0; i < skeleton.GetNumJoints(); i++)
    {
        Joint* joint = skeleton.GetJointByID(i);
        if (!joint->GetParent()) continue;
        if (joint->GetNumChildren() == 0) continue;

        vec3 forwardAxis = joint->GetGlobalRotation().Transpose() * vec3(0,0,1);

        vec3 twistAxis = joint->GetChildAt(0)->GetLocalTranslation();
        twistAxis.Normalize();

        vec3 rightAxis = twistAxis.Cross(forwardAxis); 
        rightAxis.Normalize();

        forwardAxis = rightAxis.Cross(twistAxis);
        forwardAxis.Normalize();

        if (isHinge(joint))
        {
            jointDOFs[joint->GetName()][0].axis = rightAxis;
        }
        else if (isSwing(joint))
        {
            jointDOFs[joint->GetName()][0].axis = forwardAxis;
            jointDOFs[joint->GetName()][1].axis = rightAxis;
        }
        else if (isBall(joint))
        {
            jointDOFs[joint->GetName()][0].axis = twistAxis;
            jointDOFs[joint->GetName()][1].axis = forwardAxis;
            jointDOFs[joint->GetName()][2].axis = rightAxis;
        }
    }

    // Joint limits (MB_Mapping for Matt model: ASN TODO: support all our models)
}

bool Anthropometrics::isBall(AJoint* joint)
{
    if (jointDOFs.find(joint->GetName()) != jointDOFs.end())
    {
        return jointDOFs[joint->GetName()].size() == 3;
    }
    return false;
}

bool Anthropometrics::isSwing(AJoint* joint)
{
    if (jointDOFs.find(joint->GetName()) != jointDOFs.end())
    {
        return jointDOFs[joint->GetName()].size() == 2;
    }
    return false;
}

bool Anthropometrics::isHinge(AJoint* joint)
{
    if (jointDOFs.find(joint->GetName()) != jointDOFs.end())
    {
        return jointDOFs[joint->GetName()].size() == 1;
    }
    return false;
}

void Anthropometrics::addBallJoint(const std::string& name, Limits twist, Limits swing, Limits phi)
{
    jointDOFs[name].clear();
    jointDOFs[name].push_back(DOF(twist)); 
    jointDOFs[name].push_back(DOF(swing)); 
    jointDOFs[name].push_back(DOF(phi)); 
}

void Anthropometrics::addSwingJoint(const std::string& name, Limits swing, Limits phi)
{
    jointDOFs[name].clear();
    jointDOFs[name].push_back(DOF(swing)); 
    jointDOFs[name].push_back(DOF(phi)); 
}

void Anthropometrics::addHingeJoint(const std::string& name, Limits phi)
{
    jointDOFs[name].clear();
    jointDOFs[name].push_back(DOF(phi)); 
}

Quaternion Anthropometrics::getBallJointQuaternion(AJoint* joint, double twistRad, double swingRad, double phiRad)
{
    if (!isBall(joint))
    {
        std::cout << "ERROR: " << joint->GetName().c_str() << " not a ball joint\n";
    }

    vec3 twistAxis = jointDOFs[joint->GetName()][0].axis;
    vec3 forwardAxis = jointDOFs[joint->GetName()][1].axis;
    vec3 rightAxis = jointDOFs[joint->GetName()][2].axis;

    Quaternion twistQ;   // Twist angle
    Quaternion swingQ;   // swing angle
    twistQ.FromAxisAngle(twistAxis, twistRad);
    vec3 swingDirection = cos(swingRad) * rightAxis + sin(swingRad) * forwardAxis;
    swingQ.FromAxisAngle(swingDirection, phiRad);
    Quaternion rot = swingQ * twistQ;
    return rot;
}

Quaternion Anthropometrics::getSwingJointQuaternion(AJoint* joint, double swingRad, double phiRad)
{
    if (!isSwing(joint))
    {
        std::cout << "ERROR: " << joint->GetName().c_str() << " not a swing joint\n";
    }

    vec3 forwardAxis = jointDOFs[joint->GetName()][0].axis;
    vec3 rightAxis = jointDOFs[joint->GetName()][1].axis;

    Quaternion swingQ;   // swing angle
    vec3 swingDirection = cos(swingRad) * rightAxis + sin(swingRad) * forwardAxis;
    swingQ.FromAxisAngle(swingDirection, phiRad);
    return swingQ;
}

Quaternion Anthropometrics::getHingeJointQuaternion(AJoint* joint, double phiRad)
{
    if (!isHinge(joint))
    {
        std::cout << "ERROR: " << joint->GetName().c_str() << " not a hinge joint\n";
    }

    vec3 rightAxis = jointDOFs[joint->GetName()][0].axis;

    Quaternion hingeQ;   // hinge angle
    hingeQ.FromAxisAngle(rightAxis, phiRad);
    return hingeQ;
}

void Anthropometrics::getBallJointDecomposition(AJoint* joint, double& twistRad, double& swingRad, double& phiRad)
{
    if (!isBall(joint))
    {
        std::cout << "ERROR: " << joint->GetName().c_str() << " not a ball joint\n";
    }

    Quaternion q = joint->GetLocalRotation().ToQuaternion();
    vec3 ra(q.X(), q.Y(), q.Z());

    vec3 twistAxis = jointDOFs[joint->GetName()][0].axis;
    vec3 forwardAxis = jointDOFs[joint->GetName()][1].axis;
    vec3 rightAxis = jointDOFs[joint->GetName()][2].axis;

    //vec3 p = projection ( ra, twistAxis); // project to get parallel component of ra onto twistAxis
    vec3 p = Dot(ra, twistAxis) * twistAxis;
    Quaternion twist(q.W(), p[0], p[1], p[2]);
    twist.Normalize();
    if (Dot(p, twistAxis) < 0)
    {
        twist = -twist;
    }
    Quaternion swing = q * twist.Inverse();
    //std::cout << swing << std::endl;

    twistRad = 2 * acos(q.W());
    phiRad = 2 * acos(swing.W());

    vec3 swingAxis; double angle;
    swing.ToAxisAngle(swingAxis, angle);

    double tmp1 = Dot(swingAxis, forwardAxis);
    double tmp2 = Dot(swingAxis, rightAxis);
    //vec3 test2 = tmp1 * forwardAxis + tmp2 * rightAxis;
    swingRad = atan2(tmp1, tmp2);
}

void Anthropometrics::getSwingJointDecomposition(AJoint* joint, double& swingRad, double& phiRad)
{
    if (!isSwing(joint))
    {
        std::cout << "ERROR: " << joint->GetName().c_str() << " not a swing joint\n";
    }
}

void Anthropometrics::getHingeJointDecomposition(AJoint* joint, double& phiRad)
{
    if (!isHinge(joint))
    {
        std::cout << "ERROR: " << joint->GetName().c_str() << " not a hinge joint\n";
    }

    vec3 rightAxis = jointDOFs[joint->GetName()][0].axis;
}
*/

/*

static std::map<std::string, std::pair<float,float>> MASS_EQNS; // From Robbin& Wu 2003
static vec3 GRAVITY;
std::map<std::string, std::pair<float,float>> InvDynBVH::MASS_EQNS;

void SetupMassEqns()
{
    if (InvDynBVH::MASS_EQNS.size() > 0) return;

    InvDynBVH::MASS_EQNS["Foot"] = std::pair<float,float>(0.007, 0.4655);
    InvDynBVH::MASS_EQNS["Shank"] = std::pair<float,float>(0.038, 0.3487);
    InvDynBVH::MASS_EQNS["Thigh"] = std::pair<float,float>(0.1166, -1.0759);
    InvDynBVH::MASS_EQNS["Hand"] = std::pair<float,float>(0.0055, 0.0675);
    InvDynBVH::MASS_EQNS["LowerArm"] = std::pair<float,float>(0.0191, -0.1729);
    InvDynBVH::MASS_EQNS["UpperArm"] = std::pair<float,float>(0.0277, -0.0324);
    InvDynBVH::MASS_EQNS["Torso"] = std::pair<float,float>(0.5468, -5.1209);
    InvDynBVH::MASS_EQNS["Head"] = std::pair<float,float>(0.0303, 2.4772);
}

float GetMass(const std::string& name, float tmass)
{
    SetupMassEqns();
    std::pair<float,float> ab = InvDynBVH::MASS_EQNS[name];
    return ab.first * tmass + ab.second;
}
*/
