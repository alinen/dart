#include "Anthropometrics.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <utility>
#include <algorithm>
#include "AJoint.h"

using namespace glm;

enum MoUnit {MM,CM,M,INCH,FT};  // Motion units
enum Weight {KG,LB}; // Weight units

std::map<MoUnit, std::map<MoUnit,float>> initConversions()
{
    std::map<MoUnit, std::map<MoUnit,float>> m;
    m[MM][MM] = 1.0;    m[MM][CM] = 0.1;    m[MM][M] = 0.001;    m[MM][INCH] = 0.0393701;m[MM][FT] = 0.00328084;
    m[CM][MM] = 10.0;   m[CM][CM] = 1.0;    m[CM][M] = 0.01;     m[CM][INCH] = 0.393701; m[CM][FT] = 0.0328084;
    m[M][MM] = 1000.0;  m[M][CM] = 100.0;   m[M][M] = 1.0;       m[M][INCH] = 39.3701;   m[M][FT] = 3.28084;
    m[INCH][MM] = 25.4; m[INCH][CM] = 2.54; m[INCH][M] = 0.0254; m[INCH][INCH] = 1.0;    m[INCH][FT] = 0.0833333;
    m[FT][MM] = 304.8;  m[FT][CM] = 30.48;  m[FT][M] = 0.3048;   m[FT][INCH] = 12.0;     m[FT][FT] = 1.0;
    return m;
}

std::map<Weight, std::map<Weight,float>> initWeightConversions()
{
    std::map<Weight, std::map<Weight,float>> m;
    m[LB][KG] = 0.453592;
    m[KG][LB] = 2.20462;
    return m;
}

std::map<MoUnit, std::map<MoUnit,float>> MoUnitTable = initConversions();
std::map<Weight, std::map<Weight,float>> WeightTable = initWeightConversions();

//Anthropometrics::BodyMap Anthropometrics::CMU_Mapping = Anthropometrics::BodyMap();
//Anthropometrics::BodyMap Anthropometrics::MB_Mapping = Anthropometrics::BodyMap();
//Anthropometrics::BodyMap Anthropometrics::ASL_Mapping = Anthropometrics::BodyMap();

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

    // TODO: ASL mapping fix this so it works with physics!
    ASL_Mapping["Hips"] = BodyData(Trunk, 0.0);
    ASL_Mapping["LeftUpLeg"] = BodyData(Trunk, 0.2);
    ASL_Mapping["LeftLeg"] = BodyData(Thigh, 1.0);
    ASL_Mapping["LeftFoot"] = BodyData(Shank, 1.0);
    ASL_Mapping["LeftFootHeel"] = BodyData(Foot, 0.1);
    ASL_Mapping["LeftHeelOutside"] = BodyData(Foot, 0.1);
    ASL_Mapping["LeftFootHeelSite"] = BodyData(Foot, 0.4);
    ASL_Mapping["LeftHeelOutsideSite"] = BodyData(Foot, 0.4);

    ASL_Mapping["RightUpLeg"] = BodyData(Trunk, 0.2);
    ASL_Mapping["RightLeg"] = BodyData(Thigh, 1.0);
    ASL_Mapping["RightFoot"] = BodyData(Shank, 1.0);
    ASL_Mapping["RightFootHeel"] = BodyData(Foot, 0.1);
    ASL_Mapping["RightHeelOutside"] = BodyData(Foot, 0.1);
    ASL_Mapping["RightFootHeelSite"] = BodyData(Foot, 0.4);
    ASL_Mapping["RightHeelOutsideSite"] = BodyData(Foot, 0.4);

    ASL_Mapping["Spine"] = BodyData(Trunk, 0.15);
    ASL_Mapping["Spine1"] = BodyData(Trunk, 0.15);
    ASL_Mapping["Neck"] = BodyData(Trunk, 0.1);
    ASL_Mapping["Head"] = BodyData(HeadNeck, 0.2);
    ASL_Mapping["HeadSite"] = BodyData(HeadNeck, 0.8);

    ASL_Mapping["LeftShoulder"] = BodyData(Trunk, 0.05);
    ASL_Mapping["LeftArm"] = BodyData(Trunk, 0.05);
    ASL_Mapping["LeftForeArm"] = BodyData(UpperArm, 1.0);
    ASL_Mapping["LeftHand"] = BodyData(Forearm, 1.0);
    ASL_Mapping["LeftmiddleA"] = BodyData(Hand, 1.0);

    ASL_Mapping["RightShoulder"] = BodyData(Trunk, 0.05);
    ASL_Mapping["RightArm"] = BodyData(Trunk, 0.05);
    ASL_Mapping["RightForeArm"] = BodyData(UpperArm, 1.0);
    ASL_Mapping["RightHand"] = BodyData(Forearm, 1.0);
    ASL_Mapping["RightmiddleA"] = BodyData(Hand, 1.0);

    // TODO: Move to unit test
    // Test mapping: all body parts should sum to 1
    std::map<Segment, float> sum = 
    { 
      {Hand,0.0}, 
      {Forearm,0.0}, 
      {UpperArm,0.0}, 
      {Foot,0.0}, 
      {Shank,0.0}, 
      {Thigh,0.0}, 
      {HeadNeck,0.0}, 
      {Trunk,0.0}
    };
    for (auto it = ASL_Mapping.begin(); it != ASL_Mapping.end(); it++)
    {
      BodyData data = it->second;
      sum[data.first] += data.second;
    }
    for (auto it = sum.begin(); it != sum.end(); it++)
    {
      std::cout << it->first << " " << it->second << std::endl;
    }
}

Anthropometrics::~Anthropometrics()
{
}

void ScaleSkeleton(ASkeleton& skeleton, double factor)
{
    // Zero out joints
    skeleton.getRoot()->setLocalTranslation(vec3(0,0,0));
    for (int i = 0; i < skeleton.getNumJoints(); i++)
    {
        AJoint* joint = skeleton.getByID(i); 
        joint->setLocalRotation(IdentityQ);
        joint->setLocalTranslation(joint->getLocalTranslation() * (float) factor);
    }
    skeleton.fk(); 
}

void Anthropometrics::init(const ASkeleton& inSkeleton, double factor) 
{
    ASkeleton skeleton = inSkeleton;
    ScaleSkeleton(skeleton, factor);

    // Find up direction
    int upidx = 2; 
    vec3 dim = getDimensions(skeleton);
    if (dim[1] > dim[2]) upidx = 1;

    double height = estimateHeight(skeleton, upidx);
    double mass = getWeight(height);
    std::cout << "height: " << height << " weight: " << mass << std::endl; 
    setupBoneShapes(skeleton, height, mass);

    // TODO: Move to unit test
/*
    double d = getBodyDensity(height, mass);
    double volume = mass / d;
    double test1 =  getDensity(Hand, d); 
    double test2 =  getDensity(Forearm, d); 
    double test3 =  getDensity(UpperArm, d); 
    double test4 =  getDensity(Foot, d); 
    double test5 =  getDensity(Shank, d); 
    double test6 =  getDensity(Thigh, d); 
    double test7 =  getDensity(HeadNeck, d); 
    double test8 =  getDensity(Trunk, d); 

    std::vector<double> volumes(8,0);
    volumes[0] =  2*(getMass(Hand, mass) / test1); // note: density * vol = mass
    volumes[1] =  2*(getMass(Forearm, mass) / test2); // note: density * vol = mass
    volumes[2] =  2*(getMass(UpperArm, mass) / test3); // note: density * vol = mass
    volumes[3] =  2*(getMass(Foot, mass) / test4); // note: density * vol = mass
    volumes[4] =  2*(getMass(Shank, mass) / test5); // note: density * vol = mass
    volumes[5] =  2*(getMass(Thigh, mass) / test6); // note: density * vol = mass
    volumes[6] =  getMass(HeadNeck, mass) / test7; // note: density * vol = mass
    volumes[7] =  getMass(Trunk, mass) / test8; // note: density * vol = mass

    double sum = 0;
    for (int i = 0; i < (int) volumes.size(); i++) sum += volumes[i];
    std::cout << sum << " " << volume << std::endl;
*/
}

void Anthropometrics::init(const ASkeleton& inSkeleton, 
   double height, double weight, double factor)
{
    ASkeleton skeleton = inSkeleton;
    ScaleSkeleton(skeleton, factor);

    // Find up direction
    int upidx = 2; 
    vec3 dim = getDimensions(skeleton);
    if (dim[1] > dim[2]) upidx = 1;

    setupBoneShapes(skeleton, height, weight);
}

void Anthropometrics::setupBoneShapes(const ASkeleton& skeleton, double height, double totalMass)
{
    AJoint* root = skeleton.getRoot();

    double d = getBodyDensity(height, totalMass);
    for (int i = 0; i < skeleton.getNumJoints(); i++)
    {
        AJoint* j = skeleton.getByID(i);
        BodyData data = ASL_Mapping[j->getName()];
        _jmass[j->getName()] = getMass(data.first, totalMass) * data.second;
        //std::cout << j->getName() << " " << data.first << " " << getMass(data.first, totalMass) << " " << data.second << " " << _jmass[j->getName()] << std::endl;
        _jdensity[j->getName()] = getDensity(data.first, d); 
        _comOffset[j->getName()] = getCOMProximal(data.first); 
    }

    std::map<std::string,double>::iterator it;
    for (it = _jmass.begin(); it != _jmass.end(); it++)
    {
        _comOffset[it->first] = 0.5;
        _jdensity[it->first] = 1.0;
    }

   computeMass(skeleton);
}

void Anthropometrics::computeMass(const ASkeleton& skeleton)
{
   for (int i = 0; i < skeleton.getNumJoints(); i++)
   {
       AJoint* j = skeleton.getByID(i);
       double density = _jdensity[j->getName()] * 1000;
       float length = glm::length(j->getLocalTranslation());
       double tmp = density * length * 4; // cuboid
       //double tmp = density * (3.0/4.0) * (1.0/2.0) * M_PI * length; // ellipsoid
       //double tmp = density * M_PI * length; // cylinder
       if (tmp > 0.0 && _jmass[j->getName()] > 0)
       {
           _aspx[j->getName()] = sqrt(_jmass[j->getName()]/tmp); 
       }
       else
       {
           _aspx[j->getName()] = 0.0;
       }
       //std::cout << j->getName() << " " << _jmass[j->getName()] << " " << length << " " << _aspx[j->getName()] << std::endl;
   }

   double fTotalMass = 0;
   for (int i = 0; i < skeleton.getNumJoints(); i++)
   {
        AJoint* j = skeleton.getByID(i);
        fTotalMass += _jmass[j->getName()];
    }
    std::cout << "totalMass = " << fTotalMass << std::endl;
    _totalMass = fTotalMass;
}

double Anthropometrics::getRadius(const std::string& name)
{
   return _aspx[name];
}

double Anthropometrics::getMass(const std::string& name)
{
   return _jmass[name];
}

double Anthropometrics::getDensity(const std::string& name)
{
   return _jdensity[name];
}

double Anthropometrics::getCOMProximal(const std::string& name)
{
   return _comOffset[name];
}

/*
void Anthropometrics::setupBoneShapes(const ASkeleton& skeleton, 
   const BodyMap& mapping, double height, double totalMass)
{
    AJoint* root = skeleton.getRoot();

    _height = height;
    _totalMass = totalMass;
    std::cout << "SetupBoneShapes: height = " << height << " weight = " << totalMass << std::endl;

    double d = getBodyDensity(height, totalMass);
    for (int i = 0; i < skeleton.getNumJoints(); i++)
    {
        AJoint* j = skeleton.getByID(i);
        Anthropometrics::BodyData::iterator data = mapping.find(j->getName()); 
        _jmass[j->getName()] = getMass(data.first, totalMass) * data.second;
        _jdensity[j->getName()] = getDensity(data.first, d); // might not be right -> what about scale?
        _comOffset[j->getName()] = getCOMProximal(data.first); // might not be right -> what about scale
    }
    
    //computeMass(skeleton);
    //computeInertia(skeleton);
}
*/
/*

mat3 InvDynBVH::computeBoxInertia(double hw, double hl, double hd, double density, int direction)
{
    double mass = computeBoxMass(hw,hl,hd,density);
    double Il = (1.0/12.0)*mass*(4*hw*hw + 4*hd*hd);
    double Iw = (1.0/12.0)*mass*(4*hl*hl + 4*hd*hd);
    double Id = (1.0/12.0)*mass*(4*hw*hw + 4*hl*hl);
    mat3 I = identity3D;
    I[0][0] = Iw;
    I[1][1] = Il;
    I[2][2] = Id;
    return I;
}   
    
double InvDynBVH::computeCapsuleMass(double r, double l, double density)
{
    double mcylinder = M_PI*r*r*l*density;
    double massphere = (4.0/3.0)*M_PI*r*r*r*density;
    return massphere + mcylinder;
}

// From http://www.gamedev.net/topic/350429-capsule-moment-of-inertia/
// direction: 0=X, 1=Y, 2=Z
mat3 InvDynBVH::computeCapsuleInertia(double r, double l, double density, int direction)
{
    assert(direction >= 0 && direction <= 2);

    mat3 m = identity3D;
    double M1 = M_PI*r*r*l*density;       // cylinder mass
    double M2 = (4.0/3.0)*M_PI*r*r*r*density;   // total cap mass

    double Ia = M1*(0.25*r*r + (1.0/12.0)*l*l) + M2*(0.4*r*r + 0.375*r*l + 0.25*l*l);
    double Ib = (M1*0.5 + M2*0.4)*r*r;
    m[0][0] = Ia;
    m[1][1] = Ia;
    m[2][2] = Ia;
    m[direction][direction] = Ib;
    //std::cout << Ia << " " << Ib << std::endl;

    return m;
}

double InvDynBVH::computeBoxMass(double hw, double hl, double hd, double density)
{
    double volume = 8*hw*hl*hd;
    return volume * density;
}


double InvDynBVH::ComputeCombinedMass(const Skeleton& skeleton, Joint* joint)
{
    double mass = jmass[joint->GetName()];
    for (int i = 0; i < joint->GetNumChildren(); i++)
    {
        Joint* child = joint->GetChildAt(i);
        mass += ComputeCombinedMass(skeleton, child);
    }
    jmassCombo[joint->GetName()] = mass;
    return mass;
}
void InvDynBVH::ComputeInertia(Skeleton& skeleton)
{
    // inertia: align bone direction with the y (UP) direction
    // create rot matrix by mapping x,y,z joint axes to right,up,direction
    // need to set aspx based on mass distribution
    for (int i = 0; i < skeleton.GetNumJoints(); i++)
   {
      Joint* pBone = skeleton.GetJointByID(i);
        float r = aspx[pBone->GetName()];
        float x = aspx[pBone->GetName()];
        float y = pBone->m_translation.Length();
        float z = aspx[pBone->GetName()];
        float mass = jmass[pBone->GetName()];
        float com = comOffset[pBone->GetName()];
      //printf("mass = %f\n", mass);

      // Use inertia of a cuboid
      //jIlocal[pBone->GetName()] = mat3(
      // vec3(mass/12.0*(y*y+4*z*z),0,0), 
      // vec3(0,mass/12.0*(4*x*x+4*z*z),0), 
      // vec3(0,0,mass/12.0*(4*x*x+y*y)));

      // Use inertia of a ellipse
      //jIlocal[pBone->GetName()] = mat3(
      // vec3(mass/5.0*(y*y+z*z),0,0), 
      // vec3(0,mass/5.0*(x*x+z*z),0), 
      // vec3(0,0,mass/5.0*(x*x+y*y)));

      // Use inertia of a cylinder
        jIlocal[pBone->GetName()] = mat3(
            vec3(mass/12.0*(3*r*r+y*y),0,0),
            vec3(0,mass/2.0*(r*r), 0),  // assume y is major direction
            vec3(0,0,mass/12.0*(3*r*r+y*y)));

        // See: http://hyperphysics.phy-astr.gsu.edu/hbase/parax.html#pax
        mat3 parallelAxisContribution(
            vec3(0,0,0),
            vec3(0, -mass*(com-0.5)*(com-0.5)*y*y,0),
            vec3(0,0,0));
        jIlocal[pBone->GetName()] += parallelAxisContribution;

        // See: http://hyperphysics.phy-astr.gsu.edu/hbase/parax.html#pax
        mat3 parallelAxisContribution(
            vec3(0,0,0),
            vec3(0, -mass*(com-0.5)*(com-0.5)*y*y,0),
            vec3(0,0,0));
        jIlocal[pBone->GetName()] += parallelAxisContribution;

        //std::cout << pBone->GetName() << " xx = " << jIlocal[pBone->GetName()][0][0] <<
        //    " yy = " << jIlocal[pBone->GetName()][1][1] <<
        //    " zz = " << jIlocal[pBone->GetName()][2][2] << std::endl;

        // Use inertia of sphere
        //double a = (2.0/5.0)*mass * y*y;
      //jIlocal[pBone->GetName()] = mat3(
         //vec3(a,0,0), 
         //vec3(0,a,0), 
         //vec3(0,0,a));
        //jI[pBone->GetName()] = jIlocal[pBone->GetName()];
        //std::cout << jIlocal[pBone->GetName()] << std::endl;


        vec3 up = pBone->GetLocalTranslation();
        up = up.Normalize();
        vec3 forward(0,0,1);
        vec3 right(1,0,0);
        right = up.Cross(forward);
        forward = right.Cross(up);

        mat3 i2j(right, up, forward);
        //std::cout << i2j << std::endl;

        //vec3 test1 = i2j * up; // test1 should be (0,1,0)
        //vec3 test2 = i2j * vec3(0,1,0); // test2 = up
        //vec3 test3 = i2j.Transpose() * vec3(0,1,0);

        // align cuboid with joint
        jI[pBone->GetName()] = i2j.Transpose() * jIlocal[pBone->GetName()] * i2j;
   }
}
*/

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

void InvDynBVH::SetBoneShapesCMU(Skeleton& skeleton)
{
   // ASN: Set desired masses directly, figure out ratios later in ComputeMass
    jmass["root"]=0.0;
    jmass["lhipjoint"]=0.0;
    jmass["lfemur"]= 4.38888;
    jmass["ltibia"]=6.0444;
    jmass["lfoot"]=1.9964;
    jmass["ltoes"]=0.252598;
    jmass["ltoesSite"]=0.0187188;

    jmass["rhipjoint"]=0.0;     // doesn't exist in BVH
    jmass["rfemur"]=4.53631;
    jmass["rtibia"]=6.15115;
    jmass["rfoot"]=2.08259;
    jmass["rtoes"]=0.268396 ;
    jmass["rtoesSite"]=0.0202983;

    jmass["lowerback"]=0; // no length for lowerback
    jmass["upperback"]=4.65925;
    jmass["thorax"]=9.95197;
    jmass["lowerneck"]=0.664636;
    jmass["upperneck"]=0.664636;
    jmass["head"]=0.659668;
    jmass["headSite"]=4.19339;

    jmass["lclavicle"]=0.287175;
    jmass["lhumerus"]=1.77743;
    jmass["lradius"]=0.599668;
    jmass["lwrist"]=0.220286;
    jmass["lhand"]=0.0149651;
    jmass["lthumb"]=0.000173234;
    jmass["lthumbSite"]=0.0;
    jmass["lfingers"]=0.000120653;
    jmass["lfingersSite"]=0.0;

    jmass["rclavicle"]=0.299184;
    jmass["rhumerus"]=1.55477;
    jmass["rradius"]=0.69524;
    jmass["rwrist"]=0.255395;
    jmass["rhand"]=0.0170309;
    jmass["rthumb"]=0.000197147;
    jmass["rthumbSite"]=0.0;
    jmass["rfingers"]=0.000137308;
    jmass["rfingersSite"]= 0.0;
    std::map<std::string,double>::iterator it;
    for (it = jmass.begin(); it != jmass.end(); it++)
    {
        comOffset[it->first] = 0.5;
    }

    ComputeMass(skeleton);
    ComputeInertia(skeleton);
}

void InvDynBVH::SetBoneShapesMB(Skeleton& skeleton)
{
    jmass["Hips"]=0.0; 
    jmass["LeftUpLeg"]=4.39;
    jmass["LeftLeg"]=6.04;
    jmass["LeftFoot"]=2.00;  
    jmass["LeftToeBase"]=0.25; 
    jmass["LeftToeBaseSite"]=0.02;

    jmass["RightUpLeg"]=4.54;
    jmass["RightLeg"]=6.15; 
    jmass["RightFoot"]=2.08; 
    jmass["RightToeBase"]=0.27; 
    jmass["RightToeBaseSite"]=0.02;

    jmass["Spine"]=3.0; 
    jmass["Spine2"]=3.8175;
    jmass["Spine3"]=3.8175;
    jmass["Spine4"]=3.8175;
    jmass["Neck"]=1.46; 
    jmass["Neck1"]=0.46;
    jmass["Head"]=0.46; 
    jmass["HeadSite"]=4.19;

    jmass["LeftShoulder"]=0.29;
    jmass["LeftArm"]=1.78;
    jmass["LeftForeArm"]=0.60;
    jmass["LeftHand"]=0.22; 
    jmass["LeftHandSite"]=0.01;

    jmass["RightShoulder"]=0.30;
    jmass["RightArm"]=1.55; 
    jmass["RightForeArm"]=0.70;
    jmass["RightHand"]=0.26; 
    jmass["RightHandSite"]=0.02;

    std::map<std::string,double>::iterator it;
    for (it = jmass.begin(); it != jmass.end(); it++)
    {
        comOffset[it->first] = 0.5;
        jdensity[it->first] = 1.0;
    }

    ComputeMass(skeleton);
    ComputeInertia(skeleton);
}

                                                                  

*/

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
