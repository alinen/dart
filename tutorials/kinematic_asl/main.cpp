
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/utils.hpp>
#include <map>
#include <cassert>
#include <iostream>
#include "AnimationToolkit.h"
#include "../ragdoll_asl/Anthropometrics.h"

const int default_ik_iterations = 4500;

const double default_force = 50.0; // N
const double default_torque = 15.0; // N-m
const int default_countdown = 100; // Number of timesteps for applying force

const double default_pelvis_radius = 0.05; // m
const double default_radius = 0.01;  // m

const double default_rest_position = 0.0;
const double delta_rest_position = 10.0 * M_PI / 180.0;

const double default_stiffness = 0.0;
const double delta_stiffness = 10;

const double default_damping = 5.0;
const double delta_damping = 1.0;

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui::glut;
using namespace dart::utils;
using namespace dart::math;

// Globals for now...
AMotion bvhMotion;
ASkeleton bvhSkeleton;
Anthropometrics anthropometrics;

class MyWindow : public SimWindow
{
public:
  /// Constructor
  MyWindow(const WorldPtr& world)
  {
    setWorld(world);
  }

  /// Handle keyboard input
  void keyboard(unsigned char key, int x, int y) override
  {
    switch (key)
    {
      case ',':
        break;
      case '.':
        break;
      default:
        SimWindow::keyboard(key, x, y);
    }
  }

  void timeStepping() override
  {
    bvhMotion.update(bvhSkeleton, mWorld->getTime());

    SkeletonPtr skeleton = mWorld->getSkeleton("biped");
    for (size_t i = 1; i < skeleton->getNumJoints(); i++)
    {
      Joint* joint = skeleton->getJoint(i);

      AJoint* bvhJoint = bvhSkeleton.getByName(joint->getName());
      assert(bvhJoint != 0);

      glm::quat rotation = bvhJoint->getLocalRotation();
      glm::vec3 euler = glm::extractEulerAngleRO(glm::XYZ, glm::mat3(rotation));

      // Set joint "position" as XYZ euler angles (local coordinates?)
      joint->setPosition(0, euler[0]);
      joint->setPosition(1, euler[1]);
      joint->setPosition(2, euler[2]);

    }
    // Step the simulation forward
    SimWindow::timeStepping();

    skeleton->computeForwardKinematics();
    skeleton->computeInverseDynamics();
    for (size_t i = 1; i < skeleton->getNumJoints(); i++)
    {
      Joint* joint = skeleton->getJoint(i);
      if (joint->getName() == "LeftForeArm")
      {
         Eigen::MatrixXd forces = joint->getForces();
         Eigen::VectorXd accels = joint->getAccelerations();
         Eigen::VectorXd vels = joint->getVelocities();
         Eigen::Vector6d sVels = joint->getRelativeSpatialVelocity();
         Eigen::Vector6d sAccs = joint->getRelativeSpatialAcceleration();
         double potentialEnergy = joint->computePotentialEnergy();
         
         std::cout << "LeftArm FORCE: " << forces.transpose() << std::endl;
         std::cout << "LeftArm ACCEL: " << accels.transpose() << std::endl;
         std::cout << "LeftArm VELS : " << vels.transpose() << std::endl;
         std::cout << "LeftArm Potential Energy: " << potentialEnergy << std::endl;
         std::cout << "LeftArm Spatial VELS: " << sVels.transpose() << std::endl;
         std::cout << "LeftArm Spatial ACCS: " << sAccs.transpose() << std::endl;
      }

    }
      
  }
};

void createBox(const BodyNodePtr& bn, AJoint* joint, const glm::vec3& offset, float radius)
{
  // Create a BoxShape to be used for both visualization and collision checking
  // Dimension shoudl be driven by joint size
  float length = glm::length(offset);
  //std::cout << joint->getName() << " " << length << " " << radius << std::endl;

  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(radius, radius, length)));

  // Create a shape node for visualization and collision checking
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Set the location of the shape node
  // Position should be centered on the limb
  Eigen::Vector3d offsetDir = Eigen::Vector3d(offset[0], offset[1], offset[2]).normalized(); 
  Eigen::Vector3d x,y,z;
  x = Eigen::Vector3d::UnitY().cross(offsetDir).normalized(); 
  if (x.norm() < 0.00001) // y and offsetDir are aligned
  {
    y = offsetDir.cross(Eigen::Vector3d::UnitX());
    y.normalize();
    x = y.cross(offsetDir);
    x.normalize();
  }
  else
  {
    x.normalize();
    y = offsetDir.cross(x);
    y.normalize();
  }
  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = offsetDir;
  //std::cout << x << " " << y << " " << offsetDir << std::endl;
  Eigen::Vector3d center = 0.5 * Eigen::Vector3d(offset[0], offset[1], offset[2]);

  Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
  box_tf.fromPositionOrientationScale(center, R, Eigen::Vector3d::Ones());
  shapeNode->setRelativeTransform(box_tf);
}

void setGeometry(const BodyNodePtr& bn, AJoint* joint, double radius)
{
  // Make a shape for the Joint
  std::shared_ptr<EllipsoidShape> ball(
        new EllipsoidShape(sqrt(2) * Eigen::Vector3d(radius, radius, radius)));
  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(ball);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Set the geometry/com of the Body
  glm::vec3 coms(0,0,0);
  float mass = 0.0; 
  for (int i = 0; i < joint->getNumChildren(); i++)
  {
    AJoint* child = joint->getChildAt(i);
    glm::vec3 offset = child->getLocalTranslation()/100.0f; // cm to m
    float radius = anthropometrics.getRadius(child->getName());

    coms += 0.5f * offset;
    mass += anthropometrics.getMass(child->getName());
    if (mass > 0) createBox(bn, child, offset, radius);
  }
  coms = coms / (float) joint->getNumChildren();
  
  // Move the center of mass to the average com of the children
  // coms are at the center of each box joint
  bn->setLocalCOM(Eigen::Vector3d(coms[0], coms[1], coms[2]));
  bn->setMass(mass);
}

BodyNode* makeRootBody(const SkeletonPtr& pendulum, AJoint* joint)
{
  std::string name = joint->getName();

  FreeJoint::Properties properties;
  properties.mName = name;
  properties.mInitialPositions = Eigen::Vector6d::Zero();
  properties.mActuatorType = Joint::LOCKED;

  BodyNodePtr bn = pendulum->createJointAndBodyNodePair<FreeJoint>(
        nullptr, properties, BodyNode::AspectProperties(name)).second;

  setGeometry(bn, joint, default_pelvis_radius);
  return bn;
}

BodyNode* addBody(const SkeletonPtr& pendulum, BodyNode* parent, AJoint* joint)
{
  glm::vec3 offset = joint->getLocalTranslation()/100.0f; // cm to m

  // Set up the properties for the Joint
  EulerJoint::Properties properties;
  properties.mName = joint->getName();
  properties.mAxisOrder = EulerJoint::AxisOrder::XYZ;
  properties.mT_ParentBodyToJoint.translation() =
      Eigen::Vector3d(offset[0], offset[1], offset[2]); // Joint offset
  properties.mActuatorType = Joint::VELOCITY;

  BodyNodePtr bn = pendulum->createJointAndBodyNodePair<EulerJoint>(
        parent, properties, BodyNode::AspectProperties(joint->getName())).second;

  // Make a shape for the Joint
  setGeometry(bn, joint, default_radius);
  return bn;
}

bool isHandJoint(AJoint* joint)
{
  while (joint)
  {
    if (joint->getName().find("Palm") != std::string::npos)
    {
      return true;
    }
    joint = joint->getParent();
  }
  return false;
}

// Load a biped model and enable joint limits and self-collision
SkeletonPtr loadBiped()
{
  ABVHReader reader;
  reader.load("/home/alinen/projects/AnimationToolkit/motions/SignLanguage-2020/p01/p01_INTRODUCTION.bvh",
    bvhSkeleton, bvhMotion);

  anthropometrics.init(bvhSkeleton, 1.7526, 70.67, 0.01); // 0.01 -> convert cm to m 
  bvhMotion.update(bvhSkeleton, 0); // set pose at time 0

  // sitting position
  //bvhSkeleton.getByID(2)->setLocalTranslation(glm::vec3(0.00,0.00,35.00));
  //bvhSkeleton.getByID(9)->setLocalTranslation(glm::vec3(0.00,0.00,35.00));

  SkeletonPtr biped = Skeleton::create("biped");
  std::map<AJoint*, BodyNode*> bodies;
  for (int i = 0; i < bvhSkeleton.getNumJoints(); i++) // left leg
  {
    AJoint* joint = bvhSkeleton.getByID(i);
    if (isHandJoint(joint)) continue;
    AJoint* parent = joint->getParent();
    BodyNode* bn = parent? 
      addBody(biped, bodies[parent], joint) : 
      makeRootBody(biped, joint);
    bodies[joint] = bn;
  }

  biped->enableSelfCollisionCheck();
  biped->disableAdjacentBodyCheck();

  return biped;
}

SkeletonPtr createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body
      = floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double floor_width = 10.0;
  double floor_height = 0.01;
  std::shared_ptr<BoxShape> box(
      new BoxShape(Eigen::Vector3d(floor_width, floor_height, floor_width)));
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Black());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, -1.0, 0.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

int main(int argc, char* argv[])
{
  SkeletonPtr floor = createFloor();
  SkeletonPtr biped = loadBiped();

 // TODO: Add chair
 //ARenderer::DrawCube(ATransform(glm::quat(0.00,0.00,0.00,1.00),glm::vec3(0.00,0.00,0.00),glm::vec3(40.00,197.00,40.00)));

  WorldPtr world = std::make_shared<World>();
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  if (dart::collision::CollisionDetector::getFactory()->canCreate("bullet"))
  {
    world->getConstraintSolver()->setCollisionDetector(
        dart::collision::CollisionDetector::getFactory()->create("bullet"));
  }

  world->addSkeleton(floor);
  world->addSkeleton(biped);

  // Create a window for rendering the world and handling user input
  MyWindow window(world);

  // Print instructions
  std::cout << "'.': forward push" << std::endl;
  std::cout << "',': backward push" << std::endl;
  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': replay simulation" << std::endl;
  std::cout << "'v': Turn contact force visualization on/off" << std::endl;
  std::cout << "'[' and ']': replay one frame backward and forward"
            << std::endl;

  // Initialize glut, initialize the window, and begin the glut event loop
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Motion tracking ASL");
  glutMainLoop();
}
