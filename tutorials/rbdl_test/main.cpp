
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/utils.hpp>
#include <map>
#include <cassert>
#include <iostream>
#include <fstream>
#include "AnimationToolkit.h"

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

class MyWindow : public SimWindow
{
public:
  /// Constructor
  MyWindow(const WorldPtr& world)
  {
    setWorld(world);

  }

  virtual ~MyWindow() 
  {
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
      case 'r':
        mTime = 0;
        break;
      default:
        SimWindow::keyboard(key, x, y);
    }
  }

  void timeStepping() override
  {
    // print current state
    SkeletonPtr skeleton = mWorld->getSkeleton("biped");

    skeleton->computeInverseDynamics(true);
    Eigen::VectorXd forces = skeleton->getForces();
    std::cout << forces.transpose() << std::endl;

    Joint* joint = skeleton->getJoint("child"); 
    BodyNode* body = skeleton->getBodyNode("child"); 
    Eigen::Matrix6d inertia = body->getArticulatedInertia();
     //Eigen::Matrix6d inertia = body->getSpatialInertia();
     std::cout << body->getName() << " SpatialInertia:\n" << inertia << std::endl;

     Eigen::Vector6d velocity = joint->getRelativeSpatialVelocity();
     std::cout << joint->getName() << " SpatialVelocity:\n" << velocity.transpose() << std::endl;

     Eigen::Vector6d acceleration = joint->getRelativeSpatialAcceleration();
     std::cout << joint->getName() << " SpatialAcceleration:\n" << acceleration.transpose() << std::endl;

    // Step the simulation forward
    mTime += mWorld->getTimeStep();

    //joint->setPosition(0, sin(mTime));
    joint->setPosition(2, 1);

    SimWindow::timeStepping();

  }

  float mTime = 0;
};

void createBox(const BodyNodePtr& bn, const std::string& name, 
   float mass, const glm::vec3& offset, float radius)
{
  // Create a BoxShape to be used for both visualization and collision checking
  float length = glm::length(offset);

  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(radius*2, radius*2, length)));

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
  //Eigen::Vector3d center(0,0,0);

  Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
  box_tf.fromPositionOrientationScale(center, R, Eigen::Vector3d::Ones());
  shapeNode->setRelativeTransform(box_tf);

  Eigen::Matrix3d inertia = shapeNode->getShape()->computeInertia(mass);
  std::cout << name << " inertia: \n" << inertia << std::endl;
}

void setGeometry(const BodyNodePtr& bn, const std::string& name, double ballR)
{
  // Make a shape for the Joint
  std::shared_ptr<EllipsoidShape> ball(
        new EllipsoidShape(sqrt(2) * Eigen::Vector3d(ballR, ballR, ballR)));
  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(ball);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  createBox(bn,name+"1",10.0,glm::vec3(0.707,0.707,0),0.25);
  //createBox(bn,name+"2",10.0,glm::vec3(-0.707,0.707,0),0.25);
  bn->setLocalCOM(Eigen::Vector3d(0.707 * 0.5, 0.707 * 0.5, 0.0));
  //bn->setLocalCOM(Eigen::Vector3d(0.0, 0.5, 0.0));
  bn->setMass(10.0);
  Eigen::Matrix6d inertia = bn->getSpatialInertia();
  std::cout << name << " SpatialInertia:\n" << inertia << std::endl;

  Eigen::Vector6d velocity = bn->getSpatialVelocity();
  std::cout << name << " SpatialVelocity:\n" << velocity.transpose() << std::endl;

  Eigen::Vector6d acceleration = bn->getSpatialAcceleration();
  std::cout << name << " SpatialAcceleration:\n" << acceleration.transpose() << std::endl;
}

BodyNode* makeRootBody(const SkeletonPtr& pendulum, const std::string& name)
{
  FreeJoint::Properties properties;
  properties.mName = name;
  properties.mInitialPositions = Eigen::Vector6d::Zero();
  properties.mActuatorType = Joint::LOCKED;

  BodyNodePtr bn = pendulum->createJointAndBodyNodePair<FreeJoint>(
        nullptr, properties, BodyNode::AspectProperties(name)).second;

  float ballR = default_pelvis_radius;
  std::shared_ptr<EllipsoidShape> ball(
        new EllipsoidShape(sqrt(2) * Eigen::Vector3d(ballR, ballR, ballR)));
  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(ball);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  createBox(bn,name,10.0,glm::vec3(0.,1.,0),0.25);
  bn->setLocalCOM(Eigen::Vector3d(0.0, 0.5, 0.0));
  bn->setMass(10.0);
  return bn;
}

BodyNode* addBody(const SkeletonPtr& pendulum, const std::string& name, BodyNode* parent)
{
  glm::vec3 offset(0,1.0,0); 

  // Set up the properties for the Joint
  EulerJoint::Properties properties;
  properties.mName = name;
  properties.mAxisOrder = EulerJoint::AxisOrder::XYZ;
  properties.mT_ParentBodyToJoint.translation() =
      Eigen::Vector3d(offset[0], offset[1], offset[2]); // Joint offset
  properties.mActuatorType = Joint::VELOCITY;

  BodyNodePtr bn = pendulum->createJointAndBodyNodePair<EulerJoint>(
        parent, properties, BodyNode::AspectProperties(name)).second;

  // Make a shape for the Joint
  setGeometry(bn, name, default_radius);
  return bn;
}

// Load a biped model and enable joint limits and self-collision
SkeletonPtr loadBiped()
{
  SkeletonPtr biped = Skeleton::create("biped");
  BodyNode* root = makeRootBody(biped, "root");
  BodyNode* joint = addBody(biped, "child", root);

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
