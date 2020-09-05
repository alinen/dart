
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/utils.hpp>
#include <map>
#include "AnimationToolkit.h"

const double default_speed_increment = 0.5;

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

class Controller
{
public:
  /// Constructor
  Controller(const SkeletonPtr& biped)
    : mBiped(biped), mPreOffset(0.0), mSpeed(0.0)
  {
    int nDofs = mBiped->getNumDofs();

    mForces = Eigen::VectorXd::Zero(nDofs);

    mKp = Eigen::MatrixXd::Identity(nDofs, nDofs);
    mKd = Eigen::MatrixXd::Identity(nDofs, nDofs);

    for (std::size_t i = 0; i < 6; ++i)
    {
      mKp(i, i) = 0.0;
      mKd(i, i) = 0.0;
    }

    for (std::size_t i = 6; i < biped->getNumDofs(); ++i)
    {
      mKp(i, i) = 1000;
      mKd(i, i) = 50;
    }

    setTargetPositions(mBiped->getPositions());
  }

  /// Reset the desired dof position to the current position
  void setTargetPositions(const Eigen::VectorXd& pose)
  {
    mTargetPositions = pose;
  }

  /// Clear commanding forces
  void clearForces()
  {
    mForces.setZero();
  }

  /// Add commanding forces from PD controllers
  void addPDForces()
  {
    // Lesson 2
    Eigen::VectorXd q = mBiped->getPositions();
    Eigen::VectorXd dq = mBiped->getVelocities();

    Eigen::VectorXd p = -mKp * (q - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;

    mForces += p + d;
    mBiped->setForces(mForces);
  }

  /// Add commanind forces from Stable-PD controllers
  void addSPDForces()
  {
    // Lesson 3
    Eigen::VectorXd q = mBiped->getPositions();
    Eigen::VectorXd dq = mBiped->getVelocities();

    Eigen::MatrixXd invM = (mBiped->getMassMatrix() + mKd * mBiped->getTimeStep()).inverse();
    Eigen::VectorXd p = -mKp * (q + dq * mBiped->getTimeStep() - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;
    Eigen::VectorXd qddot = invM * (-mBiped->getCoriolisAndGravityForces() + p + d + mBiped->getConstraintForces());

    mForces += p + d - mKd * qddot * mBiped->getTimeStep();
    mBiped->setForces(mForces);
  }

  /// add commanding forces from ankle strategy
  void addAnkleStrategyForces()
  {
    // Lesson 4
    Eigen::Vector3d COM = mBiped->getCOM();
    Eigen::Vector3d offset(0.05, 0, 0);
    Eigen::Vector3d COP = mBiped->getBodyNode("h_heel_left")->getTransform() * offset;
    double diff = COM[0] - COP[0];

    Eigen::Vector3d dCOM = mBiped->getCOMLinearVelocity();
    Eigen::Vector3d dCOP =  mBiped->getBodyNode("h_heel_left")->getLinearVelocity(offset);
    double dDiff = dCOM[0] - dCOP[0];

    int lHeelIndex = mBiped->getDof("j_heel_left_1")->getIndexInSkeleton();
    int rHeelIndex = mBiped->getDof("j_heel_right_1")->getIndexInSkeleton();
    int lToeIndex = mBiped->getDof("j_toe_left")->getIndexInSkeleton();
    int rToeIndex = mBiped->getDof("j_toe_right")->getIndexInSkeleton();
    if (diff < 0.1 && diff >= 0.0)
    {
      // Feedback rule for recovering forward push
      double k1 = 200.0;
      double k2 = 100.0;
      double kd = 10;
      mForces[lHeelIndex] += -k1 * diff - kd * dDiff;
      mForces[lToeIndex] += -k2 * diff - kd * dDiff;
      mForces[rHeelIndex] += -k1 * diff - kd * dDiff;
      mForces[rToeIndex] += -k2 * diff - kd * dDiff;
    }
    else if (diff > -0.2 && diff < -0.05)
    {
      // Feedback rule for recovering backward push
      double k1 = 2000.0;
      double k2 = 100.0;
      double kd = 100;
      mForces[lHeelIndex] += -k1 * diff - kd * dDiff;
      mForces[lToeIndex] += -k2 * diff - kd * dDiff;
      mForces[rHeelIndex] += -k1 * diff - kd * dDiff;
      mForces[rToeIndex] += -k2 * diff - kd * dDiff;
    }
    mBiped->setForces(mForces);
  }

  // Send velocity commands on wheel actuators
  void setWheelCommands()
  {
    // Lesson 6

    int index1 = mBiped->getDof("joint_front_left_2")->getIndexInSkeleton();
    mBiped->setCommand(index1, mSpeed);

    int wheelFirstIndex = mBiped->getDof("joint_front_left_1")->getIndexInSkeleton();
    for (size_t i = wheelFirstIndex; i < mBiped->getNumDofs(); ++i)
    {
        mKp(i, i) = 0.0;
        mKd(i, i) = 0.0;
    }
  }

  void changeWheelSpeed(double increment)
  {
    mSpeed += increment;
    std::cout << "wheel speed = " << mSpeed << std::endl;
  }

protected:
  /// The biped Skeleton that we will be controlling
  SkeletonPtr mBiped;

  /// Joint forces for the biped (output of the Controller)
  Eigen::VectorXd mForces;

  /// Control gains for the proportional error terms in the PD controller
  Eigen::MatrixXd mKp;

  /// Control gains for the derivative error terms in the PD controller
  Eigen::MatrixXd mKd;

  /// Target positions for the PD controllers
  Eigen::VectorXd mTargetPositions;

  /// For ankle strategy: Error in the previous timestep
  double mPreOffset;

  /// For velocity actuator: Current speed of the skateboard
  double mSpeed;
};

class MyWindow : public SimWindow
{
public:
  /// Constructor
  MyWindow(const WorldPtr& world) : mForceCountDown(0), mPositiveSign(true)
  {
    setWorld(world);

    //mController = std::make_unique<Controller>(mWorld->getSkeleton("biped"));
  }

  /// Handle keyboard input
  void keyboard(unsigned char key, int x, int y) override
  {
    switch (key)
    {
      case ',':
        mForceCountDown = default_countdown;
        mPositiveSign = false;
        break;
      case '.':
        mForceCountDown = default_countdown;
        mPositiveSign = true;
        break;
      default:
        SimWindow::keyboard(key, x, y);
    }
  }

  void timeStepping() override
  {
    //mController->clearForces();

    // Lesson 3
    //mController->addPDForces();
    //mController->addSPDForces();

    // Lesson 4
    //mController->addAnkleStrategyForces();

    // Lesson 6
    //mController->setWheelCommands();

    // Apply body forces based on user input, and color the body shape red
    /*
    if (mForceCountDown > 0)
    {
      BodyNode* bn = mWorld->getSkeleton("biped")->getBodyNode("h_abdomen");
      auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();
      shapeNodes[0]->getVisualAspect()->setColor(dart::Color::Red());

      if (mPositiveSign)
        bn->addExtForce(
            default_force * Eigen::Vector3d::UnitX(),
            bn->getCOM(),
            false,
            false);
      else
        bn->addExtForce(
            -default_force * Eigen::Vector3d::UnitX(),
            bn->getCOM(),
            false,
            false);

      --mForceCountDown;
    }
    */

    // Step the simulation forward
    SimWindow::timeStepping();
  }

protected:
  //std::unique_ptr<Controller> mController;

  /// Number of iterations before clearing a force entry
  int mForceCountDown;

  /// Whether a force should be applied in the positive or negative direction
  bool mPositiveSign;
};

void setGeometry(const BodyNodePtr& bn, AJoint* joint)
{
  // Create a BoxShape to be used for both visualization and collision checking
  // Dimension shoudl be driven by joint size
  glm::vec3 offset = joint->getLocalTranslation()/100.0f; // cm to m
  float length = glm::length(offset);
  float radius = length * 0.25f;
  //std::cout << joint->getName() << " " << length << " " << radius << std::endl;

  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(radius, radius, length)));

  // Create a shape node for visualization and collision checking
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Set the location of the shape node
  // Position should be centered on the limb
  // TODO: Cleanup repeated code from root
  Eigen::Vector3d offsetDir = Eigen::Vector3d(offset[0], offset[1], offset[2]).normalized(); 
  Eigen::Vector3d x,y,z;
  x = Eigen::Vector3d::UnitY().cross(offsetDir).normalized(); 
  if (x.norm() < 0.1) // y and offsetDir are aligned
  {
    y = offsetDir.cross(x);
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

  bn->setLocalCOM(center); // TODO: Fix me
}

BodyNode* makeRootBody(const SkeletonPtr& pendulum, AJoint* joint)
{
  std::string name = joint->getName();

  FreeJoint::Properties properties;
  properties.mName = name + "_joint";
  properties.mInitialPositions = Eigen::Vector6d::Zero();
  //properties.mRestPositions = Eigen::Vector3d::Constant(default_rest_position);
  //properties.mSpringStiffnesses = Eigen::Vector3d::Constant(default_stiffness);
  //properties.mDampingCoefficients = Eigen::Vector3d::Constant(default_damping);

  BodyNodePtr bn = pendulum->createJointAndBodyNodePair<FreeJoint>(
        nullptr, properties, BodyNode::AspectProperties(name)).second;

  // Make a shape for the Joint
  const double& R = default_pelvis_radius;
  std::shared_ptr<EllipsoidShape> ball(
        new EllipsoidShape(sqrt(2) * Eigen::Vector3d(R, R, R)));
  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(ball);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Set the geometry/com of the Body
  glm::vec3 coms(0,0,0);
  for (int i = 0; i < joint->getNumChildren(); i++)
  {
    AJoint* child = joint->getChildAt(i);
    coms += 0.5f * child->getLocalTranslation();
    setGeometry(bn, child);
  }
  coms = coms / (float) joint->getNumChildren();
  
  // Move the center of mass to the center of the children
  bn->setLocalCOM(Eigen::Vector3d(coms[0], coms[1], coms[2]));

  return bn;
}


BodyNode* addBody(const SkeletonPtr& pendulum, BodyNode* parent, AJoint* joint)
{
  glm::vec3 offset = joint->getLocalTranslation()/100.0f; // cm to m

  // Set up the properties for the Joint
  EulerJoint::Properties properties;
  properties.mName = joint->getName() + "_joint";
  properties.mAxisOrder = EulerJoint::AxisOrder::XYZ;
  properties.mT_ParentBodyToJoint.translation() =
      Eigen::Vector3d(offset[0], offset[1], offset[2]); // Joint offset
  //properties.mRestPositions[0] = default_rest_position;
  //properties.mSpringStiffnesses[0] = default_stiffness;
  //properties.mDampingCoefficients[0] = default_damping;

  BodyNodePtr bn = pendulum->createJointAndBodyNodePair<EulerJoint>(
        parent, properties, BodyNode::AspectProperties(joint->getName())).second;

  // Make a shape for the Joint
  const double R = default_radius;
  std::shared_ptr<SphereShape> sph(new SphereShape(R));

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(sph);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
  shapeNode->setRelativeTransform(tf);

  // Set the geometry of the Body
  for (int i = 0; i < joint->getNumChildren(); i++)
  {
    AJoint* child = joint->getChildAt(i);
    setGeometry(bn, child);
  }

  return bn;
}

// Load a biped model and enable joint limits and self-collision
SkeletonPtr loadBiped()
{
  ASkeleton skeleton;
  AMotion motion;

  ABVHReader reader;
  reader.load("/home/alinen/projects/AnimationToolkit/motions/SignLanguage/SIB01-story01-bvh.bvh",
    skeleton, motion);

  SkeletonPtr biped = Skeleton::create("Biped");
  std::map<AJoint*, BodyNode*> bodies;
  for (int i = 0; i < 7/*skeleton.getNumJoints()*/; i++) // left leg
  {
    AJoint* joint = skeleton.getByID(i);
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

void setInitialPose(SkeletonPtr biped)
{
  // Lession 2
  biped->setPosition(biped->getDof("j_thigh_left_z")->getIndexInSkeleton(), 0.15);
  biped->setPosition(biped->getDof("j_thigh_right_z")->getIndexInSkeleton(), 0.15);
  biped->setPosition(biped->getDof("j_shin_left")->getIndexInSkeleton(), -0.4);
  biped->setPosition(biped->getDof("j_shin_right")->getIndexInSkeleton(), -0.4);
  biped->setPosition(biped->getDof("j_heel_left_1")->getIndexInSkeleton(), 0.25);
  biped->setPosition(biped->getDof("j_heel_right_1")->getIndexInSkeleton(), 0.25);
}


// Load a skateboard model and connect it to the biped model via an Euler joint
void modifyBipedWithSkateboard(SkeletonPtr biped)
{
  // Lesson 5
  WorldPtr world = SkelParser::readWorld(DART_DATA_PATH"skel/skateboard.skel");
  SkeletonPtr skateboard = world->getSkeleton(0);

  EulerJoint::Properties properties = EulerJoint::Properties();
  properties.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(0, 0.1, 0);

  skateboard->getRootBodyNode()->moveTo<EulerJoint>(biped->getBodyNode("h_heel_left"), properties);
}

// Set the actuator type for four wheel joints to "VELOCITY"
void setVelocityAccuators(SkeletonPtr biped)
{
  // Lesson 6
 Joint* wheel1 = biped->getJoint("joint_front_left");
    wheel1->setActuatorType(Joint::VELOCITY);
}

// Solve for a balanced pose using IK
Eigen::VectorXd solveIK(SkeletonPtr biped)
{
  biped->setPosition(biped->getDof("j_shin_right")->getIndexInSkeleton(), -1.4);
  biped->setPosition(biped->getDof("j_bicep_left_x")->getIndexInSkeleton(), 0.8);
  biped->setPosition(biped->getDof("j_bicep_right_x")->getIndexInSkeleton(), -0.8);

  // Lesson 7
  Eigen::VectorXd newPose = biped->getPositions();
  BodyNodePtr leftHeel = biped->getBodyNode("h_heel_left");
  BodyNodePtr leftToe = biped->getBodyNode("h_toe_left");
  double initialHeight = -0.8;

  for (std::size_t i = 0; i < default_ik_iterations; ++i)
  {
    Eigen::Vector3d deviation = biped->getCOM() - leftHeel->getCOM();
    Eigen::Vector3d localCOM = leftHeel->getCOM(leftHeel);
    LinearJacobian jacobian = biped->getCOMLinearJacobian()
                              - biped->getLinearJacobian(leftHeel, localCOM);

    // Sagittal deviation
    double error = deviation[0];
    Eigen::VectorXd gradient = jacobian.row(0);
    Eigen::VectorXd newDirection = -0.2 * error * gradient;

    // Lateral deviation
    error = deviation[2];
    gradient = jacobian.row(2);
    newDirection += -0.2 * error * gradient;

    // Position constraint on four (approximated) corners of the left foot
    Eigen::Vector3d offset(0.0, -0.04, -0.03);
    error = (leftHeel->getTransform() * offset)[1] - initialHeight;
    gradient = biped->getLinearJacobian(leftHeel, offset).row(1);
    newDirection += -0.2 * error * gradient;

    offset[2] = 0.03;
    error = (leftHeel->getTransform() * offset)[1] - initialHeight;
    gradient = biped->getLinearJacobian(leftHeel, offset).row(1);
    newDirection += -0.2 * error * gradient;

    offset[0] = 0.04;
    error = (leftToe->getTransform() * offset)[1] - initialHeight;
    gradient = biped->getLinearJacobian(leftToe, offset).row(1);
    newDirection += -0.2 * error * gradient;

    offset[2] = -0.03;
    error = (leftToe->getTransform() * offset)[1] - initialHeight;
    gradient = biped->getLinearJacobian(leftToe, offset).row(1);
    newDirection += -0.2 * error * gradient;

    newPose += newDirection;
    biped->setPositions(newPose);
    biped->computeForwardKinematics(true, false, false);
  }
  return newPose;
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

  // Lesson 2
  //setInitialPose(biped);

  // Lesson 5
  //modifyBipedWithSkateboard(biped);

  // Lesson 6
  //setVelocityAccuators(biped);

  // Lesson 7
  //Eigen::VectorXd balancedPose = solveIK(biped);
  //biped->setPositions(balancedPose);

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
