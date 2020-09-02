/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/utils.hpp>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui::glut;
using namespace dart::utils;
using namespace dart::math;

SkeletonPtr biped = NULL;

class MyWindow : public SimWindow
{
public:
  MyWindow(const WorldPtr& world) 
  {
    setWorld(world);
  }

  /// Handle keyboard input
  void keyboard(unsigned char key, int x, int y) override
  {
    SimWindow::keyboard(key, x, y);
  }

  void timeStepping() override
  {
    SimWindow::timeStepping();
    Eigen::VectorXd q = biped->getPositions();
    Eigen::VectorXd dq = biped->getVelocities();
    Eigen::VectorXd dqq = biped->getAccelerations();

    std::cout << "pos: " << q.transpose() << std::endl; 
    std::cout << "vel: " << dq.transpose() << std::endl; 
    std::cout << "acc: " << dqq.transpose() << std::endl; 

    const auto result
      = mWorld->getConstraintSolver()->getLastCollisionResult();
    for (const auto& contact : result.getContacts())
    {
      Eigen::Vector3d v = contact.point;
      Eigen::Vector3d f = contact.force / 10.0;
      std::cout << "contacts: " << v.transpose() << " " << f.transpose() <<  std::endl;
    }
    Eigen::MatrixXd M = biped->getMassMatrix();
    Eigen::VectorXd corGrav = biped->getCoriolisAndGravityForces();
    Eigen::VectorXd forces = biped->getConstraintForces();
    std::cout << "MASS: " << M << std::endl;
    std::cout << "cor: " << corGrav.transpose() << std::endl; 
    std::cout << "forces: " << forces.transpose() << std::endl; 
    std::cout << "-------------------\n";
  }
};

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

SkeletonPtr loadModel()
{
  // Lesson 1

  // Create the world with a skeleton
  WorldPtr world = SkelParser::readWorld("dart://sample/skel/leg.skel");
  assert(world != nullptr);

  SkeletonPtr biped = world->getSkeleton("skeleton");
  for(size_t i = 0; i < biped->getNumJoints(); ++i)
    biped->getJoint(i)->setLimitEnforcement(true);

  // Enable self collision check but ignore adjacent bodies
  biped->enableSelfCollisionCheck();
  biped->disableAdjacentBodyCheck();

  return biped;
}


int main(int argc, char* argv[])
{
  SkeletonPtr floor = createFloor();
  biped = loadModel();

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

  // Initialize glut, initialize the window, and begin the glut event loop
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Multi-Pendulum Tutorial");
  glutMainLoop();
}
