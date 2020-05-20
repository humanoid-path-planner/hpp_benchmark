// Copyright (c) 2020, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-benchmark.
// hpp-benchmark is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-benchmark is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-benchmark. If not, see <http://www.gnu.org/licenses/>.

#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/center-of-mass-computation.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/urdf/util.hh>

#include <hpp/constraints/generic-transformation.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/constraints/comparison-types.hh>
#include <hpp/constraints/locked-joint.hh>
#include <hpp/constraints/relative-com.hh>

#include <hpp/core/problem-solver.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/plugin.hh>

#include "benchmark.hh"

using namespace hpp::pinocchio;
using namespace hpp::core;
using hpp::constraints::EqualToZero;

namespace hpp {
namespace benchmark {

void addLockedJoint(ProblemSolverPtr_t ps, const std::string& name, std::vector<value_type> value)
{
  vector_t vector (Eigen::Map<vector_t>(value.data(), value.size()));
  JointPtr_t joint = ps->robot()->getJointByName (name);
  ps->addNumericalConstraint(name,
      constraints::LockedJoint::create(joint,
        joint->configurationSpace()->element(vector)));
}

void createConstraints (ProblemSolverPtr_t ps, Configuration_t q0,
    const std::string& leftAnkle, const std::string& rightAnkle)
{
  DevicePtr_t robot = ps->robot();
  JointPtr_t la = robot->getJointByName(leftAnkle ),
             ra = robot->getJointByName(rightAnkle);

  robot->currentConfiguration(q0);
  robot->computeForwardKinematics();

  Transform3f oMl = la->currentTransformation(),
              oMr = ra->currentTransformation();
  CenterOfMassComputationPtr_t com (CenterOfMassComputation::create (robot));
  com->add(robot->rootJoint());
  com->compute();
  std::string name;

  // COM wrt left ankle frame
  name = "relative-com";
  ps->addNumericalConstraint(name, constraints::Implicit::create (
        constraints::RelativeCom::create (name, robot, com, la,
          oMl.actInv(com->com())
          ), 3 * EqualToZero));

  // Relative pose of the feet
  name = "relative-pose";
  ps->addNumericalConstraint(name, constraints::Implicit::create (
        constraints::RelativeTransformation::create (name, robot, la, ra, 
          Transform3f::Identity(), oMr.actInv(oMl)
          ), 6 * EqualToZero));
  
  // Pose of the left foot
  name = "pose-left-foot";
  ps->addNumericalConstraint(name, constraints::Implicit::create (
        constraints::Transformation::create (name, robot, la,
          Transform3f::Identity(), oMl,
          std::vector<bool>{false,false,true,true,true,false}
          ), 3 * EqualToZero));

  // Complement left foot
  // unused at the moment.


    // left hand closed
  addLockedJoint (ps, "LARM_JOINT6" , { 0.1, } );
  addLockedJoint (ps, "LHAND_JOINT0", { 0.0, } );
  addLockedJoint (ps, "LHAND_JOINT1", { 0.0, } );
  addLockedJoint (ps, "LHAND_JOINT2", { 0.0, } );
  addLockedJoint (ps, "LHAND_JOINT3", { 0.0, } );
  addLockedJoint (ps, "LHAND_JOINT4", { 0.0, } );

    // right hand closed
  addLockedJoint (ps, "RARM_JOINT6" , { 0.1, } );
  addLockedJoint (ps, "RHAND_JOINT0", { 0.0, } );
  addLockedJoint (ps, "RHAND_JOINT1", { 0.0, } );
  addLockedJoint (ps, "RHAND_JOINT2", { 0.0, } );
  addLockedJoint (ps, "RHAND_JOINT3", { 0.0, } );
  addLockedJoint (ps, "RHAND_JOINT4", { 0.0, } );
}

/**
\brief HRP2 on the ground

See \ref hpp_benchmark_cpp_hrp2_on_the_ground "the results".

\ingroup hpp_benchmark_cpp
 */
class hrp2_on_the_ground : public BenchmarkCase {
  private:
    ProblemSolverPtr_t ps;
    Configuration_t q_init, q_goal;

  public:
    void setup(int)
    {
      ps = ProblemSolver::create ();

      // Create robot
      DevicePtr_t device = ps->createRobot ("hrp2_14");
      urdf::loadModel (device, 0,
          "", // no prefix,
          "freeflyer", // root joint type,
          "package://hrp2_14_description/urdf/hrp2_14_capsule.urdf",
          "package://hrp2_14_description/srdf/hrp2_14_capsule.srdf");
      device->controlComputation ((Computation_t)(JOINT_POSITION | JACOBIAN));
      ps->robot (device);

      device->rootJoint()->lowerBound (0, -3);
      device->rootJoint()->upperBound (0,  3);
      device->rootJoint()->lowerBound (1, -3);
      device->rootJoint()->upperBound (1,  3);
      device->rootJoint()->lowerBound (2, 0);
      device->rootJoint()->upperBound (2, 1);

      // Create and add constraints
      Configuration_t q0 (device->model().referenceConfigurations["half_sitting"]);
      q0[2] = 0.648702;
      createConstraints(ps, q0, "LLEG_JOINT5", "RLEG_JOINT5");
      ps->addNumericalConstraintToConfigProjector ("proj", "relative-com", 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "relative-pose", 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "pose-left-foot", 0);

      ps->addNumericalConstraintToConfigProjector ("proj", "LARM_JOINT6" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "LHAND_JOINT0", 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "LHAND_JOINT1", 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "LHAND_JOINT2", 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "LHAND_JOINT3", 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "LHAND_JOINT4", 0);

      ps->addNumericalConstraintToConfigProjector ("proj", "RARM_JOINT6" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "RHAND_JOINT0", 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "RHAND_JOINT1", 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "RHAND_JOINT2", 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "RHAND_JOINT3", 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "RHAND_JOINT4", 0);

      // Create initial and final configuration
      q_init.resize(device->configSize());
      q_init <<
        0.0, 0.0, 0.705, 0., 0., 0., 1.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0;
      if (!ps->constraints ()->apply (q_init))
        throw std::runtime_error("Failed to apply constraint to init configuration.");

      q_goal.resize(device->configSize());
      q_goal <<
        0.0, 0.0, 0.705, 0., 0., 0., 1., 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0;
      if (!ps->constraints ()->apply (q_goal))
        throw std::runtime_error("Failed to apply constraint to goal configuration.");

      ps->pathValidationType("Progressive", 0.025);

      ps->initConfig (ConfigurationPtr_t(new Configuration_t(q_init)));
      ps->addGoalConfig (ConfigurationPtr_t(new Configuration_t(q_goal)));
    }

    void initializeProblem(int)
    {
      ps->resetRoadmap();
    }

    void solveProblem()
    {
      ps->solve();
    }

    void saveResolutionResult(results_t& results)
    {
      results["Number nodes"].push_back(static_cast<value_type>(ps->roadmap()->nodes().size()));
    }

    virtual bool validateSolution ()
    {
      //TODO
      return true;
    }

    void clean()
    {
      delete ps;
    }
}; // class Resolution

} // namespace benchmark
} // namespace hpp

REGISTER(::hpp::benchmark::hrp2_on_the_ground, hrp2_on_the_ground);
