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

  // lock hands and torso
  addLockedJoint (ps, "gripper_left_inner_double_joint" , { 0, } );
  addLockedJoint (ps, "gripper_left_fingertip_1_joint" , { 0, } );
  addLockedJoint (ps, "gripper_left_fingertip_2_joint" , { 0, } );
  addLockedJoint (ps, "gripper_left_inner_single_joint" , { 0, } );
  addLockedJoint (ps, "gripper_left_fingertip_3_joint" , { 0, } );
  addLockedJoint (ps, "gripper_left_joint" , { 0, } );
  addLockedJoint (ps, "gripper_left_motor_single_joint" , { 0, } );
  addLockedJoint (ps, "gripper_right_inner_double_joint" , { 0, } );
  addLockedJoint (ps, "gripper_right_fingertip_1_joint" , { 0, } );
  addLockedJoint (ps, "gripper_right_fingertip_2_joint" , { 0, } );
  addLockedJoint (ps, "gripper_right_inner_single_joint" , { 0, } );
  addLockedJoint (ps, "gripper_right_fingertip_3_joint" , { 0, } );
  addLockedJoint (ps, "gripper_right_joint" , { 0, } );
  addLockedJoint (ps, "gripper_right_motor_single_joint" , { 0, } );
  addLockedJoint (ps, "torso_1_joint" , { 0, } );
  addLockedJoint (ps, "torso_2_joint" , { 0.006761, } );
}

/**
\brief Pyrene on the ground

See \ref hpp_benchmark_cpp_pyrene_on_the_ground "the results".

\ingroup hpp_benchmark_cpp
 */
class pyrene_on_the_ground : public BenchmarkCase {
  private:
    ProblemSolverPtr_t ps;
    Configuration_t q_init, q_goal;

  public:
    void setup(int)
    {
      ps = ProblemSolver::create ();

      // Create robot
      DevicePtr_t device = ps->createRobot ("pyrene");
      urdf::loadModel (device, 0,
          "", // no prefix,
          "freeflyer", // root joint type,
	  "package://example-robot-data/robots/talos_data/robots/"
	  "talos_full_v2.urdf",
          "package://example-robot-data/robots/talos_data/srdf/talos.srdf");
      device->controlComputation ((Computation_t)(JOINT_POSITION | JACOBIAN));
      ps->robot (device);

      device->rootJoint()->lowerBound (0, -3);
      device->rootJoint()->upperBound (0,  3);
      device->rootJoint()->lowerBound (1, -3);
      device->rootJoint()->upperBound (1,  3);
      device->rootJoint()->lowerBound (2, 0);
      device->rootJoint()->upperBound (2, 1);

      // Create and add constraints
      Configuration_t q0 (device->configSize());
      q0 << 0, 0, 1.0192720229567027, 0, 0, 0, 1, 0.0, 0.0, -0.411354, 0.859395,
	-0.448041, -0.001708, 0.0, 0.0, -0.411354, 0.859395, -0.448041,
	-0.001708, 0, 0.006761, 0.25847, 0.173046, -0.0002, -0.525366, 0, 0,
	0.1, 0, 0, 0, 0, 0, 0, 0, -0.25847, -0.173046, 0.0002, -0.525366, 0, 0,
	0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
      createConstraints(ps, q0, "leg_left_6_joint", "leg_right_6_joint");
      ps->addNumericalConstraintToConfigProjector ("proj", "relative-com", 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "relative-pose", 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "pose-left-foot", 0);

      ps->addNumericalConstraintToConfigProjector ("proj","gripper_left_inner_double_joint" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "gripper_left_fingertip_1_joint" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "gripper_left_fingertip_2_joint" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "gripper_left_inner_single_joint" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "gripper_left_fingertip_3_joint" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "gripper_left_joint" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "gripper_left_motor_single_joint" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "gripper_right_inner_double_joint" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "gripper_right_fingertip_1_joint" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "gripper_right_fingertip_2_joint" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "gripper_right_inner_single_joint" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "gripper_right_fingertip_3_joint" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "gripper_right_joint" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "gripper_right_motor_single_joint" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "torso_1_joint" , 0);
      ps->addNumericalConstraintToConfigProjector ("proj", "torso_2_joint" , 0);


      // Create initial and final configuration
      q_init.resize(device->configSize());
      q_init <<
        0.46186525119743765, 0.7691484390667176, 1.0, 0.044318662659833724,
	-0.0108631325758057, -0.0005624014939687202, 0.9989582234484302,
	0.007182038754728065, -0.07804157609345573, -0.45119414082769843,
	0.9175221606997885, -0.44402665063685365, -0.012200787668632173,
	0.007200628661317587, -0.0788724231291963, -0.4956000845048934,
	1.009916799073695, -0.49201388832345117, -0.011369733964913645, 0.0,
	0.006761, 0.2408808670823526, 0.28558871367875255, 0.021347338765649856,
	-0.5979935578118766, -0.0014717027925961507, 0.006759032911476202,
	0.08832103581416396, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	1.0392843760345567, -0.10575191252250002, -0.05668798069441503,
	-1.7498341362736458, 0.0022744473854138473, 0.0015716871843022243,
	0.07078184761729372, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	-0.00020052684970875304, 0.00019305414086825983;
      if (!ps->constraints ()->apply (q_init))
        throw std::runtime_error("Failed to apply constraint to init configuration.");

      q_goal.resize(device->configSize());
      q_goal <<
        -0.03792613133823603, 0.24583989035169654, 1.0, 0.008581421388221004,
	0.044373915255123464, -0.0006050481369481731, 0.9989779520933587,
	0.0011692308149178052, -0.011583002652064677, -0.5522315456639073,
	0.9525259684676938, -0.4890594525896807, -0.007366718494771048,
	0.0011679806161439602, -0.01159704912053673, -0.5610095845869443,
	0.9704046648090222, -0.49816012449736746, -0.007352616506901346, 0.0,
	0.006761, 0.25575424894162485, 0.21391256924828497,
	0.006460912367916318, -0.5673886888192637, -0.0007964566272850148,
	0.0027266557203091918, 0.09323792816834059, 0.0, 0.0, 0.0, 0.0, 0.0,
	0.0, 0.0, -0.5020992312243198, -0.770681876188843, 2.42600766027,
	-1.8794064100743089, 0.0019251455338804122, 0.007445905986286772,
	0.06939811636044525, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	-0.0013061717130372818, 4.856617592856522e-05;
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

REGISTER(::hpp::benchmark::pyrene_on_the_ground, pyrene_on_the_ground);
