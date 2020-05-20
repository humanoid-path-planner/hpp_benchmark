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

#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/pinocchio/center-of-mass-computation.hh>

#include <hpp/constraints/generic-transformation.hh>
#include <hpp/constraints/locked-joint.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/constraints/solver/by-substitution.hh>

#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/plugin.hh>

#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/handle.hh>
#include <hpp/manipulation/problem-solver.hh>
#include <hpp/manipulation/problem.hh>
#include <hpp/manipulation/srdf/util.hh>

#include "benchmark.hh"

using namespace hpp::manipulation;
using hpp::constraints::EqualToZero;


std::map<std::string, double>
  leftHandClosed = {
    {"LHand", 0.0},
    {"LFinger12", 0.0},
    {"LFinger13", 0.0},
    {"LFinger21", 0.0},
    {"LFinger22", 0.0},
    {"LFinger23", 0.0},
    {"LFinger31", 0.0},
    {"LFinger32", 0.0},
    {"LFinger33", 0.0},
    {"LThumb1", 0.0},
    {"LThumb2", 0.0},
    {"LThumb3", 0.0}},
  leftHandOpen = {
    {"LHand", 1},
    {"LFinger12", 1.06},
    {"LFinger13", 1.06},
    {"LFinger21", 1.06},
    {"LFinger22", 1.06},
    {"LFinger23", 1.06},
    {"LFinger31", 1.06},
    {"LFinger32", 1.06},
    {"LFinger33", 1.06},
    {"LThumb1", 0},
    {"LThumb2", 1.06},
    {"LThumb3", 1.06}},
  rightHandClosed = {
    {"RHand", 0.0},
    {"RFinger12", 0.0},
    {"RFinger13", 0.0},
    {"RFinger21", 0.0},
    {"RFinger22", 0.0},
    {"RFinger23", 0.0},
    {"RFinger31", 0.0},
    {"RFinger32", 0.0},
    {"RFinger33", 0.0},
    {"RThumb1", 0.0},
    {"RThumb2", 0.0},
    {"RThumb3", 0.0}},
  rightHandOpen = {
    {"RHand", 1},
    {"RFinger12", 1.06},
    {"RFinger13", 1.06},
    {"RFinger21", 1.06},
    {"RFinger22", 1.06},
    {"RFinger23", 1.06},
    {"RFinger31", 1.06},
    {"RFinger32", 1.06},
    {"RFinger33", 1.06},
    {"RThumb1", 0},
    {"RThumb2", 1.06},
    {"RThumb3", 1.06}};

namespace hpp {
namespace benchmark {

std::vector<std::string> createLockedJointForHandsOpen (ProblemSolverPtr_t ps, const std::string& prefix)
{
  std::vector<std::string> names;
  DevicePtr_t robot = ps->robot();
  vector_t q (1);
  for (const auto& dict : { rightHandOpen, leftHandOpen }) {
    for (const auto& pair : dict) {
      std::string n = prefix + pair.first;
      names.push_back(n);
      JointPtr_t joint = robot->getJointByName (n);
      q << pair.second;
      ps->addNumericalConstraint(n,
          constraints::LockedJoint::create(joint,
            joint->configurationSpace()->element(q)));
    }
  }
  return names;
}

std::vector<std::string> createConstraints (ProblemSolverPtr_t ps, Configuration_t q0,
    const std::string& leftAnkle, const std::string& rightAnkle)
{
  DevicePtr_t robot = ps->robot();
  JointPtr_t la = robot->getJointByName(leftAnkle ),
             ra = robot->getJointByName(rightAnkle);

  robot->currentConfiguration(q0);
  robot->computeForwardKinematics();

  Transform3f oMl = la->currentTransformation(),
              oMr = ra->currentTransformation();
  pinocchio::CenterOfMassComputationPtr_t com (
      pinocchio::CenterOfMassComputation::create (robot));
  com->add(robot->rootJoint());
  com->compute();
  std::vector<std::string> names;
  std::string name;

  // COM wrt left ankle frame
  name = "relative-com";
  names.push_back(name);
  ps->addNumericalConstraint(name, constraints::Implicit::create (
        constraints::RelativeCom::create (name, robot, com, la,
          oMl.actInv(com->com())
          ), 3 * EqualToZero));

  // Relative pose of the feet
  name = "pose-right-foot";
  names.push_back(name);
  ps->addNumericalConstraint(name, constraints::Implicit::create (
        constraints::Transformation::create (name, robot, ra,
          Transform3f::Identity(), oMr), 6 * EqualToZero));
  
  // Pose of the left foot
  name = "pose-left-foot";
  names.push_back(name);
  ps->addNumericalConstraint(name, constraints::Implicit::create (
        constraints::Transformation::create (name, robot, la,
          Transform3f::Identity(), oMl), 6 * EqualToZero));

  return names;
}

/**
\brief Comparison between implicit and explicit constraints

\anchor hpp_benchmark_cpp_implicit_versus_explicit
\par Comparison for left hand only.
\image html "cpp/implicit_versus_explicit_Explicit left hand.svg"
\image html "cpp/implicit_versus_explicit_Implicit left hand.svg"

\par Comparison for right hand only.
\image html "cpp/implicit_versus_explicit_Explicit both hands.svg"
\image html "cpp/implicit_versus_explicit_Implicit both hands.svg"

\par Comparison for both hands, so in the explicit case, one of the
constraint is solved implicitely.
\image html "cpp/implicit_versus_explicit_Explicit right hand.svg"
\image html "cpp/implicit_versus_explicit_Implicit right hand.svg"

\ingroup hpp_benchmark_cpp
 */
class implicit_versus_explicit : public BenchmarkNCase {
  private:
    ProblemSolverPtr_t ps;
    Configuration_t q_init, q_goal;

    std::vector<std::string> grippers;
    std::vector<std::string> handles;

    typedef std::pair<int,int> intint_t;
    std::map<intint_t, std::string> implicitGraspConstraints, explicitGraspConstraints;

    int nCfgPerCase;
    std::vector<Configuration_t, Eigen::aligned_allocator<Configuration_t> > qs;

    // Per iteration variables
    int iqs, nsuccess;
    core::ConfigProjectorPtr_t constraint;
    bool currentCaseIsExplicit;

  public:
    std::vector<std::string> names ()
    {
      std::vector<std::string> ns;
      ns.reserve(6);
      for (std::string type : { "Implicit ", "Explicit " })
        for (std::string hand : { "right hand", "left hand", "both hands" })
          ns.push_back(type + hand);
      return ns;
    }

    void setup(int NIter)
    {
      ps = ProblemSolver::create ();

      // Create robot
      DevicePtr_t device = Device::create ("romeo");
      pinocchio::urdf::loadModel (device, 0, "romeo", "freeflyer",
          "package://example-robot-data/robots/romeo_description/urdf/romeo.urdf",
          "package://example-robot-data/robots/romeo_description/srdf/romeo.srdf");
      srdf::loadModelFromFile (device, "romeo",
          "package://example-robot-data/robots/romeo_description/srdf/romeo.srdf");

      device->controlComputation ((pinocchio::Computation_t)(pinocchio::JOINT_POSITION | pinocchio::JACOBIAN));
      ps->robot (device);

      JointPtr_t joint = device->getJointByName("romeo/root_joint");
      joint->lowerBounds (vector3_t(-1., -1., 0.));
      joint->upperBounds (vector3_t( 1.,  1., 1.));

      // Add object
      pinocchio::urdf::loadModel (device, 0, "placard", "freeflyer",
          "package://hpp_environments/urdf/placard.urdf",
          "package://hpp_environments/srdf/placard.srdf");
      srdf::loadModelFromFile (device, "placard",
          "package://hpp_environments/srdf/placard.srdf");

      joint = device->getJointByName("placard/root_joint");
      joint->lowerBounds (vector3_t(-1., -1., 0. ));
      joint->upperBounds (vector3_t( 1.,  1., 1.5));

      // Create constraints
      Configuration_t q0 (device->model().referenceConfigurations["half_sitting"]);
      // Work around bug https://github.com/stack-of-tasks/pinocchio/issues/1178
      if (q0.size() == device->configSize() - 7) {
        q0.conservativeResize(device->configSize());
        q0.tail<7>() << 0, 0, 0, 0, 0, 0, 1;
      }
      std::vector<std::string> lockHands (createLockedJointForHandsOpen(ps, "romeo/"));
      std::vector<std::string> balance (createConstraints(ps, q0, "romeo/LAnkleRoll", "romeo/RAnkleRoll"));

      // There are 3 sets of constraints
      grippers = { "romeo/l_hand", "romeo/r_hand" };
      handles = { "placard/low", "placard/high" };
      int ig = 0, ih;
      for (std::string gripper : grippers) {
        GripperPtr_t _gripper = device->grippers.get (gripper);
        ih = 0;
        for (std::string handle : handles) {
          std::string name = gripper + " grasps " + handle;

          HandlePtr_t _handle = device->handles.get (handle);
          ps->createGraspConstraint(name, gripper, handle);
          explicitGraspConstraints[std::make_pair(ig,ih)] = name + "/hold";

          name = "Relative transformation " + gripper + "/" + handle;
          ps->addNumericalConstraint(name, constraints::Implicit::create (
                constraints::RelativeTransformation::create (name, device,
                  _gripper->joint(), _handle->joint(),
                  _gripper->objectPositionInJoint(), _handle->localPosition()),
                6 * EqualToZero));
          implicitGraspConstraints[std::make_pair(ig,ih)] = name;
          ++ih;
        }
        ++ig;
      }

      // Generate the random configurations
      nCfgPerCase = 100;
      time_scale = 1/(value_type)nCfgPerCase;
      qs.resize(nCfgPerCase * NIter);
      core::ConfigurationShooterPtr_t shooter (ps->problem()->configurationShooter());
      for (Configuration_t& q : qs) shooter->shoot(q);
    }

    void initializeProblem(int iIter, int iCase)
    {
      iqs = nCfgPerCase * iIter;
      constraint = core::ConfigProjector::create(ps->robot(), "constraint", 1e-4, 40);
      currentCaseIsExplicit = false;
      switch (iCase) {
        case 0:
          constraint->add (ps->numericalConstraints.get(
                implicitGraspConstraints[intint_t(1,1)]), 0);
          break;
        case 1:
          constraint->add (ps->numericalConstraints.get(
                implicitGraspConstraints[intint_t(0,0)]), 0);
          break;
        case 2:
          constraint->add (ps->numericalConstraints.get(
                implicitGraspConstraints[intint_t(0,0)]), 0);
          constraint->add (ps->numericalConstraints.get(
                implicitGraspConstraints[intint_t(1,1)]), 0);
          break;
        case 3:
          constraint->add (ps->numericalConstraints.get(
                explicitGraspConstraints[intint_t(1,1)]), 0);
          currentCaseIsExplicit = true;
          break;
        case 4:
          constraint->add (ps->numericalConstraints.get(
                explicitGraspConstraints[intint_t(0,0)]), 0);
          currentCaseIsExplicit = true;
          break;
        case 5:
          constraint->add (ps->numericalConstraints.get(
                explicitGraspConstraints[intint_t(0,0)]), 0);
          constraint->add (ps->numericalConstraints.get(
                explicitGraspConstraints[intint_t(1,1)]), 0);
          break;
        default:
          throw std::invalid_argument("invalid case");
          break;
      }
      //std::cout << *constraint << std::endl;
    }

    void solveProblem()
    {
      nsuccess = 0;
      for (int i = iqs; i < iqs + nCfgPerCase; ++i) {
        Configuration_t q (qs[i]);
        bool success = constraint->apply(q);
        if (success) ++nsuccess;
      }
    }

    void saveResolutionResult(results_t& results)
    {
      results["success ratio"].push_back(static_cast<value_type>(nsuccess)
          / static_cast<value_type>(nCfgPerCase));
    }

    virtual bool validateSolution ()
    {
      if (currentCaseIsExplicit) {
        if (constraint->solver().reducedDimension() != 0) return false;
        // Explicit resolution should always succeed.
        if (nsuccess != nCfgPerCase) return false;
      }
      return true;
    }

    void clean()
    {
      delete ps;
      constraint.reset();
      qs.clear();
    }
}; // class Resolution

} // namespace benchmark
} // namespace hpp

REGISTER(::hpp::benchmark::implicit_versus_explicit, implicit_versus_explicit);
