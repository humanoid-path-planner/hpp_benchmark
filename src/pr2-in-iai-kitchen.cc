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
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/urdf/util.hh>

#include <hpp/core/problem-solver.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/plugin.hh>

#include "benchmark.hh"

using namespace hpp::pinocchio;
using namespace hpp::core;

namespace hpp {
namespace benchmark {
/**
\brief PR2 in IAI maps

See \ref hpp_benchmark_cpp_pr2_in_iai_maps "the results".

\ingroup hpp_benchmark_cpp
 */
class pr2_in_iai_maps : public BenchmarkCase {
  private:
    ProblemSolverPtr_t ps;
    Configuration_t q_init, q_goal;

  public:
    void setup(int)
    {
      ps = ProblemSolver::create ();

      // Create robot
      DevicePtr_t device = ps->createRobot ("pr2");
      hpp::pinocchio::urdf::loadModel (device, 0,
          "", // no prefix,
          "planar", // root joint type,
          "package://hpp_tutorial/urdf/pr2.urdf",
          "package://hpp_tutorial/srdf/pr2.srdf");
      device->controlComputation ((Computation_t)(JOINT_POSITION | JACOBIAN));
      ps->robot (device);

      device->rootJoint()->lowerBound (0, -4);
      device->rootJoint()->upperBound (0, -3);
      device->rootJoint()->lowerBound (1, -5);
      device->rootJoint()->upperBound (1, -3);

      // Add obstacle
      DevicePtr_t obstacle = Device::create ("kitchen");
      hpp::pinocchio::urdf::loadModel (obstacle, 0,
          "", // no prefix,
          "anchor", // root joint type,
          "package://hpp_tutorial/urdf/kitchen_area.urdf", "");
      obstacle->controlComputation (JOINT_POSITION);
      ps->addObstacle (obstacle, true, true);

      // Create initial and final configuration
      q_init = device->currentConfiguration();
      q_init.head<2>() << -3.2, -4;
      q_init[device->getJointByName("torso_lift_joint")->rankInConfiguration()] = 0.2;

      q_goal = q_init;
      q_goal[device->getJointByName("torso_lift_joint")->rankInConfiguration()] = 0;
      q_goal[device->getJointByName("l_shoulder_lift_joint")->rankInConfiguration()] =  0.5;
      q_goal[device->getJointByName("l_elbow_flex_joint"   )->rankInConfiguration()] = -0.5;
      q_goal[device->getJointByName("r_shoulder_lift_joint")->rankInConfiguration()] =  0.5;
      q_goal[device->getJointByName("r_elbow_flex_joint"   )->rankInConfiguration()] = -0.5;

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

REGISTER(::hpp::benchmark::pr2_in_iai_maps, pr2_in_iai_maps);
