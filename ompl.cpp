/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Mark Moll */
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <pathserver.grpc.pb.h>
#include <cmath>
#include <collision.hpp>
#include <fstream>
#include <optional>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Objective that minimizes the number of states
class StateMinimizationObjective : public ob::StateCostIntegralObjective {
 public:
  // We don't need motion interpolation.
  StateMinimizationObjective(const ob::SpaceInformationPtr& si)
      : ob::StateCostIntegralObjective(si, true) {}

  ob::Cost stateCost(const ob::State* s) const override { return ob::Cost{1.}; }
};

using OptDirection = std::optional<pathserver::PlanReply_Move_Direction>;
constexpr OptDirection map_direction(
    ob::ReedsSheppStateSpace::ReedsSheppPathSegmentType t) {
  using PMD = pathserver::PlanReply_Move_Direction;

  constexpr OptDirection map[] = {
      OptDirection{}, OptDirection{PMD::PlanReply_Move_Direction_LEFT},
      OptDirection{PMD::PlanReply_Move_Direction_STRAIGHT},
      OptDirection{PMD::PlanReply_Move_Direction_RIGHT}};

  return map[t];
}

using Move = pathserver::PlanReply_Move;
/**
 * Generate moves for all states in the path.
 *
 * \param[in] s state space
 * \param[in] p path that involves transitions between states in \c s.
 * \param[out] out destination where moves will be appended to. Not cleared,
 *  but the last state may be modified during path simplification.
 *
 * \pre \c p.getStateCount() >= 2 (i.e. when path is valid).
 */
void generate_moves(const ob::ReedsSheppStateSpace& s,
                    const og::PathGeometric& p,
                    double turn_radius,
                    std::vector<Move>& out) {
  auto initial_state = p.getState(0);
  for (std::size_t i{1}; i < p.getStateCount(); ++i) {
    auto target_state = p.getState(i);
    auto path = s.reedsShepp(initial_state, target_state);

    for (std::size_t ps{}; ps < 5; ++ps) {
      auto dir = map_direction(path.type_[ps]);
      if (!dir.has_value())
        continue;

      double length_m = path.length_[ps] * turn_radius;
      // Skip _actual_ zero length paths.
      if (std::fpclassify(length_m) == FP_ZERO)
        continue;

      if (out.size() && (out.back().direction() == *dir)) {
        // Attach move onto the previous move.
        out.back().set_distance(out.back().distance() + length_m);
      } else {
        // Add a new move.
        Move added{};
        added.set_distance(length_m);
        added.set_direction(*dir);
        out.emplace_back(added);
      }
    }

    initial_state = target_state;
  }
}

class PathServerImpl final : public pathserver::PathServer::Service {
 public:
  grpc::Status Plan(grpc::ServerContext* context,
                    const pathserver::PlanRequest* request,
                    pathserver::PlanReply* response) override {
    // Setup collision detector.
    CollisionDetector cdet{0.15, 0.10, 0.10, 0.10};
    for (const auto& ob : request->obstacles()) {
      cdet.add_obstacle(ob.x() + 0.05, ob.y() + 0.05);
    }

    // Setup OMPL
    ob::StateSpacePtr space = std::make_shared<ob::ReedsSheppStateSpace>(0.337);
    ob::ScopedState<> start{space}, goal{space};

    // Setup bounding box.
    ob::RealVectorBounds bounds{2};
    bounds.setLow(0);
    bounds.setHigh(WORLD_LENGTH);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    og::SimpleSetup ss{space};

    // Set state validity checking.
    const ob::SpaceInformation* si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker([cdet, si](const ob::State* state) {
      const auto* s = state->as<ob::SE2StateSpace::StateType>();
      // Validate for collisions and whether the robot is within its boundaries.
      return (!cdet.check_collision(s->getX(), s->getY(), s->getYaw())) &&
             (si->satisfiesBounds(state));
    });

    // Configure RRT* planner.
    ob::PlannerPtr planner =
        std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    // TODO: only use if the default paths aren't great.
    // Minimize path length while minimizing number of states.
    // ob::OptimizationObjectivePtr obj = 10 *
    // std::make_shared<StateMinimizationObjective>(ss.getSpaceInformation()) +
    //                                 std::make_shared<ob::PathLengthOptimizationObjective>(ss.getSpaceInformation());
    // ss.setOptimizationObjective(obj);

    // Set start and goal states.
    start[0] = request->current().x();
    start[1] = request->current().y();
    start[2] = request->current().theta();
    goal[0] = request->target().x();
    goal[1] = request->target().y();
    goal[2] = request->target().theta();
    ss.setStartAndGoalStates(start, goal);

    // State check at .5% resolution.
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    ss.setup();

    // Attempt to solve within 2s.
    ob::PlannerStatus solved = ss.solve(2);

    // If an approximate or exact solution is achieved.
    if (solved) {
      // Use one second to simplify the solution.
      ss.simplifySolution(1.0);
      og::PathGeometric path = ss.getSolutionPath();

      // Not very efficient as we are doing multiple allocations.
      // But it's nice for debugging.
      std::vector<Move> moves{};
      generate_moves(*(space->as<ob::ReedsSheppStateSpace>()), path, .337,
                     moves);
      for (auto& move : moves) {
        auto mp = response->add_moves();
        mp->CopyFrom(move);
      }

      for (std::size_t i{}; i < path.getStateCount(); ++i) {
        const auto* s = path.getState(i)->as<ob::SE2StateSpace::StateType>();
        auto sp = response->add_waypoints();
        sp->set_x(s->getX());
        sp->set_y(s->getY());
        sp->set_theta(s->getYaw());
      }

      return grpc::Status::OK;
    } else
      return grpc::Status{grpc::StatusCode::DEADLINE_EXCEEDED,
                          "no solution found within planning time limit"};
  }
};

int main(int, char**) {
  std::string addr{"0.0.0.0:10003"};
  PathServerImpl service{};

  grpc::ServerBuilder builder{};
  builder.AddListeningPort(addr, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);

  auto server = builder.BuildAndStart();
  server->Wait();

  return 0;
}
