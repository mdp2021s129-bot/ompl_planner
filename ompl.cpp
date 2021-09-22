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
#include <collision.hpp>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <cmath>
#include <fstream>
#include <optional>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Objective that minimizes the number of states
class StateMinimizationObjective: public ob::StateCostIntegralObjective {
public:
    // We don't need motion interpolation.
    StateMinimizationObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true) {}
 
    ob::Cost stateCost(const ob::State *s) const override {
        return ob::Cost{1.};
    }
};

/**
 * Turning directions for the robot.
 */
enum Direction {
    STRAIGHT = 0,
    LEFT,
    RIGHT
};

std::ostream &operator<<(std::ostream &os, const Direction d) {
    static constexpr const char *map[] = {"STRAIGHT", "LEFT", "RIGHT"};

    return os << map[d];
}

using OptDirection = std::optional<Direction>;
constexpr OptDirection map_direction(ob::ReedsSheppStateSpace::ReedsSheppPathSegmentType t) {

    constexpr OptDirection map[] = {
        OptDirection{}, OptDirection{Direction::LEFT}, 
        OptDirection{Direction::STRAIGHT}, OptDirection{Direction::RIGHT}
    };

    return map[t];
}

/**
 * Moves for the robot.
 */
struct Move {
    /**
     * Direction for the move.
     */
    Direction dir;
    /**
     * Move distance. In metres.
     */
    double distance;
};

std::ostream &operator<<(std::ostream &os, const Move &m) {
    return os << m.dir << ' ' << m.distance;
}


/**
 * Generate moves for all states in the path.
 * 
 * \param[in] s state space
 * \param[in] p path that involves transitions between states in \c s.
 * \param[out] out destination where moves will be appended to. Not cleared,
 *  but the last state may be modified during path simplification.
 * 
 * \pre \c p.getStateCount() >= 2
 */
void generate_moves(const ob::ReedsSheppStateSpace &s, 
                    const og::PathGeometric &p,
                    double turn_radius,
                    std::vector<Move> &out) {
    
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

            if (out.size() && (out.back().dir == *dir)) {
                // Attach move onto the previous move.
                out.back().distance += length_m;
            } else {
                // Make a new move.
                out.emplace_back(*dir, length_m);
            }
        }

        initial_state = target_state;
    }
}

/**
 * Print moves to an output stream.
 * 
 * \param mv moves.
 * \param to output stream to write to.
 */
std::ostream &print_moves(const std::vector<Move> &mv, std::ostream &to) {
    for (auto &move : mv) {
        to << move << '\n';
    }

    return to;
}

void plan(const ob::StateSpacePtr &space)
{
    ob::ScopedState<> start(space), goal(space);
    
    // Setup bounding box.
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(WORLD_LENGTH);

    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // Setup collision detector.
    CollisionDetector cdet{0.15, 0.10, 0.10, 0.10};
    // Add obstacles to collision detector.
    cdet.add_obstacle(0.25, 0.65); // (2.6)
    cdet.add_obstacle(0.55, 0.75); // (5.7)
    cdet.add_obstacle(0.95, 1.05); // (9.10)

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    const ob::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker([cdet, si](const ob::State *state)
                               { 
                                   const auto *s = state->as<ob::SE2StateSpace::StateType>();
                                   return (!cdet.check_collision(s->getX(), s->getY(), s->getYaw())) && (si->satisfiesBounds(state));
                               });

    //Configure RRT* planner.
    ob::PlannerPtr planner = std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());

    // Minimize path length while minimizing number of states.
    ob::OptimizationObjectivePtr obj = 10 * std::make_shared<StateMinimizationObjective>(ss.getSpaceInformation()) + 
        std::make_shared<ob::PathLengthOptimizationObjective>(ss.getSpaceInformation());
    // ss.setOptimizationObjective(obj);
    ss.setPlanner(planner);

    // Set start and goal states.
    // (.15, .15) 90 deg
    start[0] = start[1] = .15;
    start[2] = .5 * M_PI;
    
    // In front of obstacle (5.10)
    goal[0] = .55;
    goal[1] = 1.0;
    goal[2] = 0 * M_PI;
    ss.setStartAndGoalStates(start, goal);

    // this call is optional, but we put it in to get more output information
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    ss.setup();
    ss.print();

    // attempt to solve the problem within 2 seconds of planning time
    ob::PlannerStatus solved = ss.solve(2);

    if (solved)
    {
        std::vector<double> reals;

        std::cout << "Found solution:" << std::endl;
        // Use one second to simplify the solution.
        ss.simplifySolution(1.0);
        og::PathGeometric path = ss.getSolutionPath();

        std::vector<Move> moves{};
        generate_moves(*(space->as<ob::ReedsSheppStateSpace>()), path, .337, moves);
        
        std::ofstream out_moves{"moves.csv"};
        print_moves(moves, out_moves);

        std::ofstream out_states{"states.csv"};
        path.printAsMatrix(out_states);

        std::ofstream out_interp{"path.csv"};
        path.interpolate(1000);
        path.printAsMatrix(out_interp);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int argc, char *argv[])
{
    try
    {
        auto space = std::make_shared<ob::ReedsSheppStateSpace>(0.337);
        plan(space);
    }
    catch (std::exception &e)
    {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    catch (...)
    {
        std::cerr << "Exception of unknown type!\n";
    }

    return 0;
}