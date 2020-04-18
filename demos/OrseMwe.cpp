/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, University of Oxford
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
 *   * Neither the name of the University of Toronto nor the names of its
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

/* Author: Marlin Strub */

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include <boost/program_options.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/util/Console.h>

using seconds_d = std::chrono::duration<double>;

// The planner types to be tested.
enum class PLANNER_TYPE
{
    BITSTAR,
    RRTSTAR,
    RRTSHARP
};

std::ostream &operator<<(std::ostream &out, PLANNER_TYPE plannerType);

// The state validity checker. For this demo, our robot's state space lies in [0,1]x[0,1], with a circular obstacle of
// radius 0.25 centered at (0.5,0.5). Any states lying in this circular region are considered "in collision".
class DemoChecker : public ompl::base::StateValidityChecker
{
public:
    DemoChecker(const ompl::base::SpaceInformationPtr &spaceInfo) : ompl::base::StateValidityChecker(spaceInfo)
    {
    }

    // Returns whether the state is in the circular obstacle.
    bool isValid(const ompl::base::State *state) const override
    {
        double x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
        double y = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
        return sqrt((x - 0.5) * (x - 0.5) + (y - 0.5) * (y - 0.5)) > 0.25;
    }
};

// The planner factory.
std::unique_ptr<ompl::base::Planner> createPlanner(PLANNER_TYPE plannerType,
                                                   const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo);

// A utility to get the current best cost of a planner.
ompl::base::Cost getBestCost(ompl::base::Planner *planner, PLANNER_TYPE plannerType);

// Parse the command-line arguments.
void parseArgs(int argc, char **argv, double *logTime, double *runTime, std::size_t *numRuns);

int main(int argc, char **argv)
{
    // Parse the command-line arguments.
    double logTime{0.001};
    double runTime{1.0};
    std::size_t numRuns{100u};
    parseArgs(argc, argv, &logTime, &runTime, &numRuns);

    // Set the log level to not be so verbose.
    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

    // Construct the robot state space in which we're planning. We're planning in [0,1]x[0,1], a subset of R^2.
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(2);
    space->setBounds(0.0, 1.0);

    // Construct a space information instance for this state space.
    auto spaceInfo = std::make_shared<ompl::base::SpaceInformation>(space);
    spaceInfo->setStateValidityChecker(std::make_shared<DemoChecker>(spaceInfo));
    spaceInfo->setup();

    // Set our robot's starting state to be the bottom-left corner of the environment, i.e., (0,0).
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space);
    start[0] = 0.0;
    start[1] = 0.0;

    // Set our robot's goal state to be the top-right corner of the environment, i.e., (1,1).
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space);
    goal[0] = 1.0;
    goal[1] = 1.0;

    // Create a vector with the planner types that are to be tested in the order they are to be tested in..
    std::vector<PLANNER_TYPE> plannerTypes{PLANNER_TYPE::RRTSTAR,  PLANNER_TYPE::RRTSTAR, PLANNER_TYPE::RRTSHARP,
                                           PLANNER_TYPE::RRTSHARP, PLANNER_TYPE::RRTSTAR, PLANNER_TYPE::BITSTAR,
                                           PLANNER_TYPE::BITSTAR};

    // Prepare a vector to hold all of the planners' initial solution times.
    std::vector<std::vector<seconds_d>> initialSolutionTimes(plannerTypes.size(), std::vector<seconds_d>());
    for (auto &plannerTimes : initialSolutionTimes)
    {
        plannerTimes.reserve(numRuns);
    }

    // Loop over the numebr of runs.
    for (std::size_t run = 0u; run < numRuns; ++run)
    {
        std::cout << std::setw(4) << run << std::fixed;

        // Loop over the planners to be tested.
        for (std::size_t i = 0u; i < plannerTypes.size(); ++i)
        {
            // Create the planner.
            auto plannerType = plannerTypes.at(i);
            auto planner = createPlanner(plannerType, spaceInfo);

            // Create a problem instance.
            auto problem = std::make_shared<ompl::base::ProblemDefinition>(spaceInfo);

            // Set the start and goal states
            problem->setStartAndGoalStates(start, goal);

            // Set the objective.
            auto objective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(spaceInfo);
            objective->setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
            problem->setOptimizationObjective(objective);

            // Set the problem instance for our planner to solve
            planner->setProblemDefinition(problem);

            // Setup the planner.
            auto startSetupTime = std::chrono::steady_clock::now();
            planner->setup();
            seconds_d setupTime = std::chrono::steady_clock::now() - startSetupTime;

            // Fot this MWE, we only care about the initial solution time, which is why we can stop storing time after
            // each planner has a solution.
            bool hasSolution = false;

            // Solve the problem on a separate thread.
            std::chrono::steady_clock::time_point measurementStart;
            const auto solveStartTime = std::chrono::steady_clock::now();
            std::future<void> future =
                std::async(std::launch::async, [&planner, &runTime]() { planner->solve(runTime); });

            // Log the intermediate best costs.
            do
            {
                measurementStart = std::chrono::steady_clock::now();
                if (objective->isFinite(getBestCost(planner.get(), plannerType)) && !hasSolution)
                {
                    initialSolutionTimes.at(i).push_back(
                        (setupTime + (std::chrono::steady_clock::now() - solveStartTime)));
                    hasSolution = true;
                }

            } while (future.wait_until(measurementStart + seconds_d(logTime)) != std::future_status::ready);
            future.get();

            std::cout << std::setw(8) << plannerType << std::setw(10) << std::setprecision(6)
                      << initialSolutionTimes.at(i).back().count();
        }
        std::cout << "\n";
    }

    // Report the results.
    std::cout << "Median initial solution times of " << numRuns << " runs:\n";
    for (std::size_t i = 0u; i < plannerTypes.size(); ++i)
    {
        // Get the planner type.
        auto plannerType = plannerTypes.at(i);

        // Get the median initial solution cost. Yes, this is technically not the median if the number of runs is an
        // even number, but for the sake of this MWE it's fine.
        auto medianRun = initialSolutionTimes.at(i).begin() + (numRuns - 1u) / 2;
        std::nth_element(initialSolutionTimes.at(i).begin(), medianRun, initialSolutionTimes.at(i).end());

        std::cout << "    " << std::setw(8) << plannerType << std::setw(9) << std::setprecision(6)
                  << medianRun->count();
    }
    std::cout << '\n';

    return 0;
}

/** Parse the command line arguments. */
void parseArgs(int argc, char **argv, double *logTime, double *runTime, std::size_t *numRuns)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()("runtime,t", bpo::value<double>()->default_value(1.0),
                       "(Optional) Specify the runtime in seconds. Defaults to 1 and "
                       "must be greater than 0.")("logtime,l", bpo::value<double>()->default_value(0.00001),
                                                  "(Optional) Specify the logtime in seconds. Defaults to 0.00001 and "
                                                  "must be greater than 0.")(
        "runs,r", bpo::value<std::size_t>()->default_value(100u), "(Optional) Specify the number of runs per planner.");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    // Get the runtime as a double
    *runTime = vm["runtime"].as<double>();

    // Get the runtime as a double
    *logTime = vm["logtime"].as<double>();

    // Get the runtime as a double
    *numRuns = vm["runs"].as<std::size_t>();

    // Sanity checks
    if (*runTime <= 0.0)
    {
        std::cout << "Invalid runtime.\n\n" << desc << std::endl;
        throw std::runtime_error("Invalid runtime.");
    }
    if (*logTime <= 0.0)
    {
        std::cout << "Invalid logtime.\n\n" << desc << std::endl;
        throw std::runtime_error("Invalid logtime.");
    }
    if (*numRuns < 1u)
    {
        std::cout << "Invalid number of runs.\n\n" << desc << std::endl;
        throw std::runtime_error("Invalid number of runs.");
    }
}

std::unique_ptr<ompl::base::Planner> createPlanner(PLANNER_TYPE plannerType,
                                                   const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo)
{
    switch (plannerType)
    {
        case PLANNER_TYPE::BITSTAR:
        {
            return std::make_unique<ompl::geometric::BITstar>(spaceInfo);
        }
        case PLANNER_TYPE::RRTSHARP:
        {
            return std::make_unique<ompl::geometric::RRTsharp>(spaceInfo);
        }
        case PLANNER_TYPE::RRTSTAR:
        {
            return std::make_unique<ompl::geometric::RRTstar>(spaceInfo);
        }
        default:
        {
            throw std::runtime_error("Unsupported planner type.");
        }
    }
}

ompl::base::Cost getBestCost(ompl::base::Planner *planner, PLANNER_TYPE plannerType)
{
    switch (plannerType)
    {
        case PLANNER_TYPE::BITSTAR:
        {
            return planner->as<ompl::geometric::BITstar>()->bestCost();
        }
        case PLANNER_TYPE::RRTSHARP:
        {
            return planner->as<ompl::geometric::RRTsharp>()->bestCost();
        }
        case PLANNER_TYPE::RRTSTAR:
        {
            return planner->as<ompl::geometric::RRTstar>()->bestCost();
        }
        default:
        {
            throw std::runtime_error("Unsupported planner type.");
        }
    }
}

std::ostream &operator<<(std::ostream &out, PLANNER_TYPE plannerType)
{
    switch (plannerType)
    {
        case PLANNER_TYPE::BITSTAR:
        {
            return out << "BIT*";
        }
        case PLANNER_TYPE::RRTSHARP:
        {
            return out << "RRT#";
        }
        case PLANNER_TYPE::RRTSTAR:
        {
            return out << "RRT*";
        }
        default:
        {
            throw std::runtime_error("Unsupported planner type.");
        }
    }
}

/// @endcond
