/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Georgia Institute of Technology
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

/* Author: Florian Hauer */

#include "ompl/geometric/planners/rrt/RRTXdynamic.h"
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include "ompl/base/Goal.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/GeometricEquations.h"

ompl::geometric::RRTXdynamic::RRTXdynamic(const base::SpaceInformationPtr &si)
  : base::Planner(si, "RRTXdynamic")
  , mc_(opt_, pdef_)
  , q_(mc_)
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.canReportIntermediateSolutions = true;

    Planner::declareParam<double>("range", this, &RRTXdynamic::setRange, &RRTXdynamic::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTXdynamic::setGoalBias, &RRTXdynamic::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("epsilon", this, &RRTXdynamic::setEpsilon, &RRTXdynamic::getEpsilon, "0.:.01:10.");
    Planner::declareParam<double>("rewire_factor", this, &RRTXdynamic::setRewireFactor, &RRTXdynamic::getRewireFactor,
                                  "1.0:0.01:2."
                                  "0");
    Planner::declareParam<bool>("use_k_nearest", this, &RRTXdynamic::setKNearest, &RRTXdynamic::getKNearest, "0,1");
    Planner::declareParam<bool>("update_children", this, &RRTXdynamic::setUpdateChildren, &RRTXdynamic::getUpdateChildren,
                                "0,1");
    Planner::declareParam<int>("rejection_variant", this, &RRTXdynamic::setVariant, &RRTXdynamic::getVariant, "0:3");
    Planner::declareParam<double>("rejection_variant_alpha", this, &RRTXdynamic::setAlpha, &RRTXdynamic::getAlpha, "0.:"
                                                                                                                 "1.");
    Planner::declareParam<bool>("informed_sampling", this, &RRTXdynamic::setInformedSampling,
                                &RRTXdynamic::getInformedSampling, "0,"
                                                                  "1");
    Planner::declareParam<bool>("sample_rejection", this, &RRTXdynamic::setSampleRejection,
                                &RRTXdynamic::getSampleRejection, "0,1");
    Planner::declareParam<unsigned int>("number_sampling_attempts", this, &RRTXdynamic::setNumSamplingAttempts,
                                &RRTXdynamic::getNumSamplingAttempts, "10:10:100000");

    addPlannerProgressProperty("iterations INTEGER", [this] { return numIterationsProperty(); });
    addPlannerProgressProperty("motions INTEGER", [this] { return numMotionsProperty(); });
    addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

ompl::geometric::RRTXdynamic::~RRTXdynamic()
{
    freeMemory();
}

void ompl::geometric::RRTXdynamic::setup()
{
    if (!alreadyRan_)
    {
        Planner::setup();
        tools::SelfConfig sc(si_, getName());
        sc.configurePlannerRange(maxDistance_);
        if (!si_->getStateSpace()->hasSymmetricDistance() || !si_->getStateSpace()->hasSymmetricInterpolate())
        {
            OMPL_WARN("%s requires a state space with symmetric distance and symmetric interpolation.", getName().c_str());
        }

        if (!nn_)
            nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

        // Setup optimization objective
        //
        // If no optimization objective was specified, then default to
        // optimizing path length as computed by the distance() function
        // in the state space.
        if (pdef_)
        {
            if (pdef_->hasOptimizationObjective())
                opt_ = pdef_->getOptimizationObjective();
            else
            {
                OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed "
                            "planning time.",
                            getName().c_str());
                opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);

                // Store the new objective in the problem def'n
                pdef_->setOptimizationObjective(opt_);
            }
            // Set the bestCost_ as infinite
            bestCost_ = opt_->infiniteCost();
            mc_ = MotionCompare(opt_, pdef_);
            q_ = BinaryHeap<Motion *, MotionCompare>(mc_);
        }
        else
        {
            OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
            setup_ = false;
        }
    } else
    {
        if (pdef_)
        {
            if (pdef_->hasOptimizationObjective())
                opt_ = pdef_->getOptimizationObjective();
            else
            {
                OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed "
                            "planning time.",
                            getName().c_str());
                opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);

                // Store the new objective in the problem def'n
                pdef_->setOptimizationObjective(opt_);
            }
            // Set the bestCost_ as infinite
            bestCost_ = opt_->infiniteCost();
            mc_ = MotionCompare(opt_, pdef_);
            // q_ = BinaryHeap<Motion *, MotionCompare>(mc_);
        }

        vbot_->cost = opt_->infiniteCost();
        updateQueue(vbot_);
        
        // Check if there are more starts
        if (pis_.haveMoreStartStates() == true)
        {
            // There are, add them
            while (const base::State *st = pis_.nextStart())
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->cost = opt_->identityCost();
                nn_->add(motion);
                vbot_ = motion;
                updateQueue(vbot_);
            }

            // And assure that, if we're using an informed sampler, it's reset
            infSampler_.reset();
        }
       
        updateObstacles();

        setup_ = true;
    }

    // Calculate some constants:
    calculateRewiringLowerBounds();

}

void ompl::geometric::RRTXdynamic::clear()
{
    setup_ = false;
    Planner::clear();
    sampler_.reset();
    infSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();

    lastGoalMotion_ = nullptr;
    goalMotions_.clear();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

void ompl::geometric::RRTXdynamic::clearQuery()
{
    setup_ = false;
    Planner::clear();
    sampler_.reset();
    infSampler_.reset();

    lastGoalMotion_ = nullptr;
    goalMotions_.clear();
    pdef_->clearStartStates();
    pis_.clear();

    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

ompl::base::PlannerStatus ompl::geometric::RRTXdynamic::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    if (!alreadyRan_) {

        // Check if there are more starts
        if (pis_.haveMoreStartStates() == true)
        {
            // There are, add them
            while (const base::State *st = pis_.nextStart())
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->cost = opt_->identityCost();
                nn_->add(motion);
                vbot_ = motion;
            }

            // And assure that, if we're using an informed sampler, it's reset
            infSampler_.reset();
        }
        // No else
    }

    // TODO: this is sloppy
    alreadyRan_ = true;

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Allocate a sampler if necessary
    if (!sampler_ && !infSampler_)
    {
        allocSampler();
    }

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    if (!si_->getStateSpace()->isMetricSpace())
        OMPL_WARN("%s: The state space (%s) is not metric and as a result the optimization objective may not satisfy "
                  "the triangle inequality. "
                  "You may need to disable rejection.",
                  getName().c_str(), si_->getStateSpace()->getName().c_str());

    const base::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();

    Motion *solution = lastGoalMotion_;

    Motion *approximation = nullptr;
    double approximatedist = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;

    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();
    base::State *dstate;

    Motion *motion;
    Motion *nmotion;
    Motion *nb;

    // TODO: are these safe to comment out? I think they're just used for logging
    Motion *min;
    Motion *c;
    bool feas;

    unsigned int rewireTest = 0;
    unsigned int statesGenerated = 0;

    base::Cost incCost, cost;

    if (solution)
        OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f", getName().c_str(),
                    solution->cost.value());

    if (useKNearest_)
        OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(),
                    (unsigned int)std::ceil(k_rrt_ * log((double)(nn_->size() + 1u))));
    else
        OMPL_INFORM(
            "%s: Initial rewiring radius of %.2f", getName().c_str(),
            std::min(maxDistance_, r_rrt_ * std::pow(log((double)(nn_->size() + 1u)) / ((double)(nn_->size() + 1u)),
                                                     1 / (double)(si_->getStateDimension()))));

    while (ptc == false)
    {
        iterations_++;

        // Computes the RRG values for this iteration (number or radius of neighbors)
        calculateRRG();

        // sample random state (with goal biasing)
        // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal
        // states.
        if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ &&
            goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
        {
            // Attempt to generate a sample, if we fail (e.g., too many rejection attempts), skip the remainder of this
            // loop and return to try again
            if (!sampleUniform(rstate))
                continue;
        }

        // find closest state in the tree
        nmotion = nn_->nearest(rmotion);

        if (intermediateSolutionCallback && si_->equalStates(nmotion->state, rstate))
            continue;

        dstate = rstate;

        // find state to add to the tree
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        // Check if the motion between the nearest state and the state to add is valid
        if (si_->checkMotion(nmotion->state, dstate))
        {
            // create a motion
            motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            incCost = opt_->motionCost(nmotion->state, motion->state);
            motion->cost = opt_->combineCosts(nmotion->cost, incCost);

            // Find nearby neighbors of the new motion
            getNeighbors(motion);

            // find which one we connect the new state to
            for (auto it = motion->nbh.begin(); it != motion->nbh.end();)
            {
                nb = it->first;
                feas = it->second;

                // Compute cost using nb as a parent
                incCost = opt_->motionCost(nb->state, motion->state);
                cost = opt_->combineCosts(nb->cost, incCost);
                if (opt_->isCostBetterThan(cost, motion->cost))
                {
                    // Check range and feasibility
                    if ((!useKNearest_ || distanceFunction(motion, nb) < maxDistance_) &&
                        si_->checkMotion(nb->state, motion->state))
                    {
                        // mark than the motino has been checked as valid
                        it->second = true;

                        motion->cost = cost;
                        motion->parent = nb;
                        ++it;
                    }
                    else
                    {
                        // Remove unfeasible neighbor from the list of neighbors
                        it = motion->nbh.erase(it);
                    }
                }
                else
                {
                    ++it;
                }
            }

            // Check if the vertex should included
            if (!includeVertex(motion))
            {
                si_->freeState(motion->state);
                delete motion;
                continue;
            }

            // Update neighbor motions neighbor datastructure
            for (auto it = motion->nbh.begin(); it != motion->nbh.end(); ++it)
            {
                it->first->nbh.emplace_back(motion, it->second);
            }

            // add motion to the tree
            ++statesGenerated;
            nn_->add(motion);
            if (updateChildren_)
                motion->parent->children.push_back(motion);

            // add the new motion to the queue to propagate the changes
            updateQueue(motion);

            bool checkForSolution = false;

            // Add the new motion to the goalMotion_ list, if it satisfies the goal
            double distanceFromGoal;
            if (goal->isSatisfied(motion->state, &distanceFromGoal))
            {
                goalMotions_.push_back(motion);
                checkForSolution = true;
            }

            reduceInconsistency(false, rewireTest, checkForSolution);

            // empty q and reset handles
            while (!q_.empty())
            {
                q_.top()->data->handle = nullptr;
                q_.pop();
            }
            q_.clear();

            // Checking for solution or iterative improvement
            if (checkForSolution)
            {
                bool updatedSolution = false;
                for (auto &goalMotion : goalMotions_)
                {
                    if (opt_->isCostBetterThan(goalMotion->cost, bestCost_))
                    {
                        if (opt_->isFinite(bestCost_) == false)
                        {
                            OMPL_INFORM("%s: Found an initial solution with a cost of %.2f in %u iterations (%u "
                                        "vertices in the graph)",
                                        getName().c_str(), goalMotion->cost.value(), iterations_, nn_->size());
                        }
                        bestCost_ = goalMotion->cost;
                        updatedSolution = true;
                    }

                    sufficientlyShort = opt_->isSatisfied(goalMotion->cost);
                    if (sufficientlyShort)
                    {
                        solution = goalMotion;
                        break;
                    }
                    else if (!solution || opt_->isCostBetterThan(goalMotion->cost, solution->cost))
                    {
                        solution = goalMotion;
                        updatedSolution = true;
                    }
                }

                if (updatedSolution)
                {
                    if (intermediateSolutionCallback)
                    {
                        std::vector<const base::State *> spath;
                        Motion *intermediate_solution =
                            solution->parent;  // Do not include goal state to simplify code.

                        // Push back until we find the start, but not the start itself
                        while (intermediate_solution->parent != nullptr)
                        {
                            spath.push_back(intermediate_solution->state);
                            intermediate_solution = intermediate_solution->parent;
                        }

                        intermediateSolutionCallback(this, spath, bestCost_);
                    }
                }
            }

            // Checking for approximate solution (closest state found to the goal)
            if (goalMotions_.size() == 0 && distanceFromGoal < approximatedist)
            {
                approximation = motion;
                approximatedist = distanceFromGoal;
            }
        }

        // terminate if a sufficient solution is found
        if (solution && sufficientlyShort)
            break;
    }

    bool approximate = (solution == nullptr);
    bool addedSolution = false;
    if (approximate)
        solution = approximation;
    else
        lastGoalMotion_ = solution;

    if (solution != nullptr)
    {
        ptc.terminate();
        // construct the solution path
        std::vector<Motion *> mpath;
        std::set<Motion *> visited;
        while (solution != nullptr)
        {   
            if (solution->parent == solution){
              break;
            }
            if (!visited.insert(solution).second)
            {
                base::ScopedState<> scoped(si_->getStateSpace());
                scoped = solution->state;
                std::cerr << "Cycle detected at Motion " << scoped
                          << ", which was already visited!\n";
                break;
            }

            mpath.push_back(solution);
            base::ScopedState<> scoped(si_->getStateSpace());
            scoped = solution->state;

            base::ScopedState<> scopedParent(si_->getStateSpace());
            if (solution->parent != nullptr)
            {
                scopedParent = solution->parent->state;
                std::cerr << "Motion " << scoped << " -> parent " << scopedParent << std::endl;
            } else {
                std::cerr << "Motion " << scoped << " -> parent: NONE" << std::endl;
            }

            solution = solution->parent;
        }

        // set the solution path
        auto path = std::make_shared<PathGeometric>(si_);
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);

        // Add the solution path.
        base::PlannerSolution psol(path);
        psol.setPlannerName(getName());
        if (approximate)
            psol.setApproximate(approximatedist);
        // Does the solution satisfy the optimization objective?
        psol.setOptimized(opt_, bestCost_, sufficientlyShort);
        pdef_->addSolutionPath(psol);

        addedSolution = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree. Final solution cost "
                "%.3f",
                getName().c_str(), statesGenerated, rewireTest, goalMotions_.size(), bestCost_.value());

    return {addedSolution, approximate};
}

// -------------- begin new functions ----------------

void ompl::geometric::RRTXdynamic::reduceInconsistency(bool earlyExit, unsigned int &rewireTest, bool &checkForSolution){

    base::Cost incCost, cost;

    // Process the elements in the queue and rewire the tree until epsilon-optimality
    while (!q_.empty())
    {
        // Get element to update
        Motion* min = q_.top()->data;
        // Remove element from the queue and NULL the handle so that we know it's not in the queue anymore
        q_.pop();
        min->handle = nullptr;

        // Stop cost propagation if it is not in the relevant region
        if (opt_->isCostBetterThan(bestCost_, mc_.costPlusHeuristic(min)) && earlyExit)
            break;

        // Try min as a parent to optimize each neighbor
        for (auto it = min->nbh.begin(); it != min->nbh.end();)
        {
            Motion* nb = it->first;
            bool feas = it->second;

            // Neighbor culling: removes neighbors farther than the neighbor radius
            if ((!useKNearest_ || min->nbh.size() > rrg_k_) && distanceFunction(min, nb) > rrg_r_)
            {
                it = min->nbh.erase(it);
                continue;
            }

            // Calculate cost of nb using min as a parent
            incCost = opt_->motionCost(min->state, nb->state);
            cost = opt_->combineCosts(min->cost, incCost);

            // If cost improvement is better than epsilon
            if (opt_->isCostBetterThan(opt_->combineCosts(cost, epsilonCost_), nb->cost))
            {
                if (nb->parent != min)
                {
                    // changing parent, check feasibility
                    if (!feas)
                    {
                        feas = si_->checkMotion(nb->state, min->state);
                        if (!feas)
                        {
                            // Remove unfeasible neighbor from the list of neighbors
                            it = min->nbh.erase(it);
                            continue;
                        }
                        else
                        {
                            // mark than the motino has been checked as valid
                            it->second = true;
                        }
                    }
                    if (updateChildren_)
                    {
                        // Remove this node from its parent list
                        if (nb->parent)
                          removeFromParent(nb);
                        // add it as a children of min
                        min->children.push_back(nb);
                    }
                    // Add this node to the new parent
                    nb->parent = min;
                    ++rewireTest;
                }
                nb->cost = cost;

                // Add to the queue for more improvements
                updateQueue(nb);

                checkForSolution = true;
            }
            ++it;
        }
        if (updateChildren_)
        {
            // Propagatino of the cost to the children
            for (auto it = min->children.begin(), end = min->children.end(); it != end; ++it)
            {
                Motion* c = *it;
                incCost = opt_->motionCost(min->state, c->state);
                cost = opt_->combineCosts(min->cost, incCost);
                c->cost = cost;
                // Add to the queue for more improvements
                updateQueue(c);

                checkForSolution = true;
            }
        }
    }
}

// updateObstacles: Check every edge for feasibility and update the tree.
void ompl::geometric::RRTXdynamic::updateObstacles()
{
    // Iterate over all motions in the tree.
    std::vector<Motion *> motions;
    nn_->list(motions);
    for (auto &m : motions)
    {
        std::cout << "BREAKING CONNECTIONS" << std::endl;
        if (!si_->isValid(m->state))
        {

           m->cost = opt_->infiniteCost();
           if (m->handle)
           {
              q_.remove(m->handle);
              m->handle = nullptr;
           }

           if (m->parent)
           {
              removeFromParent(m);
           }
            
           orphanSet_.insert(m);
           updateQueue(m);

           // Iterate over the neighbor list with an iterator so we can erase safely.
           for (auto it = m->nbh.begin(); it != m->nbh.end();)
           {    
               Motion *nb = it->first;
               it = m->nbh.erase(it);

               if (nb != nullptr)
               {
                   // Also remove m from each of its neighbors
                   auto found = std::find_if(nb->nbh.begin(), nb->nbh.end(),
                         [m](const std::pair<Motion*,bool> &p) { return p.first == m; });
                   if (found != nb->nbh.end()) nb->nbh.erase(found);
               }
            }

            nn_->remove(m);
            // si_->freeState(m->state);
            // delete m;
            continue;
        }
        if (m->parent)
        {

            // Use the state space’s checkMotion to test the edge.
            // (This function returns true if the motion from the first state to the second is collision-free.)
            if (!si_->checkMotion(m->parent->state, m->state))
            {
                // The edge is no longer feasible. Invalidate by setting cost to infinite.
                m->cost = opt_->infiniteCost();
                
                // Remove m from its parent's children list.
                removeFromParent(m);

                m->parent = nullptr;

                // Mark this motion as orphaned.
                orphanSet_.insert(m);
                
                // Update the queue for m so that it gets processed in cost propagation.
                updateQueue(m);
            } 
            else 
            {
                base::Cost incCost = opt_->motionCost(m->parent->state, m->state);
                base::Cost cost = opt_->combineCosts(m->parent->cost, incCost);
                // Check if parent cost has changed, if so update
                // This only happens for nodes connected to the old start?
                if (!opt_->isCostEquivalentTo(cost, m->cost))
                {
                    m->cost = opt_->infiniteCost();
                    updateQueue(m);
                }
            }
        }
        // Iterate over the neighbor list with an iterator so we can erase safely.
        for (auto it = m->nbh.begin(); it != m->nbh.end();)
        {
            Motion *nb = it->first;
            bool feas  = it->second;  // This might be the cached feasibility
           
            // In case nb is inside an obstacle and m was not in nb's nbh list
            if (nb == nullptr)
            {
                it = m->nbh.erase(it);
                continue;
            }

            if (nb->state == nullptr)
            {
                it = m->nbh.erase(it);
                continue;
            }
            // Re-check the motion feasibility if we haven't already (or always re-check if you prefer)
            if (!si_->checkMotion(m->state, nb->state))
            {

                it->second = false;
                // Remove from m->nbh
                it = m->nbh.erase(it);

                // Optionally, also remove the edge from nb->nbh for consistency if you keep a symmetric list.
                // You can do that by finding (m) in nb->nbh and erasing it too.
                // Example:
                auto found = std::find_if(nb->nbh.begin(), nb->nbh.end(),
                      [m](const std::pair<Motion*,bool> &p) { return p.first == m; });
                if (found != nb->nbh.end()) nb->nbh.erase(found);

            }
            else
            {
                // If still valid, mark it feasible if not already
                it->second = true;
                ++it;
            }
        }
    }
    
    // Now, propagate the cost changes (i.e. the infinite cost) through the descendants.
    propagateDescendants();
    
    // Optionally, verify that the robot’s current best motion (vbot) is correctly in the queue.
    updateQueue(vbot_);
    
    // Finally, run reduceInconsistency to rewire the tree (our refactored cost propagation function).
    // TODO: confirm whether these parameters should be changed...
    uint dummyRewireTest = 0;
    bool dummyCheckForSolution = false;
    reduceInconsistency(false, dummyRewireTest, dummyCheckForSolution);
}

/*
// propagateDescendants: For all orphaned motions, recursively collect and mark all affected descendants.
void ompl::geometric::RRTXdynamic::propagateDescendants()
{
    // Create a local set to gather all affected (descendant) motions.
    std::set<Motion*> affected;
    for (Motion *m : orphanSet_)
    {
        // Recursively collect all descendants of m into the affected set.
        collectDescendants(m, affected);
    }
    
    // For each affected motion (which may not have been explicitly orphaned initially),
    // mark its cost values as infinite and update the queue so that it can be rewired.
    for (Motion *m : affected)
    {
        m->cost = opt_->infiniteCost();
        
        // If m still has a valid parent, remove it.
        if (m->parent)
            removeFromParent(m);
        
        // Update the motion in the queue.
        updateQueue(m);
    }
    
    // Clear the orphan set since all affected nodes have been processed.
    orphanSet_.clear();
}
*/
void ompl::geometric::RRTXdynamic::propagateDescendants()
{
    // 1. Gather all affected nodes: the orphan set plus all their descendants.
    std::set<Motion*> affected;
    for (Motion *m : orphanSet_)
    {
        affected.insert(m);
        collectDescendants(m, affected);
    }

    // 2. For each affected node, update its neighbors.
    for (Motion *v : affected)
    {
        // Optionally, also check the parent if not already affected.
        if (v->parent && affected.find(v->parent) == affected.end())
        {
            v->parent->cost = opt_->infiniteCost();
            updateQueue(v->parent);
        }

        // Now update each neighbor in v->nbh using an iterator.
        for (auto it = v->nbh.begin(); it != v->nbh.end(); ++it)
        {
            Motion *u = it->first;  // assuming nbh is a container of std::pair<Motion*, bool>
            if (affected.find(u) == affected.end())
            {
                u->cost = opt_->infiniteCost();
                updateQueue(u);
            }
        }
    }

    // 3. Finalize each affected node: remove from parent's children and update its own cost.
    for (Motion *v : affected)
    {
        if (v->parent)
            removeFromParent(v);
        v->cost = opt_->infiniteCost();
        updateQueue(v);
    }

    // Clear the orphan set after processing.
    orphanSet_.clear();
}

// collectDescendants: Helper function to recursively add all children of m into 'descendants'.
void ompl::geometric::RRTXdynamic::collectDescendants(Motion *m, std::set<Motion*> &descendants)
{
    for (Motion *child : m->children)
    {
        if (descendants.find(child) == descendants.end())
        {
            descendants.insert(child);
            collectDescendants(child, descendants);
        }
    }
}

// ------- end new functions --------

// This is called "verrifyQueue" in the paper
void ompl::geometric::RRTXdynamic::updateQueue(Motion *x)
{
    // If x->handle is not NULL, x is already in the queue and needs to be update, otherwise it is inserted
    if (x->handle != nullptr)
    {
        q_.update(x->handle);
    }
    else
    {
        x->handle = q_.insert(x);
    }
}

void ompl::geometric::RRTXdynamic::removeFromParent(Motion *m)
{
    for (auto it = m->parent->children.begin(); it != m->parent->children.end(); ++it)
    {
        if (*it == m)
        {
            m->parent->children.erase(it);
            break;
        }
    }
}

void ompl::geometric::RRTXdynamic::calculateRRG()
{
    auto cardDbl = static_cast<double>(nn_->size() + 1u);
    rrg_k_ = std::ceil(k_rrt_ * log(cardDbl));
    rrg_r_ = std::min(maxDistance_,
                      r_rrt_ * std::pow(log(cardDbl) / cardDbl, 1 / static_cast<double>(si_->getStateDimension())));
}

void ompl::geometric::RRTXdynamic::getNeighbors(Motion *motion) const
{
    if (motion->nbh.size() > 0)
    {
        return;
    }

    std::vector<Motion *> nbh;
    if (useKNearest_)
    {
        //- k-nearest RRT*
        nn_->nearestK(motion, rrg_k_, nbh);
    }
    else
    {
        nn_->nearestR(motion, rrg_r_, nbh);
    }

    motion->nbh.resize(nbh.size());
    std::transform(nbh.begin(), nbh.end(), motion->nbh.begin(),
                   [](Motion *m) { return std::pair<Motion *, bool>(m, false); });
}

bool ompl::geometric::RRTXdynamic::includeVertex(const Motion *x) const
{
    switch (variant_)
    {
        case 1:
            return opt_->isCostBetterThan(mc_.alphaCostPlusHeuristic(x, alpha_), opt_->infiniteCost());  // Always true?
        case 2:
            return opt_->isCostBetterThan(mc_.alphaCostPlusHeuristic(x->parent, alpha_), bestCost_);
        case 3:
            return opt_->isCostBetterThan(mc_.alphaCostPlusHeuristic(x, alpha_), bestCost_);
        default:  // no rejection
            return true;
    }
}

void ompl::geometric::RRTXdynamic::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void ompl::geometric::RRTXdynamic::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

void ompl::geometric::RRTXdynamic::setInformedSampling(bool informedSampling)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setSampleRejection, assert that:
    if (informedSampling == true && useRejectionSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // Check if we're changing the setting of informed sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (informedSampling != useInformedSampling_)
    {
        // Store the value
        useInformedSampling_ = informedSampling;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::geometric::RRTXdynamic::setSampleRejection(const bool reject)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setSampleRejection, assert that:
    if (reject == true && useInformedSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // Check if we're changing the setting of rejection sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (reject != useRejectionSampling_)
    {
        // Store the setting
        useRejectionSampling_ = reject;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::geometric::RRTXdynamic::allocSampler()
{
    // Allocate the appropriate type of sampler.
    if (useInformedSampling_)
    {
        // We are using informed sampling, this can end-up reverting to rejection sampling in some cases
        OMPL_INFORM("%s: Using informed sampling.", getName().c_str());
        infSampler_ = opt_->allocInformedStateSampler(pdef_, numSampleAttempts_);
    }
    else if (useRejectionSampling_)
    {
        // We are explicitly using rejection sampling.
        OMPL_INFORM("%s: Using rejection sampling.", getName().c_str());
        infSampler_ = std::make_shared<base::RejectionInfSampler>(pdef_, numSampleAttempts_);
    }
    else
    {
        // We are using a regular sampler
        sampler_ = si_->allocStateSampler();
    }

    if (localSeedSet_)
    {
        this->setLocalSeed(localSeed_);
    }
}

bool ompl::geometric::RRTXdynamic::sampleUniform(base::State *statePtr)
{
    // Use the appropriate sampler
    if (useInformedSampling_ || useRejectionSampling_)
    {
        // Attempt the focused sampler and return the result.
        // If bestCost is changing a lot by small amounts, this could
        // be prunedCost_ to reduce the number of times the informed sampling
        // transforms are recalculated.
        return infSampler_->sampleUniform(statePtr, bestCost_);
    }
    else
    {
        // Simply return a state from the regular sampler
        sampler_->sampleUniform(statePtr);

        // Always true
        return true;
    }
}

void ompl::geometric::RRTXdynamic::calculateRewiringLowerBounds()
{
    auto dimDbl = static_cast<double>(si_->getStateDimension());

    // k_rrt > 2^(d + 1) * e * (1 + 1 / d).  K-nearest RRT*
    k_rrt_ = rewireFactor_ * (std::pow(2, dimDbl + 1) * boost::math::constants::e<double>() * (1.0 + 1.0 / dimDbl));

    // r_rrt > (2*(1+1/d))^(1/d)*(measure/ballvolume)^(1/d)
    r_rrt_ = rewireFactor_ *
             std::pow(2 * (1.0 + 1.0 / dimDbl) * (si_->getSpaceMeasure() / unitNBallMeasure(si_->getStateDimension())),
                      1.0 / dimDbl);
}
