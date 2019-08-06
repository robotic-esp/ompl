/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Oxford
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

// Authors: Marlin Strub

#pragma once

#include <algorithm>
#include <memory>

#include "ompl/base/Planner.h"
#include "ompl/datastructures/BinaryHeap.h"
#include "ompl/geometric/planners/tbdstar/datastructures/Edge.h"
#include "ompl/geometric/planners/tbdstar/datastructures/ImplicitGraph.h"
#include "ompl/geometric/planners/tbdstar/datastructures/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        class TBDstar : public ompl::base::Planner
        {
        public:
            /** \brief Constructs a TBDstar. */
            TBDstar(const ompl::base::SpaceInformationPtr &spaceInformation);

            /** \brief Destructs a TBDstar. */
            ~TBDstar() = default;

            /** \brief Additional setup that can only be done once a problem definition is set. */
            void setup() override;

            /** \brief Solves a motion planning problem. */
            ompl::base::PlannerStatus
            solve(const ompl::base::PlannerTerminationCondition &terminationCondition) override;

            /** \brief Get the cost of the incumbent solution. */
            ompl::base::Cost bestCost() const;

            /** \brief Get the planner data. */
            void getPlannerData(base::PlannerData &data) const override;

            /** \brief Set the option to compute the backward search heuristic. */
            void setComputeBackwardSearchHeuristic(bool computeBackwardSearchHeuristic);

            /** \brief Enable LPA* repair of backward search. */
            void setRepairBackwardSearch(bool setRepairBackwardSearch);

            /** \brief Get the edge queue. */
            std::vector<tbdstar::Edge> getEdgesInQueue() const;

            /** \brief Get the vertex queue. */
            std::vector<std::shared_ptr<tbdstar::Vertex>> getVerticesInQueue() const;

            /** \brief Get the next edge in the queue. */
            tbdstar::Edge getNextEdgeInQueue() const;

            /** \brief Get the next vertex in the queue. */
            std::shared_ptr<tbdstar::Vertex> getNextVertexInQueue() const;

            /** \brief Get the vertices in the backward search tree. */
            std::vector<std::shared_ptr<tbdstar::Vertex>> getVerticesInBackwardSearchTree() const;

        private:
            /** \brief Performs one iteration of TBDstar. */
            void iterate();

            /** \brief Performs one forward search iterations. */
            void performForwardSearchIteration();

            /** \brief Performs one backward search iterations. */
            void performBackwardSearchIteration();

            /** \brief Updates a vertex in the backward search queue (LPA* update). */
            void backwardSearchUpdateVertex(const std::shared_ptr<tbdstar::Vertex> &vertex);

            /** \brief Inserts or updates a vertex in the backward queue. */
            void insertOrUpdateInBackwardQueue(const std::shared_ptr<tbdstar::Vertex> &vertex);

            /** \brief Rebuilds the forward queue. */
            void rebuildForwardQueue();

            /** \brief Returns a vector of states from the argument to a start. */
            std::vector<std::shared_ptr<tbdstar::Vertex>>
            getReversePath(const std::shared_ptr<tbdstar::Vertex> &vertex) const;

            /** \brief Computes the sort key of an edge. */
            std::array<double, 3u> computeSortKey(const std::shared_ptr<tbdstar::Vertex> &parent,
                                                  const std::shared_ptr<tbdstar::Vertex> &child) const;

            void insertOutgoingEdges(const std::shared_ptr<tbdstar::Vertex> &vertex);

            /** \brief Computes the backward search heuristic. */
            void computeBackwardSearchHeuristic();

            /** \brief Checks whether the cost to come of a goal vertex has been updated and updates the solution if so.
             */
            void updateSolution();

            /** \brief Returns the best cost-to-go-heuristic to any start in the graph. */
            ompl::base::Cost computeCostToGoToStartHeuristic(const std::shared_ptr<tbdstar::Vertex> &vertex) const;

            /** \brief Returns the best cost-to-go-heuristic to any goal in the graph. */
            ompl::base::Cost computeCostToGoToGoalHeuristic(const std::shared_ptr<tbdstar::Vertex> &vertex) const;

            /** \brief Returns the best cost to come form the goal of any start. */
            ompl::base::Cost computeBestCostToComeFromGoalOfAnyStart() const;

            /** \brief The increasingly dense sampling-based approximation. */
            tbdstar::ImplicitGraph graph_;

            /** \brief The forward queue. */
            using EdgeQueue =
                ompl::BinaryHeap<tbdstar::Edge, std::function<bool(const tbdstar::Edge &, const tbdstar::Edge &)>>;
            EdgeQueue forwardQueue_;

            /** \brief The backward queue. */
            using KeyVertexPair = std::pair<std::array<double, 2u>, std::shared_ptr<tbdstar::Vertex>>;
            using VertexQueue =
                ompl::BinaryHeap<KeyVertexPair, std::function<bool(const KeyVertexPair &, const KeyVertexPair &)>>;
            VertexQueue backwardQueue_;

            /** \brief The id of the current forward search .*/
            std::shared_ptr<std::size_t> forwardSearchId_;

            /** \brief The id of the current backward search .*/
            std::shared_ptr<std::size_t> backwardSearchId_;

            /** \brief The cost of the incumbent solution. */
            std::shared_ptr<ompl::base::Cost> solutionCost_;

            /** \brief The number of iterations that have been performed. */
            std::size_t numIterations_{0u};

            /** \brief The number of samples per batch. */
            std::size_t batchSize_{100u};

            /** \brief Flag whether the forward search has been started on the batch. */
            bool isForwardSearchStartedOnBatch_{false};

            /** \brief Flag whether the forward queue needs to be rebuilt. */
            bool forwardQueueMustBeRebuilt_{false};

            /** \brief The option that specifies whether to compute the backward search heuristic. */
            bool computeBackwardSearchHeuristic_{true};

            /** \brief The option that specifies whether to repair the backward search when detecting a collision. */
            bool repairBackwardSearch_{true};

            /** \brief Syntactic helper to get at the optimization objective of the planner base class. */
            ompl::base::OptimizationObjectivePtr optimizationObjective_;

            /** \brief Syntactic helper to get at the motion validator of the planner base class. */
            ompl::base::MotionValidatorPtr motionValidator_;
        };
    }  // namespace geometric
}  // namespace ompl