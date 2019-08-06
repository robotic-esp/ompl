/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019-present University of Oxford
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
 *   * Neither the names of the copyright holders nor the names of its
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

#include <array>
#include <memory>

#include "ompl/geometric/planners/tbdstar/datastructures/Vertex.h"
#include "ompl/base/Cost.h"

namespace ompl
{
    namespace geometric
    {
        namespace tbdstar
        {
            class Edge
            {
            public:
                /** \brief Needs a default constructor for the binary heap. */
                Edge();

                /** \brief Constructs an edge from a parent, a child, and the sort key. */
                Edge(const std::shared_ptr<Vertex> &parent, const std::shared_ptr<Vertex> &child,
                     const std::array<double, 3u> &sortKey);

                /** \brief Deconstructs an edge. */
                ~Edge() = default;

                /** \brief Returns the parent in this edge. */
                std::shared_ptr<Vertex> getParent() const;

                /** \brief Returns the child in this edge. */
                std::shared_ptr<Vertex> getChild() const;

                /** \brief Returns the sort key associated with this edge. */
                const std::array<double, 3u> &getSortKey() const;

                /** \brief Sets the sort key associated with this edge. */
                void setSortKey(const std::array<double, 3u> &key);

            private:
                /** \brief The parent in this edge. */
                std::shared_ptr<Vertex> parent_;

                /** \brief The child in this edge. */
                std::shared_ptr<Vertex> child_;

                /** \brief The sort key associated with this edge. */
                std::array<double, 3u> sortKey_;
            };

        }  // namespace tbdstar

    }  // namespace geometric

}  // namespace ompl