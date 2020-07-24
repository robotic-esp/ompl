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

/* Author: Marlin Strub */

#include "ompl/base/spaces/ConstrainedSO3StateSpace.h"

#include <algorithm>
#include <limits>
#include <cmath>

#include <boost/math/constants/constants.hpp>
#include <boost/assert.hpp>

#include "ompl/tools/config/MagicConstants.h"
#include "ompl/util/Exception.h"

using namespace boost::math::double_constants;

namespace ompl
{
    namespace base
    {
        ConstrainedSO3StateSampler::ConstrainedSO3StateSampler(const StateSpace *space) : SO3StateSampler(space)
        {
        }

        void ConstrainedSO3StateSampler::sampleUniform(State *state)
        {
            do
            {
                SO3StateSampler::sampleUniform(state);
            } while (!space_->satisfiesBounds(state));

            OMPL_DEBUG("Uniform sampled rotation of: %.2f <= %.2f", 2.0 * std::acos(state->as<ConstrainedSO3StateSpace::StateType>()->w), space_->as<ConstrainedSO3StateSpace>()->getMaxRotation());
        }

        void ConstrainedSO3StateSampler::sampleUniformNear(State *state, const State *near,
                                                           const double distance)
        {
            do
            {
                SO3StateSampler::sampleUniformNear(state, near, distance);
            } while (!space_->satisfiesBounds(state));

            OMPL_DEBUG("Near sampled rotation of: %.2f <= %.2f", 2.0 * std::acos(state->as<ConstrainedSO3StateSpace::StateType>()->w), space_->as<ConstrainedSO3StateSpace>()->getMaxRotation());
        }

        void ConstrainedSO3StateSampler::sampleGaussian(State *state, const State *mean,
                                                        const double stdDev)
        {
            do
            {
                SO3StateSampler::sampleGaussian(state, mean, stdDev);
            } while (!space_->satisfiesBounds(state));

            OMPL_DEBUG("Gaussian sampled rotation of: %.2f <= %.2f", 2.0 * std::acos(state->as<ConstrainedSO3StateSpace::StateType>()->w), space_->as<ConstrainedSO3StateSpace>()->getMaxRotation());
        }

        bool ConstrainedSO3StateSpace::satisfiesBounds(const State *state) const
        {
            return (2.0 * std::acos(state->as<ConstrainedSO3StateSpace::StateType>()->w) <= maxRotation_);
        }

        StateSamplerPtr ConstrainedSO3StateSpace::allocDefaultStateSampler() const
        {
            auto sampler = std::make_shared<ConstrainedSO3StateSampler>(this);
            return sampler;
        }

        void ConstrainedSO3StateSpace::setMaxRotation(double maxRotation)
        {
            maxRotation_ = maxRotation;
        }

        double ConstrainedSO3StateSpace::getMaxRotation() const
        {
            return maxRotation_;
        }

    }  // namespace base
}  // namespace ompl
