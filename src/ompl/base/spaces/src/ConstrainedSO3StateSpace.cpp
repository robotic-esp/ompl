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

        /** \brief Set the max rotation. */
        void ConstrainedSO3StateSampler::setMaxRotation(double maxRotation)
        {
            maxRotation_ = maxRotation;
        }

        /** \brief Get the max rotation. */
        double ConstrainedSO3StateSampler::getMaxRotation() const
        {
            return maxRotation_;
        }

        void ConstrainedSO3StateSampler::sampleUniform(State *state)
        {
            std::array<double, 4u> quaternion;
            do
            {
                rng_.quaternion(quaternion.data());
            } while (2.0 * std::acos(quaternion[3]) > maxRotation_);

            auto so3state = state->as<SO3StateSpace::StateType>();
            so3state->x = quaternion[0u];
            so3state->y = quaternion[1u];
            so3state->z = quaternion[2u];
            so3state->w = quaternion[3u];
        }

        void ConstrainedSO3StateSampler::sampleUniformNear(State * /* state */, const State * /* near */,
                                                           const double /* distance */)
        {
            throw ompl::Exception("Currently not implemented.");
        }

        void ConstrainedSO3StateSampler::sampleGaussian(State * /* state */, const State * /* mean */,
                                                        const double /* stdDev */)
        {
            throw ompl::Exception("Currently not implemented.");
        }

        StateSamplerPtr ConstrainedSO3StateSpace::allocDefaultStateSampler() const
        {
            auto sampler = std::make_shared<ConstrainedSO3StateSampler>(this);
            sampler->setMaxRotation(maxRotation_);
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
