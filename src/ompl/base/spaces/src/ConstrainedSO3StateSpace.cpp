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

#include <Eigen/Core>
#include <Eigen/Geometry>

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

        namespace
        {
            std::array<double, 4u> rotate(const std::array<double, 4u> &point, const std::array<double, 4u> &quaternion)
            {
                const Eigen::Map<const Eigen::Quaternion<double>> p(point.data());
                const Eigen::Map<const Eigen::Quaternion<double>> q(quaternion.data());
                const auto res = q * p * q.inverse();
                return {res.x(), res.y(), res.z(), res.w()};
            }

            double angleBetween(const std::array<double, 4u>& vector1, const std::array<double, 4u>& vector2) {
                const Eigen::Map<const Eigen::Vector3d> v1(vector1.data() + 1, 3u);
                const Eigen::Map<const Eigen::Vector3d> v2(vector2.data() + 1, 3u);
                return std::acos(v1.dot(v2));
            }
        }  // namespace

        bool ConstrainedSO3StateSampler::satisfiesConstraint(const std::array<double, 4u>& quaternion) const
        {
            constexpr std::array<double, 4u> x {0.0, 1.0, 0.0, 0.0};
            constexpr std::array<double, 4u> y {0.0, 0.0, 1.0, 0.0};
            constexpr std::array<double, 4u> z {0.0, 0.0, 0.0, 1.0};

            const auto xprime = rotate(x, quaternion);
            if (angleBetween(x, xprime) > maxRotation_) {
                return false;
            }

            const auto yprime = rotate(y, quaternion);
            if (angleBetween(y, yprime) > maxRotation_) {
                return false;
            }

            const auto zprime = rotate(z, quaternion);
            if (angleBetween(z, zprime) > maxRotation_) {
                return false;
            }

            return true;
        }

        void ConstrainedSO3StateSampler::sampleUniform(State *state)
        {
            std::array<double, 4u> quaternion;
            do
            {
                rng_.quaternion(quaternion.data());
            } while (!satisfiesConstraint(quaternion));

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
