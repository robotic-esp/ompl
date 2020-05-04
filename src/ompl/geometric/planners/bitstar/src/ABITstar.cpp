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

/* Authors: Marlin Strub */

#include "ompl/geometric/planners/bitstar/ABITstar.h"

namespace ompl
{
    namespace geometric
    {
        ABITstar::ABITstar(const ompl::base::SpaceInformationPtr &si, const std::string &name /*= "ABITstar"*/)
          : ompl::geometric::BITstar(si, name)
        {
            // Enable cascading rewirings.
            enableCascadingRewirings();

            // Set the default initial inflation factor to very high.
            setInitialInflationFactor(1000000.0);

            // Set the default inflation factor parameter to something reasonable.
            setInflationFactorParameter(10.0);

            // Set the default truncation factor parameter to something reasonable.
            setTruncationFactorParameter(5.0);

            // Make sure the default name reflects the default k-nearest setting.
            if (getUseKNearest() && Planner::getName() == "ABITstar")
            {
                // It's the current default r-disc ABIT* name, but we're using a k-nearest RGG.
                OMPL_WARN("ABIT*: A k-nearest version of ABIT* can not be named 'ABITstar', as this name is reserved for "
                          "the r-disc version. Changing the name to 'kABITstar'.");
                Planner::setName("kABITstar");
            }
            else if (!getUseKNearest() && Planner::getName() == "kABITstar")
            {
                // It's the current default k-nearest ABIT* name, but we're using an r-disc RGG.
                OMPL_WARN("ABIT*: An r-disc version of ABIT* can not be named 'kABITstar', as this name is reserved for "
                          "the k-nearest version. Changing the name to 'ABITstar'.");
                Planner::setName("ABITstar");
            }
        }

        void ABITstar::setInitialInflationFactor(double factor)
        {
            BITstar::setInitialInflationFactor(factor);
        }

        void ABITstar::setInflationFactorParameter(double parameter)
        {
            BITstar::setInflationFactorParameter(parameter);
        }

        void ABITstar::setTruncationFactorParameter(double parameter)
        {
            BITstar::setTruncationFactorParameter(parameter);
        }

        double ABITstar::getInitialInflationFactor() const
        {
            return BITstar::getInitialInflationFactor();
        }

        double ABITstar::getCurrentInflationFactor() const
        {
            return BITstar::getCurrentInflationFactor();
        }

        double ABITstar::getCurrentTruncationFactor() const
        {
            return BITstar::getCurrentTruncationFactor();
        }

    }  // namespace geometric

}  // namespace ompl
