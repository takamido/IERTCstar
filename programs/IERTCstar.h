/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Rice University
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

#ifndef OMPL_GEOMETRIC_PLANNERS_EXPERIENCE_IERTCstar_
#define OMPL_GEOMETRIC_PLANNERS_EXPERIENCE_IERTCstar_

#include "ompl/geometric/planners/experience/ert/PlannerData.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/PDF.h"
#include <string>
#include <queue>

#include "ompl/base/OptimizationObjective.h"

namespace ompl
{
    namespace geometric
    {
        /**
            @anchor gERTConnect
            @par Short description
            ERTConnect is a tree-based motion planner that leverages a prior
            experience to find similar motion plans. The provided experience
            is used to bias the sampling of candidate states, and their
            connections. ERTConnect grows two ERTs, one from the start and
            one from the goal, and attempts to connect them. A solution can
            also be found from an individual tree, as each exploits the
            experience differently.
            @par External documentation
            Èric Pairet, Constantinos Chamzas, Yvan Petillot, Lydia Kavraki
            Path Planning for Manipulation using Experience-driven Random Trees
            IEEE Robotics and Automation Letters, 2021
            [[10.1109/LRA.2021.3063063]](https://ieeexplore.ieee.org/document/9366973)
            [[PDF]](https://arxiv.org/abs/2103.00448)
        */

        class IERTCstar : public base::Planner
        {
        public:
            /** \brief Constructor */
            IERTCstar(const base::SpaceInformationPtr &si);

            ~IERTCstar() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            /** \brief Set the minimum fraction of the experience to be extracted */
            void setExperienceFractionMin(double segment_fraction_min)
            {
                segmentFractionMin_ = segment_fraction_min;
            }

            /** \brief Get the minimum experience fraction the planner is using */
            double getExperienceFractionMin() const
            {
                return segmentFractionMin_;
            }

            /** \brief Set the maximum fraction of the experience to be extracted */
            void setExperienceFractionMax(double segment_fraction_max)
            {
                segmentFractionMax_ = segment_fraction_max;
            }

            /** \brief Get the maximum experience fraction the planner is using */
            double getExperienceFractionMax() const
            {
                return segmentFractionMax_;
            }
            
            void setFilename(std::string Filename)
            {
                Filename_ = Filename;
            }

            std::string getFilename() const
            {
                return Filename_;
            }
            
            void setDatalength(int Datalength)
            {
                Datalength_ = Datalength;
            }

            /** \brief Get the goal bias the planner is using */
            int getDatalength() const
            {
                return Datalength_;
            }

            /** \brief Set the tubular neighbourhood around the experience to delimit the tree expansion

                This parameter determines the extend to which ERTConnect will expand around the experience.
                Currently, the tubular neighbourhood is symmetric in all dimensions */
            void setExperienceTubularRadius(double experience_tubular_radius)
            {
                experienceTubularRadius_ = experience_tubular_radius;
            }

            /** \brief Get the tubular neighbourhood around the experience the planner is using */
            double getExperienceTubularRadius() const
            {
                return experienceTubularRadius_;
            }

            /** \brief Set the experience to exploit by the ERTConnect.

                The resolution of the experience has a significant impact on the
                planner's performance. If no experience is defined, the planner
                exploits a straight path from start to goal. */
            void setExperience(std::vector<base::State*> experience)
            {
                experience_ = new Motion(si_, experience.size());
                for (size_t i = 0; i < experience.size(); ++i)
                    si_->copyState(experience_->segment[i], experience[i]);
                experience_->phase_end = experience.size() - 1;
            }

            /** \brief Get the experience the planner is using

                Before solve it will return the same experience as provided.
                After solve it will return the updated one, if enabled */
            std::vector<base::State*> getExperience() const
            {
                return experience_->segment;
            }

            /** \brief Get the experience the planner is using

                Before calling solve(), it will return the same experience as provided.
                After calling solve() it will return the updated one, if enabled */
            PathGeometric getExperienceAsPathGeometric() const
            {
                auto path(std::make_shared<PathGeometric>(si_));
                for (auto &state : experience_->segment)
                    path->append(state);
                return *path;
            }

            /** \brief Specify whether the provided experience is to be updated as its mapping onto the current planning problem */
            void setExperienceInitialUpdate(bool experienceInitialUpdate)
            {
                experienceInitialUpdate_ = experienceInitialUpdate;
            }

            /** \brief Return true if the provided experience is to be updated as its mapping onto the current planning problem */
            bool getExperienceInitialUpdate() const
            {
                return experienceInitialUpdate_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if ((tStart_ && tStart_->size() != 0) || (tGoal_ && tGoal_->size() != 0));
                clear();
                tStart_ = std::make_shared<NN<Motion *>>();
                tGoal_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

        protected:
            /** \brief Representation of a motion

                This contains a pointer to the parent motion, and relevant
                information about the sequence of states (segment) that made
                such connection valid. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state */
                //taka:: ps=planning scene
                Motion(const base::SpaceInformationPtr &si, const unsigned int &ps) :
                    state(si->allocState()),
                    phase_span(ps)
                {
                    segment.resize(ps);
                    for (auto &s : segment)
                        s = si->allocState();
                }

                ~Motion() = default;

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};
                
                /** \brief The end state of the motion */
                base::State *state{nullptr};

                /** \brief The end phase of the motion, here defined as the index of the end state in the experience */
                unsigned int phase_end;

                /** \brief All states composing the motion */
                std::vector<base::State*> segment;

                /** \brief The phase span of the motion, here defined as the segment index size */
                unsigned int phase_span{0};

                /** \brief A pointer to the corresponding element in the probability distribution function */
                // about the end state!
                PDF<Motion *>::Element *element{nullptr};

                // NOTE: ideally, this should be a customisable payload according to a desired cost function
                /** \brief The number of attempts to expand the tree from the end state of this motion */
                unsigned int selection_count{0};
                
                //taka: from rrt star
                /** \brief The cost up to this motion */
                base::Cost cost;

                /** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance
                 * computations in the updateChildCosts() method) */
                base::Cost incCost;

                /** \brief The set of motions descending from the current motion */
                std::vector<Motion *> children;
                
                bool solution_s{false};
                bool solution_g{false};
                
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            // NOTE: ideally, this function should be a pointer to a user-defined function
            /** \brief Compute motion weight in the PDF according to the number of times such motion has been picked to expand the tree */
            double weightFunction(const Motion *m)
            {
                return (1. / (100 * m->selection_count + 1));
            }

            /** \brief Returns true if the entire motion is valid. */
            bool isSegmentValid(const Motion *tmotion);

            /** \brief Compute a motion to connect or explore. The segment is not validated. */
            void mapExperienceOntoProblem(const Motion *imotion, Motion *tmotion);

            /** \brief Attempt (once) to compute a motion to connect or explore. Returns true if the motion is valid. */
            bool getValidSegment(const Motion *imotion, Motion *tmotion, const bool &connect_flag);

            /** \brief The probability distribution function over states in the tree rooted at the start state*/
            PDF<Motion *> pdf_tStart_;

            /** \brief The probability distribution function over states in the tree rooted at the goal state */
            PDF<Motion *> pdf_tGoal_;

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief A nearest-neighbor datastructure representing a tree of motions */
            typedef std::shared_ptr<NearestNeighbors<Motion *>> TreeData;

            /** \brief A nearest-neighbors datastructure containing the tree of motions rooted at the start state */
            std::shared_ptr<NearestNeighbors<Motion *>> tStart_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions rooted at the goal state */
            std::shared_ptr<NearestNeighbors<Motion *>> tGoal_;

            /** \brief The minimum fraction of the experience to be extracted as micro-experience (segment) */
            double segmentFractionMin_{0.05};

            /** \brief The maximum fraction of the experience to be extracted as micro-experience (segment) */
            double segmentFractionMax_{0.1};

            /** \brief The tubular neighbourhood around the experience */
            double experienceTubularRadius_{5};

            /** \brief The task-relevant prior experience to leverage */
            Motion *experience_{nullptr};

            /** \brief Flag indicating whether the experience is to be mapped onto the current planning problem at start */
            bool experienceInitialUpdate_{true};
            
            int Datalength_{101};            
            std::string Filename_{"/home/parallels/human_configs.csv"};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};
            
            //////////////////////////////////////////
            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr opt_;
            
            void updateChildCosts(Motion *m);
            
            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};
            
            /** \brief The measure of the problem when we pruned it (if this isn't in use, it will be set to
             * si_->getSpaceMeasure())*/
            double prunedMeasure_{0.};
            
            /** \brief Best cost found so far by algorithm */
            base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

            /** \brief The cost at which the graph was last pruned */
            base::Cost prunedCost_{std::numeric_limits<double>::quiet_NaN()};
                        
            /** \brief The tree is pruned when the change in solution cost is greater than this fraction. */
            double pruneThreshold_{.05};
            
            /** \brief Check whether the given motion passes the specified cost threshold, meaning it will be \e kept
             * during pruning */
            bool keepCondition(const Motion *motion, const base::Cost &threshold) const;
            
            base::Cost solutionHeuristic(const Motion *motion) const;
            
            /** \brief The best goal motion. */
            Motion *bestGoalMotion_{nullptr};  
         
            std::vector<Motion *> goalList_stree_;
            std::vector<Motion *> goalList_gtree_;
            
            std::vector<std::shared_ptr<Motion *>> delete_motion_list_;
            
            /** \brief Add the children of a vertex to the given list. */
            void addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion);
            
            /** \brief Stores the start states as Motions. */
            std::vector<Motion *> startMotions_;            
        };
    }
}

#endif
