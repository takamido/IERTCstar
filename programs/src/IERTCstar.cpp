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

#include "ompl/geometric/planners/experience/IERTCstar.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/PlannerDataStorage.h"
#include "ompl/base/StateStorage.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/tools/experience/ExperienceSetup.h"

#include <stdio.h>
#include <typeinfo>
#include <algorithm>
#include <limits>
#include <list>

#include <iostream>
#include <fstream>
#include <string> 
#include <time.h>
using namespace std;

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/OptimizationObjective.h"

ompl::geometric::IERTCstar::IERTCstar(const base::SpaceInformationPtr &si)
  : base::Planner(si, "IERTCstar")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    Planner::declareParam<double>("experience_fraction_min", this, &IERTCstar::setExperienceFractionMin, &IERTCstar::getExperienceFractionMin, "0.:.05:1.");
    Planner::declareParam<double>("experience_fraction_max", this, &IERTCstar::setExperienceFractionMax, &IERTCstar::getExperienceFractionMax, "0.:.05:1.");
    Planner::declareParam<double>("experience_tubular_radius", this, &IERTCstar::setExperienceTubularRadius, &IERTCstar::getExperienceTubularRadius, "0.:.05:10000.");
    Planner::declareParam<bool>("experience_initial_update", this, &IERTCstar::setExperienceInitialUpdate, &IERTCstar::getExperienceInitialUpdate, "0,1");

    Planner::declareParam<std::string>("filename", this, &IERTCstar::setFilename, &IERTCstar::getFilename, "input filename of csv file");
    Planner::declareParam<int>("data_length", this, &IERTCstar::setDatalength, &IERTCstar::getDatalength, "input number of data points of experience's path");
}

ompl::geometric::IERTCstar::~IERTCstar()
{
    freeMemory();

    /* in the deconstructor as MoveIt calls freeMemory() before planning */
    if (experience_)
    {
        for (auto &state : experience_->segment)
            if (state != nullptr)
                si_->freeState(state);
        experience_->segment.clear();
        delete experience_;
    }
}

void ompl::geometric::IERTCstar::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();

    pdf_tStart_.clear();
    pdf_tGoal_.clear();
    prunedMeasure_ = 0.0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());

    goalList_stree_.clear();
    goalList_gtree_.clear();  
    startMotions_.clear();
    //delete_motion_list_.clear();
}

void ompl::geometric::IERTCstar::freeMemory()
{
    std::vector<Motion *> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            for (auto &state : motion->segment)
                if (state != nullptr)
                    si_->freeState(state);
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (auto &motion : motions)
        {
            for (auto &state : motion->segment)
                if (state != nullptr)
                    si_->freeState(state);
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
          
}

void ompl::geometric::IERTCstar::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());      

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));

    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

    if(pdef_->hasOptimizationObjective()){
        opt_ = pdef_->getOptimizationObjective();
    }else{
        opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
        pdef_->setOptimizationObjective(opt_);
    }

    // Get the measure of the entire space:
    prunedMeasure_ = si_->getSpaceMeasure();
    bestCost_ = opt_->infiniteCost();
    prunedCost_ = opt_->infiniteCost();
}

bool ompl::geometric::IERTCstar::isSegmentValid(const Motion *tmotion)
{
    /* check first state as next checkMotion assumes it valid */
    if (!si_->isValid(tmotion->segment[0]))
        return false;

    /* check the motions (segments) between states */
    for (unsigned int i = 0; i < tmotion->phase_span - 1; ++i)
        if (!si_->checkMotion(tmotion->segment[i], tmotion->segment[i + 1]))
            return false;
    return true;
}

void ompl::geometric::IERTCstar::mapExperienceOntoProblem(const Motion *imotion, Motion *tmotion) //taka: imotion: original, tmotion: mapping
{
    const auto ss = si_->getStateSpace();
    const auto &locations = ss->getValueLocations();
    const unsigned int dimensionality = locations.size();
    std::vector<double> b(dimensionality), l(dimensionality);

    /* compute tranform parameters to connect */
    tmotion->phase_span = std::abs(int(tmotion->phase_end) - int(imotion->phase_end)) + 1;
    for (size_t i = 0; i < dimensionality; ++i)
    {
        b[i] = *(ss->getValueAddressAtLocation(imotion->state, locations[i])) - *(ss->getValueAddressAtLocation(experience_->segment[imotion->phase_end], locations[i]));
        l[i] = *(ss->getValueAddressAtLocation(tmotion->state, locations[i])) - (*(ss->getValueAddressAtLocation(experience_->segment[tmotion->phase_end], locations[i])) + b[i]);
    }

    /* transform segment */
    double t;
    for (size_t i = 0; i < tmotion->phase_span; ++i)
    {
        t = double(i) / double(tmotion->phase_span - 1);
        for (size_t j = 0; j < dimensionality; ++j)
            *(ss->getValueAddressAtLocation(tmotion->segment[i], locations[j])) = *(ss->getValueAddressAtLocation(experience_->segment[imotion->phase_end + i], locations[j])) + t * l[j] + b[j];
        ss->enforceBounds(tmotion->segment[i]);
    }
    si_->copyState(tmotion->state, tmotion->segment[tmotion->phase_span - 1]);
}

bool ompl::geometric::IERTCstar::getValidSegment(const Motion *imotion, Motion *tmotion, const bool &connect_flag)
{
    /* when connecting the trees, the nearest motion might have the same phase */
    if (imotion->phase_end == tmotion->phase_end)
    {
        if (si_->checkMotion(imotion->state, tmotion->state))
        {
            si_->copyState(tmotion->segment[0], imotion->state);
            si_->copyState(tmotion->segment[1], tmotion->state);
            tmotion->phase_span = 2;
            return true;
        }
        return false;
    }

    const auto ss = si_->getStateSpace();
    const auto &locations = ss->getValueLocations();
    const unsigned int dimensionality = locations.size();
    std::vector<double> b(dimensionality), l(dimensionality);

    /* segment direction */
    int increment = int(tmotion->phase_end) - int(imotion->phase_end);
    int direction = (increment > 0) - (increment < 0);
    tmotion->phase_span = std::abs(increment) + 1;

    /* compute tranform parameters */
    if (connect_flag)
    {
        /* compute transform parameters to connect */
        for (size_t i = 0; i < dimensionality; ++i)
        {
            b[i] = *(ss->getValueAddressAtLocation(imotion->state, locations[i])) - *(ss->getValueAddressAtLocation(experience_->segment[imotion->phase_end], locations[i]));
            l[i] = *(ss->getValueAddressAtLocation(tmotion->state, locations[i])) - (*(ss->getValueAddressAtLocation(experience_->segment[tmotion->phase_end], locations[i])) + b[i]);
        }
    }
    else
    {
        /* compute transform parameters to explore */
        base::State *xstate = si_->allocState();
        double noise = (tmotion->phase_span - 1) * experienceTubularRadius_ / (experience_->phase_span - 1);
        for (size_t i = 0; i < dimensionality; ++i)
        {
            b[i] = *(ss->getValueAddressAtLocation(imotion->state, locations[i])) - *(ss->getValueAddressAtLocation(experience_->segment[imotion->phase_end], locations[i]));
            *(ss->getValueAddressAtLocation(xstate, locations[i])) = *(ss->getValueAddressAtLocation(experience_->segment[tmotion->phase_end], locations[i])) + b[i];
        }

        /* sample and validate target state */
        sampler_->sampleUniformNear(tmotion->state, xstate, noise); /* already enforcing bounds */
        if (!si_->isValid(tmotion->state))
        {
            si_->freeState(xstate);
            return false;
        }

        for (size_t i = 0; i < dimensionality; ++i)
            l[i] = *(ss->getValueAddressAtLocation(tmotion->state, locations[i])) - *(ss->getValueAddressAtLocation(xstate, locations[i]));

        si_->freeState(xstate);
    }

    /* transform segment while valid */
    double t;
    si_->copyState(tmotion->segment[0], imotion->state);
    for (size_t i = 1; i < tmotion->phase_span; ++i)
    {
        base::State *xstate = si_->allocState(); /* need to allocate memory per state as otherwise the validity flag of the previous check skips the checking of the new values */
        t = double(i) / double(tmotion->phase_span - 1);
        for (size_t j = 0; j < dimensionality; ++j)
            *(ss->getValueAddressAtLocation(xstate, locations[j])) = *(ss->getValueAddressAtLocation(experience_->segment[imotion->phase_end + i * direction], locations[j])) + t * l[j] + b[j];

        if (!si_->checkMotion(tmotion->segment[i - 1], xstate))
        {
            si_->freeState(xstate);
            return false;
        }

        si_->copyState(tmotion->segment[i], xstate);
        si_->freeState(xstate);
    }
    return true;
}

std::vector<std::string> split_data(std::string& input, char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

ompl::base::PlannerStatus ompl::geometric::IERTCstar::solve(const base::PlannerTerminationCondition &ptc)
{
    clock_t start_time = clock();
    clock_t find_time = clock();
    checkValidity();
    bool symCost = opt_->isSymmetric();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal_s == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_, 0);
        si_->copyState(motion->state, st);
        motion->phase_end = 0;
        motion->element = pdf_tStart_.add(motion, weightFunction(motion));
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal_s->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    /* map experience onto current planning problem; currently only one experience is created for a selected start and goal */
    auto *smotion = new Motion(si_, 0);
    smotion->state = pdef_->getStartState(0);
    smotion->phase_end = 0;
    startMotions_.push_back(smotion);

    auto *gmotion = new Motion(si_, 0);
    const base::State *st = pis_.nextGoal(ptc); //pis_: PlannerInputStates
    if (st != nullptr)
    {
        si_->copyState(gmotion->state, st);
        gmotion->element = pdf_tGoal_.add(gmotion, weightFunction(gmotion));
        tGoal_->add(gmotion);
    }
    else
    {
        OMPL_ERROR("%s: Unable to sample any valid state for goal tree", getName().c_str());
        return base::PlannerStatus::TIMEOUT;
    }

    //generate experience from csv file
    //read csv file (please add the path information for two files)
    std::ifstream ifs0("~/filename_for_ompl.txt");
    getline(ifs0, Filename_);
    OMPL_INFORM("File name of experience: %s",Filename_);

    std::ifstream ifs1("~/point_num_for_ompl.txt");
    std::string length;
    getline(ifs1, length);
    Datalength_ = atoi(length.c_str());

    std::ifstream ifs2(Filename_);
    std::string line;
    double *dNum=new double[(Datalength_)*dof_];;
    int count=0;
    while (getline(ifs2, line)) {
        std::vector<std::string> strvec = split_data(line, ',');
        for (int i=0; i<strvec.size();i++){
            dNum[count] = std::stod(strvec.at(i));
            count++;
        }
    }

    double *joint_value;
    base::StateStorage space_experience=base::StateStorage(si_->getStateSpace());
    for (int number = 0; number < Datalength_; number++)
    { 
     ompl::base::State *State_x = (si_->getStateSpace())->allocState();
     for (int number2 = 0; number2 < dof_; number2++)
     {
       joint_value=(si_->getStateSpace())->getValueAddressAtIndex(State_x,number2);
       *joint_value=dNum[dof_*number+number2];
     }
     space_experience.addState(State_x);
     (si_->getStateSpace())->freeState(State_x);
    }   

    int dof_=(si_->getStateSpace())->getDimension();

    /*Set and Get experience in experience_*/
    OMPL_INFORM("Start to load data");
    std::vector<base::State*> experience_states;
    experience_states.resize(space_experience.size());
    for (unsigned i = 0; i < space_experience.size(); ++i){
        experience_states[i]=space_experience.getState(i);
    }
    setExperience(experience_states);

    if (!experience_)
    {
        OMPL_INFORM("%s: No experience provided. Setting straight experience", getName().c_str());
        experience_ = new Motion(si_, si_->getStateSpace()->validSegmentCount(smotion->state, gmotion->state) + 1);
        experience_->phase_end = experience_->phase_span - 1;
        gmotion->phase_end = experience_->phase_end;
        si_->getMotionStates(smotion->state, gmotion->state, experience_->segment, experience_->phase_span - 2, true, false);
    }
    else
    {
        /* update experience as its mapping onto the current planning problem */
        if (experienceInitialUpdate_)
        {   
            Motion *tmotion = new Motion(si_, 0);
            si_->copyState(tmotion->state, gmotion->state);
            tmotion->phase_end = experience_->phase_end;
            tmotion->segment.resize(experience_->phase_span);
            tmotion->segment = experience_->segment; /* copy the pointers to update the experience_ */
            gmotion->phase_end = experience_->phase_end;
            mapExperienceOntoProblem(smotion, tmotion);
        }
    }
    OMPL_INFORM("%s: Updated experience has %u states", getName().c_str(), experience_->phase_span);

    /* proceed with the planning */
    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(), (int)(tStart_->size() + tGoal_->size()));

    bool startTree = false;
    double approxdif = std::numeric_limits<double>::infinity();
    bool approximate = false;

    Motion *imotion;
    Motion *tmotion = new Motion(si_, experience_->phase_span); /* pre-allocate memory for candidate segments */
    Motion *addedMotion = nullptr;
    Motion *otherAddedMotion = nullptr;
    Motion *testMotion = nullptr;

    bool connect_flag;
    if (segmentFractionMin_ > segmentFractionMax_)
    {
        OMPL_WARN("segmentFractionMin_ > segmentFractionMax_, swapping parameters.");
        double tmp = segmentFractionMin_;
        segmentFractionMin_ = segmentFractionMax_;
        segmentFractionMax_ = tmp;
    }
    unsigned int min_inc = std::max((unsigned int)(segmentFractionMin_ * experience_->phase_end), (unsigned int)(1));
    unsigned int max_inc = std::max((unsigned int)(segmentFractionMax_ * experience_->phase_end), (unsigned int)(min_inc + 1));
    
    const auto ss = si_->getStateSpace();
    const auto &locations = ss->getValueLocations();
    int sindex=0;

    std::vector<base::Cost> costs;
    std::vector<base::Cost> incCosts;
    std::vector<int> valid;
    auto best_path(std::make_shared<PathGeometric>(si_));
    bool solved = false;
    bool solved_temp = false;
    int k_rrt_=1;
    bool improve=false;

    int counter=0;

    // start planning loop
    while (!ptc)
    {   
        counter++;
        
        solved_temp = false;
        improve=false;

        // change direction for every iteration
        startTree = !startTree;
        TreeData &tree = startTree ? tStart_ : tGoal_;
        TreeData &otherTree = startTree ? tGoal_ : tStart_;
        PDF<Motion*> &pdf = startTree ? pdf_tStart_ : pdf_tGoal_;
        PDF<Motion*> &otherPdf = startTree ? pdf_tGoal_ : pdf_tStart_;
        Motion *target = startTree ? gmotion : smotion;

        addedMotion = nullptr;
        otherAddedMotion = nullptr;
        testMotion=nullptr;
        Motion *startMotion = nullptr;
        Motion *goalMotion = nullptr;

        /* pick a state from the tree */
        imotion = pdf.sample(rng_.uniform01());    
        imotion->selection_count++;
        pdf.update(imotion->element, weightFunction(imotion));

        /* sample candidate segment to explore from the pick-up state */
        connect_flag = false;
        if (startTree){
            sindex=1;
            tmotion->phase_end = rng_.uniformInt(std::min(imotion->phase_end + min_inc, experience_->phase_end), std::min(imotion->phase_end + max_inc, experience_->phase_end));
        }
        else{
            sindex=0;
            tmotion->phase_end = rng_.uniformInt(std::max(0, (int)imotion->phase_end - (int)max_inc), std::max(0, (int)imotion->phase_end - (int)min_inc));
        }            

        /* attempt generating a valid segment to connect or explore */
        if (!getValidSegment(imotion, tmotion, connect_flag)){
            continue; // skip latter processes and go-back to the top of this loop
        }

        //check whether initial and end states can be directly connected
        bool direct_connection=false;
        if (si_->checkMotion(imotion->state, tmotion->state))
        {
            direct_connection=true;
        }

        // create new motion to copy the tmotion on new memory
        Motion *motion = new Motion(si_, tmotion->phase_span);
        si_->copyState(motion->state, tmotion->state);
        if (direct_connection)
        {
            motion->segment.resize(2);
            si_->copyState(motion->segment[0], imotion->state);
            si_->copyState(motion->segment[1], tmotion->state);
            motion->phase_span = 2;
        }
        else
        {
            for (size_t i = 0; i < tmotion->phase_span; ++i)
                si_->copyState(motion->segment[i], tmotion->segment[i]);
            motion->phase_span = tmotion->phase_span;
        }
        motion->phase_end = tmotion->phase_end;
        motion->parent = imotion;

        // set neighborhoods of the new motion
        std::vector<Motion *> nbh;
        auto cardDbl = static_cast<double>(tree->size() + 1u);
        unsigned int k = std::ceil(k_rrt_ * log(cardDbl));
        tree->nearestK(motion, k, nbh);

        costs.resize(nbh.size());
        incCosts.resize(nbh.size());
        valid.resize(nbh.size());
        std::fill(valid.begin(), valid.begin() + nbh.size(), 0);

        //calculate cost
        base::Cost cost_temp;
        for (size_t i = 0; i < (motion->phase_span-1); ++i)
                cost_temp=opt_->combineCosts(cost_temp,opt_->motionCost(motion->segment[i],motion->segment[i+1]));

        motion->incCost = cost_temp;
        motion->cost = opt_->combineCosts(imotion->cost, motion->incCost);  

        //reject sampling  
        if (solved)
        {
            if (!keepCondition(motion, bestCost_))
            {
                //free memory & delete motion
                si_->freeState(motion->state);
                for (int i = 0; i < motion->segment.size(); ++i)
                     si_->freeState(motion->segment[i]);
                delete motion;

                continue;  //skip the latter processes and return to the top of the loop
            }
        }

        //if acceptable, try to find beter connection with new sampling point
        for (std::size_t i = 0; i < nbh.size(); ++i)
        {
            incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);//assuming direct connection
            costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);

            if (opt_->isCostBetterThan(costs[i], motion->cost)){
                if (si_->checkMotion(nbh[i]->state, motion->state)){
                    motion->incCost = incCosts[i];
                    motion->cost = costs[i];
                    motion->parent = nbh[i];//change parent
    
                    for (int j=2; j<motion->segment.size();j++)
                         si_->freeState(motion->segment[j]);

                    motion->segment.resize(2);//directly connect from parent
                    si_->copyState(motion->segment[0], nbh[i]->state);
                    si_->copyState(motion->segment[1], motion->state);
                    motion->phase_span = 2;
                    valid[i] = 1;
                }
                else
                    valid[i]=-1;
            }
        }

        //add this motion to the tree
        tree->add(motion);
        motion->element = pdf.add(motion, 1.0);
        motion->parent->children.push_back(motion);
            
        //find better connection using new state as the parent
        bool checkForSolution = false;
        for (std::size_t i = 0; i < nbh.size(); ++i)
        {
            base::Cost nbhIncCost;
            if (symCost)
                nbhIncCost = incCosts[i];       
            else
                nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);

            base::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
            if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))
            {
                bool motionValid = (valid[i] == 1);
                if (motionValid)
                {
                    //remove this state from parent
                    for (auto it = nbh[i]->parent->children.begin(); it != nbh[i]->parent->children.end(); ++it)
                    {
                        if (*it == nbh[i])
                        {
                            nbh[i]->parent->children.erase(it);
                            break;
                        }
                    }

                    //assign new parent 
                    nbh[i]->parent = motion;
                    nbh[i]->incCost = nbhIncCost;
                    nbh[i]->cost = nbhNewCost;     

                    for (int j=2; j<nbh[i]->segment.size();j++)
                         si_->freeState(nbh[i]->segment[j]);

                    nbh[i]->segment.resize(2);//directly connect from parent
                    si_->copyState(nbh[i]->segment[0], motion->state);
                    si_->copyState(nbh[i]->segment[1], nbh[i]->state);
                    nbh[i]->phase_span = 2;

                    motion->children.push_back(nbh[i]);
                    updateChildCosts(nbh[i]);
                    checkForSolution = true;
                }
            }  
        }

        /* remember which motion was just added and check if it reaches the other end */
        addedMotion = motion;

        /* attempt connecting to last added motion from the nearest state in the other tree*/
        imotion = otherTree->nearest(addedMotion);
        imotion->selection_count++;
        otherPdf.update(imotion->element, weightFunction(imotion));

        /* attempt generating a valid segment to connect to the other tree */
        if (si_->checkMotion(imotion->state, addedMotion->state))
        {
            Motion *other_motion = new Motion(si_, 2);
            si_->copyState(other_motion->state, addedMotion->state);
            other_motion->parent = imotion;
            other_motion->phase_span = 2;
            other_motion->phase_end = addedMotion->phase_end;
            si_->copyState(other_motion->segment[0], imotion->state);
            si_->copyState(other_motion->segment[1], addedMotion->state);
            imotion->children.push_back(other_motion);
            other_motion->element = otherPdf.add(other_motion, weightFunction(other_motion));
            other_motion->incCost = opt_->motionCost(imotion->state,addedMotion->state);
            other_motion->cost = opt_->combineCosts(imotion->cost, other_motion->incCost);                
            otherTree->add(other_motion);

            /* remember the otherAddedMotion */
            otherAddedMotion = other_motion;
            solved_temp = true;
        }

        if (solved_temp)
        {
            startMotion = startTree ? addedMotion : otherAddedMotion;
            goalMotion = startTree ? otherAddedMotion : addedMotion;
        }

        // check improvements by rewiring
        if (checkForSolution && !goalList_stree_.empty())
        {
            Motion *goal;
            for (int i = 0; i < goalList_stree_.size(); ++i)
            {
                // re-construct solution path 
                goal = goalList_stree_[i];
                std::vector<Motion *> mpath1;
                mpath1.clear();
                while (goal != nullptr)
                {
                    mpath1.push_back(goal);
                    goal = goal->parent;
                }

                goal = goalList_gtree_[i];
                std::vector<Motion *> mpath2;
                mpath2.clear();
                while (goal != nullptr)
                {
                    mpath2.push_back(goal);
                    goal = goal->parent;
                }

                if (!(mpath1[0]->segment).empty())
                {

                    base::Cost total_cost=opt_->combineCosts(mpath1[0]->cost,mpath2[0]->cost);
                    auto path(std::make_shared<PathGeometric>(si_));
                    if (total_cost.value()<bestCost_.value())
                    {
                        for (int i = mpath1.size() - 1; i >= 0; --i)
                        {
                            if (!mpath1[i]->parent) //start point
                                path->append(mpath1[i]->state);
                            else
                                for (size_t j = 1; j < mpath1[i]->segment.size(); ++j)
                                    path->append(mpath1[i]->segment[j]);      
                        }

                        for (auto &temp_motion : mpath2)
                        {
                            if (!temp_motion->parent)
                                path->append(temp_motion->state);
                            else
                                for (int j = temp_motion->segment.size() - 1; j > 0; --j)
                                        path->append(temp_motion->segment[j]);
                        }

                        bestCost_=total_cost;
                        best_path=path;
                        improve=true;
                        OMPL_INFORM("update best cost by re-wiring: %f, %d samples", bestCost_.value(),counter); 
                    } 
                }    
            }
        }   

        if (startMotion || goalMotion)
        {
            if (startTree)
            {
                addedMotion->solution_s=true;
                otherAddedMotion->solution_g=true;

                goalList_stree_.push_back(addedMotion);
                goalList_gtree_.push_back(otherAddedMotion);
            }
            else
            {
                addedMotion->solution_g=true;
                otherAddedMotion->solution_s=true;

                goalList_stree_.push_back(otherAddedMotion);
                goalList_gtree_.push_back(addedMotion);
            }

            Motion *solution = startMotion;
            std::vector<Motion *> mpath1;
            while (solution != nullptr)
            {
                mpath1.push_back(solution);
                solution = solution->parent;
            }

            solution = goalMotion;
            std::vector<Motion *> mpath2;
            while (solution != nullptr)
            {
                mpath2.push_back(solution);
                solution = solution->parent;
            }  

            base::Cost total_cost=opt_->combineCosts(mpath1[0]->cost,mpath2[0]->cost);

            //if there are no existing solution, constrauct solution path and save
            if (!solved)
            {      
                auto path(std::make_shared<PathGeometric>(si_));

                for (int i = mpath1.size() - 1; i >= 0; --i)
                {
                    if (!mpath1[i]->parent)
                        path->append(mpath1[i]->state);
                    else
                        for (size_t j = 1; j < mpath1[i]->segment.size(); ++j)
                            path->append(mpath1[i]->segment[j]);
                }                    

                for (auto &motion : mpath2)
                {
                    if (!motion->parent)
                        path->append(motion->state);
                    else
                        for (int j = motion->segment.size() - 1; j > 0; --j)
                            path->append(motion->segment[j]);
                }
                
                bestCost_=total_cost;
                best_path=path;
                improve=true;
                solved=true;
                find_time=clock();
                
                OMPL_INFORM("initial solution cost: %f, %d samples", bestCost_.value(),counter);                                         
            }
            else if(solved) //if threre is a existing solution, compare the cost with it
            {
                // if it is better than the current one, replace the best path
                if(total_cost.value()<bestCost_.value())
                {    
                    auto path(std::make_shared<PathGeometric>(si_));
                    for (int i = mpath1.size() - 1; i >= 0; --i)
                    {
                        if (!mpath1[i]->parent)
                            path->append(mpath1[i]->state);
                        else
                            for (size_t j = 1; j < mpath1[i]->segment.size(); ++j)
                                path->append(mpath1[i]->segment[j]);
                    }                    

                    for (auto &motion : mpath2)
                    {
                        if (!motion->parent)
                            path->append(motion->state);
                        else
                            for (int j = motion->segment.size() - 1; j > 0; --j)
                                path->append(motion->segment[j]);
                    }

                    improve=true;
                    bestCost_=total_cost;
                    best_path=path;
                    OMPL_INFORM("update best cost by new sampling: %f, %d samples", bestCost_.value(),counter);
                }
            }

            //informed processing (pruning)
            if (improve)
            {
                //Calculate the percent improvement (expressed as a [0,1] fraction) in cost
                double fracBetter;
                if (opt_->isFinite(prunedCost_))
                    fracBetter = std::abs((bestCost_.value() - prunedCost_.value()) / prunedCost_.value());
                else
                    fracBetter = 1.0;

                if (fracBetter > pruneThreshold_)
                {
                    OMPL_INFORM("start prune tree");
                    int numPruned=0;

                    // two queues for bidirectional search trees
                    std::queue<Motion *, std::deque<Motion *>> startMotionQueue, goalMotionQueue;
                    std::queue<Motion *, std::deque<Motion *>> startLeavesToPrune, goalLeavesToPrune;
                    std::list<Motion *> startChainsToRecheck, goalChainsToRecheck;

                    // clear current tree structure and goal_motion_list
                    tStart_->clear();
                    tGoal_->clear();
                    pdf_tStart_.clear();
                    pdf_tGoal_.clear();

                    // add start and goal state
                    tStart_->add(mpath1.back());
                    tGoal_->add(mpath2.back());
                    mpath1.back()->element=pdf_tStart_.add(mpath1.back(), weightFunction(mpath1.back()));
                    mpath2.back()->element=pdf_tGoal_.add(mpath2.back(), weightFunction(mpath2.back()));   

                    //start prune from start and goal state
                    addChildrenToList(&startMotionQueue, mpath1.back());   
                    addChildrenToList(&goalMotionQueue, mpath2.back());

                    //prune start tree
                    while (!startMotionQueue.empty())
                    {
                        Motion *currentMotion = startMotionQueue.front();
                        startMotionQueue.pop();

                        if (keepCondition(currentMotion, bestCost_)) //if we keep the point
                        {
                            tStart_->add(currentMotion);
                            currentMotion->element=pdf_tStart_.add(currentMotion, weightFunction(currentMotion));
                        
                            //update queue with children nodes
                            addChildrenToList(&startMotionQueue,currentMotion);
                        }
                        else
                        {
                            if (currentMotion->children.empty())
                            {
                                startLeavesToPrune.push(currentMotion);
                            }
                            else
                            {
                                bool keepAChild = false;
                                for (unsigned int i = 0; !keepAChild && i < currentMotion->children.size(); ++i)
                                {
                                    keepAChild = keepCondition(currentMotion->children.at(i), bestCost_);
                                }

                                if (keepAChild)
                                {
                                    tStart_->add(currentMotion);
                                    currentMotion->element=pdf_tStart_.add(currentMotion, weightFunction(currentMotion));
                                }
                                else
                                {
                                    startChainsToRecheck.push_back(currentMotion);
                                }
                                addChildrenToList(&startMotionQueue, currentMotion);
                            }
                        }
                    }               

                    //prune goal tree
                    while (!goalMotionQueue.empty())
                    {
                        Motion *currentMotion = goalMotionQueue.front();
                        goalMotionQueue.pop();

                        if (keepCondition(currentMotion, bestCost_))
                        {
                            tGoal_->add(currentMotion);
                            currentMotion->element=pdf_tGoal_.add(currentMotion, weightFunction(currentMotion));
                            addChildrenToList(&goalMotionQueue, currentMotion);
                        }
                        else
                        {
                            if (currentMotion->children.empty())
                            {
                                goalLeavesToPrune.push(currentMotion);
                            }
                            else
                            {
                                bool keepAChild = false;
                                for (unsigned int i = 0; !keepAChild && i < currentMotion->children.size(); ++i)
                                {
                                    keepAChild = keepCondition(currentMotion->children.at(i), bestCost_);
                                }

                                if (keepAChild)
                                {
                                    tGoal_->add(currentMotion);
                                    currentMotion->element=pdf_tGoal_.add(currentMotion, weightFunction(currentMotion));
                                }
                                else
                                {
                                    goalChainsToRecheck.push_back(currentMotion);
                                }

                                addChildrenToList(&goalMotionQueue, currentMotion);
                            }
                        }
                    } 

                    // pruning for both trees and update Goalmotion_list
                    while (!startLeavesToPrune.empty())
                    {
                        Motion *leaf = startLeavesToPrune.front();
                        startLeavesToPrune.pop();

                        //find the corresponding index in the goal list
                        auto it = std::find(goalList_stree_.begin(), goalList_stree_.end(), leaf);
                        if (it != goalList_stree_.end())
                        {
                            // get the index of the motion
                            int index = std::distance(goalList_stree_.begin(), it);

                            // remove the motion from both GoalList and GoalList2 at the same index
                            goalList_stree_.erase(goalList_stree_.begin() + index);
                            goalList_gtree_.erase(goalList_gtree_.begin() + index);
                        }

                        // remove the leaf from its parent and prune
                        for (auto it = leaf->parent->children.begin(); it != leaf->parent->children.end(); ++it)
                        {
                            if (*it == leaf)
                            {
                                leaf->parent->children.erase(it);
                                break;
                            }
                        }

                        si_->freeState(leaf->state);
                        for (size_t j = 1; j < leaf->segment.size(); ++j)
                            si_->freeState(leaf->segment[j]);
                        delete leaf;

                        ++numPruned;
                    }

                    while (!goalLeavesToPrune.empty())
                    {
                        Motion *leaf = goalLeavesToPrune.front();
                        goalLeavesToPrune.pop();

                        auto it = std::find(goalList_gtree_.begin(), goalList_gtree_.end(), leaf);
                        if (it != goalList_gtree_.end())
                        {
                            int index = std::distance(goalList_gtree_.begin(), it);
                            goalList_gtree_.erase(goalList_gtree_.begin() + index);
                            goalList_stree_.erase(goalList_stree_.begin() + index);
                        }

                        for (auto it = leaf->parent->children.begin(); it != leaf->parent->children.end(); ++it)
                        {
                            if (*it == leaf)
                            {
                                leaf->parent->children.erase(it);
                                break;
                            }
                        }
                        si_->freeState(leaf->state);
                        for (size_t j = 1; j < leaf->segment.size(); ++j)
                            si_->freeState(leaf->segment[j]);
                        delete leaf;

                        ++numPruned;                        
                    }
                    prunedCost_ = bestCost_;
                    OMPL_INFORM("finish pruning, numPruned: %d", numPruned);
                }
            }   
        }       
    }

    pdef_->addSolutionPath(best_path);//(*best_path).cost.value
    OMPL_INFORM("final cost:%f",(*best_path).length());

    for (auto &state : tmotion->segment)
        si_->freeState(state);
    si_->freeState(tmotion->state);
    delete tmotion;
    
    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(), tStart_->size(), tGoal_->size());
    OMPL_INFORM("total iteration number: %d",counter);
    delete[] dNum;
    return solved ? base::PlannerStatus::EXACT_SOLUTION : (approximate ? base::PlannerStatus::APPROXIMATE_SOLUTION : base::PlannerStatus::TIMEOUT);    
}

void ompl::geometric::IERTCstar::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1), ert::PlannerDataEdgeSegment(motion->segment));
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
        else
            data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2), ert::PlannerDataEdgeSegment(motion->segment));
    }
}

bool ompl::geometric::IERTCstar::keepCondition(const Motion *motion, const base::Cost &threshold) const
{
    // We keep if the cost-to-come-heuristic of motion is <= threshold, by checking
    // if !(threshold < heuristic), as if b is not better than a, then a is better than, or equal to, b
    if (bestGoalMotion_ && motion == bestGoalMotion_)
    {
        // If the threshold is the theoretical minimum, the bestGoalMotion_ will sometimes fail the test due to floating point precision. Avoid that.
        return true;
    }

    return !opt_->isCostBetterThan(threshold, solutionHeuristic(motion));
}

ompl::base::Cost ompl::geometric::IERTCstar::solutionHeuristic(const Motion *motion) const
{
    base::Cost costToCome;
    costToCome = motion->cost;

    const base::Cost costToGo =
        opt_->costToGo(motion->state, pdef_->getGoal().get());  // lower-bounding cost from the state to the goal
    return opt_->combineCosts(costToCome, costToGo);
}

void ompl::geometric::IERTCstar::addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion)
{
    for (auto &child : motion->children)
    {
        motionList->push(child);
    }
}

void ompl::geometric::IERTCstar::updateChildCosts(Motion *m)
{
    //OMPL_INFORM("children size: %d", m->children.size());
    for (std::size_t i = 0; i < m->children.size(); ++i)
    {
        m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
        updateChildCosts(m->children[i]);
    }
}