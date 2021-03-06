/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include "RGRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include <limits>
#include <fstream>

ompl::control::RGRRT::RGRRT(const SpaceInformationPtr &si) : base::Planner(si, "RGRRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();
    addIntermediateStates_ = false;
    lastGoalMotion_ = NULL;

    goalBias_ = 0.05;

    Planner::declareParam<double>("goal_bias", this, &RGRRT::setGoalBias, &RGRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RGRRT::setIntermediateStates, &RGRRT::getIntermediateStates);
}

ompl::control::RGRRT::~RGRRT(void)
{
    freeMemory();
}

void ompl::control::RGRRT::setup(void)
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&RGRRT::distanceFunction, this, _1, _2));
}

void ompl::control::RGRRT::clear(void)
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = NULL;
}

void ompl::control::RGRRT::freeMemory(void)
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            if (motions[i]->control)
                siC_->freeControl(motions[i]->control);
            delete motions[i];
        }
    }
}

ompl::base::PlannerStatus ompl::control::RGRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                   *goal = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

	
		ompl::control::ControlSpacePtr CSpace = siC_->getControlSpace();
		ompl::control::ControlSamplerPtr ctrlSampler = CSpace->allocControlSampler();
		base::RealVectorBounds bounds = siC_->getControlSpace()->as<RealVectorControlSpace>()->getBounds();

	//std::cout << "a\n";
    while (const base::State *st = pis_.nextStart())
    {
        TreeMotion *tmotion = new TreeMotion(siC_,ctrlSampler);
		if (!tmotion->ReachableSet)
			tmotion->ReachableSet.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
		tmotion->ReachableSet->setDistanceFunction(boost::bind(&RGRRT::distanceFunction, this, _1, _2));
        si_->copyState(tmotion->state, st);
        siC_->nullControl(tmotion->control);
        nn_->add(tmotion);
	//std::cout << "b\n";

	
		
		for (int i = 0; i < 10; i++) {
			double val[1];
			val[0] = bounds.low[0] + (bounds.high[0] - bounds.low[0]) * i/9;
			RealVectorControlSpace::ControlType * ControlSamp = new RealVectorControlSpace::ControlType();

			ControlSamp->values = val;
			base::State* result = si_->allocState();
			siC_->propagate(tmotion->getState(), ControlSamp, 1, result);
			TreeMotion* newMotion = new TreeMotion(siC_, ctrlSampler);
	        si_->copyState(newMotion->state, result);
	        siC_->copyControl(newMotion->control, ControlSamp);
			if (si_->isValid(newMotion->state)) {
				std::cout << val[0];
				std::cout << "\n";
				tmotion->ReachableSet->add(newMotion);
			}
		}	
	//std::cout << "c\n";

    }


    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting with %u states", getName().c_str(), nn_->size());

    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();

    TreeMotion      *rmotion = new TreeMotion(siC_,ctrlSampler);
    base::State  *rstate = rmotion->state;
    Control       *rctrl = rmotion->control;
    base::State  *xstate = si_->allocState();
	TreeMotion *orig = rmotion;
		TreeMotion *firstNode = NULL;


    while (ptc == false)
    {


        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

		
		rmotion = orig;
		rmotion->state = rstate;
        /* find closest state in the tree */
        TreeMotion *nmotion = (TreeMotion*)nn_->nearest(rmotion);
		if (!firstNode) {
			firstNode = nmotion;
		}
		TreeMotion *nearestReachableState = (TreeMotion*)nmotion->ReachableSet->nearest(rmotion);
		boost::shared_ptr< NearestNeighbors<Motion*> > temp;
        temp.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
	    temp->setDistanceFunction(boost::bind(&RGRRT::distanceFunction, this, _1, _2));
		temp->add(nmotion);
		temp->add(nearestReachableState);
		TreeMotion *nearestNode = (TreeMotion*)temp->nearest(rmotion);
		
		if (nmotion == nearestNode) {
		//	std::cout << "";
//			std::cout << "cool beans\n";
			continue;
		}
		else {
			//std::cout << "cool beans\n";
		}
		rmotion = nearestReachableState;
		
        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);

        if (addIntermediateStates_)
        {
            // this code is contributed by Jennifer Barry
            std::vector<base::State *> pstates;
            cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);

            if (cd >= siC_->getMinControlDuration())
            {
                Motion *lastmotion = nmotion;
                bool solved = false;
                size_t p = 0;
                for ( ; p < pstates.size(); ++p)
                {
                    /* create a motion */
                    TreeMotion *motion = new TreeMotion(siC_,ctrlSampler);
					if (!motion->ReachableSet)
						motion->ReachableSet.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
					motion->ReachableSet->setDistanceFunction(boost::bind(&RGRRT::distanceFunction, this, _1, _2));
        
                    motion->state = pstates[p];
                    //we need multiple copies of rctrl
                    motion->control = siC_->allocControl();
                    siC_->copyControl(motion->control, rctrl);
                    motion->steps = 1;
                    motion->parent = lastmotion;
                    lastmotion = motion;
                    nn_->add(motion);
	//std::cout << "d\n";

					for (int i = 0; i < 10; i++) {
						double val[1];
						val[0] = bounds.low[0] + (bounds.high[0] - bounds.low[0]) * i/9;
			
						RealVectorControlSpace::ControlType * ControlSamp = new RealVectorControlSpace::ControlType();

						ControlSamp->values = val;

						base::State* result = si_->allocState();
						siC_->propagate(motion->getState(), ControlSamp, siC_->getMinControlDuration(), result);
						TreeMotion* newMotion = new TreeMotion(siC_, ctrlSampler);
						si_->copyState(newMotion->state, result);
				        siC_->copyControl(newMotion->control, ControlSamp);

						if (si_->isValid(newMotion->state)) {
							motion->ReachableSet->add(newMotion);
						}

					}	
			
	//std::cout << "e\n";

                    double dist = 0.0;
                    solved = goal->isSatisfied(motion->state, &dist);
                    if (solved)
                    {
                        approxdif = dist;
                        solution = motion;
                        break;
                    }
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = motion;
                    }
                }

                //free any states after we hit the goal
                while (++p < pstates.size())
                    si_->freeState(pstates[p]);
                if (solved)
                    break;
            }
            else
                for (size_t p = 0 ; p < pstates.size(); ++p)
                    si_->freeState(pstates[p]);
        }
        else
        {
            if (cd >= siC_->getMinControlDuration())
            {
	//std::cout << "f\n";

                /* create a motion */
                TreeMotion *motion = new TreeMotion(siC_,ctrlSampler);
				if (!motion->ReachableSet)
					motion->ReachableSet.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
				motion->ReachableSet->setDistanceFunction(boost::bind(&RGRRT::distanceFunction, this, _1, _2));
        
                si_->copyState(motion->state, rmotion->state);
                siC_->copyControl(motion->control, rctrl);
                motion->steps = cd;
                motion->parent = nmotion;

                nn_->add(motion);

					for (int i = 0; i < 10; i++) {
						double val[1];
						val[0] = bounds.low[0] + (bounds.high[0] - bounds.low[0]) * i/9;
			
						RealVectorControlSpace::ControlType * ControlSamp = new RealVectorControlSpace::ControlType();
						//std::cout << val[0];
						//std::cout << "\n";
			
						ControlSamp->values = val;

						base::State* result = si_->allocState();
						siC_->propagate(motion->getState(), ControlSamp, siC_->getMinControlDuration(), result);

						TreeMotion* newMotion = new TreeMotion(siC_, ctrlSampler);
						si_->copyState(newMotion->state, result);
				        siC_->copyControl(newMotion->control, ControlSamp);

						if (si_->isValid(newMotion->state)) {
							motion->ReachableSet->add(newMotion);
						}
					}	
//				std::cout << "g\n";

                double dist = 0.0;
                bool solv = goal->isSatisfied(motion->state, &dist);
                if (solv)
                {
                    approxdif = dist;
                    solution = motion;
                    break;
                }
                if (dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = motion;
                }
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == NULL)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != NULL)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        PathControl *path = new PathControl(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif);
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::control::RGRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        const Motion* m = motions[i];
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state),
                             base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state),
                             base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}
