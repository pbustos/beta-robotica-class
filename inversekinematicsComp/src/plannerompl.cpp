/*
 * Copyright 2014 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "plannerompl.h"



PlannerOMPL::PlannerOMPL(const InnerModel &innerModel_, QObject *parent)
{
// 	xMin = 0.;
// 	xMax = 10000.;
// 	zMin = -10000.;
// 	zMax = 0.;
// 	
// 	innerModel = new InnerModel(innerModel_);
// 	
// 	//Create state space as R2
// 	ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();
// 	space->addDimension(xMin, xMax);
// 	space->addDimension(zMin, zMax);
// 	
// 	//Setup class
// 	simpleSetUp.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));
// 	
// 	//set Sampler
// 	simpleSetUp->getSpaceInformation()->setValidStateSamplerAllocator(allocOBValidStateSampler);
// 	
// 	// set state validity checking for this space
// 	//simpleSetUp->setStateValidityChecker(boost::bind(&PlannerOMPL::isStateValid, this, _1));
// 	simpleSetUp->setStateValidityChecker(boost::bind(&Sampler::isStateValid, sampler, _1));
// 	space->setup();
// 	simpleSetUp->getSpaceInformation()->setStateValidityCheckingResolution(0.01);
// 	//simpleSetUp->getSpaceInformation()->setStateValidityCheckingResolution(100 / space->getMaximumExtent());
// 	simpleSetUp->setPlanner(ob::PlannerPtr(new og::RRTConnect(simpleSetUp->getSpaceInformation())));
// 	simpleSetUp->getPlanner()->as<og::RRTConnect>()->setRange(2000);
// 	//simpleSetUp->setPlanner(ob::PlannerPtr(new og::RRT(simpleSetUp->getSpaceInformation())));
// 	//simpleSetUp->setPlanner(ob::PlannerPtr(new og::RRTstar(simpleSetUp->getSpaceInformation())));
// 	//simpleSetUp->setPlanner(ob::PlannerPtr(new og::PRMstar(simpleSetUp->getSpaceInformation())));
// 	//simpleSetUp->setPlanner(ob::PlannerPtr(new og::LBTRRT(simpleSetUp->getSpaceInformation())));
}

ob::ValidStateSamplerPtr PlannerOMPL::allocOBValidStateSampler(const ob::SpaceInformation *si)
{
	return ob::ValidStateSamplerPtr(new ob::ObstacleBasedValidStateSampler(si));
}

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
	return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

/**
 * @brief Initializer for 3D points (translation only)
 * 
 * @param sampler ...
 * @param limits ...
 * @return void
 */
void PlannerOMPL::initialize(Sampler *sampler)
{
	QList<QPair<float,float> > limits = sampler->getLimits();
	qDebug() << __FUNCTION__ << limits;
	
	assert(limits.size() == 3);
	
	//Create state space as R3
	ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();
	space->addDimension("X", limits[0].first, limits[0].second);
	space->addDimension("Y", limits[1].first, limits[1].second);
	space->addDimension("Z", limits[2].first, limits[2].second);
	
// 	qDebug() << space->getBounds().low[0] << space->getBounds().high[0] ;
// 	qDebug() << space->getBounds().low[1] << space->getBounds().high[1] ;
// 	qDebug() << space->getBounds().low[2] << space->getBounds().high[2] ;
	
	//Setup class
	simpleSetUp.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));

	//set Sampler
	simpleSetUp->getSpaceInformation()->setValidStateSamplerAllocator(allocOBValidStateSampler);
	
	simpleSetUp->setStateValidityChecker(boost::bind(&Sampler::isStateValid, sampler, _1));
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(simpleSetUp->getSpaceInformation()));
	pdef->setOptimizationObjective(getPathLengthObjective(simpleSetUp->getSpaceInformation()));
	
	space->setup();
	
	simpleSetUp->getSpaceInformation()->setStateValidityCheckingResolution(0.1);
	//simpleSetUp->getSpaceInformation()->setStateValidityCheckingResolution(100 / space->getMaximumExtent());
	//simpleSetUp->setPlanner(ob::PlannerPtr(new og::RRTConnect(simpleSetUp->getSpaceInformation())));
	simpleSetUp->setPlanner(ob::PlannerPtr(new og::RRTstar(simpleSetUp->getSpaceInformation())));
	simpleSetUp->getPlanner()->as<og::RRTstar>()->setProblemDefinition(pdef);	
	
	//simpleSetUp->getPlanner()->as<og::RRTConnect>()->setRange(2000);
	simpleSetUp->getPlanner()->as<og::RRTConnect>()->setRange(0.5);	
	
}


//bool PlannerOMPL::computePath(const QVec& target, InnerModel* inner)
bool PlannerOMPL::computePath(const QVec& origin, const QVec &target, int maxTime)
{
	//Planning proper
	if (simpleSetUp == NULL)
		return false;
	
	simpleSetUp->clear();
	
	ob::ScopedState<> start(simpleSetUp->getStateSpace());
	start[0] = origin.x();	start[1] = origin.y(); start[2] = origin.z();
	ob::ScopedState<> goal(simpleSetUp->getStateSpace());
	goal[0] = target.x(); goal[1] = target.y(); goal[2] = target.z();
	simpleSetUp->setStartAndGoalStates(start, goal);
	
	simpleSetUp->getProblemDefinition()->print(std::cout);
	
	currentPath.clear();
	
	ob::PlannerStatus solved = simpleSetUp->solve(maxTime);

	if (solved)
	{
		std::cout << __FILE__ << __FUNCTION__ << "RRT, found solution with " << simpleSetUp->getSolutionPath().getStateCount() << " waypoints" << std::endl;;
	
		//if (simpleSetUp->haveSolutionPath())	
	//	simpleSetUp->simplifySolution();
		og::PathGeometric &p = simpleSetUp->getSolutionPath();
 //		simpleSetUp->getPathSimplifier()->simplify(p,5);//
//		std::cout << __FILE__ << __FUNCTION__ << "Solution after simplify: " << p. getStateCount() << ". Path length: " << p.length() << std::endl;
//		p.print(std::cout);

 		simpleSetUp->getPathSimplifier()->smoothBSpline(p);
		p.interpolate();
		
		for (std::size_t i = 0; i < p.getStateCount(); ++i)
		{
			currentPath.append( QVec::vec3( p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0], 
											p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1], 
											p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[2]));
		}
		return true;
	}
	else
		return false;
}

// void PlannerOMPL::recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out)
// {
// 	if (node->id == robotId)
// 	{
// 		inside = true;
// 	}
// 	
// 	InnerModelMesh *mesh;
// 	InnerModelPlane *plane;
// 	InnerModelTransform *transformation;
// 
// 	if ((transformation = dynamic_cast<InnerModelTransform *>(node)))
// 	{
// 		for (int i=0; i<node->children.size(); i++)
// 		{
// 			recursiveIncludeMeshes(node->children[i], robotId, inside, in, out);
// 		}
// 	}
// 	else if ((mesh = dynamic_cast<InnerModelMesh *>(node)) or (plane = dynamic_cast<InnerModelPlane *>(node)))
// 	{
// 		if (inside)
// 		{
// 			in.push_back(node->id);
// 		}
// 		else
// 		{
// 			out.push_back(node->id);
// 		}
// 	}
// }