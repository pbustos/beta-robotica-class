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

#ifndef PLANNEROMPL_H
#define PLANNEROMPL_H

#include <QObject>
#include <qmat/QMatAll>
#include <innermodel/innermodel.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/config.h>
#include <iostream>
#include "sampler.h"
#include <cassert>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class PlannerOMPL : public QObject
{
    Q_OBJECT

	public:
		PlannerOMPL(){};
		PlannerOMPL(const InnerModel &innerModel_, QObject *parent=0);
		//bool computePath(const QVec &target, InnerModel *inner);
		bool computePath(const QVec& origin, const QVec& target, int maxTime);
		//void initialize(Sampler *sampler);
		void initialize(Sampler *sampler);
		QList<QVec> getPath() { return currentPath; }
		void setSpaceLimits(float xmin, float xmax, float zmin, float zmax)		{xMin = xmin; xMax = xmax, zMin = zmin; zMax = zMax;};

	private:
		//bool isStateValid(const ob::State *state) const;
		//void recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out);
		
		QList<QVec> currentPath;   			//Results will be saved here
		og::SimpleSetupPtr simpleSetUp;
		std::vector<QString> robotNodes;
		std::vector<QString> restNodes;
		float xMin, xMax, zMin, zMax; 		//Limits of environmnent
		static ob::ValidStateSamplerPtr allocOBValidStateSampler(const ob::SpaceInformation *si);
		
};

#endif // PLANNEROMPL_H
