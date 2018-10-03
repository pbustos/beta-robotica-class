/*
 * Copyright 2018 <copyright holder> <email>
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
 */

#include "floormeter.h"

FloorMeter::FloorMeter()
{
    initialize();

}

float FloorMeter::addStep(float x, float z, float angle)
{
    auto k = pointToGrid(std::make_pair(x,z));
    bool &ref =  fmap[k].free;
    if(ref == true)
    {
        ref = false; 
        cont++;
    }
    float r = 100. * cont / fmap.size();
    //std::cout << "Covered porcentage: " << r << " %" << std::endl;	
    return  r;
}

FloorMeter::Key FloorMeter::pointToGrid(const std::pair<int,int> &p) const
{
	int kx = (p.first-HMIN)/TILE_SIZE;
	int kz = (p.second-VMIN)/TILE_SIZE;
	
	return Key(HMIN + kx*TILE_SIZE, VMIN + kz*TILE_SIZE);
}

/**
 * @brief Constructs the graph from current sampler
 * 
 */
void FloorMeter::initialize()
{
	
	uint k=0;
 	for( long int i = HMIN ; i < HMAX ; i += TILE_SIZE)
 		for( long int j = VMIN ; j < VMAX ; j += TILE_SIZE)
			fmap.emplace( Key(i,j),Value{k++,true,1}); 
	
	std::cout << "Map size: " << fmap.size() << std::endl;	
}
