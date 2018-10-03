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
	

}

void FloorMeter::addStep(float x, float z, float angle)
{
	
}


FloorMeter::Key FloorMeter::pointToGrid(int hmin, int vmin, const std::pair<int,int> &p) const
{
	int kx = (p.first-hmin)/TILE_SIZE;
	int kz = (p.second-vmin)/TILE_SIZE;
	
	return Key(hmin + kx*TILE_SIZE, vmin + kz*TILE_SIZE);
}

/**
 * @brief Constructs the graph from current sampler
 * 
 */
void FloorMeter::initialize(int hmin, int hmax, int vmin, int vmax, uint tile_size)
{
	
	uint k=0;
 	for( long int i = hmin ; i < hmax ; i += tile_size)
 		for( long int j = vmin ; j < vmax ; j += tile_size)
			fmap.emplace( Key(i,j),Value{k++,false,1}); 
	
	//qDebug() << __FILE__ << __FUNCTION__ << "Map size: " << fmap.size();	
}
