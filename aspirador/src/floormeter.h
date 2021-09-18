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

#ifndef FLOORMETER_H
#define FLOORMETER_H

#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <iostream> 

//#define TILE_SIZE 300 // grid discrtization step
 
template<class T> auto operator<<(std::ostream& os, const T& t) -> decltype(t.print(os), os) 
{ 
    t.print(os); 
    return os; 
};


class FloorMeter
{
	int TILE_SIZE = 200;
    int HMIN=-2500, HMAX=2500, VMIN=-2500, VMAX=2500;
    uint cont = 0;
         
	struct Key
		{
			long int x;
			long int z;
		
			public:
				Key(): x(0), z(0) {};
				Key(long int &&x, long int &&z): x(std::move(x)), z(std::move(z)){};
				Key(long int &x, long int &z): x(x), z(z){};
				Key(const long int &x, const long int &z): x(x), z(z){};
				bool operator==(const Key &other) const
					{ return x == other.x && z == other.z; };
				void print(std::ostream &os) const 	{ os << " x:" << x << " z:" << z; };
		};
	
	struct Value
	{
		uint id;
		bool free;
		float cost;
	};

	struct KeyHasher
		{
			std::size_t operator()(const Key& k) const
			{
				using boost::hash_value;
				using boost::hash_combine;

				// Start with a hash value of 0    .
				std::size_t seed = 0;

				// Modify 'seed' by XORing and bit-shifting in one member of 'Key' after the other:
				hash_combine(seed,hash_value(k.x));
				hash_combine(seed,hash_value(k.z));
				return seed;
			};
		};	
			
	public:
		FloorMeter();
		float addStep(float x, float z, float angle);
        void reset() { cont = 0;} ;
        void set_dimensions(int hmin, int hmax, int vmin, int vmax, int tile_size)
        {
            HMIN = hmin; HMAX = hmax; VMIN = vmin; VMAX = vmax; TILE_SIZE = tile_size;
        }

	private:
		typedef	std::unordered_map<Key, Value, KeyHasher> FMap;	
		FMap fmap;
		Key pointToGrid(const std::pair<int,int> &p) const;
		void initialize();
		
};

#endif // FLOORMETER_H
