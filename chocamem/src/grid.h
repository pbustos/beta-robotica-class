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

#ifndef GRID_H
#define GRID_H

#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <iostream> 
#include <fstream>


template<class T> auto operator<<(std::ostream& os, const T& t) -> decltype(t.save(os), os) 
{ 
    t.save(os); 
    return os; 
};
template<class T> auto operator>>(std::istream& is, T& t) -> decltype(t.read(is), is) 
{ 
    t.read(is); 
    return is; 
};

template <typename T>
class Grid
{     
	public:
		
		struct Dimensions
		{
			int TILE_SIZE = 200;
			int HMIN=-2500, HMAX=2500, VMIN=-2500, VMAX=2500;
		};
		
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
				void save(std::ostream &os) const { os << x << " " << z << " "; };	//method to save the keys
				void read(std::istream &is)  { is >> x  >> z; };	//method to read the keys
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
			
		using FMap = std::unordered_map<Key, T, KeyHasher>;
		
		Grid()																				{};
		
		std::tuple<bool,T&> getCell(long int x, long int z) 											
		{
 			if(x <= dim.HMIN or x >= dim.HMAX or z <= dim.VMIN or z >= dim.VMAX)
 			{ return std::forward_as_tuple(false, T());}
			else
				return std::forward_as_tuple(true, fmap.at(pointToGrid(x,z)));
		}
		
		typename FMap::iterator begin() 							{ return fmap.begin(); };
		typename FMap::iterator end() 								{ return fmap.end();   };
		typename FMap::const_iterator begin() const   { return fmap.begin(); };
		typename FMap::const_iterator end() const 	 	{ return fmap.begin(); };
		size_t size() const 													{ return fmap.size();  };
		
		void initialize(const Dimensions &dim_, T &&initValue)
		{
			dim = dim_;
			fmap.clear();
			for( int i = dim.HMIN ; i < dim.HMAX ; i += dim.TILE_SIZE)
				for( int j = dim.VMIN ; j < dim.VMAX ; j += dim.TILE_SIZE)
					fmap.insert_or_assign( Key(i,j), initValue);
	
			// list of increments to access the neighboors of a given position
			I = dim.TILE_SIZE;
			xincs = {I,I,I,0,-I,-I,-I,0};
			zincs = {I,0,-I,-I,-I,0,I,I};	
		
			std::cout << "Grid::Initialize. Grid initialized to map size: " << fmap.size() << std::endl;	
		}
		
		template<typename Q>
		void insert(const Key &key, const Q &value)
		{
				fmap.insert(std::make_pair(key,value));
		}
		
		void clear()
		{
				fmap.clear();
		}
		
		void saveToFile(const std::string &fich)
		{
			std::ofstream myfile;
			myfile.open (fich);
			for(auto &[k, v] : fmap)
			{
				myfile << k << v << std::endl;
			}
			myfile.close();
			std::cout << fmap.size() << " elements written to " << fich << std::endl;
		}
		
		
 		std::vector<std::pair<Key,T>> neighbours(const Key &k) const
		{
			using Cell = std::pair<Key,T>;
			std::vector<Cell> neigh;
			
			for (auto itx = this->xincs.begin(), itz = this->zincs.begin(); itx != this->xincs.end(); ++itx, ++itz)
			{
				Key lk{k.x + *itx, k.z + *itz}; 
				typename FMap::const_iterator it = fmap.find(lk);
				if( it != fmap.end() )
					neigh.push_back({lk,it->second});
			};
			return neigh;
		}	
     
     
	private:
		FMap fmap;
		Dimensions dim;
		
		// list of increments to access the neighboors of a given position
		int I;
		std::vector<int> xincs;
		std::vector<int> zincs;	
		
		auto pointToGrid(long int x, long int z) const -> decltype(Key())
		{
			int kx = (x-dim.HMIN)/dim.TILE_SIZE;
			int kz = (z-dim.VMIN)/dim.TILE_SIZE;
			return Key(dim.HMIN + kx*dim.TILE_SIZE, dim.VMIN + kz*dim.TILE_SIZE);
		};
};

#endif // FLOORMETER_H
