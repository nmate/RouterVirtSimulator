/*
 * Written by Mate Nagy
 * License: GNU General Public License Version 3
 *
 */

#ifndef DATABASE_HH_
#define DATABASE_HH_

//using namespace lemon;

#include <lemon/dijkstra.h>
#include <vector>

#include "Network.hh"
#include "Log.h"

typedef Graph::NodeMap< Graph::NodeMap<Cost>* > DistMap;
typedef Graph::NodeMap< Graph::NodeMap<Node>* > NHMap;
typedef Dijkstra<Graph, CostMap> Djs;

class Network;

class Database {

public:
	const Network *nwP;
	DistMap *distmap;
	NHMap *nhmap;

	//for unicost version of graph
	DistMap *uniDistMap;
	NHMap *uniNHMap;

public:
	Database(Network*);
	void initializeDB();
	void buildUniCostDB();
	~Database();
};



#endif /* DATABASE_HH_ */
