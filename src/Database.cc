/*
 * Written by Mate Nagy
 * License: GNU General Public License Version 3
 *
 */

#include "Database.hh"

Database::Database(Network *_nwP):nwP(_nwP){
	distmap = new DistMap(nwP->g);
	nhmap = new NHMap(nwP->g);

	// call with original costs
	initializeDB();

	uniDistMap = new DistMap(nwP->g);
	uniNHMap = new NHMap(nwP->g);

	buildUniCostDB();

	//if it is called here, then segfault comes...
	//buildUnprotectedSDSet();
}
void Database::initializeDB(){
	debug(LOG)<<"Building next-hop and distance maps... ";
	Djs dijkstra(nwP->g, *(nwP->cost));
	for (Graph::NodeIt s(nwP->g); s != INVALID; ++s){
		//clean old datas
		delete (*distmap)[s];
		delete (*nhmap)[s];

		//calculate new ones
	  dijkstra.run(s);
	  (*distmap)[s] = new Graph::NodeMap<Cost>(nwP->g);
	  (*nhmap)[s]   = new Graph::NodeMap<Node>(nwP->g);

	  for (Graph::NodeIt d(nwP->g); d != INVALID; ++d){
	    // fill in distance
	    (*(*distmap)[s])[d] = dijkstra.reached(d) ? dijkstra.dist(d) : -1;

	    // fill in next-hop
	    if(s != d && dijkstra.reached(d)){
	      Node current = d;
	      while( dijkstra.predNode(current) != s ){
	      	current = dijkstra.predNode(current);
	      }
	        (*(*nhmap)[s])[d] = current;
	    }
	    else
	      //cout<<"invalid"<<endl;
	      (*(*nhmap)[s])[d] = INVALID;
	  }//endfor d

	}//endfor s

	debug(LOG)<<"OK"<<endl;
}

void Database::buildUniCostDB(){
  CostMap uniCostM(nwP->g, 1);

  debug(LOG)<<"Building unicost distance and next-hop map..."<<endl;
  Djs dijkstra(nwP->g, uniCostM);
  for (Graph::NodeIt s(nwP->g); s != INVALID; ++s){
    //clean old datas
    delete (*uniDistMap)[s];
    delete (*uniNHMap)[s];

    //calculate new ones
    dijkstra.run(s);
    (*uniDistMap)[s] = new Graph::NodeMap<Cost>(nwP->g);
    (*uniNHMap)[s]   = new Graph::NodeMap<Node>(nwP->g);

    for (Graph::NodeIt d(nwP->g); d != INVALID; ++d){
      // fill in distance
      (*(*uniDistMap)[s])[d] = dijkstra.reached(d) ? dijkstra.dist(d) : -1;

      // fill in next-hop
      if(s != d && dijkstra.reached(d)){
        Node current = d;
        while( dijkstra.predNode(current) != s ){
          current = dijkstra.predNode(current);
        }
          (*(*uniNHMap)[s])[d] = current;
      }
      else
        //cout<<"invalid"<<endl;
        (*(*uniNHMap)[s])[d] = INVALID;
    }//endfor d

  }//endfor s

  debug(LOG)<<"OK"<<endl;
}

Database::~Database(){
	for (Graph::NodeIt s((*nwP).g); s != INVALID; ++s){
		  delete (*distmap)[s];
		  delete (*uniDistMap)[s];
			delete (*nhmap)[s];
			delete (*uniNHMap)[s];
	}
	delete distmap;
	delete nhmap;
	delete uniDistMap;
	delete uniNHMap;
}
