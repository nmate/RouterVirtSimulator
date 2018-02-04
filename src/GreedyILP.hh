/*
 * Written by Mate Nagy
 * License: GNU General Public License Version 3
 *
 */

#ifndef GREEDYILP_HH_
#define GREEDYILP_HH_

#ifdef GUROBI
#include <lemon/gurobi.h>
#else
#include <lemon/glpk.h>
#endif

#include <lemon/lp_base.h>
#include <lemon/lp.h>
#include "Network.hh"
#include "Database.hh"
#include "Log.h"

typedef Graph::NodeMap<int> INodeMap;
typedef Graph::NodeMap<double> DNodeMap;

#ifdef GUROBI
typedef GurobiMip Mip;
#else
typedef GlpkMip Mip;
#endif

struct SDGTriplet{
  Node s;
  Node d;
  Node g;
};

struct VsVg{
  Node v_s;
  Node v_g;
};

class GreedyILP{
private:
  Network *nwP;
  Graph *g;
  LinkInfoMap *linkInfo;
  NodeInfoMap *nodeInfo;
  GreedyHeuristic *greedyH;
public:
  XmlStepSet xmlStepSet;

public:
  GreedyILP(Network*);
  void callILPGreedily(const double&, const int&);
  inline const string name(const Node & n) const {
    ostringstream convert;
    convert << nwP->g.id(n);
    return convert.str();
  }
  VsVg getVsVg(const NodeSet&, const SDGTriplet&, Cost&);
  NodeSet selectAndAddVirtualNode();
  NodeSet realizeVirtualNode(const NodeSet&, const INodeMap&, const DNodeMap&);
};

bool operator<(SDGTriplet, SDGTriplet);

#endif /* GREEDYILP_HH_ */
