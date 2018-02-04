/*
 * Written by Mate Nagy
 * License: GNU General Public License Version 3
 *
 */

#ifndef GREEDYHEURISTIC_HH_
#define GREEDYHEURISTIC_HH_

#include <algorithm>
#include "Network.hh"
#include "Database.hh"
#include "PacketTracer.hh"
#include "Log.h"

typedef enum {CONN_L = 0, SPS = 1} NodeSelStrategy;

struct SDPair {
  Node source;
  Node destination;
};

//setting Greedy parameters
struct GreedyParamSet {
  bool skipEdgesOn;                       //simplify network
  bool leak_cascCondOn; //trap leak fill #2
  bool skipUpstrLfa; //trap leak fill #3
  bool smartTrapCondOn;   //strengthen trap condition => more escapes
  NodeSelStrategy  nSelStrategy;
  //  CLA: each step it only sees one # nodeset up to 4 (Classic Infocom)
  //  SPT: try to virtualize nodes along Shortest Paths
  //  FIX: algorithm sees all 1-4 node sets in each step
};

typedef vector< SDPair > UnprotectedSDSet;
typedef UnprotectedSDSet::iterator UnprotSDIt;
typedef map<Node, int> EscapeMap;
typedef map<SDPair, NodeSet>::iterator ET_Iterator;

class PacketTracer;

class GreedyHeuristic{
public:
  //pointers to other objects
  Network *nwP;
  Graph *g;
  PacketTracer *packetTracer;
  LinkInfoMap *linkInfo;
  NodeInfoMap *nodeInfo;

  //we have 4 sets
  UnprotectedSDSet unprotSDs;
  UnprotectedSDSet L;
  map<SDPair, NodeSet> E;
  UnprotectedSDSet Q;
  map<SDPair, NodeSet> T;

  // Variables for GREEDY (used in each step)
  // [bestNode => #(s,d) protects]
  // <toVirtualize, bestEscapeNodeVector>
  pair<NodeSet, NodeVector> bestSelection;
  int sumOfBestProtection;
  double bestImproveRatio;

  //parameters for handling
  //trap leakage solution
  //different ways. Valid values: 0-7
  GreedyParamSet gParamS;

  //for xml printout
  XmlStepSet xmlStepSet;

  //storing SPT set of nodes
  vector<NodeSet> incrSetOfVirtCandidates;

  //for sophisticated debugging
  int toDebug;

  //Count how many times PT was called
  int cntPacketTracerCalls_Escape;
  int cntPacketTracerCalls_Trap;

public:
  GreedyHeuristic(Network*, PacketTracer*);
  void buildUnprotectedSDSet();
  void printUnprotectedSDSet();
  void printOutUnprotectedSDSet();
  bool isLocalSRLGDisjoint(const Node&, const Arc&, const Arc&);
  void makeLSet(const Node&);
  bool isInLSet(const SDPair&);
  void makeLSet(const NodeSet&);
  void makeESet(const NodeSet&);
  void makeQSet(const NodeSet&);
  void makeTSet(const NodeSet&);
  void printLSet();
  void printESet();
  void printQSet();
  void printTSet();
  void cleanSets();
  EscapeMap getEscapeMap();
  inline int getPTCounter_Escape(){ return cntPacketTracerCalls_Escape; }
  inline int getPTCounter_Trap(){ return cntPacketTracerCalls_Trap; }
  //void updateBestSelection(const Node&);
  void updateBestSelectionForOneEscape(const NodeSet&, EscapeMap&);
public:
  pair<NodeSet, NodeVector> selectNodeGreedilyWithOneEscape();
  pair<Node, NodeVector> selectNodeGreedilyWith2Escapes();
  void printLocalSRLGs();
  void fillNodeInfoLocalSrgTables();
  double rollbackVirtualNode(const NodeSet&);
  NodeSet realizeVirtualNode(const NodeSet&, const Node&);
  void setGreedyParamSet(const GreedyParamSet&);
  void greedyHeuristicAlgorithm(const double&, const int&);
  ~GreedyHeuristic();
};

bool operator<(SDPair, SDPair);

bool operator==(SDPair, SDPair);

#endif /* GREEDYHEURISTIC_HH_ */
