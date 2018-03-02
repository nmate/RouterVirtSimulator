/*
 * Written by Mate Nagy
 * License: GNU General Public License Version 3
 *
 */

#ifndef NETWORK_HH_
#define NETWORK_HH_

#include <lemon/list_graph.h>
#include <lemon/lgf_reader.h>
#include <lemon/lgf_writer.h>
#include <lemon/time_measure.h>
#include <math.h>
#include <set>
#include <map>
#include <iomanip> //for printing results

using namespace std;
using namespace lemon;

typedef ListDigraph Graph;
typedef Graph::Node Node;
typedef Graph::Arc Arc;
typedef double Cost;

typedef Graph::ArcMap<Cost> CostMap;
typedef Graph::NodeIt NodeIt;
typedef Graph::ArcIt ArcIt;

typedef set<Node> NodeSet;
typedef set<Arc> LinkSet;
typedef set<LinkSet> SRGSet;
typedef vector<Node> NodeVector;

typedef enum {LP, NP} LFAType;

// Virtual layer identifiers:
// RR  = Remote RED      ; RB  = Remote BLUE
// VRR = Very Remote RED ; VRB = Very Remote BLUE

typedef enum {
  NON,
  PHYS = 0,
  RR,
  RB,
  VRR,
  VRB
} LayerID;

struct NodeInfo {
  bool isVirtual;
  LayerID  layerID;
  Node physicalNode;
  SRGSet localSRGs;
};

struct LinkInfo {
  bool isVirtual;
  LayerID  layerID;
  Arc physicalLink;
};

struct GreedyXmlSet{
  int sumOfVirtualNsAtStep;
  double currLFACoverage;
  double currAVPL; //Average Path Length
  TimeStamp timeStamp;
};

typedef Graph::ArcMap<LinkInfo> LinkInfoMap;
typedef Graph::NodeMap<NodeInfo> NodeInfoMap;
typedef vector< GreedyXmlSet > XmlStepSet;

#include "Database.hh"
#include "PacketTracer.hh"
#include "GreedyHeuristic.hh"

class Database;
class PacketTracer;
class GreedyHeuristic;

class Network {

public:
  string graphName;
  Graph g;
  int numOfPhysicalNodes;
  int numOfPhysicalArcs;
  Cost lspPhy; //longest shortest path of the physical graph
  CostMap *cost;
  Database *db;
  PacketTracer *packetTracer;
  LinkInfoMap *linkInfo;
  NodeInfoMap *nodeInfo;
  GreedyHeuristic *greedyH;
  Cost LSP;
  double woFaultAPL; //without fault the Average Path Length
  double wFaultAPL;  //with fault the Average Path Length
  vector<NodeSet> virtCandidates;

public:
  Network(){};
  Network(const string &);
  void setupUnprotectedSDs();
  void calculateNodeSets(const int&);
  bool haveVirtualElements();
  bool readGraph();
  bool readGraph_virtual();
  void writeGraph() const;
  void printGraph();
  bool equal(Cost, Cost) const;
  Node nh(Node, Node) const;
  bool neighbor(Node, Node) const;
  double getLegacyLFACoverage(const LFAType &);
  Cost distOpt(Graph::Node, Graph::Node) const;
  Cost distUni(Graph::Node, Graph::Node) const;
  Cost longestShortestPath();
  bool isLFACondTrue(const Node&, const Node&, const Node&) const;
  Node isLegacyLFAProtected_LP(const Node&, const Node&, const LinkSet& = LinkSet()) const;
  bool isLegacyLFAProtected_NP(const Node&, const Node&);
  void countPhysicalElements();
  double newLfaCoverageMetric_LP();
  double legacyLFACoverage(const LFAType &);
  bool isLocalSRLGDisjoint(const Node&, const Arc&, const Arc&);
  int addVirtualRouter(const Node&);
  bool checkLoopAndAVPL(int &, const bool& toPrint=false);
  void printVirtCandidateNodeSets();
  void buildUniformSetOfNodes(const int&);
  virtual ~Network();

private:
  void buildPathSetOfNodes(const int&);
  bool isNeighOfSet(const NodeSet&, const Node&);
  bool isSetInVectorOfSet(const NodeSet&, const vector<NodeSet>&);
};

#endif /* NETWORK_HH_ */
