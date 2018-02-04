/**
 * For testing virtual layers built up by 
 * redundant trees.
 *
 * Written by Mate Nagy
 * License: GNU General Public License Version 3
 *
 */

#ifndef RTBASEDMETHOD_HH_
#define RTBASEDMETHOD_HH_

#include "Network.hh"
#include "rtmanager.h"
#include "Log.h"

class RTBasedMethod {
public:
  Network *nwP;
  Graph *g;
  NodeInfoMap *nodeInfo;
  LinkInfoMap *linkInfo;
  //RTManager *rtManager;
  RedundantTree<Graph> *redundantTree;

  /* To speed up iterations on layers */
  //Physical layer
  SubDigraph<Graph> *phyLayer;
  Graph::NodeMap<bool> *phyNMap;
  Graph::ArcMap<bool> *phyAMap;

  //RemoteRed virtual layer
  SubDigraph<Graph> *rrLayer;
  Graph::NodeMap<bool> *rrNMap;
  Graph::ArcMap<bool> *rrAMap;

  //RemoteBlue virtual layer
  SubDigraph<Graph> *rbLayer;
  Graph::NodeMap<bool> *rbNMap;
  Graph::ArcMap<bool> *rbAMap;

  //VeryRemoteRed virtual layer
  SubDigraph<Graph> *vrrLayer;
  Graph::NodeMap<bool> *vrrNMap;
  Graph::ArcMap<bool> *vrrAMap;

  //VeryRemoteBlue virtual layer
  SubDigraph<Graph> *vrbLayer;
  Graph::NodeMap<bool> *vrbNMap;
  Graph::ArcMap<bool> *vrbAMap;

public:
  RTBasedMethod(Network*);
  void buildRedundantTrees(const Node&);
  bool checkTreeRedundancy(const Node&);
  Node isVirtualNodeExistInLayer(const Node&, const LayerID&);
  void addVirtualEdge(const Node&, const Node&, const LayerID&, const Cost&, const Arc&);
  Node addVirtualNode(const Node&, const LayerID&);
  void createVirtualLayer(const Node&, const LayerID&);
  void connectLayersAtRoot(const Node&);
  bool testIfOriginalPathsAreSame();
  Node srlg_IsLayerValidForLFA(const Node&, const Node&, NodeVector, const LayerID&);
  void fillLocalSRLGs();
  void buildVirtualLayers(const Node&);
  void printSpecificLayer(const LayerID&);
  void printSRGSets();
  virtual ~RTBasedMethod();
};


#endif /* RTBASEDMETHOD_HH_ */
