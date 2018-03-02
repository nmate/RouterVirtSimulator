/*
 * Written by Mate Nagy
 * License: GNU General Public License Version 3
 *
 */

#include "RTBasedMethod.hh"

RTBasedMethod::RTBasedMethod(Network *_nwP){
  nwP = _nwP;
  g = &(nwP->g);

  nodeInfo = nwP->nodeInfo;
  linkInfo = nwP->linkInfo;

  //original graph
  phyNMap = new Graph::NodeMap<bool> (*g, true);
  phyAMap = new Graph::ArcMap<bool> (*g, true);
  phyLayer = new SubDigraph<Graph>(*g, *phyNMap, *phyAMap);

  rrLayer = NULL;
  rrNMap = new Graph::NodeMap<bool> (*g, false);
  rrAMap = new Graph::ArcMap<bool> (*g, false);

  rbLayer = NULL;
  rbNMap = new Graph::NodeMap<bool> (*g, false);
  rbAMap = new Graph::ArcMap<bool> (*g, false);

  vrrLayer = NULL;
  vrrNMap = new Graph::NodeMap<bool> (*g, false);
  vrrAMap = new Graph::ArcMap<bool> (*g, false);

  vrbLayer = NULL;
  vrbNMap = new Graph::NodeMap<bool> (*g, false);
  vrbAMap = new Graph::ArcMap<bool> (*g, false);
}

void RTBasedMethod::buildRedundantTrees(const Node &root){
  redundantTree = new RedundantTree<Graph>(g);
  redundantTree->run(root);
}

bool RTBasedMethod::checkTreeRedundancy(const Node &root){
  Node testDest = g->nodeFromId(nwP->numOfPhysicalNodes-2);
  return ( redundantTree->testTrees(root, testDest) );
}

/*
 * Check if a virtual node exists in the specified
 * layer and has the given physical instance
 * Return found virtual node or INVALID
 */
Node RTBasedMethod::isVirtualNodeExistInLayer(const Node &v, const LayerID &lID){
  SubDigraph<Graph> *subGraph;

  switch (lID){
  case RR:
    subGraph = rrLayer;
    break;
  case RB:
    subGraph = rbLayer;
    break;
  case VRR:
    subGraph = vrrLayer;
    break;
  case VRB:
    subGraph = vrbLayer;
    break;
  default:
    debug(CRITICAL)<<"Invalid layer in isVirtualNode.. check!"<<endl;
  }

  for (SubDigraph<Graph>::NodeIt n(*subGraph); n != INVALID; ++n){
    if ( (*nodeInfo)[n].isVirtual  &&
         (*nodeInfo)[n].layerID == lID &&
         (*nodeInfo)[n].physicalNode == v ){
      return n;
    }
  }

  return INVALID;
}

/*
 * Based on the given RT arc it creates
 * bidirected arcs between the 2 virtual
 * nodes
 */
void RTBasedMethod::addVirtualEdge(const Node &virtSrc, const Node &virtDst,
                                   const LayerID &lId, const Cost &cost, const Arc &phyArc){
  //add Arc
  Arc virtE = g->addArc(virtSrc, virtDst);
  (*(nwP->cost))[virtE] = cost;
  (*linkInfo)[virtE].isVirtual = true;
  (*linkInfo)[virtE].layerID = lId;
  (*linkInfo)[virtE].physicalLink = phyArc;

  //add reverseArc
  Arc virtRevE = g->addArc(virtDst, virtSrc);
  (*(nwP->cost))[virtRevE] = cost;
  (*linkInfo)[virtRevE].isVirtual = true;
  (*linkInfo)[virtRevE].layerID = lId;
  Arc phyRevArc = findArc(*g, g->target(phyArc), g->source(phyArc));
  (*linkInfo)[virtRevE].physicalLink = phyRevArc;

}

Node RTBasedMethod::addVirtualNode(const Node &phyN, const LayerID& lId){
  Node virtN = g->addNode();
  (*nodeInfo)[virtN].isVirtual = true;
  (*nodeInfo)[virtN].physicalNode = phyN;
  (*nodeInfo)[virtN].layerID = lId;

  //mark it in the proper subgraph
  switch (lId){
  case RR:
    (*rrNMap)[virtN] = true;
    break;
  case RB:
    (*rbNMap)[virtN] = true;
    break;
  case VRR:
    (*vrrNMap)[virtN] = true;
    break;
  case VRB:
    (*vrbNMap)[virtN] = true;
    break;
  default:
    debug(CRITICAL)<<"Error: invalid layer ID is given!"<<endl;
  }

  /*
   * anchor virtual node to the physical one
   * (K: remote layer, 2K: very remote layer)
   */
  Cost crossCost;
  if ( lId == RR || lId == RB )
    crossCost = nwP->lspPhy; //K
  else if ( lId == VRR || lId == VRB )
    crossCost = 2*(nwP->lspPhy); //2K
  else
    debug(CRITICAL)<<"Error: invalid layer ID is given!"<<endl;

  Arc crossArc = g->addArc(phyN, virtN);
  (*(nwP->cost))[crossArc] = crossCost;
  (*linkInfo)[crossArc].isVirtual = true;
  (*linkInfo)[crossArc].layerID = lId;
  (*linkInfo)[crossArc].physicalLink = INVALID;

  Arc revCrossArc = g->addArc(virtN, phyN);
  (*(nwP->cost))[revCrossArc] = crossCost;
  (*linkInfo)[revCrossArc].isVirtual = true;
  (*linkInfo)[revCrossArc].layerID = lId;
  (*linkInfo)[revCrossArc].physicalLink = INVALID;

  return virtN;
}

void RTBasedMethod::createVirtualLayer(const Node &root, const LayerID &lId){
  Cost innerCost = (double)1/(2*(nwP->numOfPhysicalNodes)); //within virtual layer
  Graph::ArcMap<bool> *tree; //points to proper tree

  switch (lId){
  case RR:
    rrLayer = new SubDigraph<Graph>(*g, *rrNMap, *rrAMap);
    tree = redundantTree->getREDTree();
    break;
  case VRR:
    vrrLayer = new SubDigraph<Graph> (*g, *vrrNMap, *vrrAMap);
    tree = redundantTree->getREDTree();
    break;
  case RB:
    rbLayer = new SubDigraph<Graph> (*g, *rbNMap, *rbAMap);
    tree = redundantTree->getBLUETree();
    break;
  case VRB:
    vrbLayer = new SubDigraph<Graph> (*g, *vrbNMap, *vrbAMap);
    tree = redundantTree->getBLUETree();
    break;
  default:
    debug(CRITICAL)<<"Error: trying to virtualize "
      "with wrong or invalid layer ID!"<<endl;
  }

  for (ArcIt e(*g);  e != INVALID; ++e) {
    if ( (*tree)[e] ){
      Node eSrc = g->source(e);
      Node eDst = g->target(e);
      Node virtSrc = isVirtualNodeExistInLayer(eSrc, lId);
      Node virtDst = isVirtualNodeExistInLayer(eDst, lId);

      //case #1: have virtSrc and have virtDst
      if ( virtSrc != INVALID &&  virtDst != INVALID){
        addVirtualEdge(virtSrc, virtDst, lId, innerCost, e);
      }

      //case #2: have virtSrc but not have virtDst
      if ( virtSrc != INVALID && virtDst == INVALID ){
        virtDst = addVirtualNode(eDst, lId);
        addVirtualEdge(virtSrc, virtDst, lId, innerCost, e);
      }

      //case #3: not have virtSrc but have virtDst
      if ( virtSrc == INVALID && virtDst != INVALID ){
        virtSrc = addVirtualNode(eSrc, lId);
        addVirtualEdge(virtSrc, virtDst, lId, innerCost, e);
      }

      //case #4: not have virtSrc and not have virtDst
      if ( virtSrc == INVALID && virtDst == INVALID ){
        virtSrc = addVirtualNode(eSrc, lId);
        virtDst = addVirtualNode(eDst, lId);
        addVirtualEdge(virtSrc, virtDst, lId, innerCost, e);
      }

    }//endif arcFilterRED
  }//endfor arcIt e
}

void RTBasedMethod::connectLayersAtRoot(const Node &root){
  //within virtual layer
  Cost virtCrossCost = (double)1/(2*(nwP->numOfPhysicalNodes));

  //suppose that we have already created
  //all 4 layers...
  Node virtRR  = isVirtualNodeExistInLayer(root, RR);
  Node virtVRB = isVirtualNodeExistInLayer(root, VRB);
  addVirtualEdge(virtRR, virtVRB, NON, virtCrossCost, INVALID);

  Node virtRB  = isVirtualNodeExistInLayer(root, RB);
  Node virtVRR = isVirtualNodeExistInLayer(root, VRR);
  addVirtualEdge(virtRB, virtVRR, NON, virtCrossCost, INVALID);
}

bool RTBasedMethod::testIfOriginalPathsAreSame(){
  LinkSet empty;
  for (Graph::NodeIt s(nwP->g); s != INVALID; ++s){
    for (Graph::NodeIt d(nwP->g); d != INVALID; ++d){
      if ( !(*nodeInfo)[s].isVirtual && !(*nodeInfo)[d].isVirtual){
        nwP->packetTracer->haveValidPath(s, d, empty);
        if ( nwP->packetTracer->nHopIsPhysical == false )
          return false;
      }
    }
  }
  return true;
}

/*******************************************
 * For a given physical node and virtual layer,
 * it returns the virtual instance that is NOT
 * a fake LFA to any destinations, or INVALID
 * if not exists.
 *******************************************/
Node RTBasedMethod::srlg_IsLayerValidForLFA(const Node &s, const Node &nh, 
                                            NodeVector dests, const LayerID &lId){
  NodeVector::iterator nIt;
  Node LFA = INVALID;

  Node virt = isVirtualNodeExistInLayer(s, lId);
  LFA = virt;
  for (nIt = dests.begin(); nIt != dests.end(); ++nIt){
    Node nHOfVirt = nwP->nh(virt, *nIt);

    debug(DEBUG1)<<"s: "<<nwP->g.id(s)<<" d: "<<nwP->g.id(*nIt)<<" pNh: "
                 <<nwP->g.id(nh)<<" vNh: "<<nwP->g.id(nHOfVirt)<<"("
                 <<nwP->g.id((*nodeInfo)[nHOfVirt].physicalNode)<<")"<<" layer: "<<lId<<endl;

    if ( (*nodeInfo)[nHOfVirt].physicalNode == nh ){
      LFA = INVALID;
    }
  }
  return LFA;
}

/*********************************************
 * An LFA in the virtual layer is fake if its
 * next-hop is not disjunct with its physical
 * parent's next-hop
 * -if a virtual link fails, it does not affect
 *  any other physical or virtual links
 * -if a physical link fails it matters
 *********************************************/
void RTBasedMethod::fillLocalSRLGs(){
  debug(LOG)<<"============================================"<<endl;
  debug(LOG)<<"           filling SRLG infos "<<endl;
  Node validLFA = INVALID;
  for (SubDigraph<Graph>::NodeIt s(*phyLayer); s != INVALID; ++s){
    map<Node, NodeVector> nhToDestMap;
    for (SubDigraph<Graph>::NodeIt d(*phyLayer); d != INVALID; ++d){
      if ( s != d ){
        Node nhop = nwP->nh(s,d);
        nhToDestMap[nhop].push_back(d);
      }
    }

    NodeVector::iterator nIt;
    map<Node, NodeVector>::iterator it;
    /* Debugging nh->destination data structure*/
    debug(LOG)<<"============================================"<<endl;
    for (it=nhToDestMap.begin(); it!=nhToDestMap.end();++it){
      debug(LOG)<<"[s: "<<nwP->g.id(s)<<" nh: "<<nwP->g.id(it->first)<<" ] => ( ";
      for (nIt=it->second.begin(); nIt!=it->second.end(); ++nIt){
        debug(LOG)<<nwP->g.id(*nIt)<<" ";
      }
      debug(LOG)<<")"<<endl;
    }

    for (it=nhToDestMap.begin(); it!=nhToDestMap.end();++it){
      // check all the layers
      for (int lId = RR; lId <= VRB; ++lId){
        validLFA = srlg_IsLayerValidForLFA(s, it->first, it->second, (LayerID)lId);
        if ( validLFA != INVALID ){
          break;
        }
      }

      if ( validLFA == INVALID ){
        debug(CRITICAL)<<"ERROR: Not exists, only fake LFAs!"<<endl;
      }
      else {
        debug(LOG)<<"Valid LFA found in layer "<<(*nodeInfo)[validLFA].layerID<<" for ("
                  <<nwP->g.id(s)<<", "<<nwP->g.id(it->first)<<")"<<endl;

        // !!! ready for filling SRGs !!!
        LinkSet linkSet;

        Arc nhArc = findArc(*g, s, it->first);
        linkSet.insert(nhArc);
        Arc revNhArc = findArc(*g, it->first, s);
        linkSet.insert(revNhArc);

        for (int lId = RR; lId <= VRB; lId++){
          Node virtNeigh = isVirtualNodeExistInLayer(s, (LayerID)lId);
          if ( virtNeigh != validLFA ){
            linkSet.insert(findArc(*g, s, virtNeigh));
            linkSet.insert(findArc(*g, virtNeigh, s));
          }
        }
        (*nodeInfo)[s].localSRGs.insert(linkSet);

      }//end else (have valid LFA)
    }// endfor next hops
  }//endfor s
}

/*
 * Builds up all 4 virtual layers
 */
void RTBasedMethod::buildVirtualLayers(const Node &root){
  createVirtualLayer(root, RR);
  createVirtualLayer(root, RB);
  createVirtualLayer(root, VRR);
  createVirtualLayer(root, VRB);
  connectLayersAtRoot(root);

  //recalculate shortest paths
  nwP->db->initializeDB();

  debug(INFO)<<"Layers are done, is SPT unchanged... ";
  if ( testIfOriginalPathsAreSame() )
    debug(INFO)<<"OK"<<endl;
  else
    debug(INFO)<<"NOK => ERROR"<<endl;

  //fill local SRLG table
  fillLocalSRLGs();
}

void RTBasedMethod::printSpecificLayer(const LayerID &lId){
  SubDigraph<Graph> *subGraph;

  cout<<"Printing ";
  switch (lId){
  case PHYS:
    subGraph = phyLayer;
    cout<<"PHYS";
    break;
  case RR:
    subGraph = rrLayer;
    cout<<"RR";
    break;
  case RB:
    subGraph = rbLayer;
    cout<<"RB";
    break;
  case VRR:
    subGraph = vrrLayer;
    cout<<"VRR";
    break;
  case VRB:
    subGraph = vrbLayer;
    cout<<"VRB";
    break;
  default:
    debug(CRITICAL)<<"Error: invalid layer ID for printing!"<<endl;
  }
  cout<<" layer: "<<endl;
  for (SubDigraph<Graph>::NodeIt n(*subGraph); n != INVALID; ++n ){
    cout<<"n: "<<nwP->g.id(n)<<" subId: "<<subGraph->id(n)<<endl;
  }
}

void RTBasedMethod::printSRGSets(){
  cout<<"============================================"<<endl;
  cout<<"            Printing SRLG infos "<<endl;
  cout<<"============================================"<<endl;
  for (SubDigraph<Graph>::NodeIt n(*phyLayer); n != INVALID; ++n ){
    SRGSet::iterator srgIt = (*nodeInfo)[n].localSRGs.begin();
    while (srgIt != (*nodeInfo)[n].localSRGs.end()){
      LinkSet::iterator lsIt;
      cout<<"n: "<<nwP->g.id(n)<<" -> ( ";
      for (lsIt = srgIt->begin(); lsIt != srgIt->end(); ++lsIt){
        cout<<g->id(*lsIt)<<" ";
      }
      cout<<")"<<endl;
      srgIt++;
    }
  }
}

RTBasedMethod::~RTBasedMethod(){
  //destroy RTManager object
  delete redundantTree;

  delete rrLayer;
  delete rrNMap;
  delete rrAMap;

  delete rbLayer;
  delete rbNMap;
  delete rbAMap;

  delete vrrLayer;
  delete vrrNMap;
  delete vrrAMap;

  delete vrbLayer;
  delete vrbNMap;
  delete vrbAMap;

  delete phyLayer;
  delete phyNMap;
  delete phyAMap;
}
