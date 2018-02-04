/*
 * Written by Mate Nagy
 * License: GNU General Public License Version 3
 *
 */

#include "GreedyHeuristic.hh"

GreedyHeuristic::GreedyHeuristic(Network *_nwP, PacketTracer *_packetTracer){
  nwP = _nwP;
  g = &(nwP->g);
  packetTracer = _packetTracer;
  nodeInfo = nwP->nodeInfo;
  linkInfo = nwP->linkInfo;

  //unprotSDs = new UnprotectedSDSet;

  bestSelection.first.clear();
  bestSelection.second.clear();
  sumOfBestProtection = 0;
  bestImproveRatio = 0;

  //no trap condition leakage handling so far
  gParamS.leak_cascCondOn = 0x0;
  gParamS.skipEdgesOn = 0x0;
  gParamS.skipUpstrLfa = 0x0;
  gParamS.nSelStrategy = CONN_L; //original INFOCOM strategy

  xmlStepSet.clear();
  toDebug = 0;
  
  cntPacketTracerCalls_Escape = 0;
  cntPacketTracerCalls_Trap = 0;
}

void GreedyHeuristic::buildUnprotectedSDSet(){
  //cleaning old version:
  unprotSDs.clear();

  debug(LOG)<<"Building set of unprotected physical (s-d)'s..."<<endl;
  //SDPair sdPair;
  for (Graph::NodeIt s(nwP->g); s != INVALID; ++s)
    for (Graph::NodeIt d(nwP->g); d != INVALID; ++d){
      if ( (s != d) &&
           (*nodeInfo)[s].isVirtual == false &&
           (*nodeInfo)[d].isVirtual == false ){
        // have no LFA
        if ( nwP->isLegacyLFAProtected_LP(s, d) == INVALID )
          {
            SDPair sdPair;
            sdPair.source      = s;
            sdPair.destination = d;
            unprotSDs.push_back(sdPair);
          }
      }
    }

  // double check that if unprotected s-d set is empty
  // the LFA coverage MUST be 1!
  if ( unprotSDs.empty() ){
    toDebug = 1;
    double finalCov = nwP->newLfaCoverageMetric_LP(LP);
    if ( nwP->equal(finalCov, 1) == false)
      cout<<"ERROR: S-D set is empty but LFA coverage != 1 ! ! !"<<endl;
  }
}

void GreedyHeuristic::printOutUnprotectedSDSet(){
  UnprotSDIt it;

  if ( !unprotSDs.empty() ){
    for (it = unprotSDs.begin(); it != unprotSDs.end(); ++it){
      cout<<"  unprotected: [ s = "<<nwP->g.id(it->source)<<" , d = "<<
        nwP->g.id(it->destination)<<" ]"<<endl;
    }
  }
  else {
    cout<<"INFO: s-d set is empty. "<<endl;
  }
}

void GreedyHeuristic::printUnprotectedSDSet(){
  UnprotSDIt it;
  int cnt = 0;

  if ( !unprotSDs.empty() ){
    for (it = unprotSDs.begin(); it != unprotSDs.end(); ++it){
      debug(INFO)<<"  unprotected: [ s = "<<nwP->g.id(it->source)<<" , d = "<<
	nwP->g.id(it->destination)<<" ]"<<endl;
      cnt++;
    }
  }
  else {
    debug(INFO)<<"INFO: s-d set is empty. "<<endl;
  }
  debug(INFO)<<"# of unprot SDs: "<<cnt<<endl;
}

/********************************************
 * This is needed for having map<SDPair, NodeSet>
 * If we store objects in maps, operator<
 * needs to be implemented!
 * Maps have to decide if an element is already
 * inserted or not. In our case, 2 SDPairs
 * are equivalent if s1 and s2 and d1 and d2
 * are equivalent.
 ********************************************/
bool operator<(SDPair sp1, SDPair sp2){
  if ( sp1.source < sp2.source )
    return true;
  if ( (sp1.source == sp2.source) &&
       (sp1.destination < sp2.destination) )
    return true;

  return false;
}

bool operator==(SDPair sp1, SDPair sp2){
  return ( (sp1.source == sp2.source) &&
           (sp1.destination == sp2.destination) );
}

bool GreedyHeuristic::isLocalSRLGDisjoint(const Node &n, const Arc &link1, const Arc &link2){
  SRGSet::iterator it;
  NodeInfoMap *nodeInfoP = nwP->nodeInfo;

  if ( link1 == link2 ) return false;

  for (it = (*nodeInfoP)[n].localSRGs.begin(); it != (*nodeInfoP)[n].localSRGs.end(); ++it)
    if ( it->count(link1) > 0 &&
         it->count(link2) > 0    ){
      return false;
    }
  return true;
}

bool GreedyHeuristic::isInLSet(const SDPair &l){
  bool isIn = false;
  UnprotSDIt lIt;

  for (lIt = L.begin(); lIt != L.end(); ++lIt)
    if (*lIt == l) isIn = true;

  return isIn;
}

/*************************************************
 * 1. Build L set (subset of unprotSDs):
 *  -s is adjacent to v'Set
 *  -s-d is unprotected
 *  -(s,nh(s,d)) && (s,v'Set) are local SRLG disjoint
 *************************************************/
void GreedyHeuristic::makeLSet(const NodeSet &vS){
  Graph *g = &(nwP->g);
  SDPair l;
  UnprotSDIt sdIt;
  NodeSet::iterator vIt;
  
  //debug(DEBUG1)<<"L set: "<<endl;
  
  //create vS parent set
  NodeSet vSParentS;
  NodeInfoMap *nodeInfoP = nwP->nodeInfo;
  for (vIt = vS.begin(); vIt != vS.end(); vIt++){
    vSParentS.insert((*nodeInfoP)[*vIt].physicalNode);
  }

  for (vIt = vS.begin(); vIt != vS.end(); vIt++){
    for (sdIt = unprotSDs.begin(); sdIt != unprotSDs.end(); ++sdIt){
      Node source = sdIt->source;
      Node dest   = sdIt->destination;

      //only consider those node sets that
      //do not contain source
      if ( vS.find(source) == vS.end() &&
           vSParentS.find(source) == vSParentS.end() &&
           nwP->neighbor(*vIt, source)  ) {

        Arc nhLink = findArc(*g, source, nwP->nh(source, dest));
        Arc vLink  = findArc(*g, source, *vIt);
        if ( isLocalSRLGDisjoint(source, nhLink, vLink) ){
          SDPair l;
          l.source = source;
          l.destination = dest;

          if ( !isInLSet(l) ){
            L.push_back(l);
            //cout<<" [ "<<g->id(l->source)<<", "<<g->id(l->destination)<<" ]"<<endl;
          }
        }
      }
    }//endfor
  }//endfor v
  //UnprotectedSDSet::iterator lIt;
  //for (lIt = L.begin(); lIt != L.end(); ++lIt){
  //  debug(DEBUG1)<<" [ "<<g->id(lIt->source)<<", "<<g->id(lIt->destination)<<" ]"<<endl;
  //}
}


/*************************************************
 * 2. Select escape nodes (E set):
 *  -g is neighbour of vSet item
 *  -if (s,nh(s,d)) fails, packet would reach d
 *************************************************/
void GreedyHeuristic::makeESet(const NodeSet &vS){
  UnprotSDIt lIt;
  Graph *g = &(nwP->g);
  NodeSet::iterator vIt;
  NodeInfoMap *nodeInfoP = nwP->nodeInfo;

  for (lIt = L.begin(); lIt != L.end(); ++lIt){
    NodeSet *escCandidates = new NodeSet();

    for (NodeIt escIt(*g); escIt != INVALID; ++escIt){
      Node s = lIt->source;
      Node d = lIt->destination;
      Node t = nwP->nh(s, d);
      for (vIt = vS.begin(); vIt != vS.end(); vIt++){
        // if v is set, escape cannot be part of
        // this set's virtual or physical nodes
        if ( nwP->neighbor(*vIt,escIt) &&
             vS.find((*nodeInfoP)[escIt].physicalNode) == vS.end() &&
             escIt != s ){

          Arc failTo   = findArc(*g, s, t);
          Arc failFrom = findArc(*g, t, s);

          LinkSet failLinks;
          failLinks.insert(failTo);
          failLinks.insert(failFrom);

          //cout<<"L item is under processing: "<<endl;
          //cout<<" ["<<g->id(s)<<" , "<<g->id(d)<<" ]";
          //cout<<" g: "<<g->id(escIt)<<endl;

          cntPacketTracerCalls_Escape++;
          if ( packetTracer->haveAllPathsValid(escIt, d, failLinks) &&
	       !(gParamS.skipUpstrLfa && (*nodeInfoP)[escIt].physicalNode == s) ){
	    escCandidates->insert(escIt);
	    //cout<<" Escape node candidate: "<<g->id(escIt)<<endl;
          }
        }
      }//endof v nodes
    }//endof esc candidates
    pair<SDPair, NodeSet> p(*lIt, *escCandidates);
    E.insert(p);

    //This caused SEGFAULT!
    //E[(**lIt)] = escCandidates;

    //cleanup
    delete escCandidates;
  }

  //Debug printout:
  map<SDPair, NodeSet>::iterator eIt;
  NodeSet::iterator nSetIt;
  debug(DEBUG1)<<"E set: "<<endl;
  for ( eIt = E.begin(); eIt != E.end(); ++eIt ){
    debug(DEBUG1)<<" [ "<<g->id((eIt->first).source)<<", "<<g->id((eIt->first).destination)<<" ] => ( ";
    for ( nSetIt = (eIt->second).begin(); nSetIt != (eIt->second).end(); ++nSetIt ){
      debug(DEBUG1)<<g->id((*nSetIt))<<" ";
    }
    debug(DEBUG1)<<")"<<endl;
  }
}

/*************************************************
 * 3. Select critical nodes who might have
 *    trap nodes. Q set.
 *    - s is adjacent to v'
 *    - nh(s, d) != v
 *
 *    Purpose of this set is to collect all the
 *    unprotected AND protected [s,d] pairs that
 *    might get fake LFA with the new virtual node.
 *************************************************/
void GreedyHeuristic::makeQSet(const NodeSet& vS){
  Graph *g = &(nwP->g);
  NodeInfoMap *nodeInfoP = nwP->nodeInfo;
  NodeSet::iterator vIt;

  //debug(DEBUG1)<<"Q set: "<<endl;

  // iterate on all neighbours of s
  for (Graph::NodeIt s(*g); s != INVALID; ++s)
    for (Graph::NodeIt d(*g); d != INVALID; ++d)
      for (vIt = vS.begin(); vIt != vS.end(); vIt++){
        Node phyV = (*nodeInfoP)[*vIt].physicalNode;
        if ( s != d && nwP->neighbor(phyV, s) && nwP->nh(s, d) != phyV){
          if ( !((*nodeInfoP)[s].isVirtual ||  (*nodeInfoP)[d].isVirtual) ){
            SDPair q;
            q.source = s;
            q.destination = d;
            Q.push_back(q);
            //debug(DEBUG1)<<" [ "<<g->id(q.source)<<", "<<g->id(q.destination)<<" ]"<<endl;
          }//endif
        }//endif
      }//endfor
}

/*************************************************
 * 4. Select trap nodes of Q, T set.
 *   - g is neighbour of v
 *     (Problem: s was trap node to [s,d], but
 *      it would have never been selected as LFA.
 *      Too loose condition, caused unnecessary
 *      trap nodes.)
 *   instead:
 *   - g is not upstream of s towards d:
 *     dist(g,d) < dist(g,s) + dist(s,d)
 *     (can g be LFA of [s,d]
 *      Note that here g is not neighbour of s
 *      like in LFA condition. It seems to be a
 *      remote LFA condition...)
 *   - if (s,nh(s,d)) fails, packet would NOT reach d
 *
 *************************************************/
void GreedyHeuristic::makeTSet(const NodeSet &vS){
  Graph *g = &(nwP->g);
  UnprotSDIt qIt;
  NodeSet::iterator vIt;

  for (qIt = Q.begin(); qIt != Q.end(); ++qIt){
    NodeSet *trapCandidates = new NodeSet();
    for (NodeIt trapIt(*g); trapIt != INVALID; ++trapIt){
      Node s = qIt->source;
      Node d = qIt->destination;
      for (vIt = vS.begin(); vIt != vS.end(); vIt++)
        // below inequality for eliminating trapNode leakage (smartTrapCondition)
        if ( nwP->neighbor(*vIt,trapIt) && ( !gParamS.smartTrapCondOn || /*|| trapIt != s ) ){*/
	     nwP->distOpt(trapIt, d) < nwP->distOpt(trapIt, s) + nwP->distOpt(s,d)) ){
	  Node nhop = nwP->nh(s, d);
	  Arc  failTo   = findArc(*g, s, nhop);
	  Arc  failFrom = findArc(*g, nhop, s);

	  LinkSet failLinks;
	  failLinks.insert(failTo);
	  failLinks.insert(failFrom);

          //cout<<"Q item is under processing: "<<endl;
          //cout<<" ["<<nwP->g.id(s)<<" , "<<nwP->g.id(d)<<" ]";
          //cout<<" g: "<<nwP->g.id(trapIt)<<endl;

	  cntPacketTracerCalls_Trap++;
          if ( !packetTracer->haveAllPathsValid(trapIt, d, failLinks) ){
            trapCandidates->insert(trapIt);
            //cout<<" Trap node candidate: "<<nwP->g.id(trapIt)<<endl;
          }
        }
    }//endof trap candidates
    pair<SDPair, NodeSet> p(*qIt, *trapCandidates);
    T.insert(p);

    //This can cause SEGFAULT!!
    //T[(**qIt)] = trapCandidates;

    //cleanup
    delete trapCandidates;
  }

  //Debug printout:
  ET_Iterator tIt;
  NodeSet::iterator nSetIt;
  debug(DEBUG1)<<"T set: "<<endl;
  for ( tIt = T.begin(); tIt != T.end(); ++tIt ){
    debug(DEBUG1)<<" [ "<<g->id((tIt->first).source)<<", "<<g->id((tIt->first).destination)<<" ] => ( ";
    for ( nSetIt = (tIt->second).begin(); nSetIt != (tIt->second).end(); ++nSetIt ){
      debug(DEBUG1)<<g->id((*nSetIt))<<" ";
    }
    debug(DEBUG1)<<")"<<endl;
  }
}


void GreedyHeuristic::printLSet(){
  UnprotSDIt lIt;
  cout<<"L set:"<<endl;
  for (lIt = L.begin(); lIt != L.end(); ++lIt){
    cout<<" [ "<<g->id(lIt->source)<<", "<<g->id(lIt->destination)<<" ]"<<endl;
  }
}

void GreedyHeuristic::printESet(){
  //Debug printout:
  ET_Iterator eIt;
  NodeSet::iterator nSetIt;

  cout<<"E set: "<<endl;
  for ( eIt = E.begin(); eIt != E.end(); ++eIt ){
    cout<<" [ "<<g->id((eIt->first).source)<<", "<<g->id((eIt->first).destination)<<" ] => ( ";
    for ( nSetIt = (eIt->second).begin(); nSetIt != (eIt->second).end(); ++nSetIt ){
      cout<<g->id((*nSetIt))<<" ";
    }
    cout<<")"<<endl;
  }
}

void GreedyHeuristic::printQSet(){
  UnprotSDIt qIt;
  cout<<"Q set:"<<endl;
  for (qIt = Q.begin(); qIt != Q.end(); ++qIt){
    cout<<" [ "<<g->id(qIt->source)<<", "<<g->id(qIt->destination)<<" ]"<<endl;
  }
}

void GreedyHeuristic::printTSet(){
  //Debug printout:
  ET_Iterator tIt;
  NodeSet::iterator nSetIt;
  cout<<"T set: "<<endl;
  for ( tIt = T.begin(); tIt != T.end(); ++tIt ){
    cout<<" [ "<<g->id((tIt->first).source)<<", "<<g->id((tIt->first).destination)<<" ] => ( ";
    for ( nSetIt = (tIt->second).begin(); nSetIt != (tIt->second).end(); ++nSetIt ){
      cout<<g->id((*nSetIt))<<" ";
    }
    cout<<")"<<endl;
  }
}

void GreedyHeuristic::cleanSets(){
  if ( L.size() != 0 )
    L.clear();
  if ( E.size() != 0 )
    E.clear();
  if ( Q.size() != 0 )
    Q.clear();
  if ( T.size() != 0 )
    T.clear();
}

/*************************************************
 * If we have E and T sets, we have to select
 * the node to be virtualized. Elements of T
 * are substracted from E set, and the node will
 * be selected that provides the most protection.
 * escCandidateMap:[Node => int(num of occurence)]
 *
 * E.g.:
 * E: [s1, d1] => (0, 1)
 *    [s2, d2] => (0, 2)
 * T: [s2, d2] => (1)
 * escCandidateMap: [0: 2]
 *                          [2: 1]
 * bestEscapeNode = 0;
 *************************************************/
EscapeMap GreedyHeuristic::getEscapeMap(){
  Graph *g = &(nwP->g);

  EscapeMap escCandidateMap;

  // 1.: count E elements in finalSet
  ET_Iterator eIt;
  NodeSet::iterator nSetIt;
  pair<EscapeMap::iterator,bool> ret;

  for ( eIt = E.begin(); eIt != E.end(); ++eIt )
    for ( nSetIt = (eIt->second).begin(); nSetIt != (eIt->second).end(); ++nSetIt ){
      //insert with "1" if does not exist already
      ret = escCandidateMap.insert(pair<Node, int>(*nSetIt, 1));
      if ( ret.second == false){
        // element is already inserted in finalSet
        // so step counter
        ret.first->second++;
      }
    }

  // 2.: remove elements of T from escCandidateMap (E)
  ET_Iterator tIt;
  EscapeMap::iterator eCandIt;

  for ( tIt = T.begin(); tIt != T.end(); ++tIt )
    for ( nSetIt = (tIt->second).begin(); nSetIt != (tIt->second).end(); ++nSetIt ){
      eCandIt = escCandidateMap.find(*nSetIt);
      if ( eCandIt != escCandidateMap.end() ){
        escCandidateMap.erase(eCandIt);
      }
    }

  //Debug
  debug(DEBUG1)<<"BestEscapeCandidateMap: "<<endl;
  for ( eCandIt = escCandidateMap.begin(); eCandIt != escCandidateMap.end(); ++eCandIt){
    debug(DEBUG1)<<"[ "<<g->id(eCandIt->first)<<" : "<<eCandIt->second<<" ]"<<endl;
  }

  // 3.: return escCandidateMap
  return escCandidateMap;
}

/*************************************************
 * Make a decision about the best node to
 * virtualize if one escape is allowed
 * -input: escCandidateMap
 * -output bestSelection <NodeSet, NodeVector> structure
 * - take into account the number of nodes to be virtualized!
 *
 * 06/05/2014: measure best selection with tunnels like this:
 *  improveRatio := # nodes to be covered / # virt nodes
 *************************************************/
void GreedyHeuristic::updateBestSelectionForOneEscape(const NodeSet &vS, EscapeMap &eMap){
  EscapeMap::iterator escMIt;

  for ( escMIt = eMap.begin(); escMIt != eMap.end(); ++escMIt){
    // calculate current improveRatio
    double _imprRatio = double(escMIt->second) / double(vS.size());

    if ( _imprRatio > bestImproveRatio ){
      bestSelection.first = vS;
      bestSelection.second.clear();
      bestSelection.second.push_back(escMIt->first);
      sumOfBestProtection = escMIt->second;
      bestImproveRatio = _imprRatio;
    }
    //prefer smaller vSet if equality
    /*else if ( escMIt->second == sumOfBestProtection ){
      if ( vS.size() < bestSelection.first.size() ){
      bestSelection.first = vS;
      bestSelection.second.clear();
      bestSelection.second.push_back(escMIt->first);
      sumOfBestProtection = escMIt->second;
      }
      }*/
  }
}


/****************************************
 * Represents 1 greedy iteration
 * It tries to virtualize nodes after
 * each other, and finally selects the one
 * that provides the most protection
 * without traps.
 * Returns <nodeToVirtulize, itsEscapeNode>
 ****************************************/
pair<NodeSet,NodeVector> GreedyHeuristic::selectNodeGreedilyWithOneEscape(){
  UnprotSDIt lIt;
  UnprotSDIt qIt;
  ET_Iterator eIt;
  ET_Iterator tIt;
  EscapeMap escMap;
  vector<NodeSet> virtCandidates;
  vector<NodeSet>::iterator vCIt;
  NodeSet::iterator nSIt;

  virtCandidates = nwP->virtCandidates;

  for (vCIt = virtCandidates.begin(); vCIt != virtCandidates.end(); ++vCIt){

    debug(DEBUG1)<<"============ v': ";
    for (nSIt = vCIt->begin(); nSIt != vCIt->end(); ++nSIt){
      debug(DEBUG1)<<nwP->g.id(*nSIt)<<" ";
    }
    debug(DEBUG1)<<"============"<<endl;

    makeLSet(*vCIt);
    makeESet(*vCIt);
    makeQSet(*vCIt);
    makeTSet(*vCIt);

    //printESet();
    //printTSet();

    escMap = getEscapeMap();

    //One escape is allowed
    updateBestSelectionForOneEscape(*vCIt, escMap);

    //cleanup memory
    L.clear();

    for (eIt = E.begin(); eIt != E.end(); ++eIt){
      eIt->second.clear();
    }
    E.clear();

    Q.clear();

    for (tIt = T.begin(); tIt != T.end(); ++tIt){
      tIt->second.clear();
    }
    T.clear();
  }

  // cleanup member variables
  // for next greedy step
  pair<NodeSet, NodeVector> tmpBestSel(bestSelection.first, bestSelection.second);
  bestSelection.first.clear();
  bestSelection.second.clear();
  sumOfBestProtection   = 0;
  bestImproveRatio = 0;

  //Debug
  /*cout<<"tmpBestSel"<<endl;
    NodeSet::iterator it;
    for (it=tmpBestSel.first.begin(); it!=tmpBestSel.first.end();++it){
    cout<<" (v): "<<nwP->g.id(*it);
    }*/

  return tmpBestSel;
}

/****************************************
 * Just to check that SRLGs are stored
 * correctly
 ****************************************/
void GreedyHeuristic::printLocalSRLGs(){
  for (NodeIt n(*g); n != INVALID; ++n){
    cout<<"node: "<<g->id(n);
    SRGSet srlg = (*nodeInfo)[n].localSRGs;
    SRGSet::iterator srlgIt;
    LinkSet::iterator lsIt;

    cout<<" #SRLGs: "<<srlg.size()<<endl;
    for (srlgIt = srlg.begin(); srlgIt != srlg.end(); ++srlgIt){
      cout<<" srlg: ";
      for (lsIt = srlgIt->begin(); lsIt != srlgIt->end(); ++lsIt){
        cout<<g->id(*lsIt)<<" ("<<g->id(g->source(*lsIt))<<"->"
            <<g->id(g->target(*lsIt))<<") ";
      }
      cout<<endl;
    }
  }
}

/*
 * It is only good for greedy heuristic
 * SRLGs are different when trees are used:
 * E.g.: (v,v') link is virtual but does not
 * have physical pair, however it is in SRLG!
 */
void GreedyHeuristic::fillNodeInfoLocalSrgTables(){
  for (NodeIt n(*g); n != INVALID; ++n){
    (*nodeInfo)[n].localSRGs.clear();

    for (Graph::OutArcIt phyL(*g, n); phyL != INVALID; ++phyL){
      if ( (*linkInfo)[phyL].isVirtual == false ){
        LinkSet _srlg;

        for (Graph::OutArcIt virL(*g, n); virL != INVALID; ++virL){
          if (  (*linkInfo)[virL].isVirtual == true )
            if ( (*linkInfo)[virL].physicalLink ==  phyL ){
              _srlg.insert(phyL);
              _srlg.insert(virL);

              //reverse arcs will also fail jointly...
              Arc reversePhyL = findArc(*g, g->target(phyL), g->source(phyL));
              Arc reverseVirL = findArc(*g, g->target(virL), g->source(virL));
              _srlg.insert(reversePhyL);
              _srlg.insert(reverseVirL);
            }//endif
        }//endfor

        if ( _srlg.size() != 0 )
          (*nodeInfo)[n].localSRGs.insert(_srlg);

      }//endif phyisical
    }//endfor phyL
  }//endfor n
}

/***************************************
 * removes a virtual node
 ***************************************/
double GreedyHeuristic::rollbackVirtualNode(const NodeSet& virtS){
  NodeSet::iterator nSIt;

  for (nSIt = virtS.begin(); nSIt != virtS.end(); ++nSIt){
    g->erase(*nSIt);

    //recalculate everything
    nwP->db->initializeDB();
    fillNodeInfoLocalSrgTables();
    buildUnprotectedSDSet();
  }
  return nwP->newLfaCoverageMetric_LP(LP);
}

/***************************************
 * creates a new virtual node and
 * gives back new lfaCoverage for
 * greedy algorithm... 
 ***************************************/
NodeSet GreedyHeuristic::realizeVirtualNode(const NodeSet& vS, const Node& escape){
  Cost LSP = nwP->LSP;
  NodeSet virtualNodes;
  NodeSet::iterator nSIt;
  ET_Iterator eIt;
  NodeSet vSParentS;

  // count sets before touching the topology!
  // continue below... forget this in current impl...
  if ( gParamS.skipEdgesOn ){
    makeLSet(vS);
    makeESet(vS);
  }

  // put virtual node(s)
  // create vSParentS
  for (nSIt = vS.begin(); nSIt != vS.end(); ++nSIt){
    Node virtualV = g->addNode();
    virtualNodes.insert(virtualV);
    (*nodeInfo)[virtualV].isVirtual    = true;
    (*nodeInfo)[virtualV].physicalNode = (*nodeInfo)[*nSIt].physicalNode;
    vSParentS.insert((*nodeInfo)[*nSIt].physicalNode);
  }

  // make 1 cost links within virtual neighbours
  // (if have more virtual nodes)
  NodeSet::iterator vSIt;
  NodeSet::iterator neighVSIt;
  for (vSIt = virtualNodes.begin(); vSIt != virtualNodes.end(); ++vSIt){
    Node phyV = (*nodeInfo)[*vSIt].physicalNode;
    for (neighVSIt = virtualNodes.begin(); neighVSIt != virtualNodes.end(); ++neighVSIt){
      if ( *vSIt < *neighVSIt ){
        Node phyNeighV = (*nodeInfo)[*neighVSIt].physicalNode;
        if ( nwP->neighbor(phyV, phyNeighV) ){
          Arc lowCostArc1 = g->addArc(*vSIt, *neighVSIt);
          Arc lowCostArc2 = g->addArc(*neighVSIt, *vSIt);
          Arc phyArc1 = findArc(*g, phyV, phyNeighV);
          Arc phyArc2 = findArc(*g, phyNeighV, phyV);

          (*(nwP->cost))[lowCostArc1] = (*(nwP->cost))[phyArc1];
          (*(nwP->cost))[lowCostArc2] = (*(nwP->cost))[phyArc2];

          (*linkInfo)[lowCostArc1].isVirtual    = true;
          (*linkInfo)[lowCostArc1].physicalLink = phyArc1;
          (*linkInfo)[lowCostArc2].isVirtual    = true;
          (*linkInfo)[lowCostArc2].physicalLink = phyArc2;
        }
      }
    }
  }

  // create links between virtual and physical nodes
  for (vSIt = virtualNodes.begin(); vSIt != virtualNodes.end(); ++vSIt){
    Node phyV = (*nodeInfo)[*vSIt].physicalNode;
    for (Graph::OutArcIt e(*g, phyV); e!= INVALID; ++e){
      Node neighOfV = g->target(e);
      if ( neighOfV == escape ){
        Arc escapeArc1 = g->addArc(*vSIt, neighOfV);
        Arc escapeArc2 = g->addArc(neighOfV, *vSIt);
        (*(nwP->cost))[escapeArc1] = 1;
        (*(nwP->cost))[escapeArc2] = 1;

        (*linkInfo)[escapeArc1].isVirtual    = true;
        (*linkInfo)[escapeArc1].physicalLink = (*linkInfo)[e].physicalLink;
        Arc reverse = findArc(*g, neighOfV, phyV);
        (*linkInfo)[escapeArc2].isVirtual    = true;
        (*linkInfo)[escapeArc2].physicalLink = (*linkInfo)[reverse].physicalLink;
      }
      // if virtualizing multiple nodes (tunnel), do not connect
      // a virtual node to a physical one if it is a parent of someone
      // from the tunnel
      else if ( find(virtualNodes.begin(), virtualNodes.end(), neighOfV) == virtualNodes.end() 
                && vSParentS.find(neighOfV) == vSParentS.end() ) {
        Arc highCostArc1 = g->addArc(*vSIt, neighOfV);
        Arc highCostArc2 = g->addArc(neighOfV, *vSIt);
        (*(nwP->cost))[highCostArc1] = LSP;
        (*(nwP->cost))[highCostArc2] = LSP;

        (*linkInfo)[highCostArc1].isVirtual    = true;
        (*linkInfo)[highCostArc1].physicalLink = (*linkInfo)[e].physicalLink;
        Arc reverse = findArc(*g, neighOfV, phyV);
        (*linkInfo)[highCostArc2].isVirtual    = true;
        (*linkInfo)[highCostArc2].physicalLink = (*linkInfo)[reverse].physicalLink;
      }
    }
  }

  //update databases
  nwP->db->initializeDB();
  fillNodeInfoLocalSrgTables();
  buildUnprotectedSDSet();

  //cleanup sets
  UnprotectedSDSet::iterator lIt;
  L.clear();
  for (eIt = E.begin(); eIt != E.end(); ++eIt){
    eIt->second.clear();
  }
  E.clear();

  return virtualNodes;
}

void GreedyHeuristic::setGreedyParamSet(const GreedyParamSet &_gPS){
  gParamS.skipEdgesOn = _gPS.skipEdgesOn;
  gParamS.leak_cascCondOn = _gPS.leak_cascCondOn;
  gParamS.skipUpstrLfa = _gPS.skipUpstrLfa;
  gParamS.smartTrapCondOn = _gPS.smartTrapCondOn;
  if ( _gPS.skipUpstrLfa ) packetTracer->setUseOfUpstrLFA(true);
  if ( _gPS.leak_cascCondOn ) packetTracer->setUseOfCascLFA(false);
  gParamS.nSelStrategy = _gPS.nSelStrategy;
}

void GreedyHeuristic::greedyHeuristicAlgorithm(const double &ratio, const int &maxNodeSetSize){
  NodeSet ToVirtualizeS;
  Node toBeEscape;
  NodeSet::iterator nSIt;

  double currLFACoverage = 0; //dummy
  double nextLFACoverage = 5;  //dummy
  //int allowedVirtualNodes = ratio*(nwP->numOfPhysicalNodes);
  int numOfVirtualNodes = 0;
  Timer t;

  /*
   * Create node sets to virt
   */
  nwP->calculateNodeSets(maxNodeSetSize);

  /*
   * First we try to improve with 1 virtual node/step
   *
   * Runs until
   *  1.: (#virtual nodes) / (#physical nodes) != ratio
   *  2.: we can improve LFA coverage >>nextLFACoverage > currLFACoverage<<
   * (3.: coverage = 1 >>!equal(newLFACoverage, 1)<< )
   */

  t.restart();  
  while ( /*(numOfVirtualNodes <= allowedVirtualNodes) &&*/
	 !( unprotSDs.empty() )         )
    {
      pair<NodeSet,NodeVector> bestSelection = selectNodeGreedilyWithOneEscape();
      ToVirtualizeS = bestSelection.first;

      if ( bestSelection.second.size() != 0 ){
        toBeEscape   = bestSelection.second.front();
      }

      if ( ToVirtualizeS.size() == 0 ){
        debug(CRITICAL)<<"INFO: Any node is selected by greedy..."<<endl;

        if ( gParamS.nSelStrategy == SPS ){
          debug(INFO)<<"INFO: SPT Based Virtualization stuck...exit."<<endl;
          break;
        }
        break;

      }
      else {
        currLFACoverage = nwP->newLfaCoverageMetric_LP(LP);

        NodeSet virtS = realizeVirtualNode(ToVirtualizeS, toBeEscape);
        numOfVirtualNodes += virtS.size();

        nextLFACoverage = nwP->newLfaCoverageMetric_LP(LP);

        if ( nwP->equal(nextLFACoverage, currLFACoverage) ||
             nextLFACoverage < currLFACoverage                          ){
          debug(INFO)<<"==================================="<<endl;
          debug(INFO)<<"INFO: Greedy has reached maximal protection with "<<ToVirtualizeS.size()<<" nodes."<<endl;
          debug(INFO)<<"INFO: Removing last virtual node(set)."<<endl;

          rollbackVirtualNode(virtS);
          numOfVirtualNodes -= virtS.size();

          nextLFACoverage = currLFACoverage;

          debug(INFO)<<"============ Virtual node of ";
          for (nSIt = ToVirtualizeS.begin(); nSIt != ToVirtualizeS.end(); ++nSIt){
            debug(INFO)<<nwP->g.id(*nSIt)<<" ";
          }
          debug(INFO)<<" is unnecessary ========="<<endl;
          break;

        }
        else {
          debug(INFO)<<"==================================="<<endl;
          for (nSIt = ToVirtualizeS.begin(); nSIt != ToVirtualizeS.end(); ++nSIt){
            debug(INFO)<<" Node To Virtualize = "<<g->id(*nSIt)<<endl;
          }
          GreedyXmlSet gXS;
          gXS.sumOfVirtualNsAtStep = numOfVirtualNodes;
          gXS.currLFACoverage = nextLFACoverage;
          gXS.timeStamp = t;

          // calculate current AVPL
          int tmpLoopingSD;
          nwP->checkLoopAndAVPL(tmpLoopingSD);
          gXS.currAVPL = nwP->wFaultAPL;
          xmlStepSet.push_back(gXS);
        }

        debug(INFO)<<" Escape node = "<<g->id(toBeEscape)<<endl;
        debug(INFO)<<" Current LFA cov = "<<currLFACoverage<<endl;
        debug(INFO)<<" New LFA coverage = "<<nextLFACoverage<<endl;

        printUnprotectedSDSet();
      }
    }//endwhile

    t.stop();
}


GreedyHeuristic::~GreedyHeuristic(){
  unprotSDs.clear();
  xmlStepSet.clear();
}
