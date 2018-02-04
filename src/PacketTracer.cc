/*
 * Written by Mate Nagy
 * License: GNU General Public License Version 3
 *
 */

#include "PacketTracer.hh"

PacketTracer::PacketTracer(Network *_nwP){
  nwP = _nwP;
  haveLoop = false;
  nHopIsPhysical = true;
  path.clear();
  ecmpPath.clear();

  //by default, PT can hop on
  //upstream LFAs and cascade LFAs
  skipUpstrLFA = false;
  useCascLFA = false;
  lfaCnt = 0;
  source = INVALID;
  destin = INVALID;

  numOfPossiblePaths = 0;
}

void PacketTracer::setUseOfUpstrLFA(const bool &_uCOn){
  skipUpstrLFA = _uCOn;
}

void PacketTracer::setUseOfCascLFA(const bool &_cCOn){
  useCascLFA = _cCOn;
  lfaCnt = 0;
}

// Collects all possible next-hops to d (in case of ECMP there are more)
// except those to whom link is failing.
void PacketTracer::getAllNextHopsToD(const Node &s, const Node &d,
                                     const LinkSet &failLinks, NodeVector &nHops){
  Node pNH = nwP->nh(s,d); //primary next-hop
  nHops.push_back(pNH);

  for (Graph::OutArcIt e(nwP->g, s); e != INVALID; ++e){
    Node n = nwP->g.target(e);
    if ( nwP->distOpt(s, d) == nwP->distOpt(s,n) + nwP->distOpt(n,d) &&
            n != pNH){
      haveValidPath(s, d, failLinks);
      if ( find(path.begin(), path.end(), n) == path.end() ){
        // now we know that n is an ECMP neighbor
        // is it safe?
        if ( failLinks.count(e) == 0 )
          nHops.push_back(n);
      }
    }
  }
}

void PacketTracer::getAllLFAsToD(const Node &s, const Node &d,
                                 const LinkSet &failLinks, NodeVector &nHops){
  Node nHop = nwP->nh(s, d);
  Arc nHopLink  = findArc(nwP->g, s, nHop);
  LinkInfoMap *linkInfo = nwP->linkInfo;
  GreedyHeuristic *greedyH = nwP->greedyH;

  // collect all the LFAs (except those, whose
  // link is failing jointly)
  for (Graph::OutArcIt e(nwP->g, s); e != INVALID; ++e){
    Node n = nwP->g.target(e);

    if (n != nHop &&
        nwP->distOpt(n, d) <
        nwP->distOpt(s, n) + nwP->distOpt(s, d) &&
        greedyH->isLocalSRLGDisjoint(s, nHopLink, e) &&
        failLinks.count((*linkInfo)[e].physicalLink) == 0 ) {

      // if LFA is not yet in nHops vector, than insert it
      // (can be previously inserted if n is ECMP neighbor)
      if ( find(nHops.begin(), nHops.end(), n) == nHops.end() ){
        nHops.push_back(n);
      }
    }
  }
}

Node PacketTracer::selectForwardingNode(const Node &currNode, const Node &d,
                                        const LinkSet &linksFail, bool &isLFA){
  GreedyHeuristic *algorithms = nwP->greedyH;

  // mark if returned node was LFA or not
  isLFA = false;

  //select one failing link, SRLGs have both directions
  //do we need failing LinkSet???
  LinkSet::iterator lsIt = linksFail.begin();
  //Arc failingLink = *lsIt;

  Node nHop = nwP->nh(currNode, d);
  Arc nHopLink = findArc(nwP->g, currNode, nHop);
  NodeInfoMap *nodeInfoP = nwP->nodeInfo;
  LinkInfoMap *linkInfoP = nwP->linkInfo;

  /*
   * Note, that nHopLink and lfaLink are never
   * in a common SRLG (we select LFAs so)
   */

  // case 1:
  bool localSRLDisjoint = true;
  for (lsIt = linksFail.begin(); lsIt != linksFail.end(); ++lsIt){
    if ( ! (algorithms->isLocalSRLGDisjoint(currNode, *lsIt, nHopLink)) )
      localSRLDisjoint = false;
  }

  if ( !localSRLDisjoint ){
    Node lfa = nwP->isLegacyLFAProtected_LP(currNode, d, linksFail);
    if ( (lfa != INVALID) ){
      if (  skipUpstrLFA &&
            ( nwP->distOpt(lfa, d) <
              nwP->distOpt(lfa,(*nodeInfoP)[currNode].physicalNode) +
              nwP->distOpt((*nodeInfoP)[currNode].physicalNode, d) )
            )
        {
          debug(DEBUG2)<<"Jump to LFA (!useUpstrLFA)."<<endl;
          isLFA = true;
          return lfa;
        }
      else if ( !skipUpstrLFA ){
        debug(DEBUG2)<<"Jump to LFA (useUpstrLFA)."<<endl;
        isLFA = true;
        return lfa;
      }
      else {
        debug(DEBUG2)<<"There is no or just upstream LFA..."<<endl;
        return INVALID;
      }
    }
    else {
      debug(DEBUG2)<<"Neither next-hop, nor LFA is accessible...Blackhole!"<<endl;
      return INVALID;
    }
  }

  /*
   * Is nHopLink a failing component?
   * -nHopLink is physical and failing OR
   * -nHopLink is virtual and its phys pair is failing
   */
  if ( linksFail.count(nHopLink) > 0 ||
       linksFail.count((*linkInfoP)[nHopLink].physicalLink) > 0){
    //do we have LFA that is not reached via failing Link?
    Node lfa = nwP->isLegacyLFAProtected_LP(currNode, d, linksFail);
    Arc lfaLink = findArc(nwP->g, currNode, lfa);

    if ( lfaLink != INVALID ){
      bool lfaLinkFail = linksFail.count((*linkInfoP)[lfaLink].physicalLink);

      if ( !lfaLinkFail ){
          if ( skipUpstrLFA &&
             ( nwP->distOpt(lfa, d) <
               nwP->distOpt(lfa,(*nodeInfoP)[currNode].physicalNode) +
               nwP->distOpt((*nodeInfoP)[currNode].physicalNode, d) )
             )
          {
            isLFA = true;
            return lfa;
          }
        else if ( !skipUpstrLFA ){
          isLFA = true;
          return lfa;
        }
        else {
          debug(DEBUG2)<<"Lfa link is not failing but it is upstream LFA."<<endl;
          return INVALID;
        }
      }
      else {
        debug(DEBUG2)<<"Next-hop link is failing with lfaLink...Blackhole!"<<endl;
        return INVALID;
      }

    }//lfaLink == INVALID
    else {
      debug(DEBUG2)<<"Next-hop link is failing and no LFA...Blackhole!"<<endl;
      return INVALID;
    }
  }

  /****************************************
   * To be able to check if packet is
   * forwarded in physical layer (to test
   * if original SPT has not changed)
   * No link failure is considered when
   * we check this => there is no LFA in
   * the forwarding
   ****************************************/
  if ( (*(nwP->nodeInfo))[nHop].isVirtual == true )
    nHopIsPhysical = false;

  return nHop;
}

bool PacketTracer::jumpToNextNode(int &TTL, Node currNode,
                                  const Node& d, const LinkSet &linksFail){
  //needed for multiple paths for jumpOnAllShortestPaths
  bool isLFA;

  // loop protection
  if ( TTL <= 0 ){
    debug(DEBUG2)<<" We are in loop. "<<endl;
    haveLoop = true;
    return false;
  }

  // are we in d?
  if ( currNode == d ){
    return true;
  }

  Node nextToJump = selectForwardingNode(currNode, d, linksFail, isLFA);
  path.push_back(currNode);
  if ( nextToJump == INVALID ){
    debug(DEBUG2)<<"Not able to forward packet."<<endl;
    return false;
  }

  // No LFA is allowed in the virtual layer
  if ( isLFA && (*(nwP->nodeInfo))[currNode].isVirtual ){
    return false;
  }

  TTL--;
  debug(DEBUG2)<<" Processed node: "<<(*nwP).g.id(currNode)<<endl;
  //cout<<"Processed node: "<<(*nwP).g.id(currNode)<<endl;

  jumpToNextNode(TTL, nextToJump, d, linksFail);

  // decision is made if true or false
  // before we get here
}

void PacketTracer::jumpOnAllShortestPaths(int &TTL, const Node &s, const Node &d,
                                          const LinkSet &linksFail, NodeVector &currPath, bool &allPathsSafe){
  // If there are more possible next-hops (ECMP)
  // or more possible LFAs (we arrive to a failing link)
  // we store them in nHops
  NodeVector nHops;
  NodeVector::iterator vIt;

  // loop protection
  if ( TTL <= 0 ){
    debug(DEBUG2)<<" We are in loop. "<<endl;

    allPathsSafe = false;
    haveLoop = true;
    return;
  }

  getAllNextHopsToD(s, d, linksFail, nHops);

  // if there is a step when forwarding
  // is impossible => fail
  bool nextIsLFA;
  Node nextToJump = selectForwardingNode(s, d, linksFail, nextIsLFA);

  if ( nextToJump == INVALID ){
    allPathsSafe = false;
    return;
  }

  // No LFA is allowed in the virtual layer
  if ( nextIsLFA && (*(nwP->nodeInfo))[s].isVirtual ){
    allPathsSafe = false;
    return;
  }

  // if we continue on an LFA, insert all the
  // LFAs to nHops and take out the faulty next-hop
  if ( nextIsLFA ){
    getAllLFAsToD(s, d, linksFail, nHops);

    Node failNHop = nwP->nh(s,d);
    NodeVector::iterator delIt = find(nHops.begin(), nHops.end(), failNHop);
    if ( delIt == nHops.end() ){
      cerr<<"ERROR: Trying to remove an element that is not in nHops!"<<endl;
      exit(-1);
    }
    nHops.erase(delIt);
  }

  for (vIt = nHops.begin(); vIt != nHops.end(); ++vIt){
    if ( *vIt == d ) {
      //cout<<"Branch finished with "<<nwP->g.id(*vIt)<<endl;
      currPath.push_back(*vIt);

      // Store max 10 ECMP paths
      if ( ecmpPath.size() < 10 ){
        ecmpPath.push_back(currPath);
      }
      currPath.pop_back();

      numOfPossiblePaths++;
      continue;
    }
    //cout<<"NextP: "<<nwP->g.id(*vIt)<<endl;
    currPath.push_back(*vIt);
    TTL--;
    jumpOnAllShortestPaths(TTL, *vIt, d, linksFail, currPath, allPathsSafe);

    // just for storing the right nodes in the path
    NodeVector::iterator leaf = find(currPath.begin(), currPath.end(), *vIt);
    currPath.erase(leaf, currPath.end());
  }
}

void PacketTracer::printAllPossiblePaths(){
  cout<<" PRINTING ALL POSSIBLE PATHS:"<<endl;
  cout<<" Consider that max 10 paths are stored! "<<endl;
  cout<<" Num of all ECMPs: "<<numOfPossiblePaths<<endl;
  cout<<" Num of stored ECMPs: "<<ecmpPath.size()<<endl;
  vector<NodeVector>::iterator nVIt;
  NodeVector::iterator vIt;
  NodeInfoMap *nodeInfo = nwP->nodeInfo;

  for (nVIt = ecmpPath.begin(); nVIt != ecmpPath.end();++nVIt){
    cout<<"  [";
    for (vIt = nVIt->begin(); vIt != nVIt->end(); ++vIt){
      cout<<" "<<nwP->g.id(*vIt)<<"("<<
        nwP->g.id((*nodeInfo)[*vIt].physicalNode)<<") ";
    }
    cout<<" ]"<<endl;
  }
}

// The "path" member variable contains the result
// of function haveValidPath(). After running that
// this one prints out the visited path.
void PacketTracer::printPrimaryPath(){
  NodeInfoMap *nodeInfo = nwP->nodeInfo;
  cout<<" PRINTING THE PRIMARY PATH:"<<endl;
  cout<<" Path ("<<nwP->g.id(source)<<"->"<<nwP->g.id(destin)<<"): [ ";
  vector<Node>::iterator it;
  for (it = path.begin(); it != path.end();++it)
    cout<<nwP->g.id(*it)<<"("<<nwP->g.id((*nodeInfo)[*it].physicalNode)<<")"<<" ";
  cout<<"]"<<endl;
  cout<<" Size: "<<path.size()<<endl;
}

bool PacketTracer::haveAllPathsValid(const Node &s, const Node &d, const LinkSet &linksFail){
  ecmpPath.clear();
  numOfPossiblePaths = 0;
  haveLoop = false;
  NodeVector currPath;
  int TTL = 64;

  currPath.push_back(s);
  bool allPathsSafe = true;

  if ( s == d ){
    return true;
  }

  jumpOnAllShortestPaths(TTL, s, d, linksFail, currPath, allPathsSafe);

  // just for Debugging
  //if ( haveLoop ) cerr<<"INFO: Loop is detected in the network!"<<endl;
  return allPathsSafe;
}

bool PacketTracer::haveValidPath(const Node &s, const Node &d, const LinkSet &linksFail){
  haveLoop = false;
  source = s;
  destin = d;
  int TTL = 64;
  path.clear();
  lfaCnt = 0;

  if ( s == d){
    return true;
  }

  debug(DEBUG2)<<"  haveValidPath("<<nwP->g.id(s)<<", "<<nwP->g.id(d)<<"): ";
  bool havePath = jumpToNextNode(TTL, s, d, linksFail);
  debug(DEBUG2)<<havePath<<endl;

  return havePath;
}

const NodeVector& PacketTracer::getPath(){
  return path;
}


