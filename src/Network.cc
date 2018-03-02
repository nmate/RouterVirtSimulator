/*
 * Written by Mate Nagy
 * License: GNU General Public License Version 3
 *
 */

#include "Network.hh"

Network::Network(const string & argument){
  graphName = argument;
  cost = new CostMap(g);

  linkInfo = new LinkInfoMap(g);
  nodeInfo = new NodeInfoMap(g);

  packetTracer = new PacketTracer(this);
  greedyH = new GreedyHeuristic(this, packetTracer);

  if ( haveVirtualElements() ){
    readGraph_virtual();
  }
  else {
    readGraph();
  }

  db = new Database(this);

  countPhysicalElements();
  lspPhy = longestShortestPath();

  greedyH->buildUnprotectedSDSet();

  LSP = 2*longestShortestPath();

  woFaultAPL = 0;
  wFaultAPL = 0;

  virtCandidates.clear();
}

void Network::calculateNodeSets(const int &size){
  switch (greedyH->gParamS.nSelStrategy){
  case CONN_L:
    buildUniformSetOfNodes(size);
    //printVirtCandidateNodeSets();
    break;
  case SPS:
    buildPathSetOfNodes(size);
    //printVirtCandidateNodeSets();
    break;
  }
}

bool Network::haveVirtualElements(){
  ifstream file(graphName.c_str());
  string line;

  while ( file.good() ) {
    getline (file,line);
    if ( line.find("physicalNode") != string::npos ){
      file.close();
      return true;
    }
  }
  file.close();
  return false;
}

bool Network::readGraph(){
  try {
    digraphReader(g, graphName).
      arcMap("cost", *cost).
      run();
  }
  catch (Exception& error) {
    cerr << "Error: " << error.what() << endl;
    cerr << "Not able to read " <<graphName<<endl;
    throw;
  }

  /********************************************
   * fill Node/ArcInfo Map with default values
   ********************************************/
  for (NodeIt n(g); n != INVALID; ++n){
    (*nodeInfo)[n].isVirtual = false;
    (*nodeInfo)[n].physicalNode = n;
    (*nodeInfo)[n].layerID = PHYS;
    //no SRLGs yet
  }

  for (ArcIt e(g);  e!= INVALID; ++e){
    (*linkInfo)[e].isVirtual = false;
    (*linkInfo)[e].physicalLink = e;
    (*linkInfo)[e].layerID = PHYS;
  }

  return true;
}

bool Network::readGraph_virtual(){
  Graph::NodeMap<int> tmpPhysNMap(g);
  Graph::ArcMap<int> tmpPhysLMap(g);

  try {
    digraphReader(g, graphName).
      arcMap("cost", *cost).
      nodeMap("physicalNode", tmpPhysNMap).
      arcMap("physicalLink", tmpPhysLMap).
      run();
  }
  catch (Exception& error) {
    cerr << "Error: " << error.what() << endl;
    cerr << "Not able to read " <<graphName<<endl;
    return false;
  }

  /*****************************************
   * mark virtual nodes in nodeInfo table:
   * srg parameters are filled
   * later in fillInfoTables()
   *****************************************/
  for (NodeIt n(g); n != INVALID; ++n){
    if ( tmpPhysNMap[n] != g.id(n) ){
      (*nodeInfo)[n].isVirtual = true;
      (*nodeInfo)[n].physicalNode = g.nodeFromId(tmpPhysNMap[n]);
    }
    else {
      (*nodeInfo)[n].isVirtual = false;
      (*nodeInfo)[n].physicalNode = g.nodeFromId(tmpPhysNMap[n]);
    }
  }

  for (ArcIt e(g);  e!= INVALID; ++e){
    if ( tmpPhysLMap[e] != g.id(e) ){
      (*linkInfo)[e].isVirtual = true;
      (*linkInfo)[e].physicalLink = g.arcFromId(tmpPhysLMap[e]);
    }
    else {
      (*linkInfo)[e].isVirtual = false;
      (*linkInfo)[e].physicalLink = g.arcFromId(tmpPhysLMap[e]);
    }
  }

  greedyH->fillNodeInfoLocalSrgTables();

  return true;
}

void Network::writeGraph() const{
  Graph::NodeMap<int> phyM(g);
  Graph::ArcMap<int> phyMArc(g);

  for (NodeIt n(g); n != INVALID; ++n){
    phyM[n] = g.id((*nodeInfo)[n].physicalNode);
  }
  for (ArcIt e(g); e != INVALID; ++e){
    phyMArc[e] = g.id((*linkInfo)[e].physicalLink);
  }
  digraphWriter(g, "./result_graph.lgf").
    nodeMap("physicalNode", phyM).
    arcMap("cost", *cost).
    arcMap("physicalLink", phyMArc).
    run();
}

void Network::printGraph(){
  cout<<"============================================================="<<endl;
  cout<<"                    PRINT GRAPH PROPERTIES                   "<<endl;
  cout<<"============================================================="<<endl;
  cout<<"| Node | isVirtual | layerID | physical |"<<endl;
  for (NodeIt s(g); s != INVALID; ++s){
    cout<<"|";
    cout<<setw(5)<<right<<g.id(s);
    cout<<setw(2)<<right<<"|";
    cout<<setw(6)<<right<<(*nodeInfo)[s].isVirtual;
    cout<<setw(6)<<right<<"|";
    cout<<setw(5)<<right<<(*nodeInfo)[s].layerID;
    cout<<setw(5)<<right<<"|";
    cout<<setw(6)<<right<<g.id((*nodeInfo)[s].physicalNode);
    cout<<setw(5)<<right<<"|"<<endl;
  }

  cout<<endl;
  cout<<"| Link | Src -> Dst | cost | isVirtual | layerID | physical |"<<endl;
  for (Graph::ArcIt arc(g); arc != INVALID; ++arc){
    cout<<"|";
    cout<<setw(5)<<right<<g.id(arc);
    cout<<setw(2)<<right<<"|";
    cout<<setw(4)<<right<<g.id(g.source(arc))<<" ->";
    cout<<setw(4)<<right<<g.id(g.target(arc));
    cout<<setw(2)<<right<<"|";
    cout<<setw(5)<<setprecision(3)<<right<<(*cost)[arc];
    cout<<setw(2)<<"|";
    cout<<setw(6)<<right<<(*linkInfo)[arc].isVirtual;
    cout<<setw(6)<<right<<"|";
    cout<<setw(5)<<right<<(*linkInfo)[arc].layerID;
    cout<<setw(5)<<right<<"|";
    cout<<setw(6)<<right<<g.id((*linkInfo)[arc].physicalLink);
    cout<<setw(5)<<right<<"|"<<endl;

    //cout<<"  Arc: "<<g.id(arc)<<" "<<g.id(g.source(arc))<<" --> " <<
    //          g.id(g.target(arc))<<endl;

  }
  cout<<"============================================================="<<endl;
  cout<<" # physical nodes : "<<numOfPhysicalNodes<<endl;
  cout<<" # physical arcs  : "<<numOfPhysicalArcs<<endl;
  cout<<" # virtual  nodes : "<<countNodes(g)-numOfPhysicalNodes<<endl;
  cout<<" # virtual  arcs  : "<<countArcs(g)-numOfPhysicalArcs<<endl;
  cout<<"============================================================="<<endl;
}

Node Network::nh(Graph::Node s, Graph::Node d) const{
  Node _next_hop  = (*(*(db->nhmap))[s])[d];
  if(_next_hop == INVALID){
    cerr<<"There is no next-hop between: "<<g.id(s)<<" and "<<g.id(d)<<endl;
    cerr << "Graph is not strongly connected, bailing out..." << endl;
    exit(-1);
  }
  return _next_hop;
}

bool Network::neighbor(Graph::Node s, Graph::Node d) const{
  bool n = false;
  for (Graph::OutArcIt e(g, s); e!= INVALID; ++e)
    if ( d == g.target(e) )
      n = true;
  return n;
}

bool Network::equal(Cost a, Cost b) const{
  // 0.001 was too small for sptBasedVirt
  double epsilon = 0.00001;
  return ( fabs(a-b) < epsilon );
}

Cost Network::distOpt(Graph::Node s, Graph::Node d) const{
  Cost _dist  = (*(*(db->distmap))[s])[d];

  if( equal(_dist, -1) ){
    cerr<<"There is no distance between: "<<g.id(s)<<" and "<<g.id(d)<<endl;
    cerr << "Graph is not strongly connected, bailing out..." << endl;
    exit(-1);
  }
  return _dist;
}

// returns distance of unicost version of the graph
Cost Network::distUni(Graph::Node s, Graph::Node d) const{
  Cost _dist  = (*(*(db->uniDistMap))[s])[d];

  if( equal(_dist, -1) ){
    cerr<<"There is no distance between: "<<g.id(s)<<" and "<<g.id(d)<<endl;
    cerr << "Graph is not strongly connected, bailing out..." << endl;
    exit(-1);
  }
  return _dist;
}

bool Network::isLFACondTrue(const Node &s, const Node &d, const Node &n) const{
  Cost RHS = distOpt(s, n) + distOpt(s, d);
  Cost LHS = distOpt(n, d);

  if ( (RHS-LHS) > 0.01 ){
    return true;
  }
  return false;
}

Node Network::isLegacyLFAProtected_LP(const Node &s, const Node &d,
                                      const LinkSet &lsFail) const{
  if ( s != d ){
    //prefer physical LFA's
    Node lfa = INVALID;
    Node nHop = nh(s, d);
    Arc nHopLink  = findArc(g, s, nHop);

    // Give back an LFA if have such neighbor.
    // If the LFA link is failing, give back
    // another LFA if exists.
    for (Graph::OutArcIt e(g, s); e != INVALID; ++e){
      if (g.target(e) != nHop &&
          isLFACondTrue(s,d,g.target(e)) &&
          greedyH->isLocalSRLGDisjoint(s, nHopLink, e) &&
          lsFail.count((*linkInfo)[e].physicalLink) == 0 )
        {
          lfa = g.target(e);
        }
    }//endfor
    return lfa;
  }//endif

  return INVALID;
}

bool Network::isLegacyLFAProtected_NP(const Node &s, const Node &d){
  if ( s != d ){
    Node nhop = nh(s, d);
    for (Graph::OutArcIt e(g, s); e != INVALID; ++e)
      //check all the neighbors except next-hop
      if ( g.target(e) != nhop ){
        //if d == next-hop, only LP condition is needed
        if ( d == nhop ){
          if ( distOpt(g.target(e), d) <
               distOpt(s, g.target(e)) + distOpt(s, d) )
            {
              return true;
            }
        }
        //if d != next-hop, LP + NP conditions should fulfill
        else if ( distOpt(g.target(e), d) <
                  distOpt(s, g.target(e)) + distOpt(s, d) &&
                  distOpt(g.target(e), d) <
                  distOpt(g.target(e), nhop) + distOpt(nhop, d) ){
              return true;
            }
      }
  }
  return false;
}

/*
 * It can be that virtual elements are
 * read from lgf file.
 */
void Network::countPhysicalElements(){
  int nodeCounter = 0;
  int arcCounter  = 0;

  for (Graph::NodeIt n(g); n != INVALID; ++n){
    if ((*nodeInfo)[n].isVirtual == false)
      nodeCounter++;
  }

  for (Graph::ArcIt e(g); e != INVALID; ++e){
    if ((*linkInfo)[e].isVirtual == false)
      arcCounter++;
  }

  numOfPhysicalNodes = nodeCounter;
  numOfPhysicalArcs  = arcCounter;
}

/*
 * It is the same as legacy method, the only
 * difference is the packet-tracer check test
 * Should be extended with NP case measuring later...
 */
double Network::newLfaCoverageMetric_LP(){
  int N = numOfPhysicalNodes;
  int I = -1;
  int summa = 0;
  double coverage = 0;

  for (Graph::NodeIt s(g); s != INVALID; ++s)
    for (Graph::NodeIt d(g); d != INVALID; ++d)
      if (  s != d &&
            (*nodeInfo)[s].isVirtual == false &&
            (*nodeInfo)[d].isVirtual == false   )
        {
          Node lfa  = isLegacyLFAProtected_LP(s, d);
          I = 0;
          if ( lfa != INVALID ){
            Node nHop = nh(s,d);
            Arc fail1 = findArc(g, s, nHop);
            Arc fail2 = findArc(g, nHop, s);
            LinkSet failLinks;
            failLinks.insert(fail1);
            failLinks.insert(fail2);

            packetTracer->haveValidPath(s, d, failLinks) ? I = 1 : I = 0;
          }
          summa += I;
      }//endif physical nodes

  coverage = (double)summa / (double)(N*(N-1));

  return coverage;
}

double Network::legacyLFACoverage(const LFAType &covType){
  int N = numOfPhysicalNodes;
  double coverage = 0;

  for (Graph::NodeIt s(g); s != INVALID; ++s)
    for (Graph::NodeIt d(g); d != INVALID; ++d){
      if ((*nodeInfo)[s].isVirtual == false &&
          (*nodeInfo)[d].isVirtual == false )
        switch ( covType ){
        case LP:
          if ( isLegacyLFAProtected_LP(s, d) != INVALID )
            coverage++;
          break;
        case NP:
          if ( isLegacyLFAProtected_NP(s, d) )
            coverage++;
          break;
        default:
          cout<<"ERROR: LFA coverage type is given badly!"<<endl;
          cout<<"0: link-protecting\n1: node-protecting"<<endl;
          return -1;
        }
    }
  return double(coverage)/double(N*(N-1));
}

double Network::getLegacyLFACoverage(const LFAType &covType){
  return legacyLFACoverage(covType);
}

Cost Network::longestShortestPath(){
  double lsp = 0;
  for (Graph::NodeIt s(g); s != INVALID; ++s)
    for (Graph::NodeIt d(g); d != INVALID; ++d){
      if ( distOpt(s, d) > lsp )
        lsp = distOpt(s, d);
    }
  return lsp;
}

//for being able to sort sPathSetToReturn
bool myCompare(const NodeSet &n1, const NodeSet &n2){
  return ( n1.size() < n2.size() );
}

/*
 * Build tunnels along shortest paths
 * including s and d.
 *
 * In an ascending order of set size
 */
void Network::buildPathSetOfNodes(const int &sizeOfSet ){
  vector<NodeSet>::iterator nVIt;

  //including single nodes
  for (Graph::NodeIt n(g); n != INVALID; ++n){
    NodeSet nS;
    nS.insert(n);
    virtCandidates.push_back(nS);
  }

  for (Graph::NodeIt s(g); s != INVALID; ++s){
    for (Graph::NodeIt d(g); d != INVALID; ++d){
      if ( s != d &&
           (*nodeInfo)[s].isVirtual == false &&
           (*nodeInfo)[d].isVirtual == false  ){
        NodeVector path;
        LinkSet empty;
        bool havePath = packetTracer->haveValidPath(s, d, empty);
        if ( !havePath ) {cerr<<"No shortest-path between "<<g.id(s)<<" and "<<g.id(d)<<"!Exit!"<<endl; exit(-1);}
        path = packetTracer->getPath();
        //packetTracer->printPrimaryPath();

        // For an unprotected (s,d) we need to have the tunnel
        // s-[p-q-d]. When creating the path we always need to have 'q'.
        // It can happen that 'q' is the last element in a shortest path,
        // thus when we collect shortest paths we always need to store the
        // last element.

        //destination is not included in the path
        path.push_back(d);

        if ( path.size() > 0 ){

          // Do not duplicate => check if not have vector already
          bool isInSet = false;
          for (nVIt = virtCandidates.begin(); nVIt != virtCandidates.end(); ++nVIt){
            if ( nVIt->size() == path.size() ){
              // create tmp vector from set that is already
              // inserted in order to be able to compare this
              // set with path
              NodeVector insertedSet(nVIt->begin(), nVIt->end());
              // to store result of comparison
              int size = path.size();
              NodeVector resultV(size);
              NodeVector::iterator it;

              std::sort (insertedSet.begin(), insertedSet.end());
              std::sort (path.begin(), path.end());

              it=std::set_difference (path.begin(), path.end(),
                                      insertedSet.begin(), insertedSet.end(), resultV.begin());

              resultV.resize(it-resultV.begin());
              if ( resultV.empty() ){ isInSet = true; }

            }
          }

          NodeSet vSet;
          copy(path.begin(), path.end(), inserter(vSet, vSet.begin()));

          if ( virtCandidates.size() == 0 || (!isInSet && vSet.size() <= sizeOfSet) ){
            virtCandidates.push_back(vSet);
          }
        }//endif size > 0 RD
      }
    }//endfor d
  }//endfor s

  //sorting
  std::sort(virtCandidates.begin(), virtCandidates.end(), myCompare);
}

void Network::printVirtCandidateNodeSets(){
  vector<NodeSet>::const_iterator i;
  NodeSet::const_iterator vIt;

  if (greedyH->gParamS.nSelStrategy == CONN_L){
    cout<<" CONNECTED-L-SETS: "<<endl;
  }
  else if (greedyH->gParamS.nSelStrategy == SPS){
    cout<<" SHORTEST-PATH-SLICE SETS: "<<endl;
  }

  for (i = virtCandidates.begin(); i != virtCandidates.end(); ++i){
    cout<<"(";
    for (vIt = i->begin(); vIt != i->end(); ++vIt){
      cout<<" "<<g.id(*vIt);
    }
    cout<<" )"<<endl;
  }
}

bool isNodeInSet(const NodeSet &w, const Node &n){
  if ( std::find(w.begin(), w.end(), n) != w.end() )
    return true;
  return false;
}

bool Network::isNeighOfSet(const NodeSet &w, const Node &n){
  NodeSet::const_iterator wIt;
  for (wIt = w.begin(); wIt != w.end(); ++wIt){
    if ( *wIt != n && neighbor(*wIt, n) )
      return true;
  }
  return false;
}

bool Network::isSetInVectorOfSet(const NodeSet &nSet, const vector<NodeSet> &vNS){
  vector<NodeSet>::const_iterator vNSIt;
  NodeVector nVect(nSet.begin(), nSet.end());  //set -> vector for nSet

  for (vNSIt = vNS.begin(); vNSIt != vNS.end(); ++vNSIt){
    NodeVector vNS_Element(vNSIt->begin(), vNSIt->end()); //set -> vector for *vNSIt

    // we will subtract the two vectors and check the result
    int size = nVect.size();
    NodeVector resultV(size);
    NodeVector::iterator it;

    std::sort (vNS_Element.begin(), vNS_Element.end());
    std::sort (nVect.begin(), nVect.end());

    it=std::set_difference (nVect.begin(), nVect.end(),
                            vNS_Element.begin(), vNS_Element.end(), resultV.begin());

    resultV.resize(it-resultV.begin());
    if ( resultV.empty() ){ return true; }

  }
  return false;
}

void Network::buildUniformSetOfNodes(const int &sizeOfSet){
  virtCandidates.clear();
  vector<NodeSet>::iterator wIt;
  NodeSet::iterator nIt;
  vector<NodeSet> W_prev;
  vector<NodeSet> W_next;
  NodeSet nSet;
  int k;

  // initially fill with 1 nodes
  for (Graph::NodeIt v(g); v != INVALID; ++v){
    //does this cond changes anything??
    if ( (*nodeInfo)[v].isVirtual == false ){
      NodeSet nSet;
      nSet.insert(v);
      W_prev.push_back(nSet);
      //virtCandidates.push_back(nSet);
    }
  }
  W_next = W_prev;

  for (k = 2; k <= sizeOfSet; ++k){
    for (wIt = W_prev.begin(); wIt != W_prev.end(); ++wIt){
      for (Graph::NodeIt n(g); n != INVALID; ++n){
        if ( ((*nodeInfo)[n].isVirtual == false) && !isNodeInSet(*wIt, n) )
          if ( isNeighOfSet(*wIt,n) ){
            nSet = *wIt;
            nSet.insert(n);
            if ( !isSetInVectorOfSet(nSet, W_next) ){
              W_next.push_back(nSet);
              //for (nIt=nSet.begin();nIt!=nSet.end(); ++nIt){
              // cout<<g.id(*nIt)<<" ";
              //}
              //cout<<endl;
            }
          }
      }
    }
    W_prev = W_next;
  }

  //sorting
  std::sort(W_next.begin(), W_next.end(), myCompare);
  virtCandidates = W_next;
}


/*
 * - Check if have loops in the augmented network
 * - Check the average path length
 */
bool Network::checkLoopAndAVPL(int &numOfLoopingSDs, const bool &toPrint) {
  numOfLoopingSDs = 0;
  woFaultAPL = 0;
  wFaultAPL  = 0;

  double sum_APL_NoFault         = 0;
  int    nOf_ProtectedSd_NoFault = 0;
  double sum_APL_Fault           = 0;
  int    nOf_ProtectedSd_Fault   = 0;
  for (Graph::NodeIt s(g); s != INVALID; ++s){
    for (Graph::NodeIt d(g); d != INVALID; ++d){
      if ( s != d &&
           (*nodeInfo)[s].isVirtual == false &&
           (*nodeInfo)[d].isVirtual == false    ){

        // empty set - no link failure
        LinkSet faulty;
        bool woFHavePath = packetTracer->haveValidPath(s, d, faulty);
        if ( woFHavePath ) {
          sum_APL_NoFault += packetTracer->path.size();
          nOf_ProtectedSd_NoFault++;
        }
        else {
          cerr<<"Error: No path between two nodes when no link failure!"<<endl;
          exit(-1);// exception handling would be better...
        }

        // simulate link failure
        Node nHop = nh(s, d);
        faulty.insert(findArc(g, s, nHop));
        faulty.insert(findArc(g, nHop, s));

        bool wFHavePath = packetTracer->haveValidPath(s, d, faulty);
        if ( toPrint ){
          packetTracer->printPrimaryPath();
        }

        // count only those who have path
        // Note, this function can be called even
        // if LFA coverage is not 1.
        if ( wFHavePath ){
          sum_APL_Fault += packetTracer->path.size();
          nOf_ProtectedSd_Fault++;
        }

        if (!wFHavePath && packetTracer->haveLoop == true){
          numOfLoopingSDs++;

          cerr<<"Error: Loop is added to the network! "<<endl;
          cerr<<"s: "<<g.id(s)<<" , d: "<<g.id(d)<<endl;
          cerr<<"EXIT!"<<endl;
          exit(-1);
        }
      }//end if
    }//end for d
  }//end for s

  if (nOf_ProtectedSd_NoFault > 0){
    woFaultAPL += ( sum_APL_NoFault / nOf_ProtectedSd_NoFault );
  }
  if ( nOf_ProtectedSd_Fault > 0){
    wFaultAPL  += ( sum_APL_Fault  / nOf_ProtectedSd_Fault );
    if ( toPrint ){
      cout<<"sum_APL_Fault: "<<sum_APL_Fault<<endl;
      cout<<"nOf_ProtectedSd_Fault: "<<nOf_ProtectedSd_Fault<<endl;
      cout<<"wFaultAPL: "<<wFaultAPL<<endl;
    }
  }

  if (numOfLoopingSDs > 0) return true;

  return false;
}

Network::~Network(){
  // Clean up the mess
  delete cost;
  delete db;
  delete packetTracer;
  delete linkInfo;
  delete nodeInfo;
  delete greedyH;

  virtCandidates.clear();
}
