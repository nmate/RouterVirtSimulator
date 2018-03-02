/*
 * Written by Mate Nagy
 * License: GNU General Public License Version 3
 *
 */

#include "GreedyILP.hh"

using namespace lemon;

GreedyILP::GreedyILP(Network *_nwP) {
  nwP = _nwP;
  g = &(nwP->g);
  nodeInfo = nwP->nodeInfo;
  linkInfo = nwP->linkInfo;
  greedyH = nwP->greedyH;

  xmlStepSet.clear();
}

bool operator<(SDGTriplet sdg1, SDGTriplet sdg2){
  if ( sdg1.s < sdg2.s )
    return true;
  if ( (sdg1.s == sdg2.s) && (sdg1.d < sdg2.d) )
    return true;
  if ( (sdg1.s == sdg2.s) && (sdg1.d == sdg2.d) && (sdg1.g < sdg2.g) )
    return true;

  return false;
}

bool operator==(SDGTriplet sdg1, SDGTriplet sdg2){
  return ( (sdg1.s == sdg2.s) &&
           (sdg1.d == sdg2.d) &&
           (sdg1.g == sdg2.g) );
}

VsVg GreedyILP::getVsVg(const NodeSet &nSet, const SDGTriplet &sdg, Cost &dist_VsVg){
  NodeSet::iterator vIt;
  VsVg w;
  w.v_s = INVALID;
  w.v_g = INVALID;

  // for each v' we have to prevent trap to be an escape
  // find out who is v_g (neigh of the trap)
  for (vIt = nSet.begin(); vIt != nSet.end(); ++vIt){
   if ( nwP->neighbor(*vIt, sdg.s) )   w.v_s = (*nodeInfo)[*vIt].physicalNode;
   if ( nwP->neighbor(*vIt, sdg.g) )   w.v_g = (*nodeInfo)[*vIt].physicalNode;
  }
  // One node is virtualized, no tunnel
  if ( nSet.size() == 1 ){
    dist_VsVg = 0;
  }
  else if ( w.v_s != INVALID && w.v_g != INVALID) {
    dist_VsVg = nwP->distOpt(w.v_s, w.v_g);
  }
  else {
    cout<<"ERROR: No distance available between v_s and v_g!"<<endl;
    cout<<"Node v_s: "<<nwP->g.id(w.v_s)<<" v_g: "<<nwP->g.id(w.v_g)<<
        " length of tunnel: "<<nSet.size()<<endl;
  }
  return w;
}

NodeSet GreedyILP::selectAndAddVirtualNode() {
  //GLPK/GUROBI Mixed Integer Program
  // #ifndef GUROBI
  //  GlpkMip mip;
  // #endif

  // #ifdef GUROBI
  //  GurobiMip mip;
  // #endif
  Mip mip;

  double bestRelSolValue = -1000;

  //for storing the best selection
  NodeSet vWinner;
  INodeMap xWinner(*g, 0);
  DNodeMap cWinner(*g, 0);
  vector<NodeSet>::iterator vCIt;
  NodeSet::iterator nSIt;

  //nwP->printVirtCandidateNodeSets();

  //for all the NodeSets
  for (vCIt = nwP->virtCandidates.begin(); vCIt != nwP->virtCandidates.end(); ++vCIt){

    debug(LOG)<<"==============================="<<endl;
    debug(LOG)<<"    ILP 1-step result printout"<<endl;
    debug(LOG)<<"    NODE: ";
    for (nSIt = vCIt->begin(); nSIt != vCIt->end(); ++nSIt){
      debug(LOG)<<nwP->g.id(*nSIt)<<" ";
    }
    debug(LOG)<<endl;
    debug(LOG)<<"==============================="<<endl;

    //clean than build sets
    greedyH->cleanSets();
    greedyH->makeLSet(*vCIt);
    greedyH->makeESet(*vCIt);
    greedyH->makeQSet(*vCIt);
    greedyH->makeTSet(*vCIt);

    //Debug
    //greedyH->printUnprotectedSDSet();
    //greedyH->printESet();
    //greedyH->printTSet();

    UnprotectedSDSet *lSetP = &(greedyH->L);
    map<SDPair, NodeSet> *eSetP = &(greedyH->E);
    map<SDPair, NodeSet> *tSetP = &(greedyH->T);

    UnprotSDIt lIt;
    ET_Iterator eIt;
    ET_Iterator tIt;

    /*************************************************
     * ADD_COL: x for all the neighbors
     * x_n : 1 if create link between v',n
     */
    debug(DEBUG1)<<"Adding column: x."<<endl;
    Graph::NodeMap<Mip::Col> x(*g, INVALID);
    for (Graph::NodeIt n(*g); n != INVALID; ++n){
      //if ( nwP->neighbor(n, root) ){
        x[n] = mip.addCol();

        mip.colType(x[n], Mip::INTEGER);
        mip.colBounds(x[n], 0, 1);
        mip.colName(x[n], "x[" + name(n) + "]");
      //}
    }

    /*************************************************
     * ADD_COL: y_sd for all the unprotected sd's
     * y_sd: 1 if v' gives protection to s-d
     * yMap is needed to be able to access y_sd variables later
     */
    debug(DEBUG1)<<"Adding column: y."<<endl;
    map<SDPair, Mip::Col> yMap;
    for (lIt = lSetP->begin(); lIt != lSetP->end(); ++lIt) {
      pair<SDPair, Mip::Col> y_sd;
      y_sd.first = *lIt;
      y_sd.second = mip.addCol();
      yMap.insert(y_sd);

      mip.colType(yMap.at(*lIt), Mip::INTEGER);
      mip.colBounds(yMap.at(*lIt), 0, 1);
    }

    /*************************************************
     * ADD_COL: zMap with z_gsd variables
     * z_gsd: 1 if g is next-hop(v',d) where s,d \in L, g \in E
     */
    debug(DEBUG1)<<"Adding column: z."<<endl;
    map<SDGTriplet, Mip::Col> zMap;
    for (eIt = eSetP->begin(); eIt != eSetP->end(); ++eIt) {
      for (nSIt = eIt->second.begin(); nSIt != eIt->second.end(); ++nSIt){
        SDGTriplet sdg;
        sdg.s = eIt->first.source;
        sdg.d = eIt->first.destination;
        sdg.g = *nSIt;

        pair<SDGTriplet, Mip::Col> z_gsd;
        z_gsd.first = sdg;
        z_gsd.second = mip.addCol();

        if ( zMap.insert(z_gsd).second == false) {
          cout<<"ERROR: Insertion of element failed."<<endl;
        }

        mip.colType(z_gsd.second, Mip::INTEGER);
        mip.colBounds(z_gsd.second, 0, 1);
      }
    }

    /*************************************************
     * ADD_COL: delta_d variables
     * delta_d: cost of v'-d distance
     *
     * UPDATE: condition needs to be commented out
     * if we use tunnels.
     */
    debug(DEBUG1)<<"Adding column: delta."<<endl;
    Graph::NodeMap<Mip::Col> delta_d(*g, INVALID);
    for (Graph::NodeIt d(*g); d != INVALID; ++d){
      //if ( find(vCIt->begin(), vCIt->end(), d) == vCIt->end() ){
        delta_d[d] = mip.addCol();

        mip.colType(delta_d[d], Mip::REAL);
        mip.colLowerBound(delta_d[d], 0);
      //}
    }

    /*************************************************
     * ADD_COL: c variables
     * c: real, cost of the virtual link (v',n)
     */
    debug(DEBUG1)<<"Adding column: c."<<endl;
    Graph::NodeMap<Mip::Col> c(*g, INVALID);
    for (Graph::NodeIt n(*g); n != INVALID; ++n){
      //if ( nwP->neighbor(n, root) ){
        c[n] = mip.addCol();

        mip.colType(c[n], Mip::REAL);
        mip.colLowerBound(c[n], 0);
        //}
    }

    /*************************************************
     * ADD_ROW: (6) y_sd <= x_s
     */
    debug(DEBUG1)<<"Adding row: 6a."<<endl;
    for (lIt = lSetP->begin(); lIt != lSetP->end(); ++lIt) {
      Node s = lIt->source;

      mip.addRow(yMap[*lIt] <= x[s]);
    }

    /*************************************************
     * ADD_ROW: (6) z_gsd <= x_g
     */
    debug(DEBUG1)<<"Adding row: 6b."<<endl;
    for (eIt = eSetP->begin(); eIt != eSetP->end(); ++eIt) {
      for (nSIt = eIt->second.begin(); nSIt != eIt->second.end(); ++nSIt){
        SDGTriplet sdg;
        sdg.s = eIt->first.source;
        sdg.d = eIt->first.destination;
        sdg.g = *nSIt;

        mip.addRow(zMap[sdg] <= x[*nSIt]);
      }
    }

    /*************************************************
     * ADD_ROW: (7) y_sd <= summa z_gsd
     */
    debug(DEBUG1)<<"Adding row: 7."<<endl;
    for (eIt = eSetP->begin(); eIt != eSetP->end(); ++eIt) {
      Mip::Expr eq7;
      for (nSIt = eIt->second.begin(); nSIt != eIt->second.end(); ++nSIt){
        SDGTriplet sdg;
        sdg.s = eIt->first.source;
        sdg.d = eIt->first.destination;
        sdg.g = *nSIt;
        eq7 += zMap[sdg];
      }
      SDPair sd;
      sd.source = eIt->first.source;
      sd.destination = eIt->first.destination;

      mip.addRow(yMap[sd] <= eq7);
    }

    /*************************************************
     * ADD_ROW: (8) delta_d + K(1 - z_gsd) >= dist(g,d) + c_g
     * UPDATE 2014-05-25: if v' is a set, condition needs to be modified.
     * We also have to consider the distance between v'_g and v'_s.
     * v_s denotes the node that is connected to s and similarly v_g is
     * the one that is connected to g.
     */
    debug(DEBUG1)<<"Adding row: 8."<<endl;
    Cost K = 2*(nwP->longestShortestPath() + 1);
    debug(DEBUG1)<<"Value of K: "<<K<<endl;
    for (eIt = eSetP->begin(); eIt != eSetP->end(); ++eIt) {
      for (nSIt = eIt->second.begin(); nSIt != eIt->second.end(); ++nSIt){
        SDGTriplet sdg;
        sdg.s = eIt->first.source;
        sdg.d = eIt->first.destination;
        sdg.g = *nSIt;

        SDPair sd;
        sd.source = sdg.s;
        sd.destination = sdg.d;

        Cost dist_VsVg = 0;
        getVsVg(*vCIt, sdg, dist_VsVg);

        //mip.addRow(delta_d[sdg.d] + K*(1-zMap[sdg]) >= nwP->distOpt(sdg.g, sdg.d)
        //    + nwP->distUni(v_s, v_g) + c[sdg.g]);

        //mip.addRow(deltaMap[sd] + K*(1-zMap[sdg]) >= nwP->distOpt(sdg.g, sdg.d)
        //      + dist_VsVg + c[sdg.g]);

        mip.addRow(delta_d[sdg.d] + K*(1-zMap[sdg]) >= nwP->distOpt(sdg.g, sdg.d)
                + dist_VsVg + c[sdg.g]);

      }
    }

    /*************************************************
     * ADD_ROW: (9) delta_d + C <= dist(g,d) + c_g + K*(1-x_s) + K*(1-x_g)
     * UPDATE: 2014-06-25: if v' is a set, condition needs to be modified.
     * UPDATE: 2014-06-26: v' is not always neighbour of s from Q set!
     *
     * g \in T, s,d \in Q
     */
    debug(DEBUG1)<<"Adding row: 9."<<endl;
    int C = 1;
    for (tIt = tSetP->begin(); tIt != tSetP->end(); ++tIt) {
      for (nSIt = tIt->second.begin(); nSIt != tIt->second.end(); ++nSIt){
        SDGTriplet sdg;
        sdg.s = tIt->first.source;
        sdg.d = tIt->first.destination;
        sdg.g = *nSIt;

        Node trap = sdg.g;

        /* for deltaMap */
        SDPair sd;
        sd.source = sdg.s;
        sd.destination = sdg.d;

        Cost dist_VsVg;
        getVsVg(*vCIt, sdg, dist_VsVg);

        // create equation for each v' that is not v_g
        //for (vIt = vCIt->begin(); vIt != vCIt->end(); ++vIt){
         //   v_s = (*nodeInfo)[*vIt].physicalNode;
            //mip.addRow( delta_d[d] + C <= nwP->distOpt(trap,d) + nwP->distUni(v_s, v_g)
            //    + c[trap]+ K*(1 -  x[s]) + K*(1 - x[trap]) /*+ K*(1-isHiddenUpstr)*/ );

            //mip.addRow( deltaMap[sd] + C <= nwP->distOpt(trap,d) + dist_VsVg
            //                + c[trap]+ K*(1 -  x[s]) + K*(1 - x[trap]) );

        	mip.addRow( delta_d[sdg.d] + dist_VsVg + C <= nwP->distOpt(trap,sdg.d) +
                               c[trap]+ K*(1 -  x[sdg.s]) + K*(1 - x[trap]) );

        //}
      }
    }

    /*************************************************
     * ADD_ROW: (10) c_n >= c_s(v,n) + C
     */
    debug(DEBUG1)<<"Adding row: 10."<<endl;
    for (Graph::NodeIt n(*g); n != INVALID; ++n){
      for (nSIt = vCIt->begin(); nSIt != vCIt->end(); ++nSIt)
       if ( nwP->neighbor(n, *nSIt) ){
        Arc arcRN = findArc(nwP->g, *nSIt, n);
        Cost costRootN = (*nwP->cost)[arcRN];

        mip.addRow(c[n] >= costRootN + C);
      }
    }

    /*************************************************
     * ADD_ROW: (new) delta_d[d] <= c_n + dist(n,d)
     *          where n \in neigh(v) and d \in E set
     *
     * All virtual links that are not trap should not have
     * less cost than the escape node. Because otherwise it can happen that
     * a non-escape and non-trap node's link cost will be shorter
     * than the path going through the escape.
     */
    /*debug(DEBUG1)<<"Adding row: new."<<endl;
    for (eIt = eSetP->begin(); eIt != eSetP->end(); ++eIt) {
      Node d = eIt->first.destination;
      for (Graph::NodeIt n(*g); n != INVALID; ++n){
       NodeSet::iterator vIt;
       for (vIt = vCIt->begin(); vIt != vCIt->end(); ++vIt){
        if ( nwP->neighbor(n, *vIt) ){

         mip.addRow(delta_d[d] <= c[n] + nwP->distOpt(n,d));
    	}
      }
     }
    }*/

    /*************************************************
     * ADD_ROW: (new 2) if c_s + dist(s,d) <= delta_d[d]
     *          for each g \in E: z_gsd = 0
     *
     * If I want to select g as an escape to s-d (z_gsd=1),
     * than I need to ensure that path going through s will not
     * be shorter than the path going through the escape
     *
     * path through the escape: delta_d
     * path through s: c_s + dist(s,d)
     */
    debug(DEBUG1)<<"Adding row: new 2."<<endl;
    for (eIt = eSetP->begin(); eIt != eSetP->end(); ++eIt) {
      for (nSIt = eIt->second.begin(); nSIt != eIt->second.end(); ++nSIt){
       SDGTriplet sdg;
       sdg.s = eIt->first.source;
       sdg.d = eIt->first.destination;
       sdg.g = *nSIt;

       Cost dist_VsVg;
       getVsVg(*vCIt, sdg, dist_VsVg);

       mip.addRow( delta_d[sdg.d] + dist_VsVg + C <= c[sdg.s] + nwP->distOpt(sdg.s,sdg.d) + K*(1 - zMap[sdg]) );
     }
    }

    /*************************************************
     *  ADD_ROW: (new 3)
     *  [s_1, d_1]->e_1
     *  [s_2, d_2]->e_2
     *
     *  delta_d_1 + C <= c[e_2] + dist(e_2, s_1) + dist(s_1, d_1) + K*(1-z[s_1,d_1,e_1])
     *
     * If e_1 is selected as an escape to d_1, than
     * ensure that path through other escape nodes (e_2) won't
     * include s_1. In other words it should be true that
     * the any path is going through some other escape's
     * source node.
     */
    ET_Iterator fIt;
    NodeSet::iterator mSIt;
    debug(DEBUG1)<<"Adding row: new 3."<<endl;
    for (eIt = eSetP->begin(); eIt != eSetP->end(); ++eIt) {
      for (nSIt = eIt->second.begin(); nSIt != eIt->second.end(); ++nSIt){
        SDGTriplet sdg;
        sdg.s = eIt->first.source;
        sdg.d = eIt->first.destination;
        sdg.g = *nSIt;

        Cost dist_Vs1Vg1 = 0;
        VsVg w1 = getVsVg(*vCIt, sdg, dist_Vs1Vg1);

        for (fIt = eSetP->begin(); fIt != eSetP->end(); ++fIt) {
          if (eIt != fIt)
           for (mSIt = fIt->second.begin(); mSIt != fIt->second.end(); ++mSIt){
            SDGTriplet sde;
            sde.s = fIt->first.source;
            sde.d = fIt->first.destination;
            sde.g = *mSIt;

            Cost dist_Vs2Vg2 = 0;
            VsVg w2 = getVsVg(*vCIt, sdg, dist_Vs2Vg2);

            if ( sde.g != sdg.g ){
              mip.addRow( delta_d[sdg.d] + dist_Vs1Vg1 + C <= c[sde.g] + nwP->distOpt(w1.v_s, w2.v_g) +
                  nwP->distOpt(sde.g, sdg.s) + nwP->distOpt(sdg.s,sdg.d) + K*(1 - zMap[sdg]) );
            }
          }
        }

      }
    }
    /*************************************************
     *  ADD_ROW: (new 4)
     *  (s_1,d_1) is unprotected and there is no escape
     *  only trap nodes for (s_1,d_1) and V'.
     *  [s_1, d_1]-> t_1
     *
     *  The only constraint is the trap constraint above
     *  but it does not contain lower bound for delta_d
     *  so it will be set to 0 and the trap can eventually
     *  act as an escape with not properly chosen c_g.
     *
     *  So if only trap nodes are in V', let delta_d
     *  (the shortest path from V' to d) be going through
     *  's' => it will be not selected as LFA, will not
     *  be considered as protected.
     */
    for (tIt = tSetP->begin(); tIt != tSetP->end(); ++tIt) {
      Node s = tIt->first.source;
      Node d = tIt->first.destination;
      ET_Iterator escOfSDIt = eSetP->find(tIt->first);

      // (s,d) is in E but there are no escape belonging to it:
      if ( (escOfSDIt != eSetP->end()) && (escOfSDIt->second.size() == 0)){
    	  mip.addRow( delta_d[d] >= c[s] + nwP->distOpt(s,d) + C);
      }
    }


    /*************************************************
     * Here comes the objective function
     *
     * (10) max ( summa [y_sd] - epsilon*summa[c_n + x_n] )
     */
    debug(DEBUG1)<<"Adding objective function."<<endl;
    mip.max();

    Mip::Expr exp;
    Cost epsilon = 0.00005;

    for (lIt = lSetP->begin(); lIt != lSetP->end(); ++lIt) {
     exp += yMap.at(*lIt);
    }
    Mip::Expr exp_latter;
    for (Graph::NodeIt n(*g); n != INVALID; ++n){
      //for (nSIt = vCIt->begin(); nSIt != vCIt->end(); ++nSIt)
        //if ( nwP->neighbor(n, *nSIt) ){
          //exp2 += (c[n] * x[n]);
    	  exp_latter += c[n] + x[n];
      //}
    }
    exp -= epsilon*exp_latter;

    mip.obj(exp);
    if(debug.level() >= 3)
        mip.messageLevel(Mip::MESSAGE_VERBOSE);

    debug(LOG)<<"Solve mip with #of virtual nodes: "<<vCIt->size()<<endl;
    mip.solve();

    /*************************************************
     * Debug printout
     */
    if (mip.type() == Mip::OPTIMAL) {
      debug(LOG) << "Size of objective function value: " << mip.solValue() << endl;
    }
    for (Graph::NodeIt n(*g); n != INVALID; ++n){
      for (nSIt = vCIt->begin(); nSIt != vCIt->end(); ++nSIt)
        if ( nwP->neighbor(n, *nSIt) ){
          debug(LOG)<< " x["<<nwP->g.id(n)<<"]: "<<mip.sol(x[n])<<endl;
          debug(LOG)<< " c["<<nwP->g.id(n)<<"]: "<<mip.sol(c[n])<<endl;
      }
    }

    for (eIt = eSetP->begin(); eIt != eSetP->end(); ++eIt) {
     SDPair sd;
     sd.source = eIt->first.source;
     sd.destination = eIt->first.destination;

     debug(LOG)<<" y["<<nwP->g.id(sd.source)<<","<<nwP->g.id(sd.destination)<<
         "]: "<<mip.sol(yMap.at(sd))<<endl;

     for (nSIt = eIt->second.begin(); nSIt != eIt->second.end(); ++nSIt){
       SDGTriplet sdg;
       sdg.s = eIt->first.source;
       sdg.d = eIt->first.destination;
       sdg.g = *nSIt;

       debug(LOG)<<" z["<<nwP->g.id(sdg.s)<<","<<nwP->g.id(sdg.d)<<","<<nwP->g.id(sdg.g)<<
           "]: "<<mip.sol(zMap.at(sdg))<<endl;
     }
    }

    for (Graph::NodeIt d(*g); d != INVALID; ++d){
        if ( find(vCIt->begin(), vCIt->end(), d) == vCIt->end() ){
          debug(LOG)<<" delta["<<nwP->g.id(d)<<"]:"<<mip.sol(delta_d[d])<<endl;
        }
    }

    /*************************************************
     * Storing the best ILP result
     * Use relative metric for goodness. If 2 nodes
     * does not improve at least twice as 1 node,
     * it is not better than 1 node solution.
     */
    //update the winner
    double currRelSolValue = mip.solValue() / double(vCIt->size());
    if ( (currRelSolValue > bestRelSolValue) &&
         (fabs( currRelSolValue - bestRelSolValue ) > 0.001) &&
          mip.type() == Mip::OPTIMAL  )
    {
      debug(LOG)<<"Storing the best ILP result..."<<endl;
      vWinner.clear();
      vWinner = *vCIt;
      //save x items to xWinner, and similarly c to cWinner
      for (Graph::NodeIt node(*g); node != INVALID; ++node){
        xWinner[node] = mip.sol(x[node]);
        cWinner[node] = mip.sol(c[node]);
      }
      bestRelSolValue = currRelSolValue;
    }//end of update
  }//end for all the nodes in Gs

  //Debug: print out best selection
  debug(INFO)<<"==============================="<<endl;
  debug(INFO)<<" Best selection"<<endl;
  debug(INFO)<<"  Obj function value: "<<bestRelSolValue<<endl;

  NodeSet::iterator wIt;
  for (wIt = vWinner.begin(); wIt != vWinner.end(); ++wIt){
   debug(INFO)<<"  Node To Virtualize: "<<nwP->g.id(*wIt)<<endl;
   for (Graph::NodeIt n(*g); n != INVALID; ++n){
    if ( nwP->neighbor(n, *wIt) ){
       debug(INFO)<< "  x["<<nwP->g.id(n)<<"]: "<<xWinner[n]<<endl;
       debug(INFO)<< "  c["<<nwP->g.id(n)<<"]: "<<cWinner[n]<<endl;
    }
   }
  }

  debug(INFO)<<"  Current LFA coverage: "<<nwP->newLfaCoverageMetric_LP()<<endl;

  /*************************************************
   * Adding the best ILP selected node (if exists) to the
   * topology
   */
  NodeSet virtualNodes;
  virtualNodes = realizeVirtualNode(vWinner, xWinner, cWinner);

  debug(INFO)<<"  New LFA coverage: "<<nwP->newLfaCoverageMetric_LP()<<endl;
  greedyH->printUnprotectedSDSet();

  return virtualNodes;
}

NodeSet GreedyILP::realizeVirtualNode(const NodeSet& vS, const INodeMap &xWin, const DNodeMap &cWin){
  NodeSet virtualNodes;
  NodeSet::iterator nSIt;

  if ( vS.size() == 0 ){
    cout<<"No vWinner is selected by ILP...finish"<<endl;
    return virtualNodes;
  }

  for (nSIt = vS.begin(); nSIt != vS.end(); ++nSIt){
     Node virtualV = nwP->g.addNode();
     Node phyV = (*nodeInfo)[*nSIt].physicalNode;
     virtualNodes.insert(virtualV);
     (*nodeInfo)[virtualV].isVirtual    = true;
     (*nodeInfo)[virtualV].physicalNode = phyV;
     for (Graph::NodeIt n(*g); n != INVALID; ++n){
       Node phyNeigh = (*nodeInfo)[n].physicalNode;
       if ( xWin[n] == 1 &&
            nwP->neighbor(phyNeigh, phyV) ){
         Arc virtArc1 = nwP->g.addArc(virtualV, n);
         Arc virtArc2 = nwP->g.addArc(n, virtualV);
         (*(nwP->cost))[virtArc1] = cWin[n];
         (*(nwP->cost))[virtArc2] = cWin[n];

         (*linkInfo)[virtArc1].isVirtual = true;
         Arc phyArc1 = findArc(*g, phyV, phyNeigh);
         (*linkInfo)[virtArc1].physicalLink = phyArc1;

         (*linkInfo)[virtArc2].isVirtual = true;
         Arc phyArc2 = findArc(*g, phyNeigh, phyV);
         (*linkInfo)[virtArc2].physicalLink = phyArc2;
       }
     }
  }

  // Connect virtual nodes
  NodeSet::iterator vSIt;
  NodeSet::iterator neighVSIt;
  for (vSIt = virtualNodes.begin(); vSIt != virtualNodes.end(); ++vSIt){
    Node phyV = (*nodeInfo)[*vSIt].physicalNode;
    for (neighVSIt = virtualNodes.begin(); neighVSIt != virtualNodes.end(); ++neighVSIt){
      if ( *vSIt < *neighVSIt ){
        Node phyNeighV = (*nodeInfo)[*neighVSIt].physicalNode;
        if ( nwP->neighbor(phyV, phyNeighV) ){
          // find the physical arcs
          Arc phyArc1 = findArc(*g, phyV, phyNeighV);
          Arc phyArc2 = findArc(*g, phyNeighV, phyV);
          Arc lowCostArc1 = g->addArc(*vSIt, *neighVSIt);
          Arc lowCostArc2 = g->addArc(*neighVSIt, *vSIt);
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

  //update databases
  nwP->db->initializeDB();
  greedyH->fillNodeInfoLocalSrgTables();
  greedyH->buildUnprotectedSDSet();

  return virtualNodes;
}

void GreedyILP::callILPGreedily(const double &ratio, const int &maxNodeSetSize){
  double currLFACoverage = 0; //dummy
  double nextLFACoverage = 5;  //dummy
  /*int allowedVirtualNodes = ratio*(nwP->numOfPhysicalNodes);*/
  int numOfVirtualNodes = 0;

  // will store newly added virtual node, for init
  // we need to set it a dummy valid one
  NodeSet addedNodes;
  addedNodes.insert(nwP->g.nodeFromId(0));
 
  Timer t;

  // Create node sets to virt
  nwP->calculateNodeSets(maxNodeSetSize);

  /*
   * We try to improve with 1 virtual node/step
   *
   * Runs until
   *  1.: (#virtual nodes) / (#physical nodes) != ratio
   *  2.: we can improve LFA coverage >>nextLFACoverage > currLFACoverage<<
   *  (3.: if coverage = 1)
   */

  t.restart();
  while ( /*(numOfVirtualNodes <= allowedVirtualNodes) &&*/
           !( greedyH->unprotSDs.empty() && nextLFACoverage > currLFACoverage ) )
  {
    currLFACoverage = nwP->newLfaCoverageMetric_LP();
    addedNodes = selectAndAddVirtualNode();
    if ( !addedNodes.empty() ) numOfVirtualNodes += addedNodes.size();
    nextLFACoverage = nwP->newLfaCoverageMetric_LP();

    if ( nwP->equal(nextLFACoverage, currLFACoverage) ||
         nextLFACoverage < currLFACoverage ){

      // Rollback
      cout<<"LFA coverage did not improve..."<<endl;
      cout<<"Removing last node(s)...exit"<<endl;
      greedyH->rollbackVirtualNode(addedNodes);
      numOfVirtualNodes -= addedNodes.size();

      debug(INFO)<<"INFO: Virtualization stuck...exit."<<endl;
      break;
    }
    else {
      // store for xml output
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
  }
  t.stop();
}



