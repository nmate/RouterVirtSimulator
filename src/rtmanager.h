/* This class builds redundant trees on top
 * of the physical network. The algorithm was
 * proposed by Gabor Enyedi.
 *
 * Written by Mate Nagy
 * License: GNU General Public License Version 3
 */
#ifndef RTMANAGER_H
#define RTMANAGER_H

#include <vector>
#include <stack>
#include <algorithm> //std::find

#include <lemon/adaptors.h>
#include <lemon/bfs.h>
#include <lemon/hao_orlin.h>

using namespace std;

namespace lemon {

  template <typename GR>
    class RedundantTree {
  public:
    /* type definitions */
    typedef ListDigraph Graph;
    typedef Graph::Node Node;
    typedef Graph::Arc  Arc;
    typedef vector<Node> NodeVector;
    typedef Bfs<SubDigraph<const Graph> > BfsSubType;
    typedef Graph::NodeMap<bool> VisitedMap;
    typedef Graph::ArcMap<int> CostMap; //for connectivity check
    typedef SplitNodes<const Graph> SplitGraph; //for vertex connectivity
    typedef SplitGraph::ArcMap<int> SplitCostMap;

  private:
    /* needed data structure definitions */
    struct RTParameters {
      int DFSNum;
      int lowpointNum;
      Node lpSource; //node from where it obtained its lowpoint
      NodeVector myChild; //children in the DFS tree
    };

    //DFS and lowpoint data (for one dedicated root node)
    typedef Graph::NodeMap<RTParameters> RTDataMap;

    /* Member variables */
    const GR *g;

    // needed for storing data
    RTDataMap *rtDataMap;
    int DFSOrderCnt;

    //needed for ADAG
    Graph::ArcMap<bool> *arcFilterADAG;

    //2 redundant trees:
    Graph::ArcMap<bool> *arcFilterRED;
    Graph::ArcMap<bool> *arcFilterBLUE;

    //edge connectivity check
    int edgeConnValue;
    bool biEdgeConn;
    bool biNodeConn;

  public:
    /***************************************
     * Constructor, we only need the graph
     ***************************************/
    RedundantTree (const GR *_graph){
        g = _graph;

        DFSOrderCnt = 0;
        rtDataMap = new RTDataMap(*g);

        arcFilterADAG = new Graph::ArcMap<bool>(*g, false);

        arcFilterRED  = new Graph::ArcMap<bool>(*g, false);
        arcFilterBLUE = new Graph::ArcMap<bool>(*g, false);

        edgeConnValue = setEdgeConnectivityValue();
        checkGraphConnectivity();
      }

    /***************************************
     * allVisited() - checks if have node
     * that is not visited yet in DFS
     ***************************************/
    bool allVisited(VisitedMap &vMap){
      for (Graph::NodeIt n(*g); n != INVALID; ++n){
        if ( vMap[n] == false ) return false;
      }
      return true;
    }

    /***************************************
     * getChildWithLowestLP() - needed for
     * lowpoint number calculation
     ***************************************/
    Node getChildWithLowestLP(const Node &currNode){
      Node child = INVALID;
      if ( (*rtDataMap)[currNode].myChild.size() != 0 ){
        NodeVector::iterator it = (*rtDataMap)[currNode].myChild.begin();

        int minLP = (*rtDataMap)[*it].lowpointNum;
        while ( it != (*rtDataMap)[currNode].myChild.end() ){
          if ( (*rtDataMap)[*it].lowpointNum <= minLP ){
            minLP = (*rtDataMap)[*it].lowpointNum;
            child = *it;
          }
          it++;
        }
        return child;
      }
      return INVALID;
    }

    /***************************************
     * getMinDfsNumOfNeighs() - needed for
     * lowpoint number calculation
     ***************************************/
    pair<Node, int> getMinDfsNumOfNeighs(const Node &currNode){
      int min = 10000;
      Node source = INVALID;
      for (Graph::OutArcIt e(*g, currNode); e != INVALID; ++e){
        Node neighbour = g->target(e);
        if ( (*rtDataMap)[neighbour].DFSNum < min ) {
          min = (*rtDataMap)[neighbour].DFSNum;
          source = neighbour;
        }
      }
      return make_pair(source, min);
    }

    /***************************************
     * setLowpointNumber() - figures out
     * the lowpoint number and stores it
     * in RTDataMap
     ***************************************/
    void setLowpointNumber(const Node &root, const Node &currNode){
      //root node does not have lowpoint num;
      if ( root == currNode ){
        (*rtDataMap)[root].lowpointNum = -1;
        (*rtDataMap)[root].lpSource = INVALID;
        return;
      }

      //do I have child?
      Node child = getChildWithLowestLP(currNode);

      //neighbour with lowest DFS:
      Node fromDfs = getMinDfsNumOfNeighs(currNode).first;
      int minDfs = getMinDfsNumOfNeighs(currNode).second;

      // I am leaf => my LP is min(DFS of my neighbours)
      if ( child == INVALID ) {
        (*rtDataMap)[currNode].lowpointNum = minDfs;
        (*rtDataMap)[currNode].lpSource    = fromDfs;
      }
      // I am not leaf => my LP is min(child LP && neigh's DFS)
      else {
        int childLP = (*rtDataMap)[child].lowpointNum;

        if ( childLP <= minDfs ) {
          (*rtDataMap)[currNode].lowpointNum = childLP;
          (*rtDataMap)[currNode].lpSource = child;
        }
        else {
          (*rtDataMap)[currNode].lowpointNum = minDfs;
          (*rtDataMap)[currNode].lpSource    = fromDfs;
        }
      }
    }

    /***************************************
     * selectNextNode - needed for DFS
     ***************************************/
    Node selectNextNode(Node currNode, VisitedMap &visitedMap,
                        NodeVector &stack){
      for (Graph::OutArcIt e(*g, currNode); e != INVALID; ++e){
        Node neighbour = g->target(e);
        if ( visitedMap[neighbour] == false ){
          (*rtDataMap)[currNode].myChild.push_back(neighbour);
          return neighbour;
        }
      }
      // we are on a leaf, have to jump back to
      // a parent that has not visited children
      stack.pop_back();
      Node parent = stack.back();
      Node selectedParent = selectNextNode(parent, visitedMap, stack);

      return selectedParent;
    }

    /***************************************
     * jumpToNextNode - needed for DFS
     * stepping node by node recursively
     ***************************************/
    void jumpToNextNode(Node currNode,
                        VisitedMap &visitedMap, NodeVector &stack, NodeVector &visitOrder) {

      //mark that we have been here
      visitedMap[currNode] = true;
      stack.push_back(currNode);

      //fill Parameter field
      (*rtDataMap)[currNode].DFSNum = DFSOrderCnt;
      DFSOrderCnt++;
      visitOrder.push_back(currNode);

      //exit condition
      if ( allVisited(visitedMap) ) return;

      currNode = selectNextNode(currNode, visitedMap, stack);

      if ( currNode == INVALID ){
        cerr<<"INVALID node is selected!"<<endl;
        return;
      }

      jumpToNextNode(currNode, visitedMap, stack, visitOrder);

    }

    /***************************************
     * fillDfsAndLowPoint()
     *
     * Own implementation of DFS because LEMON's DFS class does not
     * calculate lowpoint numbers, and moreover I was not able to
     * figure out how to obtain DFS order after calling dfs.run().
     ***************************************/
    void fillDfsAndLowPoint(const Node &root){
      VisitedMap visited(*g, false);
      NodeVector stack;
      NodeVector visitOrder;
      DFSOrderCnt = 0;

      //initialize data structure
      RTParameters rtInit = {-1,-1};

      //set DFS numbers, starting from root
      jumpToNextNode(root, visited, stack, visitOrder);

      //set lowpoint numbers on reverse order of
      //visitOrder vector
      NodeVector::reverse_iterator rIt;
      for (rIt=visitOrder.rbegin(); rIt != visitOrder.rend(); ++rIt){
        setLowpointNumber(root, *rIt);
      }
    }

    /***************************************
     * neighbor() - returns true if s and d
     * are neighbours in g
     ***************************************/
    bool neighbor(Graph::Node s, Graph::Node d) const{
      bool n = false;
      for (Graph::OutArcIt e(*g, s); e!= INVALID; ++e)
        if ( d == g->target(e) )
          n = true;
      return n;
    }

    /***************************************
     * parentInDfs - needed for ADAG calculation
     *
     * If I am in the child list of my parent,
     * then I am its child.
     ***************************************/
    Node parentInDfs(const Node &n){
      int dfsOfn = (*rtDataMap)[n].DFSNum;

      for (Graph::NodeIt p(*g); p != INVALID; ++p){
        if ( (*rtDataMap)[p].DFSNum < dfsOfn ){
          NodeVector childrenOfP = (*rtDataMap)[p].myChild;
          NodeVector::iterator cnIt;
          for (cnIt=childrenOfP.begin(); cnIt != childrenOfP.end(); ++cnIt){
            if (*cnIt == n)
              return p;
          }
        }
      }
      return INVALID;
    }

    /***********************************
     * setupADAG
     ***********************************/
    void setupADAG(const Node &root){
      //cout<<"=================================="<<endl;
      //cout<<"Start calculating ADAG for "<<g->id(root)<<endl;
      //cout<<"=================================="<<endl;
      VisitedMap ready(*g, false);
      stack<Node> stack;
      NodeVector::iterator nLIt;

      stack.push(root);
      ready[root] = true;
      while ( stack.size() != 0 ){
        Node current = stack.top(); //take first
        stack.pop();                //remove first

        NodeVector earVector; //store nodes in ear
        int dfsOfCurr = (*rtDataMap)[current].DFSNum;
        NodeVector children = (*rtDataMap)[current].myChild;

        //cout<<"pop stack <-- Current: "<<g->id(current)<<endl;

        /********************************************************
         * for each child 'n' of current
         ********************************************************/
        earVector.clear();
        for (nLIt=children.begin(); nLIt != children.end(); ++nLIt){
          Node n = *nLIt;
          if ( ready[n] == false ){
            earVector.push_back(n);
            while ( ready[n] == false ){
              NodeVector childrenOfN = (*rtDataMap)[n].myChild;
              NodeVector::iterator cnIt;

              //has no children (=leaf)
              if ( childrenOfN.size() == 0 ){
                //connect it to this already visited node
                n = (*rtDataMap)[n].lpSource;
                break;
              }

              for (cnIt=childrenOfN.begin(); cnIt != childrenOfN.end(); ++cnIt){
                int lpOfNChild = (*rtDataMap)[*cnIt].lowpointNum;

                if ( lpOfNChild == 0 || lpOfNChild < dfsOfCurr ){
                  n = *cnIt;
                  if (n != current){
                    earVector.push_back(*cnIt);
                  }
                  break;
                }
                else {
                  n = (*rtDataMap)[n].lpSource;
                  if ( !ready[n] ){
                    earVector.push_back(n);
                  }
                  break;
                }
              }//endfor

            }//endwhile

            /****************************
             * set ready to found ear members
             * and push them to the stack
             * set arcs in ADAG
             ****************************/
            NodeVector::reverse_iterator rIt = earVector.rbegin();
            for (rIt = earVector.rbegin(); rIt != earVector.rend(); ++rIt){
              ready[*rIt] = true;
              stack.push(*rIt);

              //set arcs in ADAG
              if ( (rIt+1) != earVector.rend() ){
                Node next = *(rIt+1);
                Arc wellDirectedArc = findArc(*g, next, *rIt);
                if ( (*arcFilterADAG)[wellDirectedArc] == false ){
                  (*arcFilterADAG)[wellDirectedArc] = true;
                }
              }
            }//endfor reverse

            //set first arc between current --> x1
            Arc init = findArc(*g, current, earVector.front());
            (*arcFilterADAG)[init] = true;

            //set last arc between x_k-1 --> ready
            Arc last = findArc(*g, earVector.back(), n);
            (*arcFilterADAG)[last] = true;

            //ear found, clear earVector
            earVector.clear();

          }//endif ready
        }//endfor children

        /********************************************************
         * for each neighbor 'n' of current which is not a child
         ********************************************************/
        earVector.clear();
        for (Graph::NodeIt nIt(*g); nIt != INVALID; ++nIt){
          NodeVector children = (*rtDataMap)[current].myChild;
          Node n = nIt;

          if (  neighbor(current, n) &&
                find(children.begin(), children.end(), n) == children.end()     ){

            if ( ready[n] == false &&
                 (*rtDataMap)[n].lpSource == current ){

              while ( ready[n] == false ){
                Node e = parentInDfs(n);
                if ( e == INVALID ){
                  cerr<<"RedundantTree: "<<g->id(n)<<" has no parent in DFS!"<<endl;
                  break;
                }
                earVector.push_back(n);
                n = e;
              }//endwhile

              /****************************
               * set ready found ear members
               * and push them to the stack
               * set them in ADAG
               ****************************/
              NodeVector::reverse_iterator rIt = earVector.rbegin();
              for (rIt = earVector.rbegin(); rIt != earVector.rend(); ++rIt){
                ready[*rIt] = true;
                stack.push(*rIt);
                //cout<<"push stack --> "<<g->id(*rIt)<<endl;

                //set arcs in ADAG
                if ( (rIt+1) != earVector.rend() ){
                  Node next = *(rIt+1);
                  Arc wellDirectedArc = findArc(*g, next, *rIt);
                  if ( (*arcFilterADAG)[wellDirectedArc] == false ){
                    (*arcFilterADAG)[wellDirectedArc] = true;
                    //cout<<"<< arc in adag: "<<g->id(wellDirectedArc)<<" >>"<<endl;
                  }
                }
              }//endfor reverse

              //set first arc between current --> x1
              Arc init = findArc(*g, current, earVector.front());
              (*arcFilterADAG)[init] = true;
              //cout<<"<< init arc in adag: "<<g->id(init)<<" >>"<<endl;

              //set last arc between x_k-1 --> ready
              Arc last = findArc(*g, earVector.back(), n);
              (*arcFilterADAG)[last] = true;
              //cout<<"<< last arc in adag: "<<g->id(last)<<" >>"<<endl;

              //clear earVector
              earVector.clear();

            }//endif inner
          }//endif out

        }//endfor neighbours

      }//endwhile
      //cout<<"=================================="<<endl;
      //cout<<"ADAG is done."<<endl;
      //cout<<"=================================="<<endl;
    }

    /****************************************
     * On the top of the ADAG it calculates
     * the first spanning tree => RED one
     ****************************************/
    void createREDTree(const Node &root){
      //create ADAG subdigraph from g
      Graph::NodeMap<bool> nodeFilterADAG(*g, true);
      SubDigraph<const Graph> ADAG(*g, nodeFilterADAG, *arcFilterADAG);
      BfsSubType bfsAdag(ADAG);

      bfsAdag.run(root); //run BFS
      const BfsSubType::PredMap &bfsAdagPred = bfsAdag.predMap();

      for (SubDigraph<const Graph>::NodeIt n(ADAG); n != INVALID; ++n){
        if ( bfsAdagPred[n] != INVALID ){
          (*arcFilterRED)[bfsAdagPred[n]] = true;
        }
      }
    }

    /*****************************************************
     * On the top of the modified ADAG (reversed arcs)
     * it calculates the second spanning tree => BLUE one
     *****************************************************/
    void createBLUETree(const Node &root){
      //create ADAG subdigraph from g
      Graph::NodeMap<bool> nodeFilterADAG(*g, true);
      Graph::ArcMap<bool> arcFilterReverseADAG(*g, false);
      SubDigraph<const Graph> ADAG(*g, nodeFilterADAG, *arcFilterADAG);
      SubDigraph<const Graph> rADAG(*g, nodeFilterADAG, arcFilterReverseADAG);


      //reverse the arcs in ADAG => rADAG
      for (SubDigraph<const Graph>::ArcIt e(ADAG); e != INVALID; ++e){
        if ( (*arcFilterADAG)[e] ){
          Node sourceOfE = g->source(e);
          Node destOfE = g->target(e);
          Arc revOfE = findArc(*g, destOfE, sourceOfE);
          arcFilterReverseADAG[revOfE] = true;
        }
      }

      //BFS on rADAG
      BfsSubType bfsRadag(rADAG);

      bfsRadag.run(root); //run BFS
      const BfsSubType::PredMap &bfsRadagPred = bfsRadag.predMap();

      for (SubDigraph<const Graph>::NodeIt n(rADAG); n != INVALID; ++n){
        if ( bfsRadagPred[n] != INVALID )
          (*arcFilterBLUE)[bfsRadagPred[n]] = true;
      }
    }

    void run(const Node &root){
      if (edgeConnValue <= 1)
        cerr<<"Graph is only "<<edgeConnValue<<" edge connected!"<<endl;

      fillDfsAndLowPoint(root);
      setupADAG(root);
      createREDTree(root);
      createBLUETree(root);
    }

    Graph::ArcMap<bool>* getREDTree(){
      if (arcFilterRED == NULL){
        cerr<<"arcFilterRED is not initialized!"<<endl;
      }
      return arcFilterRED;
    }

    Graph::ArcMap<bool>* getBLUETree(){
      if (arcFilterBLUE == NULL){
        cerr<<"arcFilterBLUE is not initialized!"<<endl;
      }
      return arcFilterBLUE;
    }

    int getDFSNumber(const Node &n){
      return (*rtDataMap)[n].DFSNum;
    }

    bool isArcInADAG(const Arc &e){
      return (*arcFilterADAG)[e];
    }

    int getEdgeConnectivity(){
      return edgeConnValue;
    }

    bool isBiEdgeConnected(){
      return biEdgeConn;
    }

    bool isBiNodeConnected(){
      return biNodeConn;
    }

    /***************************************
     * Determines edge connectivity (returns
     * the number)
     ***************************************/
    int setEdgeConnectivityValue(){
      //need to assign capacity
      const CostMap cost(*g, 1);
      HaoOrlin<const Graph, const CostMap> hoTest(*g, cost);

      hoTest.init();
      hoTest.run();

      return hoTest.minCutValue();
    }

    /***************************************
     * Check connectivity
     * does not work yet, do not know why...
     ***************************************/
    void checkGraphConnectivity(){
      Undirector<const Graph> undirG(*g);

      //biEdgeConn = biEdgeConnected(undirG);
      //biNodeConn = biNodeConnected(undirG);
    }

    /***************************************
     * needed for test
     ***************************************/
    void testJumpInTree(const Node &root, Node &current, vector<Node> &path,
                        const SubDigraph<const Graph> &sg){
      if (current == root){
        return;
      }
      path.push_back(current);
      for (SubDigraph<const Graph>::ArcIt e(sg); e != INVALID; ++e){
        if (sg.target(e) == current){
          current = sg.source(e);
          break;
        }
      }
      testJumpInTree(root, current, path, sg);
    }

    /***************************************
     * Test function for verifying the results
     ***************************************/
    bool testTrees(const Node &root, const Node &node){
      Node current = node;
      vector<Node> redPath;
      vector<Node> bluePath;
      Graph::NodeMap<bool> nodeFilter(*g, true);
      SubDigraph<const Graph> redSG(*g, nodeFilter, *arcFilterRED);
      SubDigraph<const Graph> blueSG(*g, nodeFilter, *arcFilterBLUE);

      redPath.clear();
      testJumpInTree(root, current, redPath, redSG);

      bluePath.clear();
      current = node;
      testJumpInTree(root, current, bluePath, blueSG);

      //first element is the source node, remove it
      if ( redPath.size()  > 0 ) redPath.erase(redPath.begin());
      if ( bluePath.size() > 0 ) bluePath.erase(bluePath.begin());

      //check if they have common elements
      for (vector<Node>::iterator it=redPath.begin(); it!= redPath.end(); ++it){
        if ( find(bluePath.begin(), bluePath.end(), *it) != bluePath.end() ){
          cout<<" testIfRTsAreDisjuncts: FAIL!"<<endl;
          //cout<<" common node: "<<g->id(*it)<<endl;

          //cout<<"   RedTreePath: [ ";
          for (vector<Node>::iterator it=redPath.begin(); it!= redPath.end(); ++it){
            //  cout<<g->id(*it)<<" ";
          }
          //cout<<"]"<<endl;

          //cout<<"   BlueTreePath: [ ";
          for (vector<Node>::iterator it=bluePath.begin(); it!= bluePath.end(); ++it){
            //  cout<<g->id(*it)<<" ";
          }
          //cout<<"]"<<endl;

          return false;
        }
      }

      //cout<<" testIfRTsAreNodeDisjuncts: PASS"<<endl;
      //cout<<" root: "<<g->id(root)<<" node: "<<g->id(node)<<endl;
      //cout<<"   RedTreePath: [ ";
      for (vector<Node>::iterator it=redPath.begin(); it!= redPath.end(); ++it){
        //      cout<<g->id(*it)<<" ";
      }
      //cout<<"]"<<endl;

      //cout<<"   BlueTreePath: [ ";
      for (vector<Node>::iterator it=bluePath.begin(); it!= bluePath.end(); ++it){
        //      cout<<g->id(*it)<<" ";
      }
      //cout<<"]"<<endl<<endl;

      int numOfRedArcs  = 0;
      int numOfBlueArcs = 0;
      for (Graph::ArcIt e(*g); e != INVALID; ++e){
        if ( (*arcFilterRED)[e] ) numOfRedArcs++;
        if ( (*arcFilterBLUE)[e] ) numOfBlueArcs++;
      }
      //cout<<" # arcs in RED tree: "<<numOfRedArcs<<endl;
      //cout<<" # arcs in BLUE tree: "<<numOfBlueArcs<<endl;
      //string verdict = "PASS";
      //if (numOfBlueArcs != numOfRedArcs) verdict="FAIL";
      //cout<<" testIfNumOfArcsAreEqual: "<<verdict<<endl;
      //cout<<"=================================="<<endl;

      return true;
    }

    /***************************************
     * Destructor - cleaning up memory
     ***************************************/
    ~RedundantTree(){
      delete rtDataMap;
      delete arcFilterADAG;
      delete arcFilterRED;
      delete arcFilterBLUE;
    }

  };
}

#endif
