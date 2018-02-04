/*
 * Test compilation:
 *    > cd RouterVirtSimulator
 *    > cmake -DWITH_TEST . && make
 *
 * Test execution:
 *    > cd RouterVirtSimulator/build/test
 *    > ./routerVirtSimTest --log_level=test_suite
 *    OR
 *    > ./routerVirtSimTest --log_level=all --run_test=<TestSuite/TestCase>
 *
 *  Debugging a testcase with EMACS:
 *      Alt+x:gdb and give --args option and use --run_test=...
 *
 *  Written by Mate Nagy
 *  License: GNU General Public License Version 3
 */

#define BOOST_TEST_MODULE RouterVirtSimulator_Test
#define BOOST_AUTO_TEST_MAIN
#include <boost/test/included/unit_test.hpp>
#include <vector>
#include "../src/Network.hh"
#include "../src/RTBasedMethod.hh"
#include "../src/GreedyILP.hh"
#include "../src/rtmanager.h"
#include "../src/Log.h"
#include <lemon/arg_parser.h>

#define NODESET_SIZE 4

LogStream* LogStream::mInstance = NULL;
LogStream debug(std::cout, INFO);

const std::string LGF_PATH = "../../lgf_files/";

void parseDebugLevel(int argc, char **argv){
  cout<<"Parsing debug level... ";

  ArgParser ap(argc, argv);
  int debugLevel = -1;

  ap.refOption("v", "Verbosity level", debugLevel);
  ap.synonym("-verbose" , "v");

  ap.parse();

  switch(debugLevel){
  case 1:
   debug.level() = CRITICAL;
   break;
  case 2:
   debug.level() = INFO;
   break;
  case 3:
   debug.level() = LOG;
   break;
  case 4:
   debug.level() = DEBUG1;
   break;
  case 5:
   debug.level() = DEBUG2;
   break;
  default:
   debug.level() = INFO;
  }

  cout<<debug.level()<<endl;

}

/**************************************
 * Testing basic network functionalities
 **************************************/
BOOST_AUTO_TEST_SUITE( TS_01_Basics_Networking )

BOOST_AUTO_TEST_CASE( tc_01_01_test_example1 )
{
    string graph = LGF_PATH + "test_example1.lgf";
    Network test_network(graph);

    BOOST_CHECK( test_network.longestShortestPath() > 0 );

    Node s = (test_network.g).nodeFromId(0);
    Node d = (test_network.g).nodeFromId(1);

    BOOST_CHECK_CLOSE( test_network.distOpt(s, d), 3, 0.01 );

    LinkSet faulty;
    d = (test_network.g).nodeFromId(2);
    Node n = (test_network.g).nodeFromId(3);
    faulty.insert(findArc(test_network.g, s, n));
    faulty.insert(findArc(test_network.g, n, s));

    bool havePath = (test_network.packetTracer)->haveValidPath(s, d, faulty);

    BOOST_CHECK( havePath == 1 );
}

BOOST_AUTO_TEST_CASE( tc_01_01b_uniCostBD )
{
    string graph = LGF_PATH + "counter_example.lgf";
    Network netw(graph);

    BOOST_CHECK( netw.longestShortestPath() > 0 );

    Node s = (netw.g).nodeFromId(0);
    Node d = (netw.g).nodeFromId(0);
    BOOST_CHECK( netw.distUni(s, d) == 0 );

    s = (netw.g).nodeFromId(1);
    d = (netw.g).nodeFromId(3);
    BOOST_CHECK( netw.distUni(s, d) == 2 );

    s = (netw.g).nodeFromId(0);
    d = (netw.g).nodeFromId(4);
    BOOST_CHECK( netw.distUni(s, d) == 1 );

    s = (netw.g).nodeFromId(1);
    d = (netw.g).nodeFromId(4);
    BOOST_CHECK( netw.distUni(s, d) == 1 );
}

BOOST_AUTO_TEST_CASE( tc_01_02_readingVirtualStuffs ){
    string graph = LGF_PATH + "test_example1_virtual.lgf";

    Network network(graph);

    //check if virtual flags are correctly set:
    Node p0 = (network.g).nodeFromId(0);
    Node p1 = (network.g).nodeFromId(1);
    Node v      = (network.g).nodeFromId(4);

    BOOST_CHECK( (*(network.nodeInfo))[p0].isVirtual    == false );
    BOOST_CHECK( (*(network.nodeInfo))[p1].isVirtual    == false );
    BOOST_CHECK( (*(network.nodeInfo))[v].isVirtual     == true  );
    BOOST_CHECK( (*(network.nodeInfo))[v].physicalNode  == p1    );

    Arc pArc = findArc(network.g, p0, p1);
    Arc vArc = findArc(network.g, p0, v);

    BOOST_CHECK( (*(network.linkInfo))[pArc].isVirtual == false );
    BOOST_CHECK( (*(network.linkInfo))[vArc].isVirtual == true  );

    // check # of links in 0->1 local SRG sets
    // should contain 4 links
    SRGSet srg = (*(network.nodeInfo))[p0].localSRGs;
    SRGSet::iterator srgIt;
    for ( srgIt = srg.begin(); srgIt != srg.end();++srgIt){
        //cout<<"SRG "<<srgIt->size()<<endl;
        //LinkSet::iterator lIt;
        //for (lIt = srgIt->begin(); lIt != srgIt->end(); ++lIt){
        //  cout<<"  link: "<<(network.g).id(*lIt)<<endl;
        //}
        if ( srgIt->size() > 0 )
          BOOST_CHECK( srgIt->size() == 4 );
    }
}

BOOST_AUTO_TEST_CASE( tc_01_03_readingNonVirtualStuffs ){
    debug.level() = LOG;
    string graph = LGF_PATH + "test_example1.lgf";

    Network network(graph);

    //check if virtual flags are correctly set:
    Node p0 = (network.g).nodeFromId(0);
    Node p1 = (network.g).nodeFromId(1);

    BOOST_CHECK( (*(network.nodeInfo))[p0].isVirtual     == false );
    BOOST_CHECK( (*(network.nodeInfo))[p1].isVirtual       == false );
    BOOST_CHECK( (*(network.nodeInfo))[p0].physicalNode  == p0  );

    Arc pArc = findArc(network.g, p0, p1);

    BOOST_CHECK( (*(network.linkInfo))[pArc].isVirtual == false );

    // check # of links in 0->1 local SRG sets
    // should contain 0 links
    SRGSet srg = (*(network.nodeInfo))[p0].localSRGs;
    SRGSet::iterator srgIt;

    for ( srgIt = srg.begin(); srgIt != srg.end();++srgIt){
        BOOST_CHECK( srgIt->size() == 0 );
    }
}

BOOST_AUTO_TEST_CASE( tc_01_04_count_average_path_length ){
    debug.level() = LOG;
    string graph = LGF_PATH + "test_example1.lgf";
    Network network(graph);
    int numOfLoops = 0;

    // 4 virtual nodes will extend LFA cov to 1
    network.greedyH->greedyHeuristicAlgorithm(1, NODESET_SIZE);

    bool haveLoop = network.checkLoopAndAVPL(numOfLoops);
    BOOST_CHECK( haveLoop == 0 );

    //cout<<"woFAPL: "<<network.woFaultAPL<<endl;
    // network.woFaultAPL should be 1.66667
    BOOST_CHECK( network.woFaultAPL > 1.6 );
    BOOST_CHECK( network.woFaultAPL < 1.7 );

    //cout<<"wFAPL: "<<network.wFaultAPL<<endl;
    // network.wFaultAPL should be 2.3333
    BOOST_CHECK( network.wFaultAPL > 2.3 );
    BOOST_CHECK( network.wFaultAPL < 2.4 );
}

BOOST_AUTO_TEST_CASE( tc_01_05_count_average_path_length ){
    debug.level() = LOG;
    string graph = LGF_PATH + "test_example2.lgf";
    Network network(graph);
    int numOfLoops = 0;

    // 4 virtual nodes will extend LFA cov to 1

    //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
    GreedyParamSet gPS = {false, false, true, true, CONN_L};
    network.greedyH->setGreedyParamSet(gPS);

    network.greedyH->greedyHeuristicAlgorithm(1, NODESET_SIZE);
    //write result into file
    //network.writeGraph();

    bool haveLoop = network.checkLoopAndAVPL(numOfLoops, false);
    BOOST_CHECK( haveLoop == 0 );

    //cout<<"woFAPL: "<<network.woFaultAPL<<endl;
    // network.woFaultAPL should be 1.86667
    BOOST_CHECK( network.woFaultAPL > 1.8 );
    BOOST_CHECK( network.woFaultAPL < 1.9 );

    //cout<<"wFAPL: "<<network.wFaultAPL<<endl;
    // network.wFaultAPL should be 2.7333
    BOOST_CHECK( network.wFaultAPL > 2.7 );
    BOOST_CHECK( network.wFaultAPL < 2.8 );
}

/*
 * Check if average path length calculation
 * works during network manipulation. If there
 * is no path between two nodes, we count with 0.
 */
BOOST_AUTO_TEST_CASE( tc_01_06_count_average_path_length ){
    debug.level() = LOG;
    string graph = LGF_PATH + "test_example2.lgf";
    Network network(graph);
    int numOfLoops = 0;

    // 4 virtual nodes will extend LFA cov to 1

    //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
    GreedyParamSet gPS = {false, false, true, true, CONN_L};
    network.greedyH->setGreedyParamSet(gPS);

    // Step 1.: Node to virt: 1. Escape: 4
    Node v = network.g.nodeFromId(1);
    Node e = network.g.nodeFromId(4);
    NodeSet nSet;
    nSet.insert(v);
    network.greedyH->realizeVirtualNode(nSet, e);

    bool haveLoop = network.checkLoopAndAVPL(numOfLoops);
    BOOST_CHECK( haveLoop == 0 );

    //cout<<"wFAPL: "<<network.wFaultAPL<<endl;
    // network.wFaultAPL should be 2.65
    BOOST_CHECK( network.wFaultAPL > 2.6 );
    BOOST_CHECK( network.wFaultAPL < 2.7 );

    // Step 2.: Node to virt: 4. Escape: 1
    v = network.g.nodeFromId(4);
    e = network.g.nodeFromId(1);
    nSet.clear(); nSet.insert(v);
    network.greedyH->realizeVirtualNode(nSet, e);
    network.writeGraph();

    haveLoop = network.checkLoopAndAVPL(numOfLoops);
    BOOST_CHECK( haveLoop == 0 );
    //cout<<"wFAPL: "<<network.wFaultAPL<<endl;
    // network.wFaultAPL should be 2.65
    BOOST_CHECK( network.wFaultAPL > 2.6 );
    BOOST_CHECK( network.wFaultAPL < 2.7 );
}

BOOST_AUTO_TEST_SUITE_END()

/**************************************
 * Testing PacketTracer functionality
 **************************************/
BOOST_AUTO_TEST_SUITE( TS_02_PacketTracer )

/*
 * Testing greedy heuristic final network if have loops
 * happy case without challanges
 */
BOOST_AUTO_TEST_CASE( tc_02_01_greedy_test_example1 ){
  parseDebugLevel(boost::unit_test::framework::master_test_suite().argc,
                  boost::unit_test::framework::master_test_suite().argv);
  string graph = LGF_PATH + "test_example1.lgf";
  Network network(graph);
  double ratio = 1;

  network.greedyH->greedyHeuristicAlgorithm(ratio, NODESET_SIZE);
  //network.writeGraph();

  for (Graph::NodeIt s(network.g); s != INVALID; ++s)
    for (Graph::NodeIt d(network.g); d != INVALID; ++d){
       if ( s != d &&
           (*(network.nodeInfo))[s].isVirtual == false &&
           (*(network.nodeInfo))[d].isVirtual == false &&
           network.isLegacyLFAProtected_LP(s, d) != INVALID ){
        Node nh = network.nh(s, d);
        LinkSet faulty;
        faulty.insert(findArc(network.g, s, nh));
        faulty.insert(findArc(network.g, nh, s));

        bool havePath = (network.packetTracer)->haveValidPath(s, d, faulty);
        BOOST_CHECK( havePath == 1 );
        if (!havePath){
          cout<<"s: "<<(network.g).id(s)<<" , d: "<<(network.g).id(d)<<endl;
        }
       }
     }
     int numOfVirtualNodes = countNodes(network.g)-network.numOfPhysicalNodes;
     BOOST_CHECK( numOfVirtualNodes == 4 ); //4(0),5(1),6(2),7(3)

     double finalLfaCov = network.newLfaCoverageMetric_LP(LP);
     BOOST_CHECK_CLOSE( finalLfaCov, 1, 0.01 );
}

/*
 * Testing greedy heuristic with PT if have loops
 * in the final graph
 */
BOOST_AUTO_TEST_CASE( tc_02_02_greedy_test_example2 ){
  string graph = LGF_PATH + "test_example2.lgf";
  Network network(graph);
  double ratio = 1;

  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, false, true, true, CONN_L};
  network.greedyH->setGreedyParamSet(gPS);

  network.greedyH->greedyHeuristicAlgorithm(ratio, NODESET_SIZE);
  //network.writeGraph();

  for (Graph::NodeIt s(network.g); s != INVALID; ++s)
    for (Graph::NodeIt d(network.g); d != INVALID; ++d){
      if ( s != d &&
           (*(network.nodeInfo))[s].isVirtual == false &&
           (*(network.nodeInfo))[d].isVirtual == false &&
           network.isLegacyLFAProtected_LP(s, d) != INVALID ){
            Node nh = network.nh(s, d);
            LinkSet faulty;
            faulty.insert(findArc(network.g, s, nh));
            faulty.insert(findArc(network.g, nh, s));

            bool havePath = (network.packetTracer)->haveValidPath(s, d, faulty);

            BOOST_CHECK( havePath == 1 );
            if (!havePath){
              cout<<"s: "<<(network.g).id(s)<<" , d: "<<(network.g).id(d)<<endl;
            }

            bool haveAllPaths = network.packetTracer->haveAllPathsValid(s, d, faulty);
            //network.packetTracer->printAllPossiblePaths();

            BOOST_CHECK( haveAllPaths == 1 );
            if (!haveAllPaths){
              cout<<"s: "<<(network.g).id(s)<<" , d: "<<(network.g).id(d)<<endl;
            }
       }
    }
  int numOfVirtualNodes = countNodes(network.g)-network.numOfPhysicalNodes;
  BOOST_CHECK( numOfVirtualNodes == 4 ); //
}

/*
 * Pure PT testing without greedy or ILP
 *
 * One alternate path, but includes the
 * failing link, so LFA loop will turn up.
 *
 * (PacketTracer should consider local SRLGs and
 * faulty link itself). In this case node 4 is a trap
 * for 0-2 unprotected SD pairs
 */
BOOST_AUTO_TEST_CASE( tc_02_03_one_path_detect_loop_1 ){
  string graph = LGF_PATH + "test_example2_virtual.lgf";

  Network network (graph);

  Node s = (network.g).nodeFromId(0);
  Node d = (network.g).nodeFromId(2);
  Node nh = network.nh(s,d);
  LinkSet faulty;
  if ( nh != INVALID ){
   faulty.insert(findArc(network.g, s, nh));
   faulty.insert(findArc(network.g, nh, s));
  }
  else {
    cout<<"Next-hop is INVALID, bailing out...!"<<endl;
  }
  bool havePath = (network.packetTracer)->haveValidPath(s, d, faulty);
  //Debug:
  //(network.greedyH)->printLocalSRLGs();
  BOOST_CHECK( havePath == 0 );
  BOOST_CHECK( network.packetTracer->haveLoop = 1 );

  bool haveAllPaths = network.packetTracer->haveAllPathsValid(s, d, faulty);
  //network.packetTracer->printAllPossiblePaths();
  BOOST_CHECK( haveAllPaths == 0 );
  BOOST_CHECK( network.packetTracer->haveLoop = 1 );
}

/*
 * Pure PT testing without greedy or ILP
 *
 * One alternate path, but now the fake LFA
 * has virtual (jointly failing) link.
 * Fake LFA cause loop => should be detected
 */
BOOST_AUTO_TEST_CASE( tc_02_04_one_path_detect_loop_2 ){
  string graph = LGF_PATH + "test_example4_virtual.lgf";

  Network network (graph);
  NodeInfoMap *nodeInfo = network.nodeInfo;

  Node s = network.g.nodeFromId(0);
  Node d = network.g.nodeFromId(1);
  LinkSet failLinks;
  Node nhop = network.nh(s, d);
  Arc  failTo   = findArc(network.g, s, nhop);
  Arc  failFrom = findArc(network.g, nhop, s);
  failLinks.insert(failTo);
  failLinks.insert(failFrom);

  bool havePath = network.packetTracer->haveValidPath(s, d, failLinks);
  //network.packetTracer->printPrimaryPath();
  //(network.greedyH)->printLocalSRLGs();
  BOOST_CHECK( havePath == 0 );
  BOOST_CHECK( network.packetTracer->haveLoop = 1 );

  bool haveAllPaths = network.packetTracer->haveAllPathsValid(s, d, failLinks);
  //network.packetTracer->printAllPossiblePaths();
  BOOST_CHECK( haveAllPaths == 0 );
  BOOST_CHECK( network.packetTracer->haveLoop = 1 );
}

/*
 * Pure PT testing without greedy or ILP
 *
 * One alternate path, and it includes the
 * failing link. However there are two LFAs
 * one fake and one valid. Paths are not safe.
 */
BOOST_AUTO_TEST_CASE( tc_02_05_one_valid_path_and_one_loop ){
  string graph = LGF_PATH + "test_example6_virtual.lgf";

  Network network (graph);
  NodeInfoMap *nodeInfo = network.nodeInfo;

  Node s = network.g.nodeFromId(0);
  Node d = network.g.nodeFromId(1);
  LinkSet failLinks;

  Node nhop = network.nh(s, d);
  Arc  failTo   = findArc(network.g, s, nhop);
  Arc  failFrom = findArc(network.g, nhop, s);
  failLinks.insert(failTo);
  failLinks.insert(failFrom);

  // check with single path tracer (can select the working path)
  bool havePath = network.packetTracer->haveValidPath(s, d, failLinks);
  //network.packetTracer->printPrimaryPath();
  BOOST_CHECK( havePath == 0 );

  // check all possible paths
  bool haveAllPaths = network.packetTracer->haveAllPathsValid(s, d, failLinks);
  //network.packetTracer->printAllPossiblePaths();
  BOOST_CHECK( haveAllPaths == 0 );
  BOOST_CHECK( network.packetTracer->haveLoop = 1 );
}

/*
 * Pure PT testing without greedy or ILP
 *
 * One alternate path, and it includes the
 * failing link. However there should not be
 * a valid LFA so packet will not reach d.
 *
 * NOTE: This case cannot happen if we create
 * all the virtual edges between virtual nodes
 * and its neighbours. Now (6,4) is removed.
 * (6,3) was not created at all.
 */
BOOST_AUTO_TEST_CASE( tc_02_06_no_valid_path_with_LFA ){
  string graph = LGF_PATH + "test_example6_virtual.lgf"; //tobe modified

  Network network (graph);
  NodeInfoMap *nodeInfo = network.nodeInfo;

  //---------------------------------------
  // Removing 6->4 and 4->6 arcs, to avoid
  // 4 be selected as a fake LFA
  Node v6 = network.g.nodeFromId(6);
  Node n4 = network.g.nodeFromId(4);
  Arc toRemove1 = findArc(network.g, v6, n4);
  Arc toRemove2 = findArc(network.g, n4, v6);
  network.g.erase(toRemove1);
  network.g.erase(toRemove2);
  //---------------------------------------

  Node s = network.g.nodeFromId(0);
  Node d = network.g.nodeFromId(1);
  LinkSet failLinks;

  Node nhop = network.nh(s, d);
  Arc  failTo   = findArc(network.g, s, nhop);
  Arc  failFrom = findArc(network.g, nhop, s);
  failLinks.insert(failTo);
  failLinks.insert(failFrom);

  // check with single path tracer
  bool havePath = network.packetTracer->haveValidPath(s, d, failLinks);
  //network.packetTracer->printPrimaryPath();
  BOOST_CHECK( havePath == 0 );

  // check all possible paths
  bool haveAllPaths = network.packetTracer->haveAllPathsValid(s, d, failLinks);
  //network.packetTracer->printAllPossiblePaths();
  BOOST_CHECK( haveAllPaths == 0 );
}

BOOST_AUTO_TEST_CASE( tc_02_07_improved_PT_find_all_next_hops ){
  string graph = LGF_PATH + "packet_trace_tester_1.lgf";

  Network network (graph);
  NodeInfoMap *nodeInfo = network.nodeInfo;
  LinkSet emptyFailLinks;
  NodeVector nHops;
  NodeVector::iterator vIt;
  Node s5 = network.g.nodeFromId(5);
  Node d3 = network.g.nodeFromId(3);

  network.packetTracer->getAllNextHopsToD(s5, d3, emptyFailLinks, nHops);
  BOOST_CHECK( nHops.size() == 2 ); //0 and 4
  //cout<<"[ ";
  //for (vIt = nHops.begin(); vIt != nHops.end(); ++vIt){
  //  cout<<network.g.id(*vIt)<<" ";
  //}
  //cout<<"]"<<endl;

  Node s4 = network.g.nodeFromId(4);
  nHops.clear();
  network.packetTracer->getAllNextHopsToD(s4, d3, emptyFailLinks, nHops);
  BOOST_CHECK( nHops.size() == 3 ); //1 and 2 and 3
  //cout<<"[ ";
  //for (vIt = nHops.begin(); vIt != nHops.end(); ++vIt){
  //  cout<<network.g.id(*vIt)<<" ";
  //}
  //cout<<"]"<<endl;

  Node s0 = network.g.nodeFromId(0);
  nHops.clear();
  network.packetTracer->getAllNextHopsToD(s0, d3, emptyFailLinks, nHops);
  BOOST_CHECK( nHops.size() == 1 ); //1
}

/*
 * Pure PT testing without greedy or ILP
 *
 * More alternate paths and there is no
 * fault yet in the network. Purpose is to
 * find all the possible ECMP paths.
 * Definitely a happy case.
 */
BOOST_AUTO_TEST_CASE( tc_02_08_improved_PT_find_ecmp_paths_no_fault ){
  string graph = LGF_PATH + "packet_trace_tester_1.lgf";
  Network network (graph);
  LinkSet linkFails; //empty now

  Node s5 = network.g.nodeFromId(5);
  Node d3 = network.g.nodeFromId(3);
  network.packetTracer->haveAllPathsValid(s5, d3, linkFails);
  BOOST_CHECK( network.packetTracer->numOfPossiblePaths == 4 );
  //network.packetTracer->printAllPossiblePaths();

  Node s0 = network.g.nodeFromId(0);
  Node d4 = network.g.nodeFromId(4);
  network.packetTracer->haveAllPathsValid(s0, d4, linkFails);
  BOOST_CHECK( network.packetTracer->numOfPossiblePaths == 2 );
  //network.packetTracer->printAllPossiblePaths();

  network.packetTracer->haveAllPathsValid(s0, d3, linkFails);
  BOOST_CHECK( network.packetTracer->numOfPossiblePaths == 1 );
  //network.packetTracer->printAllPossiblePaths();
}

/*
 * Pure PT testing without greedy or ILP
 *
 * More alternate paths because at some
 * point there are more than 1 LFAs.
 * We need to check all the possibilities
 * to be able to determine if all the paths
 * are safe or loops can arise.
 * No ECMP this time.
 */
BOOST_AUTO_TEST_CASE( tc_02_09_improved_PT_jumps_on_all_LFAs ){
  string graph = LGF_PATH + "packet_trace_tester_1.lgf";
  Network network (graph);
  bool allPathsSafe;

  LinkSet failLinks;
  Node s4 = network.g.nodeFromId(4);
  Node d1 = network.g.nodeFromId(1);
  Node nhop = network.nh(s4, d1);
  Arc  failTo   = findArc(network.g, s4, nhop);
  Arc  failFrom = findArc(network.g, nhop, s4);
  failLinks.insert(failTo);
  failLinks.insert(failFrom);

  allPathsSafe = network.packetTracer->haveAllPathsValid(s4, d1, failLinks);
  BOOST_CHECK( allPathsSafe == 1 );
  BOOST_CHECK( network.packetTracer->numOfPossiblePaths == 2 );
  //network.packetTracer->printAllPossiblePaths();
}

/*
 * Pure PT testing without greedy or ILP
 *
 * More ECMP alternate paths in physical layer.
 * Some of them consists the faulty link
 * and at that point there is no LFA,
 * so PT stucks.
 *
 * faulty link (1,2), PT goes from 5 to 3.
 * 4 ECMPs:
 * [5,4,3] [5,4,1,2,3] [5,4,2,3] [5,0,1,2,3]
 * 2 includes (1,2) => not all the alternate
 * paths are safe
 */
BOOST_AUTO_TEST_CASE( tc_02_10_improved_PT_phy_layer_stuck ){
  string graph = LGF_PATH + "packet_trace_tester_1.lgf";
  Network network (graph);
  bool allPathsSafe;
  LinkSet failLinks;

  Node n1 = network.g.nodeFromId(1);
  Node d3 = network.g.nodeFromId(3);
  failLinks.clear();
  Node nhop = network.nh(n1, d3);
  Arc failTo = findArc(network.g, n1, nhop);
  Arc failFrom = findArc(network.g, nhop, n1);
  failLinks.insert(failTo);
  failLinks.insert(failFrom);
  Node s5 = network.g.nodeFromId(5);
  allPathsSafe = network.packetTracer->haveAllPathsValid(s5, d3, failLinks);
  BOOST_CHECK( allPathsSafe == 0 );
  BOOST_CHECK( network.packetTracer->numOfPossiblePaths == 2 );
  //network.packetTracer->printAllPossiblePaths();
}

/*
 * Pure PT testing without greedy or ILP
 *
 * More ECMP alternate paths in physical layer.
 * Some of them consists the faulty link
 * and at that point there is a valid LFA,
 * so PT arrives to d.
 *
 * Faulty link (1,4), PT goes from 5 to 3.
 * Node 4 has an LFA (2) to 3.
 * Possible paths should be:
 * [5,4,3] [5,4,2,3] [5,0,1,2,3]
 */
BOOST_AUTO_TEST_CASE( tc_02_11_improved_PT_phy_layer_LFA ){
  string graph = LGF_PATH + "packet_trace_tester_1.lgf";
  Network network (graph);
  bool allPathsSafe;
  LinkSet failLinks;

  Node n1 = network.g.nodeFromId(1);
  Node d4 = network.g.nodeFromId(4);
  Arc failTo = findArc(network.g, n1, d4);
  Arc failFrom = findArc(network.g, d4, n1);
  failLinks.insert(failTo);
  failLinks.insert(failFrom);

  Node s5 = network.g.nodeFromId(5);
  Node d3 = network.g.nodeFromId(3);
  allPathsSafe = network.packetTracer->haveAllPathsValid(s5, d3, failLinks);
  BOOST_CHECK( allPathsSafe == 1 );
  BOOST_CHECK( network.packetTracer->numOfPossiblePaths == 3 );
  //network.packetTracer->printAllPossiblePaths();
}

BOOST_AUTO_TEST_SUITE_END()

/**************************************
 * Testing Trap Leakage phenomenon
 **************************************/
BOOST_AUTO_TEST_SUITE( TS_03_Trap_Leakage )

//After revealing trap condition leakage,
//modified code (by leakParamSet) should not
//create loops anymore
BOOST_AUTO_TEST_CASE( tc_03_02_trapLeakage ){
  string graph = LGF_PATH + "test_example2.lgf";

  Network network (graph);
  NodeInfoMap *nodeInfo = network.nodeInfo;
  LinkInfoMap *linkInfo = network.linkInfo;

  //do not use all the virtual links
  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, false, false, true, CONN_L};
  network.greedyH->setGreedyParamSet(gPS);

  network.greedyH->greedyHeuristicAlgorithm(0.666, NODESET_SIZE);

  int numOfLoops;
  BOOST_CHECK( network.checkLoopAndAVPL(numOfLoops) == 0 );

  int numOfVirtualNodes = countNodes(network.g)-network.numOfPhysicalNodes;
  BOOST_CHECK( numOfVirtualNodes  == 4 );

  int numOfVirtualArcs = countArcs(network.g)-network.numOfPhysicalArcs;
  BOOST_CHECK( numOfVirtualArcs  == 32 );
}

BOOST_AUTO_TEST_CASE( tc_03_03_trapLeakage ){
  string graph = LGF_PATH + "test_example2.lgf";

  Network network (graph);
  NodeInfoMap *nodeInfo = network.nodeInfo;
  LinkInfoMap *linkInfo = network.linkInfo;

  //do not use all the virtual links
  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, true, false, true, CONN_L};
  network.greedyH->setGreedyParamSet(gPS);

  network.greedyH->greedyHeuristicAlgorithm(0.666, NODESET_SIZE);

  int numOfLoops;
  BOOST_CHECK( network.checkLoopAndAVPL(numOfLoops) == 0 );

  int numOfVirtualNodes = countNodes(network.g)-network.numOfPhysicalNodes;
  BOOST_CHECK( numOfVirtualNodes  == 4 );

  int numOfVirtualArcs = countArcs(network.g)-network.numOfPhysicalArcs;
  BOOST_CHECK( numOfVirtualArcs  == 32 );
}

// These significantly slows down testing, so run only manually.
/*BOOST_AUTO_TEST_CASE( tc_03_05_trapLeakage_Geant ){
  string graph = "../lgf_files/directed_graphs/n031_e0049_Geant.lgf";

  Network network (graph);
  NodeInfoMap *nodeInfo = network.nodeInfo;
  LinkInfoMap *linkInfo = network.linkInfo;

  //(skipEdges, cascade, skipUpstream, smartTrap, isSPTBasedVirt)
  GreedyParamSet gPS = {false, true, false, true, false};
  network.greedyH->setGreedyParamSet(gPS);

  network.greedyH->greedyHeuristicAlgorithm(0.666);

  int numOfVirtualNodes = countNodes(network.g)-network.numOfPhysicalNodes;
  cout<<"#numOfVirtualNodes: "<<numOfVirtualNodes<<endl;

  int numOfLoops;
  BOOST_CHECK( network.checkLoopAndAVPL(numOfLoops) == 0 );
}

BOOST_AUTO_TEST_CASE( tc_03_06_trapLeakage_Geant ){
  string graph = "../lgf_files/directed_graphs/n031_e0049_Geant.lgf";

  Network network (graph);
  NodeInfoMap *nodeInfo = network.nodeInfo;
  LinkInfoMap *linkInfo = network.linkInfo;

  //(skipEdges, cascade, skipUpstream, smartTrap, isSPTBasedVirt)
  GreedyParamSet gPS = {false, false, true, true, false};
  network.greedyH->setGreedyParamSet(gPS);

  network.greedyH->greedyHeuristicAlgorithm(0.666);

  int numOfVirtualNodes = countNodes(network.g)-network.numOfPhysicalNodes;
  cout<<"#numOfVirtualNodes: "<<numOfVirtualNodes<<endl;

  int numOfLoops;
  BOOST_CHECK( network.checkLoopAndAVPL(numOfLoops) == 0 );
}

BOOST_AUTO_TEST_CASE( tc_03_07_trapLeakage_Geant ){
  string graph = "../lgf_files/directed_graphs/n031_e0049_Geant.lgf";
  Network network (graph);

  //(skipEdges, cascade, skipUpstream, smartTrap, isSPTBasedVirt)
  GreedyParamSet gPS = {true, true, true, true, false};
  network.greedyH->setGreedyParamSet(gPS);

  network.greedyH->greedyHeuristicAlgorithm(0.666);

  int numOfVirtualNodes = countNodes(network.g)-network.numOfPhysicalNodes;
  cout<<"#numOfVirtualNodes: "<<numOfVirtualNodes<<endl;

  int numOfLoops;
  BOOST_CHECK( network.checkLoopAndAVPL(numOfLoops) == 0 );
  }

BOOST_AUTO_TEST_CASE( tc_03_09_trapLeakage_Italy_cost ){
  string graph = "../lgf_files/directed_graphs/n033_e0056_Italy_cost.lgf";
  Network network (graph);

  //(skipEdges, cascade, skipUpstream, smartTrap, isSPTBasedVirt)
  GreedyParamSet gPS = {false, false, true, true, false};
  network.greedyH->setGreedyParamSet(gPS);

  network.greedyH->greedyHeuristicAlgorithm(0.666);

  int numOfLoops;
  BOOST_CHECK( network.checkLoopAndAVPL(numOfLoops) == 0 );
  }*/

BOOST_AUTO_TEST_CASE( tc_03_10_trapLeakage_paper2 ){
  string graph = LGF_PATH + "paper2.lgf";
  Network network (graph);

  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, false, true, false, CONN_L};
  network.greedyH->setGreedyParamSet(gPS);

  network.greedyH->greedyHeuristicAlgorithm(1, NODESET_SIZE);

  double finalLfaCov = network.newLfaCoverageMetric_LP(LP);
  int numOfLoops = 0;

  BOOST_CHECK( network.checkLoopAndAVPL(numOfLoops) == 0 );
  BOOST_CHECK_CLOSE( finalLfaCov, 1, 0.01 );
}

//check smart trap condition
BOOST_AUTO_TEST_CASE( tc_03_11_smartTrap_paper2 ){
  string graph = LGF_PATH + "paper2.lgf";

  Network network (graph);
  NodeInfoMap *nodeInfo = network.nodeInfo;
  LinkInfoMap *linkInfo = network.linkInfo;

  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, false, true, false, CONN_L};
  network.greedyH->setGreedyParamSet(gPS);

  //cout<<"Node to virt: 4/ Escape: 1"<<endl;
  Node v = network.g.nodeFromId(4);
  Node e = network.g.nodeFromId(1);
  NodeSet nSet;
  nSet.insert(v);
  network.greedyH->realizeVirtualNode(nSet, e);
  //network.greedyH->printOutUnprotectedSDSet();

  //cout<<"Nodes to virt: 4,1/ Escape: 5"<<endl;
  Node v2 = network.g.nodeFromId(1);
  e = network.g.nodeFromId(5);
  nSet.clear();
  nSet.insert(v);
  nSet.insert(v2);
  network.greedyH->realizeVirtualNode(nSet, e);
  //network.greedyH->printOutUnprotectedSDSet();

  //cout<<"Node to virt: 3/ Escape: 6"<<endl;
  v = network.g.nodeFromId(3);
  e = network.g.nodeFromId(6);
  nSet.clear();
  nSet.insert(v);
  network.greedyH->realizeVirtualNode(nSet, e);
  //network.greedyH->printOutUnprotectedSDSet();

  // Check if packet tracer finds LFA loop
  // when skipUpstreamLFA is on and s = 4 and d = 5
  Node s = network.g.nodeFromId(4);
  Node d = network.g.nodeFromId(5);
  Node nhop = network.nh(s,d);
  Arc  failTo   = findArc(network.g, s, nhop);
  Arc  failFrom = findArc(network.g, nhop, s);
  LinkSet failLinks;
  failLinks.insert(failTo);
  failLinks.insert(failFrom);

  bool havePath = network.packetTracer->haveValidPath(s, d, failLinks);

  BOOST_CHECK( havePath == 0 );
}

BOOST_AUTO_TEST_SUITE_END()

/**************************************
 * Testing Greedy Heuristic functionality
 * (Happy Cases)
 **************************************/
BOOST_AUTO_TEST_SUITE( TS_04_Greedy_Heuristic_Happy  )

BOOST_AUTO_TEST_CASE( tc_04_01_greedyFirstStep_L_Set_E_Set ){
  string graph = LGF_PATH + "test_example2.lgf";
  Network network(graph);

  double ratio = 1;
  Node p0 = (network.g).nodeFromId(3);
  NodeSet nSet;
  nSet.insert(p0);

  /*
   * Check L set
   *  for node #3
   *  [4, 5]
   *  [2, 1]
   */
  (network.greedyH)->makeLSet(nSet);
  UnprotectedSDSet::iterator lIt;
  bool isSourceOK = true;
  bool isDestOK = true;
  for (lIt=(network.greedyH)->L.begin(); lIt != (network.greedyH)->L.end(); ++lIt){
    if ( (network.g).id(lIt->source) != 4 &&
         (network.g).id(lIt->source) != 2   )
           isSourceOK = false;
    if ( network.g.id(lIt->destination) != 5 &&
         network.g.id(lIt->destination) != 1      )
           isDestOK = false;
    }
    BOOST_CHECK( isSourceOK == 1 );
    BOOST_CHECK( isDestOK == 1 );

    /*
     * Check E set
     *  for node #3
     *  [4, 5] => (5)
     *  [2, 1] => ( )
     */
    (network.greedyH)->makeESet(nSet);
    ET_Iterator eIt;
    NodeSet::iterator nSetIt;
    bool isEscapeOK = true;
    for ( eIt = (network.greedyH)->E.begin(); eIt != (network.greedyH)->E.end(); ++eIt )
      for ( nSetIt = (eIt->second).begin(); nSetIt != (eIt->second).end(); ++nSetIt )
        if ( network.g.id(*nSetIt) != 5 ) isEscapeOK = false;

        BOOST_CHECK( isEscapeOK == 1 );

        //cleanup memory
        (network.greedyH)->L.clear();

        for (eIt = (network.greedyH)->E.begin(); eIt != (network.greedyH)->E.end(); ++eIt){
          eIt->second.clear();
        }
        (network.greedyH)->E.clear();
}

BOOST_AUTO_TEST_CASE( tc_04_02_twoNodeGreedy_1_2 ){
  string graph = LGF_PATH + "test_example2.lgf";
  Network network(graph);
  Graph *g = &network.g;
  Network* netwP = &network;

  NodeSet nSet;
  Node v1 = (network.g).nodeFromId(1);
  Node v2 = (network.g).nodeFromId(2);

  //network.greedyH->printOutUnprotectedSDSet();

  nSet.insert(v1);
  nSet.insert(v2);
  network.greedyH->makeLSet(nSet);
  //network.greedyH->printLSet();

  network.greedyH->makeESet(nSet);
  //network.greedyH->printESet();

  ET_Iterator eIt;
  NodeSet::iterator nSetIt;
  bool isOK = true;
  for ( eIt = (network.greedyH)->E.begin(); eIt != (network.greedyH)->E.end(); ++eIt )
    if (eIt->second.size() != 2 && eIt->second.size() != 0){
      isOK = false;
      cout<<"Size of ["<<g->id(eIt->first.source)<<", "<<
        g->id(eIt->first.destination)<<"]: "<<eIt->second.size()<<endl;
    }

  BOOST_CHECK( isOK == 1 );

  network.greedyH->makeQSet(nSet);
  //network.greedyH->printQSet();

  network.greedyH->makeTSet(nSet);
  //network.greedyH->printTSet();

  network.greedyH->cleanSets();
}

BOOST_AUTO_TEST_CASE( tc_04_03_twoNodeGreedy_2_3 ){
  string graph = LGF_PATH + "test_example2.lgf";
  Network network(graph);
  Graph *g = &network.g;
  Network* netwP = &network;

  NodeSet nSet;
  Node v3 = (network.g).nodeFromId(3);
  Node v2 = (network.g).nodeFromId(2);

  //network.greedyH->printOutUnprotectedSDSet();

  nSet.insert(v3);
  nSet.insert(v2);
  network.greedyH->makeLSet(nSet);
  //network.greedyH->printLSet();

  network.greedyH->makeESet(nSet);
  //network.greedyH->printESet();

  //[4,5]=>(5) is valid only
  ET_Iterator eIt;
  NodeSet::iterator nSetIt;
  bool isOK = true;
  for ( eIt = (network.greedyH)->E.begin(); eIt != (network.greedyH)->E.end(); ++eIt )
    if (eIt->second.size() != 1 && eIt->second.size() != 0){
      isOK = false;
      cout<<"Size of ["<<g->id(eIt->first.source)<<", "<<
        g->id(eIt->first.destination)<<"]: "<<eIt->second.size()<<endl;
    }

  BOOST_CHECK( isOK == 1 );

  network.greedyH->makeQSet(nSet);
  //network.greedyH->printQSet();

  network.greedyH->makeTSet(nSet);
  //network.greedyH->printTSet();

  network.greedyH->cleanSets();
}

BOOST_AUTO_TEST_CASE( tc_04_04_twoNodeGreedy_3_4 ){
  string graph = LGF_PATH + "test_example2.lgf";
  Network network(graph);
  Graph *g = &network.g;
  Network* netwP = &network;

  NodeSet nSet;
  Node v3 = (network.g).nodeFromId(3);
  Node v4 = (network.g).nodeFromId(4);

  //network.greedyH->printOutUnprotectedSDSet();

  nSet.insert(v3);
  nSet.insert(v4);
  network.greedyH->makeLSet(nSet);
  //network.greedyH->printLSet();

  network.greedyH->makeESet(nSet);
  //network.greedyH->printESet();

  /******************
   * Valid escapes:
   * [2,1]=>(0 1)
   * [3,1]=>(0,1)
   * [3,2]=>(0,1)
   * [4,5]=>(5)
   ******************/
  ET_Iterator eIt;
  NodeSet::iterator nSetIt;
  bool isOK = true;
  for ( eIt = (network.greedyH)->E.begin(); eIt != (network.greedyH)->E.end(); ++eIt )
    if (eIt->second.size() != 2 && eIt->second.size() != 1){
      isOK = false;
      cout<<"Size of ["<<g->id(eIt->first.source)<<", "<<
        g->id(eIt->first.destination)<<"]: "<<eIt->second.size()<<endl;
    }

  BOOST_CHECK( isOK == 1 );

  network.greedyH->makeQSet(nSet);
  //network.greedyH->printQSet();

  network.greedyH->makeTSet(nSet);
  //network.greedyH->printTSet();

  network.greedyH->cleanSets();
}

BOOST_AUTO_TEST_CASE( tc_04_05_twoNodeGreedy_4_5 ){
  string graph = LGF_PATH + "test_example2.lgf";
  Network network(graph);
  Graph *g = &network.g;
  Network* netwP = &network;

  NodeSet nSet;
  Node v4 = (network.g).nodeFromId(4);
  Node v5 = (network.g).nodeFromId(5);

  //network.greedyH->printOutUnprotectedSDSet();

  nSet.insert(v4);
  nSet.insert(v5);
  network.greedyH->makeLSet(nSet);
  //network.greedyH->printLSet();

  network.greedyH->makeESet(nSet);
  //network.greedyH->printESet();

  /******************
   * Valid escapes:
   * [3,1]=>(0,1)
   * [3,2]=>(1)
   ******************/
  ET_Iterator eIt;
  bool isOK = true;
  for ( eIt = (network.greedyH)->E.begin(); eIt != (network.greedyH)->E.end(); ++eIt )
    if (eIt->second.size() != 2 && eIt->second.size() != 1 ){
      isOK = false;
      cout<<"Size of ["<<g->id(eIt->first.source)<<", "<<
        g->id(eIt->first.destination)<<"]: "<<eIt->second.size()<<endl;
    }

  BOOST_CHECK( isOK == 1 );

  network.greedyH->makeQSet(nSet);
  //network.greedyH->printQSet();

  /******************
   * Valid traps:
   * [3,1]=>(3,4,5)
   * [3,2]=>(0,3,4,5)
   ******************/
  ET_Iterator tIt;
  network.greedyH->makeTSet(nSet);
  //network.greedyH->printTSet();
  //for ( tIt = (network.greedyH)->T.begin(); tIt != (network.greedyH)->T.end(); ++tIt )
  //  if (tIt->second.size() != 3 && eIt->second.size() != 4 && eIt->second.size() != 0 ){
  //    isOK = false;
  //    cout<<"Size of ["<<g->id(tIt->first.source)<<", "<<
  //    g->id(tIt->first.destination)<<"]: "<<tIt->second.size()<<endl;
  //  }
  //BOOST_CHECK( isOK == 1 );

  network.greedyH->cleanSets();
}

BOOST_AUTO_TEST_CASE( tc_04_06_twoNodeGreedy_4_0 ){
  string graph = LGF_PATH + "test_example2.lgf";
  Network network(graph);
  Graph *g = &network.g;
  Network* netwP = &network;

  NodeSet nSet;
  Node v4 = (network.g).nodeFromId(4);
  Node v0 = (network.g).nodeFromId(0);

  //network.greedyH->printOutUnprotectedSDSet();

  nSet.insert(v4);
  nSet.insert(v0);
  network.greedyH->makeLSet(nSet);
  //network.greedyH->printLSet();

  network.greedyH->makeESet(nSet);
  //network.greedyH->printESet();

  /******************
   * Valid escapes:
   * [3,1]=>(0,1)
   * [3,2]=>(1)
   * [4,5]=>()
   ******************/
  ET_Iterator eIt;
  NodeSet::iterator nSetIt;
  bool isOK = true;
  for ( eIt = (network.greedyH)->E.begin(); eIt != (network.greedyH)->E.end(); ++eIt )
    if (eIt->second.size() != 2 && eIt->second.size() != 1 && eIt->second.size() != 0){
      isOK = false;
      cout<<"Size of ["<<g->id(eIt->first.source)<<", "<<
        g->id(eIt->first.destination)<<"]: "<<eIt->second.size()<<endl;
    }

  BOOST_CHECK( isOK == 1 );

  network.greedyH->makeQSet(nSet);
  //network.greedyH->printQSet();

  network.greedyH->makeTSet(nSet);
  //network.greedyH->printTSet();

  //network.greedyH->cleanSets();
}

BOOST_AUTO_TEST_CASE( tc_04_07_twoNodeGreedy_4_1 ){
  string graph = LGF_PATH + "test_example2.lgf";
  Network network(graph);
  Graph *g = &network.g;
  Network* netwP = &network;

  NodeSet nSet;
  Node v4 = (network.g).nodeFromId(4);
  Node v1 = (network.g).nodeFromId(1);

  //network.greedyH->printOutUnprotectedSDSet();

  nSet.insert(v4);
  nSet.insert(v1);
  network.greedyH->makeLSet(nSet);
  //network.greedyH->printLSet();

  network.greedyH->makeESet(nSet);
  //network.greedyH->printESet();

  /******************
   * Valid escapes:
   * [3,1]=>(0,2)
   * [3,2]=>(2)
   * [2,3]=>(0,3,5)
   * [2,4]=>(0,3,5)
   * [2,5]=>(0,3,5)
   ******************/
  ET_Iterator eIt;
  NodeSet::iterator nSetIt;
  bool isOK = true;
  for ( eIt = (network.greedyH)->E.begin(); eIt != (network.greedyH)->E.end(); ++eIt )
    if (eIt->second.size() != 1 &&
        eIt->second.size() != 2 && eIt->second.size() != 3 ){
      isOK = false;
      cout<<"Size of ["<<g->id(eIt->first.source)<<", "<<
        g->id(eIt->first.destination)<<"]: "<<eIt->second.size()<<endl;
    }

  BOOST_CHECK( isOK == 1 );

  network.greedyH->makeQSet(nSet);
  //network.greedyH->printQSet();

  network.greedyH->makeTSet(nSet);
  //network.greedyH->printTSet();

  network.greedyH->cleanSets();
}

BOOST_AUTO_TEST_CASE( tc_04_08a_twoNodeGreedy_checkUniformSets ){
  string graph = LGF_PATH + "test_example1.lgf";
  Network network(graph);
  Graph *g = &network.g;
  Network* netwP = &network;

  network.buildUniformSetOfNodes(1);
  //network.printVirtCandidateNodeSets();

  BOOST_CHECK( network.virtCandidates.size() == 4);

  network.buildUniformSetOfNodes(2);
  //network.printVirtCandidateNodeSets();;

  BOOST_CHECK( network.virtCandidates.size() == 8);

  network.buildUniformSetOfNodes(3);
  //network.printVirtCandidateNodeSets();

  BOOST_CHECK( network.virtCandidates.size() == 12);

  network.buildUniformSetOfNodes(4);
  //network.printVirtCandidateNodeSets();

  BOOST_CHECK( network.virtCandidates.size() == 13);
}

BOOST_AUTO_TEST_CASE( tc_04_08b_twoNodeGreedy_checkUniformSets ){
  string graph = LGF_PATH + "test_example2.lgf";
  Network network(graph);
  Graph *g = &network.g;
  Network* netwP = &network;

  network.buildUniformSetOfNodes(1);
  //network.printVirtCandidateNodeSets();

  BOOST_CHECK( network.virtCandidates.size() == 6);

  network.buildUniformSetOfNodes(2);
  //network.printVirtCandidateNodeSets();

  BOOST_CHECK(network.virtCandidates.size()  == 14);

  network.buildUniformSetOfNodes(3);
  //network.printVirtCandidateNodeSets();

  BOOST_CHECK( network.virtCandidates.size() == 25);

  network.buildUniformSetOfNodes(4);
  //network.printVirtCandidateNodeSets();

  BOOST_CHECK( network.virtCandidates.size()  == 36);
}

BOOST_AUTO_TEST_CASE( tc_04_08c_twoNodeGreedy_checkUniformSets ){
  string graph = LGF_PATH + "six_cyc.lgf";
  Network network(graph);
  Graph *g = &network.g;
  Network* netwP = &network;

  network.buildUniformSetOfNodes(1);
  //network.printVirtCandidateNodeSets();

  BOOST_CHECK( network.virtCandidates.size() == 6);

  network.buildUniformSetOfNodes(2);
  //network.printVirtCandidateNodeSets();

  BOOST_CHECK( network.virtCandidates.size() == 12);

  network.buildUniformSetOfNodes(3);
  //network.printVirtCandidateNodeSets();

  BOOST_CHECK( network.virtCandidates.size() == 18);

  network.buildUniformSetOfNodes(4);
  //network.printVirtCandidateNodeSets();

  BOOST_CHECK( network.virtCandidates.size() == 24);
}

BOOST_AUTO_TEST_CASE( tc_04_11_twoNodeGreedy_solves_counter_example ){
  string graph = LGF_PATH + "counter_example.lgf";
  Network network(graph);
  Graph *g = &network.g;
  Network* netwP = &network;
  int ratio = 1;

  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, false, true, true, CONN_L};
  network.greedyH->setGreedyParamSet(gPS);
  network.greedyH->greedyHeuristicAlgorithm(ratio, NODESET_SIZE);

  double finalLfaCov = network.newLfaCoverageMetric_LP(LP);

  BOOST_CHECK(countNodes(netwP->g)-netwP->numOfPhysicalNodes == 2);

  BOOST_CHECK_CLOSE( finalLfaCov, 1, 0.01 );

  // check that in case of tunnels we connect those
  // neighbours that are not in the virtualized set or
  // their physical instances
  // E set: [2,3]
  // low  cost arcs : (6,5); (5,3);
  // high cost arcs : (2,6); (4,6); (4,5);

  BOOST_CHECK(countArcs(netwP->g)-netwP->numOfPhysicalArcs == 10);
}

/*
 * If virutalizing 2 nodes paralelly,
 * when creating E set we need to take care
 * not to virtualize "s". This would guarantee
 * that in case of failure the packet would
 * return to "s", and we want to avoid that.
 *
 */
BOOST_AUTO_TEST_CASE( tc_04_11a_s_is_no_virt_when_2_paralel_v ){
  string graph = LGF_PATH + "test_example1.lgf";
  Network network(graph);
  Graph *g = &network.g;

  NodeSet nSet;
  Node toV3 = (network.g).nodeFromId(3);
  Node toV0 = (network.g).nodeFromId(0);
  nSet.insert(toV0);
  nSet.insert(toV3);

  /*
   * Only [2,1] is valid
   */
  network.greedyH->makeLSet(nSet);
  //network.greedyH->printLSet();

  UnprotectedSDSet::iterator lIt;
  bool sourceIsVirtualized = false;
  for (lIt=(network.greedyH)->L.begin(); lIt != (network.greedyH)->L.end(); ++lIt){
    if ( (network.g).id(lIt->source) == 0 ||
         (network.g).id(lIt->source) == 3 )
      sourceIsVirtualized == true;
  }

  BOOST_CHECK( sourceIsVirtualized == 0 );

  network.greedyH->cleanSets();
}

/*
 * Same case then previous, but it can happen
 * that source from unprotected SD pair can
 * not be seen in vS, because parent of vS item
 * is also virtual and it will not match with
 * the source. (Check vSParentS in code vSParentS).
 */
BOOST_AUTO_TEST_CASE( tc_04_11b_s_is_no_virt_when_2_paralel_v ){
  string graph = LGF_PATH + "test_example1.lgf";
  Network network(graph);
  Graph *g = &network.g;

  NodeSet nSet;
  Node toV3 = (network.g).nodeFromId(3);
  Node toV0 = (network.g).nodeFromId(0);
  nSet.insert(toV0);
  nSet.insert(toV3);

  Node esc1 = network.g.nodeFromId(1);

  // realize these two nodes
  network.greedyH->realizeVirtualNode(nSet, esc1);

  /*
   * L set should be empty, however unprotSD set is not
   */
  network.greedyH->makeLSet(nSet);

  BOOST_CHECK( network.greedyH->L.size() == 0 );

  network.greedyH->cleanSets();
}

BOOST_AUTO_TEST_CASE( tc_04_12_make_tunnel_with_skipEdge ){
  string graph = LGF_PATH + "test_tunnel_limit.lgf";
  Network network(graph);
  Graph *g = &network.g;

  NodeSet nSet;
  Node toV1 = (network.g).nodeFromId(1);
  Node toV0 = (network.g).nodeFromId(0);
  nSet.insert(toV0);
  nSet.insert(toV1);

  Node escN = network.g.nodeFromId(2);
  network.greedyH->realizeVirtualNode(nSet, escN);

  BOOST_CHECK(countArcs(network.g)-network.numOfPhysicalArcs == 6);

  //network.printGraph();
}

/*
 * Checks Shortest Path based virtualization
 * init step. Are node sets created correctly?
 * Creating max 4 long sets.
 */
BOOST_AUTO_TEST_CASE( tc_04_13_check_buildPathSetOfNodes ){
  string graph = LGF_PATH + "test_example2.lgf";
  Network network(graph);
  Graph *g = &network.g;

  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, false, true, false, SPS};
  network.greedyH->setGreedyParamSet(gPS);

  network.calculateNodeSets(NODESET_SIZE);
  // Result of shortest paths should be (with removed dest):
  // ( 1 )( 2 )( 0 )( 3 )( 5 )( 4 )
  // ( 0 1 )( 1 2 )( 4 5 )( 2 3 )( 0 4 )( 3 4 )
  // ( 3 4 5 )( 1 2 3 )( 2 3 4 ) ( 3 4 0 ) ( 0 4 5 ) ( 2 1 0 )
  // ( 2 3 4 5 ) ( 1 2 3 4 )
  // ( 1 2 3 4 5 ) -> do not calculate due to speed reasons
  // Total: 21

  //network.printVirtCandidateNodeSets();
  BOOST_CHECK( network.virtCandidates.size() == 20);
}

/*
 * Same check as above but with different topology
 */
BOOST_AUTO_TEST_CASE( tc_04_14_check_buildPathSetOfNodes ){
  string graph = LGF_PATH + "test_example1.lgf";
  Network network(graph);
  Graph *g = &network.g;

  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, false, true, false, SPS};
  network.greedyH->setGreedyParamSet(gPS);

  network.calculateNodeSets(NODESET_SIZE);
  // Result should be:
  // ( 1 )( 3 )( 0 )( 2 )
  // ( 2 3 )( 0 3 )( 1 2 )
  // ( 1 2 3 )( 0 2 3 )
  // ( 1 2 3 0 )
  // Total: 10

  //network.printPathSetOfNodes();
  BOOST_CHECK( network.virtCandidates.size() == 10);
}

/*
 * Check how SPT Based Virtalization works
 * test_example2 can be solved with 4 nodes
 */
BOOST_AUTO_TEST_CASE( tc_04_15_check_SPTBased_Virt ){
  string graph = LGF_PATH + "test_example2.lgf";
  Network network(graph);
  Graph *g = &network.g;

  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, false, true, true, SPS};
  network.greedyH->setGreedyParamSet(gPS);

  network.greedyH->greedyHeuristicAlgorithm(1, NODESET_SIZE);
  // 4 nodes are enough for 1 coverage:
  //  v: 1; Esc: 0
  //  v: 4; Esc: 1
  //  v: 3; Esc: 5
  //  v: 3; Esc: 7

  int numOfVirtNodes = countNodes(network.g)-network.numOfPhysicalNodes;
  BOOST_CHECK( numOfVirtNodes == 4);
}

/*
 * Check how SPT Based Virtalization works
 * counter_example can be solved with 2 nodes
 */
BOOST_AUTO_TEST_CASE( tc_04_16_check_SPTBased_Virt ){
  string graph = LGF_PATH + "counter_example.lgf";
  Network network(graph);
  Graph *g = &network.g;

  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, false, true, true, SPS};
  network.greedyH->setGreedyParamSet(gPS);

  network.greedyH->greedyHeuristicAlgorithm(1, NODESET_SIZE);
  // 2 nodes are enough for 1 coverage:
  //  v: 0,4 ; Esc: 3

  int numOfVirtNodes = countNodes(network.g)-network.numOfPhysicalNodes;
  BOOST_CHECK( numOfVirtNodes == 2);
}

/*
 * Check if all SRLG sets are constructed properly
 * when classic greedyHeuristic finished.
 */
BOOST_AUTO_TEST_CASE( tc_04_17_check_fillLocalSRLGs ){
  string graph = LGF_PATH + "test_example2.lgf";
  Network network(graph);
  Graph *g = &network.g;
  NodeInfoMap *nodeInfo = network.nodeInfo;

  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, false, true, true, CONN_L};
  network.greedyH->setGreedyParamSet(gPS);

  network.greedyH->greedyHeuristicAlgorithm(2, NODESET_SIZE);

  //network.greedyH->printLocalSRLGs();
  for (NodeIt n(*g); n != INVALID; ++n){
    SRGSet srlg = (*nodeInfo)[n].localSRGs;

    if ( g->id(n) == 5 ) BOOST_CHECK( srlg.size() == 2 );
    if ( g->id(n) == 4 ) BOOST_CHECK( srlg.size() == 2 );
    if ( g->id(n) == 3 ) BOOST_CHECK( srlg.size() == 1 );
    if ( g->id(n) == 2 ) BOOST_CHECK( srlg.size() == 2 );
    if ( g->id(n) == 1 ) BOOST_CHECK( srlg.size() == 1 );
    if ( g->id(n) == 0 ) BOOST_CHECK( srlg.size() == 2 );
  }
}

/*
 * There were two problems with bunch of virtual layers.
 * First if we connect virtual nodes to its neighbours,
 * the physical neighborhood has to be considered (mod in
 * realizeVirtualNode). Second problem was with Q set
 * generation, physical neighborhood has to be considered
 * here as well (mod in makeQSet)
 */
BOOST_AUTO_TEST_CASE( tc_04_18_classic_heu_cycle_reach_1 ){
  string graph = LGF_PATH + "six_cyc.lgf";
  Network network(graph);
  Graph *g = &network.g;

  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, false, true, true, CONN_L};
  network.greedyH->setGreedyParamSet(gPS);

  network.greedyH->greedyHeuristicAlgorithm(2, NODESET_SIZE);
  double finalLfaCov = network.newLfaCoverageMetric_LP(LP);
  bool perfCoverage = network.equal(finalLfaCov, 1);
  bool emptyUnprotSet = network.greedyH->unprotSDs.empty();

  BOOST_CHECK( (perfCoverage && emptyUnprotSet) == 1);
}


BOOST_AUTO_TEST_SUITE_END()

/**************************************
 * Testing Greedy ILP
 **************************************/
BOOST_AUTO_TEST_SUITE( TS_05_GreedyILP )

//
// Below testcase was born when big topology was
// debugged. It can be used as a skeleton for
// other cases but keep in mind that calculateNodeSets
// function call should be commented out from callILPgreedily
// function otherwise it will calculate the inserted
// virtual nodes in nodesets.
//
/*BOOST_AUTO_TEST_CASE( tc_05_debug_china ){
 string graph = "../lgf_files/directed_graphs/n020_e0044_ChinaTelecom.lgf";

 Network network(graph);
 Network* netwP = &network;
 double ratio = 2;

 //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
 GreedyParamSet gPS = {false, false, true, true, CONN_L};
 network.greedyH->setGreedyParamSet(gPS);

 GreedyILP greedyILP(netwP);
 netwP->calculateNodeSets();

 //=== Adding node 20 on top of node 0 ===
 Node n20 = netwP->g.nodeFromId(0);
 NodeSet nSet; nSet.insert(n20);
 Node n18 = netwP->g.nodeFromId(18);
 Node n17 = netwP->g.nodeFromId(17);
 Node n14 = netwP->g.nodeFromId(14);
 Node n11 = netwP->g.nodeFromId(11);
 Node n8  = netwP->g.nodeFromId(8);
 Node n6  = netwP->g.nodeFromId(6);
 INodeMap xwin(netwP->g, 0);
 xwin[n18] = 1;
 xwin[n17] = 1;
 xwin[n14] = 1;
 xwin[n11] = 1;
 xwin[n8]  = 1;
 xwin[n6]  = 1;
 DNodeMap cwin(netwP->g, 0);
 cwin[n18] = 157;
 cwin[n17] = 157;
 cwin[n14] = 158;
 cwin[n11] = 157;
 cwin[n8]  = 152;
 cwin[n6]  = 151;

 greedyILP.realizeVirtualNode(nSet,xwin,cwin);
  cout<<"LFA coverage:"<<network.newLfaCoverageMetric_LP(LP)<<endl;
 //=== Adding node 21 on top of node 18 ===
 Node n21 = netwP->g.nodeFromId(18);
 nSet.clear(); nSet.insert(n21);
 Node n16 = netwP->g.nodeFromId(16);
 //Node n14 = netwP->g.nodeFromId(14);
 Node n12 = netwP->g.nodeFromId(12);
 Node n7 = netwP->g.nodeFromId(7);
 Node n0  = netwP->g.nodeFromId(0);

 for (NodeIt n(netwP->g); n != INVALID; ++n){
   xwin[n] = 0;
   cwin[n] = 0;
 }
 xwin[n16] = 1;
 xwin[n14] = 1;
 xwin[n12] = 1;
 xwin[n7]  = 1;
 xwin[n0]  = 1;

 cwin[n16] = 157;
 cwin[n14] = 153;
 cwin[n12] = 79;
 cwin[n7]  = 157;
 cwin[n0]  = 157;

 greedyILP.realizeVirtualNode(nSet,xwin,cwin);
 cout<<"LFA coverage:"<<network.newLfaCoverageMetric_LP(LP)<<endl;
//=== Adding node 22 and 23 on top of node 1 and 10 ===
 Node n22 = netwP->g.nodeFromId(1);
 Node n23 = netwP->g.nodeFromId(10);
 nSet.clear(); nSet.insert(n22); nSet.insert(n23);
 //for 22:
 Node n13 = netwP->g.nodeFromId(13);
 Node n3 = netwP->g.nodeFromId(3);
 Node n2 = netwP->g.nodeFromId(2);
 //for 23:
 Node n19 = netwP->g.nodeFromId(19);
 //Node n18 = netwP->g.nodeFromId(18);
 Node n9  = netwP->g.nodeFromId(9);

 for (NodeIt n(netwP->g); n != INVALID; ++n){
   xwin[n] = 0;
   cwin[n] = 0;
 }
 xwin[n13] = 1;
 xwin[n8]  = 1;
 xwin[n6]  = 1;
 xwin[n3]  = 1;
 xwin[n2]  = 1;
 xwin[n19] = 1;
 xwin[n18] = 1;
 xwin[n9]  = 1;
 xwin[n6]  = 1;

 cwin[n13] = 157;
 cwin[n8]  = 157;
 cwin[n6]  = 159;
 cwin[n3]  = 157;
 cwin[n2]  = 157;
 cwin[n19] = 158;
 cwin[n18] = 159;
 cwin[n9]  = 157;
 cwin[n6]  = 159;

 greedyILP.realizeVirtualNode(nSet,xwin,cwin);
 network.greedyH->printOutUnprotectedSDSet();

 cout<<"LFA coverage:"<<network.newLfaCoverageMetric_LP(LP)<<endl;
 //=== Adding node 24 on top of node 8 ===
 Node n24 = netwP->g.nodeFromId(8);
 nSet.clear(); nSet.insert(n24);
 n22 = netwP->g.nodeFromId(22);
 for (NodeIt m(netwP->g); m != INVALID; ++m){
   xwin[m] = 0;
   cwin[m] = 0;
 }

 xwin[n18]  = 1;
 xwin[n22]  = 1;

 cwin[n18]  = 315;
 cwin[n22]  = 158;

 greedyILP.realizeVirtualNode(nSet,xwin,cwin);
 network.greedyH->printOutUnprotectedSDSet();
 cout<<"LFA coverage:"<<network.newLfaCoverageMetric_LP(LP)<<endl;

 int numOfVirtNodes = countNodes(netwP->g)-netwP->numOfPhysicalNodes;
 BOOST_CHECK( numOfVirtNodes == 5 );

 int numOfVirtArcs = countArcs(netwP->g)-netwP->numOfPhysicalArcs;
 BOOST_CHECK( numOfVirtArcs == 48 );

 cout<<"# virtual arcs: "<<numOfVirtArcs<<endl;
 //network.printGraph();

 netwP->db->initializeDB();
 netwP->greedyH->fillNodeInfoLocalSrgTables();
 netwP->greedyH->buildUnprotectedSDSet();

 greedyILP.callILPGreedily(ratio);

 double finalLfaCov = network.newLfaCoverageMetric_LP(LP);
 BOOST_CHECK_CLOSE( finalLfaCov, 1, 0.01 );

 }*/
/*
 * ILP solves test_example2 with 3 nodes
 * while greedy needs 4 virtual nodes.
 * See tc_02_02_greedy_test_example2
 */
BOOST_AUTO_TEST_CASE( tc_05_01_basic_ILP_test ){
  string graph = LGF_PATH + "test_example2.lgf";

  Network network(graph);
  Network* netwP = &network;
  double ratio = 2;

  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, false, true, false, CONN_L};
  network.greedyH->setGreedyParamSet(gPS);

  GreedyILP greedyILP(netwP);
  greedyILP.callILPGreedily(ratio, NODESET_SIZE);

  //virtual nodes
  // To virtualize: 1
  //   unprotected: (4,5) (3,2) (3,1) (2,1)
  // To virtualize: 4
  //   unprotected: (4,5) (2,1)
  // To virtualize: 3
  //   unprotected: empty
  int numOfVirtNodes = countNodes(netwP->g)-netwP->numOfPhysicalNodes;
  BOOST_CHECK( numOfVirtNodes == 3 );

  int numOfVirtArcs = countArcs(netwP->g)-netwP->numOfPhysicalArcs;
  BOOST_CHECK( numOfVirtArcs == 16 );
}

/*
 * ILP solves counter_example with 2 nodes and
 * inner link cost is 1
 */
BOOST_AUTO_TEST_CASE( tc_05_02_basic_ILP_test ){
  string graph = LGF_PATH + "counter_example.lgf";

  Network network(graph);
  Network* netwP = &network;
  double ratio = 2;

  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, false, true, true, CONN_L};
  network.greedyH->setGreedyParamSet(gPS);

  GreedyILP greedyILP(netwP);
  greedyILP.callILPGreedily(ratio, NODESET_SIZE);

  // virtual nodes:
  //  To virtualize: 4 0
  //   unprotected: (2,3)
  //
  // virtual links:
  //  5-6: 1
  //  5-3: ?
  //  6-2: ?
  Node n5 = network.g.nodeFromId(5);
  Node n6 = network.g.nodeFromId(6);
  Arc innerArc = findArc(network.g, n5, n6);
  BOOST_CHECK( (*(network.cost))[innerArc] == 1 );

  int numOfVirtNodes = countNodes(netwP->g)-netwP->numOfPhysicalNodes;
  BOOST_CHECK( numOfVirtNodes == 2 );

  int numOfVirtArcs = countArcs(netwP->g)-netwP->numOfPhysicalArcs;
  BOOST_CHECK( numOfVirtArcs == 6 );

  //network.printGraph();
}

/*
 * ILP solves counter_example with 3 nodes
 */
BOOST_AUTO_TEST_CASE( tc_05_03_basic_ILP_test ){
  parseDebugLevel(boost::unit_test::framework::master_test_suite().argc,
                  boost::unit_test::framework::master_test_suite().argv);
  string graph = LGF_PATH + "counter_example_3.lgf";

  Network network(graph);
  Network* netwP = &network;
  double ratio = 2;

  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, false, true, true, CONN_L};
  network.greedyH->setGreedyParamSet(gPS);

  GreedyILP greedyILP(netwP);
  greedyILP.callILPGreedily(ratio, NODESET_SIZE);

  // virtual nodes:
  //  To virtualize: X X X
  //   unprotected: (3,4)
  //
  // virtual links:
  //  5-6: 0.1
  //  5-3: 11
  //  6-2: 11

  int numOfVirtNodes = countNodes(netwP->g)-netwP->numOfPhysicalNodes;
  BOOST_CHECK( numOfVirtNodes == 3 );

  int numOfVirtArcs = countArcs(netwP->g)-netwP->numOfPhysicalArcs;
  BOOST_CHECK( numOfVirtArcs == 8 );

  //network.printGraph();
}

/*
 * ILP solves counter_example with 3 nodes with SPT slices
 *
 */
BOOST_AUTO_TEST_CASE( tc_05_04_basic_ILP_test ){
  string graph = LGF_PATH + "counter_example_3.lgf";

  Network network(graph);
  Network* netwP = &network;
  double ratio = 2;

  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, false, true, true, SPS};
  network.greedyH->setGreedyParamSet(gPS);

  GreedyILP greedyILP(netwP);
  greedyILP.callILPGreedily(ratio, NODESET_SIZE);

  // virtual nodes:
  //  To virtualize: X X X
  //   unprotected: (3,4)
  //
  // virtual links:
  //  5-6: 0.1
  //  5-3: 11
  //  6-2: 11

  int numOfVirtNodes = countNodes(netwP->g)-netwP->numOfPhysicalNodes;
  BOOST_CHECK( numOfVirtNodes == 3 );

  int numOfVirtArcs = countArcs(netwP->g)-netwP->numOfPhysicalArcs;
  BOOST_CHECK( numOfVirtArcs == 8 );

  //network.printGraph();
}

/*
 * Check if modified ILP solves four long cycle
 */
BOOST_AUTO_TEST_CASE( tc_05_05_improved_ILP_check_four_cyc ){
  string graph = LGF_PATH + "four_cyc.lgf";

  Network network(graph);
  Network* netwP = &network;
  double ratio = 2;

  //(skipEdges, cascade, skipUpstream, smartTrap, nSelStrategy)
  GreedyParamSet gPS = {false, false, true, true, CONN_L};
  network.greedyH->setGreedyParamSet(gPS);

  GreedyILP greedyILP(netwP);
  greedyILP.callILPGreedily(ratio, NODESET_SIZE);

  int numOfVirtNodes = countNodes(netwP->g)-netwP->numOfPhysicalNodes;
  BOOST_CHECK( numOfVirtNodes == 8 );

  int numOfVirtArcs = countArcs(netwP->g)-netwP->numOfPhysicalArcs;
  BOOST_CHECK( numOfVirtArcs == 32 );

}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE( TS_06_RedundantTree )

BOOST_AUTO_TEST_CASE( tc_06_01_DepthFirstSearch ){
  string graph = LGF_PATH + "test_example2.lgf";
  Network network(graph);
  Graph *g = &network.g;
  Network* netwP = &network;

  Node root = (network.g).nodeFromId(4);
  RedundantTree<Graph> rtManager(g);
  rtManager.fillDfsAndLowPoint(root);

  // Verify a DFS tree
  // order should be (it can change):
  // 4->1->2->3->5->0

  Node null = (network.g).nodeFromId(0);
  Node one = (network.g).nodeFromId(1);
  Node two = (network.g).nodeFromId(2);
  Node three = (network.g).nodeFromId(3);
  Node five = (network.g).nodeFromId(5);

  BOOST_CHECK( rtManager.getDFSNumber(five)  == 4 );
  BOOST_CHECK( rtManager.getDFSNumber(one)   == 1 );
  BOOST_CHECK( rtManager.getDFSNumber(two)   == 2 );
  BOOST_CHECK( rtManager.getDFSNumber(three) == 3 );
  BOOST_CHECK( rtManager.getDFSNumber(null)  == 5 );
  BOOST_CHECK( rtManager.getDFSNumber(root)  == 0 );

}

BOOST_AUTO_TEST_CASE( tc_06_02_ADAG_on_Enyedi_graph_root_0 ){
  string graph = LGF_PATH + "enyedi_test_graph.lgf";
  Network network(graph);
  Graph *g = &network.g;
  Network *netwP = &network;

  //root := 0
  Node root = (network.g).nodeFromId(0);
  RedundantTree<Graph> rtManager(g);
  rtManager.fillDfsAndLowPoint(root);

  rtManager.setupADAG(root);

  Graph::ArcMap<bool> validArcMap(network.g, false);

  //set reference map:
  Arc arc1 = (network.g).arcFromId(1);
  validArcMap[arc1] = true;
  Arc arc3 = (network.g).arcFromId(3);
  validArcMap[arc3] = true;
  Arc arc4 = (network.g).arcFromId(4);
  validArcMap[arc4] = true;
  Arc arc6 = (network.g).arcFromId(6);
  validArcMap[arc6] = true;
  Arc arc8 = (network.g).arcFromId(8);
  validArcMap[arc8] = true;
  Arc arc10 = (network.g).arcFromId(10);
  validArcMap[arc10] = true;
  Arc arc13 = (network.g).arcFromId(13);
  validArcMap[arc13] = true;
  Arc arc15 = (network.g).arcFromId(15);
  validArcMap[arc15] = true;
  Arc arc17 = (network.g).arcFromId(17);
  validArcMap[arc17] = true;

  for (Graph::ArcIt e(netwP->g); e!= INVALID; ++e){
        if ( validArcMap[e] == false )
          BOOST_CHECK( rtManager.isArcInADAG(e) == false);
        else
          BOOST_CHECK( rtManager.isArcInADAG(e) == true);
  }
}

BOOST_AUTO_TEST_CASE( tc_06_03_ADAG_on_Enyedi_graph_root_4 ){
        string graph = LGF_PATH + "enyedi_test_graph.lgf";
        Network network(graph);
        Graph *g = &network.g;
        Network* netwP = &network;

        //root := 4
        Node root = (network.g).nodeFromId(4);
        RedundantTree<Graph> rtManager(g);
        rtManager.fillDfsAndLowPoint(root);

        rtManager.setupADAG(root);

        Graph::ArcMap<bool> validArcMap(network.g, false);

        //set reference map:
        Arc arc0 = (network.g).arcFromId(0);
        validArcMap[arc0] = true;
        Arc arc2 = (network.g).arcFromId(2);
        validArcMap[arc2] = true;
        Arc arc5 = (network.g).arcFromId(5);
        validArcMap[arc5] = true;
        Arc arc7 = (network.g).arcFromId(7);
        validArcMap[arc7] = true;
        Arc arc9 = (network.g).arcFromId(9);
        validArcMap[arc9] = true;
        Arc arc13 = (network.g).arcFromId(13);
        validArcMap[arc13] = true;
        Arc arc15 = (network.g).arcFromId(15);
        validArcMap[arc15] = true;
        Arc arc16 = (network.g).arcFromId(16);
        validArcMap[arc16] = true;

        for (Graph::ArcIt e(netwP->g); e!= INVALID; ++e){
                if ( validArcMap[e] == false )
                  BOOST_CHECK( rtManager.isArcInADAG(e) == false);
                else
                  BOOST_CHECK( rtManager.isArcInADAG(e) == true);
        }

}

BOOST_AUTO_TEST_CASE( tc_06_04_Redundant_Trees ){
        string graph = LGF_PATH + "test_example2.lgf";
        Network network(graph);
        Graph *g = &network.g;
        Network* netwP = &network;

        for (Graph::NodeIt root(netwP->g); root!= INVALID; ++root){
                RedundantTree<Graph> rtManager(g);
                rtManager.fillDfsAndLowPoint(root);
                rtManager.setupADAG(root);
                rtManager.createREDTree(root);
                rtManager.createBLUETree(root);
                for (Graph::NodeIt n(netwP->g); n!= INVALID; ++n){
                        BOOST_CHECK( rtManager.testTrees(root, n) == true);
                }
        }
}

BOOST_AUTO_TEST_CASE( tc_06_05_Redundant_Trees ){
        string graph = LGF_PATH + "enyedi_test_graph.lgf";
        Network network(graph);
        Graph *g = &network.g;
        Network* netwP = &network;

        for (Graph::NodeIt root(netwP->g); root!= INVALID; ++root){
                RedundantTree<Graph> rtManager(g);
                rtManager.fillDfsAndLowPoint(root);
                rtManager.setupADAG(root);
                rtManager.createREDTree(root);
                rtManager.createBLUETree(root);
                for (Graph::NodeIt n(netwP->g); n!= INVALID; ++n){
                        BOOST_CHECK( rtManager.testTrees(root, n) == true);
                }
        }
}

BOOST_AUTO_TEST_CASE( tc_06_06_virtual_layers_1 ){
        string graph = LGF_PATH + "test_example1.lgf";
        Network network(graph);
        Network* netwP = &network;

        Node root = netwP->g.nodeFromId(0);
        RTBasedMethod rtMethod(netwP);
        rtMethod.buildRedundantTrees(root);
        BOOST_CHECK( rtMethod.checkTreeRedundancy(root) == true );

        rtMethod.createVirtualLayer(root, RR);
        BOOST_CHECK( countNodes(netwP->g)-netwP->numOfPhysicalNodes == 4);
        BOOST_CHECK( countArcs(netwP->g)-(netwP->numOfPhysicalArcs) == 14);

        rtMethod.createVirtualLayer(root, RB);
        BOOST_CHECK( countNodes(netwP->g)-netwP->numOfPhysicalNodes == 8);
        BOOST_CHECK( countArcs(netwP->g)-(netwP->numOfPhysicalArcs) == 28);

        rtMethod.createVirtualLayer(root, VRR);
        BOOST_CHECK( countNodes(netwP->g)-netwP->numOfPhysicalNodes == 12);
        BOOST_CHECK( countArcs(netwP->g)-(netwP->numOfPhysicalArcs) == 42);

        rtMethod.createVirtualLayer(root, VRB);
        BOOST_CHECK( countNodes(netwP->g)-netwP->numOfPhysicalNodes == 16);
        BOOST_CHECK( countArcs(netwP->g)-(netwP->numOfPhysicalArcs) == 56);
}

BOOST_AUTO_TEST_CASE( tc_06_07_virtual_layers ){
        string graph = LGF_PATH + "test_example2.lgf";
        Network network(graph);
        Network* netwP = &network;

        Node root = netwP->g.nodeFromId(0);
        RTBasedMethod rtMethod(netwP);
        rtMethod.buildRedundantTrees(root);
        BOOST_CHECK( rtMethod.checkTreeRedundancy(root) == true );
        rtMethod.createVirtualLayer(root, RR);
        rtMethod.createVirtualLayer(root, RB);
        rtMethod.createVirtualLayer(root, VRR);
        rtMethod.createVirtualLayer(root, VRB);

        BOOST_CHECK( countNodes(netwP->g)-netwP->numOfPhysicalNodes == 24);
        BOOST_CHECK( countArcs(netwP->g)-(netwP->numOfPhysicalArcs) == 88);

        //check cost settings
        Node virtNodeRR = rtMethod.isVirtualNodeExistInLayer(root, RR);
        Arc  crossArcRR = findArc(netwP->g, root, virtNodeRR);
        BOOST_CHECK( (*(network.cost))[crossArcRR] == network.lspPhy );

        Node virtNodeVRR = rtMethod.isVirtualNodeExistInLayer(root, VRR);
        Arc  crossArcVRR = findArc(netwP->g, root, virtNodeVRR);
        BOOST_CHECK( (*(network.cost))[crossArcVRR] == (2*network.lspPhy) );

        //cost within virtual layer => 1/n=0,166
        Node virtNodeRR_1 = rtMethod.isVirtualNodeExistInLayer(netwP->g.nodeFromId(1), RR);
        Arc  innerArcRR = findArc(netwP->g, virtNodeRR, virtNodeRR_1);

        BOOST_CHECK( (*(network.cost))[innerArcRR] < 0.2 );
        BOOST_CHECK( (*(network.cost))[innerArcRR] > 0 );

}

BOOST_AUTO_TEST_CASE( tc_06_08_SRLGs ){
        string graph = LGF_PATH + "test_example1.lgf";
        Network network(graph);
        Network* nwP = &network;
        NodeInfoMap *nodeInfo;
        LinkInfoMap *linkInfo;

        Node root = nwP->g.nodeFromId(0);
        RTBasedMethod rtMethod(nwP);
        rtMethod.buildRedundantTrees(root);
        BOOST_CHECK( rtMethod.checkTreeRedundancy(root) == true );
        rtMethod.buildVirtualLayers(root);

        nodeInfo = nwP->nodeInfo;
        linkInfo = nwP->linkInfo;

        for (SubDigraph<Graph>::NodeIt n(*(rtMethod.phyLayer)); n != INVALID; ++n ){
          SRGSet::iterator srgIt = (*nodeInfo)[n].localSRGs.begin();
          while (srgIt != (*nodeInfo)[n].localSRGs.end()){
            LinkSet::iterator lsIt;
            for (lsIt = srgIt->begin(); lsIt != srgIt->end(); ++lsIt){
              if ( nwP->g.id(n) == 0 ) {
                BOOST_CHECK( (*linkInfo)[*lsIt].layerID != 2 );
              }
              if ( nwP->g.id(n) == 1 ){
                BOOST_CHECK( (*linkInfo)[*lsIt].layerID != 4 );
              }
              if ( nwP->g.id(n) == 2 ){
                Arc phyArc = findArc(nwP->g, n, nwP->g.nodeFromId(3));
                if ( srgIt->count(phyArc) > 0 ){
                  BOOST_CHECK( (*linkInfo)[*lsIt].layerID != 4 );
                }
                else {
                  BOOST_CHECK( (*linkInfo)[*lsIt].layerID != 3 );
                }
              }
              if ( nwP->g.id(n) == 3 ){
                Arc phyArc = findArc(nwP->g, n, nwP->g.nodeFromId(0));
                if ( srgIt->count(phyArc) > 0 ){
                  BOOST_CHECK( (*linkInfo)[*lsIt].layerID != 2 );
                }
                else {
                  BOOST_CHECK( (*linkInfo)[*lsIt].layerID != 3 );
                }
              }
            }
            srgIt++;
           }
        }
}

BOOST_AUTO_TEST_SUITE_END()

