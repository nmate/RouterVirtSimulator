/**
 * Provisions virtual network to improve the
 * LFA coverage of a network. It can be run
 * with three different strategies:
 * - greedy heuristic
 * - greedy ILP
 * - redundant trees
 *
 * Written by Mate Nagy
 * License: GNU General Public License Version 3
 *
 */

#include <lemon/arg_parser.h>
#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <string>
#include <fstream>

#include "Network.hh"
#include "RTBasedMethod.hh"
#include "GreedyILP.hh"

/*
 * For debugging functionality this is
 * needed:
 *  - link this: lemon-1.2.3/lemon/arg_parser.o
 *  - add this define to preprocessor -DDEBUG_LOG
 */
#include "Log.h"

#define NODESET_SIZE 3

LogStream* LogStream::mInstance = NULL;
LogStream debug(std::cout, INFO);

using namespace lemon;
using namespace std;

const string XmlRes = "../result/";

void usage(){
  cout<<"==========================================="<<endl;
  cout<<"                 USAGE                     "<<endl;
  cout<<"==========================================="<<endl;
  cout<<" Try: ./RouterVirtSimulator [-v <log level>] [-xml] -r <ratio> -m <mode> graph.lgf"<<endl;
  cout<<"   -v  : log level [1-4]"<<endl;
  cout<<"   -m  : mode [1:greedyHeur 2:layers 3:greedyILP]"<<endl;
  cout<<"   -xml: generate xml file in xml directory"<<endl;
  cout<<"   -r  : ratio of virtual/physical nodes (opt, mode dependent parameter)"<<endl;
  cout<<"         1 => 0,33; 2 => 0,66"<<endl;
  cout<<"         3 => 1;    4 => 2"<<endl;
  cout<<"   -s  : Use SPT nodeset for virtualization or not [1,0]"<<endl;
  cout<<endl;
  cout<<" Two type of LGF files can be used: "<<endl;
  cout<<" 1.: Regular LGF file"<<endl;
  cout<<" 2.: Regular LGF file with virtual elements"<<endl;
  cout<<"==========================================="<<endl;
}

string stripPathNameToFileName(string& inFile){
  size_t endpos   = inFile.find_last_of('.');
  size_t firstpos = inFile.find_last_of('/');
  firstpos++;                                     // just to ignore '/' char
  inFile.erase(endpos);   // remove .lgf part
  string outFile;

  outFile = inFile.substr(firstpos, endpos);


  return outFile;
}

bool isFileExist(const string& file){
  ofstream fileStream;
  fileStream.open(file, fstream::in);

  if ( !fileStream ){
    return false;
  }
  return true;
}

string int2str(const int& number){
  ostringstream convert;  // stream used for the conversion
  convert << number;  // insert the textual representation of 'Number'

  return convert.str();
}

void writeToXml(const int &numOfVirtualNodes, double initialLfaCoverage,
                double finalLfaCoverage, string inputFile,
                int inputRatio, const double &ratio,
                const Network &network, const int &mode, const XmlStepSet &xSS,
                const bool &isSPTBasedHeu){
  /*
   * Filling xml file
   */
  ofstream xml;
  string tempName = inputFile;
  string xmlName = stripPathNameToFileName(tempName);
  string pathName;
  string algName;

  if ( mode == 1 ){
    pathName= XmlRes + "xml_heu/"+ xmlName + "_res_" + int2str(inputRatio) + ".xml";
    if ( isSPTBasedHeu )
      algName = "HEU_SPS";
    else
      algName = "HEU_CONN";
  }
  else if ( mode == 3 ){
    pathName= XmlRes + "xml_ilp/"+ xmlName + "_res_" + int2str(inputRatio) + ".xml";
    if ( isSPTBasedHeu )
      algName = "ILP_SPS";
    else
      algName = "ILP_CONN";
  }
  else if ( mode == 2){
    pathName= XmlRes + "xml_rtt/"+ xmlName + "_res.xml";
    algName = "RT_4L"; //Redundant Tree with 4 layers
  }
  else{
    cerr<<"Not able to write to xml, no mode is given!"<<endl;
  }

  cout<<"out File: "<<pathName<<endl;

  xml.open(pathName.c_str());
  if (!xml) {
    debug(CRITICAL)<<"Error opening file "<<pathName;
    exit (-1);                   // abnormal exit: error opening file
  }
  xml<<"<simulator>"<<endl;
  xml<<"\t<data>"<<endl;
  xml<<"\t\t<filename>"<<inputFile<<"</filename>"<<endl;
  xml<<"\t\t<N>"<<network.numOfPhysicalNodes<<"</N>"<<endl;
  xml<<"\t\t<E>"<<network.numOfPhysicalArcs<<"</E>"<<endl;
  xml<<"\t\t<LFA_cov_ini>"<<initialLfaCoverage<<"</LFA_cov_ini>"<<endl;
  // SRLG Mode [ 1: local SRLG, 2: global SRLG ]
  xml<<"\t\t<SRLGMode>"<<1<<"</SRLGMode> <!-- 1: local -->"<<endl;
  // Algorithm [1: heuristic, 2: ILP]
  xml<<"\t\t<LFAVirtAlg>"<<algName<<"</LFAVirtAlg> <!-- 1: heuristic 3: ILP -->"<<endl;
  xml<<"\t\t<virt_ratio>"<<ratio<<"</virt_ratio>"<<endl;
  xml<<"\t\t<woFAvgPathL>"<<network.woFaultAPL<<"</woFAvgPathL>"<<endl;
  xml<<"\t\t<wFAvgPathL>"<<network.wFaultAPL<<"</wFAvgPathL>"<<endl;

  if (mode == 2){
   xml<<"\t\t<LFA_cov_fin>"<<finalLfaCoverage<<"</LFA_cov_fin>"<<endl;
   xml<<"\t</data>"<<endl;
   xml<<"</simulator>"<<endl;
   return;
  }

  xml<<"\t</data>"<<endl;
  xml<<"\t<result>"<<endl;
  XmlStepSet::const_iterator xIt;
  int counter = 1;
  double virtPerc = 0;
  for (xIt = xSS.begin(); xIt != xSS.end(); ++xIt){
    virtPerc = 100 * (double)xIt->sumOfVirtualNsAtStep / (double)network.numOfPhysicalNodes;

    xml<<"\t\t<step>"<<endl;
    xml<<"\t\t <count>"<<counter<<"</count>"<<endl;
    xml<<"\t\t <node_num>"<<network.numOfPhysicalNodes+xIt->sumOfVirtualNsAtStep<<"</node_num>"<<endl;
    xml<<"\t\t <virt_perc>"<<virtPerc<<"</virt_perc>"<<endl;
    xml<<"\t\t <lfa_cov>"<<xIt->currLFACoverage<<"</lfa_cov>"<<endl;
    xml<<"\t\t <curr_avpl>"<<xIt->currAVPL<<"</curr_avpl>"<<endl;
    xml<<"\t\t <elapsed_time>"<<xIt->timeStamp.realTime()<<"</elapsed_time>"<<endl;
    xml<<"\t\t</step>"<<endl;
    counter++;
  }

  xml<<"\t</result>"<<endl;
  xml<<"</simulator>"<<endl;
}

void callPostFinishCheck(/*const*/ Network &network, const string &inputFile, const double &ratio){
  int numOfLoopingSDs;
  //test if have loops here + count average path lengths
  if ( network.checkLoopAndAVPL(numOfLoopingSDs) ){
    ofstream file;
    string fileName = "checkLoop.log";

    file.open(fileName.c_str(), ofstream::app);
    if (!file) {
      cerr<<"Error opening file "<<fileName;
      exit (-1);            // abnormal exit: error opening file
    }
    file<<"graph: "<<inputFile<<endl;
    file<<"ratio: "<<ratio<<endl;
    file<<"numOfLoopingSDs: "<<numOfLoopingSDs<<endl;
  }
}

int main(int argc, char **argv){

  debug(INFO)<<"Program has started."<<endl;
  ArgParser ap(argc, argv);

  int debugLevel = -1;
  int inputRatio = -1;
  int mode = -1; //1: greedyHeur, 2: redundant tree 3: greedyILP
  string inputFile;
  int strategy;
  NodeSelStrategy nSelStrategy;

  ap.refOption("v", "Verbosity level", debugLevel);
  ap.synonym("-verbose" , "v");

  ap.refOption("r", "Ratio of virtualized nodes", inputRatio);
  ap.synonym("-ratio" , "r");

  ap.refOption("m", "Improvement mode 1:greedy, 2:layers", mode);
  ap.synonym("-mode" , "m");

  ap.refOption("s", "Fix set(up to 4): 2 SPT based virt: 1, classic: 0", strategy);
  ap.synonym("-spt" , "s");

  ap.other("input file", "LGF input file")
    .boolOption("xml","Generate xml result files in xml directory.")
    .other("...");

  ap.parse();

  if(ap.files().size() == 0){
    cerr << "Error: LGF input file argument not specified" << endl;
    usage();
    exit(-1);
  }

  inputFile = ap.files()[0];

  if ( !isFileExist(inputFile) ){
    cerr << "Error: the given LGF file does not exist. Exit." << endl;
    exit(-1);
  }

  if ( !ap.given("s") && ( mode==1 || mode==3 ) ){
    cerr << "Error: sptMode is missing" << endl;
    usage();
    exit(-1);
  }
  else if ( mode != 2 ){
    switch ( strategy ){
    case 0:
      nSelStrategy = CONN_L;
      break;
    case 1:
      nSelStrategy = SPS;
      break;
    default:
      cout<<"No Node Selection Strategy is given!"<<endl;
    }
  }

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

  //Read network and setup simulator
  Network network(inputFile);

  // Calculate initial LFA coverage
  double initialLfaCoverage = network.newLfaCoverageMetric_LP(LP);

  /*
   * Transform ratio of virtual nodes like this:
   *   1 => 0.33
   *   2 => 0.66
   *   3 => 1
   *   4 => 2
   */
  double ratio = -1;

  switch (inputRatio){
  case 1:
    ratio = 0.333;
    break;
  case 2:
    ratio = 0.666;
    break;
  case 3:
    ratio = 1;
    break;
  case 4:
    ratio = 2;
    break;
  default:
    if ( mode == 2 ){
      debug(INFO)<<"INFO: Ratio is not used now."<<endl;
    }
    else {
      debug(CRITICAL)<<" ERROR: For this mode valid ratio is needed!"<<endl;
      usage();
      exit(-1);
    }
  }

  // Print header
  debug(INFO)<<"==================================="<<endl;
  debug(INFO)<<" Start simulation with logging level: "<<debugLevel<<endl;
  debug(INFO)<<" Ratio: "<<ratio<<endl;
  debug(INFO)<<" Graph: "<<inputFile<<endl;
  debug(INFO)<<" Mode : ";
  if ( mode == 1 )
    cout<<"GreedyHeur"<<endl;
  else if ( mode == 2 )
    cout<<"Redundant Layers"<<endl;
  else if ( mode == 3 )
    cout<<"GreedyILP"<<endl;
  else {
    cout<<"Unknown mode, failure!"<<endl;
  }
  debug(INFO)<<" N: "<<network.numOfPhysicalNodes
             <<" ; E: "<<countArcs(network.g)<<endl;
  debug(INFO)<<" Initial LP LFA coverage: "<<initialLfaCoverage<<endl;

  cout<<"Debug level is: "<<debugLevel<<endl;

  switch ( mode ){
  case 1:
    {
      //(skipEdge, cascLFA, upstreamLFA, smartTrapCond, sPTVirt)
      GreedyParamSet gPS = {false, false, true, true, nSelStrategy};
      network.greedyH->setGreedyParamSet(gPS);

      /*** Greedy heuristic starts here ***/
      debug(INFO)<<"======================================="<<endl;
      debug(INFO)<<"     Greedy heuristic starts"<<endl;
      debug(INFO)<<"     Used parameters: "<<endl;
      debug(INFO)<<"       skipEdgesOn    : "<<gPS.skipEdgesOn<<endl;
      debug(INFO)<<"       cascLFA        : "<<gPS.leak_cascCondOn<<endl;
      debug(INFO)<<"       upstreamLFA    : "<<gPS.skipUpstrLfa<<endl;
      debug(INFO)<<"       smartTrapCond  : "<<gPS.smartTrapCondOn<<endl;
      debug(INFO)<<"       nSelStrategy   : "<<gPS.nSelStrategy<<endl;
      debug(INFO)<<"======================================="<<endl;

      // Run greedy heuristic algorithm
      // network.greedyH->printUnprotectedSDSet();

      network.greedyH->greedyHeuristicAlgorithm(ratio, NODESET_SIZE);

      //check if have loops in the network + count Avg Path Length
      callPostFinishCheck(network, inputFile, ratio);

      // Recalculate final LFA coverage
      double finalLfaCoverage = network.newLfaCoverageMetric_LP(LP);

      debug(INFO)<<"==================================="<<endl;
      debug(INFO)<<" Final LP LFA coverage: "<<finalLfaCoverage<<endl;

      int numOfVirtualNodes = countNodes(network.g)-network.numOfPhysicalNodes;
      debug(INFO)<<" # virtual nodes needed: "<<numOfVirtualNodes<<endl;
      debug(INFO)<<" # PT calls (esc): "<<network.greedyH->getPTCounter_Escape()<<endl;
      debug(INFO)<<" # PT calls (trap): "<<network.greedyH->getPTCounter_Trap()<<endl;

      if (ap.given("xml")){
        writeToXml(numOfVirtualNodes, initialLfaCoverage, finalLfaCoverage, inputFile,
                   inputRatio, ratio, network, mode, network.greedyH->xmlStepSet, gPS.nSelStrategy);
      }

      //network.printGraph();

      break; //from case
    }
  case 2:
    {
      /*** Redundant Tree Based (4 layered) coverage improvement starts here ***/
      debug(INFO)<<endl;
      debug(INFO)<<"======================================="<<endl;
      debug(INFO)<<"  Redundant Tree Based method starts"<<endl;
      debug(INFO)<<"======================================="<<endl;
      // test RedundantTree Manager class:
      Network* netwP = &network;

      Node root = netwP->g.nodeFromId(0);

      RTBasedMethod rtMethod(netwP);
      rtMethod.buildRedundantTrees(root);
      rtMethod.checkTreeRedundancy(root);
      rtMethod.buildVirtualLayers(root);

      debug(INFO)<<"==================================="<<endl;

      // check LFA coverage and loop:
      double finalLfaCoverage = netwP->newLfaCoverageMetric_LP(LP);
      debug(INFO)<<" Final LP LFA coverage: "<<finalLfaCoverage<<endl;

      //Debug
      //rtMethod.printSRGSets();
      //netwP->greedyH->printLocalSRLGs();

      //rtMethod.printSpecificLayer(RR);
      //rtMethod.printSpecificLayer(RB);
      //rtMethod.printSpecificLayer(VRR);
      //rtMethod.printSpecificLayer(VRB);
      //rtMethod.printSpecificLayer(PHYS);
      //network.printGraph();

      callPostFinishCheck(network, inputFile, ratio);

      int numOfVirtualNodes = countNodes(network.g)-network.numOfPhysicalNodes;
      debug(INFO)<<" # virtual nodes needed: "<<numOfVirtualNodes<<endl;

      //debug
      network.greedyH->buildUnprotectedSDSet();
      //printOutUnprotectedSDSet();

      XmlStepSet dummyXSS;
      if (ap.given("xml")){
        writeToXml(numOfVirtualNodes, initialLfaCoverage, finalLfaCoverage, inputFile,
                   inputRatio, ratio, network, mode, dummyXSS, false);
      }
      network.writeGraph();

      break; //from case
    }
  case 3:
    {
      /*** Greedy ILP starts here ***/
      Network* netw = &network;
      GreedyILP greedyILP(netw);

      //(skipEdge, cascLFA, upstreamLFA, smartTrapCond, nodeSelectionStrategy)
      // smartTrapCond can not be used in case of ILP
      // due to multiple exit nodes
      GreedyParamSet gPS = {false, false, true, true, nSelStrategy};
      network.greedyH->setGreedyParamSet(gPS);

      debug(INFO)<<"======================================="<<endl;
      debug(INFO)<<"     Greedy ILP starts"<<endl;
      debug(INFO)<<"     Used parameters: "<<endl;
      debug(INFO)<<"       skipEdgesOn    : "<<gPS.skipEdgesOn<<endl;
      debug(INFO)<<"       cascLFA        : "<<gPS.leak_cascCondOn<<endl;
      debug(INFO)<<"       upstreamLFA    : "<<gPS.skipUpstrLfa<<endl;
      debug(INFO)<<"       smartTrapCond  : "<<gPS.smartTrapCondOn<<endl;
      debug(INFO)<<"       nSelStrategy   : "<<gPS.nSelStrategy<<endl;
      debug(INFO)<<"======================================="<<endl;

      network.greedyH->printUnprotectedSDSet();

      greedyILP.callILPGreedily(ratio, NODESET_SIZE);

      //check if have loops in the network + count Avg Path Length
      callPostFinishCheck(network, inputFile, ratio);

      debug(INFO)<<"==================================="<<endl;

      double finalCoverage = network.newLfaCoverageMetric_LP(LP);
      cout<<"Final LP LFA coverage: "<<finalCoverage<<endl;

      int numOfVirtualNodes = countNodes(network.g)-network.numOfPhysicalNodes;
      debug(INFO)<<" # virtual nodes needed: "<<numOfVirtualNodes<<endl;
      debug(INFO)<<" # PT calls (esc): "<<network.greedyH->getPTCounter_Escape()<<endl;
      debug(INFO)<<" # PT calls (trap): "<<network.greedyH->getPTCounter_Trap()<<endl;

      if (ap.given("xml")){
        writeToXml(numOfVirtualNodes, initialLfaCoverage, finalCoverage, inputFile,
                   inputRatio, ratio, network, mode, greedyILP.xmlStepSet, gPS.nSelStrategy);
      }

      //network.printGraph();
      network.writeGraph();

      break;
    }
  default:
    debug(CRITICAL)<<" ERROR: Wrong mode is given."<<endl;
    usage();
    exit(-1);

  }//end mode switch

  return 1;
}
