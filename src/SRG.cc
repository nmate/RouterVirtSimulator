/*
 * Written by Mate Nagy
 * License: GNU General Public License Version 3
 *
 */

#include "SRG.hh"

SRG::SRG(Network *_nwP){
  nwP = _nwP;
}

void SRG::parseSRGsFromFile(const string &graph_name) {
  vector<string> srgLines;
  SectionReader srgReader(graph_name);

  srgReader.sectionLines("srgs", SrgSection(srgLines));
  try {
    srgReader.run();
  } catch (FormatError &error) {
    cout << "Syntax error on line " << error.line() << ": " << error.message() << endl;
    // no SRGs from input
    localSRG.clear();
    globalSRG.clear();
  }
}

void SRG::buildLocalSRGSet(){

}
