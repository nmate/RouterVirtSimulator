/*
 * Written by Mate Nagy
 * License: GNU General Public License Version 3
 *
 */

#ifndef SRG_HH_
#define SRG_HH_

#include "Network.hh"

//typedef set<Arc> SRGUnit;
//typedef set<SRGUnit> SRGSet;

class Database;
class Network;

class SRG {
public:
  Network *nwP;
  SRGSet localSRG;
  SRGSet globalSRG;

  SRG(Network*);
  void parseSRGsFromFile(const string&);
  void buildLocalSRGSet();
  //~SRG();
};


// read srgs
struct SrgSection {
  vector<string> &_data;
  SrgSection(vector<string>& data) : _data(data) {};
  void operator()(const string& line){
    _data.push_back(line);
  }
};

#endif /* SRG_HH_ */
