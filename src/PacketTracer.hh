/*
 * Written by Mate Nagy
 * License: GNU General Public License Version 3
 *
 */

#ifndef PACKETTRACER_HH_
#define PACKETTRACER_HH_

#include "Network.hh"
#include "Log.h"
#include <vector>

class Database;
class Network;

class PacketTracer {
public:
        const Network *nwP;
        bool haveLoop;
        bool nHopIsPhysical;
        NodeVector path;
        vector<NodeVector> ecmpPath; //store max 5 ECMP paths
        int numOfPossiblePaths;
        Node source;
        Node destin;

private:
        //these are needed for filling
        //the trap-node condition caused leakage
        bool skipUpstrLFA;
        bool useCascLFA;
        int lfaCnt;
public:
        PacketTracer(Network*);
        void setUseOfUpstrLFA(const bool&);
        void setUseOfCascLFA(const bool&);
        void getAllNextHopsToD(const Node&, const Node&, const LinkSet&, NodeVector&);
        void getAllLFAsToD(const Node&, const Node&, const LinkSet&, NodeVector&);
        Node selectForwardingNode(const Node&, const Node&, const LinkSet &, bool&);
        bool jumpToNextNode(int &, Node, const Node&, const LinkSet &);
        void jumpOnAllShortestPaths(int&, const Node&, const Node&, const LinkSet&, NodeVector&, bool&);
        void printAllPossiblePaths();
        void printPrimaryPath();
        bool haveAllPathsValid(const Node&, const Node&, const LinkSet&);
        bool haveValidPath(const Node&, const Node&, const LinkSet&);
        const NodeVector& getPath();
};

#endif /* PACKETTRACER_HH_ */
