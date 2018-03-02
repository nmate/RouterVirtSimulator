RouterVirtSimulator - Router Virtualization for Improving LFA Protection
=========

Intro
------

For IP to evolve into a true carrier-grade transport facility, it
needs to support fast resilience out-of-the-box. IP-level failure
protection based on the IP Fast ReRoute/Loop-Free Alternates (LFA)
specification has become industrial requirement recently. The success
of LFA lies in its inherent simplicity, but this comes at the expense
of letting certain failure scenarios go unprotected. Realizing full
failure coverage with LFA so far has only been possible through
completely re-engineering the network around LFA-compliant design
patterns.

This tool is intended to propose a virtual overlay on top of the
given physical network so that the resultant LFA coverage becomes
much better than initially or even becomes perfect. For this we 
present three different strategies:

- greedy heuristic
- greedy ILP (Integer Linear Program)
- redundant trees

The first two schemes select the best set of nodes to be virtualized
in each step, while the third provides perfect LFA coverage in
one step. If the goal is to simply increase the level of protection
with a fix number of virtual nodes, then the greedy approach should
be used, while on the other hand, if there is no restriction on the 
number of virtual nodes, then one would choose the virtual layers 
formed by redundant trees.

Dependencies
-------------
- LEMON v1.3.1 [http://lemon.cs.elte.hu/trac/lemon]
- GLPK or GUROBI
- BOOST (only for testing) /easiest with libboost-all-dev/

Please install lemon and place glpk.cc.o under `${LEMON}/lemon`.

Compile
--------
Download and compile with GLPK:
```sh
cmake -DGUROBI=OFF . && make
```
or with GUROBI:
```sh
cmake -DGUROBI=ON . && make
```
The executable is generated under `build/src/`.

Test
-----
Compile with:
```sh
cmake -DWITH_TEST=ON . && make
```
and run with:
```sh
cd build/test
./routerVirtSimTest --log_level=test_suite
```

Licensing 
--------- 
RouterVirtSimulator is a free software provided under the GNU General
Public License (GPLv3). Copyright (C) 2018 Mate Nagy All Right
Reserved. See the GNU General Public License for more details.

Author
--------
Mate Nagy
