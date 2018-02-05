# before executing it do 
# g++ -I$HOME/lemon-1.3.1/ -c glpk.cc -o glpk.o
include_directories($ENV{HOME}/lemon-1.3.1)

if (GUROBI)
  add_definitions(-DGUROBI)
  include_directories($ENV{HOME}/gurobi702/linux64/include)
endif()

if (DEBUG_LOG)
  add_definitions(-DDEBUG_LOG)
endif()

add_executable(routerVirtSim 
  main.cc
  Network.cc
  Database.cc
  SRG.cc
  PacketTracer.cc
  GreedyHeuristic.cc
  RTBasedMethod.cc
  GreedyILP.cc
)

target_link_libraries(routerVirtSim
-L$ENV{HOME}/lemon-1.3.1/build/lemon
-lemon
)

if (GUROBI)
  target_link_libraries(routerVirtSim
  $ENV{HOME}/lemon-1.3.1/lemon/gurobi.o
  -L$ENV{HOME}/gurobi702/linux64/lib
  -lgurobi_c++
  -lgurobi70
 )
else()
 target_link_libraries(routerVirtSim
 $ENV{HOME}/lemon-1.3.1/lemon/glpk.o
 -lglpk
 )
endif()
