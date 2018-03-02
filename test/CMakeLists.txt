include_directories($ENV{HOME}/lemon-1.3.1)

if (GUROBI)
  add_definitions(-DGUROBI)
  include_directories($ENV{HOME}/gurobi702/linux64/include)
endif()


add_executable(routerVirtSimTest
 UnitTest.cc
 ../src/Network.cc
 ../src/Database.cc
 ../src/SRG.cc
 ../src/PacketTracer.cc
 ../src/GreedyHeuristic.cc
 ../src/RTBasedMethod.cc
 ../src/GreedyILP.cc
)

target_link_libraries(routerVirtSimTest
 -L$ENV{HOME}/lemon-1.3.1/build/lemon
 -lemon
)

if (GUROBI)
  target_link_libraries(routerVirtSimTest
  $ENV{HOME}/lemon-1.3.1/lemon/gurobi.o
  -L$ENV{HOME}/gurobi702/linux64/lib
  -lgurobi_c++
  -lgurobi70
  )
else()
 target_link_libraries(routerVirtSimTest
 $ENV{HOME}/lemon-1.3.1/lemon/glpk.o
 -lglpk
 )
endif()
