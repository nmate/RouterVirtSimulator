include_directories($ENV{HOME}/lemon-1.3.1)

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

else()
 target_link_libraries(routerVirtSimTest
 $ENV{HOME}/lemon-1.3.1/lemon/glpk.o
 -lglpk
 )
endif()
