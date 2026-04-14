CXX=g++
CXXFLAGS=-O3 -fopenmp -Wfatal-errors -std=gnu++17 -fPIC `pkg-config --cflags --libs graph-tool-py3.9` -shared
ALL: lib_dequasearch.so

lib_dequasearch.so: dequa_graph_dijkstra.cc
	${CXX} ${CXXFLAGS} dequa_graph_dijkstra.cc -o lib_dequasearch.so 
