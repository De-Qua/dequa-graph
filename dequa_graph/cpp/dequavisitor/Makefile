CXX=g++
CXXFLAGS=-O3 -fopenmp -std=gnu++17 -Wall -fPIC `pkg-config --cflags --libs graph-tool-py3.9` -shared
ALL: lib_dequadistance.so

lib_dequadistance.so: dequa_get_dists.cc
	${CXX} ${CXXFLAGS} dequa_get_dists.cc -o lib_dequadistance.so 
