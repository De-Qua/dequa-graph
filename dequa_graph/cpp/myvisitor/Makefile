CXX=g++
CXXFLAGS=-O3 -fopenmp -std=gnu++17 -Wall -fPIC `pkg-config --cflags --libs graph-tool-py3.9` -shared
ALL: libmyvisitor.so

libmyvisitor.so: my_visitor.cc
	${CXX} ${CXXFLAGS} my_visitor.cc -o libmyvisitor.so 
