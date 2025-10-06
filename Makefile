CXX = g++
CXXFLAGS = -std=c++17 -O3 -pthread -fopenmp -ltbb
TARGETS = hw1
SOURCES = main.cpp deadlock.cpp astar.cpp
OBJECTS = $(SOURCES:.cpp=.o)

.PHONY: all
all: $(TARGETS)

hw1: $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o hw1 $(OBJECTS)

%.o: %.cpp sokoban.h
	$(CXX) $(CXXFLAGS) -c $< -o $@
.PHONY: clean
clean:
	rm -f $(TARGETS) $(OBJECTS)
