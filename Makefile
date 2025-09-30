CXX = g++
CXXFLAGS = -std=c++17 -O3 -pthread -fopenmp -ltbb
TARGETS = hw1

.PHONY: all
all: $(TARGETS)

hw1: hw1.cpp
	$(CXX) $(CXXFLAGS) -o hw1 hw1.cpp
B1
.PHONY: clean
clean:
	rm -f $(TARGETS)
