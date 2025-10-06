#ifndef SOKOBAN_H
#define SOKOBAN_H

#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_unordered_set.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <unordered_map>
#include <fstream>
#include <algorithm>
#include <climits>
#include <chrono>
#include <boost/functional/hash.hpp>
#include <bitset>

using namespace std;

const int MAX_SIZE = 256; // spec say maxium grid size = 16*16
using GridBitset = bitset<MAX_SIZE + 9>; // +9 for padding to avoid boundary checks

// Inline utility functions
inline int get_row(int index, int cols) {
    return index / cols;
}

inline int get_col(int index, int cols) {
    return index % cols;
}

inline int to_index(int r, int c, int cols) {
    return r * cols + c;
}

// State structure
struct State {
    int player_idx;  
    GridBitset boxes;
    string transition;
    int cost;
    int parent;
};

// Hash function for GridBitset
namespace std {
    template <>
    struct hash<GridBitset> {
        size_t operator()(const GridBitset& bs) const {
            size_t seed = 0;
            for (size_t w = 0; w < (MAX_SIZE + 9 + 63) / 64; ++w) {
                boost::hash_combine(seed, bs._M_w[w]);
            }
            return seed;
        }
    };
}

// Priority queue item storing index and priority
struct PQItem {
    int state_index;
    int priority;  // f = g + w * h, by A Star

    bool operator>(const PQItem& other) const {
        return priority > other.priority;
    }
};

// StateComparator for heuristic calculation
struct StateComparator {
    static int heuristic(const GridBitset& boxes);
};

// Global variables
extern GridBitset wall_bitset;
extern GridBitset target_bitset;
extern GridBitset fragile_bitset;
extern int rows, cols;
extern vector<int> target_list;
extern vector<vector<int>> dist_to_target;

// Wall and deadlock detection functions
bool is_wall(int r, int c);
bool is_wall_row(int r);
bool is_wall_col(int c);
bool row_col_deadlock(const GridBitset& boxes);
bool is_deadlock(int box_idx);
bool is_fragile(int idx);
bool has_freeze_deadlock(const GridBitset& boxes);
bool is_invalid_state(const GridBitset& boxes);

// Initialization and preprocessing functions
void mark_unreachable_as_fragile();
void precompute_distances();

// Path finding and player movement functions
void compute_player_reach(int start_idx, const GridBitset& boxes, GridBitset& reachable, vector<int>& prev);
string get_path_to(int target_idx, int start_idx, const vector<int>& prev);
string reconstruct_path(const vector<State>& states, int goal_index);

// A* search algorithm
string AStar(const State& initial_state);

#endif // SOKOBAN_H