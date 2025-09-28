#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_unordered_set.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <unordered_set>
#include <fstream>
#include <algorithm>
#include <climits>
#include <omp.h> 
#include <boost/functional/hash.hpp>
#include <mutex>
#include <atomic>
#include <bitset>

using namespace std;

const int MAX_SIZE = 256; // 16*16 max grid size
using GridBitset = bitset<MAX_SIZE+9>;

// Helper functions for index operations
inline int get_row(int index, int cols) {
    return index / cols;
}

inline int get_col(int index, int cols) {
    return index % cols;
}

inline int to_index(int r, int c, int cols) {
    return r * cols + c;
}

struct State{
    int player_idx;  // Player position as index
    GridBitset boxes;
    GridBitset fragiles;
    string move;
    int cost;

    bool operator==(const State& other) const {
        return player_idx == other.player_idx && boxes == other.boxes && fragiles == other.fragiles;
    }
};

namespace std {
    template <>
    struct hash<State> {
        size_t operator()(const State& s) const {
            size_t seed = 0;
            boost::hash_combine(seed, s.player_idx);

            // Hash bitsets efficiently
            auto boxes_hash = std::hash<GridBitset>{}(s.boxes);
            auto fragiles_hash = std::hash<GridBitset>{}(s.fragiles);
            boost::hash_combine(seed, boxes_hash);
            boost::hash_combine(seed, fragiles_hash);

            return seed;
        }
    };
}

GridBitset wall_bitset;  // For walls
GridBitset target_bitset; // For targets
int rows, cols;

// Priority queue item storing index and priority
struct PQItem {
    int state_index;
    int priority;  // f = g + h

    bool operator>(const PQItem& other) const {
        return priority > other.priority;
    }
};

struct StateComparator {
    static int heuristic(const State& state);
};


bool is_wall(int r, int c){
    if (r < 0 || r >= rows || c < 0 || c >= cols) return true;
    return wall_bitset[r * cols + c];
}
bool is_wall_row(int r) {
    for (int c = 0; c < cols; c++) {
        if (!wall_bitset[r * cols + c]) return false;
    }
    return true;
}

bool is_wall_col(int c) {
    for (int r = 0; r < rows; r++) {
        if (!wall_bitset[r * cols + c]) return false;
    }
    return true;
}

bool row_col_deadlock(const GridBitset& boxes) {
    for (int idx = 0; idx < rows * cols; ++idx) {
        if (!boxes[idx]) continue;

        int r = get_row(idx, cols);
        int c = get_col(idx, cols);
        bool has_target = false;

        if (is_wall_row(r+1)) {
            for (int cc = 0; cc < cols; cc++) {
                if (target_bitset[r * cols + cc]) {
                    has_target = true;
                    break;
                }
            }
            if (target_bitset[idx]) has_target = true;
            if (!has_target) return true;
        }

        if (is_wall_row(r-1)) {
            for (int cc = 0; cc < cols; cc++) {
                if (target_bitset[r * cols + cc]) {
                    has_target = true;
                    break;
                }
            }
            if (target_bitset[idx]) has_target = true;
            if (!has_target) return true;
        }

        if (is_wall_col(c+1)) {
            for (int rr = 0; rr < rows; rr++) {
                if (target_bitset[rr * cols + c]) {
                    has_target = true;
                    break;
                }
            }
            if (target_bitset[idx]) has_target = true;
            if (!has_target) return true;
        }

        if (is_wall_col(c-1)) {
            for (int rr = 0; rr < rows; rr++) {
                if (target_bitset[rr * cols + c]) {
                    has_target = true;
                    break;
                }
            }
            if (target_bitset[idx]) has_target = true;
            if (!has_target) return true;
        }
    }
    return false;
}

void mark_unreachable_as_fragile(State& initial_state) {
    GridBitset reachable;
    queue<int> q;

    // Start BFS from all targets
    for (int idx = 0; idx < rows * cols; ++idx) {
        if (target_bitset[idx] && !wall_bitset[idx]) {
            q.push(idx);
            reachable[idx] = 1;
        }
    }

    int dr[] = {-1, 0, 1, 0};
    int dc[] = {0, -1, 0, 1};

    while (!q.empty()) {
        int current_idx = q.front();
        q.pop();

        int r = get_row(current_idx, cols);
        int c = get_col(current_idx, cols);

        for (int i = 0; i < 4; ++i) {
            int nr = r + dr[i];
            int nc = c + dc[i];
            int next_idx = to_index(nr, nc, cols);

            if (is_wall(nr, nc) || reachable[next_idx]) {
                continue;
            }

            reachable[next_idx] = 1;
            q.push(next_idx);
        }
    }

    for (int idx = 0; idx < rows * cols; ++idx) {
        if (!wall_bitset[idx] && !reachable[idx]) {
            initial_state.fragiles[idx] = 1;
        }
    }
}

bool is_deadlock(int box_idx){
    if (target_bitset[box_idx]) return false;

    int r = get_row(box_idx, cols);
    int c = get_col(box_idx, cols);
    if((is_wall(r+1,c) && is_wall(r,c+1)) || (is_wall(r-1,c) && is_wall(r,c-1)) ||
       (is_wall(r-1,c) && is_wall(r,c+1)) || (is_wall(r+1,c) && is_wall(r,c-1)))
        return true;

    return false;
}

bool is_fragile(int idx, const GridBitset& fragiles) {
    return fragiles[idx];
}

string find_path(int start_idx, int end_idx, const GridBitset& boxes, const GridBitset& fragiles) {
    if (start_idx == end_idx) return "";

    queue<pair<int, string>> q;
    GridBitset visited;

    q.push({start_idx, ""});
    visited[start_idx] = 1;

    int dr[] = {-1, 0, 1, 0};
    int dc[] = {0, -1, 0, 1};
    char move_chars[] = {'W', 'A', 'S', 'D'};

    while (!q.empty()) {
        auto [current_idx, path] = q.front();
        q.pop();

        int r = get_row(current_idx, cols);
        int c = get_col(current_idx, cols);

        for (int i = 0; i < 4; ++i) {
            int nr = r + dr[i];
            int nc = c + dc[i];
            int next_idx = to_index(nr, nc, cols);

            if (next_idx == end_idx) {
                return path + move_chars[i];
            }

            if (is_wall(nr, nc) || visited[next_idx] ||
                boxes[next_idx] || fragiles[next_idx]) {
                continue;
            }

            visited[next_idx] = 1;
            q.push({next_idx, path + move_chars[i]});
        }
    }

    return "";
}
GridBitset get_reachable_positions(int player_idx, const GridBitset& boxes, const GridBitset& fragiles) {
    GridBitset reachable;
    queue<int> q;
    GridBitset visited;

    q.push(player_idx);
    visited[player_idx] = 1;
    reachable[player_idx] = 1;

    while (!q.empty()) {
        int current_idx = q.front();
        q.pop();

        int r = get_row(current_idx, cols);
        int c = get_col(current_idx, cols);

        int dr[] = {-1, 0, 1, 0};
        int dc[] = {0, -1, 0, 1};

        for (int i = 0; i < 4; ++i) {
            int nr = r + dr[i];
            int nc = c + dc[i];
            int next_idx = to_index(nr, nc, cols);

            if (is_wall(nr, nc) || visited[next_idx]) {
                continue;
            }

            if (boxes[next_idx]) {
                reachable[current_idx] = 1;
                continue;
            }

            visited[next_idx] = 1;
            reachable[next_idx] = 1;
            q.push(next_idx);
        }
    }

    return reachable;
}

string AStar(const State& initial_state){
    // Use vector to store states and priority queue for indices
    vector<State> states;
    priority_queue<PQItem, vector<PQItem>, greater<PQItem>> pq;
    tbb::concurrent_unordered_set<State> closed_set;

    State start_state = initial_state;
    start_state.cost = 0;
    states.push_back(start_state);
    pq.push({0, start_state.cost + StateComparator::heuristic(start_state)});

    while(!pq.empty()){
        PQItem current_item = pq.top();
        pq.pop();

        State& current_state = states[current_item.state_index];

        if (closed_set.count(current_state)) {
            continue;
        }
        closed_set.insert(current_state);
        
        // Check if all boxes are on targets using bitwise AND
        if ((current_state.boxes & target_bitset) == current_state.boxes) {
            return current_state.move;  // Solution found and returned here
        }

        GridBitset push_positions = get_reachable_positions(current_state.player_idx, current_state.boxes, current_state.fragiles);
        
        int dr[] = {-1, 0, 1, 0};
        int dc[] = {0, -1, 0, 1};
        char move_chars[] = {'W', 'A', 'S', 'D'};
        
        // Create work items (position + direction pairs) for better load balancing
        struct WorkItem {
            int push_idx;
            int direction;
        };

        std::vector<WorkItem> work_items;
        for (int push_idx = 0; push_idx < rows * cols; ++push_idx) {
            if (!push_positions[push_idx]) continue;

            int r = get_row(push_idx, cols);
            int c = get_col(push_idx, cols);

            for (int j = 0; j < 4; ++j) {
                int box_idx = to_index(r + dr[j], c + dc[j], cols);
                // Only add work item if there's actually a box at this position
                if (current_state.boxes[box_idx]) {
                    work_items.push_back({push_idx, j});
                }
            }
        }
        
        // Use concurrent_vector to store generated next states
        tbb::concurrent_vector<State> next_states;
        
        // Use smaller grain size for better work distribution across threads
        // Calculate grain size to aim for roughly 6x more chunks than threads
        size_t grain_size = max(size_t(1), work_items.size() / 24); // 24 = 6 threads * 4 chunks per thread
        
        tbb::parallel_for(tbb::blocked_range<size_t>(0, work_items.size(), grain_size),
            [&](const tbb::blocked_range<size_t>& range) {
                for (size_t i = range.begin(); i != range.end(); ++i) {
                    const WorkItem& work = work_items[i];
                    int push_idx = work.push_idx;
                    int j = work.direction;

                    int push_r = get_row(push_idx, cols);
                    int push_c = get_col(push_idx, cols);
                    int box_idx = to_index(push_r + dr[j], push_c + dc[j], cols);
                    int next_box_idx = to_index(push_r + 2*dr[j], push_c + 2*dc[j], cols);

                    if (!current_state.boxes[box_idx]) continue;

                    int next_r = get_row(next_box_idx, cols);
                    int next_c = get_col(next_box_idx, cols);
                    if (is_wall(next_r, next_c) ||
                        current_state.boxes[next_box_idx] ||
                        current_state.fragiles[next_box_idx] ||
                        is_deadlock(next_box_idx)) {
                        continue;
                    }

                    // Create temporary bitset to check for deadlocks
                    GridBitset temp_boxes = current_state.boxes;
                    temp_boxes[box_idx] = 0;
                    temp_boxes[next_box_idx] = 1;

                    if (row_col_deadlock(temp_boxes)) {
                        continue;
                    }

                    string path_to_push = find_path(current_state.player_idx, push_idx, current_state.boxes, current_state.fragiles);

                    State next_state = current_state;
                    next_state.player_idx = box_idx;
                    next_state.boxes[box_idx] = 0;
                    next_state.boxes[next_box_idx] = 1;
                    next_state.move += path_to_push + move_chars[j];
                    next_state.cost = current_state.cost + path_to_push.length() + 1;
                    
                    if (closed_set.find(next_state) == closed_set.end()) {
                        next_states.push_back(next_state);
                    }
                }
            });
        
        // After parallel region, add states to vector and push indices to priority_queue
        for (const auto& state : next_states) {
            states.push_back(state);
            int state_index = states.size() - 1;
            int priority = state.cost + StateComparator::heuristic(state);
            pq.push({state_index, priority});
        }
    }

    return ""; // No solution found
}

int StateComparator::heuristic(const State& state) {
    int total_distance = 0;
    GridBitset target_used = target_bitset; // Copy target bitset

    // Iterate through set bits in boxes bitset
    for (int box_idx = 0; box_idx < rows * cols; ++box_idx) {
        if (!state.boxes[box_idx]) continue;

        int box_r = get_row(box_idx, cols);
        int box_c = get_col(box_idx, cols);
        int min_dist = INT_MAX;
        int best_target_idx = -1;

        // Find closest unused target
        for (int target_idx = 0; target_idx < rows * cols; ++target_idx) {
            if (!target_used[target_idx]) continue;

            int target_r = get_row(target_idx, cols);
            int target_c = get_col(target_idx, cols);
            int dist = abs(box_r - target_r) + abs(box_c - target_c);
            if (dist < min_dist) {
                min_dist = dist;
                best_target_idx = target_idx;
            }
        }

        if (best_target_idx != -1) {
            target_used[best_target_idx] = 0; // Mark target as used
            total_distance += min_dist;
        }
    }

    return total_distance;
}

int main(int argc, char* argv[]) {
    
    // 1) check args and file open
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <input_file>\n";
        return 1;
    }
    ifstream infile(argv[1]);
    if (!infile.is_open()) {
        cerr << "Error: cannot open file " << argv[1] << "\n";
        return 1;
    }

    string line;
    State initial_state;
    initial_state.move = "";
    initial_state.boxes.reset();
    initial_state.fragiles.reset();

    int r = 0;
    size_t max_len = 0;
    vector<string> raw_lines;
    while (getline(infile, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        raw_lines.push_back(line);
        if (line.size() > max_len) max_len = line.size();
        r++;
    }
    infile.close();

    if (raw_lines.empty()) {
        cerr << "Error: empty board\n";
        return 1;
    }

    // pad lines to same length (fill with walls)
    cols = (int)max_len;
    rows = (int)raw_lines.size();

    // Initialize bitsets
    wall_bitset.reset();
    target_bitset.reset();

    // Parse board directly into bitsets
    for (int i = 0; i < rows; ++i) {
        const string& line = raw_lines[i];
        for (int c = 0; c < cols; ++c) {
            char ch = (c < line.size()) ? line[c] : '#';
            int idx = i * cols + c;

            if (ch == '#') {
                wall_bitset[idx] = 1;
            }
            if (ch == 'o' || ch == 'O' || ch == '!') {
                initial_state.player_idx = idx;
            }
            if (ch == 'x' || ch == 'X') {
                initial_state.boxes[idx] = 1;
            }
            if (ch == '.' || ch == 'X' || ch == 'O') {
                target_bitset[idx] = 1;
            }
            if (ch == '@' || ch == '!') {
                initial_state.fragiles[idx] = 1;
            }
        }
    }

    // Mark unreachable positions as fragile (deadlock detection)
    mark_unreachable_as_fragile(initial_state);

    string solution = AStar(initial_state);
    cout << solution << endl;
    return 0;
}