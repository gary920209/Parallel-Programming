#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <fstream>
#include <algorithm>
#include <climits>
#include <boost/functional/hash.hpp>
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
    string transition;
    int cost;
    int parent;

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

GridBitset wall_bitset; 
GridBitset target_bitset;
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
    if (r < 0 || r >= rows) return true;
    for (int c = 0; c < cols; c++) {
        if (!wall_bitset[r * cols + c]) return false;
    }
    return true;
}

bool is_wall_col(int c) {
    if (c < 0 || c >= cols) return true;
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

            if (is_wall(nr, nc) || reachable[next_idx] || initial_state.fragiles[next_idx]) {
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

bool is_blocked(int nr, int nc, const GridBitset& boxes, const GridBitset& fragiles, vector<bool>& checked, int orig_axis, const int dr[], const int dc[]) {
    if (is_wall(nr, nc)) return true;
    int idx = to_index(nr, nc, cols);
    if (is_fragile(idx, fragiles)) return true;
    if (!boxes[idx]) return false;
    if (checked[idx]) return true; // treat as wall
    checked[idx] = true;
    // check if this neighbor box is frozen on the current axis
    int r = get_row(idx, cols);
    int c = get_col(idx, cols);
    bool blocked1 = is_blocked(r + dr[0], c + dc[0], boxes, fragiles, checked, 1 - orig_axis, dr, dc);
    bool blocked2 = is_blocked(r + dr[1], c + dc[1], boxes, fragiles, checked, 1 - orig_axis, dr, dc);
    return blocked1 && blocked2;
}

bool is_axis_frozen(int box_idx, int axis, const GridBitset& boxes, const GridBitset& fragiles, vector<bool>& checked) {
    int r = get_row(box_idx, cols);
    int c = get_col(box_idx, cols);
    int dr[2], dc[2];
    if (axis == 0) { // horizontal
        dr[0] = 0; dc[0] = -1;
        dr[1] = 0; dc[1] = 1;
    } else { // vertical
        dr[0] = -1; dc[0] = 0;
        dr[1] = 1; dc[1] = 0;
    }
    bool blocked1 = is_blocked(r + dr[0], c + dc[0], boxes, fragiles, checked, axis, dr, dc);
    bool blocked2 = is_blocked(r + dr[1], c + dc[1], boxes, fragiles, checked, axis, dr, dc);
    return blocked1 && blocked2;
}

bool has_freeze_deadlock(const GridBitset& boxes, const GridBitset& fragiles) {
    for (int idx = 0; idx < rows * cols; ++idx) {
        if (!boxes[idx] || target_bitset[idx]) continue;
        vector<bool> checked(rows * cols, false);
        bool frozen_h = is_axis_frozen(idx, 0, boxes, fragiles, checked);
        vector<bool> checked_v(rows * cols, false);
        bool frozen_v = is_axis_frozen(idx, 1, boxes, fragiles, checked_v);
        if (frozen_h && frozen_v) return true;
    }
    return false;
}

bool is_invalid_state(const GridBitset& boxes, const GridBitset& fragiles) {
    if (row_col_deadlock(boxes)) return true;
    if (has_freeze_deadlock(boxes, fragiles)) return true;
    return false;
}

// compute player reachable positions and predecessors
void compute_player_reach(int start_idx, const GridBitset& boxes, const GridBitset& fragiles, GridBitset& reachable, vector<int>& prev) {
    reachable.reset();
    prev.assign(rows * cols, -1);
    queue<int> q;

    q.push(start_idx);
    reachable[start_idx] = 1;

    int dr[] = {-1, 0, 1, 0};
    int dc[] = {0, -1, 0, 1};

    while (!q.empty()) {
        int cur = q.front();
        q.pop();

        int r = get_row(cur, cols);
        int c = get_col(cur, cols);

        for (int i = 0; i < 4; ++i) {
            int nr = r + dr[i];
            int nc = c + dc[i];
            int next = to_index(nr, nc, cols);

            if (is_wall(nr, nc) || boxes[next] || reachable[next]) continue;

            reachable[next] = 1;
            prev[next] = cur * 4 + i;  // Encode prev_idx * 4 + dir
            q.push(next);
        }
    }
}

// reconstruct path using predecessors
string get_path_to(int target_idx, int start_idx, const vector<int>& prev) {
    if (target_idx == start_idx) return "";

    string path;
    int cur = target_idx;
    char move_chars[] = {'W', 'A', 'S', 'D'};

    while (cur != start_idx) {
        if (prev[cur] == -1) return "";  // Not reachable
        int dir = prev[cur] % 4;
        int p = prev[cur] / 4;
        path.push_back(move_chars[dir]);
        cur = p;
    }

    reverse(path.begin(), path.end());
    return path;
}

string reconstruct_path(const vector<State>& states, int goal_index) {
    vector<string> parts;
    int idx = goal_index;
    while (idx != 0) {  // 0 is start
        const State& s = states[idx];
        parts.push_back(s.transition);
        idx = s.parent;
    }
    reverse(parts.begin(), parts.end());
    string full_move;
    for (const auto& p : parts) {
        full_move += p;
    }
    return full_move;
}

string AStar(const State& initial_state){
    // Use vector to store states and priority queue for indices
    vector<State> states;
    priority_queue<PQItem, vector<PQItem>, greater<PQItem>> pq;
    unordered_map<GridBitset, GridBitset> visited;

    State start_state = initial_state;
    start_state.cost = 0;
    start_state.parent = -1;
    start_state.transition = "";
    states.push_back(start_state);
    pq.push({0, StateComparator::heuristic(start_state)});  // Greedy best-first search

    while(!pq.empty()){
        PQItem current_item = pq.top();
        pq.pop();

        State& current_state = states[current_item.state_index];

        // Compute player reachable and prev once per state
        GridBitset reachable;
        vector<int> prev(rows * cols, -1);
        compute_player_reach(current_state.player_idx, current_state.boxes, current_state.fragiles, reachable, prev);

        if (visited[current_state.boxes][current_state.player_idx]) {
            continue;
        }
        visited[current_state.boxes] |= reachable;
        
        // Check if all boxes are on targets using bitwise AND
        if ((current_state.boxes & target_bitset) == current_state.boxes) {
            return reconstruct_path(states, current_item.state_index);  // Solution found and returned here
        }
        
        int dr[] = {-1, 0, 1, 0};
        int dc[] = {0, -1, 0, 1};
        char move_chars[] = {'W', 'A', 'S', 'D'};
        
        // Use vector to store generated next states
        vector<State> next_states;
        
        for (int push_idx = 0; push_idx < rows * cols; ++push_idx) {
            if (!reachable[push_idx]) continue;

            int r = get_row(push_idx, cols);
            int c = get_col(push_idx, cols);

            for (int j = 0; j < 4; ++j) {
                int box_idx = to_index(r + dr[j], c + dc[j], cols);
                if (!current_state.boxes[box_idx]) continue;

                int next_box_idx = to_index(r + 2*dr[j], c + 2*dc[j], cols);

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

                if (is_invalid_state(temp_boxes, current_state.fragiles)) {
                    continue;
                }

                string path_to_push = get_path_to(push_idx, current_state.player_idx, prev);
                if (path_to_push.empty() && push_idx != current_state.player_idx) continue;

                State next_state = current_state;
                next_state.player_idx = box_idx;
                next_state.boxes = temp_boxes;
                next_state.transition = path_to_push + move_chars[j];
                next_state.cost = current_state.cost + path_to_push.length() + 1;
                next_state.parent = current_item.state_index;
                
                next_states.push_back(next_state);
            }
        }
        
        // After loop, add states to vector and push indices to priority_queue if not visited
        for (auto& state : next_states) {
            auto it = visited.find(state.boxes);
            bool is_visited = (it != visited.end() && it->second[state.player_idx]);
            if (!is_visited) {
                states.push_back(state);
                int state_index = states.size() - 1;
                int priority = StateComparator::heuristic(state);  // Greedy best-first search
                pq.push({state_index, priority});
            }
        }
    }

    return ""; // No solution found
}

int StateComparator::heuristic(const State& state) {
    vector<int> box_list;
    vector<int> target_list;
    for (int idx = 0; idx < rows * cols; ++idx) {
        if (state.boxes[idx]) box_list.push_back(idx);
        if (target_bitset[idx]) target_list.push_back(idx);
    }

    int k = box_list.size();
    int m = target_list.size();
    if (k > m) return INT_MAX;  

    // Compute exact min matching if m small, else greedy
    if (m <= 20) {  
        vector<vector<int>> cost(k, vector<int>(m, 0));
        for (int i = 0; i < k; ++i) {
            int br = get_row(box_list[i], cols);
            int bc = get_col(box_list[i], cols);
            for (int j = 0; j < m; ++j) {
                int tr = get_row(target_list[j], cols);
                int tc = get_col(target_list[j], cols);
                cost[i][j] = abs(br - tr) + abs(bc - tc);
            }
        }

        vector<int> dp(1 << m, INT_MAX / 2);
        dp[0] = 0;
        for (int mask = 0; mask < (1 << m); ++mask) {
            int x = __builtin_popcount(mask);
            if (x >= k) continue;
            for (int j = 0; j < m; ++j) {
                if (mask & (1 << j)) continue;
                int nmask = mask | (1 << j);
                dp[nmask] = min(dp[nmask], dp[mask] + cost[x][j]);
            }
        }
        int min_h = INT_MAX / 2;
        for (int mask = 0; mask < (1 << m); ++mask) {
            if (__builtin_popcount(mask) == k) {
                min_h = min(min_h, dp[mask]);
            }
        }
        return min_h;
    } else {
        // Fallback to original greedy
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