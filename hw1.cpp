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

inline int get_row(int index, int cols) {
    return index / cols;
}
inline int get_col(int index, int cols) {
    return index % cols;
}
inline int to_index(int r, int c, int cols) {
    return r * cols + c;
}
struct State {
    int player_idx;  
    GridBitset boxes;
    string transition;
    int cost;
    int parent;
};

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

GridBitset wall_bitset;
GridBitset target_bitset;
GridBitset fragile_bitset;
int rows, cols;
vector<int> target_list;
vector<vector<int>> dist_to_target;  // dist_to_target[pos][target_index in target_list]

// Priority queue item storing index and priority
struct PQItem {
    int state_index;
    int priority;  // f = g + w * h, by A Star

    bool operator>(const PQItem& other) const {
        return priority > other.priority;
    }
};

struct StateComparator {
    static int heuristic(const GridBitset& boxes);
};
// rol and col deadlock detection for test case 5
bool is_wall(int r, int c) {
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
        bool has_target = target_bitset[idx];

        if (is_wall_row(r + 1)) {
            has_target = false;
            for (int cc = 0; cc < cols; cc++) {
                if (target_bitset[r * cols + cc]) {
                    has_target = true;
                    break;
                }
            }
            if (!has_target) return true;
        }

        has_target = target_bitset[idx];
        if (is_wall_row(r - 1)) {
            has_target = false;
            for (int cc = 0; cc < cols; cc++) {
                if (target_bitset[r * cols + cc]) {
                    has_target = true;
                    break;
                }
            }
            if (!has_target) return true;
        }

        has_target = target_bitset[idx];
        if (is_wall_col(c + 1)) {
            has_target = false;
            for (int rr = 0; rr < rows; rr++) {
                if (target_bitset[rr * cols + c]) {
                    has_target = true;
                    break;
                }
            }
            if (!has_target) return true;
        }

        has_target = target_bitset[idx];
        if (is_wall_col(c - 1)) {
            has_target = false;
            for (int rr = 0; rr < rows; rr++) {
                if (target_bitset[rr * cols + c]) {
                    has_target = true;
                    break;
                }
            }
            if (!has_target) return true;
        }
    }
    return false;
}
// Mark unreachable cells as fragile, so that boxes cannot be pushed there
void mark_unreachable_as_fragile() {
    GridBitset reachable;
    queue<int> q;

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

            if (is_wall(nr, nc) || reachable[next_idx] || fragile_bitset[next_idx]) continue;

            reachable[next_idx] = 1;
            q.push(next_idx);
        }
    }

    for (int idx = 0; idx < rows * cols; ++idx) {
        if (!wall_bitset[idx] && !reachable[idx]) {
            fragile_bitset[idx] = 1;
        }
    }
}
// Precompute distances from each cell to each target using BFS
void precompute_distances() {
    int n = rows * cols;
    dist_to_target.assign(n, vector<int>(target_list.size(), INT_MAX / 2));
    int dr[] = {-1, 0, 1, 0};
    int dc[] = {0, -1, 0, 1};

    for (size_t j = 0; j < target_list.size(); ++j) {
        int goal = target_list[j];
        vector<int> dist(n, INT_MAX / 2);
        queue<int> q;
        q.push(goal);
        dist[goal] = 0;

        while (!q.empty()) {
            int cur = q.front();
            q.pop();

            int r = get_row(cur, cols);
            int c = get_col(cur, cols);

            for (int i = 0; i < 4; ++i) {
                int nr = r + dr[i];
                int nc = c + dc[i];
                int next = to_index(nr, nc, cols);

                if (is_wall(nr, nc) || dist[next] != INT_MAX / 2) continue;

                dist[next] = dist[cur] + 1;
                q.push(next);
            }
        }

        for (int pos = 0; pos < n; ++pos) {
            dist_to_target[pos][j] = dist[pos];
        }
    }
}

bool is_deadlock(int box_idx) {
    if (target_bitset[box_idx]) return false;

    int r = get_row(box_idx, cols);
    int c = get_col(box_idx, cols);
    if ((is_wall(r + 1, c) && is_wall(r, c + 1)) || (is_wall(r - 1, c) && is_wall(r, c - 1)) ||
        (is_wall(r - 1, c) && is_wall(r, c + 1)) || (is_wall(r + 1, c) && is_wall(r, c - 1)))
        return true;

    return false;
}

bool is_fragile(int idx) {
    return fragile_bitset[idx];
}
// Check if a box is frozen along an axis (horizontal or vertical)
bool is_blocked(int nr, int nc, const GridBitset& boxes, vector<bool>& checked, int orig_axis, const int dr[], const int dc[]) {
    if (is_wall(nr, nc)) return true;
    int idx = to_index(nr, nc, cols);
    if (is_fragile(idx)) return true;
    if (!boxes[idx]) return false;
    if (checked[idx]) return true;
    checked[idx] = true;
    int r = get_row(idx, cols);
    int c = get_col(idx, cols);
    bool blocked1 = is_blocked(r + dr[0], c + dc[0], boxes, checked, 1 - orig_axis, dr, dc);
    bool blocked2 = is_blocked(r + dr[1], c + dc[1], boxes, checked, 1 - orig_axis, dr, dc);
    return blocked1 && blocked2;
}
bool is_axis_frozen(int box_idx, int axis, const GridBitset& boxes, vector<bool>& checked) {
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
    bool blocked1 = is_blocked(r + dr[0], c + dc[0], boxes, checked, axis, dr, dc);
    bool blocked2 = is_blocked(r + dr[1], c + dc[1], boxes, checked, axis, dr, dc);
    return blocked1 && blocked2;
}
bool has_freeze_deadlock(const GridBitset& boxes) {
    for (int idx = 0; idx < rows * cols; ++idx) {
        if (!boxes[idx] || target_bitset[idx]) continue;
        vector<bool> checked(rows * cols, false);
        bool frozen_h = is_axis_frozen(idx, 0, boxes, checked);
        checked.assign(rows * cols, false);
        bool frozen_v = is_axis_frozen(idx, 1, boxes, checked);
        if (frozen_h && frozen_v) return true;
    }
    return false;
}

bool is_invalid_state(const GridBitset& boxes) {
    if (row_col_deadlock(boxes)) return true;
    // if (has_freeze_deadlock(boxes)) return true;  // Comment to speed up
    return false;
}
// Compute all reachable positions for the player using BFS
void compute_player_reach(int start_idx, const GridBitset& boxes, GridBitset& reachable, vector<int>& prev) {
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
            prev[next] = cur * 4 + i;
            q.push(next);
        }
    }
}

string get_path_to(int target_idx, int start_idx, const vector<int>& prev) {
    if (target_idx == start_idx) return "";

    string path;
    int cur = target_idx;
    char move_chars[] = {'W', 'A', 'S', 'D'};

    while (cur != start_idx) {
        if (prev[cur] == -1) return "";
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
    while (idx != 0) {
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

string AStar(const State& initial_state) {
    vector<State> states;
    priority_queue<PQItem, vector<PQItem>, greater<PQItem>> pq;
    unordered_map<GridBitset, GridBitset> visited;

    int states_explored = 0;

    State start_state = initial_state;
    start_state.cost = 0;
    start_state.parent = -1;
    start_state.transition = "";
    states.push_back(start_state);
    pq.push({0, start_state.cost + 5 * StateComparator::heuristic(start_state.boxes)});  // w=5

    while (!pq.empty()) {
        PQItem current_item = pq.top();
        pq.pop();

        State& current_state = states[current_item.state_index];

        GridBitset reachable;
        vector<int> prev(rows * cols, -1);
        compute_player_reach(current_state.player_idx, current_state.boxes, reachable, prev);

        if (visited[current_state.boxes][current_state.player_idx]) {
            continue;
        }
        visited[current_state.boxes] |= reachable;
        states_explored++;

        if ((current_state.boxes & target_bitset) == current_state.boxes) {
            // cerr << "States explored: " << states_explored << endl;
            return reconstruct_path(states, current_item.state_index);
        }

        int dr[] = {-1, 0, 1, 0};
        int dc[] = {0, -1, 0, 1};
        char move_chars[] = {'W', 'A', 'S', 'D'};

        struct WorkItem {
            int push_idx;
            int direction;
        };
        vector<WorkItem> work_items;
        for (int push_idx = 0; push_idx < rows * cols; ++push_idx) {
            if (!reachable[push_idx]) continue;

            int r = get_row(push_idx, cols);
            int c = get_col(push_idx, cols);

            for (int j = 0; j < 4; ++j) {
                int box_idx = to_index(r + dr[j], c + dc[j], cols);
                if (current_state.boxes[box_idx]) {
                    work_items.push_back({push_idx, j});
                }
            }
        }

        tbb::concurrent_vector<State> next_states_concurrent;

        size_t grain = max(size_t(1), work_items.size() / 24);
        tbb::parallel_for(tbb::blocked_range<size_t>(0, work_items.size(), grain),
            [&](const tbb::blocked_range<size_t>& range) {
                for (size_t i = range.begin(); i < range.end(); ++i) {
                    const WorkItem& work = work_items[i];
                    int push_idx = work.push_idx;
                    int j = work.direction;

                    int push_r = get_row(push_idx, cols);
                    int push_c = get_col(push_idx, cols);
                    int box_idx = to_index(push_r + dr[j], push_c + dc[j], cols);
                    int next_box_idx = to_index(push_r + 2 * dr[j], push_c + 2 * dc[j], cols);

                    int next_r = get_row(next_box_idx, cols);
                    int next_c = get_col(next_box_idx, cols);
                    if (is_wall(next_r, next_c) ||
                        current_state.boxes[next_box_idx] ||
                        is_fragile(next_box_idx) ||
                        is_deadlock(next_box_idx)) {
                        continue;
                    }

                    GridBitset temp_boxes = current_state.boxes;
                    temp_boxes[box_idx] = 0;
                    temp_boxes[next_box_idx] = 1;

                    if (is_invalid_state(temp_boxes)) {
                        continue;
                    }

                    string path_to_push = get_path_to(push_idx, current_state.player_idx, prev);
                    if (path_to_push.empty() && push_idx != current_state.player_idx) continue;

                    State next_state;
                    next_state.player_idx = box_idx;
                    next_state.boxes = temp_boxes;
                    next_state.transition = path_to_push + move_chars[j];
                    next_state.cost = current_state.cost + 1;  // Count pushes
                    next_state.parent = current_item.state_index;

                    next_states_concurrent.push_back(next_state);
                }
            });

        vector<State> next_states(next_states_concurrent.begin(), next_states_concurrent.end());

        for (auto& state : next_states) {
            auto it = visited.find(state.boxes);
            bool is_visited = (it != visited.end() && it->second[state.player_idx]);
            if (!is_visited) {
                states.push_back(state);
                int state_index = states.size() - 1;
                int h = StateComparator::heuristic(state.boxes);
                int priority = state.cost + 5 * h;  // w=5
                pq.push({state_index, priority});
            }
        }
    }

    // cerr << "States explored: " << states_explored << endl;
    return "";
}

int StateComparator::heuristic(const GridBitset& boxes) {
    vector<int> box_list;
    for (int idx = 0; idx < rows * cols; ++idx) {
        if (boxes[idx]) box_list.push_back(idx);
    }

    int k = box_list.size();
    int m = target_list.size();
    if (k > m) return INT_MAX;

    int total_distance = 0;
    vector<bool> used(m, false);

    for (int i = 0; i < k; ++i) {
        int box = box_list[i];
        int min_dist = INT_MAX;
        int best_j = -1;
        for (int j = 0; j < m; ++j) {
            if (used[j]) continue;
            int dist = dist_to_target[box][j];
            if (dist < min_dist) {
                min_dist = dist;
                best_j = j;
            }
        }
        if (best_j != -1) {
            used[best_j] = true;
            total_distance += min_dist;
        }
    }

    return total_distance;
}

int main(int argc, char* argv[]) {
    auto start_time = chrono::high_resolution_clock::now();

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
    fragile_bitset.reset();

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

    cols = (int)max_len;
    rows = (int)raw_lines.size();

    wall_bitset.reset();
    target_bitset.reset();

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
                target_list.push_back(idx);
            }
            if (ch == '@' || ch == '!') {
                fragile_bitset[idx] = 1;
            }
        }
    }

    mark_unreachable_as_fragile();

    precompute_distances();

    string solution = AStar(initial_state);

    auto end_time = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);

    // cerr << "Total time: " << duration.count() << " ms" << endl;
    cout << solution << endl;
    return 0;
}