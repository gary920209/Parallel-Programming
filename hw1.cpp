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
#include <omp.h> // Include the OpenMP header
#include <boost/functional/hash.hpp>
#include <mutex>
#include <atomic>
#include <bitset>

using namespace std;

const int MAX_SIZE = 256; // 16*16 max grid size
using GridBitset = bitset<MAX_SIZE>;

struct Point{
    int r, c;

    bool operator==(const Point& other) const {
        return r == other.r && c == other.c;
    }
};

// Helper functions for bitset operations
inline int point_to_index(const Point& p, int cols) {
    return p.r * cols + p.c;
}

inline Point index_to_point(int index, int cols) {
    return {index / cols, index % cols};
}

namespace std {
    template <>
    struct hash<Point> {
        size_t operator()(const Point& p) const {
            size_t seed = 0;
            boost::hash_combine(seed, p.r);
            boost::hash_combine(seed, p.c);
            return seed;
        }
    };
}

struct State{
    Point player;
    GridBitset boxes;
    GridBitset fragiles;
    string move;
    int cost;

    bool operator==(const State& other) const {
        return player == other.player && boxes == other.boxes && fragiles == other.fragiles;
    }
};

namespace std {
    template <>
    struct hash<State> {
        size_t operator()(const State& s) const {
            size_t seed = 0;
            boost::hash_combine(seed, hash<Point>()(s.player));

            // Hash bitsets efficiently
            auto boxes_hash = std::hash<GridBitset>{}(s.boxes);
            auto fragiles_hash = std::hash<GridBitset>{}(s.fragiles);
            boost::hash_combine(seed, boxes_hash);
            boost::hash_combine(seed, fragiles_hash);

            return seed;
        }
    };
}

vector<string> board;
vector<Point> targets;
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
    return board[r][c] == '#';
}
bool is_wall_row(int r) {
    for (int c = 0; c < cols; c++) {
        if (board[r][c] != '#') return false;
    }
    return true;
}

bool is_wall_col(int c) {
    for (int r = 0; r < rows; r++) {
        if (board[r][c] != '#') return false;
    }
    return true;
}

bool row_col_deadlock(const GridBitset& boxes) {
    for (int idx = 0; idx < rows * cols; ++idx) {
        if (!boxes[idx]) continue;

        Point box = index_to_point(idx, cols);
        int r = box.r, c = box.c;
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
    unordered_set<Point> reachable;
    queue<Point> q;

    for (const Point& target : targets) {
        if (!is_wall(target.r, target.c)) {
            q.push(target);
            reachable.insert(target);
        }
    }

    int dr[] = {-1, 0, 1, 0};
    int dc[] = {0, -1, 0, 1};

    while (!q.empty()) {
        Point current = q.front();
        q.pop();

        for (int i = 0; i < 4; ++i) {
            Point next = {current.r + dr[i], current.c + dc[i]};

            if (is_wall(next.r, next.c) || reachable.count(next)) {
                continue;
            }

            reachable.insert(next);
            q.push(next);
        }
    }

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            Point pos = {r, c};
            if (!is_wall(r, c) && reachable.find(pos) == reachable.end()) {
                initial_state.fragiles[point_to_index(pos, cols)] = 1;
            }
        }
    }
}

bool is_deadlock(const Point& box_pos){
    if (target_bitset[point_to_index(box_pos, cols)]) return false;

    int r = box_pos.r, c = box_pos.c;
    if((is_wall(r+1,c) && is_wall(r,c+1)) || (is_wall(r-1,c) && is_wall(r,c-1)) ||
       (is_wall(r-1,c) && is_wall(r,c+1)) || (is_wall(r+1,c) && is_wall(r,c-1)))
        return true;

    return false;
}

bool is_fragile(const Point& pos, const GridBitset& fragiles) {
    return fragiles[point_to_index(pos, cols)];
}

string find_path(const Point& start, const Point& end, const GridBitset& boxes, const GridBitset& fragiles) {
    if (start == end) return "";

    queue<pair<Point, string>> q;
    unordered_set<Point> visited_pos;

    q.push({start, ""});
    visited_pos.insert(start);

    int dr[] = {-1, 0, 1, 0};
    int dc[] = {0, -1, 0, 1};
    char move_chars[] = {'W', 'A', 'S', 'D'};

    while (!q.empty()) {
        auto [current, path] = q.front();
        q.pop();

        for (int i = 0; i < 4; ++i) {
            Point next = {current.r + dr[i], current.c + dc[i]};

            if (next == end) {
                return path + move_chars[i];
            }

            int next_idx = point_to_index(next, cols);
            if (is_wall(next.r, next.c) || visited_pos.count(next) ||
                boxes[next_idx] || fragiles[next_idx]) {
                continue;
            }

            visited_pos.insert(next);
            q.push({next, path + move_chars[i]});
        }
    }

    return "";
}
unordered_set<Point> get_reachable_positions(const Point& player_pos, const GridBitset& boxes, const GridBitset& fragiles) {
    unordered_set<Point> reachable;
    queue<Point> q;
    unordered_set<Point> visited_pos;

    q.push(player_pos);
    visited_pos.insert(player_pos);
    reachable.insert(player_pos);

    while (!q.empty()) {
        Point current = q.front();
        q.pop();

        int dr[] = {-1, 0, 1, 0};
        int dc[] = {0, -1, 0, 1};

        for (int i = 0; i < 4; ++i) {
            Point next = {current.r + dr[i], current.c + dc[i]};

            if (is_wall(next.r, next.c) || visited_pos.count(next)) {
                continue;
            }

            int next_idx = point_to_index(next, cols);
            if (boxes[next_idx]) {
                reachable.insert(current);
                continue;
            }

            visited_pos.insert(next);
            reachable.insert(next);
            q.push(next);
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

        unordered_set<Point> push_positions = get_reachable_positions(current_state.player, current_state.boxes, current_state.fragiles);
        
        int dr[] = {-1, 0, 1, 0};
        int dc[] = {0, -1, 0, 1};
        char move_chars[] = {'W', 'A', 'S', 'D'};
        
        // Create work items (position + direction pairs) for better load balancing
        struct WorkItem {
            Point push_pos;
            int direction;
        };
        
        std::vector<WorkItem> work_items;
        for (const Point& push_pos : push_positions) {
            for (int j = 0; j < 4; ++j) {
                Point box_pos = {push_pos.r + dr[j], push_pos.c + dc[j]};
                
                // Only add work item if there's actually a box at this position
                int box_idx = point_to_index(box_pos, cols);
                if (current_state.boxes[box_idx]) {
                    work_items.push_back({push_pos, j});
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
                    const Point& push_pos = work.push_pos;
                    int j = work.direction;
                    
                    Point box_pos = {push_pos.r + dr[j], push_pos.c + dc[j]};
                    Point next_box_pos = {box_pos.r + dr[j], box_pos.c + dc[j]};

                    int box_idx = point_to_index(box_pos, cols);
                    int next_box_idx = point_to_index(next_box_pos, cols);

                    if (!current_state.boxes[box_idx]) continue;

                    if (is_wall(next_box_pos.r, next_box_pos.c) ||
                        current_state.boxes[next_box_idx] ||
                        current_state.fragiles[next_box_idx] ||
                        is_deadlock(next_box_pos)) {
                        continue;
                    }

                    // Create temporary bitset to check for deadlocks
                    GridBitset temp_boxes = current_state.boxes;
                    temp_boxes[box_idx] = 0;
                    temp_boxes[next_box_idx] = 1;

                    if (row_col_deadlock(temp_boxes)) {
                        continue;
                    }

                    string path_to_push = find_path(current_state.player, push_pos, current_state.boxes, current_state.fragiles);

                    State next_state = current_state;
                    next_state.player = box_pos;
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
    vector<bool> target_used(targets.size(), false);

    // Iterate through set bits in boxes bitset
    for (int idx = 0; idx < rows * cols; ++idx) {
        if (!state.boxes[idx]) continue;

        Point box = index_to_point(idx, cols);
        int min_dist = INT_MAX;
        int best_target = -1;

        for (size_t i = 0; i < targets.size(); ++i) {
            if (target_used[i]) continue;
            int dist = abs(box.r - targets[i].r) + abs(box.c - targets[i].c);
            if (dist < min_dist) {
                min_dist = dist;
                best_target = i;
            }
        }

        if (best_target != -1) {
            target_used[best_target] = true;
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
    for (auto &ln : raw_lines) {
        if (ln.size() < max_len) ln += string(max_len - ln.size(), '#');
        board.push_back(ln);
    }
    rows = (int)board.size();

    // Initialize target bitset
    target_bitset.reset();

    // Parse board
    for (int i = 0; i < rows; ++i) {
        for (int c = 0; c < cols; ++c) {
            char ch = board[i][c];
            int idx = point_to_index({i, c}, cols);

            if (ch == 'o' || ch == 'O' || ch == '!') {
                initial_state.player = {i, c};
            }
            if (ch == 'x' || ch == 'X') {
                initial_state.boxes[idx] = 1;
            }
            if (ch == '.' || ch == 'X' || ch == 'O') {
                targets.push_back({i, c});
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