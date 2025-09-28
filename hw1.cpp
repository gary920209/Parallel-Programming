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
#include <boost/functional/hash.hpp> // Add Boost hash header
#include <mutex>
#include <atomic>

using namespace std;

struct Point{
    int r, c;

    bool operator==(const Point& other) const {
        return r == other.r && c == other.c;
    }
};

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
    vector<Point> boxes;
    vector<Point> fragiles;
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
            
            // Hash the vectors using boost
            for (const auto& box : s.boxes) {
                boost::hash_combine(seed, hash<Point>()(box));
            }
            
            for (const auto& fragile : s.fragiles) {
                boost::hash_combine(seed, hash<Point>()(fragile));
            }
            
            return seed;
        }
    };
}

vector<string> board;
vector<Point> targets;
int rows, cols;

struct StateComparator {
    bool operator()(const State& a, const State& b) const {
        return (a.cost + heuristic(a)) > (b.cost + heuristic(b));
    }
    
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

bool row_col_deadlock(const vector<Point>& boxes) {
    for (const auto& box : boxes) {
        int r = box.r, c = box.c;
        bool has_target = false;

        if (is_wall_row(r+1)) {
            for (int cc = 0; cc < cols; cc++) {
                if (find(targets.begin(), targets.end(), Point{r,cc}) != targets.end()) {
                    has_target = true;
                    break;
                }
                // if box on target, got it!
            }
            if (find(targets.begin(), targets.end(), box) != targets.end())
                has_target = true;

            if (!has_target) return true;
        }
        if (is_wall_row(r-1)) {
            for (int cc = 0; cc < cols; cc++) {
                if (find(targets.begin(), targets.end(), Point{r,cc}) != targets.end()) {
                    has_target = true;
                    break;
                }
            }
            if (find(targets.begin(), targets.end(), box) != targets.end())
                has_target = true;

            if (!has_target) return true;
        }

        if (is_wall_col(c+1)) {
            for (int rr = 0; rr < rows; rr++) {
                if (find(targets.begin(), targets.end(), Point{rr,c}) != targets.end()) {
                    has_target = true;
                    break;
                }
            }
            if (find(targets.begin(), targets.end(), box) != targets.end())
                has_target = true;

            if (!has_target) return true;
        }
        if (is_wall_col(c-1)) {
            for (int rr = 0; rr < rows; rr++) {
                if (find(targets.begin(), targets.end(), Point{rr,c}) != targets.end()) {
                    has_target = true;
                    break;
                }
            }
            if (find(targets.begin(), targets.end(), box) != targets.end())
                has_target = true;
            if (!has_target) return true;
        }
    }
    return false;
}

void mark_unreachable_as_fragile(State& initial_state) {
    // BFS from all target positions to find reachable cells
    unordered_set<Point> reachable;
    queue<Point> q;
    
    // Start BFS from all targets
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
    
    // Mark all unreachable non-wall cells as fragile
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            Point pos = {r, c};
            if (!is_wall(r, c) && reachable.find(pos) == reachable.end()) {
                // Check if this position is not already fragile
                if (find(initial_state.fragiles.begin(), initial_state.fragiles.end(), pos) == initial_state.fragiles.end()) {
                    initial_state.fragiles.push_back(pos);
                }
            }
        }
    }
    
    // Sort fragiles again after adding new ones
    sort(initial_state.fragiles.begin(), initial_state.fragiles.end(), [](const Point& a, const Point& b) {
        return a.r < b.r || (a.r == b.r && a.c < b.c);
    });
}

bool is_deadlock(const Point& box_pos){
    // if box in target, got it!
    if (find(targets.begin(),targets.end(), box_pos) != targets.end())
        return false;
    int r = box_pos.r, c = box_pos.c;
    if((is_wall(r+1,c) && is_wall(r,c+1)) || (is_wall(r-1,c) && is_wall(r,c-1)) || (is_wall(r-1,c) && is_wall(r,c+1)) || (is_wall(r+1,c) && is_wall(r,c-1)))
        return true;
        
    
    return false;
}

bool is_fragile(const Point& pos, const vector<Point>& fragiles) {
    return find(fragiles.begin(), fragiles.end(), pos) != fragiles.end();
}

string find_path(const Point& start, const Point& end, const vector<Point>& boxes, const vector<Point>& fragiles) {
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
            
            if (is_wall(next.r, next.c) || visited_pos.count(next) ||
                find(boxes.begin(), boxes.end(), next) != boxes.end() ||
                is_fragile(next, fragiles)) {
                continue;
            }
            
            visited_pos.insert(next);
            q.push({next, path + move_chars[i]});
        }
    }
    
    return "";
}
unordered_set<Point> get_reachable_positions(const Point& player_pos, const vector<Point>& boxes, const vector<Point>& fragiles) {
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
                        
            auto box_it = find(boxes.begin(), boxes.end(), next);
            if (box_it != boxes.end()) {
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
    // Use regular priority_queue (single-threaded)
    priority_queue<State, vector<State>, StateComparator> pq;
    tbb::concurrent_unordered_set<State> closed_set;
    
    State start_state = initial_state;
    start_state.cost = 0;
    pq.push(start_state);

    while(!pq.empty()){
        State current_state = pq.top();
        pq.pop();
        
        if (closed_set.count(current_state)) {
            continue;
        }
        closed_set.insert(current_state);
        
        bool all_on_target = true;
        for(const auto& box: current_state.boxes){
            if (find(targets.begin(), targets.end(), box) == targets.end()){
                all_on_target = false;
                break;
            }
        }
        if (all_on_target) {
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
                if (find(current_state.boxes.begin(), current_state.boxes.end(), box_pos) != current_state.boxes.end()) {
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
                    
                    auto box_it = find(current_state.boxes.begin(), current_state.boxes.end(), box_pos);
                    if (box_it == current_state.boxes.end()) continue;
                    
                    if (is_wall(next_box_pos.r, next_box_pos.c) ||
                        find(current_state.boxes.begin(), current_state.boxes.end(), next_box_pos) != current_state.boxes.end() ||
                        is_fragile(next_box_pos, current_state.fragiles) ||
                        is_deadlock(next_box_pos)) {
                        continue;
                    }
                    
                    // Create temporary next state to check for deadlocks
                    vector<Point> temp_boxes = current_state.boxes;
                    temp_boxes[distance(current_state.boxes.begin(), box_it)] = next_box_pos;
                    
                    // Check for row/column deadlock
                    if (row_col_deadlock(temp_boxes)) {
                        continue;
                    }
                    
                    string path_to_push = find_path(current_state.player, push_pos, current_state.boxes, current_state.fragiles);
                    
                    State next_state = current_state;
                    next_state.player = box_pos;
                    next_state.boxes[distance(current_state.boxes.begin(), box_it)] = next_box_pos;
                    sort(next_state.boxes.begin(), next_state.boxes.end(), [](const Point& a, const Point& b) {
                        return a.r < b.r || (a.r == b.r && a.c < b.c);
                    });
                    next_state.move += path_to_push + move_chars[j];
                    next_state.cost = current_state.cost + path_to_push.length() + 1;
                    
                    if (closed_set.find(next_state) == closed_set.end()) {
                        next_states.push_back(next_state);
                    }
                }
            });
        
        // After parallel region, push all generated states to priority_queue
        for (const auto& state : next_states) {
            pq.push(state);
        }
    }

    return ""; // No solution found
}

int StateComparator::heuristic(const State& state) {
    int total_distance = 0;
    vector<bool> target_used(targets.size(), false);
    
    for (const auto& box : state.boxes) {
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

    // parse board
    for (int i = 0; i < rows; ++i) {
        for (int c = 0; c < cols; ++c) {
            char ch = board[i][c];
            if (ch == 'o' || ch == 'O' || ch == '!') {
                initial_state.player = {i, c};
            }
            if (ch == 'x' || ch == 'X') {
                initial_state.boxes.push_back({i, c});
            }
            if (ch == '.' || ch == 'X' || ch == 'O') {
                targets.push_back({i, c});
            }
            if (ch == '@' || ch == '!') {
                initial_state.fragiles.push_back({i, c});
            }
        }
    }

    sort(initial_state.boxes.begin(), initial_state.boxes.end(), [](const Point& a, const Point& b) {
        return a.r < b.r || (a.r == b.r && a.c < b.c);
    });

    sort(initial_state.fragiles.begin(), initial_state.fragiles.end(), [](const Point& a, const Point& b) {
        return a.r < b.r || (a.r == b.r && a.c < b.c);
    });

    // Mark unreachable positions as fragile (deadlock detection)
    mark_unreachable_as_fragile(initial_state);

    string solution = AStar(initial_state);
    cout << solution << endl;
    return 0;
}