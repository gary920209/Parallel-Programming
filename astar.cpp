#include "sokoban.h"

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

// Get path from start to target position
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

// Reconstruct the full path from initial state to goal
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

// Heuristic function for A* search
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

// A* search algorithm
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