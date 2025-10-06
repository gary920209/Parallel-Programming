#include "sokoban.h"

// Helper functions for deadlock detection
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

// Wall checking functions
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

// Row and column deadlock detection for test case 5
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

// Simple deadlock detection (corner deadlock)
bool is_deadlock(int box_idx) {
    if (target_bitset[box_idx]) return false;

    int r = get_row(box_idx, cols);
    int c = get_col(box_idx, cols);
    if ((is_wall(r + 1, c) && is_wall(r, c + 1)) || (is_wall(r - 1, c) && is_wall(r, c - 1)) ||
        (is_wall(r - 1, c) && is_wall(r, c + 1)) || (is_wall(r + 1, c) && is_wall(r, c - 1)))
        return true;

    return false;
}

// Check if a position is fragile
bool is_fragile(int idx) {
    return fragile_bitset[idx];
}

// Check if boxes have freeze deadlock
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

// Check if the state is invalid (has deadlock)
bool is_invalid_state(const GridBitset& boxes) {
    if (row_col_deadlock(boxes)) return true;
    // if (has_freeze_deadlock(boxes)) return true;  // Comment to speed up
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