#include "sokoban.h"

// Global variable definitions
GridBitset wall_bitset;
GridBitset target_bitset;
GridBitset fragile_bitset;
int rows, cols;
vector<int> target_list;
vector<vector<int>> dist_to_target;

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