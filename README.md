# Parallel Sokoban Solver

A high-performance Sokoban puzzle solver using A* algorithm and parallelization techniques.

## Project Overview

This project implements an advanced Sokoban solver with the following features:
- **A* Search Algorithm**: Uses heuristic functions to find optimal solutions
- **Parallel Processing**: Leverages Intel TBB library for multi-threaded acceleration
- **Deadlock Detection**: Implements multiple deadlock detection mechanisms to avoid invalid states
- **Optimized Design**: Uses bitset for improved memory efficiency and execution speed

## File Structure

```
Parallel-Sokoban-Solver/
├── main.cpp           # Main program and initialization
├── sokoban.h          # Header file: struct definitions and function declarations
├── deadlock.cpp       # Deadlock detection functions
├── astar.cpp          # A* search algorithm implementation
├── Makefile           # Build configuration
├── samples/           # Test cases
│   ├── 01.txt
│   ├── 02.txt
│   └── ...
└── utils/             # Utility tools
    ├── play.py        # Game player
    └── validate.py    # Solution validator
```

## System Requirements

- **Compiler**: g++ or clang++ with C++17 support
- **Libraries**:
  - Intel TBB (Threading Building Blocks)
  - Boost (for hash functions)
- **Operating System**: Linux, macOS, or Windows (with WSL)

## Installing Dependencies

### macOS (使用 Homebrew)
```bash
brew install tbb boost
```

### Ubuntu/Debian
```bash
sudo apt-get install libtbb-dev libboost-dev
```

### CentOS/RHEL
```bash
sudo yum install tbb-devel boost-devel
```

## Compilation

```bash
make clean
make
```

After compilation, the executable `hw1` will be generated.

## Usage

### Basic Usage
```bash
./hw1 <input_file>
```

### Examples
```bash
# Solve example level
./hw1 samples/01.txt

# Output move sequence
./hw1 samples/05.txt > solution.txt
```

## Input Format

Sokoban levels are represented using text files with the following symbols:

| Symbol | Meaning |
|--------|----------|
| `#` | Wall |
| `.` | Target position |
| `o` | Player |
| `x` | Box |
| `O` | Player on target |
| `X` | Box on target |
| `@` | Fragile floor (boxes cannot be pushed here) |
| `!` | Player on fragile floor |
| ` ` | Empty space |

### Example Level
```
#####
#.o.#
#x x#
#...#
#####
```

## Output Format

The program outputs a string containing the move sequence required to solve the level:
- `W`: Move up
- `A`: Move left  
- `S`: Move down
- `D`: Move right

## Algorithm Features

### A* Search
- Uses Manhattan distance as heuristic function
- Weight parameter w=5 balances search efficiency and solution quality
- Priority queue manages candidate states

### Deadlock Detection
1. **Corner Deadlock**: Box pushed to corner that's not a target position
2. **Row/Column Deadlock**: Entire row or column blocked by walls with no targets
3. **Freeze Deadlock**: Box completely blocked by other boxes (optional)

### Parallel Optimization
- Uses TBB parallel_for to accelerate state generation
- Dynamic grain size adjustment for optimal load balancing
- Concurrent containers avoid thread contention

### Memory Optimization
- BitSet representation of game states saves memory
- Pre-computed distance matrix accelerates heuristic calculation
- State deduplication avoids redundant searches

## Performance Characteristics

- **Time Complexity**: O(b^d), where b is branching factor and d is solution depth
- **Space Complexity**: O(b^d), mainly limited by visited state table
- **Parallelization Effect**: 2-8x speedup on multi-core systems
- **Applicable Scale**: Supports levels up to 16×16

## Utility Tools

### Game Player (play.py)
```bash
python utils/play.py samples/01.txt WASDWASD
```

### Solution Validator (validate.py)
```bash
python utils/validate.py samples/01.txt solution.txt
```

## Development Information

### Compilation Options
- `-std=c++17`: Use C++17 standard
- `-O3`: Highest level optimization
- `-pthread`: Multi-threading support
- `-fopenmp`: OpenMP support
- `-ltbb`: Link TBB library

### Debug Mode
Uncomment `cerr` outputs in the code to display debug information:
```cpp
cerr << "States explored: " << states_explored << endl;
```

## License

This project is licensed under the MIT License.

## Author

Gary Lee (gary920209)

## Acknowledgments

- Intel TBB team for providing excellent parallelization library
- Boost community for high-performance utility functions
- Sokoban game creator Hiroyuki Imabayashi