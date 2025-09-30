#!/bin/python
import sys
import collections


class Oops(Exception):
    pass


def loadstate(filename):
    m = []
    o = None
    widths = []
    stats = collections.Counter()
    with open(filename) as file:
        for y, line in enumerate(file):
            line = line.rstrip()
            try:
                x = line.lower().index('o')
            except ValueError:
                pass
            else:
                o = y, x
            try:
                x = line.lower().index('!')
            except ValueError:
                pass
            else:
                o = y, x
            widths.append(len(line))
            m.append(line)
            stats.update(line)
    if not m:
        raise Oops('input file is empty')
    invalid_tiles = set(stats)
    invalid_tiles.difference_update('xXoO. #@!')
    if invalid_tiles:
        raise Oops(f'input file contains invalid characters: {invalid_tiles!r}')
    boxes = stats['x'] + stats['X']
    targets = stats['.'] + stats['X'] + stats['O']
    if boxes != targets:
        raise Oops(f"got {boxes} boxes and {targets} targets in input")
    nplayers = stats['o'] + stats['O'] + stats['!']
    if nplayers != 1:
        raise Oops(f'got {nplayers} players in input')
    if len(set(widths)) != 1:
        raise Oops(f'input rows having different widths: {widths!r}')
    return tuple(m), o


DYDX = {
    'W': (-1, 0),
    'A': (0, -1),
    'S': (1, 0),
    'D': (0, 1),
}


def try_move(m, y, x, dy, dx):
    """
    try to move at the given direction
    return the next game state if the move is valid
    return None otherwise
    """
    n = [list(r) for r in m]
    yy = y + dy
    yyy = yy + dy
    xx = x + dx
    xxx = xx + dx
    if m[yy][xx] == ' ':
        n[yy][xx] = 'o'
    elif m[yy][xx] == '.':
        n[yy][xx] = 'O'
    elif m[yy][xx] == '@':
        n[yy][xx] = '!'
    elif m[yy][xx] in ('x', 'X') and m[yyy][xxx] in (' ', '.'):
        if m[yy][xx] == 'x':
            n[yy][xx] = 'o'
        else:
            n[yy][xx] = 'O'
        if m[yyy][xxx] == ' ':
            n[yyy][xxx] = 'x'
        else:
            n[yyy][xxx] = 'X'
    else:
        return None
    if m[y][x] == 'o':
        n[y][x] = ' '
    elif m[y][x] == '!':
        n[y][x] = '@'
    else:
        n[y][x] = '.'
    return tuple(''.join(r) for r in n), (yy, xx)


def is_solved(m):
    return all(c != 'x' for r in m for c in r)


def main(filename: str):
    """
    Generalize the game into a search problem on a directed graph.
    A state is a tuple of (game map, player location).
    Each game state is a vertex on the graph.
    And there exists a edge from vertices v -> u if there
    exists a move that changes the game state from v to u.
    Here, BFS is used on the state graph, from the initial state,
    to obtain the solution.

    This code serves as a reference implementation that simply produces a
    solution, and it is implemented to (hopefully) be easy to understand,
    rather than to be performant.
    Keep this in mind when writing your code, and think about techniques
    to optimize it.
    """
    visited = {}
    state = loadstate(filename)
    visited = {state: ''}
    todo = collections.deque([state])
    while todo:
        m, (y, x) = currstate = todo.popleft()
        if is_solved(m):
            return visited[currstate]
        for key, (dy, dx) in DYDX.items():
            if newstate := try_move(m, y, x, dy, dx):
                if newstate not in visited:
                    visited[newstate] = visited[currstate] + key
                    todo.append(newstate)
    raise Oops('no solution')


if __name__ == '__main__':
    print(main(sys.argv[1]))
