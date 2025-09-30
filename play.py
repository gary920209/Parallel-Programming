#!/bin/python

import sys
from functools import partial

from example_solver import is_solved, try_move, loadstate, DYDX


clear = '\x1bc\x1b]104\x07\x1b[!p\x1b[?3;4l\x1b[4l\x1b>\x1b[?69l'


def play(state):
    def print(*s):
        __builtins__.print(*s, end='\r\n', flush=True)
    width = len(state[0])
    states = [state]
    actions = []
    err = ''
    while True:
        __builtins__.print(end=clear)
        m, (y, x) = states[-1]
        for row in m:
            print(row)
        print('current action sequence:', ''.join(actions))
        print('solved:', is_solved(m))
        print(err)
        err = ''

        key = ''
        while not key:
            print('W/A/S/D to move; U to undo; Q to quit')
            key = sys.stdin.read(1).upper().strip()
        if key == 'Q':
            raise SystemExit(0)
        elif key == 'U':
            if actions:
                actions.pop()
                states.pop()
            else:
                err = 'cannot undo'
        else:
            try:
                dy, dx = DYDX[key]
            except KeyError:
                err = f'{key!r} is not a valid key'
            else:
                if newstate := try_move(m, y, x, dy, dx):
                    actions.append(key)
                    states.append(newstate)
                else:
                    err = f'cannot move at {key!r}'


def main():
    import argparse
    import tty
    STDIN = sys.stdin.fileno()
    restore = tty.tcgetattr(STDIN)
    tty.setraw(STDIN)
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('filename')
        args = parser.parse_args()
        play(loadstate(args.filename))
    finally:
        tty.tcsetattr(STDIN, tty.TCSAFLUSH, restore)


if __name__ == '__main__':
    main()
