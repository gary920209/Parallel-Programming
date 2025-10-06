#!/bin/python
import sys
import argparse

from example_solver import loadstate, try_move, is_solved, Oops, DYDX


def validate(state, actions):
    for i, action in enumerate(actions):
        m, (y, x) = state
        try:
            dy, dx = DYDX[action]
        except KeyError:
            raise Oops(f'{action!r} is not a valid action')
        nextstate = try_move(m, y, x, dy, dx)
        if nextstate is None:
            raise Oops('invalid action at:\n{}\n{}"{}"{}'.format(
                '\n'.join(m), actions[:i], action, actions[i + 1:]))
        state = nextstate
    m, (y, x) = state
    if not is_solved(m):
        raise Oops('problem is not solved:\n{}'.format('\n'.join(m)))


def files(input, output):
    state = loadstate(input)
    try:
        if output == '-':
            output = sys.stdin.fileno()
        with open(output) as file:
            actions = file.read()
    except OSError as e:
        raise Oops(f'problem opening output file {e}')
    if not actions:
        raise Oops('output empty')
    if actions[-1] != '\n':
        raise Oops('output not ending with a newline')
    actions = actions[:-1]
    validate(state, actions)


def main0():
    parser = argparse.ArgumentParser()
    parser.add_argument('input')
    parser.add_argument('output')
    args = parser.parse_args()
    files(args.input, args.output)
    print('OK')


def main():
    try:
        main0()
    except Oops as oops:
        print(oops)
        raise SystemExit(2)


if __name__ == '__main__':
    main()
