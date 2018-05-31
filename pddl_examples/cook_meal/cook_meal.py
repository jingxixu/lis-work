#!/usr/bin/env python2.7
from __future__ import print_function

import sys
sys.path.append("/Users/jingxixu/pddlstream")

from pddlstream.downward import solve_from_pddl
from pddlstream.incremental import solve_incremental, solve_exhaustive
from pddlstream.focused import solve_focused
from pddlstream.utils import print_solution, read
import os


def read_pddl(filename):
    directory = os.path.dirname(os.path.abspath(__file__))
    return read(os.path.join(directory, filename))

##################################################

def solve_pddl():
    domain_pddl = read_pddl('domain.pddl')
    problem_pddl = read_pddl('problem.pddl')

    plan, cost = solve_from_pddl(domain_pddl, problem_pddl)
    print('Plan:', plan)
    print('Cost:', cost)

##################################################

def get_problem():
    domain_pddl = read_pddl('domain.pddl')
    constant_map = {}
    stream_pddl = None
    stream_map = {}

    init = [
        ('isGripper', 'gripper'),
        ('isSpoon', 'spoon'),
        ('isSpatula', 'spatula'),
        ('isWok', 'wok'),
        ('isMeat', 'meat'),
        ('isVeg', 'veg'),
        ('isOil', 'oil'),
        ('isSalt', 'salt'),
        ('gripperEmpty', ),
        ('spoonEmpty', ),
        ('wokEmpty', ), ] \
    + [('onTable', x) for x in ['spoon', 'spatula', 'wok', 'meat', 'veg', 'oil', 'salt']] \
    + [('isGraspable', x) for x in ['meat', 'veg', 'spoon', 'spatula']] \
    + [('isScoopable', x) for x in ['oil', 'salt']]

    # goal = ['and', ('gripperHolding', 'spatula'), ('oilAdded', ), ('inwok', 'meat'), ('onTable', 'wok'), ('isSpatula', 'spatula')]
    goal = ['and', ('vegCooked', ), 
                   ('gripperEmpty', ), 
                   ('saltAdded', ),
                   ('oilAdded', ), 
                   ('meatCooked', )]

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

def solve_pddlstream(focused=True):
    pddlstream_problem = get_problem()
    if focused:
        solution = solve_focused(pddlstream_problem, unit_costs=True)
    else:
        #solution = solve_exhaustive(pddlstream_problem, unit_costs=True)
        solution = solve_incremental(pddlstream_problem, unit_costs=True)
    print_solution(solution)

##################################################

# TODO: could extract the FD parser by itself
# TODO: include my version of FD as a submodule

def main():
    #solve_pddl()
    solve_pddlstream()

if __name__ == '__main__':
    main()