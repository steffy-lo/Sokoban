#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems

def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are stored.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.

    # Create new lists to support indexing
    boxes = []
    storages = []
    for box in state.boxes:
        boxes.append(box)
    for storage in state.storage:
        storages.append(storage)

    manhattan = 0
    for box in boxes:
        nearest = float("inf")
        for storage in storages:
            distance = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
            if nearest > distance:
                nearest = distance
        manhattan += nearest
    return manhattan

#SOKOBAN HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  count = 0
  for box in state.boxes:
    if box not in state.storage:
        count += 1
  return count

def heur_alternate(state):
#IMPLEMENT
    '''a better heuristic
    Accounts for:
    - the sum of distance to the nearest matched storage
    - the sum of distance of robots to boxes
    - the sum of obstacles near a box going towards the matched storage
    - takes into account deadlocks
    '''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.

    boxes = []
    storages = []
    robots = []
    cost = 0

    for box in state.boxes:
        boxes.append(box)
        if box not in state.storage:
            cost += 1
    for storage in state.storage:
        storages.append(storage)
    for robot in state.robots:
        robots.append(robot)

    matched_goals = []
    for i in range(len(boxes)):
        if boxes[i] not in state.storage:
            if corner_deadlock(state, boxes[i]):
                # print("corner deadlock")
                return float("inf")
            if wall_box_deadlock(state, boxes, boxes[i]):
                # print("wall box deadlock")
                return float("inf")

        goal_to_match = []
        for storage in storages:
            if storage not in matched_goals:
                distance = abs(boxes[i][0] - storage[0]) + abs(boxes[i][1] - storage[1])
                goal_to_match.append([distance, storage])
        goal_to_match.sort()
        matched_goals.append(goal_to_match[0][1])

        if wall_goal_deadlock(state, boxes[i], matched_goals[i]):
            # print("wall goal deadlock")
            return float("inf")

        if boxes[i] not in state.storage:
            cost += abs(boxes[i][0] - matched_goals[i][0]) + abs(boxes[i][1] - matched_goals[i][1])
            cost += len(state.obstacles.intersection(get_directions(boxes[i], state, matched_goals, i)))
            directions = ((boxes[i][0]+1, boxes[i][1]), (boxes[i][0]+1, boxes[i][1]+1), (boxes[i][0]-1, boxes[i][1]+1),
                          (boxes[i][0]+1, boxes[i][1]-1), (boxes[i][0]-1, boxes[i][1]-1), (boxes[i][0]-1, boxes[i][1]),
                          (boxes[i][0], boxes[i][1]-1), (boxes[i][0], boxes[i][1]+1))
            cost += len(state.boxes.intersection(directions))
            cost += len(state.boxes.intersection(robots))

    min_dist_to_box = 0
    for robot in robots:
        nearest = float("inf")
        for box in boxes:
            distance = abs(robot[0] - box[0]) + abs(robot[1] - box[1])
            if nearest > distance:
                nearest = distance
        min_dist_to_box += nearest

    # if robots finished moving boxes to storage, they should move on to push other boxes
    if min_dist_to_box <= len(state.boxes.intersection(state.storage)):
        for i in range(len(boxes)):
            cost += abs(boxes[i][0] - matched_goals[i][0]) + abs(boxes[i][1] - matched_goals[i][1])
    else:
        cost += min_dist_to_box
    return cost

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight * sN.hval

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
    # Provides an implementation of anytime weighted a-star, as described in the HW1 handout
    # INPUT: a sokoban state that represents the start state and a timebound (number of seconds)
    # OUTPUT: A goal state (if a goal is found), else False
    # implementation of weighted astar algorithm

    start = os.times()[0]
    end = start + timebound

    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    search_engine = SearchEngine(strategy='custom', cc_level='default')
    search_engine.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)

    curr = float("inf")
    solution = search_engine.search(timebound, (curr, curr, curr))
    result = solution
    k = 1
    while start < end:
        if not solution:
            return result
        timebound -= (os.times()[0] - start)  # update timebound
        start = os.times()[0]
        if solution.gval <= curr:
            curr = solution.gval + heur_fn(solution)
            result = solution
        search_engine.fval_function = (lambda sN: fval_function(sN, weight - 1 * min(k, 10)))
        solution = search_engine.search(timebound, (solution.gval, heur_fn(solution), curr))
        k += 1
    return result

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
    # Provides an implementation of anytime greedy best-first search, as described in the HW1 handout
    # INPUT: a sokoban state that represents the start state and a timebound (number of seconds)
    # OUTPUT: A goal state (if a goal is found), else False
    # implementation of weighted astar algorithm
    #
    start = os.times()[0]
    end = start + timebound

    search_engine = SearchEngine(strategy='best_first', cc_level='default')
    search_engine.init_search(initial_state, sokoban_goal_state, heur_fn)
    curr = float("inf")
    solution = search_engine.search(timebound, (curr, curr, curr))
    result = False
    while start < end:
        if not solution:
            return result
        timebound -= (os.times()[0] - start)  # update timebound
        start = os.times()[0]
        if solution.gval <= curr:
            curr = solution.gval
            result = solution
        solution = search_engine.search(timebound, (curr, curr, curr))
    return result

# ============================================= Helper Functions =======================================================


def get_directions(position, state, matched_goals, i):
    directions = []
    if matched_goals[i][1] > position[1] and (position[0], position[1] + 1) in state.obstacles:
        # if nearest goal is below but there's obstacles
        directions.append((position[0], position[1] + 1))
        if position[0] > 0:
            directions.append((position[0] - 1, position[1]))  # go left if we can
            directions.append((position[0] - 1, position[1] + 1))
        if position[0] < state.width - 1:
            directions.append((position[0] + 1, position[1]))  # go right if we can
            directions.append((position[0] + 1, position[1] + 1))

    elif matched_goals[i][1] < position[1] and (position[0], position[1] - 1) in state.obstacles:
        # if nearest goal is above but there's obstacles
        directions.append((position[0], position[1] - 1))
        if position[0] > 0:
            directions.append((position[0] - 1, position[1]))  # go left if we can
            directions.append((position[0] - 1, position[1] - 1))
        if position[0] < state.width - 1:
            directions.append((position[0] + 1, position[1]))  # go right if we can
            directions.append((position[0] + 1, position[1] - 1))

    if matched_goals[i][0] > position[0] and (position[0] + 1, position[1]) in state.obstacles:
        # if nearest goal is to the right but there's obstacles
        directions.append((position[0] + 1, position[1]))
        if position[1] < state.height - 1:
            directions.append((position[0], position[1] + 1))  # go down if we can
            directions.append((position[0] + 1, position[1] + 1))
        if position[1] > 0:
            directions.append((position[0], position[1] - 1))  # go up if we can
            directions.append((position[0] + 1, position[1] - 1))

    elif matched_goals[i][0] < position[0] and (position[0] - 1, position[1]) in state.obstacles:
        # if nearest goal is to the left but there's obstacles
        directions.append((position[0] - 1, position[1]))
        if position[1] < state.height - 1:
            directions.append((position[0], position[1] + 1))  # go down if we can
            directions.append((position[0] - 1, position[1] + 1))
        if position[1] > 0:
            directions.append((position[0], position[1] - 1))  # go up if we can
            directions.append((position[0] - 1, position[1] - 1))

    return frozenset(tuple(directions))


def corner_deadlock(state, box):
    down = (box[1] == 0) or ((box[0], box[1] + 1) in state.obstacles)
    up = (box[1] == state.height - 1) or ((box[0], box[1] - 1) in state.obstacles)
    left = (box[0] == 0) or ((box[0] - 1, box[1]) in state.obstacles)
    right = (box[0] == state.width - 1) or ((box[0] + 1, box[1]) in state.obstacles)

    return (up or down) and (left or right)


def wall_goal_deadlock(state, box, storage):
    # Box is leaning against the rightmost or leftmost wall, yet the matched storage is not along the wall
    horz_deadlock = ((box[0] == 0) or (box[0] == state.width - 1)) and (box[0] != storage[0])

    # Box is leaning against the bottommost or topmost wall, yet the storage is not along the wall
    vert_deadlock = ((box[1] == 0) or (box[1] == state.height - 1)) and (box[1] != storage[1])

    return horz_deadlock or vert_deadlock


def wall_box_deadlock(state, boxes, box):
    # Box is not at storage and leaning against the rightmost or leftmost wall with another box next to it leaning on
    # the same wall
    horz_deadlock = ((box[0] == 0) or (box[0] == state.width - 1)) and \
                    ((box[0], box[1] + 1) in boxes or (box[0], box[1] - 1) in boxes)

    # Box is not at storage and leaning against the bottommost or topmost wall with another box next to it leaning on
    # the same wall
    vert_deadlock = ((box[1] == 0) or (box[1] == state.height - 1)) and \
                    ((box[0] + 1, box[1]) in boxes or (box[0] - 1, box[1]) in boxes)

    return horz_deadlock or vert_deadlock
