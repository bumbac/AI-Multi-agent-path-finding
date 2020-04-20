import heapq


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


####################################################################
# vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    constraints_table = []
    for constraint in constraints:
        if constraint['agent'] == agent:
            constraints_table.append(constraint)
    return constraints_table
####################################################################
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


####################################################################
# vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    # Check if a move from curr_loc to next_loc at time step next_time violates
    # any given constraint. For efficiency the constraints are indexed in a constraint_table
    # by time step, see build_constraint_table.
    # represents number of steps of other agents
    maximum = 0
    # goal restriction flag, agent is on goal position for timesteps > maximum
    flag = False
    for const in constraint_table:
        if const['loc'] == [curr_loc, next_loc] or const['loc'][1] == next_loc or const['loc'] == [next_loc, curr_loc]:
            if const['timestep'] == 0:
                flag = True
            if const['timestep'] == next_time:
                return True
            maximum = max(const['timestep'], maximum)
    # if timestep of action is bigger than highest step collision could happen only if agent achieved goal position
    # (flag == True)
    if flag and next_time > maximum:
        return True
    return False
####################################################################
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


####################################################################
# vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints, maxSteps):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    # A star search in the space-time domain rather than space domain, only.
    # agent starts at timestep 1, movement from start position to any position has incremented timestep by one
    # agent A from start (timestep 1) to any position(timestep 2)

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 1
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': earliest_goal_timestep}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0 and maxSteps > 0:
        maxSteps -= 1
        curr = pop_node(open_list)
        earliest_goal_timestep = curr['timestep']
        earliest_goal_timestep += 1
        tab = build_constraint_table(constraints, agent)
        #############################
        # goal test condition to handle goal constraints
        if curr['loc'] == goal_loc:
            # checking if staying at goal positions collides with earlier agents
            if not is_constrained(curr['loc'], curr['loc'], earliest_goal_timestep, tab):
                curr['timestep'] = 0
                return get_path(curr)
            # goal position is still relevant but not at this timestamp
            closed_list[(curr['loc'], curr['timestep'])] = curr
            continue
        # make child movements in 5 directions top, right, bottom, left, stay
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            # is wall
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'timestep': earliest_goal_timestep}
            # collision check
            if is_constrained(curr['loc'], child['loc'], earliest_goal_timestep, tab):
                closed_list[(child['loc'], child['timestep'])] = child
                continue
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                # a star check
                """Return true is child is better than existing_node."""
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
####################################################################
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
