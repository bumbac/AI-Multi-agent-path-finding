import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    ####################################################################
    # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    def find_solution(self):
        # restrained maximum number of steps in case there is no solution
        # example exp2_3.txt puts agent of higher priority in bottleneck position
        # which prevents other agent from achieving goal without collision
        maxSteps = 100
        start_time = timer.time()
        result = []
        constraints = []
        next = []
        """ Finds paths for all agents from their start locations to their goal locations."""
        print("DO YOU WANT TO SHOW COMPUTED PATH FINDING OR CONSTRAINED?")
        print("TYPE >CONST< IF YOU WANT CONSTRAINED, else for computed path finding.")
        answer = input()
        if answer == "CONST" or answer == "const":
            # naive solution with restrictions on movement for exp2_1.txt
            # not scalable
            constraints.append({'agent': 1, 'loc': [(1, 2), (1, 1)], 'timestep': 2})
            constraints.append({'agent': 1, 'loc': [(1, 3), (1, 2)], 'timestep': 3})
            constraints.append({'agent': 1, 'loc': [(1, 3), (1, 4)], 'timestep': 4})
            for i in range(self.num_of_agents):  # Find path for each agent
                path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                              i, constraints, maxSteps)
                if path is None:
                    raise BaseException('No solutions')
                result.append(path)
        else:
            # a star path finding for each agent in agent priority order
            # every step is restriction for movement from one position to another in concrete time-space
            for i in range(self.num_of_agents):  # Find path for each agent
                print(next)
                path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                              i, next, maxSteps)
                if path is None:
                    raise BaseException('No solutions')
                j = 2
                for x in range(len(path) - 1):
                    k = i + 1
                    while k < self.num_of_agents:
                        next.append({'agent': k, 'loc': [path[x], path[x + 1]], 'timestep': j})
                        k += 1
                    j += 1
                k = i + 1
                # timestep 0 represents agent staying at goal position until end
                # see single_agent_planner.py is_constrained() for details
                while k < self.num_of_agents:
                    next.append({'agent': k, 'loc': [path[len(path) - 1], path[len(path) - 1]], 'timestep': 0})
                    k += 1

                result.append(path)

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
#####################################################################
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
