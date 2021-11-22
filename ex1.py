import itertools
import json
import search
import random
import math
import copy
import numpy as np

ids = ["318295029", "316327451"]


class DroneProblem(search.Problem):
    """This class implements a medical problem according to problem description file"""

    def __init__(self, initial):
        """Don't forget to implement the goal test
        You should change the initial to your own representation.
        search.Problem.__init__(self, initial) creates the root node"""
        drone_init = {}
        package_init = {}
        clients_init = {}
        for key in initial['drones']:
            drone_init[key] = {'loc': initial['drones'][key], 'holding': ["null", "null"]}
        for key in initial['packages']:
            package_init[key] = {'loc': initial['packages'][key], 'belong': "null", 'holder': "null"}
        for key in initial['clients']:
            clients_init[key] = {'path': initial['clients'][key]['path'],
                                 'packages': initial['clients'][key]['packages'],
                                 'pattern_cur': 0,
                                 'loc': initial['clients'][key]['path'][0]}
        for client in clients_init:
            for package in clients_init[client]['packages']:
                package_init[package]['belong'] = client
        data = {
            'drones': drone_init,
            'packages': package_init,
            'clients': clients_init
        }
        self.map = initial['map']
        self.clock = 0
        self.d_num = len(drone_init.keys())
        self.p_num = len(package_init.keys())
        self.centroids_dict = {}
        for client in clients_init:
            self.centroids_dict[client] = self.find_centroid(clients_init[client]['path'])

            # self.centroids_dict[client]=clients_init[client]['path'][0]
        used_packages = {}
        for package, package_dict in data['packages'].items():
            if package_dict['belong'] != 'null':
                used_packages[package] = package_dict
        data['packages'] = used_packages
        data['clock'] = 0  # TESTIES
        ######## dist metric ##########
        adj_dict = self.set_up_graph(self.map)
        dist_table = {}
        for point, value in adj_dict.items():
            dist_table[point] = {}
        for index_1, y in enumerate(self.map):
            for index_2, x in enumerate(y):
                if self.map[index_1][index_2] == 'I':
                    continue
                root = (index_1, index_2)
                self.bfs(graph=adj_dict, root=root)
                self.fill_table(graph=adj_dict, node=root, dist_table=dist_table)
                self.reset(self.map, adj_dict)

        self.bfs_dist = dist_table
        # for key, value in dist_table.items():
        # print(f'The key is {key} and the value is {value}')

        data = json.dumps(data, sort_keys=True)
        search.Problem.__init__(self, data)

    def actions(self, state):
        """Returns all the actions that can be executed in the given s
               state. The result should be a tuple (or other iterable) of actions
               as defined in the problem description file"""

        state = json.loads(state)
        width = len(self.map)
        length = len(self.map[0])
        possible_actions_dict = {}
        ########    MOVE   #########
        for drone in state['drones'].keys():
            possible_actions_dict[drone] = []
            possible_actions_dict[drone].append(('wait', drone))
        for drone, drone_dict in state['drones'].items():
            loc1 = drone_dict['loc'][0]
            loc2 = drone_dict['loc'][1]
            if loc1 - 1 >= 0 and self.map[loc1 - 1][loc2] == 'P':
                possible_actions_dict[drone].append(('move', drone, (loc1 - 1, loc2)))
            if loc1 + 1 < width and self.map[loc1 + 1][loc2] == 'P':
                possible_actions_dict[drone].append(('move', drone, (loc1 + 1, loc2)))
            if loc2 - 1 >= 0 and self.map[loc1][loc2 - 1] == 'P':
                possible_actions_dict[drone].append(('move', drone, (loc1, loc2 - 1)))
            if loc2 + 1 < length and self.map[loc1][loc2 + 1] == 'P':
                possible_actions_dict[drone].append(('move', drone, (loc1, loc2 + 1)))

        ########    PICK   ##########
        for package, package_dict in state['packages'].items():
            if package_dict['holder'] == 'null':
                for drone, drone_dict in state['drones'].items():
                    have_empty_place = drone_dict['holding'][0] == 'null' or drone_dict['holding'][1] == 'null'
                    if drone_dict['loc'] == package_dict['loc'] and have_empty_place:
                        possible_actions_dict[drone].append(('pick up', drone, package))

        ########    DELIVER   ##########
        clients_and_robots = {}
        for drone in state['drones']:
            for client in state['clients']:
                if state['clients'][client]['loc'] == state['drones'][drone]['loc']:
                    if drone in clients_and_robots.keys():
                        clients_and_robots[drone].append(client)
                    else:
                        clients_and_robots[drone] = [client]

        for drone in clients_and_robots.keys():
            for i in range(2):
                name_of_pack = state['drones'][drone]['holding'][i]
                if name_of_pack != 'null' and state["packages"][name_of_pack]["belong"] in clients_and_robots[drone]:
                    possible_actions_dict[drone].append(
                        ('deliver', drone, state["packages"][name_of_pack]["belong"], name_of_pack))

        all_possible_actions = []
        for drone in state['drones'].keys():
            all_possible_actions.append(possible_actions_dict[drone])
        # all_possible_actions = tuple(all_possible_actions)
        all_possible_actions = list(itertools.product(*all_possible_actions))

        for index, possible_action in enumerate(all_possible_actions):
            test_lst = []
            for drone_combination in possible_action:
                if drone_combination[0] == 'pick up':
                    test_lst.append(drone_combination[2])
            if len(test_lst) != len(set(test_lst)):
                del all_possible_actions[index]

        # all_possible_actions = copy_possible_actions

        return tuple(all_possible_actions)

    def MoveTurn(self, state):
        self.clock += 1
        state['clock'] += 1
        # turn_Num = self.clock
        for client in state['clients']:
            path_idx = state['clients'][client]['pattern_cur']
            path_len = len(state['clients'][client]['path'])
            if path_len - 1 == path_idx:
                state['clients'][client]['pattern_cur'] = 0
            else:
                state['clients'][client]['pattern_cur'] += 1
            path_idx = state['clients'][client]['pattern_cur']
            state['clients'][client]['loc'] = state['clients'][client]['path'][path_idx]
        return state

    def result(self, state, action):
        state = json.loads(state)
        for specific_drone_action in action:
            action_type = specific_drone_action[0]
            d_name = specific_drone_action[1]

            if action_type == 'wait':
                continue
            # (“move”,“drone_name”, (x, y))
            elif action_type == 'move':
                new_loc = specific_drone_action[2]
                state['drones'][d_name]['loc'] = new_loc
                for p in state['drones'][d_name]['holding']:
                    if p != 'null':
                        state['packages'][p]['loc'] = new_loc
            # (“pick up”,“drone_name”, “package_name”)
            elif action_type == 'pick up':
                p_name = specific_drone_action[2]
                if state['drones'][d_name]['holding'][0] == 'null':
                    state['drones'][d_name]['holding'][0] = p_name
                else:
                    state['drones'][d_name]['holding'][1] = p_name
                state['packages'][p_name]['holder'] = d_name
                # state['packages'][p_name]['loc'] = (-1, -1)
                state['packages'][p_name]['loc'] = state['drones'][d_name]['loc']  # NEW!
                # (“deliver”, “drone_name”, “client_name”, “package_name”).
            elif action_type == 'deliver':
                c_name = specific_drone_action[2]
                p_name = specific_drone_action[3]
                del state['packages'][p_name]
                state['clients'][c_name]['packages'].remove(p_name)
                if state['drones'][d_name]['holding'][0] == p_name:  # i can DELETE WRONG PACK IF I HAVE DIFFERENT 2!
                    state['drones'][d_name]['holding'][0] = 'null'
                elif state['drones'][d_name]['holding'][1] == p_name:
                    state['drones'][d_name]['holding'][1] = 'null'
                else:
                    print("WTF ?!")
        state = self.MoveTurn(state)

        """Return the state that results from executing the given
              action in the given state. The action must be one of
              self.actions(state)."""
        return json.dumps(state, sort_keys=True)

    def goal_test(self, state):
        """ Given a state, checks if this is the goal state.
         Returns True if it is, False otherwise."""
        state = json.loads(state)
        if not state['packages']:
            return True

        return False

    def manhattan(self, a, b):
        return sum(abs(val1 - val2) for val1, val2 in zip(a, b))

    def find_centroid(self, list_of_points):
        X = [item[0] for item in list_of_points]
        Y = [item[1] for item in list_of_points]
        return round(sum(X) / len(X)), round(sum(Y) / len(Y))

    def geometric_median(self, X, numIter=200):
        """
        Compute the geometric median of a point sample.
        The geometric median coordinates will be expressed in the Spatial Image reference system (not in real world metrics).
        We use the Weiszfeld's algorithm (http://en.wikipedia.org/wiki/Geometric_median)

        :Parameters:
         - `X` (list|np.array) - voxels coordinate (3xN matrix)
         - `numIter` (int) - limit the length of the search for global optimum

        :Return:
         - np.array((x,y,z)): geometric median of the coordinates;
        """
        # -- Initialising 'median' to the centroid
        y = np.mean(X, 1)
        # -- If the init point is in the set of points, we shift it:
        while (y[0] in X[0]) and (y[1] in X[1]) and (y[2] in X[2]):
            y += 0.1

        convergence = False  # boolean testing the convergence toward a global optimum
        dist = []  # list recording the distance evolution

        # -- Minimizing the sum of the squares of the distances between each points in 'X' and the median.
        i = 0
        while (not convergence) and (i < numIter):
            num_x, num_y, num_z = 0.0, 0.0, 0.0
            denum = 0.0
            m = X.shape[1]
            d = 0
            for j in range(0, m):
                div = math.sqrt((X[0, j] - y[0]) ** 2 + (X[1, j] - y[1]) ** 2 + (X[2, j] - y[2]) ** 2)
                num_x += X[0, j] / div
                num_y += X[1, j] / div
                num_z += X[2, j] / div
                denum += 1. / div
                d += div ** 2  # distance (to the median) to miminize
            dist.append(d)  # update of the distance evolution

            if denum == 0.:
                return [0, 0, 0]

            y = [num_x / denum, num_y / denum, num_z / denum]  # update to the new value of the median
            if i > 3:
                convergence = (
                        abs(dist[i] - dist[i - 2]) < 0.1)  # we test the convergence over three steps for stability
                # ~ print abs(dist[i]-dist[i-2]), convergence
            i += 1
        if i == numIter:
            raise ValueError("The Weiszfeld's algoritm did not converged after" + str(numIter) + "iterations !!!!!!!!!")
        # -- When convergence or iterations limit is reached we assume that we found the median.

        return np.array(y)

    def h(self, node):
        """ This is the heuristic. It gets a node (not a state,
        state can be accessed via node.state)
        and returns a goal distance estimate"""
        state = json.loads(node.state)

        price = 0
        # params:
        a = 1  # steps before lifting
        b = 1  # steps after lifting!
        c = 4  # centroid caluster size
        d = 0.8  # bigger = better and slower

        for pack in state['packages'].keys():
            owner = state['packages'][pack]['belong']
            try:  # consider better solution, centroind in unreachable
                price += (b * self.bfs_dist[tuple(state['packages'][pack]['loc'])][self.centroids_dict[owner]])
            except:
                price += (b * self.bfs_dist[tuple(state['packages'][pack]['loc'])][tuple(state['clients'][owner]['path'][0])])

        dist_dict = {}
        overall_nearest = []
        if self.d_num >= self.p_num:
            for pack in state['packages']:
                dist_dict[pack] = []
                for drone in state['drones']:
                    dist_tuple = (self.bfs_dist[tuple(state['packages'][pack]['loc'])][tuple(state['drones'][drone]['loc'])], drone)
                    # dist_tuple = (self.manhattan(state['packages'][pack]['loc'], state['drones'][drone]['loc']), drone)
                    dist_dict[pack].append(dist_tuple)
                dist_dict[pack] = (sorted(dist_dict[pack]))
                overall_nearest.append((dist_dict[pack][0], pack))

            used_drones = []
            used_packs = []
            overall_nearest = sorted(overall_nearest)
            while overall_nearest:
                current_leader = overall_nearest.pop(0)
                leading_drone = current_leader[0][1]
                leading_pack = current_leader[1]
                if leading_pack not in used_packs and leading_drone not in used_drones:
                    price += a * current_leader[0][0]
                    used_drones.append(leading_drone)
                    used_packs.append(leading_pack)
        else:

            for drone in state['drones']:
                dist_dict[drone] = []
                # dist_dict[drone + "b"] = []
                if state['packages']:
                    for pack in state['packages']:
                        dist_tuple = (
                            self.manhattan(state['drones'][drone]['loc'], state['packages'][pack]['loc']), pack)
                        dist_dict[drone].append(dist_tuple)
                    dist_dict[drone] = sorted(dist_dict[drone])
                    points = []
                    for i in range(c):
                        try:
                            points.append(dist_dict[drone][i][0])
                        except:
                            price += 0

                    price += (a * round(sum(points) / len(points)))
        price += state['clock'] * d
        return price

    """Feel free to add your own functions
    (-2, -2, None) means there was a timeout"""

    def set_up_graph(self, map):
        adj_dict = {}

        for index_1, y in enumerate(map):
            for index_2, x in enumerate(y):
                if map[index_1][index_2] == 'P':
                    adj_dict[(index_1, index_2)] = ([], ['white', 1000000, 'null'])

        for index_1, y in enumerate(map):
            for index_2, x in enumerate(y):
                if map[index_1][index_2] == 'I':
                    continue
                if index_1 - 1 >= 0 and map[index_1 - 1][index_2] == 'P':
                    adj_dict[(index_1, index_2)][0].append((index_1 - 1, index_2))
                if index_1 + 1 < len(map) and map[index_1 + 1][index_2] == 'P':
                    adj_dict[(index_1, index_2)][0].append((index_1 + 1, index_2))
                if index_2 - 1 >= 0 and map[index_1][index_2 - 1] == 'P':
                    adj_dict[(index_1, index_2)][0].append((index_1, index_2 - 1))
                if index_2 + 1 < len(map[0]) and map[index_1][index_2 + 1] == 'P':
                    adj_dict[(index_1, index_2)][0].append((index_1, index_2 + 1))

        return adj_dict

    def reset(self, map, adj_dict):
        for index_1, y in enumerate(map):
            for index_2, x in enumerate(y):
                if map[index_1][index_2] == 'P':
                    adj_dict[(index_1, index_2)][1][0] = 'white'
                    adj_dict[(index_1, index_2)][1][1] = 1000000
                    adj_dict[(index_1, index_2)][1][2] = 'null'

    def bfs(self, graph, root):
        visited = []  # List to keep track of visited nodes.
        queue = []  # Initialize a queue

        graph[root][1][0] = 'grey'
        graph[root][1][1] = 0
        visited.append(root)
        queue.append(root)

        while queue:
            s = queue.pop(0)
            for neighbour in graph[s][0]:
                if graph[neighbour][1][0] == 'white':
                    graph[neighbour][1][0] = 'grey'
                    graph[neighbour][1][1] = graph[s][1][1] + 1
                    graph[neighbour][1][2] = s
                    queue.append(neighbour)

    def fill_table(self, graph, node, dist_table):
        for vertix in graph:
            dist_table[node][vertix] = graph[vertix][1][1]


def create_drone_problem(game):
    return DroneProblem(game)
