import itertools
import json
import search
import random
import math
import copy

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
        used_packages = {}
        for package, package_dict in data['packages'].items():
            if package_dict['belong'] != 'null':
                used_packages[package] = package_dict
        data['packages'] = used_packages
        data['clock'] = 0  # TESTIES

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

    def h(self, node):
        """ This is the heuristic. It gets a node (not a state,
        state can be accessed via node.state)
        and returns a goal distance estimate"""
        state = json.loads(node.state)

        price = 0
        # params:
        a = 0.7 # steps before lifting
        b = 2 # steps after lifting
        c = 4 # centroid caluster size
        d = 1.1 # bigger = better and slower

        for pack in state['packages'].keys():
            owner = state['packages'][pack]['belong']
            price += (b * self.manhattan(state['packages'][pack]['loc'], self.centroids_dict[owner]))

        dist_dict = {}
        overall_nearest = []
        if self.d_num >= self.p_num:
            for pack in state['packages']:
                dist_dict[pack] = []
                for drone in state['drones']:
                    dist_tuple = (self.manhattan(state['packages'][pack]['loc'], state['drones'][drone]['loc']), drone)
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

                    price += int((a * round(sum(points) / len(points))))
        price += state['clock']*d
        return price

    """Feel free to add your own functions
    (-2, -2, None) means there was a timeout"""


def create_drone_problem(game):
    return DroneProblem(game)
