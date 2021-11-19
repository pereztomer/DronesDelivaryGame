import itertools
import json
import search
import random
import math

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
        used_packages = {}
        for package, package_dict in data['packages'].items():
            if package_dict['belong'] != 'null':
                used_packages[package] = package_dict
        data['packages'] = used_packages

        data = json.dumps(data, sort_keys=True)
        # print(data)
        # pupa= json.loads(data)
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
            if drone_dict['loc'][0] - 1 >= 0 and state['map'][drone_dict['loc'][0] - 1][drone_dict['loc'][1]] == 'P':
                possible_actions_dict[drone].append(drone + ' up')
            if drone_dict['loc'][0] + 1 < length and state['map'][drone_dict['loc'][0] + 1][
                drone_dict['loc'][1]] == 'P':
                possible_actions_dict[drone].append(drone + ' down')
            if drone_dict['loc'][1] - 1 >= 0 and state['map'][drone_dict['loc'][0]][drone_dict['loc'][1] - 1] == 'P':
                possible_actions_dict[drone].append(drone + ' left')
            if drone_dict['loc'][1] + 1 < width and state['map'][drone_dict['loc'][0]][drone_dict['loc'][1] + 1] == 'P':
                possible_actions_dict[drone].append(drone + ' right')

        for package, package_dict in state['packages'].items():
            if package_dict['holder'] == 'null' and package_dict['belong'] != 'null':
                for drone, drone_dict in state['drones'].items():
                    if drone_dict['loc'] == package_dict['loc']:
                        possible_actions_dict[drone].append('pick_up_' + drone + '_' + package)

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
        pass
        # for drone, drone_dict in state['drones']:
        #     for val in action:
        #         if drone in val:
        #             if 'up' in val:
        #             if 'down' in val:
        #             if 'left' in val:
        #             if 'right' in val:
        #             if 'wait' in val:
        #             if 'pick_up' in val:
        #
        #     pass

        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""

    def goal_test(self, state):
        """ Given a state, checks if this is the goal state.
         Returns True if it is, False otherwise."""
        state = json.loads(state)
        if not state['packages']:
            return True
        # for client in self.clients:
        #     for package in self.clients[client]['packages']:
        #         if state['packages'][package]['given'] == False:
        #             return False
        return False

    def h(self, node):
        """ This is the heuristic. It gets a node (not a state,
        state can be accessed via node.state)
        and returns a goal distance estimate"""
        return 0

    """Feel free to add your own functions
    (-2, -2, None) means there was a timeout"""


def create_drone_problem(game):
    return DroneProblem(game)
