from typing import Tuple
from numpy import array
import search
import numpy as np
from math import pi, sin, cos
import random

# you can want to use the class registration_iasd
# from your solution.py (from previous assignment)
from solution import registration_iasd


# Choose what you think it is the best data structure
# for representing actions.
Action = None

# Choose what you think it is the best data structure
# for representing states.


class State():

    def __init__(self, x_min = -180, x_max = 180, y_min = -180, y_max = 180, z_min = -180, z_max = 180):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max

        self.R = np.eye(3)
        self.t = [0,0,0]


class align_3d_search_problem(search.Problem):

    def __init__(
            self,
            scan1: array((...,3)),
            scan2: array((...,3)),
            ) -> None:
        """Module that instantiate your class.
        You CAN change the content of this __init__ if you want.
        :param scan1: input point cloud from scan 1
        :type scan1: np.array
        :param scan2: input point cloud from scan 2
        :type scan2: np.array
        """

        # Creates an initial state.
        # You may want to change this to something representing
        # your initial state.

        self.scan1 = scan1
        self.scan2 = scan2

        self.initial = State()
        

        return


    def actions(
            self,
            state: State
            ) -> Tuple:
        """Return the actions that can be executed in the given state.
        The result would be a list, since there are only four possible actions
        in any given state of the environment
        :param state: Abstract representation of your state
        :type state: State
        :return: Tuple with all possible actions
        :rtype: Tuple
        """

        available_actions = ()
        
        if state.x_min != state.x_max:
            available_actions = available_actions + ('x+',) +('x-',)
        
        if state.y_min != state.y_max:
            available_actions = available_actions + ('y+',) +('y-',)

        if state.z_min != state.z_max:
            available_actions = available_actions + ('z+',) +('z-',)

        return available_actions



    def result(
            self,
            state: State,
            action: Action
            ) -> State:
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state).
        :param state: Abstract representation of your state
        :type state: [type]
        :param action: An action
        :type action: [type]
        :return: A new state
        :rtype: State
        """

        x_min = state.x_min
        x_max = state.x_max
        y_min = state.y_min
        y_max = state.y_max
        z_min = state.z_min
        z_max = state.z_max

        if action[0] == 'x':
            if action[1] == '-':
                x_min = x_min
                x_max = x_min + (x_max - x_min)//2
            else:
                x_max = x_max
                x_min = x_max - (x_max - x_min)//2

        elif action[0] == 'y':
            if action[1] == '-':
                y_min = y_min
                y_max = y_min + (y_max - y_min)//2
            else:
                y_max = y_max
                y_min = y_max - (y_max - y_min)//2

        elif action[0] == 'z':
            if action[1] == '-':
                z_min = z_min
                z_max = z_min + (z_max - z_min)//2
            else:
                z_max = z_max
                z_min = z_max - (z_max - z_min)//2

        return State(x_min, x_max, y_min, y_max, z_min, z_max)

    def _distance(self, scan1, scan2):
        distances = 0

        for point in scan1:
            distance_vec = np.linalg.norm(scan2-point, axis = 1)
            min = np.amin(distance_vec)
            distances += min**2

        return np.sqrt(distances)

    def goal_test(
            self,
            state: State
            ) -> bool:
        """Return True if the state is a goal. The default method compares the
        state to self.goal or checks for state in self.goal if it is a
        list, as specified in the constructor. Override this method if
        checking against a single self.goal is not enough.
        :param state: gets as input the state
        :type state: State
        :return: returns true or false, whether it represents a node state or not
        :rtype: bool
        """

        x_mid = (state.x_min + (state.x_max - state.x_min)/2) * pi/180
        y_mid = (state.y_min + (state.y_max - state.y_min)/2) * pi/180
        z_mid = (state.z_min + (state.z_max - state.z_min)/2) * pi/180

        rot_x = np.array([[1, 0, 0], [0, cos(x_mid), -sin(x_mid)], [0, sin(x_mid), cos(x_mid)]])
        rot_y = np.array([[cos(y_mid), 0, sin(y_mid)], [0, 1, 0], [-sin(y_mid), 0, cos(y_mid)]])
        rot_z = np.array([[cos(z_mid), -sin(z_mid), 0], [sin(z_mid), cos(z_mid), 0], [0, 0, 1]])

        rotation_matrix = np.matmul(rot_z, np.matmul(rot_y, rot_x))

        scan1_updated = np.matmul(rotation_matrix, self.scan1.T).T
        
        scan1_sample = random.choices(scan1_updated, k = int(len(scan1_updated)*0.1))

        if self._distance(scan1_sample, self.scan2) < 0.5:

            reg = registration_iasd(scan1_updated, self.scan2)

            R, t = reg.get_compute()

            num_points, _ = scan1_updated.shape
            scan1_updated = (
                            np.matmul(R, scan1_updated.T) +
                            np.matmul(t.reshape((3,1)),np.ones((1,num_points)))
                            ).T

            scan1_updated = random.choices(scan1_updated, k = int(len(scan1_updated)*0.75))
            
            if self._distance(scan1_updated, self.scan2) < 0.1:
                state.R = np.matmul(R, rotation_matrix)
                state.t = t
                return True

        return False


    def path_cost(
            self,
            c,
            state1: State,
            action: Action,
            state2: State
            ) -> float:
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2. If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path.
        :param c: cost to get to the state1
        :type c: [type]
        :param state1: parent node
        :type state1: State
        :param action: action that changes the state from state1 to state2
        :type action: Action
        :param state2: state2
        :type state2: State
        :return: [description]
        :rtype: float
        """

        pass

    def heuristic(
                self,
                node) -> float:
            """Returns the heuristic at a specific node.
            note: use node.state to access the state
            :param node: node to include the heuristic
            :return: heuristic value
            :rtype: float
            """
        
            pass


def compute_alignment(
        scan1: array((...,3)),
        scan2: array((...,3)),
        ) -> Tuple[bool, array, array, int]:
    """Function that will return the solution.
    You can use any UN-INFORMED SEARCH strategy we study in the
    theoretical classes.
    :param scan1: first scan of size (..., 3)
    :type scan1: array
    :param scan2: second scan of size (..., 3)
    :type scan2: array
    :return: outputs a tuple with: 1) true or false depending on
        whether the method is able to get a solution; 2) rotation parameters
        (numpy array with dimension (3,3)); 3) translation parameters
        (numpy array with dimension (3,)); and 4) the depth of the obtained
        solution in the proposes search tree.
    :rtype: Tuple[bool, array, array, int]
    """

    scan1_average = np.mean(scan1, axis=0)
    scan2_average = np.mean(scan2, axis=0)

    scan1_aligned = scan1 - scan1_average
    scan2_aligned = scan2 - scan2_average

    output = search.breadth_first_graph_search(align_3d_search_problem(scan1_aligned, scan2_aligned))

    translation = - np.dot(output.state.R, scan1_average.T) + output.state.t + scan2_average

    return output!=None, output.state.R, translation, output.depth
    