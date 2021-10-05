from typing import Tuple
from math import sqrt, inf
import numpy as np
from copy import deepcopy

from visualization_vtk import point_clouds_visualization
import time

class registration:

    def __init__(
            self,
            scan_1: np.array((...,3)),
            scan_2: np.array((...,3))
            ) -> None:
        """initializes the registration class

        :param scan_1: gets as inputs the point cloud to be transformed
        :type scan_1: np.array
        :param scan_2: the point cloud in the final reference frame
        :type scan_2: np.array
        """
        self.scan_1 = deepcopy(scan_1)
        self.scan_2 = deepcopy(scan_2)

        return

    def find_closest_points(self) -> dict:
        """Computes the closest points in the two scans.
        There are many strategies. We are taking all the points in the first scan
        and search for the closes in the second. This means that we can have > than 1 points in scan
        1 corresponding to the same point in scan 2. All points in scan 1 will have correspondence.
        Points in scan 2 do not have necessarily a correspondence.

        :return: a dictionary with the correspondences. Keys are numbers identifying the id of the correspondence.
                    Values are a dictionaries with 'point_in_pc_1', 'point_in_pc_2' identifying the pair of points in the correspondence.
        :rtype: dict
        """
        
        # Dictionary of correspondance
        correspondance = {}

        row_1 = 0
        for point_1 in self.scan_1: #np.array, dim (N,3)
            # for every point in cloud 1

            closest_point = inf
            key = 0

            # Extract coordinates
            x_1 = point_1[0]
            y_1 = point_1[1]
            z_1 = point_1[2]

            row_2 = 0
            for point_2 in self.scan_2: #np.array, dim (N,3)
                # for every point in cloud 2

                # Extract coordinates
                x_2 = point_2[0]
                y_2 = point_2[1]
                z_2 = point_2[2]

                # Calculate geometric distance between the points:
                dist_between_points = sqrt((x_1-x_2)**2 + (y_1-y_2)**2 + (z_1-z_2)**2)

                # If distance is shorter, update
                if dist_between_points < closest_point:
                    closest_point = dist_between_points
                    key = int( (str(row_1) + str(row_2)))

                row_2 += 1

            # Save closest point
            correspondance[key] = { 'point_in_pc_1' : point_1, 
                                        'point_in_pc_1' : point_2,
                                        'dist2' : dist_between_points
                                        }
            row_1 += 1

        return correspondance

    def compute_pose(
            self,
            correspondences: dict
            ) -> Tuple[np.array, np.array]:
        """compute the transformation that aligns two
        scans given a set of correspondences

        :param correspondences: set of correspondences
        :type correspondences: dict
        :return: rotation and translation that align the correspondences
        :rtype: Tuple[np.array, np.array]
        """
        
        pass


    def __sum_square_error(
            self,
            correspondences: dict
            ) -> float:
        """Computes the sum of the square error for the
        computed correspondences

        :param correspondences: gets as input the set of correspondences
        :type correspondences: dict
        :return: the sum of the square error
        :rtype: float
        """
        return sum(correspondence['dist2'] for correspondence in correspondences.values())

    def __update_pointclouds(
            self,
            r: np.array((3,3)),
            t: np.array((3,))
            ) -> None:
        """Updates the 3D coordinates of all the points in
        in point cloud 1 (the one we want to be transformed to
        align with point cloud 2)

        :param r: the rotation matrix
        :type r: np.array
        :param t: translation vector
        :type t: np.array
        """
        num_points, _ = self.scan_1.shape
        self.scan_1 = (
                        np.dot(r, self.scan_1.T) +
                        np.dot(t.reshape((3,1)),np.ones((1,num_points)))
                        ).T

        return


    def compute(
            self,
            vtk_visualization: point_clouds_visualization = None,
            vtk_pc1: str = 'empty',
            max_iter: int = 100,
            step_limit: float = 10**(-16),
            show_visualization: bool=False,
            verbose: bool = False
            ) -> Tuple[np.array, np.array]: 
        """computes the 3D registration from a given set of pointclouds.

        :param vtk_visualization: for visualization, gets the header of the object, defaults to None
        :type vtk_visualization: point_clouds_visualization, optional
        :param vtk_pc1: label identifying the pointcloud for visualization, defaults to 'empty'
        :type vtk_pc1: str, optional
        :param max_iter: maximum number of iterations in the registration loop, defaults to 100
        :type max_iter: int, optional
        :param step_limit: delta for the stoping criteria, defaults to 10**(-16)
        :type step_limit: float, optional
        :param show_visualization: if want to visualized, defaults to True
        :type show_visualization: bool, optional
        :param verbose: if want to show details on the optimization, defaults to True
        :type verbose: bool, optional
        :return: 4x4 matrix with the output transformation
        :rtype: Tuple[np.array, np.array]
        """

        if verbose:
            print('3D registration verbose:')

        if show_visualization:
            vtk_visualization.render(block=False)
            list_lines = []

        output_transformation = np.eye(4)
        step_fn = inf

        # registration cycle!
        for iter_reg in range(max_iter):

            # if verbose, compute the iteration time
            if verbose:
                dt = time.time()

            # remove the lines shown in the previous frames
            if show_visualization:
                vtk_visualization.rm_lines(list_lines)
                list_lines = []

            # get the correspondences
            correspondences = self.find_closest_points()
            for correspondence in correspondences.values():
                point_pc_1 = correspondence['point_in_pc_1']
                point_pc_2 = correspondence['point_in_pc_2']
                # create the lines if we want to visualize
                if show_visualization:
                    actor = vtk_visualization.make_line(point_pc_1, point_pc_2, color=((0.1,0.8,0.1)), line_width=1)
                    list_lines.append(actor)

            # compute the sum of the square errors of
            # the correspondences
            current_fn = self.__sum_square_error(correspondences)

            # compute 3D pose
            r, t = self.compute_pose(correspondences)
            T = np.eye(4)
            T[0:3,0:3] = r
            T[0:3, 3] = t

            # integrates the output transformation
            output_transformation = np.dot(T, output_transformation)

            # update point cloud #1
            self.__update_pointclouds(r, t)

            # computed the step
            if iter_reg > 0:
                step_fn = abs(abs(previous_fn) - abs(current_fn))
            previous_fn = current_fn

            # verbose and visualization
            if verbose:
                elapsed = time.time() - dt
                print(
                    '  -> iteration #', iter_reg + 1,
                    '; execution time ', elapsed,
                    '; sum square distance error ', current_fn,
                    '; delta fn ', step_fn
                    )

            if show_visualization:
                vtk_visualization.render(block=False)
                time.sleep(.01)
                vtk_visualization.transform_pointcloud(vtk_pc1,output_transformation)

            # in case we reach the step increment limit
            if step_fn < step_limit:
                if verbose:
                    print('Reached the minimum fn step: ', step_fn, '<', step_limit)
                break

        if verbose:
            print('\nBest Rotation:')
            print(output_transformation)
            print(' ')
        if show_visualization:
            print('Close the window!')
            vtk_visualization.render(block=True)

        return output_transformation[0:3,0:3], output_transformation[0:3, 3]