from registration import registration
from get_pointcloud import point_cloud_data

import numpy as np
from typing import Tuple


class registration_iasd(registration):

    def __init__(self, scan_1: np.array((..., 3)), scan_2: np.array((..., 3))) -> None:

        # inherit all the methods and properties from registration
        super().__init__(scan_1, scan_2)

        return

    def compute_pose(self,correspondences: dict) -> Tuple[np.array, np.array]:
        """compute the transformation that aligns two
        scans given a set of correspondences

        :param correspondences: set of correspondences
        :type correspondences: dict
        :return: rotation and translation that align the correspondences
        :rtype: Tuple[np.array, np.array]
        """
        
        pass


    def find_closest_points(self) -> dict:
        """Computes the closest points in the two scans.
        There are many strategies. We are taking all the points in the first scan
        and search for the closes in the second. This means that we can have > than 1 points in scan
        1 corresponding to the same point in scan 2. All points in scan 1 will have correspondence.
        Points in scan 2 do not have necessarily a correspondence.

        :param search_alg: choose the searching option
        :type search_alg: str, optional
        :return: a dictionary with the correspondences. Keys are numbers identifying the id of the correspondence.
                Values are a dictionaries with 'point_in_pc_1', 'point_in_pc_2' identifying the pair of points in the correspondence.
        :rtype: dict
        """

        pass


class point_cloud_data_iasd(point_cloud_data):

    def __init__(
            self,
            fileName: str,
            ) -> None:
            
        super().__init__(fileName)

        return


    def load_point_cloud(
            self,
            file: str
            ) -> bool:

        #f = open(file, "r")
        #print(f.read())

        Array = []
        lines = []
        with open(file) as f:
            lines = f.readlines()

        print(f)

        count1 = -1
        count2 = 0
        count3 = 0
        is_vertex = False
        element_vertex = 0

        for line in lines:
            count1 += 1

            if "element vertex" in line:
                element_vertex = int(line[15:])
                print(element_vertex)

            if "property" in line:
                print(count3)
                print(line)

                if "property float x" in line:
                    x_index = count3
                if "property float y" in line:
                    y_index = count3
                if "property float z" in line:
                    z_index = count3

                count3 += 1
            
            if "end_header" in line:
                count2 = count1
                is_vertex = True

            #print(element_vertex)

            #if (((count1-count2) < element_vertex) and is_vertex):
            #    print(count1-count2)
            
            #print(Array)

        print(x_index)
        print(y_index)
        print(z_index)

        """Loads a point cloud from a ply file

        :param file: source file
        :type file: str
        :return: returns true or false, depending on whether the method
        the ply was OK or not
        :rtype: bool
        """
        
        pass