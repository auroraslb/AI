from registration import registration
from get_pointcloud import point_cloud_data

import numpy as np
from typing import Tuple
from math import sqrt


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

        pi = []
        qi = []

        for value in correspondences.keys():
            pi.append(correspondences[value]['point_in_pc_1'])
            qi.append(correspondences[value]['point_in_pc_2'])

        #Get the center point of S1 
        p_average = np.mean(pi, axis=0)
        #Get the center point of S2
        q_average = np.mean(qi, axis=0)

        #Frame aligned with the center of S1
        p_aligned = pi - p_average
        #Frame aligned with the center of S2
        q_aligned = qi - q_average

        #Stack all ep i in a matrix of dimension N x 3
        P = np.stack( p_aligned, axis=0 )
        #Stack all eq i in a matrix of dimension N x 3
        Q = np.stack( q_aligned, axis=0 )

        #Get matrix A of dimension 3 x 3
        A = (Q.transpose()).dot(P)

        #U, Sum, and Vt using SVD, all of dimension 3 x 3
        U, Sum, V_t = np.linalg.svd(A, full_matrices=True)
        
        #Get the output rotation
        R_out = (U.dot(np.diag([1,1,np.linalg.det(U.dot(V_t))]))).dot(V_t)
        
        #Get the output translation
        t_out = q_average - R_out.dot(p_average)
        
        #Return tuple of R_out and t_out
        return R_out, t_out


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
        # Dict to be returned from function
        correspondences_dict = {}

        scan_1 = self.scan_1
        scan_2 = self.scan_2

        for pc_1_index in range(len(scan_1)):

            # The result of these three operations is a vector with distances. Using numpy functions to improve speed.
            square = np.square(scan_2-scan_1[pc_1_index])
            row_sum = np.sum(square, axis=1)
            square_root = np.sqrt(row_sum)

            # Extracting the closest point in scan2
            point_in_scan2 = scan_2[square_root.argmin()]

            # Appending values to the innemost dictionary
            inner_dict = {}
            inner_dict['point_in_pc_1'] = scan_1[pc_1_index]
            inner_dict['point_in_pc_2'] = point_in_scan2
            inner_dict['dist2'] = np.sqrt((scan_1[pc_1_index][0]-point_in_scan2[0])**2 + (
                scan_1[pc_1_index][1]-point_in_scan2[1])**2 + (scan_1[pc_1_index][2]-point_in_scan2[2])**2)

            correspondences_dict[pc_1_index] = inner_dict

        return correspondences_dict


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

        """Loads a point cloud from a ply file

        :param file: source file
        :type file: str
        :return: returns true or false, depending on whether the method
        the ply was OK or not
        :rtype: bool
        """

        lines = []
        with open(file) as f:
            lines = f.readlines()

        reading_vertices = False

        for line_number, line in enumerate(lines, 0):
            if 'element vertex' in line:
                nb_vertices = int(line.split()[2])
                first_property_line = line_number+1

            if 'property float x' in line:
                x = line_number - first_property_line
            elif 'property float y' in line:
                y = line_number - first_property_line
            elif 'property float z' in line:
                z = line_number - first_property_line

            if reading_vertices:
                nb_vertices = nb_vertices-1
                if nb_vertices == 0:
                    reading_vertices = False
                properties = line.split()

                try:
                    coordinates = [float(properties[int(x)]), float(properties[int(y)]), float(properties[int(z)])]
                    self.data[line_number-first_vertice_line] = coordinates
                except:
                    return False
            
            if 'end_header' in line:
                reading_vertices = True
                first_vertice_line = line_number+1

        return True