from numpy.core.numeric import Inf
from numpy.core.records import array
from registration import registration
from get_pointcloud import point_cloud_data

import numpy as np
from typing import Tuple
from math import nan, sqrt


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
        
        # Dictionary of correspondance
        correspondance = {}

       # sorted_scan2_index = np.argsort(self.scan_2)
        #sorted_scan2 = np.array(self.scan_2[sorted_scan2_index])

        #scan_2 = np.asarray(self.scan_2)
        
        for point_1 in self.scan_1: #np.array, dim (N,3)
            # for every point in cloud 1
            idx = np.argmin(np.sum((self.scan_2-point_1)**2))
            
        
            #idx = np.searchsorted(sorted_scan2, point_1, side="left")
            
            """
            if idx > 0 and (idx == len(sorted_scan2) or np.abs(point_1 - sorted_scan2[idx-1]) < np.abs(point_1 - sorted_scan2[idx])):
                point_2 = sorted_scan2[idx-1]
            else:
                point_2 = sorted_scan2[idx]
            """
            """
            if idx >= len(self.scan_2):
                idx_point_2 = sorted_scan2_index[len(self.scan_2) - 1]
            elif idx == 0:
                idx_point_2 = sorted_scan2_index[0]
            else:
                if abs(point_1 - sorted_scan2[idx-1]) < abs(point_1 - sorted_scan2[idx]):
                    idx_point_2 = sorted_scan2_index[idx-1]
                else:
                    idx_point_2 = sorted_scan2_index[idx]
            
            point_2 = self.scan_2[idx_point_2]
            
            key = int( (str(point_1) + str(point_2)))
            closest_dist = np.sqrt(np.sum((point_1-point_2)**2))
            """

            correspondance[key] = { 'point_in_pc_1': point_1, 
                                    'point_in_pc_2': point_2,
                                    'dist2': sqrt(closest_dist)
                                  }
        
        #print('Corr_dict:', correspondance)
        return correspondance


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
