from solution import registration_iasd, point_cloud_data_iasd
from visualization_vtk import point_clouds_visualization

from sys import argv

def run(name_test: str, number_iterations: int) -> None:

    # loads the point clouds
    dataset_path = name_test
    point_cloud_1 = point_cloud_data_iasd(fileName=dataset_path+'_1.ply')
    point_cloud_2 = point_cloud_data_iasd(fileName=dataset_path+'_2.ply')

    # transform the pointclouds from dictionary style to numpy
    # (for visualization and registration purposes)
    np_point_cloud_1 = point_cloud_1.get_point_cloud(numpy_array=True)
    np_point_cloud_2 = point_cloud_2.get_point_cloud(numpy_array=True)

    # gets the center of the points, 2nd point cloud
    # for visualization purposes
    avg_x, avg_y, avg_z = point_cloud_2.gets_point_cloud_center()

    # creates a renderer and set the default settings6
    figure = point_clouds_visualization()

    # add the camera position and orientation
    # to the environment
    figure.set_camera_settings(
        camera_position= (avg_x, avg_y+1, avg_z+.35),
        camera_focal_point= (-avg_x, -avg_y, -avg_z)
        )

    # defines the name of the window and size if need (not included)
    figure.set_windows_name_size(name='IASD21/22: Point Cloud Registration')

    # include the pointclouds to the environment
    # keeps the id of the first, which should rotate and translate to the pc2
    # updates on the position and orientation of the vtk object is done in the
    # registration
    id_pc1, _ = figure.make_point_cloud(np_point_cloud_1, point_weight=3, point_cloud_color=(0.9, 0.1, 0.1))
    _, _ = figure.make_point_cloud(np_point_cloud_2, point_weight=3, point_cloud_color=(0.1, 0.1, 0.9))

    # creates the registration object
    reg = registration_iasd(np_point_cloud_1, np_point_cloud_2)
    # computes the registration
    r, t = reg.compute(
        show_visualization=True,
        verbose=True,
        vtk_visualization=figure,
        vtk_pc1=id_pc1,
        max_iter=number_iterations
        )

    return
    
def main(arguments):

    # Problems are a dictionary indicating
    # with values being a string with the problem.
    # The load_solution function will load the data and
    # gt from the respective files.
    PROBLEMS ={
        'PUB1': ('test_nr1', 5),
        'PUB2': ('test_nr2', 5),
        'PUB3': ('test_nr3', 15),
        'PUB4': ('test_nr4', 10),
        'PUB5': ('test_nr5', 10),
        'PUB6': ('test_nr6', 25),
        'PUB7': ('test_nr7', 20),
        'PUB8': ('test_nr8', 20)
    }

    if len(arguments) < 1:
        test = 'PUB7'
    else:
        test = arguments[0]

    run(PROBLEMS[test][0], PROBLEMS[test][1])

if __name__=="__main__":
    main(argv[1:])
