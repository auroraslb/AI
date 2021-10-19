import numpy as np
import vtk
from vtk.util import numpy_support

from typing import NoReturn, Tuple


class point_clouds_visualization:

    def __init__(self) -> None:
        """initializes the vtk renderer, camera, render window, and render window iterator
        """

        # renderer
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.95, 0.9, 0.95)

        # camera
        self.camera = self.renderer.GetActiveCamera()
        self.set_camera_settings((0,0,0))

        # render window
        self.renderWindow = vtk.vtkRenderWindow()
        self.set_windows_name_size()
        self.renderWindow.AddRenderer(self.renderer)

        # render window iterator
        self.renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        self.renderWindowInteractor.SetRenderWindow(self.renderWindow)
        self.style = vtk.vtkInteractorStyleTerrain()
        self.renderWindowInteractor.SetInteractorStyle(self.style)

        self._point_cloud_assembly = {}

        return

    def set_camera_settings(
            self,
            camera_position: Tuple[float, float, float],
            camera_focal_point: Tuple[float, float, float] = (0.0, 0.0, 0.0),
            camera_view_up: Tuple[float, float, float] = (0.0, -1.0, 0.0)
            ) -> None:
        """Defines the camera settings for rendering

        :param camera_position: sets the camera position in the world
        :type camera_position: Tuple[float, float, float]
        :param camera_focal_point: sets the point where the camera is looking at, defaults to (0.0, 0.0, 0.0)
        :type camera_focal_point: Tuple[float, float, float], optional
        :param camera_view_up: defines the up direction, defaults to (0.0, -1.0, 0.0)
        :type camera_view_up: Tuple[float, float, float], optional
        """
        self.camera.SetPosition(camera_position[0], camera_position[1], camera_position[2])
        self.camera.SetFocalPoint(camera_focal_point[0], camera_focal_point[1], camera_focal_point[2])
        self.camera.SetViewUp(camera_view_up[0], camera_view_up[1], camera_view_up[2])
        
        return



    def set_windows_name_size(self,
            name: str = "empty",
            size: Tuple[int, int] = (600, 600)
            ) -> None:
        """Sets the name and sizes of the window

        :param name: name, defaults to "empty"
        :type name: str, optional
        :param size: size, defaults to (1280, 720)
        :type size: Tuple[int, int], optional
        """
        self.renderWindow.SetWindowName(name)
        self.renderWindow.SetSize(size[0], size[1])

        return

    def plot_camera_position_and_viewup(self) -> None:

        view_up = self.camera.GetViewUp()
        position = self.camera.GetPosition()
        orientation = self.camera.GetOrientationWXYZ()

        print('Camera Settings:')
        print('  -> position: ', position)
        print('  -> orientation: ', orientation)
        print('  -> viewup: ', view_up)

        return

    def make_text(
            self,
            text: str,
            position: Tuple[float, float, float],
            opacity: float = 0.5,
            background_color: Tuple[float, float, float] = (0.0, 0.0, 0.0),
            font_size: int = 16,
            ) -> vtk.vtkTextActor:
        """creates an object for a text in the pointcloud

        :param text: text we want to add to the point cloud
        :type text: str
        :param position: where do we want to have the text
        :type position: Tuple[float, float, float]
        :param opacity: sets the opacity level, defaults to 0.5
        :type opacity: float, optional
        :param background_color: sets the background color if necessary, defaults to (0.0, 0.0, 0.0)
        :type background_color: Tuple[float, float, float], optional
        :param font_size: defines the font size to the text, defaults to 16
        :type font_size: int, optional
        :return: returns the vtk actor with the text
        :rtype: vtk.vtkTextActor
        """
        actor = vtk.vtkTextActor()
        actor.SetInput(text)
        prop = actor.GetTextProperty()
        prop.SetBackgroundColor(
            background_color[0], background_color[1], background_color[2]
            )
        prop.SetBackgroundOpacity(opacity)
        prop.SetFontSize(font_size)
        coord = actor.GetPositionCoordinate()
        coord.SetCoordinateSystemToWorld()
        coord.SetValue(position)
        self.renderer.AddActor(actor)

        return actor

    def make_line(
            self,
            point_A: Tuple[float, float, float],
            point_B: Tuple[float, float, float],
            color: Tuple[float, float, float] = (0.9, 0.2, 0,9),
            line_width: float = 3
            ) -> vtk.vtkActor:
        """creates a line from two points

        :param point_A: first point
        :type point_A: Tuple[float, float, float]
        :param point_B: 2nd point
        :type point_B: Tuple[float, float, float]
        :param color: an optional value for the color, defaults to (0.9, 0.2, 0,9)
        :type color: Tuple[float, float, float], optional
        :param line_width: set the line width to add
        :type line_width: float, optional
        :return: returns the vtk actor
        :rtype: vtk.vtkActor
        """
        line_source = vtk.vtkLineSource()
        line_source.SetPoint1(point_A[0], point_A[1], point_A[2])
        line_source.SetPoint2(point_B[0], point_B[1], point_B[2])
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(line_source.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetLineWidth(line_width)
        actor.GetProperty().SetColor(color[0], color[1], color[2])
        self.renderer.AddActor(actor)

        return actor

    def rm_lines(
        self,
        list_actors: list
        ) -> None:

        for actor in list_actors:
            self.renderer.RemoveActor(actor)

        return

    def np_to_vtk_se3_pose(
            self,
            se3_pose: np.array((4,4))
            ) -> vtk.vtkTransform:
        """converts SE3 transformation from numpy to vtk

        :param se3_pose: transformation in the original form
        :type se3_pose: np.array
        :return: transformation, vtk type
        :rtype: vtk.vtkTransform
        """
        t = vtk.vtkTransform()
        t.SetMatrix(se3_pose.flatten())

        return t


    def __numpy_to_poly_data(
            self,
            pts: np.array((...,3))
            ) -> vtk.vtkPolyData:
        """auxiliary function to transform points from numpy to vtk type

        :param pts: original data in numpy
        :type pts: np.array
        :return: outputs the points in vtk type
        :rtype: vtk.vtkPolyData
        """
        pd = vtk.vtkPolyData()
        pd.SetPoints(vtk.vtkPoints())
        # Makes a deep copy
        pd.GetPoints().SetData(numpy_support.numpy_to_vtk(pts.copy()))

        f = vtk.vtkVertexGlyphFilter()
        f.SetInputData(pd)
        f.Update()
        pd = f.GetOutput()

        return pd


    def make_point_cloud(
            self,
            point_cloud_data: np.array((...,3)),
            point_weight: float=1.0,
            waypoint_tform_cloud: np.array((4,4)) = np.eye(4),
            point_cloud_id: str = None,
            color_depth: bool = False,
            make_axis: bool = False,
            axis_lenght: Tuple[float, float, float] = (.2, .2, .2),
            point_cloud_color: Tuple[float, float, float] = (.2, .2, .2)
            ) -> Tuple[str, vtk.vtkAssembly]:
        """creates a vtk assembly with the respective pointcloud

        :param point_cloud_data: original point cloud in matrix form
        :type point_cloud_data: np.array
        :param point_weight: sets the 
        :type point_weight: float
        :param waypoint_tform_cloud: if there is a transformation from the pointcloud to a fixed baselink, defaults to np.eye(4)
        :type waypoint_tform_cloud: np.array, optional
        :param point_cloud_id: specifies an id for the pointcloud, defaults to None
        :type point_cloud_id: str, optional
        :param color_depth: in case we want to plot information about the depth (different colors), defaults to False
        :type color_depth: bool, optional
        :param make_axis: if we want to plot an axis at the origin, defaults to False
        :type make_axis: bool, optional
        :param axis_lenght: specifies the length of the axis to plot, defaults to (.2, .2, .2)
        :type axis_lenght: Tuple[float, float, float], optional
        :param point_cloud_color: sets the color for the entire pointcloud, defaults to (.2, .2, .2)
        :type point_cloud_color: Tuple[float, float, float], optional
        :return: returns both an id for the stored pointcloud and the assembly
        :rtype: Tuple[str, vtk.vtkAssembly]
        """
        assembly = vtk.vtkAssembly()
        num_points, _ = point_cloud_data.shape

        # transforms the pointcloud type from numpy to vtk
        poly_data = self.__numpy_to_poly_data(point_cloud_data)

        # if we want to have color identifying the depth...
        if color_depth:
            arr = vtk.vtkFloatArray()
            for i in range(num_points):
                arr.InsertNextValue(point_cloud_data[i, 2])
            arr.SetName("z_coord")
            poly_data.GetPointData().AddArray(arr)
            poly_data.GetPointData().SetActiveScalars("z_coord")

        # creates the actor
        point_cloud_actor = vtk.vtkActor()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly_data)
        mapper.ScalarVisibilityOn()
        point_cloud_actor.SetMapper(mapper)
        point_cloud_actor.GetProperty().SetPointSize(point_weight)
        point_cloud_actor.GetProperty().SetColor(
            point_cloud_color[0],
            point_cloud_color[1],
            point_cloud_color[2]
            )
        point_cloud_actor.SetUserTransform(self.np_to_vtk_se3_pose(waypoint_tform_cloud))


        # if we want to show the axis
        # creates an actor for that
        if make_axis:
            actor = vtk.vtkAxesActor()
            actor.SetTotalLength(
                axis_lenght[0],
                axis_lenght[1],
                axis_lenght[2]
                )

        # assembly all actors involved
        assembly.AddPart(point_cloud_actor)
        if make_axis:
            assembly.AddPart(actor)

        # add the assembly to the renderer
        self.renderer.AddActor(assembly)

        # creates and id (if not set) for the pointcloud and save it
        if point_cloud_id == None:
            point_cloud_id = 'id_' + str(len(self._point_cloud_assembly))
        self._point_cloud_assembly[point_cloud_id] = assembly

        return point_cloud_id, assembly

    def transform_pointcloud(self,
            point_cloud_id: str,
            tform: np.array((4,4)),
            ) -> None:
        
        self._point_cloud_assembly[point_cloud_id].SetUserTransform(self.np_to_vtk_se3_pose(tform))

        return

    def render(
            self,
            block: bool = True
            ) -> None:
        """render the image

        :param block: choose to either block or leave the code running, defaults to True
        :type block: bool, optional
        """
        self.renderer.ResetCamera()
        self.renderWindow.Render()
        self.renderWindow.Start()
        if block:
            self.renderWindowInteractor.Start()

        return
