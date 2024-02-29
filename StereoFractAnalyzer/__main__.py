import argparse
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import copy

class StereoFractAnalyzer:
    def __init__(self, stl_path = None, img_path=None):
        self.stl_path = stl_path
        if self.stl_path is not None: 
            try:
                self.STL = o3d.io.read_triangle_mesh(stl_path)
            except Exception as e:
                self.STL = None
                print(f"Failed to load STL file: {e}")
        else:
            self.STL=None

        self.img_path=img_path
        self.slope_min_point= -0.75
        self.slope_max_point= 0

    def transform(self, translation=(0,0,0), rotation=(0*(np.pi/180),0*(np.pi/180),0*(np.pi/180)), scale=1.0):

        """
        Apply geometric transformations to the 3D mesh, including translation, rotation, and scaling.

        This method modifies the 3D mesh associated with the StereoFractAnalyzer instance by applying a series of 
        geometric transformations. It supports translating the mesh in 3D space, rotating it around its center, and 
        uniformly scaling its size. 

        Parameters:
            translation (tuple of float, optional): Specifies the translation vector as (x, y, z), dictating how much 
                the mesh should be moved along each axis. The default is (0, 0, 0), indicating no translation.
            rotation (tuple of float, optional): Specifies the rotation angles in radians for each of the x, y, and z 
                axes, respectively. The rotation is applied in the order of x, y, z axes. Defaults to (0, 0, 0), 
                indicating no rotation.
            scale (float, optional): Specifies the scaling factor to be applied to the mesh. A value of 1.0 indicates 
                no scaling, values greater than 1.0 enlarge the mesh, and values less than 1.0 shrink it. The default 
                is 1.0.

        Returns:
            open3d.geometry.TriangleMesh: A new TriangleMesh object representing the transformed mesh.

        Example:
            >>> analyzer = StereoFractAnalyzer(stl_path="path/to/mesh.stl")
            >>> transformed_mesh = analyzer.transform(translation=(10, 5, 0), rotation=(0, 0, np.pi/4), scale=2.0)
            >>> analyzer.view()  # View the transformed mesh

        Note:
            The rotation angles are specified in radians as required by the underlying Open3D methods.
            The original mesh gets modified by the transformations, and the method returns a new TriangleMesh object. 
        """
        if self.STL.is_empty():
            print("No STL file loaded.")
            return
        
        if not isinstance(scale, (int, float)) or scale <= 0:
            print("Scale must be a positive number.")
            return None

        mesh_copy = copy.deepcopy(self.STL)
        mesh_copy.translate(translation)
        mesh_copy.rotate(mesh_copy.get_rotation_matrix_from_xyz(rotation))
        mesh_copy.scale(scale, center=mesh_copy.get_center())
        self.STL=mesh_copy
        return mesh_copy
    
    def get_voxel(self,size=1):
        """
        Converts the 3D mesh associated with this StereoFractAnalyzer instance into a voxel grid representation.

        This method creates a voxel grid from the STL mesh, enabling volumetric analysis of the structure. Each voxel 
        in the grid represents a cube of space within the mesh, with the voxel size determining the resolution of the 
        grid. A smaller voxel size results in a higher resolution grid, allowing for more detailed analysis of the mesh 
        geometry.

        Parameters:
            size (int or float, optional): The edge length of each cubic voxel in the grid. Defines the resolution 
                of the voxel grid, with smaller values leading to finer resolution. Default is 1, representing a 
                voxel edge length of 1 unit in the mesh's coordinate system.

        Returns:
            numpy.ndarray: An array of voxel indices, each representing the (x, y, z) grid index of a voxel multiplied 
                by the specified size. This array provides a spatial representation of where each voxel is located in 
                the grid relative to the mesh.

        Example:
            >>> analyzer = StereoFractAnalyzer(stl_path="path/to/mesh.stl")
            >>> voxel_grid = analyzer.get_voxel(size=0.5)
            >>> print(voxel_grid)
            # Outputs the grid indices of the voxels, scaled according to the specified voxel size -> coordinates of the voxels

        Note:
            The mesh must be loaded (by providing an STL path at initialization) before calling this method. If no mesh 
            is loaded, the method will return an empty array. It is recommended to check the successful loading of the 
            mesh before proceeding with voxelization.
        """
        if self.STL.is_empty():
            print("No STL file loaded.")
            return

        voxel_grid=o3d.geometry.VoxelGrid.create_from_triangle_mesh(self.STL,voxel_size=size)
        voxels = voxel_grid.get_voxels()  
        indices = np.stack(list(vx.grid_index for vx in voxels))
        return indices*size
        
    def view(self,point_cloud_data=0,):
        """
            Visualizes the 3D mesh or a point cloud representation of the mesh associated with this StereoFractAnalyzer instance.

            This method provides a convenient way to visually inspect the 3D mesh loaded into the StereoFractAnalyzer. By default,
            it displays the original mesh. However, it can also generate and display a point cloud representation of the mesh 
            based on Poisson disk sampling, which might be useful for analyzing the surface structure or for visual inspections 
            where rendering the full mesh is computationally expensive.

            Parameters:
                point_cloud_data (int, optional): Determines the visualization mode. If set to 0 (default), the original 3D mesh 
                    is visualized. If set to a positive integer, it represents the number of points to sample for creating a point 
                    cloud representation of the mesh for visualization.

            Example:
                >>> analyzer = StereoFractAnalyzer(stl_path="path/to/mesh.stl")
                >>> analyzer.view()  # Visualizes the original 3D mesh
                >>> analyzer.view(point_cloud_data=1000)  # Visualizes a point cloud sampled from the mesh

            Note:
                The mesh must be loaded (by providing an STL path at initialization) before calling this method. Visualization 
                relies on the Open3D visualization toolkit, which opens a new window to display the mesh or point cloud. Ensure 
                that your environment supports GUI operations, as this might not work in non-GUI (headless) environments.
            """
        if self.STL.is_empty():
            print("No STL file loaded.")
            return

        if point_cloud_data:
            view = self.STL.sample_points_poisson_disk(number_of_points=point_cloud_data)
        else:
            view=self.STL
        o3d.visualization.draw_geometries([view,o3d.geometry.TriangleMesh.create_coordinate_frame()], mesh_show_wireframe=True, width=800, height=600)

    def calculate_intersection_point(self, p1, p2, z_height):
        """
        Calculates the intersection point between a line segment and a horizontal plane at a given z-height.

        This method determines the point at which a line segment, defined by two points p1 and p2, intersects with a 
        horizontal plane located at a specific z-height. This is particularly useful for slicing operations or for 
        analyzing the spatial relationship between geometric features and defined planes within the 3D space of the mesh.

        Parameters:
            p1 (tuple or list): The (x, y, z) coordinates of the first point defining the line segment.
            p2 (tuple or list): The (x, y, z) coordinates of the second point defining the line segment.
            z_height (float): The z-coordinate of the horizontal plane with which the intersection is calculated.

        Returns:
            numpy.ndarray: The (x, y, z) coordinates of the intersection point between the line segment and the horizontal plane.
            If the line segment does not intersect with the plane (e.g., it is parallel to the plane), the return value will be None.

        Example:
            >>> analyzer = StereoFractAnalyzer()
            >>> p1 = (1, 2, 3)
            >>> p2 = (1, 2, 5)
            >>> z_height = 4
            >>> intersection_point = analyzer.calculate_intersection_point(p1, p2, z_height)
            >>> print(intersection_point)
            # Outputs: [1. 2. 4.]

        Note:
            This method does not require a mesh to be loaded as it operates purely on the geometric arguments provided. 
            It is a utility function that can be used independently of the mesh analysis features of the StereoFractAnalyzer.
        """
        try:
            t = (z_height - p1[2]) / (p2[2] - p1[2])
        except ZeroDivisionError:
            return None
        x = p1[0] + t * (p2[0] - p1[0])
        y = p1[1] + t * (p2[1] - p1[1])
        return np.array([x, y, z_height])

    def calculate_slice(self, MESH, z_height):
        """
        Calculates a horizontal slice of the 3D mesh at a specified z-height.

        This method computes the intersection lines between the mesh and a horizontal plane at the given z-height. 
        It is useful for analyzing cross-sections of the mesh, which can be crucial for understanding the internal 
        structure or for preparing data for further analysis such as calculating the fractal dimension of the slice.

        Parameters:
            MESH (open3d.geometry.TriangleMesh): The 3D mesh from which the slice is to be calculated. This mesh should 
                be a valid Open3D TriangleMesh object.
            z_height (float): The z-coordinate of the horizontal plane used to slice the mesh. The method calculates the 
                intersections of this plane with the mesh triangles to generate the slice.

        Returns:
            numpy.ndarray: An array of line segments representing the intersections of the mesh with the horizontal plane. 
                Each line segment is represented by a pair of points (x1, y1, z_height) and (x2, y2, z_height), where each 
                point is an array of its x, y, and z coordinates.

        Example:
            >>> analyzer = StereoFractAnalyzer(stl_path="path/to/mesh.stl")
            >>> z_height = 5
            >>> slice_lines = analyzer.calculate_slice(analyzer.STL, z_height)
            >>> print(slice_lines)
            # Outputs the array of line segments that make up the slice at the specified z-height.

        Note:
            The mesh (MESH parameter) must be a loaded and valid Open3D TriangleMesh object. This method does not alter the 
            original mesh but instead analyzes it to return the calculated slice. Ensure that the mesh has been properly 
            loaded and prepared before calling this method.
        """
        if self.STL.is_empty():
            print("No STL file loaded.")
            return

        triangles = np.asarray(MESH.triangles)
        v_0 = np.asarray(MESH.vertices)[triangles[:, 0]]
        v_1 = np.asarray(MESH.vertices)[triangles[:, 1]]
        v_2 = np.asarray(MESH.vertices)[triangles[:, 2]]
        slice_lines = []
        for v1, v2, v3 in zip(v_0, v_1, v_2):
            min_z = min(v1[2], v2[2], v3[2])
            max_z = max(v1[2], v2[2], v3[2])
            if z_height < min_z or z_height > max_z:
                continue
            points = [v1, v2, v3, v1]
            intersection_points = []
            for i in range(3):
                if (points[i][2] - z_height) * (points[i+1][2] - z_height) < 0:
                    intersection_points.append(self.calculate_intersection_point(points[i], points[i+1], z_height))
            if len(intersection_points) == 2:
                slice_lines.append(intersection_points)
        return np.array(slice_lines)

    def box_counting(self, points, min_box_size, max_box_size, steps, plot=0):
        """
        Calculates the fractal dimension of a set of points using the box counting method.

        This method implements the box counting algorithm, a widely used technique in fractal geometry to estimate 
        the fractal dimension of a dataset. It works by covering the dataset with boxes of various sizes and counting 
        the number of boxes that contain at least one point from the dataset. The fractal dimension is estimated by 
        analyzing how this number changes as the box size scales.

        Parameters:
            points (numpy.ndarray): An array of points representing the dataset. Each row should represent a point 
                in the format (x, y, z).
            min_box_size (float): The size of the smallest box to use in the analysis.
            max_box_size (float): The size of the largest box to use in the analysis.
            steps (int): The number of steps between the smallest and largest box sizes, determining the granularity 
                of the analysis.
            plot (int, optional): If set to 1, a log-log plot of the box size versus the number of boxes is generated 
                to visually inspect the scaling behavior. Default is 0, which means no plot is generated.

        Returns:
            tuple: A tuple containing:
                - D (float): The estimated fractal dimension of the dataset.
                - sizes (numpy.ndarray): An array of the box sizes used in the analysis.
                - counts (numpy.ndarray): An array of the counts of boxes containing at least one point for each box size.

        Example:
            >>> analyzer = StereoFractAnalyzer()
            >>> points = np.random.rand(1000, 3)  # Generate some random points
            >>> D, sizes, counts = analyzer.box_counting(points, 0.01, 1.0, 10, plot=1)
            >>> print(f"Estimated fractal dimension: {D}")

        Note:
            The box counting method is sensitive to the choice of min_box_size, max_box_size, and steps. These parameters 
            should be chosen carefully based on the scale and density of the dataset to ensure accurate estimation of 
            the fractal dimension. The plotting feature requires matplotlib to be installed and is useful for verifying 
            the linear relationship on a log-log scale, a characteristic of fractal structures.

            The accuracy of the fractal dimension estimation relies on the appropriate selection of `slope_min_point` 
            and `slope_max_point`, which define the linear region of interest in the log-log plot. These parameters 
            should be chosen based on the dataset's characteristics and the analysis's preliminary results. Fine-tuning 
            these values is crucial for minimizing error in the slope calculation and, consequently, the fractal dimension 
            estimation. The method assumes a linear relationship in the chosen region of the plot, which is a hallmark of 
            fractal structures. The default values are set to -0.75 and 0. It can be done by setting the `slope_min_point`
            and `slope_max_point` attributes of the class. See the example below. 

            Example:
                >>> analyzer = StereoFractAnalyzer()
                >>> points = np.random.rand(1000, 3)  # Generate some random points
                >>> analyzer.slope_min_point = -1.0
                >>> analyzer.slope_max_point =  0.0
                >>> D, sizes, counts = analyzer.box_counting(points, 0.01, 1.0, 10, plot=1)
                >>> print(f"Estimated fractal dimension: {D}")

        """ 
        slope_min_point= self.slope_min_point
        slope_max_point= self.slope_max_point

        sizes = np.logspace(np.log10(min_box_size), np.log10(max_box_size), num=steps)
        counts = np.zeros(steps)
        min_point = points.min(axis=0)
        normalized_points = points - min_point
        max_dim = normalized_points.max(axis=0)
        for i, size in enumerate(sizes):
            num_boxes = np.ceil(max_dim / size).astype(int)
            box_indices = (normalized_points / size).astype(int)
            unique_boxes = np.unique(box_indices, axis=0)
            counts[i] = unique_boxes.shape[0]
        log_sizes = np.log10(1/sizes)
        log_counts = np.log10(counts)
        valid_indices = (log_sizes <= slope_max_point) & (log_sizes >= slope_min_point)
        valid_log_sizes = log_sizes[valid_indices]
        valid_log_counts = log_counts[valid_indices]
        if len(valid_log_sizes) > 1:
            D, intercept = np.polyfit(valid_log_sizes, valid_log_counts, 1)
            if plot:
                plt.figure(figsize=(8, 6))
                plt.plot(log_sizes, log_counts, 'o-', label='Log-Log Plot')
                plt.plot(valid_log_sizes, D*valid_log_sizes + intercept, 'r--', label=f'Linear fit with slope = {D:.2f}')
                plt.xlabel('Log(1/Box Size)')
                plt.ylabel('Log(Count)')
                plt.title('Box-Counting Method for Fractal Dimension')
                plt.legend()
                plt.grid(True)
                plt.show()
        else:
            D = None
        return D, sizes, counts

    def box_count_image(self, img, min_size=1, max_size=None, n_sizes=10):
        """
        Calculates the box counts for a binary image (not a strict requirement, the fractal object should be darker) 
        using the box counting method to estimate its fractal dimension. This method applies the box counting algorithm 
        to a binary image by covering the image with non-overlapping boxes of varying sizes. It counts the number of boxes
        containing at least one pixel from the foreground of the image. The fractal dimension is estimated based on how this
        count changes as the size of the boxes is varied, providing a measure of the complexity of the image's structure.

        Parameters:
            img (PIL.Image.Image): The binary image to be analyzed. The image should be in a format compatible with 
                PIL (Python Imaging Library). Ensure that the fractal object is darker than the background.
            min_size (int, optional): The minimum box size to use in the analysis. Defaults to 1 pixel.
            max_size (int, optional): The maximum box size to use in the analysis. If not specified, it defaults to 
                half of the smallest image dimension.
            n_sizes (int, optional): The number of different box sizes to use, distributed logarithmically between 
                min_size and max_size. Defaults to 10.

        Returns:
            tuple: Contains two numpy arrays:
                - sizes (numpy.ndarray): An array of the box sizes used in the analysis.
                - counts (numpy.ndarray): An array of the counts of boxes containing at least one foreground pixel 
                for each box size.

        Example:
            >>> from PIL import Image
            >>> analyzer = StereoFractAnalyzer()
            >>> img = Image.open("path/to/binary_image.png")
            >>> sizes, counts = analyzer.box_count_image(img, min_size=1, max_size=100, n_sizes=10)
            >>> print(sizes, counts)

        Note:
            The binary image is analyzed as is, without any preprocessing steps such as thresholding or conversion 
            to grayscale. Ensure the image is properly formatted as a binary image (foreground and background) before 
            analysis. The method assumes the foreground (the part of the image to be analyzed) is represented by 
            darker pixels.
        """
        img_array = np.asarray(img.convert('L')) < 128    # fractals image should be darker than the background
        if max_size is None:
            max_size = min(img.size) // 2
        sizes = np.ceil(np.logspace(np.log10(max_size), np.log10(min_size), num=n_sizes)).astype(int)
        counts = []
        for size in sizes:
            num_boxes = 0
            for x in range(0, img_array.shape[1], size):
                for y in range(0, img_array.shape[0], size):
                    if np.any(img_array[y:y + size, x:x + size]):
                        num_boxes += 1
            counts.append(num_boxes)
        return sizes, counts

    def plot_log_log_image(self, log_sizes, log_counts):
        """
        Plots the log-log graph of box sizes versus box counts for a binary image and estimates its fractal dimension.

        This method generates a log-log plot to visualize the scaling relationship between the sizes of boxes 
        (expressed as the logarithm of their inverse sizes) used in the box counting method and the corresponding 
        logarithmic counts of boxes that contain at least one pixel of the image. The slope of the linear fit to 
        this plot is used to estimate the fractal dimension of the image, providing a quantitative measure of its 
        complexity.

        Parameters:
            log_sizes (numpy.ndarray): An array of the logarithms of the inverse of the box sizes used in the 
                box counting analysis.
            log_counts (numpy.ndarray): An array of the logarithms of the counts of boxes containing at least 
                one pixel for each box size.

        Returns:
            float: The estimated fractal dimension of the image based on the slope of the linear fit to the points 
            in the log-log plot.

        Example:
            >>> analyzer = StereoFractAnalyzer()
            >>> img = Image.open("path/to/binary_image.png")
            >>> log_sizes, log_counts = analyzer.prepare_log_log_data(img)
            >>> fractal_dimension = analyzer.plot_log_log_image(log_sizes, log_counts)
            >>> print(f"Estimated fractal dimension: {fractal_dimension:.2f}")

        Note:
            The accuracy of the fractal dimension estimation depends on the quality of the log-log plot and the 
            appropriateness of the linear fit to the data points. A sufficient range of box sizes and a logarithmic 
            distribution of these sizes are crucial for accurately capturing the scaling behavior indicative of 
            fractal structures. This method assumes a linear relationship over the significant range of scales 
            represented in the plot, a hallmark of fractal geometries.
        """
        coeffs = np.polyfit(log_sizes, log_counts, 1)

        plt.figure()
        plt.scatter(log_sizes, log_counts, label='Log-Log of Box Count')
        plt.plot(log_sizes, coeffs[0]*log_sizes + coeffs[1], color='r', label=f'Fit line with dimension {coeffs[0]:.2f}')
        plt.xlabel('log(1/Box Size)')
        plt.ylabel('log(Count)')
        plt.legend()
        plt.title('Log-Log plot of Box Counting Method')
        plt.grid(True)
        plt.show()
        return coeffs[0]


    def get_image_fractal_dimension(self,img_path=0,plot=1):
        """
        Calculates and optionally plots the fractal dimension of a 2D image using the box counting method.

        This method estimates the fractal dimension of a binary image to quantify its complexity or roughness. The image is
        first converted to a binary format, if not already in one. The box counting method is then applied to calculate box
        sizes and counts, from which the fractal dimension is estimated. The log-log graph of these values can be plotted to
        visually assess the scaling behavior. The image to analyze can be specified via method argument or at class initialization.

        Parameters:
            img_path (str or int, optional): Path to the binary image file. If an image path was provided at class initialization,
                this parameter can be omitted or set to 0 to use that image. Otherwise, specify the path here. Defaults to 0.
            plot (int, optional): If set to 1, the method generates a log-log plot of the box counting analysis, aiding in the
                visualization of the fractal dimension estimation. Defaults to 1.

        Returns:
            float: The estimated fractal dimension of the image. If no valid image path is provided, or the image cannot be
            processed, the method returns None.

        Example:
            >>> analyzer = StereoFractAnalyzer(img_path="path/to/binary_image.png")
            >>> fractal_dimension = analyzer.get_image_fractal_dimension()
            >>> print("The fractal dimension is:", fractal_dimension)
            # If the image path was not provided at initialization:
            >>> fractal_dimension = analyzer.get_image_fractal_dimension("path/to/another_image.png")
            >>> print("The fractal dimension is:", fractal_dimension)

        Note:
            The method assumes the image is binary, where the fractal structure is darker than the background. Accurate fractal
            dimension estimation requires proper image preprocessing to fit this criterion. The optional plotting feature is
            beneficial for verifying the linear relationship on a log-log scale, characteristic of fractal structures.
        """
        if self.img_path is not None:
            img_path=self.img_path
        if img_path != 0:
            try:
                img = Image.open(img_path)
                sizes, counts = self.box_count_image(img)
                log_sizes = np.log(1/sizes)
                log_counts = np.log(counts)
                coeffs = np.polyfit(log_sizes, log_counts, 1)
                fractal_dimension = coeffs[0]
                if plot:
                    self.plot_log_log_image(log_sizes, log_counts)
                print("The fractal dimension is:", fractal_dimension)
                return fractal_dimension
            except Exception as e:
                return print(f"Failed to process the image: {e}")
        else:
            return print("No valid image path is provided!!")
            

    def draw_contours(self, zmin=10, zmax=10, step=1, save=0):
        """
        Draws the contours of the 3D mesh at different z-heights and optionally saves the contour plots.

        This method slices the mesh at a series of z-heights specified by the user, drawing the contours at each 
        slice. This can be particularly useful for visualizing the shape and features of the mesh across different 
        layers. The method can also save these contour plots as image files if desired.

        Parameters:
            zmin (int or float, optional): The minimum z-height at which to start slicing the mesh. Defaults to 10.
            zmax (int or float, optional): The maximum z-height at which to end slicing the mesh. Must be greater 
                than or equal to zmin. Defaults to 10.
            step (int or float, optional): The interval between successive z-heights. A smaller step results in more 
                slices and thus more detailed contour plots. Defaults to 1.
            save (int, optional): If set to 1, saves each contour plot as an image file in the current directory. 
                The files are named sequentially as 'slice_<z-height>.png'. Defaults to 0, which means plots are 
                not saved.

        Example:
            >>> analyzer = StereoFractAnalyzer(stl_path="path/to/mesh.stl")
            >>> analyzer.draw_contours(zmin=0, zmax=20, step=2, save=1)
            # This will draw and save contour plots for the mesh sliced at z-heights 0, 2, 4, ..., 20.

        Note:
            The mesh must be loaded into the StereoFractAnalyzer instance before calling this method. Ensure that 
            zmax is greater than or equal to zmin and that the step value is appropriate for the desired resolution 
            of analysis. If saving the plots, check the current directory for the output image files. The visualization 
            uses matplotlib for plotting, so ensure it is correctly installed and configured in your environment.
        """
        if self.STL.is_empty():
            print("No STL file loaded.")
            return

        for i in np.linspace(zmin, zmax, step):
            z_height = i
            slice_lines = self.calculate_slice(self.STL, z_height)
            plt.figure(figsize=(10, 6))
            for line in slice_lines:
                plt.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], 'k-')
            plt.axis('equal')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title(f'Slice at Z={z_height}')
            plt.show()
            if save:
                plt.savefig('slice_'+str(i)+'.png')

    def get_contour_fractal_dimension(self, z_height, plot=0):
        """
        Calculates the fractal dimension of a contour slice at a specified z-height within the 3D mesh.

        This method extracts a horizontal slice of the mesh at the given z-height and calculates the fractal 
        dimension of this slice using the box counting method. The fractal dimension provides a quantitative 
        measure of the complexity or roughness of the contour line at the specified slice of the mesh. Optionally, 
        this method can also generate a log-log plot of the box sizes versus the box counts used in the calculation 
        to visually assess the scaling behavior.

        Parameters:
            z_height (float): The z-coordinate of the horizontal plane used to slice the mesh and calculate the 
                fractal dimension of the resulting contour.
            plot (int, optional): If set to 1, a log-log plot of the box size versus the number of boxes is generated 
                to visually inspect the scaling behavior. Defaults to 0, which means no plot is generated.

        Returns:
            float: The estimated fractal dimension of the contour at the specified z-height. If the contour cannot be 
            calculated or does not intersect with the mesh at the specified z-height, the method may return None.

        Example:
            >>> analyzer = StereoFractAnalyzer(stl_path="path/to/mesh.stl")
            >>> z_height = 5.0
            >>> fractal_dimension = analyzer.get_contour_fractal_dimension(z_height, plot=1)
            >>> print(f"Fractal dimension of the contour at z={z_height}: {fractal_dimension}")

        Note:
            The accuracy of the fractal dimension estimation depends on the quality of the contour extraction and 
            the appropriateness of the box sizes used in the box counting method. It's important to choose a z-height 
            where the mesh intersects with the horizontal plane to ensure meaningful results. The optional log-log 
            plot can provide valuable insights into the appropriateness of the box counting analysis and the presence 
            of fractal characteristics in the contour line.
        """

        if self.STL.is_empty():
            print("No STL file loaded.")
            return
        print('Calculating Fractal Dimension of the contour at z=',z_height)
        slice_lines = self.calculate_slice(self.STL, z_height)
        slice_points = np.array([point for line in slice_lines for point in line])
        D, sizes, counts = self.box_counting(slice_points, 0.1, 10, 100, plot)
        return D

    def get_surface_fractal_dimension(self, increment=1, plot=0):
        """
        Calculates the fractal dimension of the surface of the 3D mesh based on the methodology described in 
        "Characterizing the Complexity of Computer-Generated Fractal Curves" (https://apps.dtic.mil/sti/tr/pdf/ADA129664.pdf).

        This method estimates the fractal dimension (D) of the 3D mesh surface by analyzing the complexity of its contour 
        slices at different z-heights. It iterates through the mesh, slicing it at intervals specified by the 'increment' 
        parameter, and calculates the fractal dimension of each contour slice using the box counting method. The overall 
        surface fractal dimension is then determined by averaging the fractal dimensions of these slices, providing a 
        measure of the surface complexity or roughness.

        Parameters:
            increment (int or float, optional): The interval between successive z-heights for slicing the mesh. A smaller 
                increment results in more slices and potentially more accurate estimation of the surface fractal dimension. 
                Defaults to 1.
            plot (int, optional): If set to 1, generates a plot for each contour's fractal dimension calculation to visually 
                inspect the scaling behavior. Defaults to 0, which means no plots are generated.

        Returns:
            float: The estimated fractal dimension of the mesh surface. Provides a quantitative measure of the surface's 
            complexity or roughness.

        Example:
            >>> analyzer = StereoFractAnalyzer(stl_path="path/to/mesh.stl")
            >>> surface_fractal_dimension = analyzer.get_surface_fractal_dimension(increment=0.5, plot=1)
            >>> print(f"Surface fractal dimension: {surface_fractal_dimension}")

        Note:
            The methodology for calculating the surface fractal dimension is adapted from the approach outlined in the 
            referenced paper, which discusses the characterization of fractal curves. This method extends the concept to 
            3D mesh surfaces by analyzing the fractal dimensions of contour slices. The accuracy of the estimation may 
            depend on the mesh's characteristics and the chosen increment for slicing. The optional plotting feature 
            requires matplotlib and is useful for verifying the linear relationship on a log-log scale for each contour 
            slice analyzed.
        """
        if self.STL.is_empty():
            print("No STL file loaded.")
            return 
        
        z_max = int(self.STL.get_max_bound()[2])
        z_min = int(self.STL.get_min_bound()[2])
        DF = []
        Z = []
        i = z_min + 1
        while i <= z_max:
            df = self.get_contour_fractal_dimension(i, plot)
            if df is not None:
                DF.append(df)
                Z.append(i)
                i = i + increment
        DFS = np.mean(DF) + 1
        print('Fractal Dimension of the surface =', DFS)
        return DFS

    def calculate_fractal_dimension(self):
        """
        A comprehensive test method to verify the basic functionalities of the StereoFractAnalyzer class related 
        to fractal dimension calculation.

        This method is designed for testing purposes to ensure that the core functionalities of the StereoFractAnalyzer, 
        including loading meshes and images, transforming geometries, slicing, and calculating fractal dimensions, are 
        working as expected. It sequentially executes a series of operations:
        1. Calculates the surface fractal dimension of a 3D mesh (if a mesh is loaded).
        2. Draws contours at a specified z-height and analyzes their fractal dimension.
        3. Calculates the fractal dimension of a loaded 2D image (if an image path is provided).
        This method serves as a comprehensive check to validate the analytical capabilities of the StereoFractAnalyzer.

        Returns:
            float: The fractal dimension of the surface of the 3D mesh or the 2D image. If both are provided, 
            the method returns the mesh's surface fractal dimension as it prioritizes 3D analysis. If neither a mesh 
            nor an image is loaded, or if the calculations cannot be completed, the method returns None.

        Example:
            >>> analyzer = StereoFractAnalyzer(stl_path="path/to/mesh.stl", img_path="path/to/image.png")
            >>> fractal_dimension = analyzer.calculate_fractal_dimension()
            >>> print("Calculated fractal dimension:", fractal_dimension)

        Note:
            This method is intended for testing and demonstration purposes, showcasing the integration and functionality 
            of the StereoFractAnalyzer's capabilities. It assumes that the STL file and/or the image file have been 
            properly loaded into the class instance. 
        """
        if self.STL.is_empty():
            print("No STL file loaded.")
            return

        DFS = self.get_surface_fractal_dimension(1, 0)
        self.draw_contours(30, 30, 1, 0)
        self.get_image_fractal_dimension()
        return DFS
    

def main():
    parser = argparse.ArgumentParser(description='Calculate the fractal dimension of a 3D surface or a 2D image.')
    parser.add_argument('--stl', type=str, help='Path to the STL file')
    parser.add_argument('--img', type=str, help='Path to the image file')
    args = parser.parse_args()
    if args.stl is None and args.img is None:
        print('Please provide a valid path to the STL file or the image file. -h or --help for help.')
        exit()

    elif args.stl is not None and args.img is None:
        fractal = StereoFractAnalyzer(args.stl)

        try:
            fractal.calculate_fractal_dimension()
        except Exception as e:
            print(f"An error occurred during fractal dimension calculation: {e}")

    elif args.img is not None and args.stl is None:
        fractal = StereoFractAnalyzer(img_path=args.img)
        fractal.get_image_fractal_dimension()
    else:
        fractal = StereoFractAnalyzer(args.stl, args.img)
        try:
            fractal.calculate_fractal_dimension()
        except Exception as e:
            print(f"An error occurred during fractal dimension calculation: {e}")



if __name__ == "__main__":
    main()



