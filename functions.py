"""
Contains relevant functions for completing tasks or visualizations.

----

- crop_laz:

  Necessary function for crop the LAZ/LAS file to a certain bounding box.

- plot_pointcloud:

  Used for the 3D visualization of point cloud.

- plot_grid:

  Used for the visualization of grid.
"""

import laspy
import rasterio
from matplotlib import pyplot as plt
from rasterio.transform import Affine


def crop_laz(file_path, center_x, center_y, half_size):
    """
    Crop a large LAZ or LAS file into a smaller range. The new
    bounding box is defined by a center point and its half-length.

    ----

    Parameters:

    - file_path: The path of the LAZ or LAS file.
    - center_x: The x-coordinate of the center point of the bounding box.
    - center_y: The y-coordinate of the center point of the bounding box.
    - half_size: The half-length of the bounding box.

    Returns:

    - cropped_laz: A laspy point cloud contains the points within the bounding box.
    """
    with laspy.open(file_path) as f:
        las = f.read()
        min_x, max_x = center_x - half_size, center_x + half_size
        min_y, max_y = center_y - half_size, center_y + half_size
        mask = (
            (las.x >= min_x) & (las.x <= max_x) &
            (las.y >= min_y) & (las.y <= max_y)
        )
        cropped_laz = las[mask]
        return cropped_laz


def plot_pointcloud(points, field, cmap="RdYlGn_r"):
    """
    To plot the point clouds.

    ----

    Parameters:

    - points: A laspy AHN4 point cloud.
    - field: Certain field to be colorized (e.g. points.z)
    - cmap: A string, indicates a matplotlib colormap, default cmap is "RdYlGn_r".
    """
    ax = plt.axes(projection='3d')
    ax.scatter(points.x, points.y, points.z, c=field, s=0.3, cmap=cmap)
    min_x, max_x = int(min(points.x)), int(max(points.x)) + 1
    min_y, max_y = int(min(points.y)), int(max(points.y)) + 1
    min_z, max_z = int(min(points.z)), int(max(points.z)) + 1
    range = max(max_x - min_x, max_y - min_y, max_z - min_z)
    ax.set_xlim(min_x, min_x + range)
    ax.set_ylim(min_y, min_y + range)
    ax.set_zlim(min_z, min_z + range)
    #ax.set_zlim(min_z, max_z)
    plt.show()


def plot_grid(grid, cmap="RdYlGn_r"):
    plt.imshow(grid, cmap=cmap)
    plt.colorbar()
    plt.title('Grid Plot with imshow')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.show()


def create_affine_transform(top_left_x, top_left_y, res):
    """
    Create Affine transform for the write_output function.
    """
    transform = Affine.translation(top_left_x - res / 2, top_left_y - res / 2) * Affine.scale(res, -res)
    return transform

def write_output(dataset, output, transform, name='output.tiff'):
    """
    Write grid to .tiff file.

    ----

    Parameters:

    - dataset: The point cloud read from laspy, we need its CRS --> dataset.header.parse_crs()
    - output: the output grid, a numpy grid.
    - name: the name of output file, default is 'output.tiff'.
    - transform:
      a user defined rasterio Affine object, used for the transforming the pixel coordinates
      to spatial coordinates.
    """
    output_file = name
    with rasterio.open(output_file, 'w',
                       driver='GTiff',
                       height=output.shape[0],
                       width=output.shape[1],
                       count=1,
                       dtype=output.dtype,
                       crs=dataset.header.parse_crs(),
                       transform=transform) as dst:
        dst.write(output, 1)
    print("File written to '%s'" % output_file)



if __name__ == '__main__':
    points = crop_laz("../data/46AN1_20.LAZ", 184420, 420538, 250)[0:4]
    print(points.x)