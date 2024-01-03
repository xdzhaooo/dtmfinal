import numpy as np
import laspy


def cropped_laz(filename='./46AN1_20.LAZ'):
    las = laspy.read(filename)

    x_min, x_max = 184130, 184630
    y_min, y_max = 420410, 420900

    mask = (
            (las.x >= x_min) & (las.x <= x_max) &
            (las.y >= y_min) & (las.y <= y_max)
    )
    cropped_las = las.xyz[mask].copy()
    cropped_las_ = las.points[mask].copy()

    output_file = laspy.LasData(las.header)
    output_file.points = cropped_las_
    output_file.write("cropped_las.las")

    return cropped_las


def gftin(cropped_las):
    # step 1, extract the lowest points from grids
    min_x, max_x = min(cropped_las[:, 0]), max(cropped_las[:, 0])
    min_y, max_y = min(cropped_las[:, 1]), max(cropped_las[:, 1])
    # print(min_x,min_y,max_x,max_y)

    grid_len = 80
    i, j = min_x, min_y
    lowest_p = []
    while i <= max_x and j <= max_y:
        # mask for superimposing grids
        # <= or <？？？？？？？？？
        mask1 = (
                (cropped_las[:, 0] >= i) & (cropped_las[:, 0] <= i + grid_len) &
                (cropped_las[:, 1] >= j) & (cropped_las[:, 1] <= j + grid_len)
        )
        p_in_grid = cropped_las[mask1]
        lowest_value = min(p_in_grid[:, 2])
        # mask for finding the lowest point index
        mask2 = np.isclose(p_in_grid[:, 2], lowest_value)
        p_lowest = (p_in_grid[mask2][0])
        lowest_p.append(p_lowest)
        i += grid_len
        j += grid_len
    lowest_p = np.array(lowest_p)
    print(lowest_p)

    # rudimentary TIN



if __name__ == '__main__':
    cropped_las = cropped_laz()
    gftin(cropped_las)
