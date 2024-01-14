import numpy as np
import laspy
import delaunay as dl
import startinpy
from scipy.spatial import cKDTree
from matplotlib import pyplot as plt


def cropped_laz(bbx_centre=(184380,420660),size=500,buffer=50,filename='./46AN1_20.LAZ'):
    las = laspy.read(filename)

    x_min, x_max = bbx_centre[0]-(size/2+buffer), bbx_centre[0]+(size/2+buffer) # 500m grid with buffer 50m
    y_min, y_max = bbx_centre[1]-(size/2+buffer), bbx_centre[1]+(size/2+buffer)

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

def gftin(cropped_las,bbx_centre=(184380,420660),size = 500, buffer = 50, r_max=0.2, alpha_max=0.1):
    # step 1, extract the lowest points from grids
    # min_x, max_x = min(cropped_las[:, 0]), max(cropped_las[:, 0])  # use bounding box to replace this
    # min_y, max_y = min(cropped_las[:, 1]), max(cropped_las[:, 1])
    # print(min_x,min_y,max_x,max_y)
    min_x, max_x = bbx_centre[0]-(size/2+buffer), bbx_centre[0]+(size/2+buffer) # 500m grid with buffer 50m
    min_y, max_y = bbx_centre[1]-(size/2+buffer), bbx_centre[1]+(size/2+buffer)

    grid_len = 30 #argument
    lowest_points = []
    for i in np.arange(min_x, max_x, grid_len):
        for j in np.arange(min_y, max_y, grid_len):
            # Create a mask for points within the current grid cell
            mask = (
                    (cropped_las[:, 0] >= i) & (cropped_las[:, 0] <= i + grid_len) &
                    (cropped_las[:, 1] >= j) & (cropped_las[:, 1] <= j + grid_len)
            )

            # Extract points within the grid cell
            points_in_grid = cropped_las[mask]

            if len(points_in_grid) > 0:
                # Find the point with the lowest Z-coordinate
                lowest_point = points_in_grid[np.argmin(points_in_grid[:, 2])]
                lowest_points.append(lowest_point)
            else:
                print('empty cell')

    lowest_p = np.array(lowest_points)

    # rudimentary TIN
    dt=startinpy.DT()
    dt.insert(lowest_p)
    new_dl = dt
    mask_without_buffer = (
                    (cropped_las[:, 0] >= min_x+buffer) & (cropped_las[:, 0] <= max_x - buffer) &
                    (cropped_las[:, 1] >= min_y+buffer) & (cropped_las[:, 1] <= max_y - buffer))
    cropped_las_without_buffer = cropped_las[mask_without_buffer]
    las_len = cropped_las_without_buffer.shape[0]
    thinned_indices = np.random.choice(las_len, size=int(len(cropped_las_without_buffer)*0.1), replace=False)
    cropped_las_without_buffer_thinned = cropped_las_without_buffer[thinned_indices,:]
    print(cropped_las_without_buffer_thinned)
    # step2&3 Tin refinement,First traverse the points, calculate epslon, and insert from the point closest to tin
    # until there are no points within epslon.
    e_r = 0
    e_alpha = 0
    allpts = cropped_las_without_buffer_thinned
    print('len',len(allpts))
    tin_pt = new_dl.triangles
    rudi_tin_pts = new_dl.points
    count = 0
    while e_r < r_max and e_alpha < alpha_max and len(allpts)>0:
        e_r = r_max
        e_alpha = alpha_max
        insert_pt = None
        count += 1
        print(count)
        for i,pt in enumerate(allpts):
            tri_index = new_dl.locate(pt[0],pt[1])
            tri = new_dl.points[tri_index]
            r, alpha = cal_r_alpha(pt,tri)
            if e_r > r and e_alpha > alpha:              # 写到一行对吗，不确定？？？？？？？？？？？
                print(1)
                e_r = r
                e_alpha = alpha
                i_selected = i
                insert_pt = allpts[i_selected][0],allpts[i_selected][1],allpts[i_selected][2]
        if insert_pt:
            new_dl.insert_one_pt(allpts[i_selected][0], allpts[i_selected][1],allpts[i_selected][2])
            allpts = np.delete(allpts,i_selected,axis=0)



    print(new_dl.points.shape)
    ax = plt.axes(projection='3d')
    ax.scatter(new_dl.points[:,0],new_dl.points[:,1],new_dl.points[:,2], color = "r")
    # ax.scatter(cropped_las[:, 0], cropped_las[:, 1], cropped_las[:, 2])
    # min_x, max_x = int(min(points.x)), int(max(points.x)) + 1
    # min_y, max_y = int(min(points.y)), int(max(points.y)) + 1
    # min_z, max_z = int(min(points.z)), int(max(points.z)) + 1
    # range = max(max_x - min_x, max_y - min_y, max_z - min_z)
    # ax.set_xlim(min_x, min_x + range)
    # ax.set_ylim(min_y, min_y + range)
    # ax.set_zlim(min_z, min_z + range)
    # ax.set_zlim(min_z, max_z)
    plt.show()
    f = open("k.txt", "w")
    f.writelines(str(new_dl.points.tolist()))
    f.close()
    return new_dl.points


def cal_r_alpha(pt,tri):
    A=tri[0]
    B=tri[1]
    C=tri[2]

    # 计算三角形所在平面的法向量
    normal = np.cross(B - A, C - A)

    # 计算点到平面的有向距离
    distance = np.dot(pt - A, normal) / np.linalg.norm(normal)
    distance = abs(distance)
    # 计算点到三个顶点的距离
    distances_to_vertices = [np.linalg.norm(pt - vertex) for vertex in [A, B, C]]
    l1 = distances_to_vertices[0]
    l2 = distances_to_vertices[1]
    l3 = distances_to_vertices[2]

    # 检查是否点在三角形的边界上
    if any(d == 0 for d in distances_to_vertices):
        distance = 0.0

    if l1!=0 and l2!=0 and l3!=0:
        beta_1 = np.arcsin(distance/l1)
        beta_2 = np.arcsin(distance/l2)
        beta_3 = np.arcsin(distance/l3)
        alpha = max(beta_3,beta_2,beta_1)
    else:
        alpha = np.inf

    return distance,alpha



if __name__ == '__main__':
    cropped_las = cropped_laz(size=20,buffer=30)
    gftin(cropped_las,size=20,buffer=30)

