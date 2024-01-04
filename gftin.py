import numpy as np
import laspy
import delaunay as dl
import startinpy


def cropped_laz(filename='./46AN1_20.LAZ'):
    las = laspy.read(filename)

    x_min, x_max = 184130, 184180
    y_min, y_max = 420410, 420460

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

def gftin(cropped_las, r_max=1.0, alpha_max=0.5):
    # step 1, extract the lowest points from grids
    min_x, max_x = min(cropped_las[:, 0]), max(cropped_las[:, 0])
    min_y, max_y = min(cropped_las[:, 1]), max(cropped_las[:, 1])
    # print(min_x,min_y,max_x,max_y)

    grid_len = 20
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
    # r_dl = dl.DelaunayTriangulation(lowest_p)
    # r_dl.calculate_tri()
    # tin = r_dl.triangles

    dt=startinpy.DT()
    dt.insert(lowest_p)
    new_dl = dt

    # step2&3 Tin refinement,First traverse the points, calculate epslon, and insert from the point closest to tin
    # until there are no points within epslon.
    e_r = 0
    e_alpha = 0
    allpts = cropped_las.copy()
    print('len',len(allpts))
    while e_r < r_max and e_alpha < alpha_max and len(allpts)>0:
        e_r = r_max
        e_alpha = alpha_max
        insert_pt = None
        for m in range(len(allpts)):
            pt = allpts[m]
            for n in range(len(new_dl.triangles)):
                tri_list = list(new_dl.triangles[n])
                tri = new_dl.points[tri_list]
                if p_within_tri(pt,tri):
                    r, alpha = cal_r_alpha(pt,tri)
                    if e_r > r and e_alpha > alpha:              # 写到一行对吗，不确定？？？？？？？？？？？
                        e_r = r
                        e_alpha = alpha
                        i_selected = m
                        insert_pt = allpts[i_selected][0],allpts[i_selected][1],allpts[i_selected][2]
        print(insert_pt)
        if insert_pt:
            new_dl.insert_one_pt(allpts[i_selected][0], allpts[i_selected][1],allpts[i_selected][2])
            allpts = np.delete(allpts,i_selected,axis=0)
            print(len(allpts))

        print(len(new_dl.points))

    return new_dl.points

def p_within_tri(pt,tri):
    ax = tri[0][0]
    ay = tri[0][1]
    bx = tri[1][0]
    by = tri[1][1]
    cx = tri[2][0]
    cy = tri[2][1]
    ptx = pt[0]
    pty = pt[1]
    # calculate tri_area
    ABC = 0.5 * abs(ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))

    # calculate sub_tri_area
    PBC = 0.5 * abs(ptx * (by - cy) + bx * (cy - pty) + cx * (pty - by))
    PAC = 0.5 * abs(ax * (cy - pty) + ptx * (ay - cy) + cx * (pty - ay))
    PAB = 0.5 * abs(ax * (pty - by) + bx * (ay - pty) + ptx * (by - ay))

    # 如果子三角形的面积之和等于三角形的面积，点在三角形内
    return abs(ABC - (PBC + PAC + PAB)) <=1e-8


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
    cropped_las = cropped_laz()
    gftin(cropped_las)

