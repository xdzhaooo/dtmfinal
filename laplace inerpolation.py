import numpy as np
import startinpy

def laplace_interpolation(points, center_point, resolution, half_size):
    ax = center_point[0]-half_size
    ay = center_point[1]-half_size
    bx = center_point[0]+half_size
    by = center_point[1]+half_size

    # create pixel center coordinates
    pixel_center = []
    x_number = int((bx-ax)/resolution)
    y_number = int((by-ay)/resolution)


    for i in range(y_number):
        cy = by + resolution / 2 - (i+1) * (resolution)
        for j in range(x_number):
            cx = ax - resolution / 2 + (j+1) * (resolution)
            c_point = [cx,cy,np.nan]
            pixel_center.append(c_point)

    pixel_value = np.full((x_number, y_number), np.nan)
    pixel_center = np.array(pixel_center)
    dt = startinpy.DT()
    dt.insert(points)
    for k,pt in enumerate(pixel_center):
        print(k, pt)
        if dt.is_inside_convex_hull(pt[0],pt[1]):
            dt_with_pt = startinpy.DT()
            dt_with_pt.insert(points)
            dt_with_pt.insert_one_pt(pt[0],pt[1],pt[2])
            #adjacent_vertices
            vi = dt_with_pt.closest_point(pt[0],pt[1])
            adjacent_vertices_vi = dt_with_pt.adjacent_vertices_to_vertex(vi)
            adjacent_vertices = dt_with_pt.points[adjacent_vertices_vi]
            adjacent_tris = dt_with_pt.incident_triangles_to_vertex(vi)
            adjacent_tris_coordinates = dt_with_pt.points[adjacent_tris]

            tri_centers = []
            for tri in adjacent_tris_coordinates:
                adjacent_tris_center =triangle_center(tri)
                tri_centers.append(adjacent_tris_center)

            weight_list = []
            value_list = []
            for m, pt_adjacent in enumerate(adjacent_vertices):
                value = pt_adjacent[2]
                if m == 0:
                    edge_i = distance(tri_centers[0], tri_centers[-1])
                    xp_i = distance(pt,pt_adjacent)
                    weight = edge_i/xp_i

                    weight_list.append(weight)
                    value_list.append(value)
                else:
                    edge_i = distance(tri_centers[m], tri_centers[m-1])     #差值算法有问题吗87
                    xp_i = distance(pt,pt_adjacent)
                    weight = edge_i/xp_i
                    weight_list.append(weight)
                    value_list.append(value)

            weight_list, value_list = np.array(weight_list), np.array(value_list)
            pt_value = np.sum(weight_list * value_list/np.sum(weight_list))

            pixel_center[k,2] = pt_value
            pixel_value[]


    print(pixel_center.shape)


    # 可能需要判断pixel——center是否与ground_points 中某一点重合
...


def distance(pt0,pt1):
    xy0 = np.array(pt0[:2])
    xy1 = np.array(pt1[:2])
    return np.linalg.norm(xy0-xy1)

def triangle_center(tri):
    x1,y1 = np.array(tri[0,:2])
    x2,y2 = np.array(tri[1,:2])
    x3,y3 = np.array(tri[2,:2])

    d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    ox = ((x1**2 + y1**2) * (y2 - y3) + (x2**2 + y2**2) * (y3 - y1) + (x3**2 + y3**2) * (y1 - y2)) / d
    oy = ((x1**2 + y1**2) * (x3 - x2) + (x2**2 + y2**2) * (x1 - x3) + (x3**2 + y3**2) * (x2 - x1)) / d

    return ox,oy



if __name__ == '__main__':
    bbox = [0,0,3,3]
    points = [[0,0,3],[3,0,0],[0,3,3],[3,3,0]]
    laplace_interpolation(points,[1.5,1.5],0.5, 1.5)
