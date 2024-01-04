import numpy as np
import delaunay


def laplace_interpolation(bbox,size=0.5,points):
    ax = bbox[0]
    ay = bbox[1]
    bx = bbox[2]
    by = bbox[3]

    # create pixel center coordinates
    pixel_center = []
    x_number = int((bx-ax)/size)
    y_number = int((by-ay)/size)

    for i in range(y_number):
        cy = by + size / 2 - (i+1) * (size)
        for j in range(x_number):
            cx = ax - size / 2 + (j+1) * (size)
            c_point = [cx,cy,np.nan]
            pixel_center.append(c_point)

    # append pixel center points to ground points, and create delaunay triangulate
    ground_points = points.tolist()
    # 可能需要判断pixel——center是否与ground_points 中某一点重合
...
    # C_p 才ground_p convexhull 外-nodata，并排除在外
...
    # 将所有groun_points中，每次分别插入一个像素点计算的delaunay
    for pt in pixel_pts:
        pts_with_one_pixel=points.append(pt)
        dl = delaunay.DelaunayTriangulation(pts_with_one_pixel)
        pt = np.array(pt)
        tris = np.array(dl.triangles)
        tris_with_pt = []
        for tri in tris:
            if (pt==tris).any():
                tris_with_pt.append(tri)

        pts_in_tris = tris_with_pt.reshape(-1)
        pts_in_tris = np.unique(pts_in_tris)
        pts_in_tris = pts_in_tris[~np.all(pts_in_tris == pt, axis=1)]

        #返回顺时针顺序的点
        cw_pts = delaunay.convex_hull(pts_in_tris)

        #






if __name__ == '__main__':
    bbox = [0,0,1.5,1.5]
    laplace_interpolation(bbox,0.5)
