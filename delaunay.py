import math
import numpy as np

class DelaunayTriangulation:
    def __init__(self,points):
        self.triangles = []
        self.points = points

    def calculate_tri(self):
        self.triangles = []
        list_group3 = group3(len(self.points))
        for a,b,c in list_group3:
            tri = [self.points[a],self.points[b], self.points[c]]
            if self.isDelaunay(tri):
                self.triangles.append(tri)


    def isDelaunay(self,tri):
        if self.are_collinear(tri):
            return False
        else:
            tri_circle = circumcircle(tri)
            pointlist = [1 for p in self.points if circum_covers(tri_circle,p)]
            if len(pointlist) == 3:
                return True
            else:
                return False

    def are_collinear(self, tri):

        ax = tri[0][0]
        ay = tri[0][1]
        bx = tri[1][0]
        by = tri[1][1]
        cx = tri[2][0]
        cy = tri[2][1]
        return abs((ax - cx) * (by - cy) - (bx - cx) * (ay - cy)) <= 1e-8

    def copy(self):
        new = DelaunayTriangulation(self.points)
        new.calculate_tri()
        return new

    def convex_hull(self):
        self_pts = np.array(self.points)

        # find point a with minimum y    13.2
        ymin = min(self_pts[:,1])
        mask = self_pts[:,1]==ymin
        a = self_pts[mask][0]
        pt_ymin = a



        # for pt in self_pts:
        b = [9999999999999, a[1]]
        convexhull_pts = [a.tolist()]
        bre = 0
        while bre==0:
            arc_max = 0
            max_index = None
            for i, pt in enumerate(self_pts):
                print("a",a,"b",b,'pt', pt)
                if (a==pt).all():
                    continue
                vector1 = np.array([b[0]-a[0], b[1]-a[1]])
                vector2 = np.array([pt[0]-a[0], pt[1]-a[1]])
                print(vector1,vector2)
                cos_ = np.dot(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
                sin_ = np.cross(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
                arctan2_ = np.arctan2(sin_, cos_)
                print('arctan', arctan2_)
                if arctan2_>arc_max:
                    arc_max = arctan2_
                    max_index = i
            a, b = self_pts[max_index], a
            if (pt_ymin==self_pts[max_index]).all():
                bre = 1
            else:
                convexhull_pts.append(self_pts[max_index].tolist())
        print(convexhull_pts)


        return convexhull_pts




def group3(N):
    for i in range(N-2):
        for j in range(i+1,N-1):
            for k in range(j+1,N):
                yield (i,j,k)


def circumcircle(tri):
    ax = tri[0][0]
    ay = tri[0][1]
    bx = tri[1][0]
    by = tri[1][1]
    cx = tri[2][0]
    cy = tri[2][1]
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    ux = ((ax ** 2 + ay ** 2) * (by - cy) + (bx ** 2 + by ** 2) * (cy - ay) + (cx ** 2 + cy ** 2) * (ay - by)) / d
    uy = ((ax ** 2 + ay ** 2) * (cx - bx) + (bx ** 2 + by ** 2) * (ax - cx) + (cx ** 2 + cy ** 2) * (bx - ax)) / d
    center = ux, uy
    r = math.sqrt((ax - ux) ** 2 + (ay - uy) ** 2)
    return center, r

def circum_covers(tri_circle,p):
    c_x = tri_circle[0][0]
    c_y = tri_circle[0][1]
    p_x = p[0]
    p_y = p[1]
    d = math.sqrt((c_x - p_x) ** 2 + (c_y - p_y) ** 2)
    r = tri_circle[1]
    if d > r:
        return True
    else:
        return False

def convex_hull(points):
    self_pts = np.array(points)

    # find point a with minimum y    13.2
    ymin = min(self_pts[:,1])
    mask = self_pts[:,1]==ymin
    a = self_pts[mask][0]
    pt_ymin = a



    # for pt in self_pts:
    b = [9999999999999, a[1]]
    convexhull_pts = [a.tolist()]
    bre = 0
    while bre==0:
        arc_max = 0
        max_index = None
        for i, pt in enumerate(self_pts):
            print("a",a,"b",b,'pt', pt)
            if (a==pt).all():
                continue
            vector1 = np.array([b[0]-a[0], b[1]-a[1]])
            vector2 = np.array([pt[0]-a[0], pt[1]-a[1]])
            print(vector1,vector2)
            cos_ = np.dot(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
            sin_ = np.cross(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
            arctan2_ = np.arctan2(sin_, cos_)
            print('arctan', arctan2_)
            if arctan2_>arc_max:
                arc_max = arctan2_
                max_index = i
        a, b = self_pts[max_index], a
        if (pt_ymin==self_pts[max_index]).all():
            bre = 1
        else:
            convexhull_pts.append(self_pts[max_index].tolist())
    print(convexhull_pts)


#if __name__=='__main__':