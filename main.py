import croplaz as crop
import gftin
import delaunay

if __name__=='__main__':
    #cropped_las = crop.cropped_laz()
    #gftin.gftin(cropped_las)
    #test ymin
    pts = [[1,5],[2,1],[4,5]]
    dl = delaunay.DelaunayTriangulation(pts)
    ymin = dl.convex_hull()
