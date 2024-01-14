import croplaz as crop
import gftin
import delaunay

# if __name__=='__main__':
#     #cropped_las = crop.cropped_laz()
#     #gftin.gftin(cropped_las)
#     #test ymin
#     pts = [[1,5],[2,1],[4,5]]
#     dl = delaunay.DelaunayTriangulation(pts)
#     ymin = dl.convex_hull()

import matplotlib.pyplot as plt
import numpy as np

# 从文本文件中读取数据
file_path = 'k.txt'  # 替换为实际文件路径

with open(file_path, 'r') as file:
    lines = file.readlines()
    lines = [line.replace('[', '').replace(']', '').replace(',', ' ') for line in lines]

data = np.loadtxt(file_path)

# 提取每列数据
x = data[:, 0]
y = data[:, 1]
z = data[:, 2]

# 绘制散点图
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(x, y, z, c='r', marker='o')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()