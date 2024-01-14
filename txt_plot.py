import matplotlib.pyplot as plt
import re

# 从文件中读取坐标
def read_coordinates_from_file(file_path):
    with open(file_path, 'r') as file:
        content = file.read()

    # 使用正则表达式匹配坐标
    pattern = re.compile(r'\[([\d\.-]+), ([\d\.-]+), ([\d\.-]+)\]')
    matches = pattern.findall(content)

    # 将匹配到的坐标转换为浮点数
    coordinates = [[float(match[0]), float(match[1]), float(match[2])] for match in matches]

    return coordinates

# 读取坐标
file_path = 'k.txt'  # 替换为你的文件路径
coordinates = read_coordinates_from_file(file_path)

# 将坐标拆分为 x、y、z
x = [coord[0] for coord in coordinates]
y = [coord[1] for coord in coordinates]
z = [coord[2] for coord in coordinates]

# 绘制散点图
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z, c='r', marker='o')

# 设置图表标题和坐标轴标签
ax.set_title('3D Scatter Plot')
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')

# 显示图表
plt.show()
