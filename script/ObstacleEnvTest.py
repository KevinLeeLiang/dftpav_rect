import numpy as np

def calculate_point_on_arc(radius, center, start_angle, end_angle, num_points):
  x0, y0 = center
  angle_range = end_angle - start_angle
  angle_increment = angle_range / num_points

  points = []
  for i in range(0, num_points + 1):
    angle = start_angle + i * angle_increment
    x = x0 + radius * np.cos(np.radians(angle))
    y = y0 + radius * np.sin(np.radians(angle))
    angle_degrees = np.degrees(np.arctan2(np.sin(np.radians(angle + 90)), np.cos(np.radians(angle + 90)))) % 360
    points.append((x, y, angle_degrees))

  return points

def calculate_arc_points(start_point, end_point, radius, count, clockwise=True):
  # 将起点和终点转换为numpy数组
  start = np.array(start_point)
  end = np.array(end_point)

  # 计算圆心
  mid_point = 0.5 * (start + end)
  direction = np.array([-start[1] + end[1], start[0] - end[0]])
  dis = np.linalg.norm(mid_point - start)
  if clockwise:
    center = mid_point + dis * direction / np.linalg.norm(direction)
  else:
    center = mid_point - dis * direction / np.linalg.norm(direction)

  # 计算角度范围
  start_angle = np.arctan2(start[1] - center[1], start[0] - center[0])
  end_angle = np.arctan2(end[1] - center[1], end[0] - center[0])
  if clockwise:
    if start_angle < end_angle:
      end_angle -= 2 * np.pi
  else:
    if start_angle > end_angle:
      end_angle += 2 * np.pi

  # 在角度范围内均匀采样点
  num_points = count
  theta = np.linspace(start_angle, end_angle, num_points)
  sign = -1 if clockwise else 1
  arc_points = np.column_stack((center[0] + radius * np.cos(theta), center[1] + radius * np.sin(theta), (theta + sign*np.pi/2)))

  return arc_points



def test():
  x1, y1 = 3, 10
  x2, y2 = 15, 10
  x3, y3 = 15, 45
  x4, y4 = 3, 45
  p1_2 = np.linspace(x1, x2, 120)
  p2_3 = np.linspace(10, 45, 350)
  with open('../Sim/example.txt', 'w') as file:
    for x in p1_2:
      s = str(x) + ',' + str(y1) + '\n'
      file.write(s)
    for y in p2_3:
      s = str(x2) + ',' + str(y)+ '\n'
      file.write(s)
  file.close()

  xc1, yc1 = 5, 25
  xc2, yc2 = 45, 25
  xs1, ys1, hs1 = 5, 5, 270
  xe1, ye1, he1 = 25, 25, 360
  xs2, ys2, hs2 = 25, 25, 180
  xe2, ye2, he2 = 45, 45, 90
  r1 = 20
  r2 = 20
  count = 20
  path1 = calculate_arc_points((xs1, ys1), (xe1, ye1), r1, count, False)
  path2 = calculate_arc_points((xs2, ys2), (xe2, ye2), r2, count, True)

  with open('../Sim/path.txt', 'w') as file:
    for p in path1:
      s = str(p[0]) + ',' + str(p[1]) + ',' + str(p[2]) + '\n'
      file.write(s)
    for p in path2:
      s = str(p[0]) + ',' + str(p[1]) + ',' + str(p[2]) + '\n'
      file.write(s)
  file.close()

if __name__ == '__main__':
  test()