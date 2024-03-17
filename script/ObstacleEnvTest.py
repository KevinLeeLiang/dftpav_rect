import numpy as np

def calculate_point_on_arc(radius, center, start_angle, end_angle, num_points):
  x0, y0 = center
  angle_range = end_angle - start_angle
  angle_increment = angle_range / num_points

  points = []
  for i in range(num_points + 1):
    angle = start_angle + i * angle_increment
    x = x0 + radius * np.cos(np.radians(angle))
    y = y0 + radius * np.sin(np.radians(angle))
    points.append((x, y))

  return points


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
  xe1, ye1, he1 = 20, 25, 360
  xs2, ys2, hs2 = 20, 25, 180
  xe2, ye2, he2 = 45, 45, 90
  r1 = 20
  r2 = 20
  count = 5
  path1 = calculate_point_on_arc(r1, (xc1, yc1), 270, 360, count)
  path2 = calculate_point_on_arc(r2, (xc2, yc2), 90, 180, count)
  path2 = path2[::-1]

  with open('../Sim/path.txt', 'w') as file:
    for p in path1:
      s = str(p[0]) + ',' + str(p[1]) + '\n'
      file.write(s)
    for p in path2:
      s = str(p[0]) + ',' + str(p[1]) + '\n'
      file.write(s)
  file.close()

if __name__ == '__main__':
  test()