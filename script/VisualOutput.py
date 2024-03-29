import matplotlib.pyplot as plt

def test():
  plt.axis("equal")
  file_path = "../Sim/example.txt"
  with open(file_path, 'r') as file:
    for line in file:
      x,y = [], []
      data = line.split(",")
      x.append(float(data[0]))
      y.append(float(data[1]))
      plt.plot(x, y, 'ro')

  file_path = "../Sim/path.txt"
  with open(file_path, 'r') as file:
    for line in file:
      x,y = [], []
      data = line.split(",")
      x.append(float(data[0]))
      y.append(float(data[1]))
      plt.plot(x, y, 'go')

  file_path = "../Sim/output.txt"  # 文件路径，根据实际情况修改
  # 打开文件
  with open(file_path, 'r') as file:
    # 逐行读取文件内容
    for line in file:
      x, y = [], []
      data = line.split(",")
      x = data[::2]
      y = data[1::2]
      x = [float (t) for t in x ]
      y = [float (t) for t in y ]
      x.append(x[0])
      y.append(y[0])

      plt.plot(x, y)

  # 关闭文件
  file.close()
  plt.show()
if __name__ == '__main__':
  test()