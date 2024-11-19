import matplotlib.pyplot as plt
import random
import math
from scipy.spatial import KDTree

from util.geometry import Pose2d


kWidth = 80.0
kHeight = 100.0

enum_kScissor = 1
enum_kPaper = 2
enum_kRock = 3

class Point(Pose2d):
  def __init__(self, x = 0, y = 0, heading = 0, v = 0, kappa = 0, type = 0) -> None:
    super().__init__(x, y, heading, v, kappa)
    self.type = type
    self.id = hash((self.x, self.y, self.heading))
  
  def FindNearestPoint(self, points_kd_tree):
    distance, index = points_kd_tree.query([self.x, self.y])
    return index, distance

  def UpdateChase(self, target_point):
    self.heading = math.atan2(target_point.y - self.y, target_point.x - self.x)
    self.x += self.v * math.cos(self.heading) * 0.5
    self.y += self.v * math.sin(self.heading) * 0.5
    self.x = min(max(self.x, 0), kWidth)
    self.y = min(max(self.y, 0), kHeight)

  def UpdateEvasive(self, target_point):
    self.heading = math.atan2(target_point.y - self.y, target_point.x - self.x)
    self.x += self.v * math.cos(self.heading) * 0.5
    self.y += self.v * math.sin(self.heading) * 0.5
    self.x = min(max(self.x, 0), kWidth)
    self.y = min(max(self.y, 0), kHeight)
    
  def UpdateNoTarget(self):
    self.x += self.v * math.cos(self.heading) * 0.5
    self.y += self.v * math.sin(self.heading) * 0.5
    self.x = min(max(self.x, 0), kWidth)
    self.y = min(max(self.y, 0), kHeight)


kMaxPointNum = 30

points = []
point_id_map = {}

for i in range(kMaxPointNum):
  type = random.randint(1, 3)
  x = random.uniform(0, kWidth)
  y = random.uniform(0, kHeight)
  heading = random.uniform(-math.pi, math.pi)
  v = random.uniform(1.5, 2.5)
  kappa = 0.0
  point = Point(x, y, heading, v, kappa, type)
  point_id_map[point.id] = point

finish = False

plt.figure(figsize=(3, 4))
while (not finish):
  plt.cla()
  
  plt.plot(0, 0, 'ro', alpha = 1.0, label='scissor')
  plt.plot(0, 0, 'b*', alpha = 1.0, label='paper')
  plt.plot(0, 0, 'g.', alpha = 1.0, label='rock')

  scissor_points = []
  paper_points = []
  rock_points = []
  scissor_ids = []
  paper_ids = []
  rock_ids = []
  for id, point in point_id_map.items():
    if point.type == enum_kScissor:
      scissor_points.append([point.x, point.y])
      scissor_ids.append(id)
    elif point.type == enum_kPaper:
      paper_points.append([point.x, point.y])
      paper_ids.append(id)
    elif point.type == enum_kRock:
      rock_points.append([point.x, point.y])
      rock_ids.append(id)

  no_scissor = False
  if (len(scissor_points) == 0):
    no_scissor = True
  else:
    scissor_kd_tree = KDTree(scissor_points)
  no_paper = False
  if (len(paper_points) == 0):
    no_paper = True
  else:
    paper_kd_tree = KDTree(paper_points)
  no_rock = False
  if (len(rock_points) == 0):
    no_rock = True
  else:
    rock_kd_tree = KDTree(rock_points)

  if (int(no_scissor) + int(no_paper) + int(no_rock) >= 2):
    finish = True

  for id, point in point_id_map.items():
    if point.type == enum_kScissor:
      if no_paper:
        point.UpdateNoTarget()
      else:
        index, distance = point.FindNearestPoint(paper_kd_tree)
        target_point = point_id_map[paper_ids[index]]
        point.UpdateChase(target_point)
    elif point.type == enum_kPaper:
      if no_rock:
        point.UpdateNoTarget()
      else:
        index, distance = point.FindNearestPoint(rock_kd_tree)
        target_point = point_id_map[rock_ids[index]]
        point.UpdateChase(target_point)
    elif point.type == enum_kRock:
      if no_scissor:
        point.UpdateNoTarget()
      else:
        index, distance = point.FindNearestPoint(scissor_kd_tree)
        target_point = point_id_map[scissor_ids[index]]
        point.UpdateChase(target_point)
      
    if target_point.DistanceTo(point) < 0.8:
      target_point.type = point.type

  for id, point in point_id_map.items():
    if point.type == enum_kScissor:
      plt.plot(point.x, point.y, 'ro')
    elif point.type == enum_kPaper:
      plt.plot(point.x, point.y, 'b*')
    elif point.type == enum_kRock:
      plt.plot(point.x, point.y, 'g.')

  plt.plot([0, 0, kWidth, kWidth, 0], [0, kHeight, kHeight, 0, 0], 'k--', linewidth=2, alpha = 0.2)
  
  plt.legend(loc='upper right')

  plt.axis('equal')


  plt.pause(0.01)

plt.show()


