import math

class Point2d:
  def __init__(self, x = 0, y = 0) -> None:
    self.x = x
    self.y = y
  def DistanceTo(self, other):
     return math.hypot(self.x - other.x, self.y - other.y) 
  def __eq__(self, other) -> bool:
    return self.x == other.x and self.y == other.y
  def __hash__(self):
        return hash((self.x, self.y))
  
class Pose2d(Point2d):
  def __init__(self, x = 0, y = 0, heading = 0, v = 0, kappa = 0) -> None:
    super().__init__(x, y)
    self.heading = heading
    self.v = v
    self.kappa = kappa
  def __eq__(self, other) -> bool:
    return super().__eq__(other) and self.heading == other.heading
  def __hash__(self):
        return hash((self.x, self.y, self.heading))
class LineSegment:
  def __init__(self, p0: Point2d, p1: Point2d) -> None:
    self.p0 = p0
    self.p1 = p1
  
  def DistanceTo(self, point: Point2d):
    projection = self.GetProjection(point)
    return point.DistanceTo(projection)
  
  def GetProjection(self, point: Point2d):
    if self.p0 == self.p1:
      return self.p0
    dx = self.p1.x - self.p0.x
    dy = self.p1.y - self.p0.y
    # Calculate the t that minimizes the distance.
    t = ((point.x - self.p0.x) * dx + (point.y - self.p0.y) * dy) / (dx * dx + dy * dy)
    # See if this represents one of the segment's end points or a point in the middle.
    if t < 0:
      return self.p0
    elif t > 1:
      return self.p1
    else:
      projection = Point2d(self.p0.x + t * dx, self.p0.y + t * dy)
      return projection
  
  def ProjectOnTo(self, point: Point2d):
    pass
  def ProductOnTo(self, point: Point2d):
    pass
    
  def HasIntersect(self, other):
    self_vec = Point2d(self.p1.x - self.p0.x, self.p1.y - self.p0.y)
    other_vec = Point2d(other.p1.x - other.p0.x, other.p1.y - other.p0.y)
    cross_product = self_vec.x * other_vec.y - self_vec.y * other_vec.x
    if (math.fabs(cross_product)) < 1e-6:
      return None
    da = Point2d(other.p0.x - self.p0.x, other.p0.y - self.p0.y)
    t = (da.x * other_vec.y - da.y * other_vec.x) / cross_product
    u = (da.x * self_vec.y - da.y * self_vec.x) / cross_product
    if t >= 0 and t <= 1 and u >= 0 and u <= 1:
      return Point2d(self.p0.x + t * self_vec.x, self.p0.y + t * self_vec.y)
    return None
  
  def HasIntersectWithPath(self, path):
    # consider binary search
    for i, pt in enumerate(path.path_points[:-1]):
      curr_pt = path.path_points[i]
      next_pt = path.path_points[i + 1]
      segment = LineSegment(curr_pt, next_pt)
      intersect = self.HasIntersect(segment)
      if intersect is not None:
        return intersect
    return None

class Path:
  def __init__(self, points = []) -> None:
    self.path_points = points
    self.reset_path_properties()

  def reset_path_properties(self):
    for i, pt in enumerate(self.path_points):
      curr_pt = self.path_points[i]
      heading = curr_pt.heading
      if (i == len(self.path_points) - 1):
        prev_pt = self.path_points[i - 1]
        heading = math.atan2(curr_pt.y - prev_pt.y, curr_pt.x - prev_pt.x)
      elif i == 0:
        next_pt = self.path_points[i + 1]
        heading = math.atan2(next_pt.y - curr_pt.y, next_pt.x - curr_pt.x)
      else:          
        prev_pt = self.path_points[i - 1]
        next_pt = self.path_points[i + 1]
        heading = math.atan2(next_pt.y - prev_pt.y, next_pt.x - prev_pt.x)
      self.path_points[i] = Pose2d(curr_pt.x, curr_pt.y, heading, curr_pt.v)
    for i, pt in enumerate(self.path_points):
      curr_pt = self.path_points[i]
      kappa = curr_pt.kappa
      if (i == len(self.path_points) - 1):
        prev_pt = self.path_points[i - 1]
        dist = curr_pt.DistanceTo(prev_pt)
        if (dist < 1e-6):
          kappa = 0
        else:
          kappa = (curr_pt.heading - prev_pt.heading) / dist
      elif i == 0:
        next_pt = self.path_points[i + 1]
        dist = next_pt.DistanceTo(curr_pt)
        if (dist < 1e-6):
          kappa = 0
        else:
          kappa = (next_pt.heading - curr_pt.heading) / dist
      else:          
        prev_pt = self.path_points[i - 1]
        next_pt = self.path_points[i + 1]
        dist = next_pt.DistanceTo(prev_pt)
        if (dist < 1e-6):
          kappa = 0
        else:
          kappa = (next_pt.heading - prev_pt.heading) / dist
      self.path_points[i] = Pose2d(curr_pt.x, curr_pt.y, curr_pt.heading, curr_pt.v, kappa)

  def downsample_path(self, dl = 1.0):
    new_path = []
    for i, pt in enumerate(self.path_points[:-1]):
      curr_pt = self.path_points[i]
      next_pt = self.path_points[i + 1]
      heading = math.atan2(next_pt.y - curr_pt.y, next_pt.x - curr_pt.x)
      while curr_pt.DistanceTo(next_pt) > dl:
        if (len(new_path) == 0 or new_path[-1].DistanceTo(curr_pt) > 0.1 * dl): 
          new_path.append(curr_pt)
        new_x = curr_pt.x + dl * math.cos(heading)
        new_y = curr_pt.y + dl * math.sin(heading)
        curr_pt = Pose2d(new_x, new_y, curr_pt.heading)
      new_path.append(next_pt)
    return Path(new_path)
  
  def PointToSL(self, point: Point2d):
    s = 0
    l = 0
    min_dist = 1000000
    cumulative_s = 0
    for i, pt in enumerate(self.path_points[:-1]):
      curr_pt = self.path_points[i]
      next_pt = self.path_points[i + 1]
      segment = LineSegment(curr_pt, next_pt)
      dist = segment.DistanceTo(point)
      proj = segment.GetProjection(point)
      if (dist < min_dist):
        min_dist = dist
        s = cumulative_s + curr_pt.DistanceTo(proj)
        l = point.DistanceTo(proj)
      cumulative_s += curr_pt.DistanceTo(next_pt)
    return s, l
  
  def GetPose2DByS(self, s):
    cumulative_s = 0
    for i, pt in enumerate(self.path_points[:-1]):
      curr_pt = self.path_points[i]
      next_pt = self.path_points[i + 1]
      segment_length = curr_pt.DistanceTo(next_pt)
      if cumulative_s + segment_length >= s:
        # Found the segment where 's' falls
        ratio = (s - cumulative_s) / segment_length
        x = curr_pt.x + ratio * (next_pt.x - curr_pt.x)
        y = curr_pt.y + ratio * (next_pt.y - curr_pt.y)
        direction = math.atan2(next_pt.y - curr_pt.y, next_pt.x - curr_pt.x)
        return Pose2d(x, y, direction)
      cumulative_s += segment_length
    return None