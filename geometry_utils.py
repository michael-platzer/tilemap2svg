
def convex_hull(points):
  points = list(points) # ensure we have a list (and not an iterator)
  # find up to 8 points that are for sure part of the convex hull
  pts_on_hull = [
    min(points, key=lambda pt: pt[0]        ), # smallest x coordinate (leftmost)
    min(points, key=lambda pt: pt[0] + pt[1]), # smallest x + y (most to the bottom-left)
    min(points, key=lambda pt: pt[1]        ), # smallest y coordinate (bottom-most)
    min(points, key=lambda pt: pt[1] - pt[0]), # smallest y - x (most to the bottom-right)
    max(points, key=lambda pt: pt[0]        ), # largest  x coordinate (rightmost)
    max(points, key=lambda pt: pt[0] + pt[1]), # largest  x + y (most to the top-right)
    max(points, key=lambda pt: pt[1]        ), # largest  y coordinate (topmost)
    max(points, key=lambda pt: pt[1] - pt[0])  # largest  y - x (most to the top-left)
  ]
  # remove duplicates (dict maintains insertion order since Python 3.7)
  pts_on_hull = list(dict.fromkeys(pts_on_hull))
  # remove all points that are inside these initial points by testing whether each point is to the
  # right of any of the segments of our initial set of points (if it is to the right of any segment
  # it lies outside and if it is to the left of all segments it lies inside)
  points = [(x, y) for (x, y) in points if any(
    # point (x, y) lies to the right of ((x0, y0),(x1, y1)) if the cosine sign is positive
    (x, y) == (x0, y0) or (x1 - x0) * (y - y0) - (y1 - y0) * (x - x0) > 0 for (x0, y0), (x1, y1) in
    zip(pts_on_hull, pts_on_hull[1:] + pts_on_hull[:1])
  )]
  # compute convex hull using Jarvis's march on the remaining points
  hull = [min(points, key=lambda pt: pt[0])] # start with leftmost point
  while True:
    x0, y0 = hull[-1]
    x1, y1 = next(pt for pt in points if pt != (x0, y0)) # choose some other point
    for x, y in points:
      if (x, y) != (x0, y0) and (x, y) != (x1, y1):
        if (x1 - x0) * (y - y0) - (y1 - y0) * (x - x0) > 0:
          x1, y1 = x, y
    if (x1, y1) in hull:
      return hull
    hull.append((x1, y1))


def dissolve_lines(lines, equal_dist=1.):
  # immediatly return closed lines, retain only open lines for further processing
  open_lines = []
  for line in lines:
    if line[0] == line[-1]:
      yield line
    else:
      open_lines.append(line)
  lines_to_remove = set()
  # iterate over all lines that are not closed and not scheduled for removal
  for line_idx, line in enumerate(open_lines):
    if line_idx not in lines_to_remove:
      overlap_len = 1
      while overlap_len > 0:
        # iterate over all other lines (and start over again whenever we found an overlapping one)
        overlap_len = 0
        for other_idx, other_line in enumerate(open_lines):
          if other_idx != line_idx and other_idx not in lines_to_remove:
            # special case: the end point of the current line equals the start point of the other line
            if (other_line[0][0] - line[-1][0])**2 + (other_line[0][1] - line[-1][1])**2 < equal_dist**2:
              line.pop() # remove last point
              line.extend(other_line) # append all points from the other line
              lines_to_remove.add(other_idx)
              overlap_len = 1 # ensure that we continue with that line
              break
            # iterate through all points, except start and end point
            for lx, ly in line[1:-1]:
              if overlap_len + 2 > len(other_line):
                overlap_len = 0
                break
              # get the other line's first point after the overlap
              ox, oy = other_line[overlap_len + 1]
              # check whether these two are almost equal, in which case we have overlap
              if (ox - lx)**2 + (oy - ly)**2 < equal_dist**2:
                overlap_len += 1
              else:
                overlap_len = 0
            # once at the end of the current line, merge if at least the last segment overlapped the other line
            if overlap_len >= 2:
              line.pop() # remove last point
              line.extend(other_line[overlap_len+1:]) # append all points from other line after the overlap
              lines_to_remove.add(other_idx)
              break
            overlap_len = 0
  # return lines that are not scheduled for removal
  for line_idx, line in enumerate(open_lines):
    if line_idx not in lines_to_remove:
      yield line
