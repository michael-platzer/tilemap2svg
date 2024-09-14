from sortedcontainers import SortedList

def convex_hull(points):
  points = list(points) # ensure we have a list (and not an iterator)
  # if the line has more than 100 points, try to eliminate as many as possible first
  if len(points) > 100:
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


def close_line_pairs(lines, margin=1.):
  # compute bounding boxes (with margin) for all lines
  bboxes = [(
    (min(x for x, y in line) - margin, min(y for x, y in line) - margin),
    (max(x for x, y in line)         , max(y for x, y in line)         )
  ) for line in lines]
  # box boundaries in x direction, along with info about y extent of each box
  bounds_x = (
    [(x0, True , idx, (y0, y1)) for idx, ((x0, y0), (x1, y1)) in enumerate(bboxes)] + # start coords
    [(x1, False, idx, (y0, y1)) for idx, ((x0, y0), (x1, y1)) in enumerate(bboxes)]   # end coords
  )
  # return indices of close line pairs (lines with overlapping bounding boxes)
  active_boxes_intervals = SortedList(key=lambda bound_y: bound_y[0])
  for _, start, box_idx, range_y in sorted(bounds_x, key=lambda bound: bound[0]):
    if start:
      y0_idx            = active_boxes_intervals.bisect_key_left(range_y[0])
      y1_idx            = active_boxes_intervals.bisect_key_right(range_y[1])
      active_boxes_iter = active_boxes_intervals.islice(max(y0_idx - 1, 0), y1_idx + 1)
      pre_active_boxes  = next(active_boxes_iter)[2] if y0_idx > 0 else set()
      active_boxes_y0   = None
      overlapping_boxes = set()
      active_boxes      = set()
      for _ in range(y1_idx - y0_idx):
        active_boxes       = next(active_boxes_iter)[2]
        overlapping_boxes |= active_boxes
        if active_boxes_y0 is None:
          active_boxes_y0 = pre_active_boxes & active_boxes
        active_boxes.add(box_idx)
      active_boxes_y1 = active_boxes & next(active_boxes_iter, (None, None, set()))[2]
      active_boxes_intervals.add((range_y[0], box_idx, active_boxes_y0 | {box_idx} if active_boxes_y0 is not None else {box_idx}))
      active_boxes_intervals.add((range_y[1], box_idx, active_boxes_y1 | {box_idx}))
      for idx in overlapping_boxes:
        yield (box_idx, idx) if box_idx < idx else (idx, box_idx)
    else:
      active_boxes_y0, active_boxes_y1 = None, None
      for _, idx, active_boxes in active_boxes_intervals.irange_key(range_y[0], range_y[1]):
        active_boxes.discard(box_idx)
        if idx == box_idx:
          if active_boxes_y0 is None:
            active_boxes_y0 = active_boxes
          else:
            active_boxes_y1 = active_boxes
      active_boxes_intervals.remove((range_y[0], box_idx, active_boxes_y0))
      active_boxes_intervals.remove((range_y[1], box_idx, active_boxes_y1))
  assert len(active_boxes_intervals) == 0


def dissolve_lines(lines, equal_dist=1.):
  # immediatly return closed lines, retain only open lines for further processing
  open_lines = []
  for line in lines:
    if (line[0][0] - line[-1][0])**2 + (line[0][1] - line[-1][1])**2 < equal_dist**2:
      yield line
    else:
      open_lines.append(line)
  # iterate through pairs of lines that are close to eachother, with an option to redirect indices to a new line
  redirect_indices = {}
  pair_cnt = 0
  for idx1, idx2 in close_line_pairs(open_lines, equal_dist):
    pair_cnt += 1
    # translate indices in case they have been redirected
    idx1 = redirect_indices.get(idx1, idx1)
    idx2 = redirect_indices.get(idx2, idx2)
    # get lines
    line1 = open_lines[idx1]
    line2 = open_lines[idx2]
    # check for overlaps between both lines
    for (line_idx, line), (other_idx, other_line) in (
      [((idx1, line1), (idx2, line2)), ((idx2, line2), (idx1, line1))] if idx1 != idx2 else []
    ):
      lines_merged = False
      # special case: the end point of the current line equals the start point of the other line
      if (other_line[0][0] - line[-1][0])**2 + (other_line[0][1] - line[-1][1])**2 < equal_dist**2:
        line.pop() # remove last point
        line.extend(other_line) # append all points from the other line
        lines_merged = True
      else:
        for overlap_len in range(3, len(line)):
          overlap = True
          # compare all supposedly overlapping points for the given overlap amount
          for (lx, ly), (ox, oy) in zip(line[-overlap_len:-1], other_line[1:]):
            # check whether these two points are almost equal
            if (ox - lx)**2 + (oy - ly)**2 > equal_dist**2:
              overlap = False
              break
          if overlap:
            # the other line might be shorter than the overlap, in which case it can be completely removed
            if overlap_len < len(other_line):
              line.pop() # remove last point of current line
              line.extend(other_line[overlap_len:]) # append all points from other line after the overlap
            lines_merged = True
            break
      # remove the other line and cleanup if it was merged
      if lines_merged:
        open_lines[other_idx] = None
        redirect_indices[other_idx] = line_idx
        # update all redirects that currently point to the other line
        redirects_to_other = set()
        for old_idx, new_idx in redirect_indices.items():
          if new_idx == other_idx:
            redirects_to_other.add(old_idx)
        redirect_indices.update((idx, line_idx) for idx in redirects_to_other)
        break
  # return remaining lines
  for line in open_lines:
    if line is not None:
      yield line
