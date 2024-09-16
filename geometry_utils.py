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


def close_point_pairs(points_A, points_B, margin=1.):
  points_A      = list(enumerate(points_A))
  points_B      = list(enumerate(points_B))
  # sort the points in A and B by ascending x coordinate
  points_A.sort(key=lambda ept: ept[1][0])
  points_B.sort(key=lambda ept: ept[1][0])
  points_A      = iter(points_A)
  points_B      = iter(points_B)
  close_pts_A_x = SortedList(key=lambda ept: ept[1][0]) # close points from A sorted by x coordinate
  close_pts_A_y = SortedList(key=lambda ept: ept[1][1]) # close points from A sorted by y coordinate
  close_pts_B_x = SortedList(key=lambda ept: ept[1][0]) # close points from B sorted by x coordinate
  close_pts_B_y = SortedList(key=lambda ept: ept[1][1]) # close points from B sorted by y coordinate
  # repeatedly select the point with lowest x coordinate (either from A or B) to check next
  idx_A, pt_A   = next(points_A)
  idx_B, pt_B   = next(points_B)
  while pt_A is not None or pt_B is not None:
    if pt_A is not None and (pt_B is None or pt_A[0] < pt_B[0]):
      # update lists of close points in B by removing all those whose x coordinate is out of range
      for _ in range(close_pts_B_x.bisect_key_right(pt_A[0] - margin)):
        close_pts_B_y.remove(close_pts_B_x.pop(0))
      # find points from B that are close to the current point from A
      for close_idx, close_pt in close_pts_B_y.irange_key(pt_A[1] - margin, pt_A[1] + margin):
        if (pt_A[0] - close_pt[0])**2 + (pt_A[1] - close_pt[1])**2 < margin**2:
          yield (idx_A, close_idx)
      # add the current point from A to the lists of close points and fetch the next point from A
      close_pts_A_x.add((idx_A, pt_A))
      close_pts_A_y.add((idx_A, pt_A))
      idx_A, pt_A = next(points_A, (None, None))
    elif pt_B is not None:
      # update lists of close points in A by removing all those whose x coordinate is out of range
      for _ in range(close_pts_A_x.bisect_key_right(pt_B[0] - margin)):
        close_pts_A_y.remove(close_pts_A_x.pop(0))
      # find points from A that are close to the current point from B
      for close_idx, close_pt in close_pts_A_y.irange_key(pt_B[1] - margin, pt_B[1] + margin):
        if (pt_B[0] - close_pt[0])**2 + (pt_B[1] - close_pt[1])**2 < margin**2:
          yield (close_idx, idx_B)
      # add the current point from B to the lists of close points and fetch the next point from B
      close_pts_B_x.add((idx_B, pt_B))
      close_pts_B_y.add((idx_B, pt_B))
      idx_B, pt_B = next(points_B, (None, None))


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
  # sort the box boundaries by x coordinate
  bounds_x.sort(key=lambda bound: bound[0])
  # return indices of close line pairs (lines with overlapping bounding boxes)
  active_boxes_intervals = SortedList(key=lambda bound_y: bound_y[0])
  for _, start, box_idx, range_y in bounds_x:
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
  # iterate through pairs of lines that are close to eachother and remember connections and overlaps
  covered_lines = []
  connections   = []
  overlaps      = []
  pair_cnt      = 0
  for idx1, idx2 in close_line_pairs(open_lines, equal_dist):
    assert idx1 != idx2
    pair_cnt += 1
    # get lines
    line1 = open_lines[idx1]
    line2 = open_lines[idx2]
    if line1 is not None and line2 is not None:
      # special case: the end point of one line equals the start point of the other line
      if (line2[0][0] - line1[-1][0])**2 + (line2[0][1] - line1[-1][1])**2 < equal_dist**2:
        connections.append((idx1, idx2))
      if (line1[0][0] - line2[-1][0])**2 + (line1[0][1] - line2[-1][1])**2 < equal_dist**2:
        connections.append((idx2, idx1))
      # get pairs of close points between those lines
      pairs = set(close_point_pairs(line1, line2, equal_dist))
      # search for runs of close points at the start of one and the end of the other line
      overlap_len_l1_l2 = 0     # length of longest overlap at end of line 1 and start of line 2
      overlap_len_l2_l1 = 0     # length of longest overlap at start of line 1 and end of line 2
      l1_covered        = False # line 1 is completely covered by line 2
      l2_covered        = False # line 2 is completely covered by line 1
      for pair in pairs:
        run = [pair]
        while True:
          pair = (pair[0] + 1, pair[1] + 1)
          if pair not in pairs:
            break
          run.append(pair)
        # remember overlap for runs of at least two points at start of one and end of the other line
        if len(run) > 1:
          if run[-1][0] >= len(line1) - 2 and run[0][1] < 2:
            overlap_len       = len(run) + run[0][1] + (len(line1) - 1 - run[-1][0])
            overlap_len_l1_l2 = max(overlap_len_l1_l2, overlap_len)
          if run[-1][1] >= len(line2) - 2 and run[0][0] < 2:
            overlap_len       = len(run) + run[0][0] + (len(line2) - 1 - run[-1][1])
            overlap_len_l2_l1 = max(overlap_len_l2_l1, overlap_len)
        # check whether the run completely covers one of the lines
        l1_covered = len(run) + 2 >= len(line1) and run[0][0] < 2 and run[-1][0] >= len(line1) - 2
        l2_covered = len(run) + 2 >= len(line2) and run[0][1] < 2 and run[-1][1] >= len(line2) - 2
      # remember lines that are completely covered by another
      if l1_covered:
        covered_lines.append(idx1)
      if l2_covered:
        covered_lines.append(idx2)
      # remember overlaps of at least 3 points
      if overlap_len_l1_l2 >= 3:
        overlaps.append((overlap_len_l1_l2, idx1, idx2))
      if overlap_len_l2_l1 >= 3:
        overlaps.append((overlap_len_l2_l1, idx2, idx1))
  print(f"iterated through {pair_cnt} pairs of lines and found {len(connections)} connections and {len(overlaps)} overlaps")
  # eliminate lines that are completely covered by another
  for idx in covered_lines:
    open_lines[idx] = None
  # sort overlaps from largest to smallest
  overlaps.sort(key=lambda overlap: overlap[0], reverse=True)
  # merge connected lines first, then those with overlaps
  line_end_redirects  = {}
  for overlap_len, idx1, idx2 in [(1, idx1, idx2) for idx1, idx2 in connections] + overlaps:
    orig_idx1 = idx1
    # if line 1 has been merged into another line, then its end is now the end of that other line
    idx1  = line_end_redirects.get(idx1, idx1)
    line1 = open_lines[idx1] if idx1 is not None else None
    line2 = open_lines[idx2]
    assert line2 is None or idx2 not in line_end_redirects or line_end_redirects[idx2] is None
    if idx1 == idx2:
      # line overlaps itself: close it and yield it (unless it would have 0 area after closing)
      if line1 is not None and len(line1) > overlap_len + 2:
        yield line1[:-overlap_len] + [line1[0]]
      open_lines[idx1] = None
    elif line1 is not None and line2 is not None:
      # extend line 1 with line 2
      line1.pop()
      line1.extend(line2[overlap_len-1:])
      # remove line 2
      open_lines[idx2] = None
      # the end of line 1 is now gone, hence all redirects to it are replaced with empty redirects
      line_end_redirects.update(
        [(old_idx, None) for old_idx, new_idx in line_end_redirects.items() if new_idx == idx1]
      )
      # an empty redirect is added for line 1 itself, as it might still be referenced in other overlaps
      line_end_redirects[idx1] = None
      # the former end of line 2 is now the end of line 1
      line_end_redirects[idx2] = idx1
      # update all redirects that currently point to line 2 to point to line 1 instead
      line_end_redirects.update(
        [(old_idx, idx1) for old_idx, new_idx in line_end_redirects.items() if new_idx == idx2]
      )
  # return remaining lines
  for line in open_lines:
    if line is not None:
      yield line
