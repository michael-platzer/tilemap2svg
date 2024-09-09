
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


def close_line_pairs(lines, margin=1.):
  # compute bounding boxes (with margin) for all lines
  bboxes = [(
    (min(x for x, y in line) - margin, min(y for x, y in line) - margin),
    (max(x for x, y in line)         , max(y for x, y in line)         )
  ) for line in lines]
  # list of bounding box boundaries, once along x and once along y dimension
  bounds_x, bounds_y = (
    [(x0, False, idx) for idx, ((x0, y0), (x1, y1)) in enumerate(bboxes)] + # start coordinates
    [(x1, True , idx) for idx, ((x0, y0), (x1, y1)) in enumerate(bboxes)]   # end coordinates
  ), (
    [(y0, False, idx) for idx, ((x0, y0), (x1, y1)) in enumerate(bboxes)] + # start coordinates
    [(y1, True , idx) for idx, ((x0, y0), (x1, y1)) in enumerate(bboxes)]   # end coordinates
  )
  bounds_x = sorted(bounds_x, key=lambda bound: bound[0])
  bounds_y = sorted(bounds_y, key=lambda bound: bound[0])
  # compute sets of overlapping pairs once along x and once along y dimension
  pairs_x, pairs_y = set(), set()
  active_boxes = set()
  for _, end, box_idx in bounds_x:
    if end:
      active_boxes.remove(box_idx)
    else:
      pairs_x |= {((box_idx, idx) if box_idx < idx else (idx, box_idx)) for idx in active_boxes}
      active_boxes.add(box_idx)
  assert len(active_boxes) == 0
  for _, end, box_idx in bounds_y:
    if end:
      active_boxes.remove(box_idx)
    else:
      pairs_y |= {((box_idx, idx) if box_idx < idx else (idx, box_idx)) for idx in active_boxes}
      active_boxes.add(box_idx)
  assert len(active_boxes) == 0
  # pairs appearing in both sets are those for which bounding boxes effectively do overlap
  return pairs_x & pairs_y


def dissolve_lines(lines, equal_dist=1.):
  # immediatly return closed lines, retain only open lines for further processing
  open_lines = []
  for line in lines:
    if line[0] == line[-1]:
      yield line
    else:
      open_lines.append(line)
  # get pairs of lines that are close to eachother
  pairs = close_line_pairs(open_lines, equal_dist)
  print(f"iterating through {len(pairs)} pairs to dissolve lines")
  # re-organize open lines into a dictionary for faster indexing and removal
  open_lines = dict(enumerate(open_lines))
  # iterate through the pairs, with an option to redirect indices to a new line
  redirect_indices = {}
  for idx1, idx2 in pairs:
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
        # iterate through all points, except start and end point
        overlap_len = 0
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
          lines_merged = True
      # remove the other line and cleanup if it was merged
      if lines_merged:
        open_lines.pop(other_idx)
        redirect_indices[other_idx] = line_idx
        # update all redirects that currently point to the other line
        redirects_to_other = set()
        for old_idx, new_idx in redirect_indices.items():
          if new_idx == other_idx:
            redirects_to_other.add(old_idx)
        redirect_indices.update((idx, line_idx) for idx in redirects_to_other)
        break
  # return remaining lines
  for line_idx, line in open_lines.items():
    yield line
