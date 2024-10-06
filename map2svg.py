import json
import math
import tilemap
from geometry_utils import convex_hull, dissolve_lines, polygonize_clipped_lines

map_config = None
with open('map_config.json') as cfg:
  map_config = json.load(cfg)

viewport = (
  (map_config['viewport'][0], map_config['viewport'][1]),
  (map_config['viewport'][2], map_config['viewport'][3])
)

svg_scale = 1 / 1000
svg_size  = (
  (viewport[1][0] - viewport[0][0]) * svg_scale,
  (viewport[1][1] - viewport[0][1]) * svg_scale
)

with open('test.svg', 'w') as svg:
  svg.write(f"<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n")

  svg_attr = ' '.join(f"{key}=\"{val}\"" for key, val in map_config.get('attributes', {}).items())
  svg.write(f"<svg width=\"{svg_size[0]}\" height=\"{svg_size[1]}\" xmlns=\"http://www.w3.org/2000/svg\" {svg_attr}>\n")

  for src in map_config['sources']:
    print(f"Data source: {src['url']}")
    tmap    = tilemap.VectorTileMap(src['url'])
    filters = {}
    for grp in src['groups']:
      filters[grp['layer']] = filters.get(grp['layer'], set()) | set([(flt[0], flt[1], flt[2]) for flt in grp['filters']])
    for layer in filters:
      if len(filters[layer]) == 0:
        filters[layer] = None

    # query shapes and store those of same type, layer, and with same tags in one list
    shapes = {}
    for shape_type, coords, layer_name, tags, tile_box in tmap.query_shapes(src['zoom'], viewport, filters):
      shape_class = (shape_type, layer_name, tuple(tags.items()))
      shapes[shape_class] = shapes.get(shape_class, []) + [(coords, tile_box)]

    for shape_class in shapes:
      shape_type, layer_name, tags = shape_class
      print(f"  - shapes of type {shape_type} from layer {layer_name} with tags {dict(tags)}: {len(shapes[shape_class])} of which {sum(1 for coords, _ in shapes[shape_class] if coords[0] == coords[-1])} are closed")

    for grp in src['groups']:
      grp_attr = ' '.join(f"{key}=\"{val}\"" for key, val in grp.get('attributes', {}).items())
      svg.write(f"  <g {grp_attr}>\n")

      for shape_class in shapes:
        if shape_class[1] == grp['layer']:
          tags    = dict(shape_class[2])
          filters = grp['filters']
          if len(filters) == 0:
            filters = [('==', 0, 0)] # dummy filter in case no filters are used
          for flt in filters:
            op, val1, val2 = flt[0], flt[1], flt[2]
            if val1 in tags:
              val1 = tags[val1]
            if val2 in tags:
              val2 = tags[val2]
            if eval(f"{val1} {op} {val2}"):
              if 'polygonize' in grp.get('processing', {}):
                args = grp['processing']['polygonize']
                tile_shapes = {}
                for coords, tile_box in shapes[shape_class]:
                  tile_shapes[tile_box] = tile_shapes.get(tile_box, []) + [coords]
                shapes[shape_class] = [
                  (coords, tile_box)
                  for tile_box, shapes in tile_shapes.items()
                  for coords in polygonize_clipped_lines(shapes, tile_box, args.get("corner_outset", 1.))
                ]
                #shapes[shape_class] = [(polygonize_clipped_lines(coords, tile_box, args.get("corner_margin", 1.)), tile_box) for coords, tile_box in shapes[shape_class]]

              shape_list = [coords for coords, _ in shapes[shape_class]]

              for proc, args in grp.get('processing', {}).items():

                if proc == 'dissolve_lines' and shape_class[0] == 2:
                  shape_list = dissolve_lines(shape_list)

                if proc == 'remove_small_shapes' and shape_class[0] in (2, 3):
                  large_shapes = []
                  for shape in shape_list:
                    hull = convex_hull(shape)
                    # a shape is considered small if its mean width (the perimter of its convex hull divided by pi) is below a threshold
                    if sum(
                      math.sqrt((x1 - x0)**2 + (y1 - y0)**2) for (x0, y0), (x1, y1) in zip(hull, hull[1:] + hull[:1])
                    ) / math.pi >= args.get("mean_width", 1000.):
                      large_shapes.append(shape)
                  shape_list = large_shapes

                if proc == 'coord_fir_filter' and shape_class[0] in (2, 3):
                  coeff           = args['coefficients']
                  filter_order    = len(coeff)
                  filtered_shapes = []
                  for shape in shape_list:
                    if len(shape) > filter_order:
                      filtered_shape = []
                      # last point is skipped for feature type 2 because it is assumed to be equal to first point
                      window_x       = [x for x, y in (shape[-filter_order:] if shape_class[0] == 3 else shape[-filter_order-1:-1])]
                      window_y       = [y for x, y in (shape[-filter_order:] if shape_class[0] == 3 else shape[-filter_order-1:-1])]
                      for next_pt in shape:
                        window_x.pop(0)
                        window_y.pop(0)
                        window_x.append(next_pt[0])
                        window_y.append(next_pt[1])
                        filtered_shape.append((
                          sum(x * fval for x, fval in zip(window_x, coeff)),
                          sum(y * fval for y, fval in zip(window_y, coeff))
                        ))
                      # rotate points of filtered shape back into position for lines (for polygons it does not matter)
                      if shape_class[0] == 2:
                        end_len = filter_order // 2
                        filtered_shape = shape[0:end_len] + filtered_shape[2*end_len:] + shape[-end_len:]
                        assert len(filtered_shape) == len(shape)
                    else:
                      filtered_shape = shape
                    filtered_shapes.append(filtered_shape)

              for shape in shape_list:
                coords = [f"{(x - viewport[0][0]) * svg_scale:.3f} {(y - viewport[0][1]) * svg_scale:.3f}" for x, y in shape]
                if shape_class[0] == 2:
                  svg.write(f"    <path d=\"M {' L '.join(coords)}\" style=\"fill:none;stroke:{grp['colour']}\" />\n")
                if shape_class[0] == 3:
                  svg.write(f"    <path d=\"M {' L '.join(coords)} Z\" style=\"fill:{grp['colour']};stroke:none\" />\n")

      svg.write(f"  </g>\n")

  svg.write("</svg>\n")
