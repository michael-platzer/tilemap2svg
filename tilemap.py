
from urllib.parse import urljoin
import requests
import re
import time
import logging

import vector_tile_pb2

class VectorTileMap:
    def __init__(self, index_url, style_url=None, logger=None):
        self.logger = logging.getLogger(
            VectorTileMap.__qualname__ if logger is None else logger
        )

        # load index
        req = requests.get(index_url)
        assert req.status_code == 200, f"{index_url}: status {req.status_code}"
        self.index = req.json()

        # load style
        self.style = None
        if style_url is None and 'defaultStyles' in self.index:
            style_url  = urljoin(index_url, self.index['defaultStyles'])
            style_url += '/root.json'
        if style_url is not None:
            req = requests.get(style_url)
            assert req.status_code == 200, f"{style_url}: status {req.status_code}"
            self.style = req.json()

        # layer description
        self.layer_desc = None
        if self.style is not None:
            self.layer_desc = self.style['layers']
        elif 'vector_layers' in self.index:
            self.layer_desc = self.index['vector_layers']

        # extract information from index
        self.tile_url = urljoin(index_url, self.index['tiles'][0])
        self.crs      = None
        self.orig     = (0, 0)
        self.size     = (1, 1)
        self.lods     = []
        if 'extent' in self.index:
            extent    = self.index['extent']
            self.orig = (min(extent[0], extent[1]), min(extent[0], extent[1]))
            self.size = (extent[2] - extent[0], extent[3] - extent[1])
        if 'minzoom' in self.index and 'maxzoom' in self.index:
            self.lods = [
                (zoom, max(self.size) / 2**zoom) for zoom in
                range(self.index['minzoom'], self.index['maxzoom'] + 1)
            ]
        if 'tileInfo' in self.index:
            refsys    = self.index['tileInfo']['spatialReference']
            self.crs  = refsys.get('latestWkid', refsys['wkid'])
            orig      = self.index['tileInfo']['origin']
            self.orig = (orig['x'], orig['y'])
            for lod in self.index['tileInfo']['lods']:
                #self.lods.append((lod['level'], lod['scale']))
                self.lods.append((lod['level'], lod['resolution'] * 512))


    def get_style_layers(self, zoom_level=None):
        for layer in self.layer_desc:
            zoom_range = layer.get('minzoom', 0), layer.get('maxzoom', 256)
            zoom_match = zoom_level is None or (
                zoom_level >= zoom_range[0] and zoom_level <= zoom_range[1]
            )
            if zoom_match:
                yield layer


    # convert style layers to filters (dictionary for filtering features)
    def get_style_filters(self, style_layer_patterns, zoom_level=None):
        filters = {}
        for layer in self.layer_desc:
            for pattern in style_layer_patterns:
                zoom_range = layer.get('minzoom', 0), layer.get('maxzoom', 256)
                zoom_match = zoom_level is None or (
                    zoom_level >= zoom_range[0] and zoom_level <= zoom_range[1]
                )
                if zoom_match and re.match(pattern, layer['id']):
                    source_layer = layer.get('source-layer', None)
                    layer_filter = layer.get('filter', None)
                    if source_layer is not None:
                        if layer_filter is None:
                            filters[source_layer] = None
                        else:
                            layer_filter = set([tuple(layer_filter)])
                            prev_filters = filters.get(source_layer, set())
                            if prev_filters is not None:
                                filters[source_layer] = prev_filters | layer_filter
        return filters


    # binary search to find locations of tiles (avoid trying all urls)
    def _get_tile_coords(self, lod_seq, view=None, coords=None):
        self.logger.info(f"probing tile coordinates for LOD {lod_seq[0][0]}")
        if coords is None:
            extent = 2**lod_seq[0][0]
            coords = [(x, y) for x in range(extent) for y in range(extent)]
        next_coords = []
        for cnt, (x, y) in enumerate(coords):
            # extent of this tile
            scale  = lod_seq[0][1]
            x0, y0 = (self.orig[0] +  x      * scale, self.orig[1] +  y      * scale)
            x1, y1 = (self.orig[0] + (x + 1) * scale, self.orig[1] + (y + 1) * scale)
            # probe tile if it overlaps view port
            if view is None or (view[0][0] < x1 and view[0][1] < y1 and
                                view[1][0] > x0 and view[1][1] > y0):
                self.logger.info(f"probing tile {cnt} of {len(coords)}")
                while True:
                    try:
                        req = requests.head(
                            self.tile_url.format(z=lod_seq[0][0], y=y, x=x)
                        )
                    except requests.exceptions.ConnectionError:
                        self.logger.info(f"connection error, trying again")
                        pass
                    else:
                        if req.status_code in [429, 500, 502, 503, 504]:
                            self.logger.info(f"server error, trying again")
                        else:
                            break
                if req.status_code == 200:
                    x, y = x * 2, y * 2
                    next_coords += [(x, y), (x + 1, y), (x, y + 1), (x + 1, y + 1)]
        if len(lod_seq) == 1:
            return next_coords
        return self._get_tile_coords(lod_seq[1:], view, next_coords)


    def _get_tiles(self, lod_idx, view=None, bufcnt=10):
        level, scale = self.lods[lod_idx]
        coords       = (
            self._get_tile_coords(self.lods[0:lod_idx], view) if lod_idx > 0 else
            [(x, y) for x in range(2**level) for y in range(2**level)]
        )
        tilebuf      = []
        for cnt, (x, y) in enumerate(coords):
            # extent of this tile
            x0, y0 = (self.orig[0] +  x      * scale, self.orig[1] +  y      * scale)
            x1, y1 = (self.orig[0] + (x + 1) * scale, self.orig[1] + (y + 1) * scale)
            # fetch tile if it overlaps view port
            if view is None or (view[0][0] < x1 and view[0][1] < y1 and
                                view[1][0] > x0 and view[1][1] > y0):
                self.logger.info(f"fetching tile {cnt} of {len(coords)}")
                while True:
                    try:
                        req = requests.get(self.tile_url.format(z=level, y=y, x=x))
                    except requests.exceptions.ConnectionError:
                        self.logger.info(f"connection error, trying again")
                        pass
                    else:
                        if req.status_code in [429, 500, 502, 503, 504]:
                            self.logger.info(f"server error, trying again")
                        else:
                            break
                    time.sleep(5)
                if req.status_code == 200:
                    tile = vector_tile_pb2.Tile()
                    tile.ParseFromString(req.content)
                    tilebuf.append((tile, (x, y)))
                if len(tilebuf) > bufcnt:
                    for tile in tilebuf:
                        yield tile
                    tilebuf = []
        for tile in tilebuf:
            yield tile


    def _query_features(self, lod_idx, view=None, filters=None):
        for tile, tile_pos in self._get_tiles(lod_idx, view):
            for layer in tile.layers:
                if filters is None or layer.name in filters:
                    for feature in layer.features:
                        # get tags for this feature
                        tags = {
                            layer.keys[key]: layer.values[val].ListFields()[0][1]
                            for key, val
                            in zip(feature.tags[::2], feature.tags[1::2])
                        }
                        if filters is None or filters[layer.name] is None:
                            yield (feature, tile_pos, layer.extent, layer.name, tags)
                        else:
                            # evaluate filters
                            for op, val1, val2 in filters[layer.name]:
                                if isinstance(val1, str) and val1 in tags:
                                    val1 = tags[val1]
                                if isinstance(val2, str) and val2 in tags:
                                    val2 = tags[val2]
                                if eval(f"{val1} {op} {val2}"):
                                    yield (feature, tile_pos, layer.extent, layer.name, tags)
                                    break


    def query_shapes(self, level, view=None, filters=None):
        lod_idx = next(idx for idx, lod in enumerate(self.lods) if lod[0] == level)
        for feature, tile_pos, layer_extent, layer_name, tags in self._query_features(lod_idx, view, filters):
            # origin and scale for this tile
            lod_scale  = self.lods[lod_idx][1]
            x0, y0     = (
                self.orig[0] + tile_pos[0] * lod_scale,
                self.orig[1] + tile_pos[1] * lod_scale
            )
            tile_box   = ((x0, y0), (x0 + lod_scale, y0 + lod_scale))
            tile_scale = lod_scale / layer_extent
            shape = None
            pos   = (0, 0)
            op    = None
            cnt   = 0
            for geometry in feature.geometry:
                if op is None:
                    op  = geometry & 0x7
                    cnt = (geometry >> 3) * 2
                else:
                    val = (geometry >> 1) ^ (-(geometry & 1))
                    if (cnt & 1) == 0:
                        pos = (pos[0] + val, pos[1])
                    else:
                        pos = (pos[0], pos[1] + val)
                        if op == 1:
                            # yield the current shape (if any)
                            if shape is not None:
                                yield (feature.type, shape, layer_name, tags, tile_box)
                            # start a new shape
                            shape = [(x0 + pos[0] * tile_scale, y0 + pos[1] * tile_scale)]
                        elif op == 2:
                            # append to the current line
                            shape.append((x0 + pos[0] * tile_scale, y0 + pos[1] * tile_scale))
                        else:
                            raise ValueError("invalid geometry command")
                    cnt -= 1
                if op == 7 or cnt == 0:
                    op = None
            # yield the last shape (if any)
            if shape is not None:
                yield (feature.type, shape, layer_name, tags, tile_box)
