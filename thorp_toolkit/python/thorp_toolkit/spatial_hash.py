import math
from collections import namedtuple


class Rect(namedtuple('BaseRect', 'l r b t')):
    """2D rectangle class."""

    @classmethod
    def from_blwh(cls, bl, w, h):
        """Construct a Rect from its bottom left and dimensions."""
        l, b = bl
        return Rect(
            l,
            l + w,
            b,
            b + h
        )

    @classmethod
    def from_cwh(cls, c, w, h):
        """Construct a Rect from its center and dimensions."""
        w2 = w * 0.5
        h2 = h * 0.5
        return cls(
            c.x - w2,
            c.x + w2,
            c.y - h2,
            c.y + h2
        )

    @classmethod
    def from_points(cls, p1, p2):
        """Construct the smallest Rect that contains the points p1 and p2."""
        x1, y1 = p1
        x2, y2 = p2
        if x2 < x1:
            x1, x2 = x2, x1

        if y2 < y1:
            y1, y2 = y2, y1

        return cls(
            x1,
            x2,
            y1,
            y2
        )

    @classmethod
    def as_bounding(cls, points):
        """Construct a Rect as the bounds of a sequence of points.

        :param points: An iterable of the points to bound.

        """
        xs, ys = zip(*points)
        l, r = min(xs), max(xs)
        b, t = min(ys), max(ys)
        return cls(l, r, b, t)


class SpatialHash(object):
    """ Very simple spatial hash handling rectangular, non-rotated objects """

    def __init__(self, cell_size=1.0):
        self.cell_size = float(cell_size)
        self.d = {}

    def _add(self, cell_coord, o):
        """Add the object o to the cell at cell_coord."""
        self.d.setdefault(cell_coord, []).append(o)

    def _cells_for_rect(self, r):
        """Return a set of the cells into which r extends."""
        cells = set()
        cy = math.floor(r.b / self.cell_size)
        while (cy * self.cell_size) <= r.t:
            cx = math.floor(r.l / self.cell_size)
            while (cx * self.cell_size) <= r.r:
                cells.add((int(cx), int(cy)))
                cx += 1.0
            cy += 1.0
        return cells

    def add_rect(self, r, obj):
        """Add an object obj with bounds r."""
        cells = self._cells_for_rect(r)
        for c in cells:
            self._add(c, obj)

    def _remove(self, cell_coord, o):
        """Remove the object o from the cell at cell_coord."""
        cell = self.d[cell_coord]
        cell.remove(o)

        # Delete the cell from the hash if it is empty.
        if not cell:
            del(self.d[cell_coord])

    def remove_rect(self, r, obj):
        """Remove an object obj which had bounds r."""
        cells = self._cells_for_rect(r)
        for c in cells:
            self._remove(c, obj)

    def remove(self, obj):
        """Remove entirely an object."""
        for obj_list in self.d.values():
            while obj in obj_list:
                obj_list.remove(obj)

    def query(self, r):
        """Get a set of all objects that potentially intersect r."""
        cells = self._cells_for_rect(r)
        seen = []
        for c in cells:
            for hit in self.d.get(c, []):
                if hit not in seen:
                    seen.append(hit)
        return seen

    def count(self):
        """Count objects stored in the map."""
        return len(set(sum(self.d.values(), [])))

    def content(self):
        """Return all objects stored in the map."""
        return set().union(*self.d.values())
