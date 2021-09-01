from Point import Point
import Point
from shapely import geometry
from auxiliary_functions import distance_between_point_and_segment


class Polygon:
    def __init__(self, vertices):
        self.vertices = vertices  # type Point[]

    def get_middle_of_edge(self, edge_num):
        return Point((self.vertices[edge_num].x + self.vertices[edge_num + 1].x) / 2,
                     (self.vertices[edge_num].y + self.vertices[edge_num + 1].y) / 2, 0)

    def get_area(self):
        coords = []
        for vertex in self.vertices:
            coords.append((vertex.x, vertex.y))
        if len(coords) <= 1:
            return 0.0
        poly = geometry.Polygon(coords)
        return poly.area

    def get_perimeter(self):
        sum = 0
        for vertex in range(self.get_vertices_length()):
            sum += self.vertices[vertex].distance_from_other2D(
                self.vertices[(vertex + 1) % self.get_vertices_length()])
        return sum

    def differ(self, other):
        if min(self.get_area(), other.get_area()) == 0:
            return self.get_area() != other.get_area()
        return max(self.get_area(), other.get_area()) / min(self.get_area(), other.get_area()) <= 1.05  # change to change area difference percent

    def get_vertices_length(self):
        return len(self.vertices)

    def is_inside(self, point):
        iCrossings = 0
        for (i, vertex) in enumerate(self.vertices):
            nextVertex = self.vertices[(i + 1) % len(self.vertices)]
            fSegmentStartX = vertex.x
            fSegmentStartY = vertex.y
            fSegmentEndX = nextVertex.x
            fSegmentEndY = nextVertex.y
            fTestPointX = point.x
            fTestPointY = point.y
            if ((fSegmentStartX < fTestPointX) and (fTestPointX < fSegmentEndX)) or ((fSegmentStartX > fTestPointX) and (fTestPointX > fSegmentEndX)):
                # Check if the segment is crossed in the Y axis as well (the point may lie below the segment).
                fT = (fTestPointX - fSegmentEndX) / (fSegmentStartX - fSegmentEndX)
                fCrossingY = ((fT * fSegmentStartY) + ((1 - fT) * fSegmentEndY))
                if fCrossingY >= fTestPointY:
                    iCrossings += 1

            if (fSegmentStartX == fTestPointX) and (fSegmentStartY <= fTestPointY):
                if fSegmentStartY == fTestPointY:
                    iCrossings += 1
                if fSegmentEndX == fTestPointX:
                    if ((fSegmentStartY <= fTestPointY) and (fTestPointY <= fSegmentEndY)) or ((fSegmentStartY >= fTestPointY) and (fTestPointY >= fSegmentEndY)):
                        iCrossings += 1
                elif fSegmentEndX > fTestPointX:
                    iCrossings += 1
                if self.vertices[(i-1)].x > fTestPointX:
                    iCrossings += 1
        iRemainder = iCrossings % 2
        if iRemainder != 0:
            return True
        else:
            return False

    def get_constant_length(self):
        perimeter = self.get_perimeter()
        perimeter /= self.get_vertices_length()  # get the average edge length
        perimeter /= 4  # ??? run in every 1/4th of edge length

    def filter_insiders(self, points):
        good_points = []
        for point in points:
            if not self.is_inside(point):
                good_points.append(point)
        return good_points

    def filter_epsilon(self, slices, eps):
        good_points = []
        for (i, slice) in enumerate(slices):
            vertex = self.vertices[i]
            nextVertex = self.vertices[(i + 1) % len(self.vertices)]
            for point in slice:
                dist = distance_between_point_and_segment(point.x, point.y, vertex.x, vertex.y, nextVertex.x, nextVertex.y)
                if dist > eps:
                    good_points.append(point)
        return good_points
