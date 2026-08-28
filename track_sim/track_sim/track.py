import math

import numpy as np


class Track:
    """Geometry shared by the renderer, collision check and pseudo-LiDAR."""

    def __init__(self, control_points, samples_per_segment=18):
        self.centerline = self._sample_centerline(control_points, samples_per_segment)
        self.left_boundary, self.right_boundary = self._build_boundaries()
        self.wall_segments = self._build_wall_segments()

    @staticmethod
    def _sample_centerline(control_points, samples_per_segment):
        values = np.asarray(control_points, dtype=float)
        sampled = []

        # Catmull-Rom provides broad, smooth curves while keeping the CSV easy to edit.
        for index in range(len(values) - 1):
            p0 = values[max(0, index - 1)]
            p1 = values[index]
            p2 = values[index + 1]
            p3 = values[min(len(values) - 1, index + 2)]
            for step in range(samples_per_segment):
                t = step / samples_per_segment
                t2 = t * t
                t3 = t2 * t
                point = 0.5 * (
                    2.0 * p1
                    + (-p0 + p2) * t
                    + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
                    + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
                )
                sampled.append(point)
        sampled.append(values[-1])
        return np.asarray(sampled)

    def _build_boundaries(self):
        left = []
        right = []
        centers = self.centerline[:, :2]

        for index, point in enumerate(self.centerline):
            previous = centers[max(0, index - 1)]
            following = centers[min(len(centers) - 1, index + 1)]
            tangent = following - previous
            length = np.linalg.norm(tangent)
            if length == 0.0:
                normal = np.array([0.0, 1.0])
            else:
                normal = np.array([-tangent[1], tangent[0]]) / length
            half_width = point[2] / 2.0
            left.append(point[:2] + normal * half_width)
            right.append(point[:2] - normal * half_width)

        return np.asarray(left), np.asarray(right)

    def _build_wall_segments(self):
        segments = []
        for boundary in (self.left_boundary, self.right_boundary):
            segments.extend(zip(boundary[:-1], boundary[1:]))
        segments.append((self.left_boundary[0], self.right_boundary[0]))
        segments.append((self.left_boundary[-1], self.right_boundary[-1]))
        return segments

    def distance_and_width(self, x, y):
        """Return distance to the closest centerline segment and local track width."""
        position = np.array([x, y], dtype=float)
        closest_distance = math.inf
        closest_width = self.centerline[0, 2]

        for index in range(len(self.centerline) - 1):
            start = self.centerline[index, :2]
            end = self.centerline[index + 1, :2]
            segment = end - start
            squared_length = float(np.dot(segment, segment))
            if squared_length == 0.0:
                fraction = 0.0
            else:
                fraction = float(np.clip(np.dot(position - start, segment) / squared_length, 0.0, 1.0))
            nearest = start + fraction * segment
            distance = float(np.linalg.norm(position - nearest))
            if distance < closest_distance:
                closest_distance = distance
                start_width = self.centerline[index, 2]
                end_width = self.centerline[index + 1, 2]
                closest_width = start_width + fraction * (end_width - start_width)

        return closest_distance, closest_width

    def contains_vehicle(self, x, y, radius):
        distance, width = self.distance_and_width(x, y)
        return distance + radius <= width / 2.0

    def cast_ray(self, origin_x, origin_y, angle, max_range):
        origin = np.array([origin_x, origin_y], dtype=float)
        direction = np.array([math.cos(angle), math.sin(angle)], dtype=float)
        nearest = max_range

        for start, end in self.wall_segments:
            wall = end - start
            denominator = self._cross(direction, wall)
            if abs(denominator) < 1e-9:
                continue
            offset = start - origin
            ray_distance = self._cross(offset, wall) / denominator
            wall_fraction = self._cross(offset, direction) / denominator
            if 0.0 <= ray_distance <= nearest and 0.0 <= wall_fraction <= 1.0:
                nearest = ray_distance

        return float(nearest)

    @staticmethod
    def _cross(first, second):
        return first[0] * second[1] - first[1] * second[0]

    @property
    def bounds(self):
        walls = np.vstack((self.left_boundary, self.right_boundary))
        return (
            float(walls[:, 0].min()),
            float(walls[:, 0].max()),
            float(walls[:, 1].min()),
            float(walls[:, 1].max()),
        )
