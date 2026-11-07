"""M4 bearing triangulation primitive: geometry only, no ROS and no inferred range.

Two calibrated world-frame unit bearings, measured at distinct robot positions.
Returns None on insufficient baseline, nearly parallel rays, or behind-sensor
intersections. No claim of source identity or temporal association.
"""
import math


def triangulate_2d(p0, d0, p1, d1, *, minimum_baseline=0.25,
                   minimum_sine=0.1, maximum_ray_gap=0.5):
    values = (*p0, *d0, *p1, *d1, minimum_baseline, minimum_sine, maximum_ray_gap)
    if not all(math.isfinite(x) for x in values):
        return None
    if minimum_baseline <= 0 or not 0 < minimum_sine <= 1 or maximum_ray_gap < 0:
        return None
    def norm(v):
        return math.hypot(*v)
    if norm((p1[0]-p0[0], p1[1]-p0[1])) < minimum_baseline:
        return None
    if abs(norm(d0)-1) > 1e-3 or abs(norm(d1)-1) > 1e-3:
        return None
    def cross(a, b):
        return a[0]*b[1]-a[1]*b[0]
    delta = (p1[0]-p0[0], p1[1]-p0[1])
    denominator = cross(d0, d1)
    if abs(denominator) < minimum_sine:
        return None
    t0 = cross(delta, d1)/denominator
    t1 = cross(delta, d0)/denominator
    if t0 <= 0 or t1 <= 0:
        return None
    x0 = (p0[0]+t0*d0[0], p0[1]+t0*d0[1])
    x1 = (p1[0]+t1*d1[0], p1[1]+t1*d1[1])
    if norm((x0[0]-x1[0], x0[1]-x1[1])) > maximum_ray_gap:
        return None
    return ((x0[0]+x1[0])/2, (x0[1]+x1[1])/2)
