"""Conditional planar bearing covariance for *diagnostics*, never Nav2.

Inputs require independently measured angular variance and world-frame sensor
origin 2x2 covariance. Assuming a stationary emitter, locally linearized,
independent Gaussian ray errors, estimate the inverse normal matrix of
cross-track residuals. None means insufficient/unknown/unsafe evidence.
This is NOT calibrated 3-D source position covariance, nor a new audio DSP.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PlanarRay:
    stamp_ns: int
    origin: tuple
    direction: tuple
    angular_variance_rad2: float
    # World-frame origin pose covariance (xx, xy, yx, yy), m^2; required.
    origin_covariance: tuple


@dataclass(frozen=True)
class PlanarDiagnostic:
    covariance_xx_m2: float
    covariance_xy_m2: float
    covariance_yy_m2: float
    major_std_m: float
    information_condition_number: float
    maximum_normalized_residual: float
    supporting_observations: int


def _finite(values):
    return all(type(x) in (int,float) and math.isfinite(x) for x in values)


def conditional_planar_covariance(point, rays, *,
                                  minimum_baseline_m=0.5,
                                  minimum_spacing_ns=100_000_000,
                                  maximum_window_ns=5_000_000_000,
                                  maximum_condition_number=100.0,
                                  maximum_major_std_m=1.0,
                                  maximum_normalized_residual=3.0):
    """Return locally linearized covariance, or None on unknown inputs.

    n_i = (-dy,dx), r_i = n_i dot (point-origin)
    var(r_i) ~= range_i^2 var(bearing angle) + n_i^T Sigma_origin_i n_i.
    H = sum (n_i n_i^T / var(r_i)); covariance ~= inverse(H).
    Correlated pose error, moving emitter, reflections and extrinsic
    uncertainty NOT supplied in rays invalidate this interpretation.
    """
    limits = (minimum_baseline_m,maximum_condition_number,
              maximum_major_std_m,maximum_normalized_residual)
    if (not isinstance(rays,(tuple,list)) or not 3 <= len(rays) <= 32
            or not isinstance(point,(tuple,list)) or len(point)!=2
            or not _finite(point) or not _finite(limits)
            or minimum_baseline_m <= 0
            or maximum_condition_number < 1
            or maximum_major_std_m <= 0
            or maximum_normalized_residual <= 0
            or type(minimum_spacing_ns) is not int or minimum_spacing_ns <= 0
            or type(maximum_window_ns) is not int
            or maximum_window_ns < 2*minimum_spacing_ns):
        return None

    if (any(type(r.stamp_ns) is not int or r.stamp_ns <= 0 for r in rays)
            or any(b.stamp_ns-a.stamp_ns < minimum_spacing_ns
                   for a,b in zip(rays,rays[1:]))
            or rays[-1].stamp_ns-rays[0].stamp_ns > maximum_window_ns):
        return None
    hxx=hxy=hyy=worst=0.0
    positions=[]
    for ray in rays:
        if (not isinstance(ray.origin,(tuple,list)) or len(ray.origin)!=2
                or not isinstance(ray.direction,(tuple,list))
                or len(ray.direction)!=2
                or not isinstance(ray.origin_covariance,(tuple,list))
                or len(ray.origin_covariance)!=4):
            return None
        values=(*ray.origin,*ray.direction,ray.angular_variance_rad2,
                *ray.origin_covariance)
        if not _finite(values):
            return None
        dx,dy=ray.direction
        if abs(dx*dx+dy*dy-1.0)>1e-3:
            return None
        if not 0 < ray.angular_variance_rad2 <= .3:
            return None
        a,b,c,d=ray.origin_covariance
        if a < 0 or d < 0 or abs(b-c)>1e-9 or a*d-b*c < -1e-12:
            return None
        nx,ny=-dy,dx
        vx=point[0]-ray.origin[0]
        vy=point[1]-ray.origin[1]
        distance_along_ray=vx*dx+vy*dy
        if distance_along_ray <= 0:
            return None
        # Exact distance is unknown for a hypothetical intersection; use
        # its conditional range and explicitly forbid presenting the result
        # as measured distance or validated source covariance.
        var=(distance_along_ray**2)*ray.angular_variance_rad2 + (
            nx*nx*a + nx*ny*(b+c) + ny*ny*d)
        if var <= 1e-15 or not math.isfinite(var):
            return None
        resid=abs(nx*vx+ny*vy)/math.sqrt(var)
        if not math.isfinite(resid) or resid > maximum_normalized_residual:
            return None
        worst=max(worst,resid)
        weight=1/var
        hxx+=weight*nx*nx
        hxy+=weight*nx*ny
        hyy+=weight*ny*ny
        positions.append(ray.origin)
    baseline=max(math.dist(a,b) for a in positions for b in positions)
    if baseline < minimum_baseline_m:
        return None
    trace=hxx+hyy
    delta=math.hypot(hxx-hyy,2*hxy)
    small=(trace-delta)/2
    large=(trace+delta)/2
    if not math.isfinite(small) or small <= 1e-10 or large/small > maximum_condition_number:
        return None
    major=1/math.sqrt(small)
    if major > maximum_major_std_m:
        return None
    determinant=hxx*hyy-hxy*hxy
    if determinant<=0 or not math.isfinite(determinant):
        return None
    return PlanarDiagnostic(hyy/determinant,-hxy/determinant,
                            hxx/determinant,major,large/small,
                            worst,len(rays))
