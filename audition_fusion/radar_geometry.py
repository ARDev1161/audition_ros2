"""M4-C radar measurement validation and rigid transform, independent of ROS.

Covariance is *conditional*: only sensor-reported covariance is rotated.
No radar/robot extrinsic or pose uncertainty can be inferred from TF.
"""
import math


def rotate_covariance(cov, rotation):
    """R C R^T, row-major 3x3 matrices, requires a symmetric PSD input."""
    if len(cov) != 9 or not all(math.isfinite(x) for x in cov):
        return None
    if any(abs(cov[3*i+j] - cov[3*j+i]) > 1e-7
           for i in range(3) for j in range(3)):
        return None
    # Sylvester principal minors for a symmetric 3x3 positive semidefinite matrix.
    a,b,c,d,e,f = cov[0],cov[1],cov[2],cov[4],cov[5],cov[8]
    eps=1e-9
    if (min(a,d,f) < 0 or a*d-b*b < -eps or a*f-c*c < -eps
            or d*f-e*e < -eps
            or a*d*f+2*b*c*e-a*e*e-d*c*c-f*b*b < -eps):
        return None
    tmp = [[sum(rotation[i][k]*cov[k*3+j] for k in range(3))
            for j in range(3)] for i in range(3)]
    out = [sum(tmp[i][k]*rotation[j][k] for k in range(3))
           for i in range(3) for j in range(3)]
    return out if all(math.isfinite(x) for x in out) else None


def quaternion_rotation(q):
    x,y,z,w=q
    if not all(math.isfinite(v) for v in q):
        return None
    norm=sum(v*v for v in q)
    if norm < 1e-12 or abs(norm-1.0) > .02:
        return None
    x,y,z,w=(v/math.sqrt(norm) for v in q)
    return [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]


def transform_track(position, covariance, quaternion, translation):
    if (len(position)!=3 or len(translation)!=3
            or not all(math.isfinite(x) for x in (*position,*translation))):
        return None
    rotation=quaternion_rotation(quaternion)
    if rotation is None:
        return None
    cov=rotate_covariance(covariance,rotation)
    if cov is None:
        return None
    xyz=tuple(sum(rotation[i][k]*position[k] for k in range(3))
              +translation[i] for i in range(3))
    if not all(math.isfinite(x) for x in xyz):
        return None
    return xyz,cov
