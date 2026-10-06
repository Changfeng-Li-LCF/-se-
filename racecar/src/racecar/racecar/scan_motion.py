"""Small scan registration with native CPU helpers and offline NumPy fallback."""
from dataclasses import dataclass
import ctypes
import math
import os

import numpy as np


class _NativeMatch(ctypes.Structure):
    _fields_ = [(name, ctypes.c_double) for name in
                ("dx", "dy", "dyaw", "rmse", "overlap", "condition")] + [
                (name, ctypes.c_uint32) for name in
                ("inliers", "iterations", "flags", "reason")]


def _load_native():
    path = os.environ.get("SCAN_MOTION_NATIVE_LIBRARY")
    if not path:
        try:
            from ament_index_python.packages import get_package_prefix
            path = os.path.join(get_package_prefix("racecar"), "lib", "libscan_motion_native.so")
        except (ImportError, LookupError) as error:
            return None, None, str(error)
    try:
        library = ctypes.CDLL(path)
        library.scan_motion_native_abi.argtypes = []
        library.scan_motion_native_abi.restype = ctypes.c_uint32
        if library.scan_motion_native_abi() != 2:
            raise ValueError("Unsupported scan motion native ABI")
        double_pointer = ctypes.POINTER(ctypes.c_double)
        library.scan_motion_nearest.argtypes = [
            double_pointer, ctypes.c_size_t, double_pointer, ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_int64), double_pointer]
        library.scan_motion_nearest.restype = ctypes.c_int
        library.scan_motion_surface_normals.argtypes = [
            double_pointer, ctypes.c_size_t, double_pointer,
            ctypes.POINTER(ctypes.c_uint8)]
        library.scan_motion_surface_normals.restype = ctypes.c_int
        library.scan_motion_icp.argtypes = [
            double_pointer, ctypes.c_size_t, double_pointer, ctypes.c_size_t,
            double_pointer, ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_double, ctypes.c_double, ctypes.c_double,
            ctypes.POINTER(_NativeMatch)]
        library.scan_motion_icp.restype = ctypes.c_int
        return library, path, ""
    except (OSError, AttributeError, ValueError) as error:
        return None, path, str(error)


_NATIVE, _NATIVE_PATH, _NATIVE_ERROR = _load_native()


def require_native():
    """Production node calls this before subscribing; NumPy is offline fallback."""
    if _NATIVE is None:
        raise RuntimeError("scan motion native library unavailable: " + _NATIVE_ERROR)
    return _NATIVE_PATH


def _double_pointer(array):
    return array.ctypes.data_as(ctypes.POINTER(ctypes.c_double))


@dataclass
class MatchResult:
    """p_previous = R(dyaw) @ p_current + [dx, dy], in base frames."""
    dx: float = 0.0
    dy: float = 0.0
    dyaw: float = 0.0
    rmse: float = math.inf
    overlap: float = 0.0
    condition: float = math.inf
    inliers: int = 0
    converged: bool = False
    degenerate: bool = True
    valid: bool = False
    reason: str = "insufficient_points"
    iterations: int = 0


def scan_points(ranges, angle_min, angle_increment, *, range_min=0.20,
                range_max=12.0, time_increment=0.0, yaw_rate=0.0,
                yaw_offsets=None, velocity_hint=None, max_points=180,
                reference_time_s=0.0, sensor_offset_xy=(0.0, 0.0)):
    """Return points in base coordinates at the requested within-scan time.

    Ray i has time i*time_increment, including invalid/unselected rays.
    The reference may extend one time_increment past the last ray, to the
    complete-circle boundary; translation uses last-ray velocity there.
    yaw_offsets[i] is its IMU-integrated yaw relative to reference_time_s;
    otherwise yaw_rate approximates that angle. Timing is supplied by the
    caller; this function does not claim calibrated scanner timing.
    velocity_hint, when available, is the previous valid scan estimate of
    body (vx, vy), never a Cartographer/TF velocity. It is assumed constant
    over this scan, and rotated/integrated using the same per-ray yaw.
    """
    r = np.asarray(ranges, dtype=float).reshape(-1)
    if not all(math.isfinite(x) for x in
               (angle_min, angle_increment, time_increment,
                reference_time_s, yaw_rate, range_min, range_max)):
        raise ValueError("Non-finite scan geometry or timing")
    if time_increment < 0 or range_max <= range_min or max_points < 3:
        raise ValueError("Invalid scan limits")
    times = np.arange(r.size, dtype=float) * time_increment
    if reference_time_s < 0 or (r.size and
            reference_time_s > times[-1] + time_increment + 1e-9):
        raise ValueError("Reference time is outside the scan")
    offsets = np.asarray(sensor_offset_xy, dtype=float)
    if offsets.shape != (2,) or not np.isfinite(offsets).all():
        raise ValueError("Invalid sensor offset")
    if yaw_offsets is None:
        yaw = yaw_rate * (times - reference_time_s)
    else:
        yaw = np.asarray(yaw_offsets, dtype=float)
        if yaw.shape != r.shape or not np.isfinite(yaw).all():
            raise ValueError("yaw_offsets must contain one finite angle per ray")
    ids = np.flatnonzero(np.isfinite(r) & (r >= range_min) & (r <= range_max))
    if ids.size == 0:
        return np.empty((0, 2), dtype=float)
    if ids.size > max_points:
        ids = ids[np.linspace(0, ids.size - 1, int(max_points), dtype=int)]
    angles = angle_min + ids * angle_increment
    points = r[ids, None] * np.column_stack((np.cos(angles), np.sin(angles)))
    points += offsets  # Fixed laser-to-base translation, before deskew rotation.
    c, s = np.cos(yaw[ids]), np.sin(yaw[ids])
    points = np.column_stack((c * points[:, 0] - s * points[:, 1],
                              s * points[:, 0] + c * points[:, 1]))
    if velocity_hint is not None:
        v = np.asarray(velocity_hint, dtype=float)
        if v.shape != (2,) or not np.isfinite(v).all():
            raise ValueError("Invalid body velocity hint")
        c, s = np.cos(yaw), np.sin(yaw)
        velocities = np.column_stack((c * v[0] - s * v[1],
                                       s * v[0] + c * v[1]))
        displacement = np.zeros((r.size, 2), dtype=float)
        if r.size > 1:
            displacement[1:] = np.cumsum(
                0.5 * (velocities[1:] + velocities[:-1]) * time_increment, axis=0)
        at_reference = np.array([np.interp(reference_time_s, times, displacement[:, k])
                                 for k in range(2)])
        if reference_time_s > times[-1]:
            at_reference += velocities[-1] * (reference_time_s - times[-1])
        points += displacement[ids] - at_reference
    return points


def _rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((c, -s), (s, c)), dtype=float)


def _nearest_numpy(points, reference):
    indices, squared = [], []
    for start in range(0, len(points), 64):
        delta = points[start:start + 64, None, :] - reference[None, :, :]
        distances = np.einsum("ijk,ijk->ij", delta, delta)
        ids = np.argmin(distances, axis=1)
        indices.append(ids)
        squared.append(distances[np.arange(len(ids)), ids])
    return np.concatenate(indices), np.concatenate(squared)


def _nearest(points, reference):
    if _NATIVE is None:
        return _nearest_numpy(points, reference)
    points = np.ascontiguousarray(points, dtype=np.float64)
    reference = np.ascontiguousarray(reference, dtype=np.float64)
    indices = np.empty(len(points), dtype=np.int64)
    squared = np.empty(len(points), dtype=np.float64)
    status = _NATIVE.scan_motion_nearest(
        _double_pointer(points), len(points), _double_pointer(reference), len(reference),
        indices.ctypes.data_as(ctypes.POINTER(ctypes.c_int64)), _double_pointer(squared))
    if status != 0:
        raise RuntimeError("Native nearest-neighbour input rejected")
    return indices, squared


def _surface_normals_numpy(points):
    # Local PCA normals let the final quality check detect parallel walls;
    # point-to-point residuals alone can falsely imply corridor observability.
    delta = points[:, None, :] - points[None, :, :]
    squared = np.einsum("ijk,ijk->ij", delta, delta)
    neighbours = np.argpartition(squared, 4, axis=1)[:, :5]
    local = points[neighbours]
    centred = local - local.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", centred, centred)
    values, vectors = np.linalg.eigh(covariance)
    normals = vectors[:, :, 0]
    good = ((values[:, 1] > 1e-5) &
            (values[:, 0] < 0.25 * values[:, 1]) &
            (squared[np.arange(len(points))[:, None], neighbours].max(axis=1) < 1.0))
    return normals, good


def _surface_normals(points):
    if _NATIVE is None:
        return _surface_normals_numpy(points)
    points = np.ascontiguousarray(points, dtype=np.float64)
    normals = np.empty_like(points)
    good = np.empty(len(points), dtype=np.uint8)
    status = _NATIVE.scan_motion_surface_normals(
        _double_pointer(points), len(points), _double_pointer(normals),
        good.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)))
    if status != 0:
        raise RuntimeError("Native surface-normal input rejected")
    return normals, good.astype(bool)


def _icp_numpy(previous, current, normals, normal_good, yaw, translation):
    rotation = _rotation(yaw)
    translation = np.array(translation, dtype=float, copy=True)
    converged = False
    for iteration in range(18):
        moved = current @ rotation.T + translation
        nearest, squared = _nearest(moved, previous)
        candidates = np.flatnonzero(squared <= 0.40 ** 2)
        if len(candidates) < 30:
            return MatchResult(reason="insufficient_matches", iterations=iteration + 1)
        keep = max(30, int(0.80 * len(candidates)))
        selected = candidates[np.argsort(squared[candidates])[:keep]]
        source, target = moved[selected], previous[nearest[selected]]
        a, b = source.mean(axis=0), target.mean(axis=0)
        u, _, vt = np.linalg.svd((source - a).T @ (target - b))
        step_rotation = vt.T @ u.T
        if np.linalg.det(step_rotation) < 0:
            vt[-1] *= -1
            step_rotation = vt.T @ u.T
        step_translation = b - step_rotation @ a
        rotation = step_rotation @ rotation
        translation = step_rotation @ translation + step_translation
        step_yaw = math.atan2(step_rotation[1, 0], step_rotation[0, 0])
        if np.linalg.norm(step_translation) < 0.0005 and abs(step_yaw) < 0.0002:
            converged = True
            break
    moved = current @ rotation.T + translation
    nearest, squared = _nearest(moved, previous)
    candidates = np.flatnonzero(squared <= 0.40 ** 2)
    keep = max(30, int(0.80 * len(candidates)))
    selected = candidates[np.argsort(squared[candidates])[:keep]]
    rmse = float(np.sqrt(squared[selected].mean())) if len(selected) else math.inf
    overlap = float(np.mean(squared <= 0.12 ** 2))
    observed = selected[normal_good[nearest[selected]]]
    condition = math.inf
    degenerate = True
    if len(observed) >= 20:
        n = normals[nearest[observed]]
        p = moved[observed] - moved[observed].mean(axis=0)
        scale = max(0.5, float(np.sqrt(np.mean(np.sum(p * p, axis=1)))))
        jacobian = np.column_stack((n, (-n[:, 0] * p[:, 1] + n[:, 1] * p[:, 0]) / scale))
        eigenvalues = np.linalg.eigvalsh(jacobian.T @ jacobian / len(observed))
        condition = float(eigenvalues[-1] / max(eigenvalues[0], 1e-12))
        degenerate = eigenvalues[0] < 0.01 or condition > 100.0
    reason = ("insufficient_matches" if len(selected) < 30 else
              "low_overlap" if overlap < 0.55 else
              "high_residual" if rmse > 0.10 else
              "degenerate_geometry" if degenerate else "ok")
    return MatchResult(float(translation[0]), float(translation[1]),
                       math.atan2(rotation[1, 0], rotation[0, 0]),
                       rmse, overlap, condition, len(selected), converged,
                       degenerate, reason == "ok", reason, iteration + 1)


def _icp(previous, current, normals, normal_good, yaw, translation):
    if _NATIVE is None:
        return _icp_numpy(previous, current, normals, normal_good, yaw, translation)
    previous = np.ascontiguousarray(previous, dtype=np.float64)
    current = np.ascontiguousarray(current, dtype=np.float64)
    normals = np.ascontiguousarray(normals, dtype=np.float64)
    normal_good = np.ascontiguousarray(normal_good, dtype=np.uint8)
    output = _NativeMatch()
    status = _NATIVE.scan_motion_icp(
        _double_pointer(previous), len(previous), _double_pointer(current), len(current),
        _double_pointer(normals), normal_good.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
        yaw, float(translation[0]), float(translation[1]), ctypes.byref(output))
    if status != 0:
        raise RuntimeError("Native ICP input rejected")
    reasons = ("ok", "insufficient_matches", "low_overlap", "high_residual",
               "degenerate_geometry")
    return MatchResult(output.dx, output.dy, output.dyaw, output.rmse,
                       output.overlap, output.condition, output.inliers,
                       bool(output.flags & 1), bool(output.flags & 2),
                       bool(output.flags & 4), reasons[output.reason], output.iterations)


def match_scans(prev_points, curr_points, yaw_hint, translation_hint=None):
    """Register current base-frame scan onto previous; hints are not constraints.

    Search starts only at zero translation and, if supplied, the previous
    scan-motion prediction. Estimated rotation is free to differ from IMU.
    Quality uses correspondence count, residual, overlap and wall-normal
    observability, never the sign of dx or any supposed forward direction.
    """
    previous = np.asarray(prev_points, dtype=float)
    current = np.asarray(curr_points, dtype=float)
    if (previous.ndim != 2 or current.ndim != 2 or
            previous.shape[1:] != (2,) or current.shape[1:] != (2,) or
            not math.isfinite(yaw_hint)):
        return MatchResult(reason="invalid_input")
    previous = previous[np.isfinite(previous).all(axis=1)]
    current = current[np.isfinite(current).all(axis=1)]
    if min(len(previous), len(current)) < 30:
        return MatchResult()
    # Bound work even for callers that supply unsampled clouds.
    previous = previous[np.linspace(0, len(previous) - 1, min(200, len(previous)), dtype=int)]
    current = current[np.linspace(0, len(current) - 1, min(200, len(current)), dtype=int)]
    normals, good = _surface_normals(previous)
    starts = [np.zeros(2)]
    if translation_hint is not None:
        hint = np.asarray(translation_hint, dtype=float)
        if hint.shape != (2,) or not np.isfinite(hint).all():
            return MatchResult(reason="invalid_hint")
        if np.linalg.norm(hint) > 0.001:
            starts.append(hint)
    results = [_icp(previous, current, normals, good, yaw_hint, t) for t in starts]
    results.sort(key=lambda r: (not r.valid, r.rmse))
    best = results[0]
    if len(results) > 1 and best.valid and results[1].valid:
        other = results[1]
        separation = math.hypot(best.dx - other.dx, best.dy - other.dy)
        angle = abs(math.remainder(best.dyaw - other.dyaw, 2 * math.pi))
        if other.rmse <= 1.10 * best.rmse + 0.002 and (separation > 0.15 or angle > 0.10):
            best.valid = False
            best.reason = "ambiguous_initializations"
    return best
