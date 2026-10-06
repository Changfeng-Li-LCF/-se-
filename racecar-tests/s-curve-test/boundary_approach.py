"""Conservative local known-to-unknown normal; no route or motion decisions.

The returned yaw uses grid axes.  The caller must add the map origin yaw before
using it in the map frame, and may cache results for one unchanged map snapshot.
"""
import math

import numpy as np


def estimate_boundary_normal(free, unknown, boundary, source_index, resolution):
    """Return a locally supported boundary normal, or ``None`` when ambiguous.

    Only the source's 8-connected boundary component within 0.40 m is fitted.
    The cell radius is capped at 32 to bound work even on unusually fine maps.
    Free/unknown side probes are checked along their full short segments: an
    obstacle, the grid edge, a thin unknown crack, or unknown on both sides
    cannot supply a normal.  This is an orientation hint, never reachability.
    """
    free, unknown, boundary = (np.asarray(mask) for mask in (free, unknown, boundary))
    if (free.ndim != 2 or unknown.shape != free.shape or boundary.shape != free.shape
            or not math.isfinite(resolution) or resolution <= 0):
        return None
    height, width = free.shape
    if not (0 <= source_index < width * height):
        return None
    sy, sx = divmod(int(source_index), width)
    if not (free[sy, sx] and boundary[sy, sx]) or unknown[sy, sx]:
        return None

    radius = min(32, int(math.ceil(.40 / resolution)))
    radius_squared = min(float(radius), .40 / resolution) ** 2
    todo, seen, points = [(sx, sy)], {(sx, sy)}, []
    while todo:
        x, y = todo.pop()
        points.append((x, y))
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if not (dx or dy):
                    continue
                nx, ny = x + dx, y + dy
                if ((nx, ny) in seen or not (0 <= nx < width and 0 <= ny < height)
                        or (nx - sx) ** 2 + (ny - sy) ** 2 > radius_squared):
                    continue
                if not (boundary[ny, nx] and free[ny, nx]) or unknown[ny, nx]:
                    continue
                # Do not connect diagonal free pixels through two blocked sides.
                if dx and dy and not (free[y, nx] or free[ny, x]):
                    continue
                seen.add((nx, ny))
                todo.append((nx, ny))
    if len(points) < 4:
        return None

    points = np.asarray(points, dtype=float)
    centered = (points - points.mean(axis=0)) * resolution
    eigenvalues, eigenvectors = np.linalg.eigh(centered.T @ centered / len(points))
    minor, major = eigenvalues
    if major <= 1e-12 or major < 3.0 * max(minor, 1e-12):
        return None
    tangent = eigenvectors[:, 1]
    normal = np.array([-tangent[1], tangent[0]])
    projections = centered @ tangent
    span = float(np.ptp(projections))
    if span < .25 or float(np.max(np.abs(centered @ normal))) > max(.065, 1.1 * resolution):
        return None

    # Sample across the component, including its ends, rather than trusting one
    # source pixel or crossing a wall to an unrelated nearby boundary fragment.
    order = np.argsort(projections, kind='stable')
    samples = points[order[np.unique(np.linspace(0, len(order) - 1, min(9, len(order))).astype(int))]]
    probe_steps = np.arange(.5, 3.01, .5)

    def side_support(direction):
        for x, y in samples:
            outward, inward = [], []
            for sign, states in ((1, outward), (-1, inward)):
                for step in probe_steps:
                    fx, fy = x + sign * direction[0] * step, y + sign * direction[1] * step
                    # Check every cell touched around the fractional segment
                    # sample so a diagonal cannot slip between obstacle pixels.
                    low_x, high_x = int(math.floor(fx)), int(math.ceil(fx))
                    low_y, high_y = int(math.floor(fy)), int(math.ceil(fy))
                    if not (0 <= low_x <= high_x < width and 0 <= low_y <= high_y < height):
                        return False
                    for check_y in (low_y, high_y):
                        for check_x in (low_x, high_x):
                            if bool(free[check_y, check_x]) == bool(unknown[check_y, check_x]):
                                return False
                    px = int(math.floor(fx + .5))
                    py = int(math.floor(fy + .5))
                    if not (0 <= px < width and 0 <= py < height):
                        return False
                    is_free, is_unknown = bool(free[py, px]), bool(unknown[py, px])
                    if is_free == is_unknown:  # obstacle or contradictory masks
                        return False
                    states.append(is_unknown)
            # The inward segment must remain known free.  Beyond the first
            # quantized pixel, the outward segment must remain unknown; a crack
            # that exits into free space does not define a reliable approach.
            if any(inward) or not all(outward[-3:]):
                return False
            first_unknown = next((i for i, state in enumerate(outward) if state), None)
            if first_unknown is None or not all(outward[first_unknown:]):
                return False
        return True

    plus, minus = side_support(normal), side_support(-normal)
    if plus == minus:
        return None
    if minus:
        normal = -normal
    return dict(grid_yaw=math.atan2(float(normal[1]), float(normal[0])),
                coherence=float((major - minor) / (major + minor)), span_m=span)
