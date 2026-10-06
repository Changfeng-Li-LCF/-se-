"""NumPy/ctypes interface for independent open-space exploration fields.

No ROS dependency. Default library path is beside this file; override with
EXPLORATION_FIELDS_LIB or NativeExplorationFields(lib_path=...). The class owns
one map's buffers, reuses them, and rejects maps larger than 2,000,000 cells.
"""
from __future__ import annotations

import ctypes
import os
from pathlib import Path
import threading
from typing import NamedTuple

import numpy as np

MAX_CELLS = 2_000_000


class Fields(NamedTuple):
    clearance_cells: np.ndarray
    frontier_distance_cells: np.ndarray
    owner_source: np.ndarray


class NativeExplorationFields:
    """Bounded reusable workspace; compute outputs are borrowed by default.

    clearance_cells: int32 Chebyshev centre-to-centre cell distance to non-free
      or map-exterior cells. Non-free=0; free map-edge=1. A conservative physical
      lower bound is max(clearance_cells-1,0)*resolution_m.
    frontier_distance_cells: int32 4-neighbour geodesic distance through free;
      -1 for non-free/no reachable frontier. This is a SOFT direction field.
    owner_source: int32 flattened source cell index; -1 where distance is -1.

    One instance retains at most 16*N bytes of native work/output arrays (32 MB
    at 2M cells). Non-contiguous input conversion can add 2*N transient bytes.
    copy=True adds 12*N bytes owned by the caller; otherwise the next compute
    on this instance may overwrite returned arrays. Calls are serialized, but
    borrowed outputs must not be read concurrently with the next computation.
    """

    def __init__(self, lib_path=None, max_cells=MAX_CELLS):
        if isinstance(max_cells, bool) or not isinstance(max_cells, (int, np.integer)):
            raise TypeError("max_cells must be an integer")
        if not 1 <= int(max_cells) <= MAX_CELLS:
            raise ValueError("max_cells must be in [1, 2000000]")
        self.max_cells = int(max_cells)
        default_name = "exploration_fields.dll" if os.name == "nt" else "libexploration_fields.so"
        path = lib_path or os.environ.get("EXPLORATION_FIELDS_LIB") or Path(__file__).with_name(default_name)
        self.lib_path = str(Path(path).expanduser().resolve())
        self._lib = ctypes.CDLL(self.lib_path)
        abi = self._lib.racecar_exploration_fields_abi
        abi.argtypes = []
        abi.restype = ctypes.c_int
        if abi() != 1:
            raise RuntimeError("unsupported exploration fields ABI")
        self._run = self._lib.racecar_exploration_fields
        self._run.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int,
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p,
            ctypes.c_void_p, ctypes.c_int,
        ]
        self._run.restype = ctypes.c_int
        self._lock = threading.Lock()
        self._shape = None
        self._fields = None
        self._queue = None
        self.last_source_count = 0

    @staticmethod
    def _mask(array, name):
        if not isinstance(array, np.ndarray):
            raise TypeError(name + " must be a NumPy ndarray")
        if array.ndim != 2 or array.shape[0] <= 0 or array.shape[1] <= 0:
            raise ValueError(name + " must be a nonempty 2-D array")
        if array.dtype not in (np.dtype(np.bool_), np.dtype(np.uint8)):
            raise TypeError(name + " must have bool or uint8 dtype")
        # bool and uint8 are both byte-sized. View avoids a bool->uint8 copy.
        return np.ascontiguousarray(array.view(np.uint8))

    def compute(self, free_mask, boundary_mask, *, copy=False):
        """Return Fields arrays for the supplied known-free and frontier masks.

        Input masks must be bool/uint8 NumPy arrays of identical (height,width)
        shape. Boundary entries on non-free cells are ignored. Empty frontier
        masks are valid: clearance remains useful and other fields are all -1.
        No map cell lists, Python grid loops, ray casts, or candidate gating.
        """
        # Check dimensions and the budget before making any contiguous copies.
        for array, name in ((free_mask, "free_mask"), (boundary_mask, "boundary_mask")):
            if not isinstance(array, np.ndarray):
                raise TypeError(name + " must be a NumPy ndarray")
            if array.ndim != 2 or not all(d > 0 for d in array.shape):
                raise ValueError(name + " must be a nonempty 2-D array")
            if array.size > self.max_cells:
                raise ValueError(name + " exceeds max_cells")
        if free_mask.shape != boundary_mask.shape:
            raise ValueError("free_mask and boundary_mask shapes differ")
        free = self._mask(free_mask, "free_mask")
        boundary = self._mask(boundary_mask, "boundary_mask")
        height, width = free.shape
        with self._lock:
            if self._shape != free.shape:
                # Drop the previous map workspace first; no unbounded shape cache.
                self._fields = None
                self._queue = None
                self._shape = None
                self._fields = Fields(*(np.empty(free.shape, dtype=np.int32) for _ in range(3)))
                self._queue = np.empty(free.size, dtype=np.int32)
                self._shape = free.shape
            fields = self._fields
            result = self._run(
                free.ctypes.data, boundary.ctypes.data, width, height,
                fields.clearance_cells.ctypes.data,
                fields.frontier_distance_cells.ctypes.data,
                fields.owner_source.ctypes.data,
                self._queue.ctypes.data, self._queue.size,
            )
            if result < 0:
                raise RuntimeError("native exploration fields failed: %d" % result)
            self.last_source_count = result
            return Fields(*(a.copy() for a in fields)) if copy else fields
