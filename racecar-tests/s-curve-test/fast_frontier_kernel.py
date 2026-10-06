"""ctypes bridge to the bounded native flood fill; CDLL releases the Python GIL."""
import ctypes
import os
from pathlib import Path
import numpy as np

_library=None


def load_kernel():
    global _library
    if _library is None:
        override=os.environ.get('RACECAR_FRONTIER_KERNEL')
        if override:path=Path(override)
        else:
            from ament_index_python.packages import get_package_prefix
            path=Path(get_package_prefix('racecar_smac_planner'))/'lib/libracecar_frontier_kernel.so'
        library=ctypes.CDLL(str(path))
        u8=np.ctypeslib.ndpointer(dtype=np.uint8,flags='C_CONTIGUOUS')
        i32=np.ctypeslib.ndpointer(dtype=np.int32,flags='C_CONTIGUOUS')
        library.racecar_frontier_bfs.argtypes=[u8,u8,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_int,
                                             i32,i32,i32,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32)]
        library.racecar_frontier_bfs.restype=ctypes.c_int
        _library=library
    return _library


def flood_fill_arrays(free,boundary,start,limit):
    """Return owned NumPy results without Python per-cell list conversions."""
    height,width=free.shape;n=width*height
    free=np.ascontiguousarray(free,dtype=np.uint8)
    boundary=np.ascontiguousarray(boundary,dtype=np.uint8)
    distance=np.empty(n,dtype=np.int32);parent=np.empty(n,dtype=np.int32)
    found=np.empty(min(n,max(0,limit)),dtype=np.int32)
    found_count=ctypes.c_int32();exhausted=ctypes.c_int32()
    count=load_kernel().racecar_frontier_bfs(free,boundary,width,height,start,limit,distance,parent,
        found,ctypes.byref(found_count),ctypes.byref(exhausted))
    if count<0:raise RuntimeError('Native frontier BFS failed: '+str(count))
    return distance,parent,found[:found_count.value],count,bool(exhausted.value)


def flood_fill(free,boundary,start,limit):
    """Retain the historical list-returning API for existing consumers."""
    distance,parent,found,count,exhausted=flood_fill_arrays(free,boundary,start,limit)
    return distance.tolist(),parent.tolist(),found.tolist(),count,exhausted
