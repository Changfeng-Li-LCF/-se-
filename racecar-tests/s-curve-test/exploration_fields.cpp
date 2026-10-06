#include <algorithm>
#include <cstdint>

// Independent of ROS and the existing frontier BFS. All scratch storage is
// caller-owned: no per-call allocation, global cache, or mutable static state.
#if defined(_WIN32)
#define FIELDS_EXPORT __declspec(dllexport)
#else
#define FIELDS_EXPORT __attribute__((visibility("default")))
#endif

extern "C" FIELDS_EXPORT int racecar_exploration_fields_abi() { return 1; }

// Returns the number of free boundary sources, or -1 for invalid arguments.
// All arrays are row-major and contain at least width*height entries; queue has
// queue_capacity entries. Inputs and outputs must not overlap. A nonzero mask
// entry is true. Non-free boundary entries are ignored.
//
// clearance: exact Chebyshev distance to the nearest non-free cell centre or
// exterior cell centre; blocked=0 and a free map-edge cell=1. For conservative
// metric clearance to occupied/unknown cell AREAS and the map edge, use
// max(clearance-1,0)*resolution. Do not interpret this as Euclidean metres.
//
// frontier_distance: shortest 4-connected path through free cells to a source,
// measured in grid steps; -1 for non-free cells or a component with no source.
// owner_source: row-major index of that source, or -1. Ties are deterministic
// (sources seeded row-major; neighbours visited left/right/up/down).
extern "C" FIELDS_EXPORT int racecar_exploration_fields(
    const uint8_t* free_cells, const uint8_t* boundary,
    int width, int height, int32_t* clearance,
    int32_t* frontier_distance, int32_t* owner_source,
    int32_t* queue, int queue_capacity) {
  const int64_t count = int64_t(width) * int64_t(height);
  if (!free_cells || !boundary || !clearance || !frontier_distance ||
      !owner_source || !queue || width <= 0 || height <= 0 ||
      count > 2000000 || queue_capacity < count) return -1;
  const int size = static_cast<int>(count);
  int tail = 0;
  // Encode queued coordinates with a power-of-two row stride. This removes
  // per-visited-cell integer division/modulo on the RISC-V target. Since
  // stride < 2*width and width*height <= 2M, the code fits safely in int32.
  int shift = 0;
  while ((int64_t(1) << shift) < width) ++shift;
  const int stride = int(1) << shift;
  const int x_mask = stride - 1;

  // First of the two exact chessboard-distance transform passes. Map exterior
  // is included explicitly, so all-free maps have finite clearance too.
  for (int y = 0; y < height; ++y) {
    const int row = y * width;
    for (int x = 0; x < width; ++x) {
      const int i = row + x;
      if (!free_cells[i]) {
        clearance[i] = 0;
      } else {
        int d = 1;
        if (x && y && x + 1 < width && y + 1 < height) {
          d = 1 + std::min(std::min(clearance[i - 1], clearance[i - width]),
                           std::min(clearance[i - width - 1], clearance[i - width + 1]));
        }
        clearance[i] = d;
      }
      if (free_cells[i] && boundary[i]) {
        frontier_distance[i] = 0;
        owner_source[i] = i;
        queue[tail++] = (y << shift) | x;
      } else {
        frontier_distance[i] = -1;
        owner_source[i] = -1;
      }
    }
  }
  for (int y = height - 1; y >= 0; --y) {
    const int row = y * width;
    for (int x = width - 1; x >= 0; --x) {
      const int i = row + x;
      // Values 0 and 1 are final, including every map edge. The remaining
      // cells are interior, so the following four neighbours always exist.
      int d = clearance[i];
      if (d > 1)
        clearance[i] = std::min(d, 1 + std::min(
            std::min(clearance[i + 1], clearance[i + width]),
            std::min(clearance[i + width - 1], clearance[i + width + 1])));
    }
  }

  const int sources = tail;
  int head = 0;
  while (head < tail) {
    const int packed = queue[head++];
    const int x = packed & x_mask;
    const int i = (packed >> shift) * width + x;
    const int next_distance = frontier_distance[i] + 1;
    const int owner = owner_source[i];
    const auto visit = [&](int j, int encoded) {
      if (free_cells[j] && frontier_distance[j] < 0) {
        frontier_distance[j] = next_distance;
        owner_source[j] = owner;
        queue[tail++] = encoded;
      }
    };
    if (x) visit(i - 1, packed - 1);
    if (x + 1 < width) visit(i + 1, packed + 1);
    if (i >= width) visit(i - width, packed - stride);
    if (i < size - width) visit(i + width, packed + stride);
  }
  return sources;
}
