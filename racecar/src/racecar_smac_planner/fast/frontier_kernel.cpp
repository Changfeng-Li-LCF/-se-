#include <algorithm>
#include <cstdint>
#include <vector>

// No ROS or static mutable state; each call owns its buffers. Matches the
// reference BFS's left/right/up/down order and its search budget exactly.
extern "C" int racecar_frontier_bfs(
  const uint8_t *free_cells, const uint8_t *boundary, int width, int height,
  int start, int limit, int32_t *distance, int32_t *parent,
  int32_t *found, int32_t *found_count, int32_t *exhausted)
{
  if (!free_cells || !boundary || !distance || !parent || !found ||
    !found_count || !exhausted || width <= 0 || height <= 0 ||
    int64_t(width)*height > 2000000 || limit < 0) return -1;
  const int size=width*height;
  if (start < 0 || start >= size || !free_cells[start]) return -2;
  try {
    std::fill_n(distance,size,-1);std::fill_n(parent,size,-1);
    std::vector<int> queue;queue.reserve(std::min(size,limit+16));
    queue.push_back(start);distance[start]=0;
    int count=0,nfound=0;
    while (count < static_cast<int>(queue.size()) && count < limit) {
      const int index=queue[count++],x=index%width,y=index/width;
      if (boundary[index]) found[nfound++]=index;
      const int next[4]={x ? index-1 : -1,x+1<width ? index+1 : -1,
                        y ? index-width : -1,y+1<height ? index+width : -1};
      for (int n:next) {
        if (n>=0 && free_cells[n] && distance[n]<0) {
          distance[n]=distance[index]+1;parent[n]=index;queue.push_back(n);
        }
      }
    }
    *found_count=nfound;*exhausted=count<static_cast<int>(queue.size());
    return count;
  } catch (...) {return -3;}
}
