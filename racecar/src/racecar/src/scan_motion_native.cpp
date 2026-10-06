// Small scalar CPU helpers for scan_motion.py; no ROS or external dependencies.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <vector>

namespace {

// Sort one reference axis once, then stop each outward search when that axis
// alone exceeds the best full squared distance. This remains an exact search;
// the original point index resolves ties regardless of traversal order.
class NearestIndex {
 public:
  NearestIndex(const double *points, std::size_t count)
      : points_(points), order_(count), finite_(true) {
    double min_x = std::numeric_limits<double>::infinity(), min_y = min_x;
    double max_x = -min_x, max_y = -min_x;
    for (std::size_t i = 0; i < count; ++i) {
      order_[i] = i;
      const double x = points[2*i], y = points[2*i+1];
      finite_ = finite_ && std::isfinite(x) && std::isfinite(y);
      min_x = std::min(min_x, x); max_x = std::max(max_x, x);
      min_y = std::min(min_y, y); max_y = std::max(max_y, y);
    }
    axis_ = max_y - min_y > max_x - min_x ? 1 : 0;
    if (finite_)
      std::sort(order_.begin(), order_.end(), [&](std::size_t a, std::size_t b) {
        const double av = points_[2*a+axis_], bv = points_[2*b+axis_];
        return av < bv || (av == bv && a < b);
      });
  }

  void nearest(double x, double y, std::int64_t &index, double &squared) const {
    squared = std::numeric_limits<double>::infinity();
    index = 0;
    const auto consider = [&](std::size_t j) {
      const double dx = x - points_[2*j], dy = y - points_[2*j+1];
      const double distance = dx*dx + dy*dy;
      if (distance < squared || (distance == squared && j < static_cast<std::size_t>(index))) {
        squared = distance; index = static_cast<std::int64_t>(j);
      }
    };
    if (!finite_ || !std::isfinite(x) || !std::isfinite(y)) {
      for (std::size_t j = 0; j < order_.size(); ++j) consider(j);
      return;
    }
    const double value = axis_ == 0 ? x : y;
    std::size_t right = static_cast<std::size_t>(std::lower_bound(
        order_.begin(), order_.end(), value, [&](std::size_t j, double v) {
          return points_[2*j+axis_] < v;
        }) - order_.begin());
    std::size_t left = right;
    while (left || right < order_.size()) {
      const double dl = left ? value - points_[2*order_[left-1]+axis_] :
          std::numeric_limits<double>::infinity();
      const double dr = right < order_.size() ? points_[2*order_[right]+axis_] - value :
          std::numeric_limits<double>::infinity();
      // Equality must remain searchable to preserve first-index ties.
      if (dl*dl > squared && dr*dr > squared) break;
      if (left && (right == order_.size() || dl <= dr)) consider(order_[--left]);
      else consider(order_[right++]);
    }
  }

  // Exact five-neighbour query, retaining the original ascending-index tie
  // order. The axis bound skips points that cannot enter the nearest five.
  void nearest_five(double x, double y, std::size_t *indices, double *squared) const {
    for (std::size_t k=0;k<5;++k) {
      squared[k]=std::numeric_limits<double>::infinity();indices[k]=0;
    }
    const bool finite=finite_ && std::isfinite(x) && std::isfinite(y);
    const auto consider=[&](std::size_t j) {
      const double dx=x-points_[2*j],dy=y-points_[2*j+1],d=dx*dx+dy*dy;
      if (d>squared[4] || (d==squared[4] && (!finite || j>=indices[4]))) return;
      std::size_t k=4;
      while (k && (d<squared[k-1] || (finite && d==squared[k-1] && j<indices[k-1]))) {
        squared[k]=squared[k-1];indices[k]=indices[k-1];--k;
      }
      squared[k]=d;indices[k]=j;
    };
    if (!finite) {
      for (std::size_t j=0;j<order_.size();++j) consider(j);
      return;
    }
    const double value=axis_==0 ? x : y;
    std::size_t right=static_cast<std::size_t>(std::lower_bound(
      order_.begin(),order_.end(),value,[&](std::size_t j,double v) {
        return points_[2*j+axis_]<v;
      })-order_.begin());
    std::size_t left=right;
    while (left || right<order_.size()) {
      const double dl=left ? value-points_[2*order_[left-1]+axis_] :
        std::numeric_limits<double>::infinity();
      const double dr=right<order_.size() ? points_[2*order_[right]+axis_]-value :
        std::numeric_limits<double>::infinity();
      if (dl*dl>squared[4] && dr*dr>squared[4]) break;
      if (left && (right==order_.size() || dl<=dr)) consider(order_[--left]);
      else consider(order_[right++]);
    }
  }

 private:
  const double *points_;
  std::vector<std::size_t> order_;
  bool finite_;
  int axis_;
};

// Eigenvalues only: Jacobi rotations of the symmetric quality Gram matrix.
void eigenvalues3(double a[3][3], double values[3]) {
  for (int iteration = 0; iteration < 32; ++iteration) {
    int p = 0, q = 1;
    if (std::abs(a[0][2]) > std::abs(a[p][q])) { p = 0; q = 2; }
    if (std::abs(a[1][2]) > std::abs(a[p][q])) { p = 1; q = 2; }
    const double size = std::abs(a[0][0]) + std::abs(a[1][1]) + std::abs(a[2][2]);
    if (std::abs(a[p][q]) <= 1e-15 * std::max(1.0, size)) break;
    const double angle = 0.5 * std::atan2(2.0 * a[p][q], a[q][q] - a[p][p]);
    const double c = std::cos(angle), s = std::sin(angle);
    const double app = a[p][p], aqq = a[q][q], apq = a[p][q];
    a[p][p] = c*c*app - 2.0*c*s*apq + s*s*aqq;
    a[q][q] = s*s*app + 2.0*c*s*apq + c*c*aqq;
    a[p][q] = a[q][p] = 0.0;
    for (int k = 0; k < 3; ++k) {
      if (k == p || k == q) continue;
      const double akp = a[k][p], akq = a[k][q];
      a[k][p] = a[p][k] = c*akp - s*akq;
      a[k][q] = a[q][k] = s*akp + c*akq;
    }
  }
  for (int i = 0; i < 3; ++i) values[i] = a[i][i];
  std::sort(values, values + 3);
}

}  // namespace

extern "C" {

std::uint32_t scan_motion_native_abi() { return 2; }

struct ScanMotionResult {
  double dx, dy, dyaw, rmse, overlap, condition;
  std::uint32_t inliers, iterations, flags, reason;
};

// Nx2/Mx2 contiguous float64 inputs. Exact squared Euclidean nearest neighbour,
// including NumPy argmin's first-index tie convention. Zero means success.
int scan_motion_nearest(const double *points, std::size_t n,
                        const double *reference, std::size_t m,
                        std::int64_t *indices, double *squared) {
  if (!points || !reference || !indices || !squared || !m) return -1;
  const NearestIndex reference_index(reference, m);
  for (std::size_t i = 0; i < n; ++i) {
    reference_index.nearest(points[2*i], points[2*i+1], indices[i], squared[i]);
  }
  return 0;
}

// Same five-neighbour local PCA and thresholds as the NumPy implementation.
// Analytic eigenvectors of each symmetric 2x2 covariance avoid batched LAPACK.
int scan_motion_surface_normals(const double *points, std::size_t n,
                                double *normals, std::uint8_t *good) {
  if (!points || !normals || !good || n < 5) return -1;
  const NearestIndex index(points,n);
  for (std::size_t i = 0; i < n; ++i) {
    double distances[5];
    std::size_t neighbours[5] = {};
    index.nearest_five(points[2*i],points[2*i+1],neighbours,distances);
    double cx = 0.0, cy = 0.0;
    for (std::size_t j : neighbours) { cx += points[2 * j]; cy += points[2 * j + 1]; }
    cx /= 5.0; cy /= 5.0;
    double xx = 0.0, xy = 0.0, yy = 0.0;
    for (std::size_t j : neighbours) {
      const double x = points[2 * j] - cx, y = points[2 * j + 1] - cy;
      xx += x * x; xy += x * y; yy += y * y;
    }
    const double spread = std::hypot(xx - yy, 2.0 * xy);
    const double small = 0.5 * (xx + yy - spread);
    const double large = 0.5 * (xx + yy + spread);
    const double angle = 0.5 * std::atan2(2.0 * xy, xx - yy);
    normals[2 * i] = -std::sin(angle);
    normals[2 * i + 1] = std::cos(angle);
    good[i] = static_cast<std::uint8_t>(large > 1e-5 && small < 0.25 * large && distances[4] < 1.0);
  }
  return 0;
}

// One ICP initialization. The two-start selection/ambiguity policy stays Python.
// flags: 1 converged, 2 degenerate, 4 valid; reason: 0 ok, 1 insufficient
// matches, 2 low overlap, 3 high residual, 4 degenerate geometry.
int scan_motion_icp(const double *previous, std::size_t n,
                    const double *current, std::size_t m,
                    const double *normals, const std::uint8_t *normal_good,
                    double yaw, double tx, double ty, ScanMotionResult *result) {
  if (!previous || !current || !normals || !normal_good || !result || n < 30 || m < 30)
    return -1;
  const double infinity = std::numeric_limits<double>::infinity();
  *result = {0.0, 0.0, 0.0, infinity, 0.0, infinity, 0, 0, 2, 1};
  std::vector<double> moved(2*m), squared(m);
  std::vector<std::int64_t> nearest(m);
  std::vector<std::size_t> selected;
  selected.reserve(m);
  const NearestIndex reference_index(previous, n);
  double rc = std::cos(yaw), rs = std::sin(yaw);
  bool converged = false;
  const auto correspond = [&]() {
    for (std::size_t i = 0; i < m; ++i) {
      moved[2*i] = rc*current[2*i] - rs*current[2*i+1] + tx;
      moved[2*i+1] = rs*current[2*i] + rc*current[2*i+1] + ty;
    }
    for (std::size_t i = 0; i < m; ++i)
      reference_index.nearest(moved[2*i], moved[2*i+1], nearest[i], squared[i]);
    selected.clear();
    for (std::size_t i = 0; i < m; ++i)
      if (squared[i] <= 0.40*0.40) selected.push_back(i);
    std::sort(selected.begin(), selected.end(), [&](std::size_t a, std::size_t b) {
      return squared[a] < squared[b];
    });
  };
  for (std::uint32_t iteration = 0; iteration < 18; ++iteration) {
    result->iterations = iteration + 1;
    correspond();
    if (selected.size() < 30) return 0;
    selected.resize(std::max<std::size_t>(30, static_cast<std::size_t>(0.80*selected.size())));
    double ax = 0.0, ay = 0.0, bx = 0.0, by = 0.0;
    for (std::size_t i : selected) {
      ax += moved[2*i]; ay += moved[2*i+1];
      bx += previous[2*nearest[i]]; by += previous[2*nearest[i]+1];
    }
    const double count = static_cast<double>(selected.size());
    ax /= count; ay /= count; bx /= count; by /= count;
    double dot = 0.0, cross = 0.0;
    for (std::size_t i : selected) {
      const double x = moved[2*i]-ax, y = moved[2*i+1]-ay;
      const double u = previous[2*nearest[i]]-bx, v = previous[2*nearest[i]+1]-by;
      dot += x*u + y*v;
      cross += x*v - y*u;
    }
    // The proper 2D Kabsch rotation, equivalent to SVD plus reflection fix.
    const double angle = std::atan2(cross, dot);
    const double c = std::cos(angle), s = std::sin(angle);
    const double step_x = bx - (c*ax - s*ay), step_y = by - (s*ax + c*ay);
    const double next_c = c*rc - s*rs, next_s = s*rc + c*rs;
    const double next_x = c*tx - s*ty + step_x;
    ty = s*tx + c*ty + step_y; tx = next_x;
    rc = next_c; rs = next_s;
    if (std::hypot(step_x, step_y) < 0.0005 && std::abs(angle) < 0.0002) {
      converged = true; break;
    }
  }
  correspond();
  selected.resize(std::min(selected.size(), std::max<std::size_t>(30,
      static_cast<std::size_t>(0.80*selected.size()))));
  double residual = 0.0;
  for (std::size_t i : selected) residual += squared[i];
  const double rmse = selected.empty() ? infinity : std::sqrt(residual/selected.size());
  std::size_t overlap_count = 0;
  for (double distance : squared) if (distance <= 0.12*0.12) ++overlap_count;
  const double overlap = static_cast<double>(overlap_count)/m;
  std::vector<std::size_t> observed;
  observed.reserve(selected.size());
  double px = 0.0, py = 0.0;
  for (std::size_t i : selected) {
    if (normal_good[nearest[i]]) {
      observed.push_back(i); px += moved[2*i]; py += moved[2*i+1];
    }
  }
  double condition = infinity;
  bool degenerate = true;
  if (observed.size() >= 20) {
    px /= observed.size(); py /= observed.size();
    double radius_squared = 0.0;
    for (std::size_t i : observed) {
      const double x = moved[2*i]-px, y = moved[2*i+1]-py;
      radius_squared += x*x + y*y;
    }
    const double scale = std::max(0.5, std::sqrt(radius_squared/observed.size()));
    double gram[3][3] = {};
    for (std::size_t i : observed) {
      const double nx = normals[2*nearest[i]], ny = normals[2*nearest[i]+1];
      const double jacobian[3] = {nx, ny,
          (-nx*(moved[2*i+1]-py) + ny*(moved[2*i]-px))/scale};
      for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b)
          gram[a][b] += jacobian[a]*jacobian[b]/observed.size();
    }
    double values[3]; eigenvalues3(gram, values);
    condition = values[2]/std::max(values[0], 1e-12);
    degenerate = values[0] < 0.01 || condition > 100.0;
  }
  const std::uint32_t reason = selected.size() < 30 ? 1 : overlap < 0.55 ? 2 :
      rmse > 0.10 ? 3 : degenerate ? 4 : 0;
  result->dx = tx; result->dy = ty; result->dyaw = std::atan2(rs, rc);
  result->rmse = rmse; result->overlap = overlap; result->condition = condition;
  result->inliers = static_cast<std::uint32_t>(selected.size());
  result->flags = (converged ? 1u : 0u) | (degenerate ? 2u : 0u) | (reason == 0 ? 4u : 0u);
  result->reason = reason;
  return 0;
}

}  // extern "C"
