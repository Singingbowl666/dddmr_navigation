#ifndef PERCEPTION_3D_GROUND_REFERENCE_H_
#define PERCEPTION_3D_GROUND_REFERENCE_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

namespace perception_3d
{

struct GroundReference
{
  double x = 0.0, y = 0.0, z = 0.0;
  double slope_x = 0.0, slope_y = 0.0;

  double heightAt(double px, double py) const
  {
    return z + slope_x * (px - x) + slope_y * (py - y);
  }
};

// Callers hold the ground-cloud mutex. Use only a small, nearby patch below
// the robot so distant terrain and other floors cannot set body clearance.
inline bool estimateGroundReference(
  pcl::KdTreeFLANN<pcl::PointXYZI> & tree,
  const pcl::PointCloud<pcl::PointXYZI> & ground,
  const Eigen::Vector3d & base, GroundReference & result,
  double xy_radius = 0.6, double max_base_height = 1.0)
{
  if (ground.empty() || !base.allFinite() || xy_radius <= 0.0 || max_base_height <= 0.1) {
    return false;
  }
  pcl::PointXYZI query;
  query.x = base.x(); query.y = base.y(); query.z = base.z();
  std::vector<int> indices;
  std::vector<float> distances;
  tree.radiusSearch(query, std::hypot(xy_radius, max_base_height), indices, distances);
  std::vector<std::pair<double, int>> nearby;
  for (const int i : indices) {
    const auto & p = ground.points[i];
    const double dxy = std::pow(p.x - base.x(), 2) + std::pow(p.y - base.y(), 2);
    const double drop = base.z() - p.z;
    if (dxy <= xy_radius * xy_radius && drop >= 0.1 && drop <= max_base_height) {
      nearby.emplace_back(dxy, i);
    }
  }
  std::sort(nearby.begin(), nearby.end());
  if (nearby.size() > 12) {nearby.resize(12);}
  if (nearby.size() < 3) {return false;}

  Eigen::MatrixXd a(nearby.size(), 3);
  Eigen::VectorXd z(nearby.size());
  for (size_t i = 0; i < nearby.size(); ++i) {
    const auto & p = ground.points[nearby[i].second];
    a.row(i) << p.x - base.x(), p.y - base.y(), 1.0;
    z[i] = p.z;
  }
  auto qr = a.colPivHouseholderQr();
  if (qr.rank() < 3) {return false;}
  Eigen::Vector3d fit = qr.solve(z);
  // Refit after rejecting isolated map/segmentation outliers.
  std::vector<int> inliers;
  for (Eigen::Index i = 0; i < z.size(); ++i) {
    if (std::abs(a.row(i).dot(fit) - z[i]) <= 0.08) {inliers.push_back(i);}
  }
  if (inliers.size() < 3) {return false;}
  if (inliers.size() != nearby.size()) {
    Eigen::MatrixXd clean_a(inliers.size(), 3);
    Eigen::VectorXd clean_z(inliers.size());
    for (size_t i = 0; i < inliers.size(); ++i) {
      clean_a.row(i) = a.row(inliers[i]); clean_z[i] = z[inliers[i]];
    }
    auto clean_qr = clean_a.colPivHouseholderQr();
    if (clean_qr.rank() < 3) {return false;}
    fit = clean_qr.solve(clean_z);
  }
  const double height = base.z() - fit[2];
  if (!fit.allFinite() || std::hypot(fit[0], fit[1]) > 1.0 ||
    height < 0.1 || height > max_base_height)
  {
    return false;
  }
  result.x = base.x(); result.y = base.y(); result.z = fit[2];
  result.slope_x = fit[0]; result.slope_y = fit[1];
  return true;
}

// Follow mapped changes in terrain height at the point's XY position. Anchor
// the local slope at the nearest supported map point to limit voxel artifacts.
inline double mappedGroundHeight(
  const GroundReference & reference, pcl::KdTreeFLANN<pcl::PointXYZI> & tree,
  const pcl::PointCloud<pcl::PointXYZI> & ground, double x, double y)
{
  const double predicted = reference.heightAt(x, y);
  pcl::PointXYZI query;
  query.x = x; query.y = y; query.z = predicted;
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  if (tree.nearestKSearch(query, 1, indices, distances) > 0) {
    const auto & p = ground.points[indices[0]];
    if (std::hypot(p.x - x, p.y - y) <= 0.45 && std::abs(p.z - predicted) <= 0.5) {
      return p.z + reference.slope_x * (x - p.x) + reference.slope_y * (y - p.y);
    }
  }
  return predicted;
}

// Extend the lower face to just above ground, retaining the configured upper
// face and XY envelope. A conservative flat lower face also covers tilted legs.
inline bool alignCuboidBottom(
  pcl::PointCloud<pcl::PointXYZ> & cuboid, const Eigen::Affine3d & pose,
  const GroundReference & ground, double clearance)
{
  if (cuboid.empty() || clearance < 0.0) {return false;}
  float old_bottom = std::numeric_limits<float>::infinity();
  float top = -std::numeric_limits<float>::infinity();
  for (const auto & p : cuboid) {
    old_bottom = std::min(old_bottom, p.z); top = std::max(top, p.z);
  }
  const auto axis = pose.linear().col(2);
  const double denominator = axis.z() - ground.slope_x * axis.x() - ground.slope_y * axis.y();
  if (denominator <= 0.3) {return false;}
  double bottom = std::numeric_limits<double>::infinity();
  for (const auto & p : cuboid) {
    if (std::abs(p.z - old_bottom) > 1e-5) {continue;}
    const Eigen::Vector3d q = pose * Eigen::Vector3d(p.x, p.y, 0.0);
    const double z = (clearance - (q.z() - ground.heightAt(q.x(), q.y()))) / denominator;
    bottom = std::min(bottom, z);
  }
  if (!std::isfinite(bottom) || bottom >= top - 0.01) {return false;}
  for (auto & p : cuboid) {
    if (std::abs(p.z - old_bottom) <= 1e-5) {p.z = bottom;}
  }
  return true;
}

}  // namespace perception_3d
#endif
