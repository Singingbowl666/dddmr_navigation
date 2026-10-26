#include <perception_3d/ground_reference.h>
#include <iostream>
#include <stdexcept>

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

int main()
{
  using namespace perception_3d;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground(new pcl::PointCloud<pcl::PointXYZI>);
  for (int x = -10; x <= 10; ++x) {
    for (int y = -10; y <= 10; ++y) {
      pcl::PointXYZI p; p.x = x * .1; p.y = y * .1; p.z = 0;
      ground->push_back(p);
    }
  }
  pcl::KdTreeFLANN<pcl::PointXYZI> tree;
  tree.setInputCloud(ground);
  for (const double height : {.45, .6, .9}) {
    GroundReference reference;
    require(estimateGroundReference(tree, *ground, {0, 0, height}, reference), "Ground not found");
    require(std::abs(reference.z) < 1e-6, "Ground height biased");
    pcl::PointCloud<pcl::PointXYZ> cuboid;
    for (const float x : {-.35f, .42f}) {
      for (const float y : {-.36f, .36f}) {
        for (const float z : {-.46f, .3f}) {cuboid.push_back(pcl::PointXYZ(x, y, z));}
      }
    }
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation().z() = height;
    require(alignCuboidBottom(cuboid, pose, reference, .05), "Cuboid alignment failed");
    float bottom = 100;
    for (const auto & p : cuboid) {bottom = std::min(bottom, p.z);}
    require(std::abs(bottom + height - .05) < 1e-5, "Cuboid lower face is not 5 cm above ground");
    require(.15 > bottom + height && .15 < height + .3, "Low obstacle outside collision envelope");
  }
  for (auto & p : *ground) {p.z = .2 * p.x - .1 * p.y - .3;}
  tree.setInputCloud(ground);
  GroundReference slope;
  require(estimateGroundReference(tree, *ground, {0, 0, .3}, slope), "Slope reference failed");
  require(std::abs(slope.heightAt(.4, .2) - (-.24)) < 1e-5, "Slope extrapolation incorrect");
  require(std::abs(mappedGroundHeight(slope, tree, *ground, .41, .21) - (-.239)) < 1e-5,
    "Mapped height lost local slope");
  GroundReference invalid;
  require(!estimateGroundReference(tree, *ground, {20, 20, .6}, invalid), "Distant ground accepted");
  require(!estimateGroundReference(tree, *ground, {0, 0, 3.0}, invalid), "Other floor accepted");
  // One bad ground-map point must not move the reference toward the robot.
  for (auto & p : *ground) {p.z = 0;}
  pcl::PointXYZI outlier; outlier.x = 0; outlier.y = 0; outlier.z = .45;
  ground->push_back(outlier); tree.setInputCloud(ground);
  require(estimateGroundReference(tree, *ground, {0, 0, .6}, invalid), "Outlier rejection failed");
  require(std::abs(invalid.z) < 1e-5, "Outlier biased ground reference");
  std::cout << "PASS ground height, varying body clearance, slopes, missing ground and outlier rejection\n";
}
