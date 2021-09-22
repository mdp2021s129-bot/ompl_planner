#define CUTE_C2_IMPLEMENTATION
#include <array>
#include <collision.hpp>
#include <vector>

constexpr std::array<c2Ray, 4> CollisionDetector::edges;

CollisionDetector::CollisionDetector(float f2c,
                                     float b2c,
                                     float l2c,
                                     float r2c) {
  // Robot initially centered around (0, 0), with rotation = 0.
  robot_shape.count = 4;
  robot_shape.verts[0] = c2v{f2c, l2c};
  robot_shape.verts[1] = c2v{f2c, -r2c};
  robot_shape.verts[2] = c2v{-b2c, -r2c};
  robot_shape.verts[3] = c2v{-b2c, l2c};
  c2MakePoly(&robot_shape);
}

void CollisionDetector::add_obstacle(float x, float y) {
  c2v obs_ctr{x, y};
  c2v obs_offset{OBSTACLE_LENGTH / 2, OBSTACLE_LENGTH / 2};
  c2AABB obs{c2Sub(obs_ctr, obs_offset), c2Add(obs_ctr, obs_offset)};

  obstacles.push_back(obs);
}

void CollisionDetector::clear_obstacles() {
  obstacles.clear();
}

bool CollisionDetector::check_collision(float x, float y, float theta) const {
  // Transform for robot shape.
  c2x xfrm{c2v{x, y}, c2Rot(theta)};

  // Check collisions against bounding edges.
  for (const auto r : edges) {
    c2Raycast out;
    if (c2RaytoPoly(r, &robot_shape, &xfrm, &out))
      return true;
  }

  // Check collisions against obstacles.
  for (const auto o : obstacles) {
    if (c2AABBtoPoly(o, &robot_shape, &xfrm))
      return true;
  }

  return false;
}
