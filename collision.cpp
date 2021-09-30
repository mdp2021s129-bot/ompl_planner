#define CUTE_C2_IMPLEMENTATION
#include <array>
#include <collision.hpp>
#include <vector>

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

constexpr std::array<c2Ray, 4> CollisionDetectorIR::edges;

CollisionDetectorIR::CollisionDetectorIR(float f2c,
                                         float b2c,
                                         float l2c,
                                         float r2c)
    : CollisionDetector{f2c, b2c, l2c, r2c} {}

void CollisionDetectorIR::add_obstacle(float x, float y) {
  c2v obs_ctr{x, y};
  c2v obs_offset{OBSTACLE_LENGTH / 2, OBSTACLE_LENGTH / 2};
  c2AABB obs{c2Sub(obs_ctr, obs_offset), c2Add(obs_ctr, obs_offset)};

  obstacles.push_back(obs);
}

void CollisionDetectorIR::clear_obstacles() {
  obstacles.clear();
}

bool CollisionDetectorIR::check_collision(float x, float y, float theta) const {
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

constexpr std::array<c2AABB, 3> CollisionDetectorFC::carpark;

CollisionDetectorFC::CollisionDetectorFC(float f2c,
                                         float b2c,
                                         float l2c,
                                         float r2c,
                                         float d)
    : CollisionDetector{f2c, b2c, l2c, r2c},
      obstacle{c2v{0.5f + d, -0.30}, c2v{0.6f + d, 0.30}} {
  // Robot initially centered around (0, 0), with rotation = 0.
  robot_shape.count = 4;
  robot_shape.verts[0] = c2v{f2c, l2c};
  robot_shape.verts[1] = c2v{f2c, -r2c};
  robot_shape.verts[2] = c2v{-b2c, -r2c};
  robot_shape.verts[3] = c2v{-b2c, l2c};
  c2MakePoly(&robot_shape);
}

/**
 * Check if the robot has collided with any object in the play area.
 *
 * \param x x-coordinate of robot's reference point.
 * \param y y-coordinate of robot's reference point.
 * \param theta robot's rotation in radians.
 *
 * \retval true if a collision is detected.
 * \retval false if a collision is not detected.
 */
bool CollisionDetectorFC::check_collision(float x, float y, float theta) const {
  // Transform for robot shape.
  c2x xfrm{c2v{x, y}, c2Rot(theta)};

  // Check collisions against carpark obstacles.
  for (const auto o : carpark) {
    if (c2AABBtoPoly(o, &robot_shape, &xfrm))
      return true;
  }

  // Check collision against main obstacle.
  if (c2AABBtoPoly(obstacle, &robot_shape, &xfrm))
    return true;

  return false;
}
