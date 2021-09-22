#pragma once

#include <cute_c2.h>

#include <array>
#include <vector>

/// Length of the world, in metres.
constexpr float WORLD_LENGTH{2.0};

/// Length of an obstacle, in metres.
constexpr float OBSTACLE_LENGTH{0.1};

/**
 * Collision detector class that can be used to detect whether the robot has
 * collided with any elements in the MDP world.
 *
 * Both obstacles and the world's boundaries are defined as objects that the
 * robot can collide with.
 *
 * The MDP "world" is defined to exist from \c (0, WORLD_LENGTH) to
 * \c (WORLD_LENGTH, WORLD_HEIGHT).
 *
 * Where \c (0, WORLD_LENGTH) is the bottom left corner of the play area, and
 * \c (WORLD_LENGTH, WORLD_HEIGHT) is the top right corner of the play area.
 */
class CollisionDetector {
 private:
  /// Rays representing the edges of the play area.
  static constexpr std::array<c2Ray, 4> edges = {
      // Bottom edge
      c2Ray{c2v{0., 0.}, c2v{1., 0.}, WORLD_LENGTH},
      // Top edge
      c2Ray{c2v{0., WORLD_LENGTH}, c2v{1., 0.}, WORLD_LENGTH},
      // Left edge
      c2Ray{c2v{0., 0.}, c2v{0., 1.}, WORLD_LENGTH},
      // Right edge
      c2Ray{c2v{WORLD_LENGTH, 0.}, c2v{0., 1.}, WORLD_LENGTH}};
  /// Polygonal shape representing the robot.
  c2Poly robot_shape;
  /// AABBs representing the obstacles.
  std::vector<c2AABB> obstacles;

 public:
  // Copy construction and copy-assign are all okay.
  // Move construction and move-assign are all okay.

  /**
   * Create a new collision detector.
   *
   * \param f2c distance from front of robot to centre.
   * \param b2c distance from back of robot to centre.
   * \param l2c distance from left of robot to centre.
   * \param r2c distance from right of robot to centre.
   */
  CollisionDetector(float f2c, float b2c, float l2c, float r2c);

  /**
   * Add an obstacle to the collision detector.
   *
   * \param x x-coordinate of obstacle's centre.
   * \param y y-coordinate of obstacle's centre.
   */
  void add_obstacle(float x, float y);

  /**
   * Clear all obstacles stored in the collision detector.
   */
  void clear_obstacles();

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
  bool check_collision(float x, float y, float theta) const;
};
