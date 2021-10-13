#pragma once

#include <cute_c2.h>

#include <array>
#include <vector>

/// Length of the world, in metres.
constexpr float WORLD_LENGTH{2.0};

/// Length of an obstacle, in metres.
constexpr float OBSTACLE_LENGTH{0.1};

/**
 * Base class for all collision detectors.
 */
class CollisionDetector {
 protected:
  /// Polygonal shape representing the robot.
  c2Poly robot_shape;

 public:
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
   * Check if the robot has collided with any object in the play area.
   *
   * \param x x-coordinate of robot's reference point.
   * \param y y-coordinate of robot's reference point.
   * \param theta robot's rotation in radians.
   *
   * \retval true if a collision is detected.
   * \retval false if a collision is not detected.
   */
  virtual bool check_collision(float x, float y, float theta) const = 0;

  virtual ~CollisionDetector() = default;
};

/**
 * Collision detector for the image recognition task.
 *
 * Both obstacles and the world's boundaries are defined as objects that the
 * robot can collide with.
 *
 * The MDP "world" is defined to exist from \c (0, WORLD_LENGTH) to
 * \c (WORLD_LENGTH, WORLD_HEIGHT).
 *
 * Where \c (0, \c WORLD_LENGTH) is the bottom left corner of the play area, and
 * \c (WORLD_LENGTH, \c WORLD_HEIGHT) is the top right corner of the play area.
 */
class CollisionDetectorIR : public CollisionDetector {
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
  CollisionDetectorIR(float f2c, float b2c, float l2c, float r2c);

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

  bool check_collision(float x, float y, float theta) const override;
};

/**
 * Collision detector class that can be used to detect whether the robot has
 * collided with any elements in the MDP world.
 *
 * Collision detector for the fastest car task.
 */
class CollisionDetectorFC : public CollisionDetector {
 private:
  /// AABBs representing the obstacles used to create the carpark.
  static constexpr std::array<c2AABB, 3> carpark = {
      {{c2v{-0.1, -0.3}, c2v{0, 0.3}},
       {c2v{-0.1, -0.4}, c2v{0.5, -0.3}},
       {c2v{-0.1, 0.3}, c2v{0.5, 0.4}}}};
  /// AABB representing the obstacle.
  const c2AABB obstacle;
  const c2Ray left_barrier;
  const c2Ray right_barrier;
  const bool left_blocked;
  const bool right_blocked;

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
   * \param d distance from center of obstacle to edge of carpark.
   * \param block_left whether to block off the left side of the carpark exit.
   * \param block_right whether to block off the right side of the carpark
   *  exit.
   */
  CollisionDetectorFC(float f2c,
                      float b2c,
                      float l2c,
                      float r2c,
                      float d,
                      bool block_left,
                      bool block_right);

  bool check_collision(float x, float y, float theta) const override;
};
