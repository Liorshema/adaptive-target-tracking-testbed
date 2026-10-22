// Copyright 2026 Adaptive Target Tracking Testbed contributors.
// Licensed under the Apache License, Version 2.0.
#ifndef ROBOT_SIMULATION_SERVO_MATH_HH_
#define ROBOT_SIMULATION_SERVO_MATH_HH_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_simulation
{
/// Compute a motor velocity demand, never overwrite physical joint state.
/// If a joint is already farther outside its position interval than one bounded
/// step can recover, request bounded inward motion. A feasible one-step position
/// interval does not exist in that case; do not invent an excessive speed.
inline double ServoVelocity(double position, double target, double lower,
                            double upper, double speed, double dt)
{
  if (!std::isfinite(position) || !std::isfinite(target) ||
      !std::isfinite(lower) || !std::isfinite(upper) ||
      !std::isfinite(speed) || !std::isfinite(dt) ||
      lower >= upper || speed <= 0.0 || dt <= 0.0 ||
      target < lower || target > upper)
    throw std::invalid_argument("Invalid native actuator state, target or limits");
  const double minimum = std::max(-speed, (lower - position) / dt);
  const double maximum = std::min(speed, (upper - position) / dt);
  if (minimum > maximum)
    return position < lower ? speed : -speed;
  return std::clamp((target - position) / dt, minimum, maximum);
}

/// Shape motor demand without assuming a bound on physical acceleration.
/// Position protection takes priority if the rate-limited demand is infeasible.
inline double GovernedServoVelocity(double q, double target, double lower,
                                   double upper, double speed, double dt,
                                   double previous, double timeConstant,
                                   double acceleration)
{
  if (!std::isfinite(previous) || std::abs(previous) > speed + 1e-12 ||
      !std::isfinite(timeConstant) || timeConstant <= 0.0 ||
      !std::isfinite(acceleration) || acceleration <= 0.0)
    throw std::invalid_argument("Invalid actuator demand governor");
  // Validate all physical inputs through the base law before computing the goal.
  ServoVelocity(q, target, lower, upper, speed, dt);
  const double goal = std::clamp((target - q) / timeConstant, -speed, speed);
  const double ramp = std::clamp(goal, previous - acceleration * dt,
                               previous + acceleration * dt);
  const double lo = std::max(-speed, (lower - q) / dt);
  const double hi = std::min(speed, (upper - q) / dt);
  if (lo > hi) return q < lower ? speed : -speed;
  return std::clamp(ramp, lo, hi);
}
}  // namespace robot_simulation
#endif  // ROBOT_SIMULATION_SERVO_MATH_HH_
