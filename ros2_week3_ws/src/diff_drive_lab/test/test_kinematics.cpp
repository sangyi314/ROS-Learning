#include <cmath>
#include <iostream>
#include <stdexcept>

#include "diff_drive_lab/diff_drive.hpp"

void check(double value, double expected, double tolerance = 1e-9)
{
  if (!std::isfinite(value) || std::abs(value - expected) > tolerance) {
    throw std::runtime_error("Numerical check failed");
  }
}

int main()
{
  using namespace diff_drive_lab;
  const double pi = std::acos(-1.0);
  DiffDrive robot(0.05, 0.30);
  auto straight = robot.inverse({0.20, 0.0});
  check(straight.left, 4.0);
  check(straight.right, 4.0);
  auto turn = robot.inverse({0.0, 1.0});
  check(turn.left, -3.0);
  check(turn.right, 3.0);
  auto arc = robot.inverse({0.20, 0.50});
  check(arc.left, 2.5);
  check(arc.right, 5.5);
  auto recovered = robot.forward(arc);
  check(recovered.v, 0.20);
  check(recovered.w, 0.50);

  for (int i = 0; i < 100; ++i) { robot.step(straight, 0.01); }
  check(robot.pose().x, 0.20);
  check(robot.pose().y, 0.0);
  robot.reset();
  robot.step(robot.inverse({0.0, pi / 2.0}), 1.0);
  check(robot.pose().x, 0.0);
  check(robot.pose().y, 0.0);
  check(robot.pose().yaw, pi / 2.0);
  robot.step(straight, 1.0);
  check(robot.pose().x, 0.0);
  check(robot.pose().y, 0.20);

  robot.reset();
  for (int i = 0; i < 200; ++i) { robot.step(arc, 0.01); }
  check(robot.pose().x, 0.4 * std::sin(1.0));
  check(robot.pose().y, 0.4 * (1.0 - std::cos(1.0)));
  check(robot.pose().yaw, 1.0);
  const auto limited = limit_wheels({10.0, 20.0}, 12.0);
  check(limited.left, 6.0);
  check(limited.right, 12.0);
  bool rejected = false;
  try { robot.step(straight, 0.0); }
  catch (const std::invalid_argument &) { rejected = true; }
  if (!rejected) { throw std::runtime_error("Zero dt was accepted"); }
  std::cout << "PASS: inverse/forward, straight, turn, arc, saturation, dt\n";
}