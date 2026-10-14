#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "diff_drive_lab/command_buffer.hpp"

void check(double actual, double expected)
{
  if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-9) {
    throw std::runtime_error("Command check failed");
  }
}

int main()
{
  diff_drive_lab::CommandBuffer buffer(0.5, 2.0, 0.5);
  check(buffer.sample(0.0).v, 0.0);
  buffer.accept(5.0, -5.0, 10.0);
  check(buffer.sample(10.25).v, 0.5);
  check(buffer.sample(10.25).w, -2.0);
  check(buffer.sample(10.50).v, 0.5);
  check(buffer.sample(10.51).v, 0.0);
  buffer.accept(0.2, 0.0, 11.0);
  const bool ok = buffer.accept(std::numeric_limits<double>::quiet_NaN(), 0.0, 11.1);
  if (ok) { throw std::runtime_error("NaN was accepted"); }
  check(buffer.sample(11.2).v, 0.0);
  buffer.accept(0.3, 0.0, 12.0);
  check(buffer.sample(11.9).v, 0.0);
  buffer.clear();
  check(buffer.sample(12.1).v, 0.0);
  std::cout << "PASS: empty, limits, timeout boundary, NaN, time reversal, clear\n";
}