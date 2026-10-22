"""Exercise the native actuator demand law without ROS or Gazebo."""

import shutil
import subprocess
from pathlib import Path

import pytest


@pytest.fixture(scope='module')
def servo_fixture(tmp_path_factory):
    """Compile the production scalar law into a standalone behavioral fixture."""
    header = (Path(__file__).parents[1] / 'robot_simulation' / 'gazebo' /
              'native' / 'servo_math.hh')
    assert header.is_file(), 'The production native servo law has not been implemented'
    compiler = shutil.which('g++')
    assert compiler, 'Native actuator verification requires g++'
    directory = tmp_path_factory.mktemp('native_servo')
    source = directory / 'fixture.cc'
    source.write_text(r"""
#include "servo_math.hh"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

int main(int argc, char **argv) {
  if (argc != 2) return 90;
  const std::string mode = argv[1];
  using robot_simulation::ServoVelocity;
  using robot_simulation::GovernedServoVelocity;
  auto close = [](double a, double b) { return std::abs(a-b) < 1e-12; };
  if (mode == "examples") {
    if (!close(ServoVelocity(0., .5, -1., 1., 1.5, .002), 1.5)) return 1;
    if (!close(ServoVelocity(0., -.5, -1., 1., 1.5, .002), -1.5)) return 2;
    if (!close(ServoVelocity(.999, 1., -1., 1., 1.5, .002), .5)) return 3;
    if (!close(ServoVelocity(-.999, -1., -1., 1., 1.5, .002), -.5)) return 4;
    if (ServoVelocity(.3, .3, -2., 2., .7, .004) != 0.) return 5;
    if (!close(ServoVelocity(0., .0001, -1., 1., 2., .002), .05)) return 6;
    if (!close(ServoVelocity(3., 2., -2., 2., .8, .1), -.8)) return 7;
    if (!close(ServoVelocity(-3., -2., -2., 2., .8, .1), .8)) return 8;
  } else if (mode == "bounds") {
    for (double vmax : {.25, 1., 1.5, 3.2}) {
      for (double dt : {.0005, .002, .02}) {
        for (int n = 0; n <= 400; ++n) {
          const double q = -2. + n * .01;
          for (double target : {-2., -.4, 0., .8, 2.}) {
            const double v = ServoVelocity(q, target, -2., 2., vmax, dt);
            if (!std::isfinite(v) || std::abs(v) > vmax + 1e-12) return 10;
            if (q + v * dt < -2. - 1e-12 || q + v * dt > 2. + 1e-12) return 11;
            if ((target - q) * v < -1e-12) return 12;
          }
        }
      }
    }
  } else if (mode == "governor") {
    double previous = 0.;
    for (int step = 0; step < 2000; ++step) {
      const double target = step < 1000 ? .8 : -.8;
      const auto next = GovernedServoVelocity(0., target, -1., 1., 1., .002,
                                             previous, .1, 4.);
      if (std::abs(next) > 1.+1e-12 || std::abs(next-previous) > .008+1e-12) return 30;
      previous = next;
    }
    if (!close(GovernedServoVelocity(0., .01, -1., 1., 1., .002, 0., .1, 100.), .1)) return 31;
    if (!close(GovernedServoVelocity(.9999, 1., -1., 1., 1., .002, 1., .1, 4.), .05)) return 32;
    for (double bad : {0., -1., std::numeric_limits<double>::quiet_NaN()}) {
      bool rejected = false;
      try { GovernedServoVelocity(0., .2, -1., 1., 1., .002, 0., bad, 4.); }
      catch (const std::invalid_argument &) { rejected = true; }
      if (!rejected) return 33;
    }
  } else if (mode == "invalid") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const double cases[][6] = {
      {nan, 0., -1., 1., 1., .002}, {0., inf, -1., 1., 1., .002},
      {0., 0., nan, 1., 1., .002}, {0., 0., -1., inf, 1., .002},
      {0., 0., -1., 1., nan, .002}, {0., 0., -1., 1., 1., inf},
      {0., 0., -1., 1., 1., 0.}, {0., 0., -1., 1., 1., -.1},
      {0., 0., -1., 1., 0., .002}, {0., 0., 1., -1., 1., .002},
      {0., 2., -1., 1., 1., .002}, {0., 0., 1., 1., 1., .002}
    };
    for (const auto &c : cases) {
      bool rejected = false;
      try { ServoVelocity(c[0], c[1], c[2], c[3], c[4], c[5]); }
      catch (const std::invalid_argument &) { rejected = true; }
      if (!rejected) return 20;
    }
  } else return 91;
  return 0;
}
""")
    binary = directory / 'fixture'
    result = subprocess.run(
        [compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
         '-I', str(header.parent), str(source), '-o', str(binary)],
        capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    return binary


@pytest.mark.parametrize('case', ['examples', 'bounds', 'invalid', 'governor'])
def test_native_servo_demand(servo_fixture, case):
    """Catch wrong sign, missing position/speed bounds and invalid-input acceptance."""
    result = subprocess.run([str(servo_fixture), case], check=False)
    assert result.returncode == 0, f'{case} native fixture failed: {result.returncode}'
