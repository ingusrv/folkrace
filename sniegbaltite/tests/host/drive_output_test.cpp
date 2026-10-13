#include "drive_output.h"
#include <cassert>
#include <cstdio>

int main() {
  DriveOutput output(1.0f, 3.0f, 0.18f);
  WheelCommands wheels{};
  for (unsigned i = 0; i < 100; ++i)
    wheels = output.update(0.0f, 0.3f, true, false, 0.01f);
  assert(wheels.left > 0.4f &&
         wheels.right < -0.4f); // Pivot, throttle is zero.
  unsigned ticks = 0;
  while (!wheels.stopped()) {
    auto previous = wheels;
    // Inhibit cancels maneuver to RUNNING but still ramps the negative wheel.
    wheels = output.update(0.0f, 0.3f, false, true, 0.01f);
    assert(wheels.left >= 0.0f && wheels.right <= 0.0f);
    assert(previous.left - wheels.left <= 0.030001f);
    assert(wheels.right - previous.right <= 0.030001f);
    assert(++ticks < 30);
  }
  assert(ticks > 1); // Blocking recovery cannot begin after only one tick.
  wheels = output.update(0.6f, 0.1f, false, false, 0.01f);
  assert(wheels.left > 0.0f && wheels.left <= 0.010001f);
  assert(wheels.right > 0.0f && wheels.right <= 0.010001f);
  for (unsigned i = 0; i < 100; ++i)
    wheels = output.update(0.2f, 0.6f, false, false, 0.01f);
  assert(wheels.right ==
         0.0f); // No reverse inner wheel during forward running.
  output.reset();
  assert(output.update(0.0f, 0.3f, true, true, 0.01f).stopped());
  puts("Banana final wheel inhibit, pivot deceleration, and resume tests "
       "passed");
}
