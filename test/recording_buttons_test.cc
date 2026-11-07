#include "recording_buttons.h"

#include <stdexcept>

void Require(bool condition) {
  if (!condition) throw std::runtime_error("recording button regression");
}

int main() {
  joystick::RecordingButtons dxs;
  Require(!dxs.Update({0}, 0, 0, 0));
  Require(dxs.Update({1}, 0, 0, 0) == true);
  Require(!dxs.Update({1}, 0, 0, 0));  // holding cannot repeat the toggle
  Require(!dxs.Update({0}, 0, 0, 0));
  Require(dxs.Update({1}, 0, 0, 0) == false);
  dxs.SetRecording(true);  // synchronize after an external /record command
  dxs.Update({0}, 0, 0, 0);
  Require(dxs.Update({1}, 0, 0, 0) == false);

  joystick::RecordingButtons ps4;
  Require(!ps4.Update({0, 0, 1}, 0, 1, 2));  // guard is not held
  Require(!ps4.Update({1, 0, 1}, 0, 1, 2));  // guard after held button is not an edge
  ps4.Update({1, 0, 0}, 0, 1, 2);
  Require(ps4.Update({1, 1, 0}, 0, 1, 2) == true);
  ps4.Update({1, 0, 0}, 0, 1, 2);
  Require(ps4.Update({1, 1, 0}, 0, 1, 2) == true);  // separate start is idempotent
  ps4.Update({1, 0, 0}, 0, 1, 2);
  Require(ps4.Update({1, 1, 1}, 0, 1, 2) == false);  // stop takes priority
  Require(!ps4.Update({}, 4, 2, 1));  // disconnected/missing buttons do not crash
}
