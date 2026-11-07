#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace joystick {

// Same start/stop button: toggle. Separate buttons: explicit start and stop.
// A held button emits one command, and pressing the guard after the button
// does not accidentally trigger a command.
class RecordingButtons {
 public:
  std::optional<bool> Update(const std::vector<int32_t>& buttons,
                             int guard, int start, int stop) {
    auto down = [&](int index) {
      return index >= 0 && index < static_cast<int>(buttons.size()) && buttons[index] != 0;
    };
    const bool start_down = down(start), stop_down = down(stop);
    const bool start_pressed = start_down && !previous_start_;
    const bool stop_pressed = stop_down && !previous_stop_;
    previous_start_ = start_down;
    previous_stop_ = stop_down;
    if (guard >= 0 && !down(guard)) return std::nullopt;
    if (start == stop && start_pressed) {
      recording_ = !recording_;
    } else if (start != stop && stop_pressed) {
      recording_ = false;
    } else if (start != stop && start_pressed) {
      recording_ = true;
    } else {
      return std::nullopt;
    }
    return recording_;
  }

  void SetRecording(bool recording) { recording_ = recording; }

 private:
  bool previous_start_ = false;
  bool previous_stop_ = false;
  bool recording_ = false;
};

}  // namespace joystick
