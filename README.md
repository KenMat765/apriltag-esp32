# AprilTag Arduino ESP32 library

AprilTag is a visual fiducial system popular in robotics research. This repository contains the most recent version of AprilTag Arduino ESP32 library.

# Changes added to original AprilTag library

- Convert structure to Arduino library's
- Change all `double` to `float` (ESP32 only natively support single-precision math)
- Reduce the max number of tags for families like `36h10` and `36h11` to fit ESP32's memory (currently to 35 tags for each of these family). See commit https://github.com/raspiduino/apriltag-esp32/commit/59468385da1e9fff37c0f4e9a8152c1ef546523e for more information. You may modify the number to fit your need, but please check the memory usage.

# Current status, plans,...

See https://github.com/raspiduino/apriltag-esp32/discussions/1
