# Overview

This repo contains three standalone robotics behaviors and one integrated “obstacle course” runner for the **Elegoo Robot Car V4.0**.

## Communication model

All scripts share the same control pattern:

- **Camera**: capture frames using an HTTP endpoint  
  `http://192.168.4.1/capture`

- **Motion + sensors**: send JSON commands over a TCP socket  
  `192.168.4.1:100`

Each script uses:
- `capture()` to fetch and decode an image frame (OpenCV)
- `cmd()` to send a JSON command and read the car’s response
- `safe_cmd()` as a wrapper that attempts a reconnect once if the socket drops

## Standalone behaviors

### 1) Obstacle Avoidance (`obstacle_avoidance.py`)
Goal: drive forward until an obstacle is detected, then navigate around it.

Logic summary:
- Continuously measure forward ultrasonic distance.
- If distance is below a threshold (`dist_min`):
  1. Stop the car
  2. Rotate the ultrasonic sensor to the right and left (servo angles)
  3. Measure distances in both directions
  4. Choose the direction with more open space and turn
  5. If neither side is clear, back up briefly and retry

Key tunables:
- `dist_min` (how close an obstacle must be to trigger avoidance)
- scanning angles for the ultrasonic servo (`ang`)
- speed and turn timing

---

### 2) Blue Line Tracker (`blue_line_tracker.py`)
Goal: follow a blue line on the ground (e.g., painter’s tape) using camera vision.

Logic summary:
- Use the bottom half of the camera image as the ROI (where the floor is).
- Convert ROI to HSV and apply a blue color mask.
- Clean the mask with morphological operations.
- Find the largest blue contour and compute its centroid.
- Convert the centroid x-position into a normalized horizontal offset in `[-1, 1]`.
- Drive forward and apply small timed correction “pulses” left/right based on offset magnitude:
  - small offset → small correction pulse
  - large offset (corner) → stronger/longer correction pulse

Key tunables:
- blue HSV bounds (`lower_blue`, `upper_blue`)
- contour area threshold (ignore tiny detections)
- `base_speed`, `turn_speed`
- correction timing scaling and thresholds (`NEUTRAL_THRESHOLD`, `CORNER_OFFSET`)

---

### 3) Red/Green Tracker (`red_green_tracker.py`)
Goal: interpret red/green signs as STOP/GO commands.

Logic summary:
- Use the upper half of the camera image as the ROI (looking ahead).
- Detect red using two HSV ranges (red wraps hue in HSV).
- Detect green using one HSV range.
- Compute the largest blob area for each color.
- Behavior rules:
  - Red detected (and not green) → STOP mode
  - Green detected → GO mode (even if there’s minor red noise)

Key tunables:
- HSV bounds for red and green
- area threshold for “sign detected” (depends on sign size/distance)

## Integrated script: Obstacle Course (`obstacle_course.py`)

The obstacle course script combines behaviors with an explicit priority order.

### Priority / arbitration
1. **Lift-to-stop**: if the car is lifted off the ground, stop and exit
2. **Orange finish**: if an orange finish region is detected near the bottom of the frame, stop and exit
3. **Traffic mode**:
   - red → STOP overrides everything
   - green → GO allows motion behaviors
4. If GO:
   - **Obstacle avoidance** if something is close (`dist <= dist_min`)
   - otherwise **blue line following**

### Orange finish detection
- Looks for a large orange region in the lower part of the image.
- Uses an HSV orange mask + a large area threshold so it only triggers when the car is very close/on top of the finish area.

Key tunables:
- orange HSV bounds
- `ORANGE_AREA_THRESH` (how large the region must be)

---

## Calibration & tuning notes

This project is sensitive to:
- lighting conditions (HSV values change with lighting)
- camera angle and height (changes what appears in the ROI)
- tape/sign material and color saturation
- speed and correction timing (too fast → overshoot, too slow → wobble)

Common tuning workflow:
1. Start with slow speeds
2. Adjust HSV bounds until masks are stable
3. Adjust contour/area thresholds to avoid false positives
4. Increase speed once tracking/avoidance is stable

