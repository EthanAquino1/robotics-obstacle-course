# Robotics Obstacle Course 

Python control + computer vision behaviors for the **Elegoo Smart Robot Car V4.0**.  
I built the robot, connected to its Wi-Fi network, and ran these scripts from **Spyder** to drive the car through an obstacle course.

This repo includes three standalone mini projects and one integrated obstacle course script:
- **Obstacle avoidance** (ultrasonic distance + scan and turn)
- **Blue line tracking** (HSV masking, centroid offset, correction pulses)
- **Red/green “traffic sign” detection** (HSV masking, stop/go mode)
- **Obstacle course** (combines everything, plus an orange “finish” detector)

## Demo
- Images of robot: `media/images/`
- Video demos of all behaviors and obstacle course run: `media/videos/`

## Hardware
- Elegoo Smart Robot Car V4.0
- ESP32 camera stream: `http://192.168.4.1/capture`
- TCP command socket: `192.168.4.1:100`
- Tested through Spyder IDE

## Project structure
- `src/behaviors/obstacle_avoidance.py` --> Creating a script for the robot to avoid obstacles and make decisions on which direction to go
- `src/behaviors/blue_line_tracker.py` --> Creating a script for the robot to track a blue line on the ground with shallow and sharp turns
- `src/behaviors/red_green_tracker.py` --> Creating a scrupt for the robot to react to red and green colors, causing it to either stop or go
- `src/obstacle_course.py` --> All behaviors integrated into one script to run through a simulated obstacle course
- `docs/setup.md` --> Setting up model + running scripts
- `docs/overview.md` --> Descriptions on how each behavior works and how it comes together in an obstacle course

## Installation
```bash
pip install -r requirements.txt
