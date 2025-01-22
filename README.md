# FTC 2023-2024 Centerstage Robotics Code

This repository contains the codebase for our FTC 2023-2024 Centerstage competition season. Our robot uses advanced techniques for autonomous navigation and teleoperation, implementing a variety of libraries and tools to maximize performance.

## Features

### Autonomous Path Planning
- **RoadRunner Library**: The main framework for path planning, enabling smooth and accurate trajectories.
- **OpenCV Integration**: Used for object detection and computer vision tasks, including real-time target tracking and blob detection for precise positioning.
- **PID Control**: Provides stable and efficient movement of subsystems during autonomous operations.
- **State Machine Architecture**: Ensures reliable transitions between robot states, such as navigation, intake, and scoring.

### TeleOp
- **Field-Centric Control**: Allows intuitive robot control relative to the field using an IMU-based orientation system.
- **Multispeed Modes**: Switch between fast and slow speeds for optimized maneuverability.
- **Outtake State Machine**: Handles scoring operations with precision, including arm positioning, bucket tilting, and slide movement.
- **OpenCV for Vision-Assisted Scoring**: Real-time distance and position calculations enhance scoring efficiency.

### Hardware
- **Drivetrain**: Powered by mecanum wheels for omnidirectional movement.
- **Camera System**: Utilizes OpenCV and EasyOpenCV for vision processing.
- **Subsystems**: Includes an intake motor, servos for the arm and outtake, and slides for vertical movement.

