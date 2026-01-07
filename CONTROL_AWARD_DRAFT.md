# Control Award - Team 27598 SJ Shogun

## Overview
Our robot features a fully integrated autonomous ball-handling system combining vision processing, sensor feedback, and intelligent mechanisms. We reduced driver workload by automating the entire sorting pipeline while maintaining precise control through custom software architecture.

## 1. Integrated Control Architecture

### Drivetrain & Movement System
- **Hardware**: 4 mecanum wheels enabling omnidirectional movement
- **Software**: Abstracted complex mecanum drive kinematics into a single `Actuation.drive()` method usable in both Autonomous and TeleOp
- **Impact**: Reduced 38 lines of raw motor control code to 2 lines (1 initialization + 1 driving call)
- **Features**: Automatic power normalization, deadzone filtering (0.05 threshold), and field-centric control support

### Three-Wheel Odometry System
- **Hardware**: 2 lateral encoders + 1 perpendicular encoder with IMU integration
- **Algorithm**: Curvature-based position tracking using rotation matrices for accurate localization
- **Accuracy**: Provides real-time pose estimation (x, y, heading) for autonomous navigation and driver assistance

### Pure Pursuit Path Following
- **Implementation**: Lookahead-based path following algorithm for smooth autonomous trajectories
- **Waypoint System**: Route class enables sequential pose navigation with automatic tolerance checking (±2 inches position, ±3° heading)
- **Integration**: Combined with odometry for closed-loop autonomous control

## 2. Vision-Guided Precision

### Limelight 3A Integration
- **Capability**: AprilTag detection and tracking using Limelight 3A vision processor
- **Pipelines**: 3 pre-configured pipelines (Obelisk tags 21/22/23, Blue goals, Red goals)
- **Auto-Aiming**: Proportional controller (kP=0.015) enables turret auto-tracking of AprilTags
- **Feedback**: Haptic rumble feedback when aligned within ±3.0° tolerance

### Turret Control System
- **Hardware**: Single motor with encoder (1538 tick range, 700 tick working range)
- **Dual Modes**:
  1. Manual control via joystick for human override
  2. Auto-aiming using AprilTag tracking for precise scoring
- **Benefit**: Allows aiming while driving, dramatically improving response time and accuracy

## 3. Automated Ball Management System

### The Integration Challenge
The DECODE season requires sorting balls by color to score pattern points. Manual sorting under pressure is error-prone and slow. Our solution: complete automation.

### Color Sensor Intelligence
- **Detection**: Identifies Green (G) and Purple (P) balls using RGB thresholds (>500)
- **Logic**: Compares color values and classifies balls in real-time
- **Integration**: Triggers automated responses across multiple subsystems

### Intake System
- **Hardware**: Single motor (reversed direction)
- **Smart Intake Mode**: Automatically coordinates with Color sensor and Sorter
  - Detects ball entering intake
  - Advances sorter to next empty port
  - Stores ball color in sorter memory
  - Blocks additional intake when sorter is full (3 balls)
- **Impact**: Zero manual sorting required during TeleOp

### Sorter (3-Port Carousel)
- **Hardware**: Single motor with encoder
- **Precision**: Rotates 128.66 ticks per port (3 ports total)
- **Memory System**: Tracks which color ball is in each port
- **Automation**: Automatically rotates to position correct ball for shooting
- **Pattern Scoring**: Can shoot motif patterns (GPP, PGP, PPG) without driver input

### Flywheel Launcher
- **Hardware**: Single motor with encoder
- **Control**: PID + feedforward velocity controller
  - Tuned gains: kP=0.005, kI=0, kD=0, kF=0.0006
  - Anti-windup protection (integral clamp at 1.0)
- **Dynamic Adjustment**: Changes speed based on distance to target
- **Reliability**: `isAtSpeed()` method ensures flywheel reaches target velocity before shooting

### Tickle/Flicker Mechanism
- **Hardware**: Dual servo actuator
- **Function**: Transfers ball from sorter to flywheel
- **Positions**: 3 preset positions (fully extended 0.75, retracted 0, ball-blocking 0.067)
- **Operation**: Quick flick motion launches ball into flywheel path

## 4. Software Architecture Excellence

### Modular Design
Our codebase separates concerns into reusable static tool classes:
- **Movement.java**: Mecanum drive kinematics with power normalization
- **Odometry.java**: Position tracking with encoder and IMU fusion
- **Intake, Sorter, Turret, Flywheel, Tickle**: Hardware abstraction for game mechanisms
- **RobotMovement.java**: Pure Pursuit algorithm implementation
- **HardwareMapper.java**: Centralized motor configuration

### Actuation Hub Pattern
`Actuation.setup()` provides one-line initialization of all robot systems:
```java
Actuation.setup(hardwareMap, startPose, telemetry);
// Initializes: motors, odometry, intake, sorter, turret, tickle, flywheel
```

### Benefits
- **Rapid Development**: New OpModes require minimal boilerplate
- **Code Reuse**: Same control code works in both Auto and TeleOp
- **Easy Testing**: Individual systems testable in isolation
- **Maintainability**: Clear separation between hardware and control logic

## 5. Advanced Features

### Real-Time Telemetry & Visualization
- **ASCII Rendering System**: Custom 2D character buffer for Driver Station display
- **Field Map Rendering**: 144×144 inch field visualization with robot position
- **Bounding Box Renderer**: Displays robot outline with real-time rotation
- **Live Debugging**: Encoder values, pose data, and system states visible during operation

### PID Tuning Infrastructure
- **FlywheelPIDTuner.java**: Interactive tuning interface via gamepad
- **Live Adjustment**: Modify kP, kI, kD, kF gains during robot operation
- **Instant Feedback**: Real-time velocity and error telemetry
- **Persistence**: Save tuned parameters directly to Flywheel class

### Custom Data Types
Purpose-built classes for robotic control:
- **Pose**: Robot position with heading (x, y, θ)
- **CurvePoint**: Waypoint with path-following metadata (speed, heading, follow distance)
- **Matrix**: 2D matrix operations for odometry transformations
- **InputState**: Gamepad state snapshots for button edge detection

## 6. Results & Impact

### Competition Performance
- **Autonomous Reliability**: Pure Pursuit navigation successfully executes complex paths
- **TeleOp Efficiency**: Automated ball sorting eliminates driver cognitive load
- **Scoring Speed**: Combined vision tracking + automated sorting maximizes points per cycle
- **Driver Assistance**: Haptic feedback and auto-aiming reduce human error

### Code Metrics
- **Lines of Code**: ~3,000+ lines of custom team code
- **Abstraction Gain**: 19x reduction in drive control code (38 lines → 2 lines)
- **Reusability**: Core systems used across 15+ OpModes (Auto, TeleOp, testing)
- **Testing**: 10+ dedicated test OpModes for subsystem validation

### Key Innovation
**The fully automated intake-to-launch pipeline is our signature achievement.** From the moment a ball enters the intake, our robot automatically:
1. Detects the ball color
2. Advances the sorter to the next port
3. Stores the ball with its color in memory
4. Tracks which balls are available
5. Positions the correct ball for shooting
6. Prevents intake overflow

This eliminates manual sorting entirely, allowing drivers to focus on strategy and positioning rather than mechanism micromanagement.

## 7. Technical Challenges Overcome

### Challenge 1: Encoder-Based Positioning
- **Problem**: Sorter and Turret required precise positioning without limit switches
- **Solution**: Encoder-based state tracking with tick counting
- **Validation**: Reset procedures and manual calibration OpModes

### Challenge 2: Mecanum Drive Complexity
- **Problem**: 4-wheel mecanum math is non-intuitive and error-prone
- **Solution**: Abstracted kinematics into Movement class with automatic normalization
- **Testing**: Comprehensive RunTest.java validated all drive vectors

### Challenge 3: Vision Processing Latency
- **Problem**: Limelight processing introduces lag in tracking loop
- **Solution**: Proportional control with tuned gains balances responsiveness and stability
- **Refinement**: AprilTagTest.java used for iterative gain tuning

### Challenge 4: Ball Detection Reliability
- **Problem**: Color sensor readings vary with lighting and ball position
- **Solution**: Threshold-based RGB comparison (>500) proved robust across conditions
- **Testing**: colorTest.java and IntakeTest.java validated detection logic

## Conclusion
Our control system demonstrates the power of software abstraction and intelligent integration. By combining sensor feedback, vision processing, and automated mechanisms, we created a robot that performs complex tasks autonomously while remaining simple for drivers to operate. Our modular architecture enabled rapid iteration and reliable competition performance, exemplifying the engineering design process at its best.

---

**Total Subsystems**: 7 major mechanisms (drivetrain, odometry, intake, sorter, turret, flywheel, tickle)
**Sensors Integrated**: Color sensor, Limelight 3A, IMU, 6 motor encoders
**Lines of Custom Code**: 3,000+
**OpModes Developed**: 15+ (6 competition modes, 9 test/tuning modes)
