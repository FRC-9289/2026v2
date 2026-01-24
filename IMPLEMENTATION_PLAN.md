# Implementation Plan: Joystick-Drivetrain Integration & Turret Subsystem

## Task 1: Joystick to Drivetrain Mapping
**Objective**: Map left joystick for translation (forward/backward/left/right) and right joystick for rotation

### Changes needed:
1. **RobotContainer.java** - Update `configureBindings()` to map:
   - Left Stick X (Axis 0) → Side Speed (strafing left/right)
   - Left Stick Y (Axis 1) → Front Speed (forward/backward)
   - Right Stick X/Rotation (Axis 2) → Turn Speed (robot rotation)

### Implementation Details:
- Keep the slider (Axis 3) for speed throttling
- Maintain deadband at 0.05
- Keep field-oriented driving enabled

---

## Task 2: Complete Turret Subsystem
**Objective**: Create a fully functional turret that can turn to face a specific angle in radians

### Files to Create/Modify:

### 2.1 Turret.java (Complete Rewrite)
- Add Spark MAX motor integration
- Implement PID position controller
- Add `setTargetAngle(double radians)` method
- Add `turnToPoint(double targetX, double targetY)` method using robot pose
- Add SmartDashboard widgets for:
  - Current angle (degrees and radians)
  - Target angle
  - Motor voltage/current
  - Coordinate input for tracking

### 2.2 Constants.java - Update TurretConstants
- Add default values for constants
- Add TURRET_OFFSET for zero position
- Add PID gains (KP, KI, KD)
- Add max speed/acceleration limits

### 2.3 Create TurretCommands.java
- `TurnToAngleCommand` - turns turret to specific angle
- `TurnToPointCommand` - turns turret to face field coordinates

### 2.4 RobotContainer.java
- Add Turret instantiation
- Configure default command for turret position tracking
- Add coordinate input from SmartDashboard

---

## Implementation Steps:

### Step 1: Update Constants.java
- Add turret-specific constants with placeholder/default values

### Step 2: Rewrite Turret.java
- Add Spark MAX motor control
- Implement PID position control
- Add target angle tracking
- Add angle calculation from coordinates

### Step 3: Create TurretCommands.java
- Command to turn turret to specific angle
- Command to turn turret toward field coordinates

### Step 4: Update RobotContainer.java
- Add Turret to subsystem list
- Configure bindings for turret control
- Add SmartDashboard widgets

### Step 5: Test and Validate
- Verify joystick mapping works correctly
- Verify turret responds to target angle commands
- Verify coordinate-based tracking works

---

## Notes:
- All constants have default/placeholder values to be changed later
- Turret uses NEO motor with CAN ID 10
- Robot pose from WolfPoseEstimator used for coordinate tracking
- Field-relative turret tracking by default

