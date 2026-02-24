# Session Notes - 2026swervecode1

Last updated: 2026-02-24
Workspace: `c:\Users\RoboHerd 5110\Desktop\2024-25 code\2026swervecode1`

## Request Summary
- Add an AndyMark programmable servo to climber control.
- Update climber logic for ratchet-and-servo actuator kit behavior.
- Update markdown documentation to reflect current code and decisions.

## 2026 FRC Competition Snapshot
- Season theme: FIRST AGE.
- 2026 FRC game: REBUILT.
- Game objective (high level): alliances collect Fuel Pods and score them in structures such as Towers, Outposts, and Fuel Hubs, including converting Fuel Pods to Fuel and launching Fuel into targets.
- Match structure (high level): one autonomous period followed by teleoperated play.

## Competition Sources Used
- https://www.firstinspires.org/robotics/frc/game-and-season
- https://www.firstinspires.org/about/press-room/first-unveils-2026-first-robotics-competition-game-rebuilt
- https://www.frcmanual.com/2026/game-overview/

## Codebase Snapshot
- Framework: WPILib command-based robot (`Robot`, `RobotContainer`, subsystems).
- Language/tooling: Java 17, GradleRIO `2026.2.1`.
- Drivetrain stack: CTRE Phoenix 6 swerve (Tuner X generated constants + custom `CommandSwerveDrivetrain`).
- Vision stack: Limelight helpers + direct NetworkTables reads (`limelight/botpose_wpiblue`, `tv`, `tl`, `cl`) for pose correction.
- Mechanism stack:
  - REV Spark Flex: intake arm/roller, conveyor, shooter loader, shooter flywheel.
  - CTRE TalonFX + PWM servo: climber with ratchet latch support.

## Climber Ratchet/Servo Changes
- Added constants:
  - `Constants.PwmPorts.CLIMBER_LATCH_SERVO = 2`
  - `Constants.Climber.LATCH_ENGAGED_POSITION`
  - `Constants.Climber.LATCH_DISENGAGED_POSITION`
  - `Constants.Climber.LATCH_RELEASE_DELAY_SEC`
- `ClimberSubsystem` now owns a `Servo` for ratchet latch actuation.
- New climber API:
  - `engageLatch()`
  - `disengageLatch()`
  - `toggleLatch()`
  - `isLatchEngaged()`
- Behavioral update:
  - `stopAndHold()` now engages the ratchet latch and commands zero motor output.
  - `run(percent)` auto-disengages latch and waits release delay before powering the motor.
  - Startup defaults to engaged latch.

## Important Current State Notes
- `src/main/java/frc/robot/commands/` contains source-backed command files:
  - `AlignToClimbPostCommand`
  - `AlignToFuelHubCommand`
  - `ShootFromDistanceCommand`
- `RobotContainer` currently binds these commands and shooter test controls.
- `gradlew compileJava` passes after climber changes (with one TalonFX constructor deprecation warning).

## Files Reviewed This Session
- `build.gradle`
- `src/main/java/frc/robot/Main.java`
- `src/main/java/frc/robot/Robot.java`
- `src/main/java/frc/robot/RobotContainer.java`
- `src/main/java/frc/robot/FieldGeometry.java`
- `src/main/java/frc/robot/LimelightHelpers.java`
- `src/main/java/frc/robot/Telemetry.java`
- `src/main/java/frc/robot/generated/TunerConstants.java`
- `src/main/java/frc/robot/subsystems/*.java`

## Suggested Next Session Starting Point
1. Tune climber latch positions and release delay on robot hardware.
2. Add operator bindings for explicit latch engage/disengage override if needed.
3. Validate behavior under load at climb top: latch engage timing, no backdrive, safe descent behavior.
4. Add a brief operator checklist for climb sequence and ratchet safety interlocks.
