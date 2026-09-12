# Control Structure Draft

This project should keep Pedro Pathing as the autonomous drivetrain owner. Pedro handles localization, path following, heading correction, centripetal correction, and drivetrain power output. Team code should own the robot-level sequencing around it.

## Layers

1. OpMode
   - Builds subsystems.
   - Builds the autonomous routine.
   - Calls routine update and subsystem update every loop.
   - Owns telemetry that helps drivers debug the current routine.

2. Subsystems
   - Wrap one robot mechanism each.
   - Expose high-level methods such as `follow(path)`, `openClaw()`, `setArmTarget(...)`.
   - Hide hardware names and low-level motor/servo details.

3. Commands
   - Small units of action with `start`, `update`, `isFinished`, and `stop`.
   - Can be sequenced so autonomous reads like a timeline.
   - `FollowPathCommand` starts Pedro once and finishes when `follower.isBusy()` becomes false.

4. Pedro Constants
   - `Constants.createFollower(hardwareMap)` remains the single drivetrain construction point.
   - The swerve pod constants live there until calibration is finished.
   - `PatchedCoaxialPod` handles the 360 encoder degrees to 240 wheel degrees correction.

## Autonomous Loop

The intended autonomous loop is:

```java
routine.update();
subsystems.update();
telemetry.update();
```

For a Pedro path command:

```java
start: follower.followPath(path, maxPower, holdEnd)
update: follower.update()
done: !follower.isBusy()
stop: follower.breakFollowing() if interrupted
```

## Why Not Commit To A Framework Yet?

NextFTC and Ivy are both command systems. They are useful once the drivetrain is physically calibrated, but changing frameworks while the swerve module math is still moving adds noise. The current draft gives us the same shape with very little dependency risk.

When the robot drives reliably, we can replace the small local command classes with Ivy commands or NextFTC commands.
