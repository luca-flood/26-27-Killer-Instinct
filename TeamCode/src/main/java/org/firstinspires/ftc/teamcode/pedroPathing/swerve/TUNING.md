# Swerve Calibration and Tuning

The four named tuners are adapted from official Pedro Quickstart commit
0470fddb3423725e5aaa018664e8bf0ec899c78d. They were commented out upstream and
removed in 55c366f. The restored versions use the current v3 API and our hardware
map. Swerve Angle Tracking Test is an additional passive diagnostic for our ratio.
No FTC SDK or Gradle versions were changed.

## 1. Analog Min / Max Tuner

Upload with Android Studio's green Run triangle using the TeamCode run configuration
and connected Control Hub. Select this TeleOp in the Pedro Swerve group.
Lift the robot and clear the moving pods. START automatically runs all steering
servos at +0.2 for approximately five wheel turns, stops each individually, pauses
0.5 seconds after all finish, then repeats at -0.2. Drive motors remain off.
B or STOP aborts. Isolated implausible jumps are rejected without updating the
last accepted angle or timestamp, or contaminating the voltage extrema. Telemetry
shows the rejected reading, its time interval and allowed jump. A 90-second
per-direction timeout, invalid voltage, or 30 ms without a trustworthy sample
stops all outputs and marks results incomplete.
Wait for COMPLETE and record the combined min/max voltage for:

| Pod | Analog input |
| --- | --- |
| Front left | frontLeft2 |
| Front right | frontRight2 |
| Back left | backLeft2 |
| Back right | backRight2 |

The measurements are raw AnalogInput voltage, not supplier-derived angles.
They are observed extrema, not guaranteed electrical endpoints. Revolution counting
uses the nominal 3.3 V range and the assumed 240 wheel degrees per encoder revolution
(five wheel turns = 7.5 encoder turns). Sweep lengths are approximate before calibration.
Stop immediately if a pod binds or wires wind up. Do not assist moving pods by hand.
The completed five-turn bidirectional sweep measured these bounds, now stored in
Constants.frontLeft/frontRight/backLeft/backRight PodCalibration values:

| Pod | Min V | Max V |
| --- | --- | --- |
| Front right | 0.036 | 3.211 |
| Front left | 0.034 | 3.228 |
| Back right | 0.036 | 3.239 |
| Back left | 0.028 | 3.251 |

## 2. Passive Angle and Offset Check

Run Swerve Angle Tracking Test. Physically align all wheels forward before START.
Record rawAngleRad (encoder radians) at forward after applying voltage calibration,
then set the corresponding straightEncoderDeg to Math.toDegrees(rawAngleRad).
Restart after changing calibration. Rotate manually through 90, 180, 270 and 360
wheel degrees, and through encoder wraps in BOTH directions. Verify the wheel angle
is continuous modulo 360, and continuousEncoderAngleDeg accumulates signed turns.
Report rawVoltage, rawAngleDeg, angleAfterOffsetDeg, loop ms and rejectedAngleSpikes.

One encoder revolution equals 240 wheel degrees. An encoder's absolute reading
cannot identify its full revolution count after restart. Align wheels forward at
startup; software cannot recover multi-turn position from one voltage sample.
The nearest straight encoder branch is chosen at startup. At the fixed 6000
encoder deg/s rejection bound, sampling must remain strictly below 30 ms to
unambiguously unwrap half a revolution; this is a mathematical bound, not a measured
speed guarantee. Long stalls can invalidate tracking without showing a spike.

## 3. SwerveOffsetsTuner

Lift the robot, align wheels forward before INIT and keep clear of all moving parts.
Hold gamepad RIGHT BUMPER to command Pedro's forward drive at 0.25 power.
Release to stop steering and drive. A toggles drivetrain diagnostics.
Confirm the pods seek the forward/backward axis and their drive directions propel
the robot forward. Shortest-path optimization can point a pod backward and reverse
its motor. Adjust each PodCalibration driveDirection/servoDirection/encoderReversed
and restart before retesting. Measure module center positions for each X_IN/Y_IN;
the current +/-7 inch values are placeholders, with positive X forward, Y left.

## 4. SwerveTurnTuner

Again test lifted first. Hold RIGHT BUMPER for 0.25 turn input, release to stop.
Verify tangent module headings and consistent rotation rather than radial X-lock.
Check wiring, lateral offsets and encoder directions if a module disagrees.
Do not proceed to path following until forward, lateral and rotation tests agree.

## 5. SwervePIDTuner with Panels

Align all wheels forward before START. Select one pod through selectedPod in Panels.
Expand frontLeft, frontRight, backLeft or backRight to edit targetDeg and p/i/d/f.
Hold RIGHT BUMPER to permit that pod's steering; release to stop. Drive motors
remain at zero, and steering output is clamped to +/-1. Gamepad never edits gains
or target. Telemetry shows received target, applied PIDF and actual servo output.

The controller is Pedro Controller.pid(P,I,D).plus(proportionalFeedforward(F)),
as in the upstream tuner. Coefficients update when changed, preserving controller
state between loops. At all-zero coefficients steering power must be zero even
with a nonzero target error. Verify this and a target change before tuning.
Shortest-path optimization may choose target+180, including with zero drive power.

The patch uses signed encoder-radian error (wheel-radian error times 1.5), not
degree-based gains from the old hand-written tuner. Inside 2 wheel degrees it
suppresses F but still applies PID. Steering direction still needs physical checking.
Tune P then D lifted; tune F under ground load. Each pod has its own coefficients.
Live values update shared Constants calibration in memory for subsequent modes,
but DO NOT persist across app restart. Record and enter final gains in Constants.

## Remaining Measurements

Voltage bounds have been measured. Recalibrated zero offsets, pod directions,
module geometry, and PIDF values still need verification. Pinpoint hardware name, pod type/directions and
offsets also remain unverified. Foresight gains, achievable velocities, deceleration,
and path constraints are placeholders until localization and driving are reliable.
X-lock is currently disabled; confirm steering and brakes before enabling it and
remeasure stopping behavior afterwards.

The official pedro.Constants.create entry point delegates to our shared patched
follower. Existing pedroPathing.Constants.createFollower callers use the same data.
The legacy Swerve Steering PID Tuner name is disabled to prevent tuning a separate
implementation. Encoder Test and Swerve Turn Speed Test remain unchanged.
