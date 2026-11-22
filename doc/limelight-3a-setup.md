# Limelight 3A setup for BIOBUZZ

## 1. Connect the camera

1. Connect the 3A to your laptop with a USB data cable. Open [Limelight Hardware Manager](https://docs.limelightvision.io/docs/resources/downloads), scan, and double-click the 3A to open its web UI. `http://limelight.local:5801` also works if the hostname resolves. Hardware Manager finds and flashes the camera. The camera web UI is where you edit pipelines, camera offsets, and maps.
2. Flash only if needed. Export existing pipelines and back up the field map and scripts first. Download the OS image specifically marked **Limelight 3A**. Unplug the camera, hold its blue config button while reconnecting it, and in Hardware Manager choose **Flash OS → select image → Refresh Device List → select device → Flash Device**. After completion, unplug and reconnect normally. [Manufacturer quick-start](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a)
3. In the web UI, open **Settings**, enter the team number, and restart the vision client. Move the cable to the Control Hub's blue USB 3 port. On the Driver Station, open **Configure Robot**, edit the active configuration, scan, and name the detected **Ethernet Device** `limelight`. Save and activate the configuration. Check that the existing `pinpoint` entry is still present. [FTC setup](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a)

## 2. Set up the AprilTag pipeline

Reconnect the camera directly to your laptop to edit the pipeline if the web UI is unavailable through the Control Hub. Put it back on the Hub's blue USB port before running the test.

1. Select the pipeline in the camera web UI that matches `LIMELIGHT_PIPELINE` in the test. The current slot is **4**. Under **Input**, set the pipeline type to **Fiducial Markers**. Start with 640 × 480, then shorten exposure until tags stay readable while turning. Keep the full image until both HIVE positions are easy to see.
2. Under **Standard**, choose **AprilTag Classic 36h11** and set **Marker Size** to **82.5 mm**. Enter comma-separated IDs for the side you are testing. Red uses `30,31,32,33,34,35,36,37`. Blue uses `38,39,40,41,42,43,44,45`. Keep both clusters of your alliance in the detection filter so tip checks can see either side. [Fiducial settings](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags) · [FIRST Arena manual](https://ftc-resources.firstinspires.org/ftc/game/manual-09)
3. Under **Advanced**, set **Full 3D Targeting** to **Yes** and **Snap Robot to Floor** to **No**. In **MegaTag Field-Space Localization Setup**, enter **LL Forward, Right, Up** in meters and **Roll, Pitch, Yaw** in degrees. Measure from the Pedro robot origin projected to the floor to the camera lens. Right and up are positive in the robot frame. Verify the mount in the 3D viewer, especially the pitch direction. [Robot-space camera pose](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2#configuring-your-limelights-robot-space-pose)
4. If you enabled **Ignore NetworkTables Index** to edit the pipeline, clear it when you finish so the OpMode can select its pipeline. [Pipeline settings](https://docs.limelightvision.io/docs/docs-limelight/getting-started/pipelines)

## 3. Upload the BluePower flat map

With the test pipeline selected, open **Advanced → MegaTag Field-Space Localization Setup → Field Map File**. Upload the BluePower flat map that matches the alliance and the tag filter you set above. The corrected pose and HIVE tip check use the BluePower hinge calculation. The raw height error is needed to calculate the hinge angle, so keep **Snap Robot to Floor** off. [Red flat map](https://raw.githubusercontent.com/BluePowerRobotics/Team19725_BIOBUZZ/6b788b4ba7fff43350b1a4c0c4ee96eb09ea0fdd/fmaps/BIOBUZZ_HiveFlat_Red.fmap) · [Blue flat map](https://raw.githubusercontent.com/BluePowerRobotics/Team19725_BIOBUZZ/6b788b4ba7fff43350b1a4c0c4ee96eb09ea0fdd/fmaps/BIOBUZZ_HiveFlat_Blue.fmap) · [BluePower’s solver derivation](https://github.com/BluePowerRobotics/Team19725_BIOBUZZ/blob/6b788b4ba7fff43350b1a4c0c4ee96eb09ea0fdd/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/VisionLocalizer/OneMap_MT1Localizer.md)

## 4. Compare vision and Pinpoint on the field

1. Place the robot at a taped starting position with a measured heading. In `LimelightMegaTag2Test`, set `START_X_IN`, `START_Y_IN`, and `START_HEADING_DEG` to that pose. The defaults are `(59.5 in, 7 in, 90°)` in Pedro coordinates. Pinpoint starts from those values and vision runs independently. [Pedro frame conversion](https://pedropathing.com/docs/pathing/reference/coordinates)
2. Run **Limelight MegaTag2 Test** on the Driver Station. **INIT** requests the configured pipeline and shows the starting pose. **PLAY** seeds Pedro, starts reading the camera, and enables the same NextFTC drive command as `fullTestNextFTC`. The **left stick** moves and strafes and the **right stick** turns. Telemetry shows the Pinpoint pose, uncorrected Limelight X/Y, HIVE state, and visible AprilTag IDs. Both X/Y readouts use Pedro field coordinates in inches, with corners at `(0, 0)` and `(144, 144)` and the center at `(72, 72)`. The camera reading is converted from the FTC field frame using `X = rawY + 72` and `Y = 72 - rawX`.
3. Park the robot with a clear view and move the practice HIVE through both settled states and intermediate angles. The raw Limelight X/Y can shift while the robot stays parked because the camera is using a flat map for a tilted HIVE. `TIPPED` means the other side is up compared with the official reset state. Red starts audience-side up and blue starts scoring-side up. [FIRST Field Reset Guide](https://ftc-resources.firstinspires.org/ftc/archive/2027/event/field-reset)

## 5. Use the mechanism in an OpMode

Call `robot.limelight.configure(hardwareMap, pipelineIndex)` during INIT, `robot.limelight.start()` on PLAY, and `robot.limelight.close()` when the OpMode ends. The drive test uses the registered NextFTC mechanism, so NextFTC runs its periodic updates and the command scheduler.

- `robot.limelight.checkTip(status -> …)` returns `TIPPED`, `NOT_TIPPED`, `MOVING`, or `UNKNOWN` through the callback.
- `robot.limelight.readPose(reading -> …)` returns `rawXYIn` in the FTC field frame, `pose` when the hinge correction succeeds, and `tagIds` for visible AprilTags. Use `reading.rawPedroXYIn()` for uncorrected X/Y in Pedro inches. The corrected `pose` uses Pedro inches and radians. If correction fails, `status` gives the reason and the uncorrected X/Y are still available.

Schedule either command like the other Ivy commands. Both read the latest observation and leave the drivetrain and odometry alone.
