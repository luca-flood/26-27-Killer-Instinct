package org.firstinspires.ftc.teamcode.opmode.test;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.KillerInstinctRobot;
import org.firstinspires.ftc.teamcode.Mechanism.Limelight;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.Collections;
import java.util.Map;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.opmode.OpModeHook;

/** Compares independent Limelight and Pinpoint poses while driving. */
@NextTeleop(name = "Limelight MegaTag2 Test", group = "Tests")
public final class LimelightMegaTag2Test extends NextOpMode {
    // Set these to the measured robot pose on the field before INIT.
    private static final double START_X_IN = 59.5;
    private static final double START_Y_IN = 7;
    private static final double START_HEADING_DEG = 90;
    private static final Pose START_POSE = PoseFactory.degrees().of(START_X_IN, START_Y_IN, START_HEADING_DEG);
    private static final int LIMELIGHT_PIPELINE = 4;

    private final KillerInstinctRobot robot;
    private final Follower follower;
    private final Command drive;
    private final Command readPose;
    private final Command checkTip;

    public LimelightMegaTag2Test(KillerInstinctRobot robot) {
        super(robot, new OpModeHook() {
            @Override
            public void afterEnd() {
                robot.limelight.close();
            }
        });
        this.robot = robot;
        follower = initFollower();
        drive = robot.drivetrain.drive(gamepad1, gamepad2);
        readPose = robot.limelight.readPose(this::showLimelight);
        checkTip = robot.limelight.checkTip(status -> telemetry.addData("HIVE", status));
        telemetry.setMsTransmissionInterval(100);
        robot.limelight.configure(hardwareMap, LIMELIGHT_PIPELINE);
    }

    @Override
    public void disabledPeriodic() {
        if (follower == null) {
            telemetry.addData("Pinpoint odo", "Initialization failed");
        } else {
            showPose("Pinpoint start (in / deg)", START_POSE);
        }
        telemetry.update();
    }

    @Override
    public void start() {
        robot.limelight.start();
        drive.schedule();
        if (follower != null) {
            follower.setPose(START_POSE);
        }
    }

    @Override
    public void periodic() {
        if (follower == null) {
            telemetry.addData("Pinpoint odo", "Initialization failed");
        } else {
            follower.update();
            showPose("Pinpoint odo (in / deg)", follower.pose());
        }
        readPose.schedule();
        checkTip.schedule();
        telemetry.update();
    }

    @Override
    public void end() {
        drive.cancel();
    }

    private Follower initFollower() {
        try {
            return new Follower(
                    new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                    new OdometryOnlyDrivetrain(),
                    new Foresight(Constants.foresightConfig));
        } catch (RuntimeException exception) {
            RobotLog.ee("LimelightMegaTag2Test", exception, "Could not initialize Pedro Pinpoint odometry");
            return null;
        }
    }

    private void showLimelight(Limelight.PoseReading reading) {
        Vector2D xy = reading.rawPedroXYIn();
        if (xy == null) {
            telemetry.addData("Limelight X/Y (Pedro in)", reading.status);
        } else {
            telemetry.addData("Limelight X/Y (Pedro in)", "X %s  Y %s", xy.x(), xy.y());
        }
        telemetry.addData("AprilTags", reading.tagIds.isEmpty() ? "None" : reading.tagIds.toString());
    }

    private void showPose(String label, Pose pose) {
        telemetry.addData(label, "X %.1f  Y %.1f  H %.1f",
                pose.x(), pose.y(), AngleUnit.normalizeDegrees(Math.toDegrees(pose.heading())));
    }

    // Pedro stops its drivetrain while idle. NextFTC owns the real drive motors.
    private static final class OdometryOnlyDrivetrain implements Drivetrain {
        @Override
        public void drive(DrivePowers powers, boolean manual) {}

        @Override
        public void stop() {}

        @Override
        public void stop(boolean brake) {}

        @Override
        public double maxScaling(DrivePowers priority, DrivePowers remaining) {
            return 0.0;
        }

        @Override
        public double interpolateVelocity(double xRadius, double yRadius, double theta) {
            return 0.0;
        }

        @Override
        public Map<String, Object> debug() {
            return Collections.emptyMap();
        }
    }
}
