package org.firstinspires.ftc.teamcode.pedroPathing.swerve;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.controllers.Controller;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PatchedCoaxialPod;

/** Adapted from official Quickstart 0470fdd SwervePIDTuner, by Havish Sripada.
 * Uses patched pods and Panels, retaining Pedro's PID + proportional feedforward.
 */
@Configurable
@TeleOp(name = "SwervePIDTuner", group = "Pedro Swerve")
public class SwervePIDTuner extends OpMode {
    public enum Pod { FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT }
    public static Pod selectedPod = Pod.FRONT_RIGHT;
    public static Gains frontLeft = new Gains(Constants.frontLeft);
    public static Gains frontRight = new Gains(Constants.frontRight);
    public static Gains backLeft = new Gains(Constants.backLeft);
    public static Gains backRight = new Gains(Constants.backRight);

    @Configurable
    public static class Gains {
        public double targetDeg = 0;
        public double p;
        public double i;
        public double d;
        public double f;

        public Gains(Constants.PodCalibration calibration) {
            p = calibration.p;
            i = calibration.i;
            d = calibration.d;
            f = calibration.f;
        }
    }

    @IgnoreConfigurable private CoaxialPodConfig[] configs;
    @IgnoreConfigurable private PatchedCoaxialPod[] pods;
    @IgnoreConfigurable private final double[][] applied = new double[4][4];
    @IgnoreConfigurable private long previousLoop;

    @Override
    public void init() {
        PanelsConfigurables.INSTANCE.refreshClass(this);
        telemetry.addLine("Align all wheels forward before START. Lift robot for initial tuning.");
        telemetry.addLine("Panels selects pod, target degrees, P/I/D/F. Hold RB to steer; drive motors stay off.");
        telemetry.update();
    }

    @Override
    public void start() {
        configs = Constants.podConfigs();
        pods = new PatchedCoaxialPod[configs.length];
        for (int i = 0; i < pods.length; i++) {
            pods[i] = new PatchedCoaxialPod(hardwareMap, configs[i], Constants.WHEEL_DEGREES_PER_ENCODER_REVOLUTION);
            for (int j = 0; j < 4; j++) applied[i][j] = Double.NaN;
            pods[i].getAngle();
        }
        previousLoop = System.nanoTime();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double loopMs = (now - previousLoop) / 1e6;
        previousLoop = now;
        int selected = selectedPod.ordinal();
        Gains[] gains = {frontLeft, frontRight, backLeft, backRight};
        Constants.PodCalibration[] calibrations = {Constants.frontLeft, Constants.frontRight,
                Constants.backLeft, Constants.backRight};
        telemetry.addData("selected pod", selectedPod);
        telemetry.addData("loop ms", loopMs);
        telemetry.addData("steering permitted", gamepad1.right_bumper);
        for (int i = 0; i < pods.length; i++) {
            Gains g = gains[i];
            boolean valid = Double.isFinite(g.p) && Double.isFinite(g.i) && Double.isFinite(g.d)
                    && Double.isFinite(g.f) && Double.isFinite(g.targetDeg);
            if (valid && (g.p != applied[i][0] || g.i != applied[i][1]
                    || g.d != applied[i][2] || g.f != applied[i][3])) {
                configs[i].turnController.set(Controller.pid(g.p, g.i, g.d)
                        .plus(Controller.proportionalFeedforward(g.f)));
                applied[i][0] = calibrations[i].p = g.p;
                applied[i][1] = calibrations[i].i = g.i;
                applied[i][2] = calibrations[i].d = g.d;
                applied[i][3] = calibrations[i].f = g.f;
            }
            if (i == selected && valid && gamepad1.right_bumper) {
                // Panels target is the reported wheel angle, not Pedro's internal vector frame.
                double frameOffset = configs[i].encoderReversed.get() ? Math.PI / 2 : -Math.PI / 2;
                pods[i].move(Math.toRadians(g.targetDeg) - frameOffset, 0, false);
            } else {
                pods[i].getAngle();
                pods[i].setServoPower(0);
                pods[i].setMotorPower(0);
                // Discard accumulated controller state while the deadman is released.
                if (valid) {
                    configs[i].turnController.set(Controller.pid(g.p, g.i, g.d)
                            .plus(Controller.proportionalFeedforward(g.f)));
                }
            }
            telemetry.addData(pods[i].name() + " applied PIDF", "%.5f / %.5f / %.5f / %.5f",
                    applied[i][0], applied[i][1], applied[i][2], applied[i][3]);
            telemetry.addData(pods[i].name() + " target deg", g.targetDeg);
            telemetry.addData(pods[i].name(), pods[i].debug());
        }
        telemetry.addLine("PID error uses encoder radians in this patch. Shortest-path may flip target 180 degrees.");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (pods != null) {
            for (PatchedCoaxialPod pod : pods) {
                pod.setServoPower(0);
                pod.setMotorPower(0);
            }
        }
    }
}
