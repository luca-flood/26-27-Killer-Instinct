package org.firstinspires.ftc.teamcode.pedroPathing.swerve;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.controllers.Controller;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PatchedCoaxialPod;

/** Adapted from official Quickstart 0470fdd SwervePIDTuner, by Havish Sripada.
 * Uses patched pods and Panels, retaining Pedro's PID + proportional feedforward.
 */
@Configurable
@TeleOp(name = "SwervePIDTuner", group = "Pedro Swerve")
public class SwervePIDTuner extends OpMode {
    public static volatile Gains frontLeft = new Gains(Constants.frontLeft);
    public static volatile Gains frontRight = new Gains(Constants.frontRight);
    public static volatile Gains backLeft = new Gains(Constants.backLeft);
    public static volatile Gains backRight = new Gains(Constants.backRight);

    @Configurable
    public static class Gains {
        public volatile double targetDeg = 140;
        public volatile double p;
        public volatile double i;
        public volatile double d;
        public volatile double f;

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
    @IgnoreConfigurable private final double[] lastTargets = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};
    @IgnoreConfigurable private final int[] gainUpdates = new int[4];
    @IgnoreConfigurable private final int[] targetUpdates = new int[4];
    @IgnoreConfigurable private long previousLoop;
    @IgnoreConfigurable private final double[] stableVoltage = new double[4];
    @IgnoreConfigurable private final long[] stableSince = new long[4];
    @IgnoreConfigurable private long enabledAt;
    @IgnoreConfigurable private boolean ready;

    @Override
    public void init() {
        PanelsConfigurables.INSTANCE.refreshClass(this);
        configs = Constants.podConfigs();
        pods = new PatchedCoaxialPod[configs.length];
        for (int i = 0; i < pods.length; i++) {
            // Neutral servo output enables the analog feedback on this hardware.
            hardwareMap.get(CRServo.class, configs[i].servoName.get()).setPower(0);
            hardwareMap.get(DcMotor.class, configs[i].motorName.get()).setPower(0);
            stableVoltage[i] = Double.NaN;
            stableSince[i] = 0;
            for (int j = 0; j < 4; j++) applied[i][j] = Double.NaN;
        }
        ready = false;
        enabledAt = System.nanoTime();
        previousLoop = System.nanoTime();
    }

    @Override
    public void init_loop() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.clearBulkCache();
        long now = System.nanoTime();
        ready = true;
        for (int i = 0; i < pods.length; i++) {
            hardwareMap.get(CRServo.class, configs[i].servoName.get()).setPower(0);
            hardwareMap.get(DcMotor.class, configs[i].motorName.get()).setPower(0);
            double voltage = hardwareMap.get(AnalogInput.class, configs[i].servoEncoderName.get()).getVoltage();
            telemetry.addData(configs[i].name.get() + " startup V", "%.3f", voltage);
            if (pods[i] == null) {
                double min = configs[i].analogMinVoltage.get();
                double max = configs[i].analogMaxVoltage.get();
                boolean valid = Double.isFinite(voltage) && voltage >= min - 0.02 && voltage <= max + 0.02;
                if (!valid || now - enabledAt < 500_000_000L) {
                    stableSince[i] = 0;
                } else if (stableSince[i] == 0 || Math.abs(voltage - stableVoltage[i]) > 0.02) {
                    stableVoltage[i] = voltage;
                    stableSince[i] = now;
                } else if (now - stableSince[i] >= 250_000_000L) {
                    configs[i].angleOffsetRad.set(Math.max(0, Math.min(1, (voltage - min) / (max - min))) * 2 * Math.PI);
                    pods[i] = new PatchedCoaxialPod(hardwareMap, configs[i], Constants.WHEEL_DEGREES_PER_ENCODER_REVOLUTION);
                }
            }
            if (pods[i] == null) {
                ready = false;
                telemetry.addData(configs[i].name.get() + " reference", "WAITING for valid stable voltage");
            } else {
                telemetry.addData(pods[i].name() + " wheel deg", Math.toDegrees(pods[i].getAngle()));
            }
        }
        telemetry.addData("startup", ready ? "READY - forward zero captured" : "WAIT - keep wheels forward; do not START");
        telemetry.update();
    }

    @Override
    public void start() {
        previousLoop = System.nanoTime();
    }

    @Override
    public void loop() {
        if (!ready) {
            telemetry.addLine("Not initialized: STOP and INIT again; wait for READY before START.");
            telemetry.update();
            return;
        }
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.clearBulkCache();
        long now = System.nanoTime();
        double loopMs = (now - previousLoop) / 1e6;
        previousLoop = now;
        Gains[] gains = {frontLeft, frontRight, backLeft, backRight};
        Constants.PodCalibration[] calibrations = {Constants.frontLeft, Constants.frontRight,
                Constants.backLeft, Constants.backRight};
        telemetry.addData("loop ms", loopMs);
        telemetry.addData("steering mode", "Continuous independent PID for all four pods");
        for (int i = 0; i < pods.length; i++) {
            Gains g = gains[i];
            // Snapshot browser-thread fields once so calculation and readback use the same values.
            double p = g.p, integral = g.i, d = g.d, f = g.f, target = g.targetDeg;
            boolean valid = Double.isFinite(p) && Double.isFinite(integral) && Double.isFinite(d)
                    && Double.isFinite(f) && Double.isFinite(target);
            if (valid && (p != applied[i][0] || integral != applied[i][1]
                    || d != applied[i][2] || f != applied[i][3])) {
                configs[i].turnController.set(Controller.pid(p, integral, d)
                        .plus(Controller.proportionalFeedforward(f)));
                applied[i][0] = calibrations[i].p = p;
                applied[i][1] = calibrations[i].i = integral;
                applied[i][2] = calibrations[i].d = d;
                applied[i][3] = calibrations[i].f = f;
                gainUpdates[i]++;
            }
            if (valid && target != lastTargets[i]) {
                lastTargets[i] = target;
                targetUpdates[i]++;
            }
            if (valid) {
                pods[i].tuneSteering(Math.toRadians(target));
            } else {
                pods[i].getAngle();
                pods[i].setServoPower(0);
                pods[i].setMotorPower(0);
            }
            telemetry.addData(pods[i].name() + " applied PIDF", "%.5f / %.5f / %.5f / %.5f",
                    applied[i][0], applied[i][1], applied[i][2], applied[i][3]);
            telemetry.addData(pods[i].name() + " received target deg", target);
            telemetry.addData(pods[i].name() + " updates gains / target", "%d / %d", gainUpdates[i], targetUpdates[i]);
            telemetry.addData(pods[i].name() + " config valid", valid);
            double measured = Math.toDegrees(pods[i].getAngle());
            double error = Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(target - measured)),
                    Math.cos(Math.toRadians(target - measured))));
            telemetry.addData(pods[i].name() + " wheel / requested error", "%.1f / %.1f deg", measured, error);
            telemetry.addData(pods[i].name() + " servo power", pods[i].debug().get("servoPower"));
        }
        telemetry.addLine("Direct steering targets: no 180-degree drive optimization. PID error uses encoder radians.");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (pods != null) {
            for (PatchedCoaxialPod pod : pods) {
                if (pod == null) continue;
                pod.setServoPower(0);
                pod.setMotorPower(0);
            }
        }
    }
}
