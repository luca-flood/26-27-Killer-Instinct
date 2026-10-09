package org.firstinspires.ftc.teamcode.pedroPathing.swerve;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PatchedCoaxialPod;

@TeleOp(name = "Swerve Drive", group = "Swerve")
public class SwerveDriveTeleOp extends OpMode {
    private final double[] stableVoltage = new double[4];
    private final long[] stableSince = new long[4];
    private final double[] targets = new double[4];
    private final double[] speeds = new double[4];
    private CoaxialPodConfig[] configs;
    private PatchedCoaxialPod[] pods;
    private long enabledAt;
    private boolean ready;
    private double radius;

    @Override
    public void init() {
        configs = Constants.podConfigs();
        pods = new PatchedCoaxialPod[configs.length];
        for (int i = 0; i < configs.length; i++) {
            hardwareMap.get(CRServo.class, configs[i].servoName.get()).setPower(0);
            hardwareMap.get(DcMotor.class, configs[i].motorName.get()).setPower(0);
            double x = configs[i].podOffset.get().x();
            double y = configs[i].podOffset.get().y();
            radius = Math.max(radius, Math.hypot(x, y));
        }
        enabledAt = System.nanoTime();
    }

    private void refreshSensors() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.clearBulkCache();
    }

    @Override
    public void init_loop() {
        refreshSensors();
        long now = System.nanoTime();
        ready = true;
        for (int i = 0; i < pods.length; i++) {
            double v = hardwareMap.get(AnalogInput.class, configs[i].servoEncoderName.get()).getVoltage();
            double min = configs[i].analogMinVoltage.get();
            double max = configs[i].analogMaxVoltage.get();
            if (pods[i] == null) {
                boolean valid = Double.isFinite(v) && v >= min - 0.02 && v <= max + 0.02;
                if (!valid || now - enabledAt < 500_000_000L) {
                    stableSince[i] = 0;
                } else if (stableSince[i] == 0 || Math.abs(v - stableVoltage[i]) > 0.02) {
                    stableVoltage[i] = v;
                    stableSince[i] = now;
                } else if (now - stableSince[i] >= 250_000_000L) {
                    configs[i].angleOffsetRad.set(Math.max(0, Math.min(1, (v - min) / (max - min))) * 2 * Math.PI);
                    pods[i] = new PatchedCoaxialPod(hardwareMap, configs[i], Constants.WHEEL_DEGREES_PER_ENCODER_REVOLUTION);
                    pods[i].setToBreak();
                }
            }
            if (pods[i] == null) ready = false;
            else pods[i].getAngle();
            telemetry.addData(configs[i].name.get(), "%.3f V / %s", v, pods[i] == null ? "WAIT" : "zero captured");
        }
        telemetry.addData("Startup", ready ? "READY" : "WAIT - keep all wheels physically forward");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (!ready) {
            stop();
            telemetry.addLine("STOP and INIT again; wait for READY before START.");
            telemetry.update();
            return;
        }
        refreshSensors();
        double forward = deadband(-gamepad1.left_stick_y);
        double left = deadband(-gamepad1.left_stick_x);
        double turn = deadband(-gamepad1.right_stick_x);
        double throttle = Math.max(0, Math.min(1, gamepad1.right_trigger));
        double translation = Math.hypot(forward, left);
        if (translation > 1) { forward /= translation; left /= translation; }
        double maxSpeed = 1;
        boolean moving = forward != 0 || left != 0 || turn != 0;
        for (int i = 0; i < pods.length; i++) {
            // Robot frame: X forward, Y left, positive rotation counterclockwise.
            double vx = forward - turn * configs[i].podOffset.get().y() / radius;
            double vy = left + turn * configs[i].podOffset.get().x() / radius;
            speeds[i] = Math.hypot(vx, vy);
            maxSpeed = Math.max(maxSpeed, speeds[i]);
            if (speeds[i] > 1e-6) targets[i] = Math.atan2(vy, vx);
        }
        for (int i = 0; i < pods.length; i++) {
            double actual = pods[i].getAngle();
            double target = targets[i];
            double error = signed(target - actual);
            double speed = speeds[i] / maxSpeed;
            if (Math.abs(error) > Math.PI / 2) {
                target += Math.PI;
                speed = -speed;
                error = signed(target - actual);
            }
            if (moving) pods[i].tuneSteering(target);
            else pods[i].setServoPower(0);
            // Suppress propulsion while wheels are substantially misaligned.
            double power = moving
                    ? throttle * speed * Math.max(0, Math.cos(error)) : 0;
            pods[i].setMotorPower(power);
            telemetry.addData(pods[i].name(), "angle %.1f / target %.1f / drive %.3f",
                    Math.toDegrees(actual), Math.toDegrees(signed(target)), power);
        }
        telemetry.addData("Right trigger throttle", throttle);
        telemetry.update();
    }

    private static double deadband(double value) {
        return Math.abs(value) < 0.08 ? 0 : Math.copySign((Math.abs(value) - 0.08) / 0.92, value);
    }

    private static double signed(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    @Override
    public void stop() {
        if (pods == null) return;
        for (PatchedCoaxialPod pod : pods) {
            if (pod == null) continue;
            pod.setMotorPower(0);
            pod.setServoPower(0);
        }
    }
}
