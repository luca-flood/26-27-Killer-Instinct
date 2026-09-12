package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp(name = "Swerve Steering PID Tuner", group = "Tests")
public class SwerveSteeringPidTuner extends OpMode {
    public static PodConfig frontLeft = new PodConfig();
    public static PodConfig frontRight = new PodConfig();
    public static PodConfig backRight = new PodConfig();
    public static PodConfig backLeft = new PodConfig();

    public static double DEADBAND_WHEEL_DEG = 2.0;

    private static final double MAX_ANALOG_VOLTAGE = 3.3;
    private static final double ENCODER_TO_WHEEL_RATIO = 240.0 / 360.0;
    private static final double WHEEL_TO_ENCODER_RATIO = 360.0 / 240.0;
    private static final double INTEGRAL_LIMIT = 80.0;
    private static final double MAX_REASONABLE_ENCODER_DEG_PER_SEC = 6000.0;
    private static final double FRONT_LEFT_OFFSET_DEG = 78.1;
    private static final double FRONT_RIGHT_OFFSET_DEG = 240.2;
    private static final double BACK_RIGHT_OFFSET_DEG = 77.9;
    private static final double BACK_LEFT_OFFSET_DEG = 10.4;
    private static final boolean FRONT_LEFT_REVERSED = true;
    private static final boolean FRONT_RIGHT_REVERSED = true;
    private static final boolean BACK_RIGHT_REVERSED = true;
    private static final boolean BACK_LEFT_REVERSED = true;

    @IgnoreConfigurable
    private final ElapsedTime loopTimer = new ElapsedTime();
    @IgnoreConfigurable
    private TunerPod[] pods;
    @IgnoreConfigurable
    private int selectedPodIndex;
    @IgnoreConfigurable
    private boolean lastA;
    @IgnoreConfigurable
    private boolean lastB;
    @IgnoreConfigurable
    private boolean lastX;

    @Configurable
    public static class PodConfig {
        public boolean enablePid = false;
        public double targetDeg = 0.0;
        public double p = 0.015;
        public double i = 0.0;
        public double d = 0.0002;
        public double f = 0.04;
        public double power = 1.0;
    }

    @Override
    public void init() {
        PanelsConfigurables.INSTANCE.refreshClass(this);

        pods = new TunerPod[] {
                new TunerPod("Front Left", "frontLeft1", "frontLeft2", frontLeft),
                new TunerPod("Front Right", "frontRight1", "frontRight2", frontRight),
                new TunerPod("Back Right", "backRight1", "backRight2", backRight),
                new TunerPod("Back Left", "backLeft1", "backLeft2", backLeft)
        };
    }

    @Override
    public void start() {
        resetAllPods();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        handleButtons();

        double dt = Math.max(loopTimer.seconds(), 0.001);
        loopTimer.reset();

        int selectedIndex = clampSelectedPod();
        for (TunerPod pod : pods) {
            pod.update(dt);
        }

        TunerPod selected = pods[selectedIndex];
        PodConfig selectedConfig = selected.getConfig();
        telemetry.addLine("Panels: frontLeft/frontRight/backRight/backLeft -> enablePid, targetDeg, p, i, d, f, power.");
        telemetry.addLine("Gamepad: A reset | B next pod | X toggle selected PID | dpad up/right/down/left set selected target.");
        telemetry.addData("selected pod", "%d - %s", selectedIndex, selected.label);
        telemetry.addData("selected PID enabled", selectedConfig.enablePid);
        telemetry.addData("selected target deg", "%.2f", normalizeDegrees(selectedConfig.targetDeg));
        telemetry.addData("actual wheel deg", "%.2f", selected.wheelDeg);
        telemetry.addData("wheel error deg", "%.2f", selected.wheelErrorDeg);
        telemetry.addData("encoder error deg", "%.2f", selected.encoderErrorDeg);
        telemetry.addData("servo power", "%.3f", selected.servoPower);
        telemetry.addData("P/I/D/F output", "%.3f / %.3f / %.3f / %.3f",
                selected.pOutput, selected.iOutput, selected.dOutput, selected.fOutput);
        telemetry.addData("loop dt ms", "%.1f", dt * 1000.0);

        for (TunerPod pod : pods) {
            telemetry.addData(pod.label,
                    "%.3f V | wrap %.1f | enc %.1f | wheel %.1f | speed %.0f wheel deg/s | spikes %d",
                    pod.rawVoltage,
                    pod.wrappedEncoderDeg,
                    pod.continuousEncoderDeg,
                    pod.wheelDeg,
                    pod.wheelDegPerSecond,
                    pod.rejectedSpikeCount);
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        for (TunerPod pod : pods) {
            pod.stop();
        }
    }

    private void handleButtons() {
        if (gamepad1.a && !lastA) {
            resetAllPods();
        }
        if (gamepad1.b && !lastB) {
            selectedPodIndex = (clampSelectedPod() + 1) % pods.length;
            resetSelectedPod();
        }
        if (gamepad1.x && !lastX) {
            TunerPod selected = pods[clampSelectedPod()];
            PodConfig selectedConfig = selected.getConfig();
            selectedConfig.enablePid = !selectedConfig.enablePid;
            resetSelectedPod();
        }

        PodConfig selectedConfig = pods[clampSelectedPod()].getConfig();
        if (gamepad1.dpad_up) {
            selectedConfig.targetDeg = 0.0;
        } else if (gamepad1.dpad_right) {
            selectedConfig.targetDeg = 90.0;
        } else if (gamepad1.dpad_down) {
            selectedConfig.targetDeg = 180.0;
        } else if (gamepad1.dpad_left) {
            selectedConfig.targetDeg = 270.0;
        }

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
    }

    private void resetAllPods() {
        for (TunerPod pod : pods) {
            pod.reset();
        }
    }

    private void resetSelectedPod() {
        pods[clampSelectedPod()].resetPidState();
    }

    private int clampSelectedPod() {
        if (selectedPodIndex < 0) {
            selectedPodIndex = 0;
        } else if (selectedPodIndex >= pods.length) {
            selectedPodIndex = pods.length - 1;
        }
        return selectedPodIndex;
    }

    private class TunerPod {
        private final String label;
        private final CRServo servo;
        private final AnalogInput encoder;

        private double rawVoltage;
        private double wrappedEncoderDeg;
        private double lastWrappedEncoderDeg;
        private double continuousEncoderDeg;
        private double wheelDeg;
        private double wheelErrorDeg;
        private double encoderErrorDeg;
        private double lastEncoderErrorDeg;
        private double integral;
        private double wheelDegPerSecond;
        private double servoPower;
        private double pOutput;
        private double iOutput;
        private double dOutput;
        private double fOutput;
        private int rejectedSpikeCount;

        private TunerPod(String label, String servoName, String encoderName, PodConfig config) {
            this.label = label;
            servo = hardwareMap.get(CRServo.class, servoName);
            encoder = hardwareMap.get(AnalogInput.class, encoderName);
            reset();
        }

        private void reset() {
            rawVoltage = encoder.getVoltage();
            wrappedEncoderDeg = voltageToEncoderDegrees(rawVoltage);
            lastWrappedEncoderDeg = wrappedEncoderDeg;
            continuousEncoderDeg = wrappedEncoderDeg;
            resetPidState();
            updateWheelAngle();
        }

        private void resetPidState() {
            integral = 0.0;
            lastEncoderErrorDeg = 0.0;
            wheelErrorDeg = 0.0;
            encoderErrorDeg = 0.0;
            wheelDegPerSecond = 0.0;
            servoPower = 0.0;
            pOutput = 0.0;
            iOutput = 0.0;
            dOutput = 0.0;
            fOutput = 0.0;
            rejectedSpikeCount = 0;
            servo.setPower(0.0);
        }

        private void update(double dt) {
            PodConfig config = getConfig();
            updateContinuousAngle(dt);
            updateWheelAngle();

            if (!config.enablePid) {
                stop();
                resetPidState();
                return;
            }

            wheelErrorDeg = signedWrappedDelta(normalizeDegrees(config.targetDeg) - wheelDeg);
            encoderErrorDeg = wheelErrorToEncoderError(label, wheelErrorDeg);

            if (Math.abs(wheelErrorDeg) <= Math.abs(DEADBAND_WHEEL_DEG)) {
                servoPower = 0.0;
                integral = 0.0;
                pOutput = 0.0;
                iOutput = 0.0;
                dOutput = 0.0;
                fOutput = 0.0;
            } else {
                integral += encoderErrorDeg * dt;
                integral = clamp(integral, -Math.abs(INTEGRAL_LIMIT), Math.abs(INTEGRAL_LIMIT));

                double derivative = (encoderErrorDeg - lastEncoderErrorDeg) / dt;
                pOutput = config.p * encoderErrorDeg;
                iOutput = config.i * integral;
                dOutput = config.d * derivative;
                fOutput = config.f * Math.signum(encoderErrorDeg);
                servoPower = clamp(pOutput + iOutput + dOutput + fOutput, -Math.abs(config.power), Math.abs(config.power));
            }

            lastEncoderErrorDeg = encoderErrorDeg;
            servo.setPower(servoPower);
        }

        private PodConfig getConfig() {
            return SwerveSteeringPidTuner.this.getConfig(label);
        }

        private void updateContinuousAngle(double dt) {
            rawVoltage = encoder.getVoltage();
            wrappedEncoderDeg = voltageToEncoderDegrees(rawVoltage);

            double deltaEncoderDeg = signedWrappedDelta(wrappedEncoderDeg - lastWrappedEncoderDeg);
            double encoderDegPerSecond = deltaEncoderDeg / dt;

            if (Math.abs(encoderDegPerSecond) <= MAX_REASONABLE_ENCODER_DEG_PER_SEC) {
                continuousEncoderDeg += deltaEncoderDeg;
                wheelDegPerSecond = encoderDegPerSecond * ENCODER_TO_WHEEL_RATIO;
                lastWrappedEncoderDeg = wrappedEncoderDeg;
            } else {
                rejectedSpikeCount++;
            }
        }

        private void updateWheelAngle() {
            double encoderDeltaDeg = continuousEncoderDeg - getOffsetDeg(label);
            if (!getEncoderReversed(label)) {
                encoderDeltaDeg *= -1.0;
            }
            wheelDeg = normalizeDegrees(encoderDeltaDeg * ENCODER_TO_WHEEL_RATIO);
        }

        private void stop() {
            servoPower = 0.0;
            servo.setPower(0.0);
        }
    }

    private double wheelErrorToEncoderError(String label, double wheelErrorDeg) {
        double encoderErrorDeg = wheelErrorDeg * WHEEL_TO_ENCODER_RATIO;
        return getEncoderReversed(label) ? encoderErrorDeg : -encoderErrorDeg;
    }

    private PodConfig getConfig(String label) {
        if (label.equals("Front Left")) {
            return frontLeft;
        } else if (label.equals("Front Right")) {
            return frontRight;
        } else if (label.equals("Back Right")) {
            return backRight;
        }
        return backLeft;
    }

    private double getOffsetDeg(String label) {
        if (label.equals("Front Left")) {
            return FRONT_LEFT_OFFSET_DEG;
        } else if (label.equals("Front Right")) {
            return FRONT_RIGHT_OFFSET_DEG;
        } else if (label.equals("Back Right")) {
            return BACK_RIGHT_OFFSET_DEG;
        }
        return BACK_LEFT_OFFSET_DEG;
    }

    private boolean getEncoderReversed(String label) {
        if (label.equals("Front Left")) {
            return FRONT_LEFT_REVERSED;
        } else if (label.equals("Front Right")) {
            return FRONT_RIGHT_REVERSED;
        } else if (label.equals("Back Right")) {
            return BACK_RIGHT_REVERSED;
        }
        return BACK_LEFT_REVERSED;
    }

    private double voltageToEncoderDegrees(double voltage) {
        return clamp(voltage / MAX_ANALOG_VOLTAGE, 0.0, 1.0) * 360.0;
    }

    private double normalizeDegrees(double angleDeg) {
        while (angleDeg >= 360.0) {
            angleDeg -= 360.0;
        }
        while (angleDeg < 0.0) {
            angleDeg += 360.0;
        }
        return angleDeg;
    }

    private double signedWrappedDelta(double angleDeltaDeg) {
        while (angleDeltaDeg > 180.0) {
            angleDeltaDeg -= 360.0;
        }
        while (angleDeltaDeg <= -180.0) {
            angleDeltaDeg += 360.0;
        }
        return angleDeltaDeg;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }
}
