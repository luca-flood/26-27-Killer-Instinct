package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Swerve Turn Speed Test", group = "Tests")
public class SwerveTurnSpeedTest extends OpMode {
    private static final double MAX_ANALOG_VOLTAGE = 3.3;
    private static final double SERVO_TEST_POWER = 1.0;
    private static final double WHEEL_DEGREES_PER_ENCODER_REVOLUTION = 240.0;
    private static final double ENCODER_TO_WHEEL_RATIO = WHEEL_DEGREES_PER_ENCODER_REVOLUTION / 360.0;
    private static final double MAX_REASONABLE_ENCODER_DEG_PER_SEC = 5000.0;

    private TurnPod frontLeft;
    private TurnPod backLeft;
    private TurnPod frontRight;
    private TurnPod backRight;

    @Override
    public void init() {
        frontLeft = new TurnPod("Front Left", "frontLeft1", "frontLeft2");
        backLeft = new TurnPod("Back Left", "backLeft1", "backLeft2");
        frontRight = new TurnPod("Front Right", "frontRight1", "frontRight2");
        backRight = new TurnPod("Back Right", "backRight1", "backRight2");
    }

    @Override
    public void start() {
        frontLeft.reset();
        backLeft.reset();
        frontRight.reset();
        backRight.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            frontLeft.reset();
            backLeft.reset();
            frontRight.reset();
            backRight.reset();
        }

        frontLeft.update(buttonPower(gamepad1.a, gamepad1.dpad_up));
        backLeft.update(buttonPower(gamepad1.b, gamepad1.dpad_left));
        frontRight.update(buttonPower(gamepad1.x, gamepad1.dpad_down));
        backRight.update(buttonPower(gamepad1.y, gamepad1.dpad_right));

        telemetry.addLine("Hold A/B/X/Y for +1.0. Hold D-pad up/left/down/right for -1.0.");
        telemetry.addLine("Use one pod at a time. Bumpers reset measured speed and angle.");
        addPodTelemetry(frontLeft);
        addPodTelemetry(backLeft);
        addPodTelemetry(frontRight);
        addPodTelemetry(backRight);
        telemetry.update();
    }

    @Override
    public void stop() {
        frontLeft.stop();
        backLeft.stop();
        frontRight.stop();
        backRight.stop();
    }

    private double buttonPower(boolean positiveButton, boolean negativeButton) {
        if (positiveButton && !negativeButton) {
            return SERVO_TEST_POWER;
        }
        if (negativeButton && !positiveButton) {
            return -SERVO_TEST_POWER;
        }
        return 0.0;
    }

    private void addPodTelemetry(TurnPod pod) {
        telemetry.addData(pod.label,
                "pwr %.1f | %.3f V | wrap %.1f deg | enc %.1f deg | wheel %.1f deg | speed %.1f wheel deg/s | peak %.1f | spikes %d",
                pod.servoPower,
                pod.rawVoltage,
                pod.wrappedEncoderDeg,
                pod.continuousEncoderDeg,
                pod.continuousWheelDeg,
                pod.wheelDegPerSecond,
                pod.peakAbsWheelDegPerSecond,
                pod.rejectedSpikeCount);
    }

    private class TurnPod {
        private final String label;
        private final CRServo servo;
        private final AnalogInput encoder;
        private final ElapsedTime sampleTimer = new ElapsedTime();

        private double rawVoltage;
        private double servoPower;
        private double wrappedEncoderDeg;
        private double lastWrappedEncoderDeg;
        private double continuousEncoderDeg;
        private double continuousWheelDeg;
        private double encoderDegPerSecond;
        private double wheelDegPerSecond;
        private double peakAbsWheelDegPerSecond;
        private int rejectedSpikeCount;
        private boolean hasSample;

        private TurnPod(String label, String servoName, String encoderName) {
            this.label = label;
            servo = hardwareMap.get(CRServo.class, servoName);
            encoder = hardwareMap.get(AnalogInput.class, encoderName);
            servo.setPower(0.0);
            reset();
        }

        private void reset() {
            rawVoltage = encoder.getVoltage();
            wrappedEncoderDeg = voltageToEncoderDegrees(rawVoltage);
            lastWrappedEncoderDeg = wrappedEncoderDeg;
            continuousEncoderDeg = wrappedEncoderDeg;
            continuousWheelDeg = continuousEncoderDeg * ENCODER_TO_WHEEL_RATIO;
            encoderDegPerSecond = 0.0;
            wheelDegPerSecond = 0.0;
            peakAbsWheelDegPerSecond = 0.0;
            rejectedSpikeCount = 0;
            hasSample = true;
            sampleTimer.reset();
        }

        private void update(double requestedServoPower) {
            servoPower = requestedServoPower;
            servo.setPower(servoPower);

            rawVoltage = encoder.getVoltage();
            wrappedEncoderDeg = voltageToEncoderDegrees(rawVoltage);

            if (!hasSample) {
                reset();
                return;
            }

            double elapsedSeconds = Math.max(sampleTimer.seconds(), 0.001);
            double deltaEncoderDeg = signedWrappedDelta(wrappedEncoderDeg - lastWrappedEncoderDeg);
            double measuredEncoderDegPerSecond = deltaEncoderDeg / elapsedSeconds;

            if (Math.abs(measuredEncoderDegPerSecond) <= MAX_REASONABLE_ENCODER_DEG_PER_SEC) {
                continuousEncoderDeg += deltaEncoderDeg;
                encoderDegPerSecond = measuredEncoderDegPerSecond;
                wheelDegPerSecond = encoderDegPerSecond * ENCODER_TO_WHEEL_RATIO;

                if (Math.abs(servoPower) > 0.05) {
                    peakAbsWheelDegPerSecond = Math.max(peakAbsWheelDegPerSecond, Math.abs(wheelDegPerSecond));
                }

                lastWrappedEncoderDeg = wrappedEncoderDeg;
            } else {
                rejectedSpikeCount++;
            }

            continuousWheelDeg = continuousEncoderDeg * ENCODER_TO_WHEEL_RATIO;
            sampleTimer.reset();
        }

        private void stop() {
            servoPower = 0.0;
            servo.setPower(0.0);
        }
    }

    private double voltageToEncoderDegrees(double voltage) {
        return Math.max(0.0, Math.min(voltage / MAX_ANALOG_VOLTAGE, 1.0)) * 360.0;
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
}
