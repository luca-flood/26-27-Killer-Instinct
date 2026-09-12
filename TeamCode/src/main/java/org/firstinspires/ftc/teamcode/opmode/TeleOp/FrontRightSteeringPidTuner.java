package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Front Right Steering PID Tuner", group = "Tests")
public class FrontRightSteeringPidTuner extends OpMode {
    private static final double MAX_ANALOG_VOLTAGE = 3.3;
    private static final double ENCODER_TO_WHEEL_RATIO = 240.0 / 360.0;
    private static final double MAX_POWER = 1.0;
    private static final double INTEGRAL_LIMIT = 80.0;
    private static final double MAX_REASONABLE_ENCODER_DEG_PER_SEC = 6000.0;
    private static final double FRONT_RIGHT_OFFSET_DEG = 240.2;
    private static final boolean FRONT_RIGHT_REVERSED = true;
    private static final double FRONT_RIGHT_RAW_0_DEG = 240.2;
    private static final double FRONT_RIGHT_RAW_90_DEG = 195.1;
    private static final double FRONT_RIGHT_RAW_180_DEG = 330.1;
    private static final double FRONT_RIGHT_RAW_270_DEG = 105.1;
    private static final String DASHBOARD_CATEGORY = "FRONT_RIGHT_STEERING";

    private final ElapsedTime loopTimer = new ElapsedTime();
    private FtcDashboard dashboard;
    private CRServo frontRightServo;
    private AnalogInput frontRightEncoder;

    private double targetEncoderDeg = FRONT_RIGHT_RAW_90_DEG;
    private double p = 0.015;
    private double i = 0.0;
    private double d = 0.0002;
    private double f = 0.04;
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
    private double unclampedOutput;
    private int rejectedSpikeCount;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        frontRightServo = hardwareMap.get(CRServo.class, "frontRight1");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRight2");
        frontRightServo.setPower(0.0);
        addDashboardVariables();
        resetAngleState();
    }

    @Override
    public void start() {
        resetAngleState();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = Math.max(loopTimer.seconds(), 0.001);
        loopTimer.reset();

        handleTargets();
        updateContinuousAngle(dt);
        updateWheelAngle();
        updatePid(dt);

        telemetry.addLine("Front Right only. PID is always active while this OpMode runs.");
        telemetry.addLine("Dashboard category: FRONT_RIGHT_STEERING. Press the save button after editing values.");
        telemetry.addData("target raw encoder deg", "%.2f", normalizeDegrees(targetEncoderDeg));
        telemetry.addData("actual raw encoder deg", "%.2f", wrappedEncoderDeg);
        telemetry.addData("live P/I/D/F", "%.5f / %.5f / %.5f / %.5f", p, i, d, f);
        telemetry.addData("actual wheel deg", "%.2f", wheelDeg);
        telemetry.addData("wheel error deg", "%.2f", wheelErrorDeg);
        telemetry.addData("encoder error deg", "%.2f", encoderErrorDeg);
        telemetry.addData("unclamped output", "%.3f", unclampedOutput);
        telemetry.addData("servo power", "%.3f", servoPower);
        telemetry.addData("P/I/D/F output", "%.3f / %.3f / %.3f / %.3f", pOutput, iOutput, dOutput, fOutput);
        telemetry.addData("raw encoder", "%.3f V | %.1f deg", rawVoltage, wrappedEncoderDeg);
        telemetry.addData("continuous encoder deg", "%.1f", continuousEncoderDeg);
        telemetry.addData("wheel speed deg/s", "%.1f", wheelDegPerSecond);
        telemetry.addData("spikes", rejectedSpikeCount);
        telemetry.addData("loop dt ms", "%.1f", dt * 1000.0);
        telemetry.update();
    }

    @Override
    public void stop() {
        dashboard.removeConfigVariable(DASHBOARD_CATEGORY, "TARGET_ENCODER_DEG");
        dashboard.removeConfigVariable(DASHBOARD_CATEGORY, "P");
        dashboard.removeConfigVariable(DASHBOARD_CATEGORY, "I");
        dashboard.removeConfigVariable(DASHBOARD_CATEGORY, "D");
        dashboard.removeConfigVariable(DASHBOARD_CATEGORY, "F");
        frontRightServo.setPower(0.0);
    }

    private void handleTargets() {
        if (gamepad1.a) {
            targetEncoderDeg = wrappedEncoderDeg;
            integral = 0.0;
            lastEncoderErrorDeg = 0.0;
        } else if (gamepad1.dpad_up) {
            targetEncoderDeg = FRONT_RIGHT_RAW_0_DEG;
        } else if (gamepad1.dpad_right) {
            targetEncoderDeg = FRONT_RIGHT_RAW_90_DEG;
        } else if (gamepad1.dpad_down) {
            targetEncoderDeg = FRONT_RIGHT_RAW_180_DEG;
        } else if (gamepad1.dpad_left) {
            targetEncoderDeg = FRONT_RIGHT_RAW_270_DEG;
        }
    }

    private void addDashboardVariables() {
        dashboard.addConfigVariable(DASHBOARD_CATEGORY, "TARGET_ENCODER_DEG", new ValueProvider<Double>() {
            @Override
            public Double get() {
                return targetEncoderDeg;
            }

            @Override
            public void set(Double value) {
                targetEncoderDeg = value;
                integral = 0.0;
                lastEncoderErrorDeg = 0.0;
            }
        });
        dashboard.addConfigVariable(DASHBOARD_CATEGORY, "P", new ValueProvider<Double>() {
            @Override
            public Double get() {
                return p;
            }

            @Override
            public void set(Double value) {
                p = value;
                integral = 0.0;
                lastEncoderErrorDeg = 0.0;
            }
        });
        dashboard.addConfigVariable(DASHBOARD_CATEGORY, "I", new ValueProvider<Double>() {
            @Override
            public Double get() {
                return i;
            }

            @Override
            public void set(Double value) {
                i = value;
                integral = 0.0;
                lastEncoderErrorDeg = 0.0;
            }
        });
        dashboard.addConfigVariable(DASHBOARD_CATEGORY, "D", new ValueProvider<Double>() {
            @Override
            public Double get() {
                return d;
            }

            @Override
            public void set(Double value) {
                d = value;
                integral = 0.0;
                lastEncoderErrorDeg = 0.0;
            }
        });
        dashboard.addConfigVariable(DASHBOARD_CATEGORY, "F", new ValueProvider<Double>() {
            @Override
            public Double get() {
                return f;
            }

            @Override
            public void set(Double value) {
                f = value;
                integral = 0.0;
                lastEncoderErrorDeg = 0.0;
            }
        });
        dashboard.updateConfig();
    }

    private void resetAngleState() {
        rawVoltage = frontRightEncoder.getVoltage();
        wrappedEncoderDeg = voltageToEncoderDegrees(rawVoltage);
        lastWrappedEncoderDeg = wrappedEncoderDeg;
        continuousEncoderDeg = wrappedEncoderDeg;
        updateWheelAngle();
        wheelErrorDeg = 0.0;
        encoderErrorDeg = 0.0;
        lastEncoderErrorDeg = 0.0;
        integral = 0.0;
        servoPower = 0.0;
        pOutput = 0.0;
        iOutput = 0.0;
        dOutput = 0.0;
        fOutput = 0.0;
        unclampedOutput = 0.0;
        rejectedSpikeCount = 0;
        frontRightServo.setPower(0.0);
    }

    private void updateContinuousAngle(double dt) {
        rawVoltage = frontRightEncoder.getVoltage();
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
        double encoderDeltaDeg = continuousEncoderDeg - FRONT_RIGHT_OFFSET_DEG;
        if (!FRONT_RIGHT_REVERSED) {
            encoderDeltaDeg *= -1.0;
        }
        wheelDeg = normalizeDegrees(encoderDeltaDeg * ENCODER_TO_WHEEL_RATIO);
    }

    private void updatePid(double dt) {
        encoderErrorDeg = signedWrappedDelta(normalizeDegrees(targetEncoderDeg) - wrappedEncoderDeg);
        wheelErrorDeg = encoderErrorDeg * ENCODER_TO_WHEEL_RATIO;

        integral += encoderErrorDeg * dt;
        integral = clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        double derivative = (encoderErrorDeg - lastEncoderErrorDeg) / dt;
        pOutput = p * encoderErrorDeg;
        iOutput = i * integral;
        dOutput = d * derivative;
        fOutput = f * Math.signum(encoderErrorDeg);
        unclampedOutput = pOutput + iOutput + dOutput + fOutput;
        servoPower = clamp(unclampedOutput, -MAX_POWER, MAX_POWER);

        lastEncoderErrorDeg = encoderErrorDeg;
        frontRightServo.setPower(servoPower);
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
