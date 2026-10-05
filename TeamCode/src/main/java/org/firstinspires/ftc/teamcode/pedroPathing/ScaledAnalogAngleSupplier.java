package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

public class ScaledAnalogAngleSupplier implements DoubleSupplier {
    private final AnalogInput analogInput;
    private final double analogMinVoltage;
    private final double analogMaxVoltage;
    private final double straightEncoderOffsetRad;
    private final boolean encoderReversed;
    private final double encoderToWheelRatio;
    private final ElapsedTime sampleTimer = new ElapsedTime();

    private double maxEncoderVelocityRadPerSecond = Math.toRadians(6000.0);
    private double lastWrappedEncoderAngleRad;
    private double continuousEncoderAngleRad;
    private boolean hasSample;
    private int rejectedSpikeCount;

    public ScaledAnalogAngleSupplier(
            AnalogInput analogInput,
            double analogMinVoltage,
            double analogMaxVoltage,
            double straightEncoderOffsetRad,
            boolean encoderReversed,
            double wheelDegreesPerEncoderRevolution) {
        if (!Double.isFinite(analogMinVoltage) || !Double.isFinite(analogMaxVoltage)
                || analogMaxVoltage <= analogMinVoltage
                || !Double.isFinite(wheelDegreesPerEncoderRevolution)
                || wheelDegreesPerEncoderRevolution <= 0) {
            throw new IllegalArgumentException("Encoder voltage range and wheel ratio must be finite and positive");
        }
        this.analogInput = analogInput;
        this.analogMinVoltage = analogMinVoltage;
        this.analogMaxVoltage = analogMaxVoltage;
        this.straightEncoderOffsetRad = straightEncoderOffsetRad;
        this.encoderReversed = encoderReversed;
        this.encoderToWheelRatio = wheelDegreesPerEncoderRevolution / 360.0;
    }

    @Override
    public double getAsDouble() {
        updateContinuousEncoderAngle();

        double encoderDeltaRad = continuousEncoderAngleRad - straightEncoderOffsetRad;
        if (!encoderReversed) {
            encoderDeltaRad *= -1.0;
        }

        return normalizeAngle(encoderDeltaRad * encoderToWheelRatio);
    }

    public double getVoltage() {
        return analogInput.getVoltage();
    }

    public double getWrappedEncoderAngleRad() {
        return readWrappedEncoderAngleRad();
    }

    public double getContinuousEncoderAngleRad() {
        return continuousEncoderAngleRad;
    }

    public int getRejectedSpikeCount() {
        return rejectedSpikeCount;
    }

    public void setMaxEncoderVelocityDegreesPerSecond(double maxEncoderVelocityDegreesPerSecond) {
        maxEncoderVelocityRadPerSecond = Math.toRadians(maxEncoderVelocityDegreesPerSecond);
    }

    public void reset() {
        hasSample = false;
        rejectedSpikeCount = 0;
        sampleTimer.reset();
    }

    private void updateContinuousEncoderAngle() {
        double wrappedEncoderAngleRad = readWrappedEncoderAngleRad();

        if (!hasSample) {
            lastWrappedEncoderAngleRad = wrappedEncoderAngleRad;
            // Choose the branch nearest straight. Multi-turn position still requires physical alignment at startup.
            continuousEncoderAngleRad = straightEncoderOffsetRad
                    + normalizeSignedAngle(wrappedEncoderAngleRad - straightEncoderOffsetRad);
            hasSample = true;
            sampleTimer.reset();
            return;
        }

        double deltaRad = normalizeSignedAngle(wrappedEncoderAngleRad - lastWrappedEncoderAngleRad);
        double elapsedSeconds = Math.max(sampleTimer.seconds(), 0.001);
        double maxDeltaRad = maxEncoderVelocityRadPerSecond * elapsedSeconds;

        if (Math.abs(deltaRad) <= maxDeltaRad) {
            continuousEncoderAngleRad += deltaRad;
            lastWrappedEncoderAngleRad = wrappedEncoderAngleRad;
            sampleTimer.reset();
        } else {
            rejectedSpikeCount++;
        }

    }

    private double readWrappedEncoderAngleRad() {
        double range = analogMaxVoltage - analogMinVoltage;
        if (range == 0.0) {
            return 0.0;
        }

        double normalized = clamp((analogInput.getVoltage() - analogMinVoltage) / range, 0.0, 1.0);
        return normalized * (2.0 * Math.PI);
    }

    private double normalizeSignedAngle(double angleRad) {
        while (angleRad > Math.PI) {
            angleRad -= 2.0 * Math.PI;
        }
        while (angleRad <= -Math.PI) {
            angleRad += 2.0 * Math.PI;
        }
        return angleRad;
    }

    private double normalizeAngle(double angleRad) {
        double normalized = angleRad % (2.0 * Math.PI);
        if (normalized < 0.0) {
            normalized += 2.0 * Math.PI;
        }
        return normalized;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
