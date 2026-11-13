package org.firstinspires.ftc.teamcode.Mechanism;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.RobotConstants.ShooterConstants;

import dev.nextftc.hardware.actuators.MotorVelocityConstants;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

/**
 * Mechanically linked flywheel motors with one velocity loop. The right
 * encoder measures speed and the left motor follows the right motor's power.
 */
public class Shooter implements Mechanism {
    private final NextMotor right = new NextMotor(
            "shootRight",
            Units.Rotations.of(1.0 / ShooterConstants.TICKS_PER_REV));
    private final NextMotor left = new NextMotor(
            "shootLeft",
            Units.Rotations.of(1.0 / ShooterConstants.TICKS_PER_REV));

    public Shooter() {
        right.setDirection(NextMotor.Direction.REVERSE);
        left.setDirection(NextMotor.Direction.FORWARD);
        setPID(ShooterConstants.KP, ShooterConstants.KD);
        setKS(ShooterConstants.KS);
        setKV(ShooterConstants.KV);
    }

    /**
     * Spins up to the given RPM and finishes once the right motor is within
     * ShooterConstants.RPM_TOLERANCE of the target.
     */
    public Command goToRPM(double rpm) {
        return Command.build()
                .setStart(() -> setTargetRPM(rpm))
                .setDone(() -> Math.abs(getRPM() - rpm)
                        < ShooterConstants.RPM_TOLERANCE)
                .requiring(this);
    }

    /** Sets the target speed immediately so the caller can time the spin-up. */
    public Command setVelocityTicksPerSecond(double ticksPerSecond) {
        double rpm = ticksPerSecond * 60.0 / ShooterConstants.TICKS_PER_REV;
        return instant(() -> setTargetRPM(rpm));
    }

    public Command stop() {
        return instant(() -> setTargetRPM(0.0));
    }

    private void setTargetRPM(double rpm) {
        if (rpm == 0.0) {
            right.setThrottle(0.0);
            left.setThrottle(0.0);
            return;
        }
        right.setVelocitySetpoint(Units.Rotations.per(Units.Minutes).of(rpm));
        left.follow(right, NextMotor.Direction.FORWARD);
    }

    public void setPID(double kP, double kD) {
        MotorVelocityConstants gains = right.getVelocityConstants();
        gains.setKP(kP);
        gains.setKD(kD);
    }

    public void setKV(double kV) {
        right.getVelocityConstants().setKV(kV);
    }

    public void setKS(double kS) {
        right.getVelocityConstants().setKS(kS);
    }

    public double getRotations() {
        return right.getEncoderPosition().into(Units.Rotations);
    }

    public double getRPM() {
        return right.getEncoderVelocity()
                .into(Units.Rotations.per(Units.Minutes));
    }

    public double getRightVelocityTicksPerSecond() {
        return velocityTicksPerSecond(right);
    }

    public double getLeftVelocityTicksPerSecond() {
        return velocityTicksPerSecond(left);
    }

    private double velocityTicksPerSecond(NextMotor motor) {
        return motor.getEncoderVelocity().into(Units.Rotations.per(Units.Seconds))
                * ShooterConstants.TICKS_PER_REV;
    }
}
