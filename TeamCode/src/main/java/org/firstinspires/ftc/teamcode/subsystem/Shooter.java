package org.firstinspires.ftc.teamcode.subsystem;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.RobotConstants.ShooterConstants;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.MotorVelocityConstants;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

/**
 * Two-motor flywheel shooter. Left motor (port 0) is the leader, right
 * motor (port 1) follows it reversed. Must be listed in
 * KillerInstinctRobot#getMechanisms() to receive its default command.
 */
public class Shooter implements Mechanism {
    private final NextMotor left = new NextMotor(
            RobotController.controlHub(),
            0,
            Units.Rotations.of(1.0 / ShooterConstants.TICKS_PER_REV));
    private final NextMotor right = new NextMotor(
            RobotController.controlHub(),
            1,
            Units.Rotations.of(1.0 / ShooterConstants.TICKS_PER_REV));

    public Shooter() {
        setVelocityGains(
                ShooterConstants.KP, ShooterConstants.KI,
                ShooterConstants.KD, ShooterConstants.KS,
                ShooterConstants.KV, ShooterConstants.KA);
    }

    /**
     * Follows the left motor, then spins up to the given RPM; finishes
     * once within ShooterConstants.RPM_TOLERANCE of it.
     */
    public Command goToRPM(double rpm) {
        return sequential(followLeft(), spinTo(rpm));
    }

    /** Makes the right motor follow the left motor in reverse. */
    public Command followLeft() {
        return instant(() ->
                right.follow(left, NextMotor.Direction.REVERSE));
    }

    private Command spinTo(double rpm) {
        return Command.build()
                .setStart(() -> left.setVelocitySetpoint(
                        Units.Rotations.per(Units.Minutes).of(rpm)))
                .setDone(() -> Math.abs(getRPM() - rpm)
                        < ShooterConstants.RPM_TOLERANCE)
                .requiring(this);
    }

    public void setVelocityGains(
            double kP, double kI, double kD,
            double kS, double kV, double kA) {
        MotorVelocityConstants gains = left.getVelocityConstants();
        gains.setKP(kP);
        gains.setKI(kI);
        gains.setKD(kD);
        gains.setKS(kS);
        gains.setKV(kV);
        gains.setKA(kA);
    }

    public double getRPM() {
        return left.getEncoderVelocity()
                .into(Units.Rotations.per(Units.Minutes));
    }
}
