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
 * Two-motor flywheel shooter. Right motor (control hub port 3, has the
 * encoder) is the leader; left motor (expansion hub port 2) follows. Must be listed in
 * KillerInstinctRobot#getMechanisms() to receive its default command.
 */
public class Shooter implements Mechanism {
    // Drop motors left over from earlier Sloth reloads so they don't claim the same ports twice.
    private static final boolean LOOP_CLEARED = clearStaleMotors();

    private static boolean clearStaleMotors() {
        NextMotor.Companion.getMotorEventLoop().clear();
        return true;
    }

    private final NextMotor left = new NextMotor(
            RobotController.expansionHub(),
            2,
            Units.Rotations.of(1.0 / ShooterConstants.TICKS_PER_REV));
    private final NextMotor right = new NextMotor(
            RobotController.controlHub(),
            3,
            Units.Rotations.of(1.0 / ShooterConstants.TICKS_PER_REV));

    public Shooter() {
        left.setDirection(NextMotor.Direction.REVERSE);
        setPID(ShooterConstants.KP, ShooterConstants.KD);
        setKS(ShooterConstants.KS);
        setKV(ShooterConstants.KV);
    }

    /**
     * Follows the left motor, then spins up to the given RPM; finishes
     * once within ShooterConstants.RPM_TOLERANCE of it.
     */
    public Command goToRPM(double rpm) {
        return sequential(followRight(), spinTo(rpm));
    }

    /** Makes the left motor copy the right motor, which has the encoder. */
    public Command followRight() {
        return instant(() ->
                left.follow(right, NextMotor.Direction.FORWARD));
    }

    private Command spinTo(double rpm) {
        return Command.build()
                .setStart(() -> right.setVelocitySetpoint(
                        Units.Rotations.per(Units.Minutes).of(rpm)))
                .setDone(() -> Math.abs(getRPM() - rpm)
                        < ShooterConstants.RPM_TOLERANCE)
                .requiring(this);
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
}
