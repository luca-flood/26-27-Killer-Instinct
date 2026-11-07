package org.firstinspires.ftc.teamcode.opmode.test;

import static com.pedropathing.ivy.commands.Commands.lazy;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;

import org.firstinspires.ftc.teamcode.KillerInstinctRobot;
import org.firstinspires.ftc.teamcode.RobotConstants.ShooterConstants;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/**
 * Shooter test and live tuner. A spins up to targetRPM, B stops. Edit the
 * gains and targetRPM from Panels; gains apply every loop, targetRPM on
 * the next A press.
 */
@Configurable
@NextTeleop(name = "Shooter Subsystem Test", group = "Test")
public class ShooterSubsystemTest extends NextOpMode {
    public static volatile double targetRPM = 3000.0;
    public static volatile double kP = ShooterConstants.KP;
    public static volatile double kI = ShooterConstants.KI;
    public static volatile double kD = ShooterConstants.KD;
    public static volatile double kS = ShooterConstants.KS;
    public static volatile double kV = ShooterConstants.KV;
    public static volatile double kA = ShooterConstants.KA;

    @IgnoreConfigurable
    private final KillerInstinctRobot robot;

    public ShooterSubsystemTest(KillerInstinctRobot robot) {
        super(robot);
        this.robot = robot;
        PanelsConfigurables.INSTANCE.refreshClass(this);
    }

    @Override
    public void start() {
        CommandGamepad pad = new CommandGamepad(gamepad1);
        pad.a().onTrue(lazy(() -> robot.shooter.goToRPM(targetRPM)));
        pad.b().onTrue(robot.shooter.goToRPM(0.0));
    }

    @Override
    public void periodic() {
        robot.shooter.setVelocityGains(kP, kI, kD, kS, kV, kA);
        Telemetry.log("Target RPM", targetRPM);
        Telemetry.log("Shooter RPM", robot.shooter.getRPM());
    }
}
