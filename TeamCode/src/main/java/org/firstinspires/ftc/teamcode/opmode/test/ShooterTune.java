package org.firstinspires.ftc.teamcode.opmode.test;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.KillerInstinctRobot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@Disabled
@NextTeleop(name = "Shooter Tuning")
public class ShooterTune extends NextOpMode {

    private final KillerInstinctRobot robot;
    private double kP = 0.00027;
    private double kD = 0.0009;
    private double kV = 0.00029;
    private double kS = 0.125;

    private double targetRPM;
    public ShooterTune(KillerInstinctRobot robot) {
        super(robot);
        this.robot = robot;
    }

    public Command changeP(double amount) {
        return instant(() -> kP += amount);
    }

    public Command changeD(double amount) {
        return instant(() -> kD += amount);
    }

    public Command changeV(double amount) {
        return instant(() -> kV += amount);
    }

    public Command changeS(double amount) {
        return instant(() -> kS += amount);
    }

    public Command changeRPM(double amount) {
        return instant(() -> targetRPM += amount);
    }

    @Override
    public void start() {
        CommandGamepad pad = new CommandGamepad(gamepad1);
        targetRPM = 1500;
        pad.a().onTrue(lazy(() -> robot.shooter.goToRPM(targetRPM)));
        pad.b().onTrue(robot.shooter.goToRPM(0.0));
        pad.rightBumper().onTrue(changeRPM(100));
        pad.leftBumper().onTrue(changeRPM(-100));
        pad.leftTrigger().isOver(0.1).onTrue(changeS(-0.005));
        pad.rightTrigger().isOver(0.1).onTrue(changeS(0.005));
        pad.dpadLeft().onTrue(changeP(-0.00001));
        pad.dpadRight().onTrue(changeP(0.00001));
        pad.dpadDown().onTrue(changeD(-0.00001));
        pad.dpadUp().onTrue(changeD(0.00001));
        pad.x().onTrue(changeV(-0.00001));
        pad.y().onTrue(changeV(0.00001));
    }

    @Override
    public void periodic() {
        robot.shooter.setPID(kP, kD);
        robot.shooter.setKV(kV);
        robot.shooter.setKS(kS);
        Telemetry.log("kV", kV);
        Telemetry.log("Rotations", robot.shooter.getRotations());
        Telemetry.log("Target RPM", targetRPM);
        Telemetry.log("kP", kP);
        Telemetry.log("kD", kD);
        Telemetry.log("kS", kS);
        Telemetry.log("Shooter RPM", robot.shooter.getRPM());
    }
}
