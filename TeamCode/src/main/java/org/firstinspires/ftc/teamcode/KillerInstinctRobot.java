package org.firstinspires.ftc.teamcode;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import org.firstinspires.ftc.teamcode.Mechanism.Blocker;
import org.firstinspires.ftc.teamcode.Mechanism.Flower;
import org.firstinspires.ftc.teamcode.Mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanism.Shooter;
import org.firstinspires.ftc.teamcode.Mechanism.intakeSubsystem;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

/**
 * The single NextRobot for the project. NextFTC scans for exactly one
 * implementation with a public no-arg constructor and injects it into
 * every NextOpMode.
 */
public class KillerInstinctRobot implements NextRobot {
    public final Shooter shooter;
    public final intakeSubsystem intake;
    public final Blocker blocker = new Blocker();
    public final Flower flower = new Flower();
    public final MecanumDrive drivetrain;

    private final Set<Mechanism> mechanisms;

    public KillerInstinctRobot() {
        drivetrain = new MecanumDrive();
        // Drop stale Sloth callbacks and keep the drive motors under the drive command's
        // control. Pedro controls the same motors in autonomous.
        NextMotor.Companion.getMotorEventLoop().clear();
        shooter = new Shooter();
        intake = new intakeSubsystem();
        mechanisms = Collections.unmodifiableSet(
                new HashSet<Mechanism>(Arrays.asList(shooter, intake, blocker, flower, drivetrain)));
    }

    @Override
    public Set<Mechanism> getMechanisms() {
        return mechanisms;
    }
}
