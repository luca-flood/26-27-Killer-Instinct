package org.firstinspires.ftc.teamcode;

import java.util.Set;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

/**
 * The single NextRobot for the project. NextFTC scans for exactly one
 * implementation with a public no-arg constructor and injects it into
 * every NextOpMode.
 */
public class KillerInstinctRobot implements NextRobot {
    public final Shooter shooter = new Shooter();

    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(shooter);
    }
}
