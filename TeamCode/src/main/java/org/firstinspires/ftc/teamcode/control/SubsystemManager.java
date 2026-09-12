package org.firstinspires.ftc.teamcode.control;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SubsystemManager {
    private final List<RobotSubsystem> subsystems = new ArrayList<>();

    public SubsystemManager(RobotSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    public void add(RobotSubsystem subsystem) {
        subsystems.add(subsystem);
    }

    public void init() {
        for (RobotSubsystem subsystem : subsystems) {
            subsystem.init();
        }
    }

    public void start() {
        for (RobotSubsystem subsystem : subsystems) {
            subsystem.start();
        }
    }

    public void update() {
        for (RobotSubsystem subsystem : subsystems) {
            subsystem.update();
        }
    }

    public void stop() {
        for (RobotSubsystem subsystem : subsystems) {
            subsystem.stop();
        }
    }
}
