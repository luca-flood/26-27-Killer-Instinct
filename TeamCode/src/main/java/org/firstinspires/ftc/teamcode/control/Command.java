package org.firstinspires.ftc.teamcode.control;

public interface Command {
    default void start() {
    }

    void update();

    boolean isFinished();

    default void stop(boolean interrupted) {
    }
}
