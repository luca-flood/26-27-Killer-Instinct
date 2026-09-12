package org.firstinspires.ftc.teamcode.control;

public class InstantCommand implements Command {
    private final Runnable action;
    private boolean finished;

    public InstantCommand(Runnable action) {
        this.action = action;
    }

    @Override
    public void start() {
        action.run();
        finished = true;
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
