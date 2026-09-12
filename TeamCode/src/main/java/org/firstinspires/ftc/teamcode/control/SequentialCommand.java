package org.firstinspires.ftc.teamcode.control;

import java.util.Arrays;
import java.util.List;

public class SequentialCommand implements Command {
    private final List<Command> commands;
    private int commandIndex;
    private Command activeCommand;

    public SequentialCommand(Command... commands) {
        this.commands = Arrays.asList(commands);
    }

    @Override
    public void start() {
        commandIndex = 0;
        startCurrentCommand();
    }

    @Override
    public void update() {
        if (activeCommand == null) {
            return;
        }

        activeCommand.update();
        if (activeCommand.isFinished()) {
            activeCommand.stop(false);
            commandIndex++;
            startCurrentCommand();
        }
    }

    @Override
    public boolean isFinished() {
        return activeCommand == null;
    }

    @Override
    public void stop(boolean interrupted) {
        if (activeCommand != null) {
            activeCommand.stop(interrupted);
            activeCommand = null;
        }
    }

    private void startCurrentCommand() {
        if (commandIndex >= commands.size()) {
            activeCommand = null;
            return;
        }

        activeCommand = commands.get(commandIndex);
        activeCommand.start();
    }
}
