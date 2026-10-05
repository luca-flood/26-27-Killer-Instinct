package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;

public class FollowPathCommand implements Command {
    private final Follower follower;
    private final Path path;
    private final boolean holdEnd;

    public FollowPathCommand(Follower follower, Path path) {
        this(follower, path, true, 1.0);
    }

    public FollowPathCommand(Follower follower, Path path, boolean holdEnd, double maxPower) {
        this.follower = follower;
        this.path = path;
        this.holdEnd = holdEnd;
    }

    @Override
    public void start() {
        if (follower != null) {
            follower.holdEnd.set(holdEnd);
            follower.follow(path);
        }
    }

    @Override
    public void update() {
        if (follower != null) {
            follower.update();
        }
    }

    @Override
    public boolean isFinished() {
        return follower == null || !follower.isBusy();
    }

    @Override
    public void stop(boolean interrupted) {
        if (interrupted && follower != null) {
            follower.stop();
        }
    }
}
