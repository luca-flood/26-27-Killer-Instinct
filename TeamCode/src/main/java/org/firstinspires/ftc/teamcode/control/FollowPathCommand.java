package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

public class FollowPathCommand implements Command {
    private final Follower follower;
    private final PathChain path;
    private final boolean holdEnd;
    private final double maxPower;

    public FollowPathCommand(Follower follower, PathChain path) {
        this(follower, path, true, 1.0);
    }

    public FollowPathCommand(Follower follower, PathChain path, boolean holdEnd, double maxPower) {
        this.follower = follower;
        this.path = path;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    @Override
    public void start() {
        follower.followPath(path, maxPower, holdEnd);
    }

    @Override
    public void update() {
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void stop(boolean interrupted) {
        if (interrupted) {
            follower.breakFollowing();
        }
    }
}
