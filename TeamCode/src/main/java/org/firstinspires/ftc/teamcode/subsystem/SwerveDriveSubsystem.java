package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.RobotSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class SwerveDriveSubsystem implements RobotSubsystem {
    private final Follower follower;

    public SwerveDriveSubsystem(HardwareMap hardwareMap, Pose startingPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
    }

    @Override
    public void update() {
        follower.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }

    public Follower getFollower() {
        return follower;
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public void follow(PathChain path, boolean holdEnd, double maxPower) {
        follower.followPath(path, maxPower, holdEnd);
    }

    public boolean isBusy() {
        return follower.isBusy();
    }
}
