package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.RobotSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class SwerveDriveSubsystem implements RobotSubsystem {
    private final Follower follower;

    public SwerveDriveSubsystem(HardwareMap hardwareMap, Pose startingPose) {
        follower = Constants.createFollower(hardwareMap);
        if (follower != null) {
            follower.setPose(startingPose);
        }
    }

    @Override
    public void update() {
        if (follower != null) {
            follower.update();
        }
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.stop();
        }
    }

    public Follower getFollower() {
        return follower;
    }

    public Pose getPose() {
        return follower == null ? Pose.zero() : follower.pose();
    }

    public void follow(Path path, boolean holdEnd, double maxPower) {
        if (follower != null) {
            follower.holdEnd.set(holdEnd);
            follower.follow(path);
        }
    }

    public boolean isBusy() {
        return follower != null && follower.isBusy();
    }
}
