package org.firstinspires.ftc.teamcode.opmode.Autonomous;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.KillerInstinctRobot;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

@NextAutonomous(name = "AutoPath1 NextFTC", group = "Autonomous")
public class redAuto1gonnerNextFTC extends NextOpMode {


    private final KillerInstinctRobot robot;
    private final Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59, 8, 90);
    private final Pose path1 = poseFactory.of(25.1789, 46.7445, 180);
    private final Pose path1Control1 = poseFactory.of(52.2383, 33.4745, 0);
    private final Pose point2Start = poseFactory.of(25.1789, 46.7445, 180);
    private final Pose point2 = poseFactory.of(72.0193, 137.9202, 270);
    private final Pose point2Control1 = poseFactory.of(41.9247, 38.5089, 0);
    private final Pose point2Control2 = poseFactory.of(0, 141.5, 0);
    private final Pose point2Control3 = poseFactory.of(61.92, 93.9336, 0);
    private final Pose point3Start = poseFactory.of(72.0193, 137.9202, 270);
    private final Pose point3 = poseFactory.of(51, 130.7649, 90);
    private final Pose point3Control1 = poseFactory.of(82.2973, 115.3149, 0);
    private final Pose point3Control2 = poseFactory.of(51.6551, 116.5689, 0);
    private final Pose point4Start = poseFactory.of(51, 130.7649, 90);
    private final Pose point4 = poseFactory.of(72.1088, 138.6487, 270);
    private final Pose point4Control1 = poseFactory.of(55.4794, 112.0161, 0);
    private final Pose point4Control2 = poseFactory.of(82.8436, 120.0451, 0);
    private final Pose point5Start = poseFactory.of(72.1088, 138.6487, 270);
    private final Pose point5 = poseFactory.of(13.4633, 105.0894, 0);
    private final Pose point5Control1 = poseFactory.of(46.6723, 106.796, 0);

    public redAuto1gonnerNextFTC(KillerInstinctRobot robot) {
        super(robot);
        this.robot = robot;
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        robot.flower.setPosition(0.0);
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                autoshoot()
//                follow(follower, path1()),
//                follow(follower, path2()),
//                autoshoot(),
//                follow(follower, path3()),
//                follow(follower, path4()),
//                autoshoot(),
//                follow(follower, path5())
        );
    }

    public Command autoshoot() {
        return sequential(
                parallel(robot.blocker.block(),
                        robot.shooter.setVelocityTicksPerSecond(1800)),
                waitMs(2500),
                parallel(robot.blocker.allowFeed(), robot.intake.eat),
                waitMs(2000),
                parallel(robot.intake.sleep, robot.blocker.block(), robot.shooter.stop())
        );
    }

    @Override
    public void start() {
        Scheduler.schedule(autoRoutine());
    }

    @Override
    public void periodic() {
        follower.update();

        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());
        telemetry.addData("right CURRENT vel: ", "%.3f",
                robot.shooter.getRightVelocityTicksPerSecond());
        telemetry.addData("left CURRENT vel: ", "%.3f",
                robot.shooter.getLeftVelocityTicksPerSecond());

        if (follower.currentPath() != null) {
            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
            telemetry.addData("Path number", follower.pathIndex());
        }

        telemetry.update();
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2Control2, point2Control3, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3Control2, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4Control2, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.curve(point5Start, point5Control1, point5).linear(point5Start, point5);
    }

}
