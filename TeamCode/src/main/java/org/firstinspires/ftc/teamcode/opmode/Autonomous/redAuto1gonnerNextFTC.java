package org.firstinspires.ftc.teamcode.opmode.Autonomous;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.loop;
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

@NextAutonomous(name = "Red Auto Flower", group = "Autonomous")
public class redAuto1gonnerNextFTC extends NextOpMode {


    private final KillerInstinctRobot robot;
    private final Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59.5, 7, 90);
    private final Pose path1 = poseFactory.of(36, 47, 180);
    private final Pose path1Control1 = poseFactory.of(52.2383, 33.4745, 0);
    private final Pose point2 = poseFactory.of(27, 47, 180);
    private final Pose point3 = poseFactory.of(36, 47, 180);
    private final Pose point4Start = poseFactory.of(36, 47, 180);
    private final Pose point4 = poseFactory.of(71, 142, 270);
    private final Pose point4Control1 = poseFactory.of(41.9247, 38.5089, 0);
    private final Pose point4Control2 = poseFactory.of(0, 141.5, 0);
    private final Pose point4Control3 = poseFactory.of(61.92, 93.9336, 0);
    private final Pose point5Start = poseFactory.of(71, 142, 270);
    private final Pose point5 = poseFactory.of(69, 124, 90);
    private final Pose point5Control1 = poseFactory.of(79, 130, 0);
    private final Pose point6 = poseFactory.of(69, 130, 90);
    private final Pose point7 = poseFactory.of(69, 124, 90);
    private final Pose point8Start = poseFactory.of(69, 124, 90);
    private final Pose point8 = poseFactory.of(71, 142, 270);
    private final Pose point8Control1 = poseFactory.of(79, 130, 0);
    private final Pose point9Start = poseFactory.of(71, 142, 270);
    private final Pose point9 = poseFactory.of(7, 105, 0);
    private final Pose point9Control1 = poseFactory.of(46.6723, 106.796, 0);

    public redAuto1gonnerNextFTC(KillerInstinctRobot robot) {
        super(robot);
        this.robot = robot;
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        // The scheduler doesn't tick during INIT.
        robot.flower.down().start();
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5())
//                follow(follower, path6()),
//                follow(follower, path7()),
//                follow(follower, path8()),
//                follow(follower, path9())
        );
    }

    public Command autoshoot() {
        return sequential(
                parallel(robot.blocker.block(),
                        robot.shooter.setVelocityTicksPerSecond(1800)),
                waitMs(2500),
                parallel(robot.blocker.allowFeed(), robot.intake.eat()),
                waitMs(2000),
                parallel(robot.intake.sleep(), robot.blocker.block(), robot.shooter.stop())
        );
    }

    @Override
    public void start() {
        loop(sequential(robot.flower.down(), waitMs(500))).schedule();
        Scheduler.schedule(autoRoutine());
    }

    @Override
    public void periodic() {
        follower.update();

        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());
        telemetry.addData("Flower targets (right / left)", "%.3f / %.3f",
                robot.flower.getRightPosition(), robot.flower.getLeftPosition());
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
        return Paths.line(path1, point2).constant(point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).constant(point3);
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4Control2, point4Control3, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.curve(point5Start, point5Control1, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).constant(point6);
    }

    public Path path7() {
        return Paths.line(point6, point7).constant(point7);
    }

    public Path path8() {
        return Paths.curve(point8Start, point8Control1, point8).linear(point8Start, point8);
    }

    public Path path9() {
        return Paths.curve(point9Start, point9Control1, point9).linear(point9Start, point9);
    }


}
