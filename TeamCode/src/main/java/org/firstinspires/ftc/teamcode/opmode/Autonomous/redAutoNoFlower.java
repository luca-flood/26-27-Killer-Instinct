package org.firstinspires.ftc.teamcode.opmode.Autonomous;

import static com.pedropathing.ivy.commands.Commands.instant;
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

@NextAutonomous(name = "Red Auto No Flower", group = "Autonomous")
public class redAutoNoFlower extends NextOpMode {
    private final KillerInstinctRobot robot;
    private final Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59.5, 7, 90);
    private final Pose point1 = poseFactory.of(34.3793, 15.529, 60);
    private final Pose path2Start = poseFactory.of(34.3793, 15.529, 60);
    private final Pose path2 = poseFactory.of(10, 105, 0);
    private final Pose path2Control1 = poseFactory.of(0, 27.6808, 0);
    private final Pose path2Control2 = poseFactory.of(39.7299, 102.0242, 0);


    public redAutoNoFlower(KillerInstinctRobot robot) {
        super(robot);
        this.robot = robot;
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        // The scheduler doesn't tick during INIT.
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                autoshoot(),
                follow(follower,path2()),
                robot.blocker.allowFeed()
        );
    }

    public Command autoshoot() {
        return sequential(
                robot.shooter.setVelocityTicksPerSecond(1680),
                waitMs(2500),
                robot.intake.eat(),
                waitMs(2000),
                parallel(robot.intake.sleep(), robot.shooter.stop())
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
        return Paths.line(start, point1).linear(start, point1);
    }

    public Path path2() {
        return Paths.curve(path2Start, path2Control1, path2Control2, path2).linear(path2Start, path2);
    }

}