package org.firstinspires.ftc.teamcode.opmode.Autonomous;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.control.SequentialCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath1", group = "Autonomous")
public class redAuto1gonner extends LinearOpMode {

    private static final double BLOCKER_CLOSED_POSITION = 0.5;
    private static final int OUTTAKE_SPIN_UP_MS = 2000;
    private static final int INTAKE_FEED_MS = 3000;

    private Follower follower;

    private DcMotorEx topMotor;
    private DcMotorEx bottomMotor;
    private DcMotorEx intake;
    private Servo blocker;

    private Servo flower;

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

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                autoshoot(),
                follow(follower, path1()),
                follow(follower, path2()),
                autoshoot(),
                follow(follower, path3()),
                follow(follower, path4()),
                autoshoot(),
                follow(follower, path5())
        );
    }

    public Command autoshoot(){
        return sequential(
                instant(() -> {
                    blocker.setPosition(1);
                    topMotor.setVelocity(1680);
                    bottomMotor.setVelocity(1680);
                }),
                waitMs(1500),
                instant(() -> {
                    blocker.setPosition(0.5);
                    intake.setPower(0.6);
                }),
                waitMs(2000),
                instant(() -> {
                    intake.setPower(0);
                    blocker.setPosition(1);
                    topMotor.setPower(0);
                    bottomMotor.setPower(0);
                })
        );
    }



    @Override
    public void runOpMode() {
        topMotor = hardwareMap.get(DcMotorEx.class, "shootRight");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "shootLeft");
        topMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomMotor.setDirection(DcMotor.Direction.FORWARD);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);

        blocker = hardwareMap.get(Servo.class, "blocker");
        flower = hardwareMap.get(Servo.class, "flower");

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        flower.setPosition(0);

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        Command routine = autoRoutine();
        try {
            Scheduler.schedule(routine);
            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();

                telemetry.addData("x", follower.pose().x());
                telemetry.addData("y", follower.pose().y());
                telemetry.addData("heading", follower.pose().heading());

                if (follower.currentPath() != null) {
                    telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                    telemetry.addData("Path number", follower.pathIndex());
                    telemetry.addData("right CURRENT vel: ", "%.3f", topMotor.getVelocity());
                    telemetry.addData("left CURRENT vel: ", "%.3f", bottomMotor.getVelocity());
                }

                telemetry.update();
            }
        } finally {
            Scheduler.cancel(routine);
        }
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
