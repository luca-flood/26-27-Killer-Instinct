package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.KillerInstinctRobot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name = "testing NextFTC", group = "TeleOp")
public class fullTestNextFTC extends NextOpMode {

    private static final double SHOOTER_TRIGGER_THRESHOLD = 0.5;

    private final KillerInstinctRobot robot;
    private double commandedVelocity;
    private double intakePower;
    private Boolean blockerBlocked;


    public fullTestNextFTC(KillerInstinctRobot robot) {
        super(robot);
        this.robot = robot;
    }

    @Override
    public void disabledPeriodic() {
        robot.flower.down().schedule();
        telemetry.addLine("Set to Low.");
        telemetry.addData("Flower Pos: ", robot.flower.getRightPosition());
    }

    @Override
    public void start() {
        robot.drivetrain.drive(gamepad1, gamepad2).schedule();

        CommandGamepad driver = new CommandGamepad(gamepad1);
        driver.y().onTrue(lazy(this::toggleBlocker).requiring(robot.blocker));
    }

    @Override
    public void periodic() {
        updateShooter();
        updateIntake();

        if (gamepad1.a){
            telemetry.addLine("Running Autoshoot");
            autoshoot().schedule();
        }

        if (gamepad1.x) {
            telemetry.addData("Flower Pos: ", robot.flower.getRightPosition());
            if (robot.flower.getRightPosition() == 0.025) {
                robot.flower.up().schedule();
            }
            else {
                robot.flower.down().schedule();
            }
        }


        telemetry.addData("Right shooter velocity", "%.3f",
                robot.shooter.getRightVelocityTicksPerSecond());
        telemetry.addData("Left shooter velocity", "%.3f",
                robot.shooter.getLeftVelocityTicksPerSecond());
        telemetry.addData("Intake power", "%.2f", intakePower);
        telemetry.addData("Flower Right Pos :", robot.flower.getRightPosition());
        telemetry.addData("Flower Left Pos: ", robot.flower.getLeftPosition());
        if (blockerBlocked != null) {
            telemetry.addData("Blocker target", blockerBlocked ? "Blocking" : "Feed");
        }
        telemetry.addData("Flower target position", "%.2f", robot.flower.getRightPosition());
        telemetry.update();
    }

    private void updateShooter() {


        double velocity = 0.0;
        if (gamepad1.right_trigger >= SHOOTER_TRIGGER_THRESHOLD) {
            velocity = 1680;
        } else if (gamepad1.left_trigger >= SHOOTER_TRIGGER_THRESHOLD) {
            velocity = -1680;
        }

        if (velocity != commandedVelocity) {
            commandedVelocity = velocity;
            Command command = velocity == 0.0
                    ? robot.shooter.stop()
                    : robot.shooter.setVelocityTicksPerSecond(velocity);
            command.schedule();
        }


    }

    private void updateIntake() {
        double power = 0.0;
        if (gamepad1.right_bumper) {
            power = 0.55;
        } else if (gamepad1.left_bumper) {
            power = -0.4;
        }
        if (power != intakePower) {
            intakePower = power;
            robot.intake.setSuck(power).schedule();
        }
    }

    private Command toggleBlocker() {
        blockerBlocked = !Boolean.TRUE.equals(blockerBlocked);
        return blockerBlocked ? robot.blocker.block() : robot.blocker.allowFeed();
    }

    private Command autoshoot() {
        return sequential(
                parallel(robot.blocker.block(),
                        robot.shooter.setVelocityTicksPerSecond(1700)),
                waitMs(2500),
                parallel(robot.blocker.allowFeed(), robot.intake.eat),
                waitMs(2000),
                parallel(robot.intake.sleep, robot.blocker.block(), robot.shooter.stop())
        );
    }
}
