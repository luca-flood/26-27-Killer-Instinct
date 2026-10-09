package org.firstinspires.ftc.teamcode.pedroPathing.swerve;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * This is the SwerveTurnTest
 * You should use this to check your encoder directions and x/y pod offsets
 * @author Kabir Goyal
 * @author Havish Sripada
 *
 */
@TeleOp(name = "SwerveTurnTuner", group = "Pedro Swerve")
// Adapted from official Quickstart commit 0470fdd; hold RB to permit motion.
public class SwerveTurnTuner extends OpMode {
    boolean debugStringEnabled = false;
    Swerve drivetrain;

    @Override
    public void init() {
        drivetrain = Constants.createDrivetrain(hardwareMap);
    }

    /** Drivetrain-only test: no Pinpoint or other localizer is constructed. */
    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled;
        }


        telemetry.addLine("This OpMode will run all four swerve pods in their turning direction (perpendicular to the center of the robot) "
                + "\nrun this once off the ground to check servo directions and motor directions before testing on the ground");
        telemetry.addLine("Hold RIGHT BUMPER to run; release to stop. Align wheels forward before INIT.");
        telemetry.addLine("Drivetrain debug string " + (((debugStringEnabled) ? "enabled" : "disabled")) +
                " (press gamepad a to toggle)");
        telemetry.update();
        drivetrain.drive(DrivePowers.zero(), true);
    }

    @Override
    public void start() {
        drivetrain.drive(DrivePowers.zero(), true);
    }

    /** Apply manual drive powers directly, without pose estimation. */
    @Override
    public void loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled;
        }

        if (gamepad1.right_bumper) {
            drivetrain.drive(new DrivePowers(0, 0, 0.25), true);
        } else {
            drivetrain.drive(DrivePowers.zero(), true);
        }

        if (debugStringEnabled) {
            telemetry.addLine("Drivetrain Debug String:\n" +
                    drivetrain.debug());
        }
        telemetry.update();
    }
    @Override
    public void stop() {
        if (drivetrain != null) {
            drivetrain.stop();
        }
        for (com.pedropathing.revhub.drivetrains.CoaxialPodConfig config : Constants.podConfigs()) {
            hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, config.servoName.get()).setPower(0);
            hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, config.motorName.get()).setPower(0);
        }
    }
}
