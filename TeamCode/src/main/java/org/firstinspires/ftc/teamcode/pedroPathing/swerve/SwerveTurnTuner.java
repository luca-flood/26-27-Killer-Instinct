package org.firstinspires.ftc.teamcode.pedroPathing.swerve;

import com.pedropathing.follower.Follower;
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
    Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    /** This initializes the PoseUpdater, the drive motors, and the Panels telemetry. */
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
        follower.update();
    }

    @Override
    public void start() {
        follower.update();
    }

    /**
     * This updates the robot's pose estimate, the simple drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled;
        }

        if (gamepad1.right_bumper) {
            follower.manual(0, 0, 0.25);
        } else {
            follower.manual(0, 0, 0);
        }
        follower.update();

        if (debugStringEnabled) {
            telemetry.addLine("Drivetrain Debug String:\n" +
                    follower.drivetrain.debug());
        }
        telemetry.update();
    }
    @Override
    public void stop() {
        if (follower != null) {
            follower.manual(0, 0, 0);
            follower.update();
        }
        for (com.pedropathing.revhub.drivetrains.CoaxialPodConfig config : Constants.podConfigs()) {
            hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, config.servoName.get()).setPower(0);
            hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, config.motorName.get()).setPower(0);
        }
    }
}
