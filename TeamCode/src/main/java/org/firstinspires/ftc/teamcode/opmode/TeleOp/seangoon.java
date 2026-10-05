package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "seangoon", group = "TeleOp")
 public class seangoon extends LinearOpMode {
    //private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // define velocites
    float topTargetVel = 500;
    float bottomTargetVel = 500;
    float defaultVel = 0;
//    public seangoon (LinearOpMode opmode) {
//        myOpMode = opmode;
//    }


    // start runopmode
    @Override
    public void runOpMode() {

        // define motors, set direction
        DcMotorEx bottomMotor = (DcMotorEx) hardwareMap.get("bottomMotor");
        DcMotorEx topMotor = (DcMotorEx) hardwareMap.get("topMotor");
        topMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomMotor.setDirection(DcMotor.Direction.FORWARD);

        // LOOP
        waitForStart();
        while (opModeIsActive()) {
            // whole command "if" jawn
            if (gamepad1.left_trigger >=0.5) {
                bottomMotor.setVelocity(bottomTargetVel);
            }
            else {
                bottomMotor.setVelocity(defaultVel);
            }

            if (gamepad1.right_trigger >= 0.5) {
                topMotor.setVelocity(topTargetVel);
            }
            else {
                topMotor.setVelocity(defaultVel);
            }
            if (gamepad1.a) {
                topTargetVel = topTargetVel - 50;
            }

            if (gamepad1.b) {
                topTargetVel = topTargetVel + 50;
            }

            if (gamepad1.x) {
                bottomTargetVel = bottomTargetVel - 50;
            }

            if (gamepad1.y) {
                bottomTargetVel = bottomTargetVel + 50;
            }

            // telemetry tings
            telemetry.addData("[top] current vel: ", "%.3f", topMotor.getVelocity());
            telemetry.addData("[top] target vel: ", "%.3f", topTargetVel);
            telemetry.addData("[btm] current vel: ", "%.3f", bottomMotor.getVelocity());
            telemetry.addData("[btm] target vel: ", "%.3f", bottomTargetVel);
            telemetry.update();
        }



}

}
