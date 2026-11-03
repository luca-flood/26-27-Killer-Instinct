package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "seangoon", group = "TeleOp")
 public class seangoon extends LinearOpMode {
    //private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // define velocites
    float targetVel = 2000;
    float defaultVel = 0;
//    public seangoon (LinearOpMode opmode) {
//        myOpMode = opmode;
//    }


    // start runopmode
    @Override
    public void runOpMode() {

        // define motors, set direction
        DcMotorEx bottomMotor = (DcMotorEx) hardwareMap.get("shootLeft");
        DcMotorEx topMotor = (DcMotorEx) hardwareMap.get("shootRight");
        topMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomMotor.setDirection(DcMotor.Direction.FORWARD);

        // LOOP
        waitForStart();
        while (opModeIsActive()) {
            // whole command "if" jawn
            if (gamepad1.left_trigger >= 0.5) {
                bottomMotor.setVelocity(-targetVel);
            }
            else {
                bottomMotor.setVelocity(-targetVel);
            }

            if (gamepad1.right_trigger >= 0.5) {
                topMotor.setVelocity(targetVel);
                bottomMotor.setVelocity(targetVel);
            }
            else {
                topMotor.setVelocity(defaultVel);
                bottomMotor.setVelocity(defaultVel);
            }
            if (gamepad1.a) {
                targetVel = targetVel - 25;
            }

            if (gamepad1.b) {
                targetVel = targetVel + 25;
            }

            // telemetry tings
            telemetry.addData("CURRENT vel: ", "%.3f", topMotor.getVelocity());
            telemetry.addData("TARGET vel: ", "%.3f", targetVel);& "$env:LOCALAPPDATA\Android\Sdk\platform-tools\adb.exe" -d tcpip 5555
            telemetry.update();
        }



}

}
