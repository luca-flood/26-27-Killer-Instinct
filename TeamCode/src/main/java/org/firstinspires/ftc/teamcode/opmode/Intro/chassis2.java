package org.firstinspires.ftc.teamcode.opmode.Intro;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="chassis2", group="intro")

public class chassis2 extends LinearOpMode {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor arm;


    @Override
    public void runOpMode() throws InterruptedException {

        frontRight = hardwareMap.get(DcMotor.class, "fr");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        arm = hardwareMap.get(DcMotor.class, "arm");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        arm.setTargetPosition(100);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setDirection(DcMotor.Direction.REVERSE);

        arm.setTargetPosition(100);

        waitForStart();

        arm.setTargetPosition(100);
        //int pos = 0;


        while (opModeIsActive()) {

            double rightStickInput = gamepad1.right_stick_x;
            double leftStickInput = gamepad1.left_stick_y;

            double leftTriggerInput = gamepad1.left_trigger;
            double rightTriggerInput = gamepad1.right_trigger;

            if (leftStickInput > 0.1) {
                frontRight.setPower(leftStickInput);
                frontLeft.setPower(leftStickInput);
                backRight.setPower(leftStickInput);
                backLeft.setPower(leftStickInput);
            }
            if (leftStickInput < 0.1) {
                frontRight.setPower(leftStickInput);
                frontLeft.setPower(leftStickInput);
                backRight.setPower(leftStickInput);
                backLeft.setPower(leftStickInput);

            }
            if (rightStickInput > 0.1) {
                frontRight.setPower(rightStickInput);
                frontLeft.setPower(-rightStickInput);
                backRight.setPower(rightStickInput);
                backLeft.setPower(-rightStickInput);
            }
            if (rightStickInput < 0.1) {
                frontRight.setPower(rightStickInput);
                frontLeft.setPower(-rightStickInput);
                backRight.setPower(rightStickInput);
                backLeft.setPower(-rightStickInput);
            }

            if (leftStickInput > 0.1) {
                frontRight.setPower(leftStickInput);
                frontLeft.setPower(leftStickInput);
                backRight.setPower(leftStickInput);
                backLeft.setPower(leftStickInput);
            }

            if (rightTriggerInput > 0.1) {

                arm.setPower(rightTriggerInput * 0.3);
                //pos = pos + 10;
            }
            if (leftTriggerInput > 0.1) {

                arm.setPower(-leftTriggerInput*0.3);
                //pos = pos + 10;
            } else {
                arm.setPower(0.1);
            }
            //arm.setTargetPosition(pos);

            telemetry.addData("left trigger input", leftTriggerInput);
            telemetry.addData("right trigger input", rightTriggerInput);
            telemetry.addData("Motor position", arm.getCurrentPosition());
            //telemetry.addData("Position (var): ", pos);
            telemetry.update();


        }
    }
}

