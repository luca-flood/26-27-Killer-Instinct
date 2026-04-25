package org.firstinspires.ftc.teamcode.opmode.Intro;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="chassis", group="intro")

public class chassis1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo rightServo = hardwareMap.get(Servo.class, "Longa_James ");
        Servo leftServo = hardwareMap.get(Servo.class, "Noah-chan");
        DcMotor leftMotor = hardwareMap.dcMotor.get("left");
        //Noah-chan is 1
        DcMotor rightMotor = hardwareMap.dcMotor.get("right");
        //Longa james is in 0

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);



        double leftStickY = 0;
        double leftServoPos = 0.5;
        double rightServoPos = 0.5;





        waitForStart();


        while (opModeIsActive()) {

            leftStickY = gamepad1.left_stick_y;



            if (leftStickY>0.1 || leftStickY<0.1){
                leftMotor.setPower(leftStickY);
                rightMotor.setPower(leftStickY);
            }

            if (gamepad1.right_stick_x > 0.1){
                leftMotor.setPower(gamepad1.right_stick_x);
                rightMotor.setPower(gamepad1.right_stick_x * -1);

            }

            if (gamepad1.right_stick_x < -0.1){
                leftMotor.setPower(gamepad1.right_stick_x);
                rightMotor.setPower(gamepad1.right_stick_x * -1);
            }else {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }


            if (gamepad1.a){
                leftServoPos += 0.1 ;
            }
            if (gamepad1.b){
                rightServoPos += 0.1;
            }


            if (gamepad1.x){
                leftServoPos -= 0.1;
            }
            if (gamepad1.y){
                rightServoPos -= 0.1;
            }

            leftServo.setPosition(leftServoPos);
            rightServo.setPosition(rightServoPos);

            telemetry.addData("right pos", rightServo.getPosition());
            telemetry.addData("left pos", leftServo.getPosition());



            telemetry.update();

        }
    }
}
