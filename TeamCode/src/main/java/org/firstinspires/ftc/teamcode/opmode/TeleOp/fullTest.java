package org.firstinspires.ftc.teamcode.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "testing", group = "TeleOp")
public class fullTest extends LinearOpMode {

    // define velocites
    float targetVel = 2000;
    float defaultVel = 0;

    // start runopmode
    @Override
    public void runOpMode() {

        // define motors, set direction

        DcMotorEx frontLeft = (DcMotorEx) hardwareMap.get("frontLeft");
        DcMotorEx backLeft = (DcMotorEx) hardwareMap.get("backLeft");
        DcMotorEx frontRight = (DcMotorEx) hardwareMap.get("frontRight");
        DcMotorEx backRight = (DcMotorEx) hardwareMap.get("backRight");

        DcMotorEx topMotor = (DcMotorEx) hardwareMap.get("shootRight");
        DcMotorEx bottomMotor = (DcMotorEx) hardwareMap.get("shootLeft");
        topMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        DcMotorEx intake = (DcMotorEx) hardwareMap.get("intake");
        intake.setDirection(DcMotor.Direction.FORWARD);

        Servo blocker = hardwareMap.get(Servo.class, "blocker");

        // LOOP
        waitForStart();



        double blockerPos = blocker.getPosition();
        double blockerTarget = 0.5;
        double intakeVelo = intake.getVelocity();

        while (opModeIsActive()) {
            // whole command "if" jawn
            if (gamepad1.left_trigger >= 0.5) {
                bottomMotor.setVelocity(-targetVel);
            }
            if (gamepad1.right_trigger >= 0.5) {
                bottomMotor.setVelocity(targetVel);
            }
              else{
                topMotor.setPower(0);
                bottomMotor.setPower(0);
            }

            if (gamepad1.a) {
                targetVel = targetVel - 25;
            }

            if (gamepad1.b) {
                targetVel = targetVel + 25;
            }

            if(gamepad1.right_bumper){
                intake.setPower(1);
            }
            else if(gamepad1.left_bumper){
                intake.setPower(-1);
            }
            else{
                intake.setPower(0);
            }

            if(gamepad1.x){
                blocker.setPosition(1);
            }
//                blocker.setPosition(blockerTarget);
//                blockerTarget += 0.05;

            if(gamepad1.y){
                blocker.setPosition(0.5);
//                blocker.setPosition(blockerTarget);
//                blockerTarget -= 0.05;
            }
            else{
                blocker.setPosition(1);
            }


// mecanum drive Code
            double y = gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x * 1.1;
            double x = -gamepad1.left_stick_x;
            double y2 = gamepad2.left_stick_y;
            double rx2 = -gamepad2.right_stick_x * 1.1;
            double x2 = -gamepad2.left_stick_x;

            telemetry.addLine("left stick y" + y);
            telemetry.addLine("left stick x" + x);
            telemetry.addLine("right stick x" + rx);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double denominator2 = Math.max(Math.abs(y2) + Math.abs(x2) + Math.abs(rx2), 1);
            double frontLeftPower2 = (y2 + x2 + rx2) / denominator2;
            double backLeftPower2 = (y2 - x2 + rx2) / denominator2;
            double frontRightPower2 = (y2 - x2 - rx2) / denominator2;
            double backRightPower2 = (y2 + x2 - rx2) / denominator2;

            if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_x >= 0.1 || gamepad1.left_stick_y <= -0.1 || gamepad1.left_stick_x <= -0.1 || gamepad1.right_stick_y >= 0.2 || gamepad1.right_stick_x >= 0.2 || gamepad1.right_stick_y <= -0.2 || gamepad1.right_stick_x <= -0.2) {
                frontLeft.setPower(frontLeftPower * 0.85);
                backLeft.setPower(backLeftPower * 0.85);
                frontRight.setPower(frontRightPower * 0.85);
                backRight.setPower(backRightPower * 0.85);
            } else if (gamepad2.left_stick_y >= 0.1 || gamepad2.left_stick_x >= 0.1 || gamepad2.left_stick_y <= -0.1 || gamepad2.left_stick_x <= -0.1 || gamepad2.right_stick_y >= 0.1 || gamepad2.right_stick_x >= 0.1 || gamepad2.right_stick_y <= -0.1 || gamepad2.right_stick_x <= -0.1) {
                frontLeft.setPower(frontLeftPower2 * 0.3);
                backLeft.setPower(backLeftPower2 * 0.3);
                frontRight.setPower(frontRightPower2 * 0.3);
                backRight.setPower(backRightPower2 * 0.3);
            } else {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }


            // telemetry tings
            telemetry.addData("CURRENT vel: ", "%.3f", topMotor.getVelocity());
            telemetry.addData("TARGET vel: ", "%.3f", targetVel);
            telemetry.addData("intake vel: ", "%.3f", intakeVelo);
            telemetry.addData("blocker pos: ", "%.3f", blockerPos);
            telemetry.update();
        }
    }
}


