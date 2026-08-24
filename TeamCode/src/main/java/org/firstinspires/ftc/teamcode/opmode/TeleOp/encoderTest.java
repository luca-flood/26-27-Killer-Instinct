package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Encoder Test", group = "Tests")
public class encoderTest extends OpMode {
    private static final double SERVO_TEST_POWER = 0.2;
    private static final double MAX_ANALOG_VOLTAGE = 3.3;

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    private CRServo frontLeft1;
    private CRServo backLeft1;
    private CRServo frontRight1;
    private CRServo backRight1;

    private AnalogInput frontLeft2;
    private AnalogInput backLeft2;
    private AnalogInput frontRight2;
    private AnalogInput backRight2;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft1 = hardwareMap.get(CRServo.class, "frontLeft1");
        backLeft1 = hardwareMap.get(CRServo.class, "backLeft1");
        frontRight1 = hardwareMap.get(CRServo.class, "frontRight1");
        backRight1 = hardwareMap.get(CRServo.class, "backRight1");

        frontLeft2 = hardwareMap.get(AnalogInput.class, "frontLeft2");
        backLeft2 = hardwareMap.get(AnalogInput.class, "backLeft2");
        frontRight2 = hardwareMap.get(AnalogInput.class, "frontRight2");
        backRight2 = hardwareMap.get(AnalogInput.class, "backRight2");

        frontLeft.setPower(0.0);
        backLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backRight.setPower(0.0);
    }

    @Override
    public void loop() {
        setServoPower(frontLeft1, gamepad1.a, gamepad1.dpad_up);
        setServoPower(backLeft1, gamepad1.b, gamepad1.dpad_left);
        setServoPower(frontRight1, gamepad1.x, gamepad1.dpad_down);
        setServoPower(backRight1, gamepad1.y, gamepad1.dpad_right);

        addEncoderTelemetry("Front Left", frontLeft2);
        addEncoderTelemetry("Back Left", backLeft2);
        addEncoderTelemetry("Front Right", frontRight2);
        addEncoderTelemetry("Back Right", backRight2);
        telemetry.update();
    }

    @Override
    public void stop() {
        frontLeft1.setPower(0.0);
        backLeft1.setPower(0.0);
        frontRight1.setPower(0.0);
        backRight1.setPower(0.0);
    }

    private void setServoPower(CRServo servo, boolean positiveButton, boolean negativeButton) {
        if (positiveButton && !negativeButton) {
            servo.setPower(SERVO_TEST_POWER);
        } else if (negativeButton && !positiveButton) {
            servo.setPower(-SERVO_TEST_POWER);
        } else {
            servo.setPower(0.0);
        }
    }

    private void addEncoderTelemetry(String location, AnalogInput encoder) {
        double rawVoltage = encoder.getVoltage();
        double degrees = rawVoltage / MAX_ANALOG_VOLTAGE * 360.0;
        telemetry.addData(location, "raw %.3f V | %.1f deg", rawVoltage, degrees);
    }
}
