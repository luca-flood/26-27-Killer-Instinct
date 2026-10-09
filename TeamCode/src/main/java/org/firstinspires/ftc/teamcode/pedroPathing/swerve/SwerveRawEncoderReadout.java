package org.firstinspires.ftc.teamcode.pedroPathing.swerve;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

/** Analog readout with all steering servo channels explicitly commanded to neutral. */
@TeleOp(name = "Swerve Raw Encoder Readout", group = "Pedro Swerve")
public class SwerveRawEncoderReadout extends OpMode {
    private static final String[] LABELS = {"Front Right", "Front Left", "Back Right", "Back Left"};
    private static final String[] INPUT_NAMES = {"frontRight2", "frontLeft2", "backRight2", "backLeft2"};
    private static final String[] SERVO_NAMES = {"frontRight1", "frontLeft1", "backRight1", "backLeft1"};
    private final AnalogInput[] inputs = new AnalogInput[4];
    private final CRServo[] servos = new CRServo[4];
    private Constants.PodCalibration[] calibration;
    private List<LynxModule> hubs;
    private LynxModule.BulkCachingMode[] originalCaching;
    private long samples;
    private String position = "Straight Forward";

    @Override
    public void init() {
        calibration = new Constants.PodCalibration[] {
                Constants.frontRight, Constants.frontLeft, Constants.backRight, Constants.backLeft};
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = hardwareMap.get(AnalogInput.class, INPUT_NAMES[i]);
            servos[i] = hardwareMap.get(CRServo.class, SERVO_NAMES[i]);
            servos[i].setPower(0.0);
        }
        hubs = hardwareMap.getAll(LynxModule.class);
        originalCaching = new LynxModule.BulkCachingMode[hubs.size()];
        for (int i = 0; i < hubs.size(); i++) {
            originalCaching[i] = hubs.get(i).getBulkCachingMode();
            hubs.get(i).setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }
        telemetry.setMsTransmissionInterval(100);
    }

    @Override
    public void init_loop() {
        displayReadings();
    }

    @Override
    public void loop() {
        displayReadings();
    }

    private void displayReadings() {
        setNeutralPower();
        // These buttons label a measurement only; they do not move the robot.
        if (gamepad1.dpad_up) position = "Straight Forward";
        else if (gamepad1.dpad_left) position = "Left";
        else if (gamepad1.dpad_down) position = "Straight Backward";
        else if (gamepad1.dpad_right) position = "Right";

        telemetry.addData("Readout", "v3 - servo power 0, direct analog reads");
        telemetry.addData("Sample counter", ++samples);
        telemetry.addData("Measurement position (label only)", position);
        boolean allZero = true;
        for (int i = 0; i < inputs.length; i++) {
            double voltage = inputs[i].getVoltage();
            allZero &= voltage <= 0.005;
            telemetry.addData(LABELS[i] + " input", INPUT_NAMES[i] + " | " + inputs[i].getConnectionInfo());
            telemetry.addData(LABELS[i] + " rawVoltage V", "%.6f", voltage);
            double span = calibration[i].analogMaxVoltage - calibration[i].analogMinVoltage;
            if (!Double.isFinite(voltage) || !Double.isFinite(span) || span <= 0) {
                telemetry.addData(LABELS[i], "Invalid voltage or calibration range");
                continue;
            }
            double normalized = (voltage - calibration[i].analogMinVoltage) / span;
            // Keep out-of-range values visible rather than hiding them by clamping to zero.
            double rawAngleDeg = normalized * 360;
            telemetry.addData(LABELS[i] + " rawAngleDeg", "%.3f", rawAngleDeg);
            telemetry.addData(LABELS[i] + " nominal angle deg", "%.3f", voltage / 3.3 * 360);
            if (normalized < 0 || normalized > 1) {
                telemetry.addData(LABELS[i] + " status", "OUTSIDE measured voltage bounds");
            }
        }
        if (allZero) telemetry.addLine("All analog signals near 0 V. Check encoder power, signal cables and robot configuration.");
        telemetry.addLine("Encoder angle uses measured min/max; no offset, reversal, gearing or angle catching.");
        telemetry.addLine("Servo commands remain 0. D-pad labels only. Never force powered servo gearing by hand.");
        telemetry.update();
    }

    @Override
    public void stop() {
        setNeutralPower();
        if (hubs != null && originalCaching != null) {
            for (int i = 0; i < hubs.size(); i++) {
                hubs.get(i).setBulkCachingMode(originalCaching[i]);
            }
        }
    }

    private void setNeutralPower() {
        for (CRServo servo : servos) {
            if (servo != null) servo.setPower(0.0);
        }
    }
}
