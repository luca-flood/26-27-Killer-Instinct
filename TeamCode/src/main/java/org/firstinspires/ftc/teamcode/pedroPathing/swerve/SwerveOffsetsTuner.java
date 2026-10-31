package org.firstinspires.ftc.teamcode.pedroPathing.swerve;

import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.List;

/** Manual direction-test adaptation of official Quickstart 0470fdd offsets tuner,
 * by Kabir Goyal and Havish Sripada. No automatic steering or Pinpoint required. */
@TeleOp(name = "SwerveOffsetsTuner", group = "Pedro Swerve")
public class SwerveOffsetsTuner extends OpMode {
    private static final double STEERING_POWER = 0.2;
    private static final double MOTOR_POWER = 0.25;
    private static final double MATCH_ENCODER_DEG = 2;
    private final CRServo[] servos = new CRServo[4];
    private final DcMotor[] motors = new DcMotor[4];
    private final AnalogInput[] encoders = new AnalogInput[4];
    private CoaxialPodConfig[] configs;
    private List<LynxModule> hubs;
    private double previousAngle;
    private double previousSteeringPower;
    private double positivePowerTravel;
    private long previousSample;
    private boolean hasSample;

    @Override
    public void init() {
        // Constants pod order is front left, front right, back left, back right.
        configs = Constants.podConfigs();
        for (int i = 0; i < configs.length; i++) {
            servos[i] = hardwareMap.get(CRServo.class, configs[i].servoName.get());
            motors[i] = hardwareMap.get(DcMotor.class, configs[i].motorName.get());
            encoders[i] = hardwareMap.get(AnalogInput.class, configs[i].servoEncoderName.get());
            servos[i].setDirection(configs[i].servoDirection.get());
            motors[i].setDirection(configs[i].driveDirection.get());
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            servos[i].setPower(0);
            motors[i].setPower(0);
        }
        hubs = hardwareMap.getAll(LynxModule.class);
        telemetry.setMsTransmissionInterval(100);
    }

    @Override
    public void init_loop() {
        stopOutputs();
        readAndDisplay(false);
    }

    @Override
    public void loop() {
        readAndDisplay(true);
    }

    private void readAndDisplay(boolean running) {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        double[] voltage = new double[4];
        for (int i = 0; i < encoders.length; i++) voltage[i] = encoders[i].getVoltage();
        double min = configs[0].analogMinVoltage.get();
        double span = configs[0].analogMaxVoltage.get() - min;
        if (!Double.isFinite(voltage[0]) || voltage[0] < 0.005 || !Double.isFinite(span) || span <= 0) {
            stopOutputs();
            telemetry.addLine("Front-left encoder signal or calibration invalid: all outputs stopped.");
            telemetry.update();
            return;
        }
        double actual = (voltage[0] - min) / span * 360;
        double target = Math.toDegrees(configs[0].angleOffsetRad.get());
        double error = signedDelta(target - actual);
        long now = System.nanoTime();
        if (hasSample && previousSteeringPower != 0) {
            double delta = signedDelta(actual - previousAngle);
            double dt = Math.max((now - previousSample) / 1e9, 0.001);
            if (Math.abs(delta) <= 6000 * dt + 5) {
                // Learn command polarity from actual motion, rather than guessing from encoderReversed.
                positivePowerTravel += delta * Math.signum(previousSteeringPower);
            }
        }
        hasSample = true;
        previousAngle = actual;
        previousSample = now;
        double steeringPower = running && gamepad1.a != gamepad1.b
                ? (gamepad1.a ? STEERING_POWER : -STEERING_POWER) : 0;
        boolean motorTest = running && gamepad1.right_bumper && !gamepad1.a && !gamepad1.b;
        for (int i = 0; i < servos.length; i++) {
            servos[i].setPower(i == 0 ? steeringPower : 0);
            motors[i].setPower(motorTest ? MOTOR_POWER : 0);
        }
        previousSteeringPower = steeringPower;

        telemetry.addLine("Lift robot. A = front-left +0.2 | B = front-left -0.2 | release = stop.");
        telemetry.addLine("RIGHT BUMPER = all drive motors +0.25; disabled while A/B is held. No steering PID.");
        telemetry.addData("Front Left forward target V", "%.5f", min + target / 360 * span);
        telemetry.addData("Front Left current V / encoder deg", "%.5f / %.3f", voltage[0], actual);
        telemetry.addData("Front Left encoder error deg", "%.3f", error);
        telemetry.addData("Front Left steering power", steeringPower);
        if (Math.abs(error) <= MATCH_ENCODER_DEG) {
            telemetry.addData("Front Left guidance", "VOLTAGE MATCH: release A/B; visually verify forward before motor test");
        } else if (Math.abs(positivePowerTravel) < 3) {
            telemetry.addData("Front Left guidance", "Polarity unknown: briefly tap A, then release to learn");
        } else {
            boolean usePositive = Math.signum(error) == Math.signum(positivePowerTravel);
            telemetry.addData("Front Left guidance", usePositive ? "Hold A (+0.2) toward target" : "Hold B (-0.2) toward target");
        }
        telemetry.addData("Positive power encoder direction",
                Math.abs(positivePowerTravel) < 3 ? "Not established" :
                        (positivePowerTravel > 0 ? "Increasing angle" : "Decreasing angle"));
        for (int i = 0; i < configs.length; i++) {
            telemetry.addData(configs[i].name.get(), "raw %.5f V | motor %.2f", voltage[i], motors[i].getPower());
        }
        telemetry.addLine("Voltage alone cannot resolve wheel revolution count. Check orientation visually; never force powered pods.");
        telemetry.update();
    }

    private static double signedDelta(double degrees) {
        return ((degrees + 180) % 360 + 360) % 360 - 180;
    }

    private void stopOutputs() {
        for (int i = 0; i < servos.length; i++) {
            if (servos[i] != null) servos[i].setPower(0);
            if (motors[i] != null) motors[i].setPower(0);
        }
        previousSteeringPower = 0;
    }

    @Override
    public void stop() {
        stopOutputs();
    }
}
