package org.firstinspires.ftc.teamcode.pedroPathing.swerve;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.List;

/** Official Quickstart 0470fdd raw extrema measurement, by Kabir Goyal and
 * Havish Sripada, adapted to an automated bidirectional steering sweep. */
@TeleOp(name = "Analog Min / Max Tuner", group = "Pedro Swerve")
public class AnalogMinMaxTuner extends OpMode {
    private static final double POWER = 0.2;
    private static final double WHEEL_REVOLUTIONS = 5;
    private static final double NOMINAL_SPAN = 3.3;
    private static final double TARGET_ENCODER_DEG = WHEEL_REVOLUTIONS * 360
            * 360 / Constants.WHEEL_DEGREES_PER_ENCODER_REVOLUTION;
    private static final double MAX_ENCODER_SPEED = 6000;
    private static final double MAX_SAMPLE_GAP_SEC = 180 / MAX_ENCODER_SPEED;
    private static final String[] LABELS = {"Front Right", "Front Left", "Back Right", "Back Left"};
    private static final String[] HARDWARE = {"frontRight", "frontLeft", "backRight", "backLeft"};
    private enum Phase { READY, POSITIVE, PAUSE, NEGATIVE, COMPLETE, ABORTED }
    private Phase phase = Phase.READY;
    private String fault = "";
    private final AnalogInput[] encoders = new AnalogInput[4];
    private final CRServo[] servos = new CRServo[4];
    private final DcMotor[] motors = new DcMotor[4];
    private final double[] minimum = new double[4];
    private final double[] maximum = new double[4];
    private final double[] lastVoltage = new double[4];
    private final double[] encoderTravel = new double[4];
    private final double[] positiveTurns = new double[4];
    private final double[] negativeTurns = new double[4];
    private final boolean[] finished = new boolean[4];
    private final long[] acceptedAt = new long[4];
    private final int[] rejectedSamples = new int[4];
    private String lastRejected = "";
    private List<LynxModule> hubs;
    private LynxModule.BulkCachingMode[] originalCaching;
    private long phaseStarted;

    @Override
    public void init() {
        for (int i = 0; i < 4; i++) {
            encoders[i] = hardwareMap.get(AnalogInput.class, HARDWARE[i] + "2");
            servos[i] = hardwareMap.get(CRServo.class, HARDWARE[i] + "1");
            motors[i] = hardwareMap.get(DcMotor.class, HARDWARE[i]);
            servos[i].setPower(0);
            motors[i].setPower(0);
            minimum[i] = Double.POSITIVE_INFINITY;
            maximum[i] = Double.NEGATIVE_INFINITY;
        }
        hubs = hardwareMap.getAll(LynxModule.class);
        originalCaching = new LynxModule.BulkCachingMode[hubs.size()];
        for (int i = 0; i < hubs.size(); i++) {
            originalCaching[i] = hubs.get(i).getBulkCachingMode();
            hubs.get(i).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry.setMsTransmissionInterval(100);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Lift robot and keep clear. START runs all steering servos at +0.2 then -0.2.");
        telemetry.addLine("Approximately 5 wheel turns each direction. B or STOP aborts. Drive motors stay off.");
        telemetry.update();
    }

    @Override
    public void start() {
        beginSweep(Phase.POSITIVE);
    }

    private void beginSweep(Phase next) {
        clearCaches();
        // Read every encoder before powering any servo.
        for (int i = 0; i < 4; i++) {
            lastVoltage[i] = encoders[i].getVoltage();
            acceptedAt[i] = System.nanoTime();
            if (!validVoltage(lastVoltage[i])) {
                abort("Invalid voltage: " + LABELS[i]);
                return;
            }
            record(i, lastVoltage[i]);
            encoderTravel[i] = 0;
            finished[i] = false;
        }
        phase = next;
        phaseStarted = System.nanoTime();
        for (CRServo servo : servos) servo.setPower(next == Phase.POSITIVE ? POWER : -POWER);
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        if (gamepad1.b && phase != Phase.COMPLETE && phase != Phase.ABORTED) abort("Operator cancelled");
        if (phase == Phase.PAUSE && seconds(now, phaseStarted) >= 0.5) {
            beginSweep(Phase.NEGATIVE);
        } else if (phase == Phase.POSITIVE || phase == Phase.NEGATIVE) {
            if (seconds(now, phaseStarted) >= 90) {
                abort("Sweep timed out; check stalled modules");
            } else {
                clearCaches();
                for (int i = 0; i < 4; i++) {
                    if (finished[i]) continue;
                    double voltage = encoders[i].getVoltage();
                    long sampledAt = System.nanoTime();
                    double dt = seconds(sampledAt, acceptedAt[i]);
                    if (dt >= MAX_SAMPLE_GAP_SEC) {
                        abort(LABELS[i] + ": no trustworthy sample for "
                                + String.format("%.1f", dt * 1000) + " ms; revolution count uncertain");
                        break;
                    }
                    if (!validVoltage(voltage)) {
                        abort("Invalid voltage: " + LABELS[i]);
                        break;
                    }
                    double delta = signedEncoderDelta(lastVoltage[i], voltage);
                    double allowedDelta = MAX_ENCODER_SPEED * Math.max(dt, 0.001) + 5;
                    if (Math.abs(delta) > allowedDelta) {
                        // Keep the last accepted angle AND timestamp, allowing one noisy sample to recover.
                        rejectedSamples[i]++;
                        lastRejected = String.format("%s: %.4f -> %.4f V, delta %.1f deg, limit %.1f deg, dt %.2f ms",
                                LABELS[i], lastVoltage[i], voltage, delta, allowedDelta, dt * 1000);
                        continue;
                    }
                    record(i, voltage);
                    lastVoltage[i] = voltage;
                    acceptedAt[i] = sampledAt;
                    encoderTravel[i] += delta;
                    double wheelTurns = Math.abs(encoderTravel[i]) / 360
                            * Constants.WHEEL_DEGREES_PER_ENCODER_REVOLUTION / 360;
                    if (phase == Phase.POSITIVE) positiveTurns[i] = wheelTurns;
                    else negativeTurns[i] = wheelTurns;
                    if (Math.abs(encoderTravel[i]) >= TARGET_ENCODER_DEG) {
                        finished[i] = true;
                        servos[i].setPower(0);
                    }
                }
                if ((phase == Phase.POSITIVE || phase == Phase.NEGATIVE) && allFinished()) {
                    stopOutputs();
                    phase = phase == Phase.POSITIVE ? Phase.PAUSE : Phase.COMPLETE;
                    phaseStarted = now;
                }
            }
        }
        telemetry.addData("Sweep", phase);
        if (!fault.isEmpty()) telemetry.addData("Stopped", fault);
        if (!lastRejected.isEmpty()) telemetry.addData("Last rejected sample", lastRejected);
        for (int i = 0; i < 4; i++) {
            if (Double.isFinite(minimum[i])) {
                telemetry.addData(LABELS[i], "MIN %.5f V | MAX %.5f V | + turns %.2f | - turns %.2f | rejected %d",
                        minimum[i], maximum[i], positiveTurns[i], negativeTurns[i], rejectedSamples[i]);
            } else telemetry.addData(LABELS[i], "No samples");
        }
        if (phase == Phase.COMPLETE) telemetry.addLine("DONE: outputs zero. Record all four MIN/MAX pairs.");
        else if (phase == Phase.ABORTED) telemetry.addLine("INCOMPLETE: do not use as final calibration.");
        telemetry.update();
    }

    // Nominal voltage is used only for travel counting; extrema are raw volts.
    static double signedEncoderDelta(double previous, double current) {
        double degrees = (current - previous) / NOMINAL_SPAN * 360;
        if (degrees > 180) degrees -= 360;
        if (degrees < -180) degrees += 360;
        return degrees;
    }

    private boolean validVoltage(double voltage) {
        return Double.isFinite(voltage) && voltage >= 0 && voltage <= NOMINAL_SPAN + 0.05;
    }

    private void record(int i, double voltage) {
        minimum[i] = Math.min(minimum[i], voltage);
        maximum[i] = Math.max(maximum[i], voltage);
    }

    private boolean allFinished() {
        for (boolean value : finished) if (!value) return false;
        return true;
    }

    private void clearCaches() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    private static double seconds(long now, long then) {
        return (now - then) / 1e9;
    }

    private void abort(String reason) {
        fault = reason;
        phase = Phase.ABORTED;
        stopOutputs();
    }

    private void stopOutputs() {
        for (int i = 0; i < 4; i++) {
            if (servos[i] != null) servos[i].setPower(0);
            if (motors[i] != null) motors[i].setPower(0);
        }
    }

    @Override
    public void stop() {
        stopOutputs();
        if (hubs != null && originalCaching != null) {
            for (int i = 0; i < hubs.size(); i++) hubs.get(i).setBulkCachingMode(originalCaching[i]);
        }
    }
}
