package org.firstinspires.ftc.teamcode.pedroPathing.swerve;

import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PatchedCoaxialPod;

import java.util.List;

@TeleOp(name = "Swerve Encoder Direction Test", group = "Pedro Swerve")
public class SwerveEncoderDirectionTest extends OpMode {
    private static final double POWER = 0.2;
    private static final double MAX_PULSE_SECONDS = 0.4;
    private PatchedCoaxialPod[] pods;
    private CoaxialPodConfig[] configs;
    private List<LynxModule> hubs;
    private final double[] baselineEncoder = new double[4];
    private final double[] encoderChange = new double[4];
    private final double[] wheelChange = new double[4];
    private boolean lastA;
    private boolean hasPulse;
    private long pulseStarted;

    @Override
    public void init() {
        configs = Constants.podConfigs();
        pods = Constants.createPods(hardwareMap);
        hubs = hardwareMap.getAll(LynxModule.class);
        telemetry.setMsTransmissionInterval(100);
        stopOutputs();
    }

    @Override
    public void init_loop() {
        stopOutputs();
        telemetry.addLine("Lift robot. View steering from ABOVE, looking down at the robot.");
        telemetry.addLine("Hold A: all steering +0.2 for at most 0.4 seconds. Release to stop/rearm.");
        telemetry.addLine("No PID or drive power. Do not move powered pods by hand.");
        telemetry.update();
    }

    @Override
    public void start() {
        lastA = gamepad1.a;
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        long now = System.nanoTime();
        boolean beginPulse = gamepad1.a && !lastA;
        if (beginPulse) {
            pulseStarted = now;
            hasPulse = true;
        }
        boolean powered = gamepad1.a && hasPulse
                && (now - pulseStarted) / 1e9 < MAX_PULSE_SECONDS;
        telemetry.addData("Steering command", powered ? POWER : 0);
        for (int i = 0; i < pods.length; i++) {
            pods[i].getAngle();
            double encoderRad = pods[i].getContinuousEncoderAngleRad();
            if (beginPulse) baselineEncoder[i] = encoderRad;
            if (hasPulse) {
                encoderChange[i] = Math.toDegrees(encoderRad - baselineEncoder[i]);
                wheelChange[i] = encoderChange[i] * Constants.WHEEL_DEGREES_PER_ENCODER_REVOLUTION / 360
                        * (configs[i].encoderReversed.get() ? 1 : -1);
            }
            pods[i].setMotorPower(0);
            pods[i].setServoPower(powered ? POWER : 0);
            telemetry.addData(pods[i].name(), "encoder change %+.2f deg | calculated wheel change %+.2f deg",
                    encoderChange[i], wheelChange[i]);
        }
        lastA = gamepad1.a;
        telemetry.addLine("Wheel change should be POSITIVE for observed CCW, NEGATIVE for observed CW (viewed from above).");
        telemetry.addLine("Report each pod's physical CW/CCW direction and both signed angle changes after release.");
        telemetry.addLine("+servo power does NOT itself guarantee CCW; this test measures that relationship.");
        telemetry.update();
    }

    private void stopOutputs() {
        if (pods != null) {
            for (PatchedCoaxialPod pod : pods) {
                pod.setServoPower(0);
                pod.setMotorPower(0);
            }
        }
    }

    @Override
    public void stop() {
        stopOutputs();
    }
}
