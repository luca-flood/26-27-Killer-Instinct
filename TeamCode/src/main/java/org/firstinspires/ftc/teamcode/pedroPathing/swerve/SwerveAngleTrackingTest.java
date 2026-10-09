package org.firstinspires.ftc.teamcode.pedroPathing.swerve;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PatchedCoaxialPod;

/** Passive diagnostic for the non-1:1 encoder ratio; not an upstream Pedro tuner. */
@TeleOp(name = "Swerve Angle Tracking Test", group = "Pedro Swerve")
public class SwerveAngleTrackingTest extends OpMode {
    private PatchedCoaxialPod[] pods;
    private long previousLoop;

    @Override
    public void init() {
        telemetry.addLine("Begin with wheels forward. Readout only; do not back-drive powered servo gearing.");
        telemetry.update();
    }

    @Override
    public void start() {
        pods = Constants.createPods(hardwareMap);
        for (PatchedCoaxialPod pod : pods) {
            pod.setMotorPower(0);
            pod.setServoPower(0);
            pod.getAngle();
        }
        previousLoop = System.nanoTime();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        telemetry.addData("loop ms", (now - previousLoop) / 1e6);
        previousLoop = now;
        for (PatchedCoaxialPod pod : pods) {
            telemetry.addData(pod.name(), pod.debug());
        }
        telemetry.addLine("Check commanded positions and wraps; never force powered pods by hand.");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (pods != null) {
            for (PatchedCoaxialPod pod : pods) {
                pod.setServoPower(0);
                pod.setMotorPower(0);
            }
        }
    }
}
